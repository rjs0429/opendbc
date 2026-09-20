"""Turns openpilot's follow plan into stock cruise button requests.

openpilot sends the plan inside CarControl: actuators.speed is the target speed (m/s, 0 when there is no
plan), actuators.accel the planner's acceleration and cruiseControl.cancel a request to coast. The ECM set
speed is estimated here and moved one tap at a time: CANCEL to coast when the lead needs more deceleration
than the set speed can give, SET to take the current speed back, SET- and RES to trim, and RES only while
the powertrain has headroom.
"""
import math
from dataclasses import dataclass

from opendbc.car.avante_md.cruise import ButtonRequest, CruiseState, CruiseStateMachine
from opendbc.car.avante_md.follow.estimator import SetSpeedEstimator
from opendbc.car.avante_md.follow.signals import FollowSignals
from opendbc.car.avante_md.values import FollowParams as P, SetSpeedParams
from opendbc.car.common.conversions import Conversions as CV


def cluster_from_wheel(v_wheel_kph: float) -> float:
  return P.CLUSTER_GAIN * v_wheel_kph + P.CLUSTER_OFFSET_KPH


def wheel_from_cluster(v_cluster_kph: float) -> float:
  return (v_cluster_kph - P.CLUSTER_OFFSET_KPH) / P.CLUSTER_GAIN


@dataclass
class FollowPlan:
  valid: bool = False
  v_target_kph: float = 0.  # cluster scale
  a_target: float = 0.
  coast: bool = False

  @classmethod
  def from_car_control(cls, CC) -> 'FollowPlan':
    speed = CC.actuators.speed
    if not math.isfinite(speed) or speed <= 0.:
      return cls()
    return cls(True, cluster_from_wheel(speed * CV.MS_TO_KPH), CC.actuators.accel, CC.cruiseControl.cancel)


class FollowController:
  def __init__(self):
    self.estimator = SetSpeedEstimator()
    self.v_user_kph: float | None = None
    self.grade_pct = 0.
    self.climb_blocked = False
    self._lamp_set_prev = False
    self._resync = False
    self._down_since: int | None = None
    self._up_since: int | None = None
    self._gate_clear_since: int | None = None
    self._gear_prev = 0
    self._downshift_nanos: int | None = None
    self._downhill_nanos: int | None = None
    self._last_res_nanos: int | None = None
    self._res_undone = True
    self._grade_nanos: int | None = None
    self._taps_since_commit: list[int] = []
    self._down_misses = 0

  @property
  def v_set_kph(self) -> float | None:
    return self.estimator.v_set if self.estimator.valid else None

  def update(self, now: int, plan: FollowPlan, cruise: CruiseStateMachine, lamp_set: bool, v_cluster: float,
             v_wheel: float, a_ego: float, signals: FollowSignals) -> ButtonRequest:
    """Runs before the button machine and returns what it should press next."""
    if not cruise.armed:
      self.estimator.reset()
      self.v_user_kph = None
      self._resync = False
      self._down_misses = 0
      self._taps_since_commit.clear()
    elif lamp_set and not self._lamp_set_prev:
      self.estimator.capture(now, v_cluster)
      self._down_misses = 0
      self._taps_since_commit.clear()
      if self.v_user_kph is None:
        self.v_user_kph = v_cluster
    self._lamp_set_prev = lamp_set and cruise.armed

    self._update_grade(now, signals, a_ego)
    self._update_gates(now, v_cluster, v_wheel, signals)

    self.estimator.update(now, v_cluster, v_wheel * CV.MS_TO_KPH, self._steady(now, lamp_set, signals))
    self._count_down_misses()

    if not (cruise.armed and plan.valid):
      self._down_since = self._up_since = None
      return ButtonRequest.NONE
    # Coasting only lowers output, so it does not wait for the powertrain signals.
    if cruise.state == CruiseState.ACTIVE and plan.coast:
      return ButtonRequest.CANCEL
    if not signals.valid:
      self._down_since = self._up_since = None
      return ButtonRequest.NONE

    if cruise.state == CruiseState.ACTIVE:
      return self._active_request(now, plan, v_cluster, signals)
    if cruise.state == CruiseState.COAST:
      return self._coast_request(now, plan, cruise, v_cluster)
    return ButtonRequest.NONE

  def after_buttons(self, now: int, cruise: CruiseStateMachine) -> None:
    """Runs after the button machine so a tap it just released reaches the estimator."""
    if cruise.tap_event is not None:
      self.estimator.tap(now, cruise.tap_event)
      self._taps_since_commit.append(cruise.tap_event)
      if cruise.tap_event > 0:
        self._last_res_nanos = now
        self._res_undone = False

  def _steady(self, now: int, lamp_set: bool, signals: FollowSignals) -> bool:
    if self.grade_pct <= SetSpeedParams.STEADY_MIN_GRADE_PCT:
      self._downhill_nanos = now
    recovering = self._downhill_nanos is not None and now - self._downhill_nanos < SetSpeedParams.DOWNHILL_RECOVERY_NANOS
    return (lamp_set and signals.valid and not signals.brake and not signals.gas and not recovering and
            self.grade_pct < SetSpeedParams.STEADY_MAX_GRADE_PCT and self._downshift_nanos is None)

  def _count_down_misses(self) -> None:
    steps = self.estimator.last_commit_steps
    if steps is None:
      return
    if self._taps_since_commit and all(d < 0 for d in self._taps_since_commit):
      self._down_misses = self._down_misses + 1 if steps == 0 else 0
    self._taps_since_commit.clear()
    self.estimator.last_commit_steps = None

  def _active_request(self, now: int, plan: FollowPlan, v_cluster: float, signals: FollowSignals) -> ButtonRequest:
    if self._res_overshoot(now, signals):
      self._res_undone = True
      return ButtonRequest.DECEL

    est = self.estimator
    if est.needs_resync and v_cluster >= P.RESYNC_MIN_SPEED_KPH:
      self._resync = True
      return ButtonRequest.CANCEL
    if not est.resolved:
      self._down_since = self._up_since = None
      return ButtonRequest.NONE

    error = plan.v_target_kph - est.v_set
    down = (error <= -P.TARGET_DEADBAND_KPH and est.v_set - P.TAP_STEP_KPH >= max(P.MIN_SPEED_KPH, P.TAP_DOWN_MIN_KPH) and
            self._down_misses < P.TAP_DOWN_MISS_LIMIT)
    up = (error >= P.TARGET_DEADBAND_KPH and plan.a_target >= P.CLIMB_MIN_ACCEL and
          self.v_user_kph is not None and est.v_set + P.TAP_STEP_KPH <= self.v_user_kph + P.TARGET_DEADBAND_KPH)

    self._down_since = (self._down_since or now) if down else None
    self._up_since = (self._up_since or now) if up else None

    if self._down_since is not None and now - self._down_since >= P.TARGET_HOLD_NANOS:
      self._down_since = None
      return ButtonRequest.DECEL
    if self._up_since is not None and now - self._up_since >= P.TARGET_HOLD_NANOS and not self.climb_blocked:
      self._up_since = None
      return ButtonRequest.RES
    return ButtonRequest.NONE

  def _res_overshoot(self, now: int, signals: FollowSignals) -> bool:
    if self._res_undone or self._last_res_nanos is None or now - self._last_res_nanos > P.RES_UNDO_WINDOW_NANOS:
      return False
    kicked_down = self._downshift_nanos is not None and self._downshift_nanos >= self._last_res_nanos
    return kicked_down or signals.rpm > P.RPM_HARD

  def _coast_request(self, now: int, plan: FollowPlan, cruise: CruiseStateMachine, v_cluster: float) -> ButtonRequest:
    self._down_since = self._up_since = None
    if plan.coast:
      self._resync = False
      return ButtonRequest.NONE
    if v_cluster < P.MIN_SPEED_KPH:
      return ButtonRequest.NONE
    if cruise.set_rejected_nanos is not None and now - cruise.set_rejected_nanos < P.RECAPTURE_RETRY_NANOS:
      return ButtonRequest.NONE

    since = now - cruise.coast_nanos
    if self._resync:
      if since >= P.RESYNC_SET_DELAY_NANOS:
        self._resync = False
        return ButtonRequest.SET
      return ButtonRequest.NONE

    settled = plan.a_target >= P.RECAPTURE_MIN_ACCEL or v_cluster <= plan.v_target_kph + P.RECAPTURE_SPEED_MARGIN_KPH
    if settled and since >= P.RECAPTURE_DELAY_NANOS:
      return ButtonRequest.SET
    return ButtonRequest.NONE

  def _update_grade(self, now: int, signals: FollowSignals, a_ego: float) -> None:
    if not signals.valid:
      self._grade_nanos = None
      return
    raw = (signals.long_accel - P.LONG_ACCEL_BIAS - a_ego) / 9.81 * 100.
    if self._grade_nanos is None:
      self.grade_pct = raw
    else:
      alpha = min(1., (now - self._grade_nanos) * 1e-9 / P.GRADE_TAU)
      self.grade_pct += alpha * (raw - self.grade_pct)
    self._grade_nanos = now

  def _update_gates(self, now: int, v_cluster: float, v_wheel: float, signals: FollowSignals) -> None:
    if not signals.valid:
      self.climb_blocked = True
      self._gate_clear_since = None
      return

    if 0 < signals.gear < self._gear_prev:
      self._downshift_nanos = now
    if signals.gear > 0:
      self._gear_prev = signals.gear
    if self._downshift_nanos is not None and now - self._downshift_nanos >= P.DOWNSHIFT_HOLD_NANOS:
      self._downshift_nanos = None

    top_gear_speed = v_cluster >= P.TOP_GEAR_MIN_KPH
    pedal_limit = P.PEDAL_HOLD_HIGH_PCT if v_cluster >= P.PEDAL_HIGH_SPEED_KPH else P.PEDAL_HOLD_PCT
    rpm_soft = P.RPM_PER_KPH_TOP_GEAR * v_wheel * CV.MS_TO_KPH + P.RPM_SOFT_MARGIN
    slip = abs(signals.rpm - signals.turbine_rpm)
    blocked = (
      (0 < signals.target_gear < signals.gear) or
      self._downshift_nanos is not None or
      (top_gear_speed and not signals.sport and signals.gear < P.TOP_GEAR) or
      self.grade_pct >= P.UPHILL_GRADE_PCT or
      signals.pedal_pct >= pedal_limit or
      (top_gear_speed and signals.rpm > rpm_soft) or
      (v_cluster >= P.LOCKUP_CHECK_MIN_KPH and slip >= P.LOCKUP_SLIP_RPM) or
      signals.gas
    )

    if blocked:
      self._gate_clear_since = None
    elif self._gate_clear_since is None:
      self._gate_clear_since = now
    self.climb_blocked = self._gate_clear_since is None or now - self._gate_clear_since < P.GATE_CLEAR_NANOS
