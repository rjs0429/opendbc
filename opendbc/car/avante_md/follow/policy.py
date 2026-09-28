"""Turns openpilot's follow plan into stock cruise button requests.

openpilot hands the plan to the car controller as a FollowCommand: the target speed (m/s), the planner's
acceleration and a request to coast. The ECM set speed is estimated here and the buttons only ever lower it: SET-
trims it one tap at a time, CANCEL coasts when the lead needs more deceleration than the set speed can give, and SET
takes the current speed back. Speeding up is the driver's: once the pedal lets the car reach a new speed, CANCEL and
SET make it the set speed, up to the speed the driver engaged at. A tap the engine did not answer is pressed again at once.
"""
import math
from dataclasses import dataclass

from opendbc.car.avante_md.cruise import ButtonRequest, CruiseState, CruiseStateMachine
from opendbc.car.avante_md.follow.command import FollowCommand
from opendbc.car.avante_md.follow.estimator import SetSpeedEstimator
from opendbc.car.avante_md.follow.response import TapResponse
from opendbc.car.avante_md.follow.signals import FollowSignals
from opendbc.car.avante_md.values import CruiseParams, FollowParams as P, SetSpeedParams
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
  def from_command(cls, cmd: FollowCommand | None) -> 'FollowPlan':
    if cmd is None or not math.isfinite(cmd.v_target) or cmd.v_target <= 0.:
      return cls()
    return cls(True, cluster_from_wheel(cmd.v_target * CV.MS_TO_KPH), cmd.a_target, cmd.coast)


class FollowController:
  def __init__(self):
    self.estimator = SetSpeedEstimator()
    self.response = TapResponse()
    self.v_user_kph: float | None = None
    self.grade_pct = 0.
    self._lamp_set_prev = False
    self._resync = False
    self._down_since: int | None = None
    self._gear_prev = 0
    self._kickdown_nanos: int | None = None
    self._downhill_nanos: int | None = None
    self._grade_nanos: int | None = None
    self._captured = False
    self._taps_since_commit: list[int] = []
    self._down_misses = 0
    self._down_paused_nanos: int | None = None
    self._retry = False
    self._quick_retries = 0
    self._pedal_since: int | None = None
    self._pedal_captured = False
    self._captured_under_pedal = False

  @property
  def v_set_kph(self) -> float | None:
    return self.estimator.v_set if self.estimator.valid else None

  @property
  def v_set_display_kph(self) -> float | None:
    """The set speed to show the driver: taps count as soon as they are sent."""
    return self.estimator.v_set_likely if self.estimator.valid else None

  def update(self, now: int, plan: FollowPlan, cruise: CruiseStateMachine, lamp_set: bool, v_cluster: float,
             v_wheel: float, a_ego: float, signals: FollowSignals, pitch: float | None = None) -> ButtonRequest:
    """Runs before the button machine and returns what it should press next."""
    pedal = signals.valid and signals.gas
    if not cruise.armed:
      self.estimator.reset()
      self.response.reset()
      self.v_user_kph = None
      self._resync = False
      self._captured = False
      self._captured_under_pedal = False
      self._clear_taps()
    elif lamp_set and not self._lamp_set_prev:
      self.estimator.capture(now, v_cluster)
      self._captured = True
      self._captured_under_pedal = pedal
      self._clear_taps()
      if self.v_user_kph is None:
        self.v_user_kph = v_cluster
    self._lamp_set_prev = lamp_set and cruise.armed

    self._update_grade(now, pitch, signals, a_ego)
    self._update_kickdown(now, signals)
    released = self._update_pedal(now, pedal)

    heard = self.response.update(now, signals, lamp_set)
    if heard is not None:
      self.estimator.answer(heard)
      self._retry = not heard and self._quick_retries < P.QUICK_RETRY_LIMIT
      self._quick_retries = self._quick_retries + 1 if self._retry else 0
    self.estimator.update(now, v_cluster, v_wheel * CV.MS_TO_KPH, self._steady(now, lamp_set, signals))
    self._count_down_misses(now)

    if not (cruise.armed and plan.valid):
      self._down_since = None
      return ButtonRequest.NONE
    # Coasting only lowers output, so it does not wait for the powertrain signals.
    if cruise.state == CruiseState.ACTIVE and plan.coast:
      return ButtonRequest.CANCEL
    if not signals.valid:
      self._down_since = None
      return ButtonRequest.NONE

    if cruise.state == CruiseState.ACTIVE:
      return self._active_request(now, plan, v_cluster, pedal, released)
    if cruise.state == CruiseState.COAST:
      return self._coast_request(now, plan, cruise, v_cluster)
    return ButtonRequest.NONE

  def after_buttons(self, now: int, cruise: CruiseStateMachine) -> None:
    """Runs after the button machine so a tap it just released reaches the estimator."""
    if cruise.tap_event is not None:
      self.estimator.tap(now, cruise.tap_event)
      self.response.start(now - CruiseParams.TAP_NANOS, cruise.tap_event)
      self._taps_since_commit.append(cruise.tap_event)
      self._retry = False

  def _steady(self, now: int, lamp_set: bool, signals: FollowSignals) -> bool:
    if self.grade_pct <= SetSpeedParams.STEADY_MIN_GRADE_PCT:
      self._downhill_nanos = now
    recovering = self._downhill_nanos is not None and now - self._downhill_nanos < SetSpeedParams.DOWNHILL_RECOVERY_NANOS
    return (lamp_set and signals.valid and not signals.brake and not signals.gas and not recovering and
            self.grade_pct < SetSpeedParams.STEADY_MAX_GRADE_PCT and not self._kicked_down(now))

  def _clear_taps(self) -> None:
    self._down_misses = 0
    self._down_paused_nanos = None
    self._taps_since_commit.clear()
    self._retry = False
    self._quick_retries = 0

  def _count_down_misses(self, now: int) -> None:
    if self._down_paused_nanos is not None and now - self._down_paused_nanos >= P.TAP_DOWN_PAUSE_NANOS:
      self._down_paused_nanos = None
      self._down_misses = 0
    steps = self.estimator.last_commit_steps
    if steps is None:
      return
    if self._taps_since_commit:
      self._down_misses = self._down_misses + 1 if steps == 0 else 0
      if self._down_misses >= P.TAP_DOWN_MISS_LIMIT and self._down_paused_nanos is None:
        self._down_paused_nanos = now
    self._taps_since_commit.clear()
    self.estimator.last_commit_steps = None

  def _update_pedal(self, now: int, pedal: bool) -> bool:
    """Whether the driver has just let go of a press long enough to have set a new speed."""
    if pedal:
      if self._pedal_since is None:
        self._pedal_since = now
        self._pedal_captured = False
      return False
    released = self._pedal_since is not None and now - self._pedal_since >= P.CAPTURE_PEDAL_NANOS
    self._pedal_since = None
    return released

  def _capture_wanted(self, plan: FollowPlan, v_cluster: float, v_set: float, pedal: bool, released: bool) -> bool:
    if self.v_user_kph is None or v_cluster < P.RESYNC_MIN_SPEED_KPH:
      return False
    plan_ok = plan.v_target_kph >= v_cluster - P.CAPTURE_PLAN_MARGIN_KPH
    if pedal:
      reached = v_cluster >= self.v_user_kph - P.CAPTURE_USER_MARGIN_KPH
      below = v_set < self.v_user_kph - P.TARGET_DEADBAND_KPH
      if reached and below and plan_ok and not self._pedal_captured:
        self._pedal_captured = True
        return True
      return False
    if not released:
      return False
    under_pedal, self._captured_under_pedal = self._captured_under_pedal, False
    if v_cluster > self.v_user_kph + P.TARGET_DEADBAND_KPH:
      return False
    if v_cluster >= v_set + P.CAPTURE_MIN_GAIN_KPH:
      return plan_ok
    return under_pedal and v_cluster <= v_set - P.CAPTURE_MIN_LOSS_KPH

  def _active_request(self, now: int, plan: FollowPlan, v_cluster: float, pedal: bool, released: bool) -> ButtonRequest:
    est = self.estimator
    v_set = est.v_set_likely
    error = plan.v_target_kph - v_set

    if est.needs_resync and v_cluster >= P.RESYNC_MIN_SPEED_KPH:
      self._resync = True
      return ButtonRequest.CANCEL
    if est.valid and self._captured and self._capture_wanted(plan, v_cluster, v_set, pedal, released):
      self._resync = True
      return ButtonRequest.CANCEL
    # A tap waits until the engine's answer is in and the estimator has resolved it, unless the set speed is
    # already several steps off. Nothing is tapped while the driver's pedal decides the speed.
    far = error <= -P.PENDING_OVERRIDE_STEPS * P.TAP_STEP_KPH
    burst = far and len(self._taps_since_commit) < P.PENDING_BURST_TAPS
    if (pedal or not est.valid or est.needs_resync or not self._captured or self.response.busy or
        (est.pending and not burst)):
      self._down_since = None
      return ButtonRequest.NONE

    down = (error <= -P.TARGET_DEADBAND_KPH and v_set - P.TAP_STEP_KPH >= max(P.MIN_SPEED_KPH, P.TAP_DOWN_MIN_KPH) and
            self._down_paused_nanos is None)
    self._down_since = (self._down_since or now) if down else None
    if not down:
      self._retry = False
    hold = 0 if self._retry else P.TARGET_HOLD_NANOS
    if self._down_since is not None and now - self._down_since >= hold:
      self._down_since = None
      return ButtonRequest.DECEL
    return ButtonRequest.NONE

  def _coast_request(self, now: int, plan: FollowPlan, cruise: CruiseStateMachine, v_cluster: float) -> ButtonRequest:
    self._down_since = None
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

  def _update_grade(self, now: int, pitch: float | None, signals: FollowSignals, a_ego: float) -> None:
    if pitch is not None:
      raw = math.tan(pitch) * 100.
    elif signals.valid:
      raw = (signals.long_accel - P.LONG_ACCEL_BIAS - a_ego) / 9.81 * 100.
    else:
      self._grade_nanos = None
      return
    raw = max(-P.GRADE_MAX_PCT, min(P.GRADE_MAX_PCT, raw))
    if self._grade_nanos is None:
      self.grade_pct = raw
    else:
      alpha = min(1., (now - self._grade_nanos) * 1e-9 / P.GRADE_TAU)
      self.grade_pct += alpha * (raw - self.grade_pct)
    self._grade_nanos = now

  def _kicked_down(self, now: int) -> bool:
    return self._kickdown_nanos is not None and now - self._kickdown_nanos < P.KICKDOWN_HOLD_NANOS

  def _update_kickdown(self, now: int, signals: FollowSignals) -> None:
    if not signals.valid:
      return
    announced = 0 < signals.target_gear < signals.gear
    if announced or 0 < signals.gear < self._gear_prev or signals.rpm > P.KICKDOWN_RPM:
      self._kickdown_nanos = now
    if signals.gear > 0:
      self._gear_prev = signals.gear
