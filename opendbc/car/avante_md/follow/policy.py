"""Turns openpilot's follow plan into stock cruise button requests.

openpilot hands the plan to the car controller as a FollowCommand: the target speed (m/s), the planner's
acceleration and a request to coast. The ECM set speed is estimated here and moved one tap at a time: CANCEL
to coast when the lead needs more deceleration than the set speed can give, SET to take the current speed
back, SET- and RES to trim, and no RES for a while after a kickdown. A tap the engine did not answer is pressed
again at once.
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
    self.climb_blocked = False
    self._lamp_set_prev = False
    self._resync = False
    self._down_since: int | None = None
    self._up_since: int | None = None
    self._gear_prev = 0
    self._kickdown_nanos: int | None = None
    self._downhill_nanos: int | None = None
    self._grade_nanos: int | None = None
    self._captured = False
    self._taps_since_commit: list[int] = []
    self._down_misses = 0
    self._down_paused_nanos: int | None = None
    self._last_tap_dir = 0
    self._last_tap_nanos: int | None = None
    self._retry_dir = 0
    self._quick_retries = 0

  @property
  def v_set_kph(self) -> float | None:
    return self.estimator.v_set if self.estimator.valid else None

  def update(self, now: int, plan: FollowPlan, cruise: CruiseStateMachine, lamp_set: bool, v_cluster: float,
             v_wheel: float, a_ego: float, signals: FollowSignals, pitch: float | None = None) -> ButtonRequest:
    """Runs before the button machine and returns what it should press next."""
    if not cruise.armed:
      self.estimator.reset()
      self.response.reset()
      self.v_user_kph = None
      self._resync = False
      self._captured = False
      self._clear_taps()
    elif lamp_set and not self._lamp_set_prev:
      self.estimator.capture(now, v_cluster)
      self._captured = True
      self._clear_taps()
      if self.v_user_kph is None:
        self.v_user_kph = v_cluster
    self._lamp_set_prev = lamp_set and cruise.armed

    self._update_grade(now, pitch, signals, a_ego)
    self._update_gates(now, signals)

    heard = self.response.update(now, signals, lamp_set)
    if heard is not None:
      self.estimator.answer(heard)
      retry = not heard and self._quick_retries < P.QUICK_RETRY_LIMIT
      self._retry_dir = self._last_tap_dir if retry else 0
      self._quick_retries = self._quick_retries + 1 if retry else 0
    self.estimator.update(now, v_cluster, v_wheel * CV.MS_TO_KPH, self._steady(now, lamp_set, signals))
    self._count_down_misses(now)

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
      self.response.start(now - CruiseParams.TAP_NANOS, cruise.tap_event)
      self._taps_since_commit.append(cruise.tap_event)
      self._last_tap_dir = cruise.tap_event
      self._last_tap_nanos = now
      self._retry_dir = 0

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
    self._last_tap_nanos = None
    self._retry_dir = 0
    self._quick_retries = 0

  def _count_down_misses(self, now: int) -> None:
    if self._down_paused_nanos is not None and now - self._down_paused_nanos >= P.TAP_DOWN_PAUSE_NANOS:
      self._down_paused_nanos = None
      self._down_misses = 0
    steps = self.estimator.last_commit_steps
    if steps is None:
      return
    if self._taps_since_commit and all(d < 0 for d in self._taps_since_commit):
      self._down_misses = self._down_misses + 1 if steps == 0 else 0
      if self._down_misses >= P.TAP_DOWN_MISS_LIMIT and self._down_paused_nanos is None:
        self._down_paused_nanos = now
    self._taps_since_commit.clear()
    self.estimator.last_commit_steps = None

  def _active_request(self, now: int, plan: FollowPlan, v_cluster: float, signals: FollowSignals) -> ButtonRequest:
    est = self.estimator
    v_set = est.v_set_likely
    error = plan.v_target_kph - v_set

    if est.needs_resync and v_cluster >= P.RESYNC_MIN_SPEED_KPH:
      self._resync = True
      return ButtonRequest.CANCEL
    # A tap waits until the engine's answer is in and the estimator has resolved it, unless the set speed is
    # already several steps off.
    far = abs(error) >= P.PENDING_OVERRIDE_STEPS * P.TAP_STEP_KPH
    burst = far and len(self._taps_since_commit) < P.PENDING_BURST_TAPS
    if (not est.valid or est.needs_resync or not self._captured or self.response.busy or
        (est.pending and not burst)):
      self._down_since = self._up_since = None
      return ButtonRequest.NONE

    recent = self._last_tap_nanos is not None and now - self._last_tap_nanos < P.REVERSAL_NANOS
    down_band = P.TARGET_DEADBAND_KPH + (P.REVERSAL_EXTRA_KPH if recent and self._last_tap_dir > 0 else 0.)
    up_band = P.TARGET_DEADBAND_KPH + (P.REVERSAL_EXTRA_KPH if recent and self._last_tap_dir < 0 else 0.)
    down = (error <= -down_band and v_set - P.TAP_STEP_KPH >= max(P.MIN_SPEED_KPH, P.TAP_DOWN_MIN_KPH) and
            self._down_paused_nanos is None)
    up = (error >= up_band and plan.a_target >= P.CLIMB_MIN_ACCEL and
          self.v_user_kph is not None and v_set + P.TAP_STEP_KPH <= self.v_user_kph + P.TARGET_DEADBAND_KPH)

    self._down_since = (self._down_since or now) if down else None
    self._up_since = (self._up_since or now) if up else None
    if not (down if self._retry_dir < 0 else up and not self.climb_blocked):
      self._retry_dir = 0
    down_hold = 0 if self._retry_dir < 0 else P.TARGET_HOLD_NANOS
    up_hold = 0 if self._retry_dir > 0 else P.TARGET_HOLD_NANOS

    if self._down_since is not None and now - self._down_since >= down_hold:
      self._down_since = None
      return ButtonRequest.DECEL
    if self._up_since is not None and now - self._up_since >= up_hold and not self.climb_blocked:
      self._up_since = None
      return ButtonRequest.RES
    return ButtonRequest.NONE

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

  def _update_gates(self, now: int, signals: FollowSignals) -> None:
    if not signals.valid:
      self.climb_blocked = True
      return
    if 0 < signals.gear < self._gear_prev or signals.rpm > P.KICKDOWN_RPM:
      self._kickdown_nanos = now
    if signals.gear > 0:
      self._gear_prev = signals.gear
    self.climb_blocked = signals.gas or self._kicked_down(now)
