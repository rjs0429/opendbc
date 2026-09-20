from collections import deque
from enum import IntEnum

from opendbc.car.avante_md.avantecan import CLU1_SW_CANCEL, CLU1_SW_NONE, CLU1_SW_RES, CLU1_SW_SET
from opendbc.car.avante_md.values import CruiseParams, FollowParams


class CruiseState(IntEnum):
  INIT = 0
  IDLE = 1
  PRESS_MAIN_ON = 2
  WAIT_SET_READY = 3
  PRESS_SET = 4
  ACTIVE = 5
  PRESS_MAIN_OFF = 6
  FAULT = 7
  TAP = 8
  PRESS_CANCEL = 9
  COAST = 10


class ButtonRequest(IntEnum):
  NONE = 0
  CANCEL = 1
  SET = 2
  RES = 3
  DECEL = 4


class CruiseStateMachine:
  """Drives the stock cruise control through CLU1 cruise switch bits.

  The ECM cruise lamps are the only state that is trusted: CruiseSwMain is an edge toggle, so an
  open-loop press would turn cruise off just as readily as on. MAIN, SET and CANCEL presses run until
  their lamp confirms them. RES and SET- taps cannot be confirmed by a lamp, so they are single short
  taps whose effect the caller verifies from the speed.

  Once a driver engagement reaches ACTIVE the machine is armed: the follow logic may cancel into COAST and
  set again later. The session ends, and MAIN is switched off, on a brake, the driver's long press or a
  lasting loss of the vehicle preconditions. Nothing is ever set again without an armed session or a
  driver engagement in progress.
  """

  def __init__(self):
    self.state = CruiseState.INIT
    self.sw_state = CLU1_SW_NONE
    self.sw_main = 0
    self.armed = False
    self.cancel_unsupported = False
    self.tap_event: int | None = None
    self.set_rejected_nanos: int | None = None
    self.coast_nanos = 0
    self._entered = 0
    self._press_start = 0
    self._settle_until: int | None = None
    self._pressing = False
    self._main_attempts = 0
    self._set_attempts = 0
    self._cancel_attempts = 0
    self._ready_since: int | None = None
    self._tap_button = CLU1_SW_NONE
    self._tap_dir = 0
    self._tap_released = False
    self._engaging = False
    self._recapture = False
    self._main_off_coast = False
    self._off_requested = False
    self._main_lamp = False
    self._set_lamp = False
    self._precond_lost: int | None = None
    self._drops: deque[int] = deque()
    self._cancel_failures: deque[int] = deque()
    self._cancel_unsupported_since: int | None = None

  @property
  def engaged(self) -> bool:
    return self.state in (CruiseState.ACTIVE, CruiseState.TAP)

  @property
  def transmitting(self) -> bool:
    return self.sw_state != CLU1_SW_NONE or self.sw_main != 0

  def _goto(self, state: CruiseState, now: int) -> None:
    self.state = state
    self.sw_state = CLU1_SW_NONE
    self.sw_main = 0
    self._entered = now
    self._press_start = now
    self._settle_until = None
    self._pressing = False
    self._ready_since = None
    if state == CruiseState.IDLE or (state == CruiseState.FAULT and not self._set_lamp):
      self._disarm()
    elif state == CruiseState.ACTIVE:
      self.armed = True
      self._engaging = False
      self._recapture = False
      self._main_off_coast = False
    elif state == CruiseState.COAST:
      self.coast_nanos = now

  def _disarm(self) -> None:
    self.armed = False
    self._engaging = False
    self._recapture = False
    self._main_off_coast = False
    self._off_requested = False

  def _switch_off(self, now: int) -> None:
    self._disarm()
    self._main_attempts = 0
    self._goto(CruiseState.PRESS_MAIN_OFF if self._main_lamp else CruiseState.IDLE, now)

  def _tap(self, now: int, reached: bool, press_nanos: int, settle_nanos: int) -> bool | None:
    """Hold the button until its lamp answers, then release; None while an attempt is still open."""
    done: bool | None = None

    if reached:
      self._pressing = False
      self._settle_until = None
      done = True
    elif self._pressing:
      if now - self._press_start >= press_nanos:
        self._pressing = False
        self._settle_until = now + settle_nanos
    elif self._settle_until is None:
      self._pressing = True
      self._press_start = now
    elif now >= self._settle_until:
      self._settle_until = None
      done = False

    return done

  def update(self, now: int, long_press: bool, lamps_valid: bool, lamp_main: bool, lamp_set: bool,
             v_cluster_kph: float, precond: bool, clu1_fresh: bool, brake: bool | None = None,
             follow_active: bool = False, request: ButtonRequest = ButtonRequest.NONE,
             coast_requested: bool = False) -> None:
    self.tap_event = None
    self._main_lamp = lamp_main
    self._set_lamp = lamp_set
    if self._cancel_unsupported_since is not None and now - self._cancel_unsupported_since >= CruiseParams.CANCEL_RETRY_NANOS:
      self._cancel_unsupported_since = None
    self.cancel_unsupported = self._cancel_unsupported_since is not None

    if self.state != CruiseState.FAULT and not (lamps_valid and clu1_fresh and precond):
      if precond:
        self._precond_lost = None
      elif self._precond_lost is None:
        self._precond_lost = now
      elif now - self._precond_lost >= CruiseParams.PRECOND_GRACE_NANOS:
        self._disarm()
      self._goto(CruiseState.INIT, now)
      return
    self._precond_lost = None

    if self._session_over(brake):
      if self.state == CruiseState.PRESS_MAIN_OFF:
        self._disarm()
      else:
        self._switch_off(now)
      return

    if self.state == CruiseState.INIT:
      if lamp_main and lamp_set:
        self._goto(CruiseState.ACTIVE, now)
      elif lamp_main and self.armed:
        self._goto(CruiseState.COAST, now)
      elif lamp_main and self._engaging:
        self._goto(CruiseState.WAIT_SET_READY, now)
      elif lamp_main:
        self._switch_off(now)
      elif self.armed and self._main_off_coast:
        self._goto(CruiseState.COAST, now)
      else:
        self._goto(CruiseState.IDLE, now)

    elif self.state == CruiseState.IDLE:
      if long_press:
        self._goto(CruiseState.PRESS_MAIN_ON, now)
        self._engaging = True
        self._main_attempts = 0
        self._set_attempts = 0

    elif self.state == CruiseState.PRESS_MAIN_ON:
      done = self._tap(now, lamp_main, CruiseParams.MAIN_PRESS_NANOS, CruiseParams.MAIN_SETTLE_NANOS)
      self.sw_main = 1 if self._pressing else 0
      # A MAIN edge may already be on its way, so an off request waits for this press to resolve.
      self._off_requested |= long_press
      if done is True and self._off_requested:
        self._switch_off(now)
      elif done is True:
        self._goto(CruiseState.WAIT_SET_READY, now)
      elif done is False and self._off_requested:
        self._goto(CruiseState.IDLE, now)
      elif done is False:
        self._main_attempts += 1
        if self._main_attempts >= CruiseParams.MAX_PRESS_ATTEMPTS:
          self._goto(CruiseState.FAULT, now)

    elif self.state == CruiseState.WAIT_SET_READY:
      in_range = CruiseParams.SET_SPEED_MIN_KPH <= v_cluster_kph <= CruiseParams.SET_SPEED_MAX_KPH
      if long_press or now - self._entered >= CruiseParams.SET_READY_TIMEOUT_NANOS:
        self._switch_off(now)
      elif lamp_set:
        self._goto(CruiseState.ACTIVE, now)
      elif not lamp_main:
        self._goto(CruiseState.IDLE, now)
      elif self._recapture and (not in_range or coast_requested):
        self._goto(CruiseState.COAST, now)
      elif now - self._entered >= CruiseParams.GAP_NANOS and in_range:
        if self._ready_since is None:
          self._ready_since = now
        elif now - self._ready_since >= CruiseParams.SET_READY_DWELL_NANOS:
          self._goto(CruiseState.PRESS_SET, now)
      else:
        self._ready_since = None

    elif self.state == CruiseState.PRESS_SET:
      done = self._tap(now, lamp_set, CruiseParams.SET_PRESS_NANOS, CruiseParams.SET_SETTLE_NANOS)
      self.sw_state = CLU1_SW_SET if self._pressing else CLU1_SW_NONE
      if long_press:
        self._switch_off(now)
      elif not lamp_main:
        self._goto(CruiseState.IDLE, now)
      elif done is True:
        self._set_attempts = 0
        self._goto(CruiseState.ACTIVE, now)
      elif done is False and self._recapture:
        self.set_rejected_nanos = now
        self._goto(CruiseState.COAST, now)
      elif done is False:
        self._set_attempts += 1
        if self._set_attempts >= CruiseParams.MAX_PRESS_ATTEMPTS:
          self._goto(CruiseState.FAULT, now)
        else:
          self._goto(CruiseState.WAIT_SET_READY, now)

    elif self.state == CruiseState.ACTIVE:
      if long_press:
        self._switch_off(now)
      elif not lamp_set:
        self._on_set_lamp_lost(now, follow_active)
      elif request == ButtonRequest.CANCEL:
        self._start_cancel(now)
      elif request in (ButtonRequest.RES, ButtonRequest.DECEL):
        self._goto(CruiseState.TAP, now)
        self._tap_button = CLU1_SW_RES if request == ButtonRequest.RES else CLU1_SW_SET
        self._tap_dir = 1 if request == ButtonRequest.RES else -1
        self._tap_released = False
        self.sw_state = self._tap_button

    elif self.state == CruiseState.TAP:
      if long_press:
        self._switch_off(now)
      elif not lamp_set:
        self._goto(CruiseState.ACTIVE, now)
      elif now - self._entered < CruiseParams.TAP_NANOS:
        self.sw_state = self._tap_button
      else:
        self.sw_state = CLU1_SW_NONE
        if not self._tap_released:
          self._tap_released = True
          self.tap_event = self._tap_dir
        if now - self._entered >= CruiseParams.TAP_NANOS + CruiseParams.TAP_GAP_NANOS:
          self._goto(CruiseState.ACTIVE, now)

    elif self.state == CruiseState.PRESS_CANCEL:
      done = self._tap(now, not lamp_set, CruiseParams.CANCEL_PRESS_NANOS, CruiseParams.CANCEL_SETTLE_NANOS)
      self.sw_state = CLU1_SW_CANCEL if self._pressing else CLU1_SW_NONE
      if long_press:
        self._switch_off(now)
      elif not lamp_main:
        self._goto(CruiseState.IDLE, now)
      elif done is True:
        self._cancel_failures.clear()
        self._cancel_unsupported_since = None
        self._goto(CruiseState.COAST, now)
      elif done is False:
        self._cancel_attempts += 1
        if self._cancel_attempts >= CruiseParams.CANCEL_MAX_ATTEMPTS:
          self._record_cancel_failure(now)
          self._coast_with_main_off(now)

    elif self.state == CruiseState.COAST:
      slow = v_cluster_kph < FollowParams.COAST_DISARM_SPEED_KPH and not coast_requested
      if long_press or slow:
        self._switch_off(now)
      elif lamp_set:
        self._goto(CruiseState.ACTIVE, now)
      elif not lamp_main and not self._main_off_coast:
        self._goto(CruiseState.IDLE, now)
      elif request == ButtonRequest.SET:
        self._recapture = True
        self._main_off_coast = False
        self._main_attempts = 0
        self._goto(CruiseState.PRESS_SET if lamp_main else CruiseState.PRESS_MAIN_ON, now)

    elif self.state == CruiseState.PRESS_MAIN_OFF:
      done = self._tap(now, not lamp_main, CruiseParams.MAIN_PRESS_NANOS, CruiseParams.MAIN_SETTLE_NANOS)
      self.sw_main = 1 if self._pressing else 0
      if done is True:
        self._goto(CruiseState.COAST if self.armed and self._main_off_coast else CruiseState.IDLE, now)
      elif done is False:
        self._main_attempts += 1
        if self._main_attempts >= CruiseParams.MAX_PRESS_ATTEMPTS:
          self._goto(CruiseState.FAULT, now)

    elif self.state == CruiseState.FAULT:
      if long_press:
        self._goto(CruiseState.IDLE, now)
      elif self.armed and not lamp_set:
        self._disarm()
      elif lamp_main and now - self._entered >= CruiseParams.FAULT_RETRY_NANOS:
        self._main_attempts = 0
        self._goto(CruiseState.PRESS_MAIN_OFF, now)

  def _session_over(self, brake: bool | None) -> bool:
    """A brake ends the session anywhere; without brake information only the cruise itself may keep going."""
    if not self.armed or self.state == CruiseState.FAULT:
      return False
    if brake is True:
      return True
    return brake is None and self.state not in (CruiseState.INIT, CruiseState.ACTIVE, CruiseState.TAP,
                                                CruiseState.PRESS_CANCEL, CruiseState.PRESS_MAIN_OFF)

  def _on_set_lamp_lost(self, now: int, follow_active: bool) -> None:
    while self._drops and now - self._drops[0] > CruiseParams.UNEXPECTED_DROP_WINDOW_NANOS:
      self._drops.popleft()

    if not follow_active or len(self._drops) + 1 >= CruiseParams.UNEXPECTED_DROP_LIMIT:
      self._drops.clear()
      self._switch_off(now)
    else:
      self._drops.append(now)
      self._goto(CruiseState.COAST, now)

  def _record_cancel_failure(self, now: int) -> None:
    if not self._cancel_failures or now - self._cancel_failures[-1] >= CruiseParams.CANCEL_EPISODE_SPACING_NANOS:
      self._cancel_failures.append(now)
    if len(self._cancel_failures) >= CruiseParams.CANCEL_FAILED_EPISODES:
      self._cancel_failures.clear()
      self._cancel_unsupported_since = now
      self.cancel_unsupported = True

  def _coast_with_main_off(self, now: int) -> None:
    self._main_off_coast = True
    self._main_attempts = 0
    self._goto(CruiseState.PRESS_MAIN_OFF, now)

  def _start_cancel(self, now: int) -> None:
    if self.cancel_unsupported:
      self._coast_with_main_off(now)
    else:
      self._cancel_attempts = 0
      self._goto(CruiseState.PRESS_CANCEL, now)
