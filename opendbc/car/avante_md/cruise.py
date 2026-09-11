from enum import IntEnum

from opendbc.car.avante_md.avantecan import CLU1_SW_NONE, CLU1_SW_SET
from opendbc.car.avante_md.values import CruiseParams


class CruiseState(IntEnum):
  INIT = 0
  IDLE = 1
  PRESS_MAIN_ON = 2
  WAIT_SET_READY = 3
  PRESS_SET = 4
  ACTIVE = 5
  PRESS_MAIN_OFF = 6
  FAULT = 7


class CruiseStateMachine:
  """Drives the stock cruise control through CLU1 cruise switch bits.

  The ECM cruise lamps are the only state that is trusted: CruiseSwMain is an edge toggle, so an
  open-loop press would turn cruise off just as readily as on. Every press runs until the lamp
  confirms it, and anything the ECM cancels on its own (brake, sub-40 km/h, cut) drops the SET
  lamp, which returns the car to a fully off state.
  """

  def __init__(self):
    self.state = CruiseState.INIT
    self.sw_state = CLU1_SW_NONE
    self.sw_main = 0
    self._entered = 0
    self._press_start = 0
    self._retry_at = 0
    self._settle_until: int | None = None
    self._pressing = False
    self._main_attempts = 0
    self._set_attempts = 0
    self._ready_since: int | None = None

  @property
  def engaged(self) -> bool:
    return self.state == CruiseState.ACTIVE

  @property
  def transmitting(self) -> bool:
    return self.sw_state != CLU1_SW_NONE or self.sw_main != 0

  def _goto(self, state: CruiseState, now: int) -> None:
    self.state = state
    self.sw_state = CLU1_SW_NONE
    self.sw_main = 0
    self._entered = now
    self._press_start = now
    self._retry_at = now
    self._settle_until = None
    self._pressing = False
    self._ready_since = None

  def _press_main(self, now: int, lamp_main: bool, target_on: bool) -> bool | None:
    if lamp_main == target_on:
      self.sw_main = 0
      return True

    if self._pressing:
      self.sw_main = 1
      if now - self._press_start >= CruiseParams.MAIN_PRESS_MAX_NANOS:
        self.sw_main = 0
        self._pressing = False
        self._main_attempts += 1
        self._retry_at = now + CruiseParams.GAP_NANOS
    else:
      self.sw_main = 0
      if self._main_attempts >= CruiseParams.MAX_PRESS_ATTEMPTS:
        return False
      if now >= self._retry_at:
        self.sw_main = 1
        self._pressing = True
        self._press_start = now
    return None

  def _press_set(self, now: int, lamp_set: bool) -> bool | None:
    if lamp_set:
      self.sw_state = CLU1_SW_NONE
      return True

    if self._pressing:
      self.sw_state = CLU1_SW_SET
      if now - self._press_start >= CruiseParams.SET_PRESS_MAX_NANOS:
        self.sw_state = CLU1_SW_NONE
        self._pressing = False
        self._set_attempts += 1
        self._settle_until = now + CruiseParams.SETTLE_NANOS
    else:
      self.sw_state = CLU1_SW_NONE
      if self._settle_until is None:
        self.sw_state = CLU1_SW_SET
        self._pressing = True
        self._press_start = now
      elif now >= self._settle_until:
        return False
    return None

  def update(self, now: int, long_press: bool, lamps_valid: bool, lamp_main: bool, lamp_set: bool,
             v_cluster_kph: float, precond: bool, clu1_fresh: bool) -> None:
    if self.state != CruiseState.FAULT and not (lamps_valid and clu1_fresh and precond):
      self._goto(CruiseState.INIT, now)
      return

    if self.state == CruiseState.INIT:
      if lamp_main and lamp_set:
        self._goto(CruiseState.ACTIVE, now)
      elif lamp_main:
        self._goto(CruiseState.WAIT_SET_READY, now)
      else:
        self._goto(CruiseState.IDLE, now)

    elif self.state == CruiseState.IDLE:
      if long_press:
        self._goto(CruiseState.PRESS_MAIN_ON, now)
        self._main_attempts = 0
        self._set_attempts = 0

    elif self.state == CruiseState.PRESS_MAIN_ON:
      done = self._press_main(now, lamp_main, target_on=True)
      if done is True:
        self._goto(CruiseState.WAIT_SET_READY, now)
      elif done is False:
        self._goto(CruiseState.FAULT, now)

    elif self.state == CruiseState.WAIT_SET_READY:
      if long_press or now - self._entered >= CruiseParams.SET_READY_TIMEOUT_NANOS:
        self._goto(CruiseState.PRESS_MAIN_OFF, now)
        self._main_attempts = 0
      elif lamp_set:
        self._goto(CruiseState.ACTIVE, now)
      elif not lamp_main:
        self._goto(CruiseState.IDLE, now)
      elif (now - self._entered >= CruiseParams.GAP_NANOS and
            CruiseParams.SET_SPEED_MIN_KPH <= v_cluster_kph <= CruiseParams.SET_SPEED_MAX_KPH):
        if self._ready_since is None:
          self._ready_since = now
        elif now - self._ready_since >= CruiseParams.SET_READY_DWELL_NANOS:
          self._goto(CruiseState.PRESS_SET, now)
      else:
        self._ready_since = None

    elif self.state == CruiseState.PRESS_SET:
      done = self._press_set(now, lamp_set)
      if not lamp_main:
        self._goto(CruiseState.IDLE, now)
      elif done is True:
        self._goto(CruiseState.ACTIVE, now)
      elif done is False:
        if self._set_attempts >= CruiseParams.MAX_PRESS_ATTEMPTS:
          self._goto(CruiseState.FAULT, now)
        else:
          self._goto(CruiseState.WAIT_SET_READY, now)

    elif self.state == CruiseState.ACTIVE:
      if long_press or not lamp_set:
        self._goto(CruiseState.PRESS_MAIN_OFF, now)
        self._main_attempts = 0

    elif self.state == CruiseState.PRESS_MAIN_OFF:
      done = self._press_main(now, lamp_main, target_on=False)
      if done is True:
        self._goto(CruiseState.IDLE, now)
      elif done is False:
        self._goto(CruiseState.FAULT, now)

    elif self.state == CruiseState.FAULT:
      if long_press:
        self._goto(CruiseState.IDLE, now)
