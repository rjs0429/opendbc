import time

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.avante_md.avantecan import CLU1, VSM1, VSM1_STALE_NANOS, vsm1_checksum_valid, vsm1_is_normal_state
from opendbc.car.avante_md.values import CanBus, CarControllerParams, CruiseParams, DBC
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase

ButtonType = structs.CarState.ButtonEvent.Type

AVANTE_VALID_SAS_STAT = 7
AVANTE_MAX_WHEEL_SPEED_KPH = 220.
AVANTE_MAX_WHEEL_SPEED_SPREAD_KPH = 20.
AVANTE_MAX_STEERING_TORQUE_NM = 10.
AVANTE_MAX_STEERING_EPS_TORQUE_NM = 100.
AVANTE_FAULT_PERMANENT_FRAMES = 10
AVANTE_DRIVER_TORQUE_SIGN = -1  # VSM2 driver torque is right-positive; openpilot expects left-positive.


class MomentaryButtonDoubleClick:
  DOUBLE_CLICK_INTERVAL = 3.0  # seconds
  DEBOUNCE_FRAMES = 3  # 30 ms at 100 Hz CAN

  def __init__(self):
    self._prev_btn: int | None = None
    self._raw_btn: int | None = None
    self._raw_count: int = 0
    self._last_click_time: float | None = None

  def reset(self) -> tuple[bool, bool]:
    had_pending = self._last_click_time is not None
    self._prev_btn = None
    self._raw_btn = None
    self._raw_count = 0
    self._last_click_time = None
    return had_pending, False

  def update(self, current_btn: int) -> tuple[bool, bool]:
    now = time.monotonic()
    current_btn = 1 if current_btn else 0

    if current_btn == self._raw_btn:
      if self._raw_count < self.DEBOUNCE_FRAMES:
        self._raw_count += 1
    else:
      self._raw_btn = current_btn
      self._raw_count = 1
    if self._raw_count < self.DEBOUNCE_FRAMES:
      return False, False

    # First call: sync prev state without emitting a spurious edge.
    if self._prev_btn is None:
      self._prev_btn = current_btn
      return False, False

    any_edge = (self._prev_btn != current_btn)
    self._prev_btn = current_btn

    single_click = False
    double_click = False

    if any_edge:
      if self._last_click_time is not None:
        elapsed = now - self._last_click_time
        if elapsed <= self.DOUBLE_CLICK_INTERVAL:
          double_click = True
          self._last_click_time = None
        else:
          # Timeout expired before second press: report first as single, start new sequence.
          single_click = True
          self._last_click_time = now
      else:
        self._last_click_time = now
    elif self._last_click_time is not None:
      if (now - self._last_click_time) > self.DOUBLE_CLICK_INTERVAL:
        single_click = True
        self._last_click_time = None

    return single_click, double_click


class EcoLongPress:
  """ECO switch held long enough to toggle stock cruise.

  While a hold is long enough to count, it is hidden from the lateral-control double-click
  detector: that detector counts switch edges, so the release edge of a cruise hold would
  otherwise toggle steering as well.
  """
  DEBOUNCE_FRAMES = MomentaryButtonDoubleClick.DEBOUNCE_FRAMES

  def __init__(self, hold_nanos: int):
    self.hold_nanos = hold_nanos
    self.consumed = False
    self._level = 0
    self._raw_btn = 0
    self._raw_count = 0
    self._press_nanos: int | None = None
    self._fired = False

  def update(self, current_btn: int, now_nanos: int) -> bool:
    current_btn = 1 if current_btn else 0

    if current_btn == self._raw_btn:
      if self._raw_count < self.DEBOUNCE_FRAMES:
        self._raw_count += 1
    else:
      self._raw_btn = current_btn
      self._raw_count = 1

    if self._raw_count >= self.DEBOUNCE_FRAMES and current_btn != self._level:
      self._level = current_btn
      self._press_nanos = now_nanos if current_btn else None
      self._fired = False

    long_press = (self._level == 1 and not self._fired and self._press_nanos is not None and
                  (now_nanos - self._press_nanos) >= self.hold_nanos)
    if long_press:
      self._fired = True

    self.consumed = self._fired and self._level == 1
    return long_press


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    self.last_wheel_speeds = None
    self.steering_angle_deg = 0.

    self.vsm1_rx_raw: bytes | None = None
    self.vsm1_rx_nanos = 0
    self.vsm1_normal = False
    self.clu1_rx_raw: bytes | None = None
    self.clu1_rx_nanos = 0
    self.can_update_nanos = 0

    self.vsm2_fault_count = 0
    self.vsm1_checksum_invalid_count = 0
    self.can_invalid_count = 0
    self.can_valid_seen_once = False
    self.steer_fault_permanent = False

    self.lat_active = False
    self.openpilot_enabled = False
    self.should_be_active = False

    self.eco_btn = MomentaryButtonDoubleClick()
    self.eco_long = EcoLongPress(CruiseParams.HOLD_NANOS)
    self.openpilot_requested = True

    self.cruise_long_press = False
    self.cruise_lamp_main = False
    self.cruise_lamp_set = False
    self.cruise_lamps_valid = False
    self.cruise_precond = False

  def update_raw_frames(self, can_packets):
    for t, frames in can_packets:
      self.can_update_nanos = max(self.can_update_nanos, t)
      for addr, dat, src in frames:
        if src != CanBus.VEHICLE or len(dat) != 8:
          continue
        if addr == VSM1:
          self.vsm1_rx_raw = bytes(dat)
          self.vsm1_rx_nanos = t
          self.vsm1_normal = vsm1_is_normal_state(self.vsm1_rx_raw)
        elif addr == CLU1:
          self.clu1_rx_raw = bytes(dat)
          self.clu1_rx_nanos = t

  def update(self, can_parsers) -> structs.CarState:
    cp_vehicle = can_parsers[Bus.pt]
    cp_eps = can_parsers[Bus.adas]
    ret = structs.CarState()

    self.is_metric = cp_vehicle.vl["CLU1"]["CF_Clu_SPEED_UNIT"] == 0
    speed_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    wheel_speeds = (
      cp_vehicle.vl["TCS5"]["WHEEL_FL"],
      cp_vehicle.vl["TCS5"]["WHEEL_FR"],
      cp_vehicle.vl["TCS5"]["WHEEL_RL"],
      cp_vehicle.vl["TCS5"]["WHEEL_RR"],
    )
    wheel_speed_spread = max(wheel_speeds) - min(wheel_speeds)
    wheel_speeds_valid = max(wheel_speeds) <= AVANTE_MAX_WHEEL_SPEED_KPH and wheel_speed_spread <= AVANTE_MAX_WHEEL_SPEED_SPREAD_KPH
    if wheel_speeds_valid:
      self.last_wheel_speeds = wheel_speeds
    elif self.last_wheel_speeds is not None:
      wheel_speeds = self.last_wheel_speeds
    else:
      wheel_speeds = (0., 0., 0., 0.)

    self.parse_wheel_speeds(ret, *wheel_speeds)
    ret.standstill = ret.vEgoRaw < 0.1
    ret.vEgoCluster = cp_vehicle.vl["CLU1"]["CF_Clu_Vanz"] * speed_conv

    sas_valid = cp_eps.vl["SAS1"]["SAS_Stat"] == AVANTE_VALID_SAS_STAT
    if sas_valid:
      self.steering_angle_deg = cp_eps.vl["SAS1"]["SAS_Angle"]
      ret.steeringRateDeg = cp_eps.vl["SAS1"]["SAS_Speed"]
    else:
      ret.steeringRateDeg = 0.
    ret.steeringAngleDeg = self.steering_angle_deg

    vsm2_fault = cp_eps.vl["VSM2"]["CF_Mdps_Def"] != 0 or cp_eps.vl["VSM2"]["CF_Mdps_SErr"] != 0
    steering_torque_nm = cp_eps.vl["VSM2"]["CR_Mdps_StrTq"]
    steering_torque_eps_nm = cp_eps.vl["VSM2"]["CR_Mdps_OutTq"]
    vsm2_torque_valid = (abs(steering_torque_nm) <= AVANTE_MAX_STEERING_TORQUE_NM and
                         abs(steering_torque_eps_nm) <= AVANTE_MAX_STEERING_EPS_TORQUE_NM)
    ret.steeringTorque = 0 if vsm2_fault or not vsm2_torque_valid else AVANTE_DRIVER_TORQUE_SIGN * round(steering_torque_nm * 100)
    ret.steeringTorqueEps = 0 if vsm2_fault or not vsm2_torque_valid else round(steering_torque_eps_nm * 100)
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)

    vsm1_stale = self.vsm1_rx_raw is None or (self.can_update_nanos - self.vsm1_rx_nanos) > VSM1_STALE_NANOS
    vsm1_checksum_invalid = not vsm1_stale and not vsm1_checksum_valid(self.vsm1_rx_raw)

    can_valid = cp_vehicle.can_valid and cp_eps.can_valid
    ret.steerFaultTemporary = (not sas_valid or
                                vsm2_fault or
                                not vsm2_torque_valid or
                                vsm1_stale or
                                not self.vsm1_normal or
                                not can_valid)

    self.vsm2_fault_count = self.vsm2_fault_count + 1 if vsm2_fault else 0
    self.vsm1_checksum_invalid_count = self.vsm1_checksum_invalid_count + 1 if vsm1_checksum_invalid else 0
    if can_valid:
      self.can_valid_seen_once = True
      self.can_invalid_count = 0
    elif self.can_valid_seen_once:
      self.can_invalid_count += 1
    else:
      self.can_invalid_count = 0
    if (self.vsm2_fault_count >= AVANTE_FAULT_PERMANENT_FRAMES or
        self.vsm1_checksum_invalid_count >= AVANTE_FAULT_PERMANENT_FRAMES or
        self.can_invalid_count >= AVANTE_FAULT_PERMANENT_FRAMES):
      self.steer_fault_permanent = True
    ret.steerFaultPermanent = self.steer_fault_permanent

    ret.brake = 0
    ret.brakePressed = False
    ret.gasPressed = False
    ret.parkingBrake = cp_vehicle.vl["CLU1"]["CF_Clu_ParkBrakeSw"] != 0

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(
      120, cp_vehicle.vl["CLU2"]["CF_Clu_TurnSigLh"], cp_vehicle.vl["CLU2"]["CF_Clu_TurnSigRh"])
    ret.doorOpen = bool(cp_vehicle.vl["CLU2"]["CF_Clu_DrvDrSw"] or
                        cp_vehicle.vl["CLU2"]["CF_Clu_AstDrSw"])
    ret.seatbeltUnlatched = cp_vehicle.vl["CLU2"]["CF_Clu_DrvSeatBeltSw"] != 0

    tcu1_cur_gr = int(cp_vehicle.vl["TCU1"]["CUR_GR"])
    tcu2_cur_gr = int(cp_vehicle.vl["TCU2"]["CUR_GR"])
    tcu1_drive = tcu1_cur_gr == 5
    tcu1_sport = tcu1_cur_gr == 8
    tcu2_forward = 1 <= tcu2_cur_gr <= 6
    if tcu1_drive and tcu2_forward:
      gear_str = "D"
    elif tcu1_sport and tcu2_forward:
      gear_str = "S"
    elif tcu1_cur_gr == 7 and tcu2_cur_gr == 14:
      gear_str = "R"
    elif tcu1_cur_gr == 6 and tcu2_cur_gr == 0:
      gear_str = "N"
    elif tcu1_cur_gr == 0 and tcu2_cur_gr == 0:
      gear_str = "P"
    else:
      gear_str = None
    ret.gearShifter = self.parse_gear_shifter(gear_str)

    forward_gear = (tcu1_drive or tcu1_sport) and tcu2_forward
    unsafe_vehicle_state = (not forward_gear or
                             ret.doorOpen or
                             ret.seatbeltUnlatched or
                             ret.parkingBrake)

    ret.cruiseState.available = True
    ret.cruiseState.enabled = False
    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = False

    should_be_active = not unsafe_vehicle_state and not ret.steerFaultTemporary and not ret.steerFaultPermanent
    self.should_be_active = should_be_active

    eco_signal = int(cp_vehicle.vl["CLU2"]["CF_Clu_ActiveEcoSW"])
    self.cruise_long_press = self.eco_long.update(eco_signal, self.can_update_nanos)
    if self.eco_long.consumed:
      self.eco_btn.reset()
    else:
      _, eco_double_click = self.eco_btn.update(eco_signal)
      if eco_double_click:
        self.openpilot_requested = not self.openpilot_requested

    self.cruise_lamp_main = cp_vehicle.vl["EMS6"]["CRUISE_LAMP_M"] != 0
    self.cruise_lamp_set = cp_vehicle.vl["EMS6"]["CRUISE_LAMP_S"] != 0
    self.cruise_lamps_valid = can_valid
    self.cruise_precond = can_valid and not unsafe_vehicle_state

    effective_active = should_be_active and self.openpilot_requested
    button_events = []
    if self.lat_active and not effective_active:
      button_events.append(structs.CarState.ButtonEvent(type=ButtonType.cancel, pressed=False))
    self.lat_active = effective_active
    ret.buttonEvents = button_events
    ret.buttonEnable = self.update_button_enable(button_events)

    return ret

  def update_button_enable(self, buttonEvents: list[structs.CarState.ButtonEvent]):
    return self.lat_active and not self.openpilot_enabled

  @staticmethod
  def get_can_parsers(CP):
    vehicle_bus_messages = [
      ("VSM1", 100),
      ("TCS5", 50),
      ("CLU1", 50),
      ("CLU2", 10),
      ("EMS6", 100),
      ("TCU1", 100),
      ("TCU2", 100),
    ]

    eps_bus_messages = [
      ("VSM2", 100),
      ("SAS1", 100),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], vehicle_bus_messages, CanBus.VEHICLE),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], eps_bus_messages, CanBus.EPS),
    }
