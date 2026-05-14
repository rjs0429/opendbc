from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.avante_md.avantecan import VSM1, VSM1_STALE_NANOS, vsm1_checksum_valid, vsm1_is_normal_state
from opendbc.car.avante_md.values import CanBus, DBC
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase

ButtonType = structs.CarState.ButtonEvent.Type

AVANTE_VALID_SAS_STAT = 7
AVANTE_MAX_WHEEL_SPEED_KPH = 220.
AVANTE_MAX_WHEEL_SPEED_SPREAD_KPH = 20.
AVANTE_MAX_STEERING_TORQUE_NM = 10.
AVANTE_MAX_STEERING_EPS_TORQUE_NM = 100.
AVANTE_STEERING_PRESSED_THRESHOLD = 150  # 1.5 Nm in 0.01 Nm units, matches safety driver_allowance
AVANTE_FAULT_PERMANENT_FRAMES = 10


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    self.last_wheel_speeds = None
    self.steering_angle_deg = 0.

    self.vsm1_rx_raw: bytes | None = None
    self.vsm1_rx_nanos = 0
    self.vsm1_normal = False
    self.can_update_nanos = 0

    self.vsm2_fault_count = 0
    self.vsm1_checksum_invalid_count = 0
    self.can_invalid_count = 0
    self.can_valid_seen_once = False
    self.steer_fault_permanent = False

    self.lat_active = False
    self.openpilot_enabled = False
    self.should_be_active = False

  def update_vsm1_raw(self, can_packets):
    for t, frames in can_packets:
      self.can_update_nanos = max(self.can_update_nanos, t)
      for addr, dat, src in frames:
        if src == CanBus.VEHICLE and addr == VSM1 and len(dat) == 8:
          self.vsm1_rx_raw = bytes(dat)
          self.vsm1_rx_nanos = t
          self.vsm1_normal = vsm1_is_normal_state(self.vsm1_rx_raw)

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
    ret.steeringTorque = 0 if vsm2_fault or not vsm2_torque_valid else round(steering_torque_nm * 100)
    ret.steeringTorqueEps = 0 if vsm2_fault or not vsm2_torque_valid else round(steering_torque_eps_nm * 100)
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > AVANTE_STEERING_PRESSED_THRESHOLD, 5)

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

    tcu1_drive = cp_vehicle.vl["TCU1"]["CUR_GR"] == 5
    tcu2_cur_gr = int(cp_vehicle.vl["TCU2"]["CUR_GR"])
    tcu2_drive = 1 <= tcu2_cur_gr <= 6
    if tcu1_drive and tcu2_drive:
      gear_str = "D"
    elif tcu2_cur_gr == 14:
      gear_str = "R"
    else:
      gear_str = None
    ret.gearShifter = self.parse_gear_shifter(gear_str)

    unsafe_vehicle_state = (not (tcu1_drive and tcu2_drive) or
                             ret.doorOpen or
                             ret.seatbeltUnlatched or
                             ret.parkingBrake)

    ret.cruiseState.available = True
    ret.cruiseState.enabled = False
    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = False

    should_be_active = not unsafe_vehicle_state and not ret.steerFaultTemporary and not ret.steerFaultPermanent
    self.should_be_active = should_be_active
    button_events = []
    if not should_be_active and self.lat_active:
      button_events.append(structs.CarState.ButtonEvent(type=ButtonType.cancel, pressed=False))
    self.lat_active = should_be_active
    ret.buttonEvents = button_events
    ret.buttonEnable = self.update_button_enable(button_events)

    return ret

  def update_button_enable(self, buttonEvents: list[structs.CarState.ButtonEvent]):
    return self.should_be_active and not self.openpilot_enabled

  @staticmethod
  def get_can_parsers(CP):
    vehicle_bus_messages = [
      ("VSM1", 100),
      ("TCS5", 50),
      ("CLU1", 50),
      ("CLU2", 10),
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
