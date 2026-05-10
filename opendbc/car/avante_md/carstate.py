from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.avante_md.avantecan import VSM1
from opendbc.car.avante_md.values import CanBus, DBC
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase

AVANTE_GEAR_SHIFTER_VALUES = {
  0: "P",
  5: "D",
  6: "N",
  7: "R",
  8: "S",
  12: "T",
}

AVANTE_CUR_GR_VALUES = {
  0: "P",
  1: "D", 2: "D", 3: "D", 4: "D",
  5: "D", 6: "D", 7: "D", 8: "D",
  14: "R",
}

AVANTE_VALID_SAS_STAT = 7
AVANTE_MAX_WHEEL_SPEED_KPH = 220.
AVANTE_MAX_WHEEL_SPEED_SPREAD_KPH = 20.
AVANTE_MAX_STEERING_TORQUE_NM = 10.
AVANTE_MAX_STEERING_EPS_TORQUE_NM = 100.
AVANTE_STEERING_PRESSED_THRESHOLD = 150  # 1.5 Nm in VSM1 torque units


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv.get("TCU4", {}).get("CR_Tcu_GearSelDisp2", AVANTE_GEAR_SHIFTER_VALUES)

    self.last_wheel_speeds = None
    self.steering_angle_deg = 0.
    self.vsm1_rx_raw: bytes | None = None
    self.vsm1_rx_nanos = 0

  def update_vsm1_raw(self, can_packets):
    for t, frames in can_packets:
      for addr, dat, src in frames:
        if src == CanBus.VEHICLE and addr == VSM1 and len(dat) == 8:
          self.vsm1_rx_raw = bytes(dat)
          self.vsm1_rx_nanos = t

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    ret = structs.CarState()

    self.is_metric = cp.vl["CLU1"]["CF_Clu_SPEED_UNIT"] == 0
    speed_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    wheel_speeds = (
      cp.vl["TCS5"]["WHEEL_FL"],
      cp.vl["TCS5"]["WHEEL_FR"],
      cp.vl["TCS5"]["WHEEL_RL"],
      cp.vl["TCS5"]["WHEEL_RR"],
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
    ret.vEgoCluster = cp.vl["CLU1"]["CF_Clu_Vanz"] * speed_conv

    sas_valid = cp.vl["SAS1"]["SAS_Stat"] == AVANTE_VALID_SAS_STAT
    if sas_valid:
      self.steering_angle_deg = cp.vl["SAS1"]["SAS_Angle"]
      ret.steeringRateDeg = cp.vl["SAS1"]["SAS_Speed"]
    else:
      ret.steeringRateDeg = 0.
    ret.steeringAngleDeg = self.steering_angle_deg

    vsm2_fault = cp.vl["VSM2"]["CF_Mdps_Def"] != 0 or cp.vl["VSM2"]["CF_Mdps_SErr"] != 0
    steering_torque_nm = cp.vl["VSM2"]["CR_Mdps_StrTq"]
    steering_torque_eps_nm = cp.vl["VSM2"]["CR_Mdps_OutTq"]
    vsm2_torque_valid = abs(steering_torque_nm) <= AVANTE_MAX_STEERING_TORQUE_NM and \
                        abs(steering_torque_eps_nm) <= AVANTE_MAX_STEERING_EPS_TORQUE_NM
    ret.steeringTorque = 0 if vsm2_fault or not vsm2_torque_valid else round(steering_torque_nm * 100)
    ret.steeringTorqueEps = 0 if vsm2_fault or not vsm2_torque_valid else round(steering_torque_eps_nm * 100)
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > AVANTE_STEERING_PRESSED_THRESHOLD, 5)
    ret.steerFaultTemporary = not sas_valid or vsm2_fault or not vsm2_torque_valid

    ret.brake = 0
    ret.brakePressed = cp.vl["TCU2"]["BRAKE_ACT_TCU"] != 0
    ret.parkingBrake = cp.vl["CLU1"]["CF_Clu_ParkBrakeSw"] != 0
    ret.espDisabled = cp.vl["TCS1"]["TCS_PAS"] == 1
    ret.espActive = cp.vl["TCS1"]["ABS_ACT"] == 1
    ret.gasPressed = bool(cp.vl["EMS6"]["CF_Ems_AclAct"])

    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(
      120, cp.vl["CLU2"]["CF_Clu_TurnSigLh"], cp.vl["CLU2"]["CF_Clu_TurnSigRh"])
    ret.doorOpen = any([
      cp.vl["CLU2"]["CF_Clu_DrvDrSw"],
      cp.vl["CLU2"]["CF_Clu_AstDrSw"],
    ])
    ret.seatbeltUnlatched = cp.vl["CLU2"]["CF_Clu_DrvSeatBeltSw"] == 0

    gear = self.shifter_values.get(cp.vl["TCU4"]["CR_Tcu_GearSelDisp2"])
    if gear is None:
      gear = AVANTE_CUR_GR_VALUES.get(cp.vl["TCU2"]["CUR_GR"])
    ret.gearShifter = self.parse_gear_shifter(gear)

    ret.cruiseState.available = cp.vl["CLU1"]["CF_Clu_CruiseSwMain"] != 0
    ret.cruiseState.enabled = False
    ret.cruiseState.standstill = False
    ret.cruiseState.nonAdaptive = False

    return ret

  @staticmethod
  def get_can_parsers(CP):
    messages = [
      ("TCS1", 100),
      ("TCS5", 50),
      ("SAS1", 100),
      ("VSM2", 100),
      ("EMS6", 100),
      ("CLU1", 50),
      ("CLU2", 10),
      ("TCU2", 100),
      ("TCU4", 10),
    ]
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], messages, CanBus.VEHICLE),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus.EPS),
    }
