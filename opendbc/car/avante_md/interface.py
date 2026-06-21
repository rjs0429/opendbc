#!/usr/bin/env python3
from opendbc.car import get_safety_config, structs
from opendbc.car.avante_md.carcontroller import CarController
from opendbc.car.avante_md.carstate import CarState
from opendbc.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  DRIVABLE_GEARS = (structs.CarState.GearShifter.sport,)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "avante_md"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.avanteMd)]
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = False
    ret.openpilotLongitudinalControl = False
    ret.pcmCruise = False
    ret.autoResumeSng = False
    ret.minEnableSpeed = -1.

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.steerAtStandstill = True
    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.8
    ret.minSteerSpeed = 0.
    ret.centerToFront = ret.wheelbase * 0.4

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, steering_angle_deadzone_deg=1.0)

    return ret

  def update(self, can_packets):
    self.CS.update_vsm1_raw(can_packets)
    return super().update(can_packets)
