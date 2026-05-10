#!/usr/bin/env python3
from opendbc.car import get_safety_config, structs
from opendbc.car.avante_md.carcontroller import CarController
from opendbc.car.avante_md.carstate import CarState
from opendbc.car.avante_md.values import CAR
from opendbc.car.interfaces import CarInterfaceBase


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "avante_md"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.avanteMd)]
    ret.radarUnavailable = True

    ret.openpilotLongitudinalControl = False
    ret.pcmCruise = False
    ret.autoResumeSng = False

    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.8
    ret.centerToFront = ret.wheelbase * 0.4

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if candidate == CAR.AVANTE_MD_2012:
      ret.minSteerSpeed = 0.

    return ret

  def update(self, can_packets):
    self.CS.update_vsm1_raw(can_packets)
    return super().update(can_packets)
