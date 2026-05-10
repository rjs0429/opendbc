from opendbc.car.avante_md import avantecan
from opendbc.car.avante_md.avantecan import VSM1_STALE_NANOS, vsm1_is_normal_state
from opendbc.car.avante_md.values import CarControllerParams
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_driver_steer_torque_limits


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(CP)
    self.apply_torque_last = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []

    raw_vsm1_fresh = CS.vsm1_rx_raw is not None and (now_nanos - CS.vsm1_rx_nanos) <= VSM1_STALE_NANOS
    vsm1_normal = raw_vsm1_fresh and vsm1_is_normal_state(CS.vsm1_rx_raw)

    apply_torque = 0
    if vsm1_normal:
      if CC.latActive:
        new_torque = int(round(CC.actuators.torque * self.params.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                        CS.out.steeringTorque, self.params)
      can_sends.append(avantecan.create_vsm1(CS.vsm1_rx_raw, apply_torque, CC.latActive))

    self.apply_torque_last = apply_torque

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends
