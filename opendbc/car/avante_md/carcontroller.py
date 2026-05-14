from opendbc.car.avante_md import avantecan
from opendbc.car.avante_md.avantecan import VSM1, VSM1_STALE_NANOS
from opendbc.car.avante_md.values import CanBus, CarControllerParams
from opendbc.car.can_definitions import CanData
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_driver_steer_torque_limits


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(CP)
    self.apply_torque_last = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []
    CS.openpilot_enabled = CC.enabled

    vsm1_fresh = CS.vsm1_rx_raw is not None and (now_nanos - CS.vsm1_rx_nanos) <= VSM1_STALE_NANOS
    control_ready = (CC.latActive and
                     CS.vsm1_normal and
                     vsm1_fresh and
                     not CS.out.steerFaultTemporary and
                     not CS.out.steerFaultPermanent)

    apply_torque = 0
    if control_ready:
      new_torque = round(CC.actuators.torque * self.params.STEER_MAX * self.params.STEER_COMMAND_SIGN)
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, self.params)
      can_sends.append(avantecan.create_vsm1(CS.vsm1_rx_raw, apply_torque, True))
    elif CS.vsm1_rx_raw is not None:
      can_sends.append(CanData(VSM1, CS.vsm1_rx_raw, CanBus.EPS))

    self.apply_torque_last = apply_torque if control_ready else 0

    new_actuators = CC.actuators.as_builder()
    if control_ready:
      new_actuators.torque = apply_torque / self.params.STEER_MAX * self.params.STEER_COMMAND_SIGN
      new_actuators.torqueOutputCan = apply_torque
    else:
      new_actuators.torque = 0.
      new_actuators.torqueOutputCan = 0

    self.frame += 1
    return new_actuators, can_sends
