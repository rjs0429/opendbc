from opendbc.car.avante_md import avantecan
from opendbc.car.avante_md.avantecan import CLU1_STALE_NANOS, VSM1, VSM1_STALE_NANOS
from opendbc.car.avante_md.cruise import CruiseStateMachine
from opendbc.car.avante_md.values import CanBus, CarControllerParams
from opendbc.car.can_definitions import CanData
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_driver_steer_torque_limits

AVANTE_CONTROL_READY_STABILIZE_NANOS = 200_000_000


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(CP)
    self.apply_torque_last = 0
    self.control_ready_last = False
    self.control_ready_stable_last = False
    self.control_ready_start_nanos = 0
    self.cruise = CruiseStateMachine()

  def update(self, CC, CS, now_nanos):
    can_sends = []
    CS.openpilot_enabled = CC.enabled

    vsm1_fresh = CS.vsm1_rx_raw is not None and (now_nanos - CS.vsm1_rx_nanos) <= VSM1_STALE_NANOS
    control_ready = (CC.latActive and
                     CS.vsm1_normal and
                     vsm1_fresh and
                     not CS.out.steerFaultTemporary and
                     not CS.out.steerFaultPermanent)

    if control_ready and not self.control_ready_last:
      self.control_ready_start_nanos = now_nanos
    elif not control_ready:
      self.control_ready_start_nanos = 0
    control_ready_stable = (control_ready and
                            (now_nanos - self.control_ready_start_nanos) >= AVANTE_CONTROL_READY_STABILIZE_NANOS)
    reset_torque_history = not self.control_ready_stable_last

    apply_torque = 0
    if control_ready_stable:
      new_torque = round(CC.actuators.torque * self.params.STEER_MAX * self.params.STEER_COMMAND_SIGN)
      torque_last = 0 if reset_torque_history else self.apply_torque_last
      apply_torque = apply_driver_steer_torque_limits(new_torque, torque_last,
                                                      CS.out.steeringTorque, self.params)
      can_sends.append(avantecan.create_vsm1(CS.vsm1_rx_raw, apply_torque, True))
    elif control_ready:
      can_sends.append(avantecan.create_vsm1(CS.vsm1_rx_raw, 0, False))
    elif CS.vsm1_rx_raw is not None:
      can_sends.append(CanData(VSM1, CS.vsm1_rx_raw, CanBus.EPS))

    clu1_fresh = CS.clu1_rx_raw is not None and (now_nanos - CS.clu1_rx_nanos) <= CLU1_STALE_NANOS
    self.cruise.update(now_nanos, CS.cruise_long_press, CS.cruise_lamps_valid, CS.cruise_lamp_main,
                       CS.cruise_lamp_set, CS.out.vEgoCluster * CV.MS_TO_KPH, CS.cruise_precond, clu1_fresh)
    # The cluster's own released frame lands between ours, so a press is sent every cycle rather
    # than once per cluster frame.
    if self.cruise.transmitting and clu1_fresh:
      can_sends.append(avantecan.create_clu1(CS.clu1_rx_raw, self.cruise.sw_state, self.cruise.sw_main))

    self.apply_torque_last = apply_torque if control_ready_stable else 0
    self.control_ready_last = control_ready
    self.control_ready_stable_last = control_ready_stable

    new_actuators = CC.actuators.as_builder()
    if control_ready_stable:
      new_actuators.torque = apply_torque / self.params.STEER_MAX * self.params.STEER_COMMAND_SIGN
      new_actuators.torqueOutputCan = apply_torque
    else:
      new_actuators.torque = 0.
      new_actuators.torqueOutputCan = 0

    self.frame += 1
    return new_actuators, can_sends
