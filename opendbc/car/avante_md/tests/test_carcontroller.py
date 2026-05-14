#!/usr/bin/env python3
import unittest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.avante_md.avantecan import VSM1, vsm1_checksum
from opendbc.car.avante_md.carcontroller import AVANTE_CONTROL_READY_STABILIZE_NANOS, CarController
from opendbc.car.avante_md.interface import CarInterface
from opendbc.car.avante_md.values import CAR, CanBus


def normal_vsm1():
  dat = bytearray(8)
  dat[1] = 0x08
  dat[7] = vsm1_checksum(dat)
  return bytes(dat)


def vsm1_torque(dat: bytes) -> int:
  raw_torque = dat[0] | ((dat[1] & 0x0f) << 8)
  return raw_torque - 2048


class FakeCarState:
  def __init__(self):
    self.vsm1_rx_raw = normal_vsm1()
    self.vsm1_rx_nanos = 0
    self.vsm1_normal = True
    self.openpilot_enabled = False
    self.out = structs.CarState()


class TestAvanteMdCarController(unittest.TestCase):
  def _setup_controller(self):
    CP = CarInterface.get_params(CAR.AVANTE_MD_2012, gen_empty_fingerprint(), [], False, False, False)
    CC = structs.CarControl.new_message()
    CC.latActive = True
    controller = CarController({}, CP)
    CS = FakeCarState()
    return controller, CC, CS

  @staticmethod
  def _update(controller, CC, CS, now_nanos):
    CS.vsm1_rx_nanos = now_nanos
    return controller.update(CC.as_reader(), CS, now_nanos)

  def _update_with_torque(self, torque: float, enabled: bool = False):
    controller, CC, CS = self._setup_controller()
    CC.enabled = enabled
    CC.actuators.torque = torque

    self._update(controller, CC, CS, 0)
    actuators, can_sends = self._update(controller, CC, CS, AVANTE_CONTROL_READY_STABILIZE_NANOS)

    self.assertEqual(1, len(can_sends))
    self.assertEqual(VSM1, can_sends[0].address)
    self.assertEqual(CanBus.EPS, can_sends[0].src)
    return actuators, vsm1_torque(can_sends[0].dat), CS

  def test_positive_openpilot_torque_sends_negative_vsm1_torque(self):
    actuators, torque, _ = self._update_with_torque(0.5)

    self.assertLess(torque, 0)
    self.assertGreater(actuators.torque, 0.)
    self.assertEqual(torque, actuators.torqueOutputCan)

  def test_negative_openpilot_torque_sends_positive_vsm1_torque(self):
    actuators, torque, _ = self._update_with_torque(-0.5)

    self.assertGreater(torque, 0)
    self.assertLess(actuators.torque, 0.)
    self.assertEqual(torque, actuators.torqueOutputCan)

  def test_syncs_openpilot_enabled_to_carstate(self):
    _, _, CS = self._update_with_torque(0., enabled=True)
    self.assertTrue(CS.openpilot_enabled)

  def test_syncs_openpilot_disabled_to_carstate(self):
    _, _, CS = self._update_with_torque(0., enabled=False)
    self.assertFalse(CS.openpilot_enabled)

  def test_torque_ramps_from_zero_at_safety_rate(self):
    controller, CC, CS = self._setup_controller()
    CC.actuators.torque = -1.

    _, can_sends = self._update(controller, CC, CS, 0)
    self.assertEqual(0, vsm1_torque(can_sends[0].dat))

    _, can_sends = self._update(controller, CC, CS, AVANTE_CONTROL_READY_STABILIZE_NANOS)
    self.assertEqual(800, vsm1_torque(can_sends[0].dat))

    _, can_sends = self._update(controller, CC, CS, AVANTE_CONTROL_READY_STABILIZE_NANOS + 10_000_000)
    self.assertEqual(800, vsm1_torque(can_sends[0].dat))

  def test_torque_restarts_after_lat_inactive(self):
    controller, CC, CS = self._setup_controller()
    CC.actuators.torque = -1.

    for frame in range(3):
      self._update(controller, CC, CS, AVANTE_CONTROL_READY_STABILIZE_NANOS + frame * 10_000_000)

    CC.latActive = False
    actuators, can_sends = self._update(controller, CC, CS, AVANTE_CONTROL_READY_STABILIZE_NANOS + 30_000_000)
    self.assertEqual(0, actuators.torqueOutputCan)
    self.assertEqual(normal_vsm1(), can_sends[0].dat)

    CC.latActive = True
    _, can_sends = self._update(controller, CC, CS, AVANTE_CONTROL_READY_STABILIZE_NANOS + 40_000_000)
    self.assertEqual(0, vsm1_torque(can_sends[0].dat))

    _, can_sends = self._update(controller, CC, CS, AVANTE_CONTROL_READY_STABILIZE_NANOS + 240_000_000)
    self.assertEqual(800, vsm1_torque(can_sends[0].dat))

  def test_torque_restarts_on_steering_pressed_rising_edge(self):
    controller, CC, CS = self._setup_controller()
    CC.actuators.torque = -1.

    _, can_sends = self._update(controller, CC, CS, 0)
    self.assertEqual(0, vsm1_torque(can_sends[0].dat))

    _, can_sends = self._update(controller, CC, CS, AVANTE_CONTROL_READY_STABILIZE_NANOS)
    self.assertEqual(800, vsm1_torque(can_sends[0].dat))

    _, can_sends = self._update(controller, CC, CS, AVANTE_CONTROL_READY_STABILIZE_NANOS + 10_000_000)
    self.assertEqual(800, vsm1_torque(can_sends[0].dat))

    CS.out.steeringPressed = True
    _, can_sends = self._update(controller, CC, CS, AVANTE_CONTROL_READY_STABILIZE_NANOS + 20_000_000)
    self.assertEqual(800, vsm1_torque(can_sends[0].dat))


if __name__ == "__main__":
  unittest.main()
