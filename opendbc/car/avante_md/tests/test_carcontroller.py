#!/usr/bin/env python3
import unittest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.avante_md.avantecan import VSM1, vsm1_checksum
from opendbc.car.avante_md.carcontroller import CarController
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
    self.out = structs.CarState()


class TestAvanteMdCarController(unittest.TestCase):
  def _update_with_torque(self, torque: float):
    CP = CarInterface.get_params(CAR.AVANTE_MD_2012, gen_empty_fingerprint(), [], False, False, False)
    CC = structs.CarControl.new_message()
    CC.latActive = True
    CC.actuators.torque = torque

    controller = CarController({}, CP)
    actuators, can_sends = controller.update(CC.as_reader(), FakeCarState(), 0)

    self.assertEqual(1, len(can_sends))
    self.assertEqual(VSM1, can_sends[0].address)
    self.assertEqual(CanBus.EPS, can_sends[0].src)
    return actuators, vsm1_torque(can_sends[0].dat)

  def test_positive_openpilot_torque_sends_negative_vsm1_torque(self):
    actuators, torque = self._update_with_torque(0.5)

    self.assertLess(torque, 0)
    self.assertGreater(actuators.torque, 0.)
    self.assertEqual(torque, actuators.torqueOutputCan)

  def test_negative_openpilot_torque_sends_positive_vsm1_torque(self):
    actuators, torque = self._update_with_torque(-0.5)

    self.assertGreater(torque, 0)
    self.assertLess(actuators.torque, 0.)
    self.assertEqual(torque, actuators.torqueOutputCan)


if __name__ == "__main__":
  unittest.main()
