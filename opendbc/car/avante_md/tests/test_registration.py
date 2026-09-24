#!/usr/bin/env python3
import re
import unittest
from pathlib import Path

from opendbc.car import gen_empty_fingerprint
from opendbc.car.avante_md.interface import CarInterface
from opendbc.car.avante_md.values import CAR
from opendbc.car.interfaces import get_torque_params
from opendbc.car.structs import CarParams
from opendbc.car.tests.routes import non_tested_cars
from opendbc.car.values import PLATFORMS
from opendbc.safety.tests.libsafety import libsafety_py

SAFETY_DIR = Path(__file__).resolve().parents[3] / "safety"
DEFINE = re.compile(r"^#define SAFETY_(\w+) (\d+)U$", re.MULTILINE)


def safety_defines(path: Path) -> dict[str, int]:
  return {name: int(num) for name, num in DEFINE.findall(path.read_text())}


class TestAvanteMdRegistration(unittest.TestCase):
  def test_safety_number_matches_between_capnp_and_c(self):
    ours = safety_defines(SAFETY_DIR / "modes" / "avante_md.h")
    self.assertEqual({"AVANTE_MD": int(CarParams.SafetyModel.avanteMd)}, ours)

  def test_safety_number_is_not_used_upstream(self):
    upstream = safety_defines(SAFETY_DIR / "declarations.h")
    self.assertNotIn("AVANTE_MD", upstream)
    self.assertNotIn(int(CarParams.SafetyModel.avanteMd), upstream.values())

  def test_panda_accepts_the_safety_mode(self):
    safety = libsafety_py.libsafety
    self.assertEqual(0, safety.set_safety_hooks(CarParams.SafetyModel.avanteMd, 0))
    self.assertEqual(int(CarParams.SafetyModel.avanteMd), safety.get_current_safety_mode())

  def test_port_requests_its_safety_mode(self):
    CP = CarInterface.get_params(CAR.AVANTE_MD_2012, gen_empty_fingerprint(), [], False, False, False)
    self.assertEqual(CarParams.SafetyModel.avanteMd, CP.safetyConfigs[0].safetyModel)

  def test_platform_is_registered(self):
    self.assertTrue(str(CAR.AVANTE_MD_2012) in PLATFORMS)
    self.assertTrue(CAR.AVANTE_MD_2012 in non_tested_cars)
    self.assertIsNotNone(get_torque_params().get(str(CAR.AVANTE_MD_2012)))


if __name__ == "__main__":
  unittest.main()
