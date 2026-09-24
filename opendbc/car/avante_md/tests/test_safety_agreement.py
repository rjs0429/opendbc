#!/usr/bin/env python3
"""The car port (Python) and panda safety (C) decode the same vehicle signals separately; they must agree."""
import unittest

from opendbc.car.avante_md.tests import test_carstate as port
from opendbc.safety.tests import common
from opendbc.safety.tests import test_avante_md as panda

STABILIZE_US = 100_000
CLU2 = 0x690


def port_allows(tcu1=5, tcu2=3, clu1=None, clu2=None, eps=None) -> bool:
  case = port.TestAvanteMdCarState("setUp")
  case.setUp()
  case.vehicle["TCU1"]["CUR_GR"] = tcu1
  case.vehicle["TCU2"]["CUR_GR"] = tcu2
  for k, v in (clu1 or {}).items():
    case.vehicle["CLU1"][k] = v
  for k, v in (clu2 or {}).items():
    case.vehicle["CLU2"][k] = v
  for (msg, sig), v in (eps or {}).items():
    case.eps[msg][sig] = v
  for _ in range(3):
    case._update()
  return case.CS.should_be_active


def panda_allows(tcu1=5, tcu2=3, parking_brake=False, drv_door=0, ast_door=0, belt=0, mdps_def=0, mdps_serr=0,
                 sas_stat=7) -> bool:
  case = panda.TestAvanteMdSafety("setUp")
  case.setUp()

  def clu2():
    cnt = case._clu2_cnt
    case._clu2_cnt = (cnt + 1) % 16
    dat = bytearray(8)
    dat[0] = (drv_door & 0x3) << 6
    dat[2] = belt & 0x3
    dat[6] = ((cnt & 0xF) << 2) | ((ast_door & 0x3) << 6)
    return common.make_msg(0, CLU2, 8, bytes(dat))

  for t in (0, STABILIZE_US, 2 * STABILIZE_US):
    case.safety.set_timer(t)
    for msg in (case._vsm1_vehicle_msg(), case._tcs1_msg(), case._tcs5_msg(), case._esp2_msg(), case._whl_pul_msg(),
                case._clu1_msg(parking_brake=parking_brake), clu2(), case._tcu1_msg(gear_disp=tcu1),
                case._tcu2_msg(gear=tcu2), case._vsm2_msg(def_flag=mdps_def, serr_flag=mdps_serr),
                case._sas1_msg(stat=sas_stat), case._mdps1_msg()):
      case.safety.safety_rx_hook(msg)
  return bool(case.safety.get_controls_allowed())


class TestAvanteMdSafetyAgreement(unittest.TestCase):
  def test_every_gear_combination(self):
    for tcu1 in range(16):
      for tcu2 in range(16):
        with self.subTest(tcu1=tcu1, tcu2=tcu2):
          self.assertEqual(port_allows(tcu1, tcu2), panda_allows(tcu1, tcu2))

  def test_drive_and_sport_are_allowed(self):
    for tcu1 in (5, 8):
      with self.subTest(tcu1=tcu1):
        self.assertTrue(port_allows(tcu1))
        self.assertTrue(panda_allows(tcu1))

  def test_parking_brake(self):
    for on in (False, True):
      with self.subTest(on=on):
        self.assertEqual(port_allows(clu1={"CF_Clu_ParkBrakeSw": int(on)}), panda_allows(parking_brake=on))

  def test_doors_and_seatbelt(self):
    for drv in range(4):
      for ast in range(4):
        for belt in range(4):
          with self.subTest(drv=drv, ast=ast, belt=belt):
            clu2 = {"CF_Clu_DrvDrSw": drv, "CF_Clu_AstDrSw": ast, "CF_Clu_DrvSeatBeltSw": belt}
            self.assertEqual(port_allows(clu2=clu2), panda_allows(drv_door=drv, ast_door=ast, belt=belt))

  def test_steering_faults(self):
    for mdps_def in (0, 1):
      for mdps_serr in (0, 1):
        with self.subTest(mdps_def=mdps_def, mdps_serr=mdps_serr):
          eps = {("VSM2", "CF_Mdps_Def"): mdps_def, ("VSM2", "CF_Mdps_SErr"): mdps_serr}
          self.assertEqual(port_allows(eps=eps), panda_allows(mdps_def=mdps_def, mdps_serr=mdps_serr))

  def test_steering_angle_sensor_state(self):
    for stat in range(256):
      with self.subTest(stat=stat):
        self.assertEqual(port_allows(eps={("SAS1", "SAS_Stat"): stat}), panda_allows(sas_stat=stat))


if __name__ == "__main__":
  unittest.main()
