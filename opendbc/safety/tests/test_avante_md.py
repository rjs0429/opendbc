#!/usr/bin/env python3
import unittest

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common


class TestAvanteMdSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  TX_MSGS = [[0x164, 2]]
  STANDSTILL_THRESHOLD = 0.1
  RELAY_MALFUNCTION_ADDRS = {2: (0x164,)}
  FWD_BLACKLISTED_ADDRS = {0: [0x164], 2: [0x164]}

  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 25
  MAX_TORQUE_LOOKUP = [0], [800]
  MAX_RT_DELTA = 300

  DRIVER_TORQUE_ALLOWANCE = 150
  DRIVER_TORQUE_FACTOR = 1

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.avanteMd, 0)
    self.safety.init_tests()

  @staticmethod
  def _checksum(dat):
    checksum = 0
    for b in dat[:7]:
      checksum ^= b
    return checksum

  def _vsm1_msg(self, torque, steer_req=1, bus=2, ctr_mode=None, def_flag=0, bad_checksum=False):
    raw_torque = int(torque) + 2048
    if ctr_mode is None:
      ctr_mode = 2 if steer_req else 0

    dat = bytearray(8)
    dat[0] = raw_torque & 0xff
    dat[1] = ((raw_torque >> 8) & 0x0f) | ((1 if steer_req else 0) << 4) | (ctr_mode << 5)
    dat[2] = 1 if def_flag else 0
    dat[6] = 7
    dat[7] = self._checksum(dat)
    if bad_checksum:
      dat[7] ^= 0xff
    return common.make_msg(bus, 0x164, 8, bytes(dat))

  def _vsm1_vehicle_msg(self, torque=0, steer_req=0, ctr_mode=0, def_flag=0):
    return self._vsm1_msg(torque, steer_req=steer_req, bus=0, ctr_mode=ctr_mode, def_flag=def_flag)

  def _torque_cmd_msg(self, torque, steer_req=1):
    return self._vsm1_msg(torque, steer_req=steer_req)

  def _torque_driver_msg(self, torque):
    raw_torque = int(torque) + 2048
    dat = bytearray(8)
    dat[0] = raw_torque & 0xff
    dat[1] = (raw_torque >> 8) & 0x0f
    return common.make_msg(0, 0x165, 8, bytes(dat))

  def _speed_msg(self, speed):
    raw_speed = int((speed / CV.KPH_TO_MS) / 0.125)
    dat = bytearray(8)
    dat[2] = raw_speed & 0xff
    dat[3] = ((raw_speed >> 8) & 0x0f) | ((raw_speed & 0x0f) << 4)
    dat[4] = (raw_speed >> 4) & 0xff
    dat[5] = raw_speed & 0xff
    dat[6] = ((raw_speed >> 8) & 0x0f) | ((raw_speed & 0x0f) << 4)
    dat[7] = (raw_speed >> 4) & 0xff
    return common.make_msg(0, 0x1f1, 8, bytes(dat))

  def _speed_msg_2(self, speed):
    return None

  def _user_brake_msg(self, brake):
    dat = bytearray(8)
    dat[3] = (1 if brake else 0) << 2
    return common.make_msg(0, 0x440, 8, bytes(dat))

  def _user_gas_msg(self, gas):
    dat = bytearray(8)
    dat[7] = (1 if gas else 0) << 6
    return common.make_msg(0, 0x260, 8, bytes(dat))

  def _pcm_status_msg(self, enable):
    dat = bytearray(8)
    dat[3] = 1 if enable else 0
    return common.make_msg(0, 0x4f0, 8, bytes(dat))

  def _set_vehicle_vsm1_normal(self):
    self.safety.safety_rx_hook(self._vsm1_vehicle_msg(torque=0, steer_req=0, ctr_mode=0, def_flag=0))

  def _set_vehicle_vsm1_non_normal(self, torque=100, steer_req=1, ctr_mode=2, def_flag=0):
    self.safety.safety_rx_hook(self._vsm1_vehicle_msg(torque=torque, steer_req=steer_req, ctr_mode=ctr_mode, def_flag=def_flag))

  def test_vsm1_checksum(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._vsm1_msg(0)))
    self.assertFalse(self._tx(self._vsm1_msg(0, bad_checksum=True)))

  def test_vsm1_control_mode(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._vsm1_msg(0, steer_req=1, ctr_mode=2)))
    self.assertFalse(self._tx(self._vsm1_msg(0, steer_req=1, ctr_mode=1)))
    self.assertTrue(self._tx(self._vsm1_msg(0, steer_req=0, ctr_mode=0)))
    self.assertFalse(self._tx(self._vsm1_msg(0, steer_req=0, ctr_mode=2)))

  def test_vsm1_def_flag(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.assertFalse(self._tx(self._vsm1_msg(0, def_flag=1)))

  def test_wrong_bus_blocked(self):
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._vsm1_msg(0, bus=0)))

  def test_non_normal_blocks_all_openpilot_vsm1(self):
    self.safety.set_controls_allowed(True)
    self._set_vehicle_vsm1_non_normal(torque=100, steer_req=1, ctr_mode=2)
    self.assertFalse(self._tx(self._vsm1_msg(100, steer_req=1, ctr_mode=2)))
    self.assertFalse(self._tx(self._vsm1_msg(0, steer_req=0, ctr_mode=0)))
    self.assertFalse(self._tx(self._vsm1_msg(100, steer_req=1, ctr_mode=2, bad_checksum=True)))

  def test_non_normal_blocks_openpilot_vsm1_regardless_of_controls(self):
    self.safety.set_controls_allowed(False)
    self._set_vehicle_vsm1_non_normal(torque=100, steer_req=1, ctr_mode=2)
    self.assertFalse(self._tx(self._vsm1_msg(100, steer_req=1, ctr_mode=2)))

  def test_normal_mode_rate_limit_still_applies(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self._set_vehicle_vsm1_normal()
    self.assertFalse(self._tx(self._vsm1_msg(500, steer_req=1, ctr_mode=2)))

  def test_def_flag_blocked_in_normal_mode(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self._set_vehicle_vsm1_normal()
    self.assertFalse(self._tx(self._vsm1_msg(0, def_flag=1)))

  def test_normal_mode_blocked_when_controls_not_allowed(self):
    self.safety.set_controls_allowed(False)
    self._set_prev_torque(0)
    self._set_vehicle_vsm1_normal()
    self.assertFalse(self._tx(self._vsm1_msg(100, steer_req=1, ctr_mode=2)))

  def test_vsm1_fwd_blocked_in_normal_state(self):
    self._set_vehicle_vsm1_normal()
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0x164))

  def test_vsm1_fwd_allowed_in_non_normal_state(self):
    self._set_vehicle_vsm1_non_normal(torque=100, steer_req=1, ctr_mode=2)
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_vsm1_fwd_bus2_always_blocked(self):
    self._set_vehicle_vsm1_normal()
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x164))
    self._set_vehicle_vsm1_non_normal(torque=100, steer_req=1, ctr_mode=2)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x164))

  def test_vsm1_fwd_before_rx_one_frame_delay(self):
    self._set_vehicle_vsm1_normal()
    non_normal_msg = self._vsm1_vehicle_msg(torque=100, steer_req=1, ctr_mode=2)
    fwd_before_rx = self.safety.safety_fwd_hook(0, 0x164)
    self.safety.safety_rx_hook(non_normal_msg)
    self.assertEqual(-1, fwd_before_rx)
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_non_normal_disengages_controls(self):
    self.safety.set_controls_allowed(True)
    self._set_vehicle_vsm1_non_normal(torque=100, steer_req=1, ctr_mode=2)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_normal_to_normal_does_not_disengage_controls(self):
    self._set_vehicle_vsm1_normal()
    self.safety.set_controls_allowed(True)
    self._set_vehicle_vsm1_normal()
    self.assertTrue(self.safety.get_controls_allowed())

  def test_corrupt_vsm1_treated_as_non_normal(self):
    self._set_vehicle_vsm1_normal()
    self.safety.set_controls_allowed(True)
    dat = bytearray(8)
    dat[1] = 0x08   # normal-state byte
    dat[7] = self._checksum(dat) ^ 0xff  # corrupt checksum
    corrupt_msg = common.make_msg(0, 0x164, 8, bytes(dat))
    self.safety.safety_rx_hook(corrupt_msg)
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))
    self.assertFalse(self.safety.get_controls_allowed())

if __name__ == "__main__":
  unittest.main()
