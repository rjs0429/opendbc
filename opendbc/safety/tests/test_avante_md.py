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
  FWD_BLACKLISTED_ADDRS = {2: [0x164]}

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
    self._refresh_control_prereqs_on_tx = True
    self._last_control_prereq_time = None
    self._last_driver_torque = 0

  def _tx(self, msg):
    if self._refresh_control_prereqs_on_tx:
      self._set_control_prereqs_normal()
    return super()._tx(msg)

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

  def _vsm2_msg(self, torque=0, torque_eps=0, def_flag=0, serr_flag=0):
    raw_torque = int(torque) + 2048
    raw_torque_eps = int(torque_eps) + 2048
    dat = bytearray(8)
    dat[0] = raw_torque & 0xff
    dat[1] = ((raw_torque >> 8) & 0x0f) | ((raw_torque_eps & 0x0f) << 4)
    dat[2] = (raw_torque_eps >> 4) & 0xff
    dat[3] = (1 if def_flag else 0) | ((1 if serr_flag else 0) << 1)
    return common.make_msg(2, 0x165, 8, bytes(dat))

  def _sas1_msg(self, stat=7):
    dat = bytearray(5)
    dat[3] = stat
    return common.make_msg(2, 0x2b0, 5, bytes(dat))

  def _torque_cmd_msg(self, torque, steer_req=1):
    return self._vsm1_msg(torque, steer_req=steer_req)

  def _torque_driver_msg(self, torque, bus=2):
    if bus != 2:
      raw_torque = int(torque) + 2048
      dat = bytearray(8)
      dat[0] = raw_torque & 0xff
      dat[1] = (raw_torque >> 8) & 0x0f
      return common.make_msg(bus, 0x165, 8, bytes(dat))
    self._last_driver_torque = torque
    return self._vsm2_msg(torque=torque)

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

  def _clu2_eco_msg(self, eco_pressed):
    dat = bytearray(8)
    dat[6] = 0x01 if eco_pressed else 0x00
    return common.make_msg(0, 0x690, 8, bytes(dat))

  def test_prev_gas(self):
    pass

  def test_allow_engage_with_gas_pressed(self):
    pass

  def test_no_disengage_on_gas(self):
    pass

  def test_prev_user_brake(self, _user_brake_msg=None, get_brake_pressed_prev=None):
    pass

  def test_allow_user_brake_at_zero_speed(self, _user_brake_msg=None, get_brake_pressed_prev=None):
    pass

  def test_not_allow_user_brake_when_moving(self, _user_brake_msg=None, get_brake_pressed_prev=None):
    pass

  def test_enable_control_allowed_from_cruise(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self.safety.safety_rx_hook(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_disable_control_allowed_from_cruise(self):
    self.safety.set_controls_allowed(True)
    self.safety.safety_rx_hook(self._pcm_status_msg(False))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_cruise_engaged_prev(self):
    pass

  def test_fwd_hook(self):
    for bus in range(3):
      for addr in self.SCANNED_ADDRS:
        if bus == 0 and addr == 0x164:
          continue
        fwd_bus = self.FWD_BUS_LOOKUP.get(bus, -1)
        if bus in self.FWD_BLACKLISTED_ADDRS and addr in self.FWD_BLACKLISTED_ADDRS[bus]:
          fwd_bus = -1
        self.assertEqual(fwd_bus, self.safety.safety_fwd_hook(bus, addr), f"{addr=:#x} from {bus=} to {fwd_bus=}")

  def _set_vehicle_vsm1_normal(self):
    self._set_control_prereqs_normal()

  def _set_control_prereqs_normal(self):
    self._refresh_control_prereqs_on_tx = True
    self.safety.safety_rx_hook(self._vsm1_vehicle_msg(torque=0, steer_req=0, ctr_mode=0, def_flag=0))
    self.safety.safety_rx_hook(self._vsm2_msg(torque=self._last_driver_torque))
    self.safety.safety_rx_hook(self._sas1_msg())

  def _set_control_prereqs_normal_at(self, t):
    self._refresh_control_prereqs_on_tx = True
    self.safety.set_timer(t)
    self._set_control_prereqs_normal()
    self._last_control_prereq_time = t

  def _refresh_control_prereqs_normal_until(self, t):
    if self._last_control_prereq_time is None:
      self._set_control_prereqs_normal_at(t)
    else:
      refresh_time = self._last_control_prereq_time + 100_000
      while refresh_time < t:
        self._set_control_prereqs_normal_at(refresh_time)
        refresh_time += 100_000
      self._set_control_prereqs_normal_at(t)

  def _set_vehicle_vsm1_non_normal(self, torque=100, steer_req=1, ctr_mode=2, def_flag=0):
    self._refresh_control_prereqs_on_tx = False
    self.safety.safety_rx_hook(self._vsm1_vehicle_msg(torque=torque, steer_req=steer_req, ctr_mode=ctr_mode, def_flag=def_flag))

  def _set_vsm2_non_normal(self, torque=0, torque_eps=0, def_flag=1, serr_flag=0):
    self._refresh_control_prereqs_on_tx = False
    self.safety.safety_rx_hook(self._vsm2_msg(torque=torque, torque_eps=torque_eps, def_flag=def_flag, serr_flag=serr_flag))

  def _set_sas1_invalid(self, stat=0):
    self._refresh_control_prereqs_on_tx = False
    self.safety.safety_rx_hook(self._sas1_msg(stat=stat))

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

  def test_vsm2_driver_torque_only_from_eps_bus(self):
    self._reset_torque_driver_measurement(0)

    self.safety.safety_rx_hook(self._torque_driver_msg(300, bus=0))
    self.assertEqual(0, self.safety.get_torque_driver_min())
    self.assertEqual(0, self.safety.get_torque_driver_max())

    self.safety.safety_rx_hook(self._torque_driver_msg(300, bus=2))
    self.assertGreater(self.safety.get_torque_driver_max(), 0)

  def test_non_normal_blocks_all_openpilot_vsm1(self):
    self.safety.set_controls_allowed(True)
    self._set_vehicle_vsm1_non_normal(torque=100, steer_req=1, ctr_mode=2)
    self.assertFalse(self._tx(self._vsm1_msg(100, steer_req=1, ctr_mode=2)))
    self.assertFalse(self._tx(self._vsm1_msg(0, steer_req=0, ctr_mode=0)))
    self.assertFalse(self._tx(self._vsm1_msg(100, steer_req=1, ctr_mode=2, bad_checksum=True)))

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

  def test_vsm2_fault_disengages_blocks_tx_and_allows_stock_fwd(self):
    self._set_vehicle_vsm1_normal()
    self.safety.set_controls_allowed(True)
    self._set_vsm2_non_normal(def_flag=1)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._tx(self._vsm1_msg(0, steer_req=0, ctr_mode=0)))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_vsm2_torque_invalid_disengages_blocks_tx_and_allows_stock_fwd(self):
    self._set_vehicle_vsm1_normal()
    self.safety.set_controls_allowed(True)
    self._set_vsm2_non_normal(torque=1001, def_flag=0)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._tx(self._vsm1_msg(0, steer_req=0, ctr_mode=0)))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_sas_invalid_disengages_blocks_tx_and_allows_stock_fwd(self):
    self._set_vehicle_vsm1_normal()
    self.safety.set_controls_allowed(True)
    self._set_sas1_invalid()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._tx(self._vsm1_msg(0, steer_req=0, ctr_mode=0)))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

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

  def test_non_normal_to_normal_does_not_reenable_controls(self):
    self.safety.set_controls_allowed(True)
    self._set_vehicle_vsm1_non_normal()
    self.assertFalse(self.safety.get_controls_allowed())
    self._set_vehicle_vsm1_normal()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_vsm2_fault_to_normal_does_not_reenable_controls(self):
    self._set_vehicle_vsm1_normal()
    self.safety.set_controls_allowed(True)
    self._set_vsm2_non_normal()
    self.assertFalse(self.safety.get_controls_allowed())
    self._set_control_prereqs_normal()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_sas_invalid_to_normal_does_not_reenable_controls(self):
    self._set_vehicle_vsm1_normal()
    self.safety.set_controls_allowed(True)
    self._set_sas1_invalid()
    self.assertFalse(self.safety.get_controls_allowed())
    self._set_control_prereqs_normal()
    self.assertFalse(self.safety.get_controls_allowed())

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

  def _eco_press(self, t):
    self.safety.set_timer(t)
    self.safety.safety_rx_hook(self._clu2_eco_msg(True))
    self.safety.safety_rx_hook(self._clu2_eco_msg(False))

  def _eco_press_with_normal_vsm1(self, t):
    self._refresh_control_prereqs_normal_until(t)
    self.safety.safety_rx_hook(self._clu2_eco_msg(True))
    self.safety.safety_rx_hook(self._clu2_eco_msg(False))

  def test_eco_double_press_enables_steering(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self._eco_press_with_normal_vsm1(0)
    self.assertFalse(self.safety.get_controls_allowed())
    self._eco_press_with_normal_vsm1(500_000)  # 0.5초 후 두 번째 누름
    self.assertTrue(self.safety.get_controls_allowed())

  def test_eco_double_press_disables_steering(self):
    self.safety.set_controls_allowed(True)
    self._eco_press_with_normal_vsm1(0)
    self._eco_press_with_normal_vsm1(500_000)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_eco_double_press_during_non_normal_does_not_prearm(self):
    self._set_vehicle_vsm1_non_normal()
    self._eco_press(0)
    self._eco_press(500_000)
    self.safety.set_timer(500_000)
    self._set_vehicle_vsm1_normal()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_eco_first_press_non_normal_second_press_after_normal_does_not_enable(self):
    # 1st press during non-normal must not arm the pending state for a later 2nd press
    self._set_vehicle_vsm1_non_normal()
    self._eco_press(0)
    self.safety.set_timer(500_000)
    self._set_vehicle_vsm1_normal()
    self._eco_press_with_normal_vsm1(500_000)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_eco_normal_press_then_non_normal_cancels_pending(self):
    # 1st press during normal, then VSM1 goes non-normal: pending must be cleared
    self._set_vehicle_vsm1_normal()
    self._eco_press(0)
    self._set_vehicle_vsm1_non_normal()
    self.safety.set_timer(500_000)
    self._set_vehicle_vsm1_normal()
    self._eco_press_with_normal_vsm1(500_000)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_eco_normal_press_then_vsm2_fault_cancels_pending(self):
    self._set_vehicle_vsm1_normal()
    self._eco_press(0)
    self._set_vsm2_non_normal()
    self.safety.set_timer(500_000)
    self._set_control_prereqs_normal()
    self._eco_press_with_normal_vsm1(500_000)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_eco_normal_press_then_sas_invalid_cancels_pending(self):
    self._set_vehicle_vsm1_normal()
    self._eco_press(0)
    self._set_sas1_invalid()
    self.safety.set_timer(500_000)
    self._set_control_prereqs_normal()
    self._eco_press_with_normal_vsm1(500_000)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_eco_single_press_no_toggle(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self._eco_press_with_normal_vsm1(0)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_eco_double_press_too_slow_no_toggle(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self._eco_press_with_normal_vsm1(0)
    self._eco_press_with_normal_vsm1(1_500_000)  # 1.5초 후 → 타임아웃, 새 첫 번째 누름으로 처리
    self.assertFalse(self.safety.get_controls_allowed())

  def test_eco_double_press_at_exactly_1s_boundary(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self._eco_press_with_normal_vsm1(0)
    self._eco_press_with_normal_vsm1(1_000_001)  # 경계 초과
    self.assertFalse(self.safety.get_controls_allowed())

  def test_eco_two_double_presses_toggle_twice(self):
    self._eco_press_with_normal_vsm1(0)
    self._eco_press_with_normal_vsm1(300_000)
    self.assertTrue(self.safety.get_controls_allowed())
    self._eco_press_with_normal_vsm1(600_000)
    self._eco_press_with_normal_vsm1(900_000)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_eco_double_press_before_vsm1_seen_does_not_enable(self):
    self._eco_press(0)
    self._eco_press(500_000)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_eco_double_press_with_stale_vsm1_does_not_enable(self):
    self.safety.set_timer(0)
    self._set_vehicle_vsm1_normal()
    self._eco_press(250_001)
    self._eco_press(500_000)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_eco_pending_cleared_when_vsm1_goes_stale(self):
    self._eco_press_with_normal_vsm1(0)
    self.safety.set_timer(250_001)
    self.safety.safety_rx_hook(self._clu2_eco_msg(False))
    self._eco_press_with_normal_vsm1(500_000)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_stale_vsm1_blocks_tx_and_allows_stock_fwd(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.safety.set_timer(0)
    self._set_vehicle_vsm1_normal()
    self.safety.set_timer(200_001)
    self._refresh_control_prereqs_on_tx = False
    self.assertFalse(self._tx(self._vsm1_msg(0, steer_req=1, ctr_mode=2)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_stale_vsm2_blocks_tx_and_allows_stock_fwd(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.safety.set_timer(0)
    self._set_vehicle_vsm1_normal()
    self.safety.set_timer(100_000)
    self.safety.safety_rx_hook(self._vsm1_vehicle_msg(torque=0, steer_req=0, ctr_mode=0, def_flag=0))
    self.safety.safety_rx_hook(self._sas1_msg())
    self.safety.set_timer(200_001)
    self._refresh_control_prereqs_on_tx = False
    self.assertFalse(self._tx(self._vsm1_msg(0, steer_req=1, ctr_mode=2)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_stale_sas1_blocks_tx_and_allows_stock_fwd(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.safety.set_timer(0)
    self._set_vehicle_vsm1_normal()
    self.safety.set_timer(100_000)
    self.safety.safety_rx_hook(self._vsm1_vehicle_msg(torque=0, steer_req=0, ctr_mode=0, def_flag=0))
    self.safety.safety_rx_hook(self._vsm2_msg())
    self.safety.set_timer(200_001)
    self._refresh_control_prereqs_on_tx = False
    self.assertFalse(self._tx(self._vsm1_msg(0, steer_req=1, ctr_mode=2)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_stale_vsm1_then_normal_does_not_reenable_controls(self):
    self.safety.set_controls_allowed(True)
    self.safety.set_timer(0)
    self._set_vehicle_vsm1_normal()
    self.safety.set_timer(200_001)
    self._set_vehicle_vsm1_normal()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_eco_pending_cleared_by_vsm1_stale_gap_without_clu2(self):
    self.safety.set_timer(0)
    self._set_vehicle_vsm1_normal()
    self._eco_press(0)
    self.safety.set_timer(250_001)
    self._set_vehicle_vsm1_normal()
    self._eco_press_with_normal_vsm1(500_000)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_vsm1_alive_counter_allowed_in_normal_state(self):
    self._set_control_prereqs_normal()
    self.safety.set_controls_allowed(True)
    for alive_cnt in range(16):
      dat = bytearray(8)
      dat[1] = 0x08
      dat[6] = alive_cnt
      dat[7] = self._checksum(dat)
      self.safety.safety_rx_hook(common.make_msg(0, 0x164, 8, bytes(dat)))
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0x164))

  def test_vsm1_data6_upper_nibble_treated_as_non_normal(self):
    self._set_vehicle_vsm1_normal()
    self.safety.set_controls_allowed(True)
    dat = bytearray(8)
    dat[1] = 0x08
    dat[6] = 0x10
    dat[7] = self._checksum(dat)
    self.safety.safety_rx_hook(common.make_msg(0, 0x164, 8, bytes(dat)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_realtime_limits(self):
    for sign in [-1, 1]:
      self.safety.set_safety_hooks(CarParams.SafetyModel.avanteMd, 0)
      self.safety.init_tests()
      self.safety.set_controls_allowed(True)
      self._refresh_control_prereqs_on_tx = True
      self._set_prev_torque(0)
      self._reset_torque_driver_measurement(0)
      for t in range(self.MAX_RT_DELTA):
        self.assertTrue(self._tx(self._torque_cmd_msg(t * sign)))
      self.assertFalse(self._tx(self._torque_cmd_msg(sign * (self.MAX_RT_DELTA + 1))))

      self._set_prev_torque(0)
      for t in range(self.MAX_RT_DELTA):
        self.assertTrue(self._tx(self._torque_cmd_msg(t * sign)))

      # Keep control prerequisites fresh while advancing the realtime torque window.
      self.safety.set_timer(100_000)
      self._set_control_prereqs_normal()
      self.safety.set_timer(common.RT_INTERVAL + 1)
      self.assertTrue(self._tx(self._torque_cmd_msg(sign * (self.MAX_RT_DELTA - 1))))
      self.assertTrue(self._tx(self._torque_cmd_msg(sign * (self.MAX_RT_DELTA + 1))))

  def test_eco_hold_through_non_normal_does_not_create_press(self):
    self.safety.set_timer(0)
    self._set_vehicle_vsm1_normal()
    self.safety.safety_rx_hook(self._clu2_eco_msg(True))
    self._set_vehicle_vsm1_non_normal()
    self.safety.set_timer(100_000)
    self._set_vehicle_vsm1_normal()
    self.safety.safety_rx_hook(self._clu2_eco_msg(True))
    self.safety.safety_rx_hook(self._clu2_eco_msg(False))
    self._eco_press_with_normal_vsm1(500_000)
    self.assertFalse(self.safety.get_controls_allowed())

  def test_brake_does_not_cancel_steering(self):
    self.safety.set_controls_allowed(True)
    dat = bytearray(8)
    dat[3] = 0x04  # brake bit
    brake_msg = common.make_msg(0, 0x440, 8, bytes(dat))
    self.safety.safety_rx_hook(brake_msg)
    self.assertTrue(self.safety.get_controls_allowed())


if __name__ == "__main__":
  unittest.main()
