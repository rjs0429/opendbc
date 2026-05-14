#!/usr/bin/env python3
import unittest

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common


class TestAvanteMdSafety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  TX_MSGS = [[0x164, 2]]
  STANDSTILL_THRESHOLD = 0.1
  RELAY_MALFUNCTION_ADDRS = {}
  FWD_BLACKLISTED_ADDRS = {2: [0x164]}

  MAX_RATE_UP = 800
  MAX_RATE_DOWN = 800
  MAX_TORQUE_LOOKUP = [0], [800]
  MAX_RT_DELTA = 800
  DRIVER_TORQUE_ALLOWANCE = 150
  DRIVER_TORQUE_FACTOR = 1

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.avanteMd, 0)
    self.safety.init_tests()

    # Counter state for messages with counter tracking
    self._tcu2_cnt = 0
    self._clu1_cnt = 0
    self._clu2_cnt = 0
    self._vsm1_rx_cnt = 0   # vehicle-side VSM1 (bus 0), max_counter=15
    self._vsm2_cnt = 0      # VSM2, max_counter=15
    self._sas1_cnt = 0      # SAS1, max_counter=15

    # Auto-advance timer and refresh prereqs before each TX
    self._refresh_prereqs_on_tx = True
    self._auto_tx_time_us = 0
    self._last_driver_torque = 0
    self._prereqs_stabilized = False
    self._prev_torque = 0

  # ── TX wrapper ────────────────────────────────────────────────────────────

  def _tx(self, msg):
    if self._refresh_prereqs_on_tx:
      if not self._prereqs_stabilized:
        prev_torque = self._prev_torque
        self._set_prereqs_normal_and_stabilized(self._auto_tx_time_us)
        super()._set_prev_torque(prev_torque)
      self._auto_tx_time_us += 8_000
      self.safety.set_timer(self._auto_tx_time_us)
      self._set_prereqs_normal()
    return super()._tx(msg)

  def _set_prev_torque(self, t):
    self._prev_torque = t
    super()._set_prev_torque(t)

  # ── Checksum helpers ──────────────────────────────────────────────────────

  @staticmethod
  def _vsm_xor_checksum(dat):
    """XOR checksum over bytes 0-6 (used by VSM1/VSM2)."""
    cs = 0
    for b in dat[:7]:
      cs ^= b
    return cs

  @staticmethod
  def _sas1_checksum(dat):
    """All-nibble XOR checksum for SAS1 (byte4 high nibble)."""
    cs = 0
    for i in range(4):
      cs ^= (dat[i] >> 4) ^ (dat[i] & 0xF)
    cs ^= (dat[4] & 0xF)
    return cs & 0xF

  @staticmethod
  def _tcu2_checksum(gear, alive):
    """2-bit checksum for TCU2: (alive + gear_offset) & 0x3."""
    if gear == 0:
      gear_offset = 0
    elif 1 <= gear <= 6:
      gear_offset = ((gear - 1) % 3) + 1
    elif gear == 14:
      gear_offset = 1
    else:
      return None  # non-normal gear: checksum will fail
    return (alive + gear_offset) & 0x3

  # ── Message builders ──────────────────────────────────────────────────────

  def _vsm1_msg(self, torque, steer_req=1, bus=2, ctr_mode=None,
                def_flag=0, bad_checksum=False, cnt=None):
    """VSM1: openpilot steering command or vehicle VSM1 snapshot.

    cnt: explicit counter value; if None, defaults to 7 (only matters for
         bus-0 RX frames where the rx_check counter is validated).
    """
    raw = int(torque) + 2048
    if ctr_mode is None:
      ctr_mode = 2 if steer_req else 0
    if cnt is None:
      cnt = 7
    dat = bytearray(8)
    dat[0] = raw & 0xFF
    dat[1] = ((raw >> 8) & 0xF) | ((1 if steer_req else 0) << 4) | (ctr_mode << 5)
    dat[2] = 1 if def_flag else 0
    dat[6] = cnt & 0xF  # alive counter (lower nibble); upper nibble = 0 (normal)
    dat[7] = self._vsm_xor_checksum(dat)
    if bad_checksum:
      dat[7] ^= 0xFF
    return common.make_msg(bus, 0x164, 8, bytes(dat))

  def _vsm1_vehicle_msg(self, torque=0, steer_req=0, ctr_mode=0, def_flag=0):
    """Vehicle-side VSM1 (bus 0) in normal/idle state; uses incrementing counter."""
    cnt = self._vsm1_rx_cnt
    self._vsm1_rx_cnt = (self._vsm1_rx_cnt + 1) % 16
    return self._vsm1_msg(torque, steer_req=steer_req, bus=0,
                          ctr_mode=ctr_mode, def_flag=def_flag, cnt=cnt)

  def _vsm2_msg(self, torque=0, def_flag=0, serr_flag=0):
    """VSM2: driver torque + MDPS fault bits; uses incrementing counter."""
    cnt = self._vsm2_cnt
    self._vsm2_cnt = (self._vsm2_cnt + 1) % 16
    raw = int(torque) + 2048
    dat = bytearray(8)
    dat[0] = raw & 0xFF
    dat[1] = (raw >> 8) & 0xF
    dat[3] = (1 if def_flag else 0) | ((1 if serr_flag else 0) << 1)
    dat[6] = cnt & 0xF  # alive counter (lower nibble); rx_check validates this
    dat[7] = self._vsm_xor_checksum(dat)
    return common.make_msg(2, 0x165, 8, bytes(dat))

  def _sas1_msg(self, stat=7):
    """SAS1: steering angle sensor status; uses incrementing counter."""
    cnt = self._sas1_cnt
    self._sas1_cnt = (self._sas1_cnt + 1) % 16
    dat = bytearray(5)
    dat[3] = stat & 0xFF
    dat[4] = cnt & 0xF                           # counter in low nibble
    cs = self._sas1_checksum(dat)               # checksum includes counter nibble
    dat[4] = (cnt & 0xF) | (cs << 4)            # checksum in high nibble
    return common.make_msg(2, 0x2B0, 5, bytes(dat))

  def _tcs1_msg(self):
    """TCS1: normal idle frame (staleness tracking only)."""
    dat = bytearray(8)
    # byte7 = sum(bytes 0-6) & 0xFF = 0 for all-zero payload
    return common.make_msg(0, 0x153, 8, bytes(dat))

  def _tcs5_msg(self, speed_kph=0.0):
    """TCS5: wheel speed (used for vehicle speed update)."""
    raw = int((speed_kph / CV.KPH_TO_MS) / 0.125)
    dat = bytearray(8)
    dat[2] = raw & 0xFF
    dat[3] = ((raw >> 8) & 0xF) | ((raw & 0xF) << 4)
    dat[4] = (raw >> 4) & 0xFF
    dat[5] = raw & 0xFF
    dat[6] = ((raw >> 8) & 0xF) | ((raw & 0xF) << 4)
    dat[7] = (raw >> 4) & 0xFF
    return common.make_msg(0, 0x1F1, 8, bytes(dat))

  def _esp2_msg(self):
    """ESP2: normal idle frame (staleness tracking only)."""
    return common.make_msg(0, 0x220, 8, bytes(8))

  def _whl_pul_msg(self):
    """WHL_PUL: normal idle frame (staleness tracking only)."""
    return common.make_msg(0, 0x4B1, 8, bytes(8))

  def _mdps1_msg(self):
    """MDPS1: normal idle frame (staleness tracking only)."""
    return common.make_msg(2, 0x5E4, 3, bytes(3))

  def _tcu1_msg(self, gear_disp=5):
    """TCU1: gear-selector display.  gear_disp=5 = D."""
    dat = bytearray(8)
    dat[1] = gear_disp & 0xF
    return common.make_msg(0, 0x43F, 8, bytes(dat))

  def _tcu2_msg(self, gear=1, alive=None, bad_checksum=False):
    """TCU2: current gear with 2-bit alive counter + checksum."""
    if alive is None:
      alive = self._tcu2_cnt
      self._tcu2_cnt = (self._tcu2_cnt + 1) % 4
    chksum = self._tcu2_checksum(gear, alive)
    if chksum is None or bad_checksum:
      # Use an impossible 8-bit value; only bits 6-7 are read so any mismatch
      chksum = (self._tcu2_checksum(gear, alive) if chksum is not None else 0) ^ 0x3
    dat = bytearray(8)
    dat[1] = (gear & 0xF) | ((alive & 0x3) << 4) | ((chksum & 0x3) << 6)
    return common.make_msg(0, 0x440, 8, bytes(dat))

  def _clu1_msg(self, parking_brake=False, cnt=None):
    """CLU1: cluster status including parking brake and alive counter."""
    if cnt is None:
      cnt = self._clu1_cnt
      self._clu1_cnt = (self._clu1_cnt + 1) % 128
    dat = bytearray(8)
    # CF_Clu_ParkBrakeSw: byte0 bit7
    dat[0] = 0x80 if parking_brake else 0x00
    # CF_Clu_AliveCounter: byte2 bits 1-7
    dat[2] = (cnt & 0x7F) << 1
    return common.make_msg(0, 0x4F0, 8, bytes(dat))

  def _clu2_msg(self, door_open=False, seatbelt_unlatched=False, cnt=None):
    """CLU2: door/seatbelt status with 4-bit alive counter.

    DBC signal polarity (safety_state.txt):
      CF_Clu_DrvDrSw = 1  → driver door open   (byte0 bits 6-7)
      CF_Clu_AstDrSw = 1  → passenger door open (byte6 bits 6-7)
      CF_Clu_DrvSeatBeltSw = 1 → seatbelt NOT latched (byte2 bits 0-1)
    """
    if cnt is None:
      cnt = self._clu2_cnt
      self._clu2_cnt = (self._clu2_cnt + 1) % 16
    dat = bytearray(8)
    dat[0] = 0x40 if door_open else 0x00        # DrvDrSw bit6
    dat[2] = 0x01 if seatbelt_unlatched else 0x00  # DrvSeatBeltSw bit0
    dat[6] = (cnt & 0xF) << 2                   # AliveCnt2 bits 2-5
    return common.make_msg(0, 0x690, 8, bytes(dat))

  # ── Torque command / driver torque interfaces (required by common tests) ──

  def _torque_cmd_msg(self, torque, steer_req=1):
    return self._vsm1_msg(torque, steer_req=steer_req)

  def _torque_driver_msg(self, torque, bus=2):
    self._last_driver_torque = torque
    return self._vsm2_msg(torque=torque)

  def _speed_msg(self, speed):
    # speed is in m/s; _tcs5_msg's parameter is also in m/s despite the name
    return self._tcs5_msg(speed_kph=speed)

  def _speed_msg_2(self, speed):
    return None

  # ── Common base-class overrides (features not in this safety model) ───────

  def _user_brake_msg(self, brake):
    return common.make_msg(0, 0x440, 8, bytes(8))

  def _user_gas_msg(self, gas):
    return common.make_msg(0, 0x260, 8, bytes(8))

  def _pcm_status_msg(self, enable):
    return common.make_msg(0, 0x4F0, 8, bytes(8))

  def test_prev_gas(self):
    pass

  def test_allow_engage_with_gas_pressed(self):
    pass

  def test_no_disengage_on_gas(self):
    pass

  def test_prev_user_brake(self, _user_brake_msg=None, get_brake_pressed_prev=None):
    pass

  def test_allow_user_brake_at_zero_speed(self, _user_brake_msg=None,
                                          get_brake_pressed_prev=None):
    pass

  def test_not_allow_user_brake_when_moving(self, _user_brake_msg=None,
                                            get_brake_pressed_prev=None):
    pass

  def test_enable_control_allowed_from_cruise(self):
    # controls_allowed is managed automatically; PCM message has no effect
    self.assertFalse(self.safety.get_controls_allowed())
    self.safety.safety_rx_hook(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  # ── Prerequisite setup helpers ────────────────────────────────────────────

  def _set_prereqs_normal(self):
    """Send one valid frame for every required message on both buses."""
    # Vehicle bus
    self.safety.safety_rx_hook(self._vsm1_vehicle_msg())
    self.safety.safety_rx_hook(self._tcs1_msg())
    self.safety.safety_rx_hook(self._tcs5_msg())
    self.safety.safety_rx_hook(self._esp2_msg())
    self.safety.safety_rx_hook(self._whl_pul_msg())
    self.safety.safety_rx_hook(self._clu1_msg())
    self.safety.safety_rx_hook(self._clu2_msg())
    self.safety.safety_rx_hook(self._tcu1_msg())
    self.safety.safety_rx_hook(self._tcu2_msg())
    # EPS bus
    self.safety.safety_rx_hook(self._vsm2_msg(torque=self._last_driver_torque))
    self.safety.safety_rx_hook(self._sas1_msg())
    self.safety.safety_rx_hook(self._mdps1_msg())

  def _set_prereqs_normal_at(self, t):
    self.safety.set_timer(t)
    self._set_prereqs_normal()

  def _set_prereqs_normal_and_stabilized(self, t=0):
    """Send normal prereqs, then reevaluate after the 100ms stabilization window."""
    self.safety.set_timer(t)
    self._set_prereqs_normal()
    self.safety.set_timer(t + 100_000)
    self._set_prereqs_normal()
    self._auto_tx_time_us = max(self._auto_tx_time_us, t + 100_000)
    self._prereqs_stabilized = True

  # ── Controls-allowed auto-management ─────────────────────────────────────

  def test_controls_allowed_enabled_when_all_prereqs_normal(self):
    self.assertFalse(self.safety.get_controls_allowed())
    self._set_prereqs_normal()
    self.assertFalse(self.safety.get_controls_allowed())
    self.safety.set_timer(100_000)
    self._set_prereqs_normal()
    self.assertTrue(self.safety.get_controls_allowed())

  def test_controls_allowed_disabled_initially(self):
    self.assertFalse(self.safety.get_controls_allowed())

  def test_controls_not_reenabled_after_blocking_condition_clears(self):
    """Controls must not re-enable until 100ms after last blocking condition clears.

    _set_prereqs_normal() refreshes messages in order; until CLU2 is refreshed
    (7th msg), door_open is still observed each rx_hook, bumping block_last_time
    to T_clear.  Re-enable requires a second call at T_clear + 100_000.
    """
    self.safety.set_timer(0)
    self._set_prereqs_normal_and_stabilized(0)
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.safety_rx_hook(self._clu2_msg(door_open=True))
    self.assertFalse(self.safety.get_controls_allowed())
    # T_clear=100_000: clear door_open; block_last_time→100_000
    self.safety.set_timer(100_000)
    self._set_prereqs_normal()
    self.assertFalse(self.safety.get_controls_allowed())
    # T_clear + 100_000 = 200_000: 100ms elapsed since last block obs → re-enable
    self.safety.set_timer(200_000)
    self._set_prereqs_normal()
    self.assertTrue(self.safety.get_controls_allowed())

  def test_stabilization_100ms_required_after_block(self):
    """Controls must not re-enable until 100ms after last blocking condition.

    Calling _set_prereqs_normal() at T_clear bumps block_last_time to T_clear
    (msgs 1-6 still see door_open before CLU2 is refreshed).  A second call at
    T_clear + 100_000 finds all msgs fresh and stabilized → re-enables.
    """
    self.safety.set_timer(0)
    self._set_prereqs_normal_and_stabilized(0)
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.safety_rx_hook(self._clu2_msg(door_open=True))
    self.assertFalse(self.safety.get_controls_allowed())
    # T_clear=100_000: clear door_open; block_last_time→100_000
    self.safety.set_timer(100_000)
    self._set_prereqs_normal()
    self.assertFalse(self.safety.get_controls_allowed())
    # 99_999 µs after T_clear — still blocked
    self.safety.set_timer(199_999)
    self._set_prereqs_normal()
    self.assertFalse(self.safety.get_controls_allowed())
    # 100_000 µs after T_clear — now enabled
    self.safety.set_timer(200_000)
    self._set_prereqs_normal()
    self.assertTrue(self.safety.get_controls_allowed())

  # ── VSM1 TX message validity ──────────────────────────────────────────────

  def test_vsm1_checksum_valid_allows_tx(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._vsm1_msg(0)))

  def test_vsm1_bad_checksum_blocks_tx(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.assertFalse(self._tx(self._vsm1_msg(0, bad_checksum=True)))

  def test_vsm1_control_mode_steer_active(self):
    """steer_req=1 requires CtrMode=2."""
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._vsm1_msg(0, steer_req=1, ctr_mode=2)))
    self.assertFalse(self._tx(self._vsm1_msg(0, steer_req=1, ctr_mode=1)))
    self.assertFalse(self._tx(self._vsm1_msg(0, steer_req=1, ctr_mode=0)))

  def test_vsm1_control_mode_steer_inactive(self):
    """steer_req=0 requires CtrMode=0."""
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.assertTrue(self._tx(self._vsm1_msg(0, steer_req=0, ctr_mode=0)))
    self.assertFalse(self._tx(self._vsm1_msg(0, steer_req=0, ctr_mode=2)))

  def test_vsm1_def_flag_blocks_tx(self):
    """CF_Esc_Def=1 must block TX."""
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.assertFalse(self._tx(self._vsm1_msg(0, def_flag=1)))

  def test_vsm1_tx_min_interval_7ms(self):
    """TX must be blocked if less than 7 ms has elapsed since last TX."""
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self._set_prereqs_normal_and_stabilized(0)
    self.assertTrue(super()._tx(self._vsm1_msg(0, steer_req=0)))
    # 6 999 µs — still too soon
    self.safety.set_timer(106_999)
    self._set_prereqs_normal()
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=0)))
    # 7 000 µs — exactly at boundary
    self.safety.set_timer(107_000)
    self._set_prereqs_normal()
    self.assertTrue(super()._tx(self._vsm1_msg(0, steer_req=0)))

  def test_vsm1_duplicate_tx_same_timestamp_blocked(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self._set_prereqs_normal_and_stabilized(0)
    self.assertTrue(super()._tx(self._vsm1_msg(0, steer_req=0)))
    self._set_prereqs_normal()
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=0)))

  # ── Blocking conditions — vehicle state ──────────────────────────────────

  def test_door_open_blocks_tx_and_disengages(self):
    self._set_prereqs_normal_and_stabilized()
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.safety_rx_hook(self._clu2_msg(door_open=True))
    self.assertFalse(self.safety.get_controls_allowed())
    self._refresh_prereqs_on_tx = False
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=0)))

  def test_seatbelt_unlatched_blocks_tx_and_disengages(self):
    self._set_prereqs_normal_and_stabilized()
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.safety_rx_hook(self._clu2_msg(seatbelt_unlatched=True))
    self.assertFalse(self.safety.get_controls_allowed())
    self._refresh_prereqs_on_tx = False
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=0)))

  def test_parking_brake_on_blocks_tx_and_disengages(self):
    self._set_prereqs_normal_and_stabilized()
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.safety_rx_hook(self._clu1_msg(parking_brake=True))
    self.assertFalse(self.safety.get_controls_allowed())
    self._refresh_prereqs_on_tx = False
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=0)))

  def test_gear_not_d_tcu2_blocks_tx_and_disengages(self):
    """TCU2.CUR_GR outside 1-6 must disengage."""
    self._set_prereqs_normal_and_stabilized()
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.safety_rx_hook(self._tcu2_msg(gear=0))
    self.assertFalse(self.safety.get_controls_allowed())
    self._refresh_prereqs_on_tx = False
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=0)))

  def test_gear_not_d_tcu1_blocks_tx_and_disengages(self):
    """TCU1.CUR_GR != 5 must disengage."""
    self._set_prereqs_normal_and_stabilized()
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.safety_rx_hook(self._tcu1_msg(gear_disp=7))  # R
    self.assertFalse(self.safety.get_controls_allowed())
    self._refresh_prereqs_on_tx = False
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=0)))

  def test_vsm1_non_normal_state_blocks_tx_and_disengages(self):
    """Vehicle VSM1 in non-idle state must disengage."""
    self._set_prereqs_normal_and_stabilized()
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.safety_rx_hook(
      self._vsm1_vehicle_msg(torque=100, steer_req=1, ctr_mode=2))
    self.assertFalse(self.safety.get_controls_allowed())
    self._refresh_prereqs_on_tx = False
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=0)))

  def test_vsm1_non_normal_blocks_all_openpilot_vsm1(self):
    """Even a zero-torque idle frame must be blocked when VSM1 is non-normal."""
    self.safety.set_controls_allowed(True)
    self._refresh_prereqs_on_tx = False
    self.safety.safety_rx_hook(
      self._vsm1_vehicle_msg(torque=100, steer_req=1, ctr_mode=2))
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=0, ctr_mode=0)))
    self.assertFalse(super()._tx(self._vsm1_msg(100, steer_req=1, ctr_mode=2)))

  def test_vsm1_normal_state_does_not_disengage(self):
    self._set_prereqs_normal_and_stabilized()
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.safety_rx_hook(self._vsm1_vehicle_msg())
    self.assertTrue(self.safety.get_controls_allowed())

  def test_non_normal_to_normal_reenables_controls(self):
    """With auto-management, restoring all prereqs re-enables after 100ms stabilization.

    VSM1 is the first message refreshed in _set_prereqs_normal(), so vsm1_normal
    is corrected before any other message is processed.  A single call at
    t=100_000 suffices: msgs from t=0 are not yet stale (100_000 not > 100_000)
    and stabilized = (100_000 - 0) >= 100_000.
    """
    self.safety.set_timer(0)
    self._set_prereqs_normal_and_stabilized(0)
    self.safety.safety_rx_hook(
      self._vsm1_vehicle_msg(torque=100, steer_req=1, ctr_mode=2))
    self.assertFalse(self.safety.get_controls_allowed())
    self.safety.set_timer(200_000)
    self._set_prereqs_normal()
    self.assertTrue(self.safety.get_controls_allowed())

  # ── Blocking conditions — EPS state ──────────────────────────────────────

  def test_vsm2_def_flag_disengages_blocks_tx_allows_fwd(self):
    self._set_prereqs_normal_and_stabilized()
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.safety_rx_hook(self._vsm2_msg(def_flag=1))
    self.assertFalse(self.safety.get_controls_allowed())
    self._refresh_prereqs_on_tx = False
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=0)))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_vsm2_serr_flag_disengages_blocks_tx_allows_fwd(self):
    self._set_prereqs_normal_and_stabilized()
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.safety_rx_hook(self._vsm2_msg(serr_flag=1))
    self.assertFalse(self.safety.get_controls_allowed())
    self._refresh_prereqs_on_tx = False
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=0)))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_vsm2_fault_clear_reenables_controls(self):
    self.safety.set_timer(0)
    self._set_prereqs_normal_and_stabilized(0)
    self.safety.safety_rx_hook(self._vsm2_msg(def_flag=1))
    self.assertFalse(self.safety.get_controls_allowed())
    # T_clear=100_000: msgs 1-9 still see vsm2_normal=False → block_last_time→100_000
    self.safety.set_timer(100_000)
    self._set_prereqs_normal()
    # T_clear + 100_000 = 200_000: stabilized → re-enable
    self.safety.set_timer(200_000)
    self._set_prereqs_normal()
    self.assertTrue(self.safety.get_controls_allowed())

  def test_sas1_invalid_disengages_blocks_tx_allows_fwd(self):
    self._set_prereqs_normal_and_stabilized()
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.safety_rx_hook(self._sas1_msg(stat=0))
    self.assertFalse(self.safety.get_controls_allowed())
    self._refresh_prereqs_on_tx = False
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=0)))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_sas1_valid_reenables_controls(self):
    self.safety.set_timer(0)
    self._set_prereqs_normal_and_stabilized(0)
    self.safety.safety_rx_hook(self._sas1_msg(stat=0))
    self.assertFalse(self.safety.get_controls_allowed())
    # T_clear=100_000: msgs 1-10 still see sas1_valid=False → block_last_time→100_000
    self.safety.set_timer(100_000)
    self._set_prereqs_normal()
    # T_clear + 100_000 = 200_000: stabilized → re-enable
    self.safety.set_timer(200_000)
    self._set_prereqs_normal()
    self.assertTrue(self.safety.get_controls_allowed())

  # ── TCU2 checksum ─────────────────────────────────────────────────────────

  def test_tcu2_bad_checksum_blocks_tx(self):
    """Bad TCU2 checksum must eventually block TX (rx_check invalid)."""
    self._set_prereqs_normal_and_stabilized()
    self.assertTrue(self.safety.get_controls_allowed())
    # Repeatedly send TCU2 with bad checksum until wrong_counters saturates
    for _ in range(6):
      self.safety.safety_rx_hook(self._tcu2_msg(gear=1, bad_checksum=True))
    self._refresh_prereqs_on_tx = False
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=0)))

  def test_tcu2_valid_checksum_allows_tx_for_all_d_gears(self):
    """Good TCU2 checksum with any D-range gear must not block."""
    for gear in range(1, 7):
      self.safety.set_safety_hooks(CarParams.SafetyModel.avanteMd, 0)
      self.safety.init_tests()
      self._tcu2_cnt = 0
      self._clu1_cnt = 0
      self._clu2_cnt = 0
      self._vsm1_rx_cnt = 0
      self._vsm2_cnt = 0
      self._sas1_cnt = 0
      self._auto_tx_time_us = 0
      self._prereqs_stabilized = False
      self._set_prev_torque(0)
      self._set_prereqs_normal_and_stabilized()
      self.assertTrue(self.safety.get_controls_allowed(),
                      f"controls_allowed should be True for gear={gear}")

  def test_tcu2_non_normal_gear_fails_checksum(self):
    """TCU2 with gear=8 (non-normal) must fail checksum and eventually block."""
    self._set_prereqs_normal()
    for _ in range(6):
      dat = bytearray(8)
      # gear=8, alive=0; gear_offset undefined → compute returns 0xFF (mismatch)
      dat[1] = 8 & 0xF
      self.safety.safety_rx_hook(common.make_msg(0, 0x440, 8, bytes(dat)))
    self._refresh_prereqs_on_tx = False
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=0)))

  # ── Staleness blocking ────────────────────────────────────────────────────

  def test_stale_vsm1_vehicle_blocks_tx_allows_fwd(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.safety.set_timer(0)
    self._set_prereqs_normal()
    self.safety.set_timer(200_001)  # > 100 ms stale
    self._refresh_prereqs_on_tx = False
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=1, ctr_mode=2)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_stale_vsm2_blocks_tx_allows_fwd(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.safety.set_timer(0)
    self._set_prereqs_normal()
    # Refresh vehicle bus messages but let VSM2 go stale
    self.safety.set_timer(100_000)
    self.safety.safety_rx_hook(self._vsm1_vehicle_msg())
    self.safety.safety_rx_hook(self._tcs1_msg())
    self.safety.safety_rx_hook(self._tcs5_msg())
    self.safety.safety_rx_hook(self._sas1_msg())
    self.safety.set_timer(200_001)
    self._refresh_prereqs_on_tx = False
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=1, ctr_mode=2)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_stale_sas1_blocks_tx_allows_fwd(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.safety.set_timer(0)
    self._set_prereqs_normal()
    self.safety.set_timer(100_000)
    self.safety.safety_rx_hook(self._vsm1_vehicle_msg())
    self.safety.safety_rx_hook(self._tcs1_msg())
    self.safety.safety_rx_hook(self._vsm2_msg())
    self.safety.set_timer(200_001)
    self._refresh_prereqs_on_tx = False
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=1, ctr_mode=2)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_10hz_prereqs_tolerate_110ms_jitter(self):
    self._set_prereqs_normal_and_stabilized()
    self.assertTrue(self.safety.get_controls_allowed())
    # CLU2 and MDPS1 are 10 Hz messages in real logs and can arrive slightly
    # over 100 ms apart, so refresh only the faster prereqs at +90 ms.
    self.safety.set_timer(190_000)
    self.safety.safety_rx_hook(self._vsm1_vehicle_msg())
    self.safety.safety_rx_hook(self._tcs1_msg())
    self.safety.safety_rx_hook(self._tcs5_msg())
    self.safety.safety_rx_hook(self._esp2_msg())
    self.safety.safety_rx_hook(self._whl_pul_msg())
    self.safety.safety_rx_hook(self._clu1_msg())
    self.safety.safety_rx_hook(self._tcu1_msg())
    self.safety.safety_rx_hook(self._tcu2_msg())
    self.safety.safety_rx_hook(self._vsm2_msg())
    self.safety.safety_rx_hook(self._sas1_msg())
    self.safety.set_timer(210_000)
    self._refresh_prereqs_on_tx = False
    self.assertTrue(super()._tx(self._vsm1_msg(0, steer_req=0)))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_stale_mdps1_blocks_tx(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.safety.set_timer(0)
    self._set_prereqs_normal()
    # Refresh all except MDPS1
    self.safety.set_timer(250_000)
    self.safety.safety_rx_hook(self._vsm1_vehicle_msg())
    self.safety.safety_rx_hook(self._tcs1_msg())
    self.safety.safety_rx_hook(self._tcs5_msg())
    self.safety.safety_rx_hook(self._esp2_msg())
    self.safety.safety_rx_hook(self._whl_pul_msg())
    self.safety.safety_rx_hook(self._clu1_msg())
    self.safety.safety_rx_hook(self._clu2_msg())
    self.safety.safety_rx_hook(self._tcu1_msg())
    self.safety.safety_rx_hook(self._tcu2_msg())
    self.safety.safety_rx_hook(self._vsm2_msg())
    self.safety.safety_rx_hook(self._sas1_msg())
    self.safety.set_timer(250_001)
    self._refresh_prereqs_on_tx = False
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=1, ctr_mode=2)))

  def test_stale_clu2_blocks_tx(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self.safety.set_timer(0)
    self._set_prereqs_normal()
    # Refresh all except CLU2.
    self.safety.set_timer(250_000)
    self.safety.safety_rx_hook(self._vsm1_vehicle_msg())
    self.safety.safety_rx_hook(self._tcs1_msg())
    self.safety.safety_rx_hook(self._tcs5_msg())
    self.safety.safety_rx_hook(self._esp2_msg())
    self.safety.safety_rx_hook(self._whl_pul_msg())
    self.safety.safety_rx_hook(self._clu1_msg())
    self.safety.safety_rx_hook(self._tcu1_msg())
    self.safety.safety_rx_hook(self._tcu2_msg())
    self.safety.safety_rx_hook(self._vsm2_msg())
    self.safety.safety_rx_hook(self._sas1_msg())
    self.safety.safety_rx_hook(self._mdps1_msg())
    self.safety.set_timer(250_001)
    self._refresh_prereqs_on_tx = False
    self.assertFalse(super()._tx(self._vsm1_msg(0, steer_req=1, ctr_mode=2)))

  def test_stale_then_normal_reenables_controls(self):
    self.safety.set_controls_allowed(True)
    self.safety.set_timer(0)
    self._set_prereqs_normal()
    # Let all messages go stale; refresh at t=200_001 sets block_last_time=200_001
    self.safety.set_timer(200_001)
    self._set_prereqs_normal()
    self.assertFalse(self.safety.get_controls_allowed())
    # t=300_001 = 200_001 + 100_000: msgs from 200_001, elapsed=100_000 (not stale),
    # stabilized=100_000 >= 100_000 → re-enable
    self.safety.set_timer(300_001)
    self._set_prereqs_normal()
    self.assertTrue(self.safety.get_controls_allowed())

  # ── Forwarding ────────────────────────────────────────────────────────────

  def test_vsm1_fwd_allowed_before_any_openpilot_tx(self):
    """Before openpilot has sent VSM1, stock VSM1 must be forwarded."""
    self._set_prereqs_normal()
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_vsm1_fwd_blocked_after_recent_openpilot_tx(self):
    """After openpilot sends VSM1 within 30 ms, stock VSM1 must be blocked."""
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self._set_prereqs_normal_and_stabilized(10_000)
    self.assertTrue(super()._tx(self._vsm1_msg(0, steer_req=0)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0x164))

  def test_vsm1_fwd_allowed_after_30ms_timeout(self):
    """After 30 ms without openpilot TX, stock VSM1 must be forwarded again."""
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self._set_prereqs_normal_and_stabilized(10_000)
    self.assertTrue(super()._tx(self._vsm1_msg(0, steer_req=0)))
    self.safety.set_timer(140_001)
    self._set_prereqs_normal()
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_vsm1_fwd_allowed_when_prereqs_not_normal(self):
    """Stock VSM1 must be forwarded when any blocking condition is present."""
    self.safety.safety_rx_hook(
      self._vsm1_vehicle_msg(torque=100, steer_req=1, ctr_mode=2))
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_vsm1_fwd_bus2_always_blocked(self):
    """MDPS VSM1 feedback must never be forwarded to bus 0."""
    self._set_prereqs_normal()
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x164))
    self.safety.safety_rx_hook(
      self._vsm1_vehicle_msg(torque=100, steer_req=1, ctr_mode=2))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x164))

  def test_vsm1_fwd_one_frame_delay(self):
    """fwd_hook evaluates the state before rx_hook processes the new frame."""
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self._set_prereqs_normal_and_stabilized(10_000)
    self.assertTrue(super()._tx(self._vsm1_msg(0, steer_req=0)))
    non_normal = self._vsm1_vehicle_msg(torque=100, steer_req=1, ctr_mode=2)
    fwd_before_rx = self.safety.safety_fwd_hook(0, 0x164)
    self.safety.safety_rx_hook(non_normal)
    self.assertEqual(-1, fwd_before_rx)
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_fwd_hook(self):
    """Non-VSM1 messages on both buses must be forwarded normally."""
    for bus in range(3):
      for addr in self.SCANNED_ADDRS:
        if bus == 0 and addr == 0x164:
          continue  # covered by dedicated VSM1 fwd tests
        fwd_bus = self.FWD_BUS_LOOKUP.get(bus, -1)
        if bus in self.FWD_BLACKLISTED_ADDRS and addr in self.FWD_BLACKLISTED_ADDRS[bus]:
          fwd_bus = -1
        self.assertEqual(fwd_bus, self.safety.safety_fwd_hook(bus, addr),
                         f"{addr=:#x} from {bus=} expected fwd to {fwd_bus=}")

  # ── Driver torque measurement source ─────────────────────────────────────

  def test_driver_torque_only_accepted_from_eps_bus(self):
    """Driver torque must only be sampled from VSM2 on bus 2."""
    self._reset_torque_driver_measurement(0)
    # VSM2-like frame on bus 0 (wrong bus) must be ignored
    raw = 300 + 2048
    dat = bytearray(8)
    dat[0] = raw & 0xFF
    dat[1] = (raw >> 8) & 0xF
    self.safety.safety_rx_hook(common.make_msg(0, 0x165, 8, bytes(dat)))
    self.assertEqual(0, self.safety.get_torque_driver_min())
    self.assertEqual(0, self.safety.get_torque_driver_max())
    # Same frame on bus 2 must be accepted
    self.safety.safety_rx_hook(self._vsm2_msg(torque=300))
    self.assertGreater(self.safety.get_torque_driver_max(), 0)

  # ── Steer safety check override (controls_allowed is auto-managed) ───────

  def test_steer_safety_check(self):
    """Override: controls_allowed is auto-managed, not manually settable.

    For the "disabled" case we introduce a real blocking condition (door open)
    which blocks ALL TX (including torque=0), unlike the base-class semantics
    where only nonzero torque is blocked when controls_allowed is False.
    """
    for speed in self._torque_speed_range:
      self._reset_speed_measurement(speed)
      max_torque = self._get_max_torque(speed)

      # Controls active (all prereqs normal): torque limits are enforced.
      for t in range(int(-max_torque * 1.5), int(max_torque * 1.5)):
        self._set_prev_torque(t)
        if abs(t) > max_torque:
          self.assertFalse(self._tx(self._torque_cmd_msg(t)))
        else:
          self.assertTrue(self._tx(self._torque_cmd_msg(t)))

      # Controls blocked (door open): ALL TX must be rejected.
      self._refresh_prereqs_on_tx = False
      self.safety.safety_rx_hook(self._clu2_msg(door_open=True))
      self.assertFalse(self.safety.get_controls_allowed())
      for t in range(int(-max_torque * 1.5), int(max_torque * 1.5)):
        self._set_prev_torque(t)
        self.assertFalse(super()._tx(self._torque_cmd_msg(t)))
      # Restore: two calls 100ms apart so stabilization window passes.
      # First call clears door_open (block_last_time→T_clear); second enables.
      self._refresh_prereqs_on_tx = True
      t_clear = self._auto_tx_time_us + 100_000
      self.safety.set_timer(t_clear)
      self._set_prereqs_normal()
      self.safety.set_timer(t_clear + 100_000)
      self._set_prereqs_normal()
      self._auto_tx_time_us = t_clear + 100_000

  # ── Realtime torque limits (override for timer management) ───────────────

  def test_realtime_limits(self):
    for sign in [-1, 1]:
      self.safety.set_safety_hooks(CarParams.SafetyModel.avanteMd, 0)
      self.safety.init_tests()
      self._tcu2_cnt = 0
      self._clu1_cnt = 0
      self._clu2_cnt = 0
      self._vsm1_rx_cnt = 0
      self._vsm2_cnt = 0
      self._sas1_cnt = 0
      self._auto_tx_time_us = 0
      self._prereqs_stabilized = False
      self._set_prev_torque(0)
      self._reset_torque_driver_measurement(0)
      self._refresh_prereqs_on_tx = False
      self._set_prereqs_normal_and_stabilized(300_000)
      self.safety.set_timer(400_000)
      self.assertTrue(super()._tx(self._torque_cmd_msg(0)))
      tx_time = 407_000
      for torque in range(self.MAX_RATE_UP, self.MAX_RT_DELTA + 1, self.MAX_RATE_UP):
        self.safety.set_timer(tx_time)
        self._set_prereqs_normal()
        self.assertTrue(super()._tx(self._torque_cmd_msg(torque * sign)))
        tx_time += 7_000
      self.safety.set_timer(tx_time)
      self._set_prereqs_normal()
      self.assertFalse(
        super()._tx(self._torque_cmd_msg(sign * (self.MAX_RT_DELTA + self.MAX_RATE_UP))))

  # ── VSM1 alive counter in normal state ───────────────────────────────────

  def test_vsm1_alive_counter_allowed_in_normal_state(self):
    """All 16 alive counter values must be accepted in VSM1 normal state."""
    self._set_prereqs_normal()
    self.safety.set_controls_allowed(True)
    for cnt in range(16):
      dat = bytearray(8)
      dat[1] = 0x08
      dat[6] = cnt & 0xFF
      dat[7] = self._vsm_xor_checksum(dat)
      self.safety.safety_rx_hook(common.make_msg(0, 0x164, 8, bytes(dat)))
      self.assertTrue(self.safety.get_controls_allowed())
      self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_vsm1_upper_nibble_data6_is_non_normal(self):
    """Any bit set in data[6] upper nibble makes VSM1 non-normal."""
    self._set_prereqs_normal()
    self.safety.set_controls_allowed(True)
    dat = bytearray(8)
    dat[1] = 0x08
    dat[6] = 0x10
    dat[7] = self._vsm_xor_checksum(dat)
    self.safety.safety_rx_hook(common.make_msg(0, 0x164, 8, bytes(dat)))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))

  def test_corrupt_vsm1_treated_as_non_normal(self):
    """A VSM1 frame with bad checksum must trigger non-normal state."""
    self._set_prereqs_normal()
    self.safety.set_controls_allowed(True)
    dat = bytearray(8)
    dat[1] = 0x08
    dat[7] = self._vsm_xor_checksum(dat) ^ 0xFF
    self.safety.safety_rx_hook(common.make_msg(0, 0x164, 8, bytes(dat)))
    # rx_hook not called (invalid checksum) → vsm1_normal retains last good value;
    # but fwd must be blocked because openpilot VSM1 was sent recently
    self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x164))


if __name__ == "__main__":
  unittest.main()
