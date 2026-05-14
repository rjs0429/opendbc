#!/usr/bin/env python3
import unittest
from collections import defaultdict

from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.avante_md.avantecan import VSM1, VSM1_STALE_NANOS, vsm1_checksum
from opendbc.car.avante_md.carstate import AVANTE_FAULT_PERMANENT_FRAMES, CarState
from opendbc.car.avante_md.interface import CarInterface
from opendbc.car.avante_md.values import CAR, CanBus

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter


class FakeCANParser:
  def __init__(self, vl, can_valid=True):
    self.vl = vl
    self.can_valid = can_valid


def nested_vl():
  return defaultdict(lambda: defaultdict(int))


def normal_vsm1():
  dat = bytearray(8)
  dat[1] = 0x08
  dat[7] = vsm1_checksum(dat)
  return bytes(dat)


class TestAvanteMdCarState(unittest.TestCase):
  def setUp(self):
    CP = CarInterface.get_params(CAR.AVANTE_MD_2012, gen_empty_fingerprint(), [], False, False, False)
    self.CS = CarState(CP)
    self.vehicle = nested_vl()
    self.eps = nested_vl()
    self.parsers = {
      Bus.pt: FakeCANParser(self.vehicle),
      Bus.adas: FakeCANParser(self.eps),
    }
    self.nanos = 0
    self._set_normal_signals()

  def _set_normal_signals(self):
    self.vehicle["CLU1"]["CF_Clu_SPEED_UNIT"] = 0
    self.vehicle["CLU1"]["CF_Clu_Vanz"] = 30
    self.vehicle["CLU1"]["CF_Clu_ParkBrakeSw"] = 0
    self.vehicle["CLU2"]["CF_Clu_DrvDrSw"] = 0
    self.vehicle["CLU2"]["CF_Clu_AstDrSw"] = 0
    self.vehicle["CLU2"]["CF_Clu_DrvSeatBeltSw"] = 0  # 0 = latched per safety spec
    self.vehicle["TCU1"]["CUR_GR"] = 5   # drive
    self.vehicle["TCU2"]["CUR_GR"] = 1   # drive gear 1

    for signal in ("WHEEL_FL", "WHEEL_FR", "WHEEL_RL", "WHEEL_RR"):
      self.vehicle["TCS5"][signal] = 30

    self.eps["SAS1"]["SAS_Stat"] = 7
    self.eps["SAS1"]["SAS_Angle"] = 0
    self.eps["SAS1"]["SAS_Speed"] = 0
    self.eps["VSM2"]["CF_Mdps_Def"] = 0
    self.eps["VSM2"]["CF_Mdps_SErr"] = 0
    self.eps["VSM2"]["CR_Mdps_StrTq"] = 0
    self.eps["VSM2"]["CR_Mdps_OutTq"] = 0

  def _update(self):
    self.nanos += 10_000_000  # 10 ms
    self.CS.update_vsm1_raw([(self.nanos, [(VSM1, normal_vsm1(), CanBus.VEHICLE)])])
    return self.CS.update(self.parsers)

  def _update_no_vsm1(self):
    self.nanos += 10_000_000
    self.CS.update_vsm1_raw([(self.nanos, [])])
    return self.CS.update(self.parsers)

  # ── Interface params ──────────────────────────────────────────────────────────

  def test_interface_params_are_lat_only_and_allow_standstill_steering(self):
    CP = CarInterface.get_params(CAR.AVANTE_MD_2012, gen_empty_fingerprint(), [], False, False, False)

    self.assertFalse(CP.openpilotLongitudinalControl)
    self.assertFalse(CP.alphaLongitudinalAvailable)
    self.assertFalse(CP.pcmCruise)
    self.assertFalse(CP.autoResumeSng)
    self.assertTrue(CP.radarUnavailable)
    self.assertEqual(-1., CP.minEnableSpeed)
    self.assertEqual(0., CP.minSteerSpeed)
    self.assertTrue(CP.steerAtStandstill)
    self.assertEqual(structs.CarParams.SteerControlType.torque, CP.steerControlType)

  # ── Auto-enable / lateral enable logic ───────────────────────────────────────

  def test_auto_enable_on_first_safe_frame(self):
    ret = self._update()

    self.assertTrue(self.CS.lat_active)
    self.assertEqual(1, len(ret.buttonEvents))
    self.assertEqual(ButtonType.accelCruise, ret.buttonEvents[0].type)
    self.assertFalse(ret.buttonEvents[0].pressed)

  def test_no_duplicate_enable_event_when_already_active(self):
    self._update()  # enable

    ret = self._update()  # second safe frame

    self.assertTrue(self.CS.lat_active)
    self.assertFalse(ret.buttonEvents)

  def test_cancel_event_when_unsafe_state_entered(self):
    self._update()  # enable

    self.vehicle["CLU1"]["CF_Clu_ParkBrakeSw"] = 1
    ret = self._update()

    self.assertFalse(self.CS.lat_active)
    self.assertEqual(1, len(ret.buttonEvents))
    self.assertEqual(ButtonType.cancel, ret.buttonEvents[0].type)
    self.assertFalse(ret.buttonEvents[0].pressed)

  def test_cancel_event_fires_only_once_per_transition(self):
    self._update()  # enable

    self.vehicle["CLU1"]["CF_Clu_ParkBrakeSw"] = 1
    self._update()  # cancel

    ret = self._update()  # still unsafe

    self.assertFalse(self.CS.lat_active)
    self.assertFalse(ret.buttonEvents)

  def test_cancel_on_temporary_steer_fault(self):
    self._update()  # enable

    self.eps["SAS1"]["SAS_Stat"] = 0
    ret = self._update()

    self.assertFalse(self.CS.lat_active)
    self.assertEqual(ButtonType.cancel, ret.buttonEvents[0].type)

  def test_cancel_on_permanent_steer_fault(self):
    self._update()  # enable

    self.CS.steer_fault_permanent = True
    ret = self._update()

    self.assertFalse(self.CS.lat_active)
    self.assertEqual(ButtonType.cancel, ret.buttonEvents[0].type)

  def test_re_enable_after_unsafe_clears(self):
    self._update()  # enable

    self.vehicle["CLU1"]["CF_Clu_ParkBrakeSw"] = 1
    self._update()  # cancel

    self.vehicle["CLU1"]["CF_Clu_ParkBrakeSw"] = 0
    ret = self._update()  # re-enable

    self.assertTrue(self.CS.lat_active)
    self.assertEqual(ButtonType.accelCruise, ret.buttonEvents[0].type)

  def test_no_enable_while_unsafe_state(self):
    self.vehicle["CLU1"]["CF_Clu_ParkBrakeSw"] = 1
    ret = self._update()

    self.assertFalse(self.CS.lat_active)
    self.assertFalse(ret.buttonEvents)

  # ── Gear detection ────────────────────────────────────────────────────────────

  def test_gear_drive_requires_tcu1_and_tcu2_both(self):
    self.vehicle["TCU1"]["CUR_GR"] = 5
    self.vehicle["TCU2"]["CUR_GR"] = 3
    ret = self._update()
    self.assertEqual(GearShifter.drive, ret.gearShifter)
    self.assertTrue(self.CS.lat_active)

  def test_gear_not_drive_when_tcu1_not_5(self):
    self.vehicle["TCU1"]["CUR_GR"] = 3  # not drive
    self.vehicle["TCU2"]["CUR_GR"] = 3
    ret = self._update()
    self.assertNotEqual(GearShifter.drive, ret.gearShifter)
    self.assertFalse(self.CS.lat_active)

  def test_gear_not_drive_when_tcu2_out_of_range(self):
    self.vehicle["TCU1"]["CUR_GR"] = 5
    self.vehicle["TCU2"]["CUR_GR"] = 7  # out of 1..6
    ret = self._update()
    self.assertNotEqual(GearShifter.drive, ret.gearShifter)
    self.assertFalse(self.CS.lat_active)

  def test_gear_reverse_from_tcu2(self):
    self.vehicle["TCU1"]["CUR_GR"] = 5
    self.vehicle["TCU2"]["CUR_GR"] = 14
    ret = self._update()
    self.assertEqual(GearShifter.reverse, ret.gearShifter)
    self.assertFalse(self.CS.lat_active)

  def test_gear_unknown_falls_through(self):
    self.vehicle["TCU1"]["CUR_GR"] = 0
    self.vehicle["TCU2"]["CUR_GR"] = 0
    ret = self._update()
    self.assertEqual(GearShifter.unknown, ret.gearShifter)
    self.assertFalse(self.CS.lat_active)

  # ── Seatbelt polarity ─────────────────────────────────────────────────────────

  def test_seatbelt_unlatched_when_signal_nonzero(self):
    self.vehicle["CLU2"]["CF_Clu_DrvSeatBeltSw"] = 1
    ret = self._update()
    self.assertTrue(ret.seatbeltUnlatched)
    self.assertFalse(self.CS.lat_active)

  def test_seatbelt_latched_when_signal_zero(self):
    self.vehicle["CLU2"]["CF_Clu_DrvSeatBeltSw"] = 0
    ret = self._update()
    self.assertFalse(ret.seatbeltUnlatched)

  # ── Longitudinal fields zeroed ────────────────────────────────────────────────

  def test_brake_and_gas_always_zero(self):
    ret = self._update()
    self.assertEqual(0, ret.brake)
    self.assertFalse(ret.brakePressed)
    self.assertFalse(ret.gasPressed)

  # ── VSM1 staleness ────────────────────────────────────────────────────────────

  def test_vsm1_stale_triggers_temporary_fault(self):
    self._update()  # prime can_update_nanos and vsm1_rx_nanos

    # Advance can_update_nanos past the stale window without updating VSM1
    stale_gap = VSM1_STALE_NANOS + 1_000_000
    self.CS.can_update_nanos = self.CS.vsm1_rx_nanos + stale_gap

    ret = self.CS.update(self.parsers)

    self.assertTrue(ret.steerFaultTemporary)

  def test_vsm1_fresh_no_fault(self):
    ret = self._update()
    self.assertFalse(ret.steerFaultTemporary)

  # ── Permanent fault latch ─────────────────────────────────────────────────────

  def test_vsm2_fault_latches_steer_fault_permanent_after_10_frames(self):
    self.eps["VSM2"]["CF_Mdps_Def"] = 1
    for _ in range(AVANTE_FAULT_PERMANENT_FRAMES):
      ret = self._update()

    self.assertTrue(ret.steerFaultPermanent)

  def test_permanent_fault_does_not_latch_before_threshold(self):
    self.eps["VSM2"]["CF_Mdps_Def"] = 1
    for _ in range(AVANTE_FAULT_PERMANENT_FRAMES - 1):
      ret = self._update()

    self.assertFalse(ret.steerFaultPermanent)

  def test_permanent_fault_latches_and_persists_after_fault_clears(self):
    self.eps["VSM2"]["CF_Mdps_Def"] = 1
    for _ in range(AVANTE_FAULT_PERMANENT_FRAMES):
      self._update()

    self.eps["VSM2"]["CF_Mdps_Def"] = 0
    for _ in range(AVANTE_FAULT_PERMANENT_FRAMES):
      ret = self._update()

    self.assertTrue(ret.steerFaultPermanent)

  def test_startup_can_invalid_does_not_latch_until_valid_seen(self):
    self.parsers[Bus.pt].can_valid = False
    self.parsers[Bus.adas].can_valid = False
    for _ in range(AVANTE_FAULT_PERMANENT_FRAMES * 2):
      ret = self._update()

    self.assertFalse(ret.steerFaultPermanent)
    self.assertFalse(self.CS.can_valid_seen_once)

    self.parsers[Bus.pt].can_valid = True
    self.parsers[Bus.adas].can_valid = True
    ret = self._update()
    self.assertFalse(ret.steerFaultPermanent)
    self.assertTrue(self.CS.can_valid_seen_once)

    self.parsers[Bus.pt].can_valid = False
    for _ in range(AVANTE_FAULT_PERMANENT_FRAMES):
      ret = self._update()

    self.assertTrue(ret.steerFaultPermanent)

  # ── Cruise state ──────────────────────────────────────────────────────────────

  def test_cruise_state_fields(self):
    ret = self._update()
    self.assertTrue(ret.cruiseState.available)
    self.assertFalse(ret.cruiseState.enabled)
    self.assertFalse(ret.cruiseState.standstill)
    self.assertFalse(ret.cruiseState.nonAdaptive)


if __name__ == "__main__":
  unittest.main()
