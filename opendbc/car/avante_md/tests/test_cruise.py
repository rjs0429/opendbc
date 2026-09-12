#!/usr/bin/env python3
import unittest

from opendbc.car.avante_md.avantecan import CLU1, CLU1_SW_SET, create_clu1
from opendbc.car.avante_md.carstate import EcoLongPress
from opendbc.car.avante_md.cruise import CruiseState, CruiseStateMachine
from opendbc.car.avante_md.values import CanBus, CruiseParams

FRAME = 10_000_000
LAMP_LATENCY = 100_000_000
MAIN_FAULT_BUDGET = CruiseParams.MAX_PRESS_ATTEMPTS * (CruiseParams.MAIN_PRESS_NANOS +
                                                       CruiseParams.MAIN_SETTLE_NANOS + 2 * FRAME)


class FakeEcm:
  """Minimal model of the measured ECM behaviour: CruiseSwMain is an edge toggle, SwState 2 sets."""

  def __init__(self, main=False, set_=False, main_responds=True, set_responds=True):
    self.main = main
    self.set = set_
    self.main_responds = main_responds
    self.set_responds = set_responds
    self._main_since = None
    self._main_toggled = False
    self._set_since = None

  def update(self, now, sw_main, sw_state):
    if sw_main:
      if self._main_since is None:
        self._main_since = now
      elif self.main_responds and not self._main_toggled and now - self._main_since >= LAMP_LATENCY:
        self.main = not self.main
        self._main_toggled = True
        if not self.main:
          self.set = False
    else:
      self._main_since = None
      self._main_toggled = False

    if sw_state == CLU1_SW_SET and self.main:
      if self._set_since is None:
        self._set_since = now
      elif self.set_responds and now - self._set_since >= LAMP_LATENCY:
        self.set = True
    else:
      self._set_since = None


class Sim:
  def __init__(self, ecm=None, v_kph=60.):
    self.sm = CruiseStateMachine()
    self.ecm = ecm if ecm is not None else FakeEcm()
    self.now = 0
    self.v = v_kph
    self.precond = True
    self.lamps_valid = True
    self.clu1_fresh = True

  def step(self, long_press=False):
    self.now += FRAME
    self.ecm.update(self.now, self.sm.sw_main, self.sm.sw_state)
    self.sm.update(self.now, long_press, self.lamps_valid, self.ecm.main, self.ecm.set,
                   self.v, self.precond, self.clu1_fresh)
    return self.sm.state

  def run(self, nanos, long_press=False):
    end = self.now + nanos
    while self.now < end:
      self.step(long_press=long_press)
      long_press = False
    return self.sm.state

  def engage(self):
    self.run(FRAME)                       # leave INIT
    self.step(long_press=True)
    self.run(5 * LAMP_LATENCY + CruiseParams.SET_READY_DWELL_NANOS + CruiseParams.GAP_NANOS)
    return self.sm.state


class TestEcoLongPress(unittest.TestCase):
  def setUp(self):
    self.det = EcoLongPress(CruiseParams.HOLD_NANOS)
    self.now = 0

  def _hold(self, nanos, level=1):
    fired = False
    end = self.now + nanos
    while self.now < end:
      self.now += FRAME
      fired |= self.det.update(level, self.now)
    return fired

  def test_short_press_does_not_fire(self):
    self._hold(FRAME * EcoLongPress.DEBOUNCE_FRAMES, 0)
    self.assertFalse(self._hold(500_000_000))
    self.assertFalse(self.det.consumed)

  def test_long_press_fires_once_and_consumes_the_press(self):
    self._hold(FRAME * EcoLongPress.DEBOUNCE_FRAMES, 0)
    self.assertTrue(self._hold(CruiseParams.HOLD_NANOS + 5 * FRAME))
    self.assertTrue(self.det.consumed)

    # still held: no second trigger
    self.assertFalse(self._hold(CruiseParams.HOLD_NANOS))
    self.assertTrue(self.det.consumed)

    # released: the press stops shadowing the steering toggle
    self._hold(FRAME * (EcoLongPress.DEBOUNCE_FRAMES + 1), 0)
    self.assertFalse(self.det.consumed)

  def test_release_voids_the_hold_window(self):
    self._hold(FRAME * EcoLongPress.DEBOUNCE_FRAMES, 0)
    self.assertFalse(self._hold(CruiseParams.HOLD_NANOS - 200_000_000))
    self._hold(FRAME * EcoLongPress.DEBOUNCE_FRAMES, 0)
    self.assertFalse(self._hold(CruiseParams.HOLD_NANOS - 200_000_000))

  def test_hold_exceeds_steering_double_click_window(self):
    from opendbc.car.avante_md.carstate import MomentaryButtonDoubleClick
    self.assertGreater(CruiseParams.HOLD_NANOS / 1e9, MomentaryButtonDoubleClick.DOUBLE_CLICK_INTERVAL)


class TestCruiseStateMachine(unittest.TestCase):
  def test_init_syncs_from_lamps(self):
    for main, set_, expected in ((False, False, CruiseState.IDLE),
                                 (True, False, CruiseState.WAIT_SET_READY),
                                 (True, True, CruiseState.ACTIVE)):
      sim = Sim(FakeEcm(main=main, set_=set_))
      self.assertEqual(expected, sim.run(FRAME))

  def test_long_press_engages_main_then_set(self):
    sim = Sim()
    self.assertEqual(CruiseState.ACTIVE, sim.engage())
    self.assertTrue(sim.ecm.main)
    self.assertTrue(sim.ecm.set)
    self.assertFalse(sim.sm.transmitting)

  def test_long_press_while_active_returns_to_idle(self):
    sim = Sim()
    sim.engage()
    sim.step(long_press=True)
    self.assertEqual(CruiseState.IDLE, sim.run(5 * LAMP_LATENCY))
    self.assertFalse(sim.ecm.main)
    self.assertFalse(sim.ecm.set)

  def test_set_lamp_drop_turns_main_off(self):
    sim = Sim()
    sim.engage()
    sim.ecm.set = False  # brake, sub-40 km/h or ECM cut
    self.assertEqual(CruiseState.PRESS_MAIN_OFF, sim.step())
    self.assertEqual(CruiseState.IDLE, sim.run(5 * LAMP_LATENCY))
    self.assertFalse(sim.ecm.main)

  def test_no_reset_without_a_new_long_press(self):
    sim = Sim()
    sim.engage()
    sim.ecm.set = False
    sim.run(5 * LAMP_LATENCY)
    self.assertEqual(CruiseState.IDLE, sim.run(10 * CruiseParams.SET_READY_TIMEOUT_NANOS))
    self.assertFalse(sim.ecm.main)

  def test_set_is_not_retapped_once_engaged(self):
    sim = Sim()
    sim.engage()
    for _ in range(500):
      self.assertNotEqual(CLU1_SW_SET, sim.sm.sw_state)
      sim.step()

  def test_externally_set_cruise_is_adopted_without_pressing(self):
    sim = Sim(v_kph=0.)
    sim.run(FRAME)
    sim.step(long_press=True)
    sim.run(5 * LAMP_LATENCY)
    self.assertEqual(CruiseState.WAIT_SET_READY, sim.sm.state)

    sim.ecm.set = True
    self.assertEqual(CruiseState.ACTIVE, sim.step())
    self.assertFalse(sim.sm.transmitting)

  def test_main_lamp_loss_while_setting_returns_to_idle(self):
    sim = Sim(FakeEcm(set_responds=False))
    sim.run(FRAME)
    sim.step(long_press=True)
    sim.run(5 * LAMP_LATENCY + CruiseParams.SET_READY_DWELL_NANOS + CruiseParams.GAP_NANOS)
    self.assertEqual(CruiseState.PRESS_SET, sim.sm.state)

    sim.ecm.main = False
    self.assertEqual(CruiseState.IDLE, sim.step())
    self.assertFalse(sim.sm.transmitting)

  def test_speed_below_minimum_blocks_set(self):
    sim = Sim(v_kph=CruiseParams.SET_SPEED_MIN_KPH - 1.)
    sim.run(FRAME)
    sim.step(long_press=True)
    self.assertEqual(CruiseState.WAIT_SET_READY, sim.run(5 * LAMP_LATENCY))
    self.assertTrue(sim.ecm.main)
    self.assertFalse(sim.ecm.set)

    sim.v = CruiseParams.SET_SPEED_MIN_KPH
    self.assertEqual(CruiseState.ACTIVE, sim.run(5 * LAMP_LATENCY + CruiseParams.SET_READY_DWELL_NANOS))

  def test_speed_above_maximum_blocks_set(self):
    sim = Sim(v_kph=CruiseParams.SET_SPEED_MAX_KPH + 1.)
    sim.run(FRAME)
    sim.step(long_press=True)
    self.assertEqual(CruiseState.WAIT_SET_READY, sim.run(5 * LAMP_LATENCY))
    self.assertFalse(sim.ecm.set)

  def test_set_ready_timeout_reverts_main(self):
    sim = Sim(v_kph=0.)
    sim.run(FRAME)
    sim.step(long_press=True)
    sim.run(5 * LAMP_LATENCY)
    self.assertTrue(sim.ecm.main)

    self.assertEqual(CruiseState.IDLE, sim.run(CruiseParams.SET_READY_TIMEOUT_NANOS + 5 * LAMP_LATENCY))
    self.assertFalse(sim.ecm.main)

  def test_main_press_without_lamp_faults_after_retries(self):
    sim = Sim(FakeEcm(main_responds=False))
    sim.run(FRAME)
    sim.step(long_press=True)
    budget = MAIN_FAULT_BUDGET
    self.assertEqual(CruiseState.FAULT, sim.run(budget + FRAME))
    self.assertFalse(sim.sm.transmitting)

  def test_set_press_without_lamp_faults_after_retries(self):
    sim = Sim(FakeEcm(set_responds=False))
    sim.run(FRAME)
    sim.step(long_press=True)
    budget = CruiseParams.MAX_PRESS_ATTEMPTS * (CruiseParams.SET_PRESS_NANOS + CruiseParams.SET_SETTLE_NANOS +
                                                CruiseParams.GAP_NANOS + CruiseParams.SET_READY_DWELL_NANOS)
    self.assertEqual(CruiseState.FAULT, sim.run(budget + 5 * LAMP_LATENCY))

  def test_fault_is_cleared_by_a_new_long_press(self):
    sim = Sim(FakeEcm(main_responds=False))
    sim.run(FRAME)
    sim.step(long_press=True)
    budget = MAIN_FAULT_BUDGET
    self.assertEqual(CruiseState.FAULT, sim.run(budget + FRAME))

    sim.ecm.main_responds = True
    self.assertEqual(CruiseState.IDLE, sim.step(long_press=True))

  def test_fault_survives_signal_loss(self):
    sim = Sim(FakeEcm(main_responds=False))
    sim.run(FRAME)
    sim.step(long_press=True)
    budget = MAIN_FAULT_BUDGET
    sim.run(budget + FRAME)

    sim.lamps_valid = False
    self.assertEqual(CruiseState.FAULT, sim.run(10 * FRAME))

  def test_lost_preconditions_stop_transmitting_and_resync(self):
    sim = Sim()
    sim.engage()

    sim.precond = False
    self.assertEqual(CruiseState.INIT, sim.step())
    self.assertFalse(sim.sm.transmitting)

    sim.precond = True
    self.assertEqual(CruiseState.ACTIVE, sim.step())

  def test_stale_clu1_stops_transmitting(self):
    sim = Sim(FakeEcm(main_responds=False))
    sim.run(FRAME)
    sim.step(long_press=True)
    sim.run(3 * FRAME)
    self.assertTrue(sim.sm.transmitting)

    sim.clu1_fresh = False
    sim.step()
    self.assertFalse(sim.sm.transmitting)

  def test_main_press_is_released_between_attempts(self):
    sim = Sim(FakeEcm(main_responds=False))
    sim.run(FRAME)
    sim.step(long_press=True)

    pressed = []
    for _ in range(int((CruiseParams.MAIN_PRESS_NANOS + CruiseParams.MAIN_SETTLE_NANOS) / FRAME) + 2):
      sim.step()
      pressed.append(sim.sm.sw_main)
    self.assertIn(1, pressed)
    self.assertIn(0, pressed[-5:])

  def test_main_press_released_as_soon_as_the_lamp_answers(self):
    sim = Sim()
    sim.run(FRAME)
    sim.step(long_press=True)
    while sim.sm.state == CruiseState.PRESS_MAIN_ON:
      sim.step()
    self.assertEqual(0, sim.sm.sw_main)
    self.assertLess(sim.now, CruiseParams.MAIN_PRESS_NANOS + CruiseParams.MAIN_SETTLE_NANOS)

  def test_long_press_is_ignored_while_a_press_is_running(self):
    sim = Sim(FakeEcm(main_responds=False))
    sim.run(FRAME)
    sim.step(long_press=True)
    sim.run(3 * FRAME)
    self.assertEqual(CruiseState.PRESS_MAIN_ON, sim.step(long_press=True))


class TestCreateClu1(unittest.TestCase):
  GENUINE = bytes([0x85, 0x78, 0x3B, 0xFA, 0x11, 0x22, 0x33, 0x44])

  def test_only_cruise_bits_change(self):
    msg = create_clu1(self.GENUINE, CLU1_SW_SET, 1)
    self.assertEqual(CLU1, msg.address)
    self.assertEqual(CanBus.VEHICLE, msg.src)

    self.assertEqual(CLU1_SW_SET, msg.dat[0] & 0x07)
    self.assertEqual(1, msg.dat[3] & 0x01)
    self.assertEqual(self.GENUINE[0] & ~0x07, msg.dat[0] & ~0x07)
    self.assertEqual(self.GENUINE[3] & ~0x01, msg.dat[3] & ~0x01)
    self.assertEqual(self.GENUINE[1:3], msg.dat[1:3])
    self.assertEqual(self.GENUINE[4:], msg.dat[4:])

  def test_released_frame_matches_the_genuine_one(self):
    genuine = bytes([0x80, 0x78, 0x3B, 0x00, 0x11, 0x22, 0x33, 0x44])
    self.assertEqual(genuine, create_clu1(genuine, 0, 0).dat)


if __name__ == "__main__":
  unittest.main()
