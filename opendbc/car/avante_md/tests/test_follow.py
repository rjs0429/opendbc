#!/usr/bin/env python3
import unittest

from opendbc.can import CANPacker
from opendbc.car.avante_md.avantecan import CLU1_SW_CANCEL, CLU1_SW_RES, CLU1_SW_SET
from opendbc.car.avante_md.cruise import ButtonRequest, CruiseState, CruiseStateMachine
from opendbc.car.avante_md.follow.command import FollowCommand
from opendbc.car.avante_md.follow.estimator import SetSpeedEstimator
from opendbc.car.avante_md.follow.policy import FollowController, FollowPlan, cluster_from_wheel, wheel_from_cluster
from opendbc.car.avante_md.follow.signals import FollowSignalDecoder, FollowSignals
from opendbc.car.avante_md.values import AVANTE_MD_DBC, CruiseParams, FollowParams, SetSpeedParams
from opendbc.car.common.conversions import Conversions as CV

FRAME = 10_000_000
SECOND = 1_000_000_000
STEP = FollowParams.TAP_STEP_KPH
WINDOW = SetSpeedParams.WINDOW_NANOS + 5 * FRAME


def some(x: float | None) -> float:
  assert x is not None
  return x


def normal_signals(**kwargs) -> FollowSignals:
  sig = FollowSignals(valid=True, rpm=1400., turbine_rpm=1400., gear=6, target_gear=6, pedal_pct=20.,
                      long_accel=FollowParams.LONG_ACCEL_BIAS)
  for k, v in kwargs.items():
    setattr(sig, k, v)
  return sig


class TestSetSpeedEstimator(unittest.TestCase):
  def setUp(self):
    self.est = SetSpeedEstimator()
    self.now = 0

  def _hold(self, nanos, v_cluster, steady=True, wheel_jitter=0.):
    end = self.now + nanos
    i = 0
    while self.now < end:
      self.now += FRAME
      wheel = wheel_from_cluster(v_cluster) + (wheel_jitter if i % 2 else 0.)
      self.est.update(self.now, v_cluster, wheel, steady)
      i += 1

  def test_capture_is_the_set_speed(self):
    self.est.capture(self.now, 82.5)
    self.assertTrue(self.est.resolved)
    self.assertEqual(82.5, self.est.v_set)

  def test_heard_tap_is_confirmed_by_the_settled_speed(self):
    self.est.capture(self.now, 80.)
    self.est.tap(self.now, 1)
    self.assertTrue(self.est.pending)
    self._hold(SetSpeedParams.SETTLE_AFTER_TAP_NANOS + SetSpeedParams.WINDOW_NANOS + FRAME, 80. + STEP)
    self.assertTrue(self.est.resolved)
    self.assertAlmostEqual(80. + STEP, self.est.v_set)

  def test_unheard_tap_is_recognised(self):
    self.est.capture(self.now, 80.)
    self.est.tap(self.now, -1)
    self._hold(SetSpeedParams.SETTLE_AFTER_TAP_NANOS + SetSpeedParams.WINDOW_NANOS + FRAME, 80.)
    self.assertTrue(self.est.resolved)
    self.assertAlmostEqual(80., self.est.v_set)

  def test_no_decision_before_the_speed_settles(self):
    self.est.capture(self.now, 80.)
    self.est.tap(self.now, 1)
    self._hold(SetSpeedParams.SETTLE_AFTER_TAP_NANOS, 80. + STEP)
    self.assertTrue(self.est.pending)

  def test_unsteady_speed_never_decides(self):
    self.est.capture(self.now, 80.)
    self.est.tap(self.now, 1)
    self._hold(20 * SECOND, 80. + STEP, wheel_jitter=2 * SetSpeedParams.STEADY_WHEEL_RANGE_KPH)
    self.assertTrue(self.est.pending)

  def test_timeout_keeps_the_likeliest_step_and_asks_for_resync(self):
    self.est.capture(self.now, 80.)
    self.est.tap(self.now, 1)
    self._hold(SetSpeedParams.VERIFY_TIMEOUT_NANOS + FRAME, 80. + STEP, steady=False)
    self.assertEqual(1, self.est.last_commit_steps)
    self.assertFalse(self.est.pending)
    self.assertTrue(self.est.needs_resync)
    self.assertAlmostEqual(80. + STEP, self.est.v_set)

  def test_speed_far_from_every_step_asks_for_resync(self):
    self.est.capture(self.now, 80.)
    self.est.tap(self.now, 1)
    self._hold(SetSpeedParams.SETTLE_AFTER_TAP_NANOS + SetSpeedParams.MISMATCH_LIMIT * WINDOW, 80. - 0.8 * STEP)
    self.assertFalse(self.est.pending)
    self.assertTrue(self.est.needs_resync)

  def test_speed_between_two_steps_is_not_evidence(self):
    for fraction in (0.4, 0.5, 0.6):
      with self.subTest(fraction=fraction):
        self.setUp()
        self.est.capture(self.now, 80.)
        self.est.tap(self.now, 1)
        self._hold(SetSpeedParams.SETTLE_AFTER_TAP_NANOS + (SetSpeedParams.MISMATCH_LIMIT - 1) * WINDOW, 80. + fraction * STEP)
        self.assertTrue(self.est.pending)
        self._hold(WINDOW, 80. + fraction * STEP)
        self.assertTrue(self.est.needs_resync)

  def test_unsettled_tap_times_out_into_a_resync(self):
    self.est.capture(self.now, 80.)
    self.est.tap(self.now, 1)
    self._hold(SetSpeedParams.VERIFY_TIMEOUT_NANOS - FRAME, 80. + STEP, steady=False)
    self.assertTrue(self.est.pending)
    self._hold(2 * FRAME, 80. + STEP, steady=False)
    self.assertTrue(self.est.needs_resync)

  def test_a_second_tap_keeps_the_first_undecided(self):
    for heard, expected in (((True, False), 80. + STEP), ((False, True), 80. - STEP), ((True, True), 80.)):
      with self.subTest(heard=heard):
        self.setUp()
        self.est.capture(self.now, 80.)
        self.est.tap(self.now, 1)
        self._hold(SECOND, 80.)
        self.est.tap(self.now, -1)
        self._hold(SetSpeedParams.SETTLE_AFTER_TAP_NANOS + 3 * SetSpeedParams.WINDOW_NANOS, expected)
        self.assertTrue(self.est.resolved)
        self.assertAlmostEqual(expected, self.est.v_set, delta=0.3)

  def test_resync_request_clears_once_the_speed_agrees_again(self):
    self.est.capture(self.now, 80.)
    self.est.tap(self.now, 1)
    self._hold(SetSpeedParams.VERIFY_TIMEOUT_NANOS + FRAME, 80. + STEP, steady=False)
    self.assertTrue(self.est.needs_resync)
    self._hold(SetSpeedParams.MISMATCH_LIMIT * WINDOW, 80. + STEP)
    self.assertFalse(self.est.needs_resync)

  def test_small_drift_is_tracked(self):
    self.est.capture(self.now, 80.)
    self._hold(SetSpeedParams.SETTLE_AFTER_CAPTURE_NANOS + 4 * SetSpeedParams.WINDOW_NANOS, 80.5)
    self.assertTrue(self.est.resolved)
    self.assertGreater(self.est.v_set, 80.2)

  def test_repeated_mismatch_asks_for_resync(self):
    self.est.capture(self.now, 80.)
    self._hold(SetSpeedParams.SETTLE_AFTER_CAPTURE_NANOS + (SetSpeedParams.MISMATCH_LIMIT - 1) * WINDOW, 82.)
    self.assertFalse(self.est.needs_resync)
    self._hold(WINDOW, 82.)
    self.assertTrue(self.est.needs_resync)

  def test_tap_without_capture_is_ignored(self):
    self.est.tap(self.now, 1)
    self.assertFalse(self.est.pending)
    self.assertFalse(self.est.valid)


class TestFollowSignals(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker(AVANTE_MD_DBC)
    self.decoder = FollowSignalDecoder()

  def _feed(self, nanos, brake_act=1, cyl_pres=1.0, acl=0, pedal=25., rpm=2100., ntc=2080., gear=6, target_gear=6,
            long_accel=0.3, range_=5):
    frames = [
      self.packer.make_can_msg("EMS_DCT1", 0, {"PV_AV_CAN": pedal}),
      self.packer.make_can_msg("EMS_DCT2", 0, {"BRAKE_ACT": brake_act}),
      self.packer.make_can_msg("ESP2", 0, {"CYL_PRES": cyl_pres, "LONG_ACCEL": long_accel}),
      self.packer.make_can_msg("EMS6", 0, {"CF_Ems_AclAct": acl}),
      self.packer.make_can_msg("EMS1", 0, {"N": rpm}),
      self.packer.make_can_msg("TCU3", 0, {"CF_Tcu_TarGr": target_gear}),
      self.packer.make_can_msg("TCU1", 0, {"CUR_GR": range_}),
      self.packer.make_can_msg("TCU2", 0, {"CUR_GR": gear, "N_TC_RAW": ntc}),
    ]
    for addr, dat, _ in frames:
      self.decoder.update_frame(addr, dat, nanos)

  def test_decodes_the_dbc_signals(self):
    self._feed(0)
    sig = self.decoder.decode(0)
    self.assertTrue(sig.valid)
    self.assertFalse(sig.brake)
    self.assertFalse(sig.gas)
    self.assertAlmostEqual(25., sig.pedal_pct, delta=0.4)
    self.assertAlmostEqual(2100., sig.rpm)
    self.assertAlmostEqual(2080., sig.turbine_rpm)
    self.assertEqual(6, sig.gear)
    self.assertEqual(6, sig.target_gear)
    self.assertFalse(sig.sport)
    self.assertAlmostEqual(0.3, sig.long_accel, places=2)
    self._feed(0, range_=8)
    self.assertTrue(self.decoder.decode(0).sport)

  def test_brake_from_switch_or_pressure(self):
    for brake_act, cyl_pres, expected in ((2, 1.0, True), (0, 1.0, True), (1, 3.0, True), (1, 1.0, False)):
      with self.subTest(brake_act=brake_act, cyl_pres=cyl_pres):
        self.setUp()
        for _ in range(FollowParams.BRAKE_PRESSURE_FRAMES):
          self._feed(0, brake_act=brake_act, cyl_pres=cyl_pres)
        self.assertEqual(expected, self.decoder.decode(0).brake)

  def test_pressure_spike_is_not_a_brake(self):
    for _ in range(10):
      self._feed(0, cyl_pres=2.0)
    self.assertFalse(self.decoder.decode(0).brake)

  def test_gas(self):
    self._feed(0, acl=1)
    self.assertTrue(self.decoder.decode(0).gas)

  def test_stale_or_missing_is_invalid(self):
    self.assertFalse(self.decoder.decode(0).valid)
    self._feed(0)
    self.assertFalse(self.decoder.decode(FollowParams.SIGNAL_STALE_NANOS + FRAME).valid)

  def test_other_frames_are_ignored(self):
    self.decoder.update_frame(0x4F0, bytes(8), 0)
    self.assertFalse(self.decoder.decode(0).valid)


class FakeEcm:
  """Stock cruise as measured: MAIN is an edge toggle, SET engages at the current speed, a short RES or SET
  tap moves the set speed one step on release, CANCEL drops SET but keeps MAIN. The car holds the set speed
  while engaged; once cruise lets go of it the car coasts, until the driver's foot is back."""
  LATENCY = 60_000_000
  HOLD = 500_000_000

  def __init__(self, v_kph=80., cancel_works=True, set_min_kph=40.):
    self.main = False
    self.engaged = False
    self.v_set = 0.
    self.v = v_kph
    self.cancel_works = cancel_works
    self.set_min_kph = set_min_kph
    self.hear_taps = True
    self.deaf = False
    self.coasting = False
    self._main_since: int | None = None
    self._main_done = False
    self._button = 0
    self._since = 0
    self._acted = False

  def update(self, now, sw_main, sw_state):
    if self.deaf:
      sw_main, sw_state = 0, 0
    if sw_main:
      if self._main_since is None:
        self._main_since = now
      elif not self._main_done and now - self._main_since >= self.LATENCY:
        self.main = not self.main
        self._main_done = True
        if not self.main and self.engaged:
          self.engaged = False
          self.coasting = True
    else:
      self._main_since = None
      self._main_done = False

    if sw_state != self._button:
      held = now - self._since
      if (self._button in (CLU1_SW_RES, CLU1_SW_SET) and self.engaged and self.hear_taps and not self._acted and
          self.LATENCY <= held < self.HOLD):
        self.v_set += FollowParams.TAP_STEP_KPH * (1 if self._button == CLU1_SW_RES else -1)
      self._button = sw_state
      self._since = now
      self._acted = False
    elif sw_state and not self._acted and now - self._since >= self.LATENCY and self.main:
      if sw_state == CLU1_SW_SET and not self.engaged and self.v >= self.set_min_kph:
        self.engaged = True
        self.coasting = False
        self.v_set = self.v
        self._acted = True
      elif sw_state == CLU1_SW_CANCEL and self.engaged and self.cancel_works:
        self.engaged = False
        self.coasting = True
        self._acted = True

    dv = 1.08 * FRAME / SECOND
    if self.engaged:
      self.v += max(-dv, min(dv, self.v_set - self.v))
    elif self.coasting:
      self.v -= 1.3 * FRAME / SECOND

  def drop(self):
    self.engaged = False
    self.coasting = True


class Sim:
  def __init__(self, ecm=None, plan=None):
    self.ecm = ecm if ecm is not None else FakeEcm()
    self.cruise = CruiseStateMachine()
    self.follow = FollowController()
    self.plan = plan if plan is not None else FollowPlan(True, 80., 0., False)
    self.signals = normal_signals()
    self.brake = False
    self.now = 0
    self.requests: list[ButtonRequest] = []

  def step(self, long_press=False):
    self.now += FRAME
    self.signals.brake = self.brake
    if self.brake and self.ecm.engaged:
      self.ecm.drop()
    request = self.follow.update(self.now, self.plan, self.cruise, self.ecm.engaged, self.ecm.v,
                                 wheel_from_cluster(self.ecm.v) * CV.KPH_TO_MS, 0., self.signals)
    if request != ButtonRequest.NONE:
      self.requests.append(request)
    self.cruise.update(self.now, long_press, True, self.ecm.main, self.ecm.engaged, self.ecm.v, True, True,
                       brake=self.signals.brake if self.signals.valid else None, follow_active=self.plan.valid,
                       request=request, coast_requested=self.plan.valid and self.plan.coast)
    self.follow.after_buttons(self.now, self.cruise)
    self.ecm.update(self.now, self.cruise.sw_main, self.cruise.sw_state)

  def run(self, nanos):
    end = self.now + nanos
    while self.now < end:
      self.step()
    return self.cruise.state

  def run_until(self, predicate, limit):
    end = self.now + limit
    while self.now < end:
      self.step()
      if predicate():
        return True
    return False

  def until_engaged(self, limit):
    engaged = self.run_until(lambda: self.cruise.state == CruiseState.ACTIVE and self.ecm.engaged, limit)
    self.step()
    return engaged

  def engage_at(self, v_kph):
    self.ecm.v = v_kph
    set_min = self.ecm.set_min_kph
    self.ecm.set_min_kph = 0.
    self.engage()
    self.ecm.set_min_kph = set_min

  def engage(self):
    self.step()
    self.step(long_press=True)
    assert self.run_until(lambda: self.cruise.state == CruiseState.ACTIVE, 10 * SECOND)
    self.run(SetSpeedParams.SETTLE_AFTER_CAPTURE_NANOS + SetSpeedParams.WINDOW_NANOS + SECOND)


class TestFollow(unittest.TestCase):
  def test_engagement_arms_and_records_the_driver_speed(self):
    sim = Sim()
    sim.engage()
    self.assertTrue(sim.cruise.armed)
    self.assertAlmostEqual(80., some(sim.follow.v_user_kph))
    self.assertTrue(sim.follow.estimator.resolved)
    self.assertAlmostEqual(sim.ecm.v_set, some(sim.follow.v_set_kph), delta=0.3)

  def test_coast_request_cancels_and_keeps_main(self):
    sim = Sim()
    sim.engage()
    sim.plan = FollowPlan(True, 70., -0.5, True)
    self.assertTrue(sim.run_until(lambda: sim.cruise.state == CruiseState.COAST, 2 * SECOND))
    self.assertTrue(sim.ecm.main)
    self.assertFalse(sim.ecm.engaged)
    self.assertTrue(sim.cruise.armed)
    self.assertEqual(ButtonRequest.CANCEL, sim.requests[0])

  def test_recapture_after_the_lead_no_longer_needs_coasting(self):
    sim = Sim()
    sim.engage()
    sim.plan = FollowPlan(True, 70., -0.5, True)
    sim.run_until(lambda: sim.cruise.state == CruiseState.COAST, 2 * SECOND)
    self.assertFalse(sim.run_until(lambda: sim.ecm.engaged, 5 * SECOND))

    sim.plan = FollowPlan(True, 70., 0., False)
    self.assertTrue(sim.until_engaged(5 * SECOND))
    self.assertAlmostEqual(sim.ecm.v_set, some(sim.follow.v_set_kph), delta=0.1)
    self.assertAlmostEqual(80., some(sim.follow.v_user_kph))

  def test_no_recapture_while_still_decelerating_toward_the_target(self):
    sim = Sim()
    sim.engage()
    sim.plan = FollowPlan(True, 60., -0.5, True)
    sim.run_until(lambda: sim.cruise.state == CruiseState.COAST, 2 * SECOND)
    sim.plan = FollowPlan(True, 60., -0.2, False)
    self.assertFalse(sim.run_until(lambda: sim.ecm.engaged, 3 * SECOND))
    self.assertTrue(sim.run_until(lambda: sim.ecm.engaged, 30 * SECOND))
    self.assertLessEqual(sim.ecm.v_set, 60. + FollowParams.RECAPTURE_SPEED_MARGIN_KPH + 0.1)

  def test_climb_taps_one_step_at_a_time_up_to_the_driver_speed(self):
    sim = Sim()
    sim.engage()
    sim.plan = FollowPlan(True, 70., -0.5, True)
    sim.run_until(lambda: sim.cruise.state == CruiseState.COAST, 2 * SECOND)
    sim.run(8 * SECOND)
    sim.plan = FollowPlan(True, 80., 0.3, False)
    sim.run_until(lambda: sim.ecm.engaged, 5 * SECOND)
    start = sim.ecm.v_set
    sim.requests.clear()

    sim.run(20 * SECOND)
    res = sim.requests.count(ButtonRequest.RES)
    self.assertGreaterEqual(res, 2)
    self.assertLessEqual(res, FollowParams.PENDING_BURST_TAPS + 3)

    sim.run(120 * SECOND)
    self.assertGreater(sim.ecm.v_set, start)
    self.assertLessEqual(sim.ecm.v_set, 80. + FollowParams.TARGET_DEADBAND_KPH)
    self.assertAlmostEqual(sim.ecm.v_set, some(sim.follow.v_set_kph), delta=0.3)

  def test_climb_never_passes_the_driver_speed(self):
    sim = Sim()
    sim.engage()
    sim.plan = FollowPlan(True, 120., 0.5, False)
    sim.run(120 * SECOND)
    self.assertNotIn(ButtonRequest.RES, sim.requests)

  def test_trim_down(self):
    sim = Sim()
    sim.engage()
    sim.plan = FollowPlan(True, 77., -0.1, False)
    sim.run(FollowParams.TARGET_HOLD_NANOS + SECOND)
    self.assertEqual([ButtonRequest.DECEL], sim.requests)
    sim.run(20 * SECOND)
    self.assertAlmostEqual(80. - STEP, sim.ecm.v_set)
    self.assertAlmostEqual(80. - STEP, some(sim.follow.v_set_kph), delta=0.3)

  def test_unheard_tap_is_retried(self):
    sim = Sim()
    sim.engage()
    sim.ecm.hear_taps = False
    sim.plan = FollowPlan(True, 77., -0.1, False)
    self.assertTrue(sim.run_until(lambda: sim.cruise.state == CruiseState.TAP, 5 * SECOND))
    sim.run(SECOND)
    sim.ecm.hear_taps = True
    sim.run(30 * SECOND)
    self.assertEqual(2, sim.requests.count(ButtonRequest.DECEL))
    self.assertAlmostEqual(80. - STEP, sim.ecm.v_set)

  def test_trimming_down_pauses_after_repeated_unheard_taps(self):
    sim = Sim()
    sim.engage()
    sim.ecm.hear_taps = False
    sim.plan = FollowPlan(True, 70., -0.1, False)
    sim.run(90 * SECOND)
    self.assertGreaterEqual(sim.requests.count(ButtonRequest.DECEL), FollowParams.TAP_DOWN_MISS_LIMIT)
    sim.requests.clear()
    sim.run(30 * SECOND)
    self.assertNotIn(ButtonRequest.DECEL, sim.requests)

  def test_no_trim_below_the_ecm_minimum(self):
    sim = Sim(FakeEcm(v_kph=45.), FollowPlan(True, 45., 0., False))
    sim.engage()
    sim.plan = FollowPlan(True, 40., -0.1, False)
    sim.run(30 * SECOND)
    self.assertNotIn(ButtonRequest.DECEL, sim.requests)

  def test_climb_gates(self):
    cases = {
      "downshift pending": dict(target_gear=5),
      "overrev": dict(rpm=FollowParams.RPM_SOFT + 200., turbine_rpm=FollowParams.RPM_SOFT + 200.),
      "pedal headroom": dict(pedal_pct=FollowParams.PEDAL_HOLD_PCT + 1.),
      "lockup slip": dict(turbine_rpm=1300.),
      "driver gas": dict(gas=True),
      "uphill": dict(long_accel=FollowParams.LONG_ACCEL_BIAS + 0.35),
    }
    for name, override in cases.items():
      with self.subTest(name):
        sim = Sim(FakeEcm(v_kph=80.), FollowPlan(True, 80., 0., False))
        sim.engage()
        sim.signals = normal_signals(**override)
        sim.run(2 * SECOND)
        sim.follow.v_user_kph = 90.
        sim.plan = FollowPlan(True, 90., 0.3, False)
        sim.requests.clear()
        sim.run(30 * SECOND)
        self.assertNotIn(ButtonRequest.RES, sim.requests)

  def test_climb_allowed_below_top_gear(self):
    for v_kph, signals in ((55., dict(gear=5, target_gear=5, rpm=2200., turbine_rpm=2200.)),
                           (80., dict(gear=5, target_gear=5, sport=True))):
      with self.subTest(v_kph=v_kph):
        sim = Sim(FakeEcm(v_kph=v_kph), FollowPlan(True, v_kph, 0., False))
        sim.engage()
        sim.follow.v_user_kph = v_kph + 10.
        sim.signals = normal_signals(**signals)
        sim.plan = FollowPlan(True, v_kph + 10., 0.3, False)
        sim.run(10 * SECOND)
        self.assertTrue(ButtonRequest.RES in sim.requests)

  def test_steady_windows_skip_descents_and_their_aftermath(self):
    sim = Sim()
    sim.engage()
    sim.signals.long_accel = FollowParams.LONG_ACCEL_BIAS - 0.3
    sim.run(3 * SECOND)
    sim.signals.long_accel = FollowParams.LONG_ACCEL_BIAS
    sim.follow.estimator.tap(sim.now, 1)
    sim.run(SetSpeedParams.DOWNHILL_RECOVERY_NANOS - 2 * SECOND)
    self.assertTrue(sim.follow.estimator.pending)

  def test_downshift_holds_climb(self):
    sim = Sim()
    sim.engage()
    sim.follow.v_user_kph = 90.
    sim.signals.gear = 5
    sim.step()
    sim.plan = FollowPlan(True, 90., 0.3, False)
    sim.run(FollowParams.DOWNSHIFT_HOLD_NANOS - SECOND)
    self.assertNotIn(ButtonRequest.RES, sim.requests)

  def _res_then(self, **after) -> 'Sim':
    sim = Sim()
    sim.engage()
    sim.follow.v_user_kph = 90.
    sim.plan = FollowPlan(True, 90., 0.3, False)
    assert sim.run_until(lambda: ButtonRequest.RES in sim.requests, 10 * SECOND)
    sim.run(CruiseParams.TAP_NANOS + CruiseParams.TAP_GAP_NANOS + FRAME)
    for k, v in after.items():
      setattr(sim.signals, k, v)
    return sim

  def test_kickdown_after_res_is_undone_once(self):
    sim = self._res_then(gear=5)
    sim.plan = FollowPlan(True, 80., 0., False)
    sim.run(3 * SECOND)
    self.assertEqual(1, sim.requests.count(ButtonRequest.DECEL))

  def test_rpm_spike_after_res_is_undone_once(self):
    sim = self._res_then(rpm=FollowParams.RPM_HARD + 100.)
    sim.plan = FollowPlan(True, 80., 0., False)
    sim.run(3 * SECOND)
    self.assertEqual(1, sim.requests.count(ButtonRequest.DECEL))

  def test_kickdown_that_did_not_overshoot_is_kept(self):
    sim = self._res_then(gear=5)
    sim.run(3 * SECOND)
    self.assertNotIn(ButtonRequest.DECEL, sim.requests)

  def test_brake_switches_main_off_and_disarms(self):
    sim = Sim()
    sim.engage()
    sim.brake = True
    sim.run(5 * SECOND)
    self.assertFalse(sim.ecm.main)
    self.assertFalse(sim.cruise.armed)
    self.assertIsNone(sim.follow.v_user_kph)
    self.assertEqual(CruiseState.IDLE, sim.cruise.state)

  def test_brake_while_coasting_switches_main_off(self):
    sim = Sim()
    sim.engage()
    sim.plan = FollowPlan(True, 70., -0.5, True)
    sim.run_until(lambda: sim.cruise.state == CruiseState.COAST, 2 * SECOND)
    sim.brake = True
    sim.run(5 * SECOND)
    self.assertFalse(sim.ecm.main)
    self.assertFalse(sim.cruise.armed)

  def test_unexpected_drop_pauses_then_recaptures(self):
    sim = Sim()
    sim.engage()
    sim.ecm.drop()
    sim.step()
    self.assertEqual(CruiseState.COAST, sim.cruise.state)
    self.assertTrue(sim.run_until(lambda: sim.ecm.engaged, 5 * SECOND))
    self.assertTrue(sim.ecm.main)

  def test_repeated_unexpected_drops_switch_off(self):
    sim = Sim()
    sim.engage()
    for _ in range(CruiseParams.UNEXPECTED_DROP_LIMIT):
      sim.ecm.drop()
      sim.step()
      sim.until_engaged(5 * SECOND)
    sim.run(5 * SECOND)
    self.assertFalse(sim.cruise.armed)
    self.assertFalse(sim.ecm.main)

  def test_unexpected_drop_without_a_plan_switches_off(self):
    sim = Sim()
    sim.engage()
    sim.plan = FollowPlan()
    sim.ecm.drop()
    sim.run(5 * SECOND)
    self.assertFalse(sim.ecm.main)

  def test_no_plan_no_buttons(self):
    sim = Sim(plan=FollowPlan())
    sim.engage()
    sim.requests.clear()
    sim.run(60 * SECOND)
    self.assertEqual([], sim.requests)
    self.assertTrue(sim.ecm.engaged)

  def test_invalid_signals_only_allow_coasting(self):
    sim = Sim()
    sim.engage()
    sim.signals = FollowSignals()
    sim.plan = FollowPlan(True, 90., 0.3, False)
    sim.follow.v_user_kph = 90.
    sim.run(10 * SECOND)
    self.assertEqual([], sim.requests)

    sim.plan = FollowPlan(True, 70., -0.5, True)
    sim.run(10 * SECOND)
    self.assertEqual([ButtonRequest.CANCEL], sim.requests)
    # without brake information the coast cannot be resumed safely, so the session ends
    self.assertFalse(sim.cruise.armed)
    self.assertFalse(sim.ecm.main)

  def test_cancel_falls_back_to_main_off_and_recaptures_through_main(self):
    sim = Sim(FakeEcm(cancel_works=False))
    sim.engage()
    sim.plan = FollowPlan(True, 70., -0.5, True)
    self.assertTrue(sim.run_until(lambda: not sim.ecm.engaged, 10 * SECOND))
    self.assertFalse(sim.ecm.main)
    self.assertFalse(sim.cruise.cancel_unsupported)
    self.assertTrue(sim.cruise.armed)

    sim.plan = FollowPlan(True, 70., 0., False)
    self.assertTrue(sim.run_until(lambda: sim.ecm.engaged, 15 * SECOND))
    self.assertAlmostEqual(80., some(sim.follow.v_user_kph))

  def _coast_and_recapture(self, sim):
    sim.plan = FollowPlan(True, 70., -0.5, True)
    sim.run_until(lambda: sim.cruise.state == CruiseState.COAST, 15 * SECOND)
    sim.plan = FollowPlan(True, 80., 0., False)
    sim.until_engaged(20 * SECOND)
    sim.ecm.v = 80.

  def test_cancel_given_up_only_after_separate_failures(self):
    sim = Sim(FakeEcm(cancel_works=False))
    sim.engage()
    for _ in range(CruiseParams.CANCEL_FAILED_EPISODES - 1):
      self._coast_and_recapture(sim)
      sim.run(CruiseParams.CANCEL_EPISODE_SPACING_NANOS)
      self.assertFalse(sim.cruise.cancel_unsupported)
    self._coast_and_recapture(sim)
    self.assertTrue(sim.cruise.cancel_unsupported)

    sim.run(CruiseParams.CANCEL_RETRY_NANOS)
    self.assertFalse(sim.cruise.cancel_unsupported)

  def test_answered_cancel_forgets_earlier_failures(self):
    sim = Sim(FakeEcm(cancel_works=False))
    sim.engage()
    self._coast_and_recapture(sim)
    sim.ecm.cancel_works = True
    sim.run(CruiseParams.CANCEL_EPISODE_SPACING_NANOS)
    self._coast_and_recapture(sim)
    self.assertEqual(0, len(sim.cruise._cancel_failures))

  def test_rejected_recapture_is_retried_later(self):
    sim = Sim(FakeEcm(v_kph=50., set_min_kph=45.))
    sim.plan = FollowPlan(True, 50., 0., False)
    sim.engage()
    sim.plan = FollowPlan(True, 40., -0.5, True)
    sim.run_until(lambda: sim.cruise.state == CruiseState.COAST, 2 * SECOND)
    sim.run_until(lambda: sim.ecm.v < 43., 10 * SECOND)
    sim.ecm.v = 43.
    sim.plan = FollowPlan(True, 43., 0., False)

    presses = []
    for _ in range(int(15 * SECOND / FRAME)):
      sim.ecm.v = 43.
      was = sim.cruise.state
      sim.step()
      if was != CruiseState.PRESS_SET and sim.cruise.state == CruiseState.PRESS_SET:
        presses.append(sim.now)
    self.assertGreaterEqual(len(presses), 2)
    for a, b in zip(presses, presses[1:], strict=False):
      self.assertGreaterEqual(b - a, FollowParams.RECAPTURE_RETRY_NANOS)
    self.assertTrue(sim.cruise.armed)
    self.assertFalse(sim.ecm.engaged)

  def test_slow_coast_ends_following(self):
    sim = Sim()
    sim.engage()
    sim.plan = FollowPlan(True, 25., -0.5, True)
    sim.run_until(lambda: sim.cruise.state == CruiseState.COAST, 2 * SECOND)
    sim.plan = FollowPlan(True, 25., 0., False)
    sim.ecm.v = FollowParams.COAST_DISARM_SPEED_KPH - 1.
    sim.run(5 * SECOND)
    self.assertFalse(sim.cruise.armed)
    self.assertFalse(sim.ecm.main)

  def test_resync_cancels_and_sets_again(self):
    sim = Sim()
    sim.engage()
    sim.follow.estimator.needs_resync = True
    self.assertTrue(sim.run_until(lambda: sim.cruise.state == CruiseState.COAST, 2 * SECOND))
    self.assertTrue(sim.until_engaged(3 * SECOND))
    self.assertFalse(sim.follow.estimator.needs_resync)

  def test_long_press_during_a_tap_switches_off(self):
    sim = Sim()
    sim.engage()
    sim.plan = FollowPlan(True, 77., -0.1, False)
    sim.run_until(lambda: sim.cruise.state == CruiseState.TAP, 5 * SECOND)
    sim.step(long_press=True)
    sim.run(5 * SECOND)
    self.assertFalse(sim.ecm.main)
    self.assertFalse(sim.cruise.armed)

  def _lose_preconditions(self, sim, nanos):
    end = sim.now + nanos
    while sim.now < end:
      sim.now += FRAME
      sim.cruise.update(sim.now, False, True, sim.ecm.main, sim.ecm.engaged, sim.ecm.v, False, True, brake=False,
                        follow_active=True)

  def test_precondition_blip_keeps_the_session(self):
    sim = Sim()
    sim.engage()
    sim.plan = FollowPlan(True, 70., -0.5, True)
    sim.run_until(lambda: sim.cruise.state == CruiseState.COAST, 2 * SECOND)
    sim.ecm.v = 85.
    self._lose_preconditions(sim, CruiseParams.PRECOND_GRACE_NANOS - 2 * FRAME)
    self.assertTrue(sim.cruise.armed)
    sim.step()
    self.assertEqual(CruiseState.COAST, sim.cruise.state)
    self.assertAlmostEqual(80., some(sim.follow.v_user_kph))

  def test_lasting_precondition_loss_ends_the_session(self):
    sim = Sim()
    sim.engage()
    sim.plan = FollowPlan(True, 70., -0.5, True)
    sim.run_until(lambda: sim.cruise.state == CruiseState.COAST, 2 * SECOND)
    self._lose_preconditions(sim, CruiseParams.PRECOND_GRACE_NANOS + FRAME)
    self.assertFalse(sim.cruise.armed)
    sim.plan = FollowPlan(True, 80., 0., False)
    sim.run(10 * SECOND)
    self.assertFalse(sim.ecm.main)
    self.assertNotIn(ButtonRequest.SET, sim.requests)

  def _start_recapture(self, sim):
    sim.plan = FollowPlan(True, 70., -0.5, True)
    sim.run_until(lambda: sim.cruise.state == CruiseState.COAST, 2 * SECOND)
    sim.plan = FollowPlan(True, 70., 0., False)
    self.assertTrue(sim.run_until(lambda: sim.cruise.state == CruiseState.PRESS_SET, 5 * SECOND))

  def test_brake_during_recapture_ends_the_session(self):
    sim = Sim(FakeEcm(set_min_kph=200.))
    sim.engage_at(80.)
    self._start_recapture(sim)
    sim.brake = True
    sim.run(5 * FRAME)
    sim.brake = False
    sim.run(10 * SECOND)
    self.assertFalse(sim.cruise.armed)
    self.assertFalse(sim.ecm.main)
    self.assertFalse(sim.ecm.engaged)

  def test_long_press_during_recapture_ends_the_session(self):
    sim = Sim()
    sim.engage()
    self._start_recapture(sim)
    sim.step(long_press=True)
    sim.run(10 * SECOND)
    self.assertFalse(sim.cruise.armed)
    self.assertFalse(sim.ecm.main)

  def test_long_press_during_main_recapture_ends_the_session(self):
    sim = Sim(FakeEcm(cancel_works=False))
    sim.engage()
    sim.plan = FollowPlan(True, 70., -0.5, True)
    sim.run_until(lambda: not sim.ecm.main, 10 * SECOND)
    sim.plan = FollowPlan(True, 70., 0., False)
    self.assertTrue(sim.run_until(lambda: sim.cruise.state == CruiseState.PRESS_MAIN_ON, 5 * SECOND))
    sim.step(long_press=True)
    sim.run(10 * SECOND)
    self.assertFalse(sim.cruise.armed)
    self.assertFalse(sim.ecm.main)

  def test_main_recapture_out_of_range_goes_back_to_coasting(self):
    sim = Sim(FakeEcm(cancel_works=False))
    sim.engage()
    sim.plan = FollowPlan(True, 70., -0.5, True)
    sim.run_until(lambda: not sim.ecm.main, 10 * SECOND)
    sim.plan = FollowPlan(True, 70., 0., False)
    self.assertTrue(sim.run_until(lambda: sim.cruise.state == CruiseState.WAIT_SET_READY, 10 * SECOND))
    sim.ecm.v = CruiseParams.SET_SPEED_MIN_KPH - 1.
    sim.step()
    self.assertEqual(CruiseState.COAST, sim.cruise.state)
    self.assertTrue(sim.cruise.armed)

  def test_init_without_a_session_switches_main_off(self):
    sim = Sim()
    sim.ecm.main = True
    sim.run(5 * SECOND)
    self.assertFalse(sim.ecm.main)
    self.assertNotIn(ButtonRequest.SET, sim.requests)

  def test_unanswered_coast_keeps_following_and_retries(self):
    sim = Sim()
    sim.engage()
    sim.ecm.deaf = True
    sim.plan = FollowPlan(True, 70., -0.5, True)
    self.assertTrue(sim.run_until(lambda: sim.cruise.state == CruiseState.FAULT, 30 * SECOND))
    self.assertTrue(sim.ecm.engaged)
    self.assertTrue(sim.cruise.armed)
    self.assertIsNotNone(sim.follow.v_user_kph)

    sim.ecm.deaf = False
    sim.run(CruiseParams.FAULT_RETRY_NANOS + 5 * SECOND)
    self.assertFalse(sim.ecm.main)
    self.assertTrue(sim.cruise.armed)
    self.assertEqual(CruiseState.COAST, sim.cruise.state)

  def test_unanswered_switch_off_is_retried(self):
    sim = Sim()
    sim.engage()
    sim.ecm.deaf = True
    sim.step(long_press=True)
    self.assertTrue(sim.run_until(lambda: sim.cruise.state == CruiseState.FAULT, 30 * SECOND))
    self.assertTrue(sim.ecm.engaged)
    self.assertFalse(sim.cruise.armed)
    sim.ecm.deaf = False
    sim.run(CruiseParams.FAULT_RETRY_NANOS + 5 * SECOND)
    self.assertFalse(sim.ecm.main)

  def test_slow_coast_waits_while_the_lead_still_needs_coasting(self):
    sim = Sim()
    sim.engage()
    sim.plan = FollowPlan(True, 20., -0.5, True)
    sim.run_until(lambda: sim.cruise.state == CruiseState.COAST, 2 * SECOND)
    sim.ecm.v = FollowParams.COAST_DISARM_SPEED_KPH - 1.
    sim.run(5 * SECOND)
    self.assertTrue(sim.cruise.armed)
    sim.plan = FollowPlan(True, 20., 0., False)
    sim.run(5 * SECOND)
    self.assertFalse(sim.cruise.armed)


class TestFollowPlan(unittest.TestCase):
  def test_no_command_means_no_plan(self):
    self.assertFalse(FollowPlan.from_command(None).valid)
    self.assertFalse(FollowPlan.from_command(FollowCommand(0., 0., False)).valid)
    self.assertFalse(FollowPlan.from_command(FollowCommand(float('nan'), 0., False)).valid)

    plan = FollowPlan.from_command(FollowCommand(20., -0.4, True))
    self.assertTrue(plan.valid)
    self.assertAlmostEqual(cluster_from_wheel(72.), plan.v_target_kph)
    self.assertAlmostEqual(-0.4, plan.a_target, places=5)
    self.assertTrue(plan.coast)

  def test_speed_scales_round_trip(self):
    self.assertAlmostEqual(63.2, cluster_from_wheel(wheel_from_cluster(63.2)))


if __name__ == "__main__":
  unittest.main()
