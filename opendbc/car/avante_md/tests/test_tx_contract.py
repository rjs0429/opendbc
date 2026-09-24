#!/usr/bin/env python3
"""Every cruise button frame the car controller sends must pass panda safety, or the ECM never hears it.

Reruns the follow scenarios while feeding panda safety the same vehicle state and checking each CLU1 the
controller would transmit, built by the port's own create_clu1.
"""
import unittest
from unittest import mock

from opendbc.car.avante_md import avantecan
from opendbc.car.avante_md.follow.policy import wheel_from_cluster
from opendbc.car.avante_md.tests import test_follow as follow
from opendbc.car.common.conversions import Conversions as CV
from opendbc.safety.tests import common
from opendbc.safety.tests import test_avante_md as panda

FRAME_US = follow.FRAME // 1000
EMS6 = 0x260
CLU1 = 0x4F0


class SafetyCheckedSim(follow.Sim):
  # frames between messages; 1 means every 10 ms control frame
  TCS5_EVERY, TCS5_COPIES, CLU1_EVERY, CLU2_EVERY = 1, 6, 1, 1

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.panda = panda.TestAvanteMdSafety("setUp")
    self.panda.setUp()
    self.frame = 0
    self.t_us = 0
    self.payload: bytes | None = None
    self.sent = 0
    self.rejected: list[tuple[int, int, int]] = []

  def _vehicle_rx(self) -> None:
    p = self.panda
    self.frame += 1
    self.t_us += FRAME_US
    p.safety.set_timer(self.t_us)
    ems6 = bytearray(8)
    ems6[3] = (0x02 if self.ecm.main else 0) | (0x04 if self.ecm.engaged else 0)
    p.safety.safety_rx_hook(common.make_msg(0, EMS6, 8, bytes(ems6)))
    if self.frame % self.TCS5_EVERY == 0:
      speed = wheel_from_cluster(self.ecm.v) * CV.KPH_TO_MS
      for _ in range(self.TCS5_COPIES):
        p.safety.safety_rx_hook(p._tcs5_msg(speed_kph=speed))
    p.safety.safety_rx_hook(p._tcu1_msg(gear_disp=5))
    p.safety.safety_rx_hook(p._tcu2_msg(gear=1))
    if self.frame % self.CLU2_EVERY == 0:
      p.safety.safety_rx_hook(p._clu2_msg())
    if self.payload is None or self.frame % self.CLU1_EVERY == 0:
      self.payload = bytes(p._clu1_payload())
      p.safety.safety_rx_hook(common.make_msg(0, CLU1, 8, self.payload))

  def step(self, long_press=False):
    super().step(long_press)
    self._vehicle_rx()
    if self.cruise.transmitting:
      self.sent += 1
      tx = avantecan.create_clu1(self.payload, self.cruise.sw_state, self.cruise.sw_main)
      if not self.panda.safety.safety_tx_hook(common.make_msg(tx.src, tx.address, len(tx.dat), tx.dat)):
        self.rejected.append((self.now, self.cruise.sw_state, self.cruise.sw_main))


class BusRateSim(SafetyCheckedSim):
  """Messages at the rates measured on the car: TCS5 and CLU1 at 50 Hz, CLU2 at 10 Hz."""
  TCS5_EVERY, TCS5_COPIES, CLU1_EVERY, CLU2_EVERY = 2, 1, 2, 10


class TestTxContract(follow.TestFollow):
  def setUp(self):
    self.sims: list[SafetyCheckedSim] = []
    sims = self.sims

    class TrackedSim(SafetyCheckedSim):
      def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        sims.append(self)

    patcher = mock.patch.object(follow, "Sim", TrackedSim)
    patcher.start()
    self.addCleanup(patcher.stop)

  def tearDown(self):
    for sim in self.sims:
      self.assertEqual([], sim.rejected, "panda safety rejected cruise frames the controller sent (nanos, sw_state, sw_main)")

  def test_engagement_frames_reach_the_ecm(self):
    sim = follow.Sim()
    sim.engage()
    self.assertTrue(sim.ecm.engaged)
    self.assertGreater(sim.sent, 0)


class TestTxContractAtBusRates(unittest.TestCase):
  def test_climb_taps_reach_the_ecm(self):
    rejected = self._rejected("test_climb_taps_one_step_at_a_time_up_to_the_driver_speed")
    # Known defect: panda raises its tracked set speed by RES_STEP_SPEED per RES while the ECM moves less per tap, so
    # the RES gate cuts RES taps. Once fixed this fails: assert rejected == [] instead.
    self.assertTrue(rejected, "the RES gate no longer cuts taps: make this test require no rejections")
    self.assertEqual({avantecan.CLU1_SW_RES}, {sw_state for _, sw_state, _ in rejected})

  @staticmethod
  def _rejected(scenario: str) -> list[tuple[int, int, int]]:
    sims: list[SafetyCheckedSim] = []

    def make(*args, **kwargs):
      sims.append(BusRateSim(*args, **kwargs))
      return sims[-1]

    with mock.patch.object(follow, "Sim", make):
      try:
        getattr(follow.TestFollow(scenario), scenario)()
      except unittest.SkipTest as e:
        raise AssertionError(f"the bus-rate contract replays {scenario}, which is skipped") from e
    return [r for sim in sims for r in sim.rejected]


if __name__ == "__main__":
  unittest.main()
