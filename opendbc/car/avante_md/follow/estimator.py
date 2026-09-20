"""Stock cruise set speed, estimated on the cluster speed scale.

The ECM never reports its set speed. SET captures the current speed, and every RES or SET- tap moves it by
one step, but a tap can go unheard. Taps therefore build a distribution over the net number of steps that
registered, and the speed the ECM settles at on a steady road decides it. The caller sends no further tap
while a decision is pending.
"""
import math
from collections import deque

from opendbc.car.avante_md.values import FollowParams, SetSpeedParams as P

TAP_PRIOR = {1: P.PRIOR_ONE_STEP, 0: P.PRIOR_MISSED, 2: P.PRIOR_TWO_STEPS}


class SetSpeedEstimator:
  def __init__(self):
    self.valid = False
    self.v_set = 0.
    self.needs_resync = False
    self.last_commit_steps: int | None = None
    self._pending: dict[int, float] | None = None
    self._pending_base = 0.
    self._pending_nanos = 0
    self._settle_until = 0
    self._mismatches = 0
    self._agreements = 0
    self._window: deque[tuple[int, float, float]] = deque()

  @property
  def pending(self) -> bool:
    return self._pending is not None

  @property
  def resolved(self) -> bool:
    return self.valid and not self.pending and not self.needs_resync

  def reset(self) -> None:
    self.__init__()

  def capture(self, now: int, v_cluster: float) -> None:
    self.valid = True
    self.v_set = v_cluster
    self.needs_resync = False
    self.last_commit_steps = None
    self._pending = None
    self._mismatches = 0
    self._agreements = 0
    self._settle(now, P.SETTLE_AFTER_CAPTURE_NANOS)

  def tap(self, now: int, direction: int) -> None:
    if not self.valid:
      return
    step = 1 if direction > 0 else -1
    prior = self._pending
    if prior is None:
      prior = {0: 1.}
      self._pending_base = self.v_set
    pending: dict[int, float] = {}
    for net, p in prior.items():
      for k, q in TAP_PRIOR.items():
        pending[net + step * k] = pending.get(net + step * k, 0.) + p * q
    self._pending = pending
    self._pending_nanos = now
    self._mismatches = 0
    self._settle(now, P.SETTLE_AFTER_TAP_NANOS)

  def update(self, now: int, v_cluster: float, v_wheel: float, steady: bool) -> None:
    if not self.valid:
      return

    if self._pending is not None and now - self._pending_nanos >= P.VERIFY_TIMEOUT_NANOS:
      self._commit(self._likeliest())
      self.needs_resync = True

    if not steady or now < self._settle_until:
      self._window.clear()
      return

    self._window.append((now, v_cluster, v_wheel))
    if now - self._window[0][0] < P.WINDOW_NANOS:
      return

    wheel = [w for _, _, w in self._window]
    if max(wheel) - min(wheel) >= P.STEADY_WHEEL_RANGE_KPH:
      self._window.popleft()
      return

    clusters = sorted(c for _, c, _ in self._window)
    z = clusters[len(clusters) // 2]
    self._window.clear()
    self._observe(z)

  def _observe(self, z: float) -> None:
    if self._pending is not None:
      if min(abs(z - self._candidate(k)) for k in self._pending) > P.AMBIGUOUS_KPH:
        self._mismatches += 1
        if self._mismatches >= P.MISMATCH_LIMIT:
          self._commit(self._likeliest())
          self.needs_resync = True
        return
      self._mismatches = 0
      posterior = {k: prior * self._likelihood(z, self._candidate(k)) for k, prior in self._pending.items()}
      total = sum(posterior.values())
      self._pending = {k: p / total for k, p in posterior.items()}
      best = self._likeliest()
      if self._pending[best] >= P.CONFIRM_PROB:
        self._commit(best)
      return

    residual = z - self.v_set
    if abs(residual) <= FollowParams.TAP_STEP_KPH / 2:
      self.v_set += P.TRACK_GAIN * residual
      self._mismatches = 0
      self._agreements += 1
      if self.needs_resync and self._agreements >= P.MISMATCH_LIMIT:
        self.needs_resync = False
    else:
      self._agreements = 0
      self._mismatches += 1
      if self._mismatches >= P.MISMATCH_LIMIT:
        self.needs_resync = True

  def _likeliest(self) -> int:
    assert self._pending is not None
    return max(self._pending, key=self._pending.__getitem__)

  def _candidate(self, net_steps: int) -> float:
    return self._pending_base + net_steps * FollowParams.TAP_STEP_KPH

  def _commit(self, net_steps: int) -> None:
    self.v_set = self._candidate(net_steps)
    self.last_commit_steps = net_steps
    self._pending = None
    self._mismatches = 0
    self._agreements = 0

  def _settle(self, now: int, nanos: int) -> None:
    self._settle_until = now + nanos
    self._window.clear()

  @staticmethod
  def _likelihood(z: float, mean: float) -> float:
    return math.exp(-0.5 * ((z - mean) / P.OBS_SIGMA_KPH) ** 2)
