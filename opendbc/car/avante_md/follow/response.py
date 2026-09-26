"""Whether the engine answered a cruise tap, read from its throttle demand in the second after the press.

Silence counts only when the demand was steady before the press. An answer also counts while the demand was
drifting the other way, which could not have produced it.
"""
import statistics
from collections import deque

from opendbc.car.avante_md.follow.signals import FollowSignals
from opendbc.car.avante_md.values import TapResponseParams as P


class TapResponse:
  def __init__(self):
    self._history: deque[tuple[int, float]] = deque()
    self._press: int | None = None
    self._direction = 0
    self._baseline: float | None = None
    self._steady = False
    self._gear = 0

  @property
  def busy(self) -> bool:
    return self._press is not None

  def reset(self) -> None:
    self.__init__()

  def start(self, press_nanos: int, direction: int) -> None:
    before = [pv for t, pv in self._history if press_nanos - P.BASELINE_NANOS <= t < press_nanos]
    self._press = press_nanos
    self._direction = 1 if direction > 0 else -1
    self._baseline = None
    self._steady = False
    if len(before) < P.BASELINE_MIN_SAMPLES:
      return
    edge = len(before) // 5
    base = statistics.fmean(before[-edge:])
    self._steady = statistics.pstdev(before) <= P.BASELINE_STD_PCT
    against = self._direction * (base - statistics.fmean(before[:edge])) < 0.
    room = (base <= P.RISE_CEILING_PCT) if direction > 0 else (base >= P.DROP_FLOOR_PCT)
    if room and (self._steady or against):
      self._baseline = base

  def update(self, now: int, signals: FollowSignals, lamp_set: bool) -> bool | None:
    """Returns True or False once the open tap's window has passed with a clear answer or clear silence."""
    steady = signals.valid and lamp_set and not signals.gas and not signals.brake
    shifted = signals.valid and self._gear != 0 and signals.gear != self._gear
    self._gear = signals.gear if signals.valid else 0
    if steady and not shifted:
      self._history.append((now, signals.pedal_pct))
    else:
      self._history.clear()
      self._baseline = None
    while self._history and now - self._history[0][0] > P.HISTORY_NANOS:
      self._history.popleft()

    if self._press is None or now - self._press < P.WINDOW_NANOS:
      return None
    base, press = self._baseline, self._press
    self._press = None
    window = [pv - base for t, pv in self._history if t >= press] if base is not None else []
    if not window:
      return None
    if max(self._direction * d for d in window) >= P.ANSWER_PCT:
      return True
    if self._steady and max(abs(d) for d in window) < P.SILENT_PCT:
      return False
    return None
