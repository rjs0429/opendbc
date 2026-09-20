"""Powertrain signals the follow logic reads straight from raw frames.

They are decoded outside the CAN parsers on purpose: a parser marks the whole bus invalid when one of its
messages goes quiet, and that validity also gates steering.
"""
from dataclasses import dataclass

from opendbc.car.avante_md.values import FollowParams

EMS_DCT1 = 0x080
EMS_DCT2 = 0x081
ESP2 = 0x220
EMS6 = 0x260
EMS1 = 0x316
TCU3 = 0x370
TCU1 = 0x43F
TCU2 = 0x440

BRAKE_ACT_RELEASED = 1
TCU1_SPORT = 8

WATCHED = (EMS_DCT1, EMS_DCT2, ESP2, EMS6, EMS1, TCU3, TCU1, TCU2)


@dataclass
class FollowSignals:
  valid: bool = False
  brake: bool = False
  gas: bool = False
  pedal_pct: float = 0.
  rpm: float = 0.
  turbine_rpm: float = 0.
  gear: int = 0
  target_gear: int = 0
  sport: bool = False
  long_accel: float = 0.


class FollowSignalDecoder:
  def __init__(self):
    self._raw: dict[int, bytes] = {}
    self._nanos: dict[int, int] = {}
    self._pressure_frames = 0

  def update_frame(self, addr: int, dat: bytes, nanos: int) -> None:
    if addr in WATCHED:
      self._raw[addr] = dat
      self._nanos[addr] = nanos
      if addr == ESP2:
        pressed = self._cyl_pres(dat) > FollowParams.BRAKE_PRESSURE_BAR
        self._pressure_frames = self._pressure_frames + 1 if pressed else 0

  @staticmethod
  def _cyl_pres(esp2: bytes) -> float:
    return (esp2[4] | ((esp2[5] & 0x0F) << 8)) * 0.1

  def decode(self, now_nanos: int) -> FollowSignals:
    fresh = all(addr in self._nanos and now_nanos - self._nanos[addr] <= FollowParams.SIGNAL_STALE_NANOS
                for addr in WATCHED)
    if not fresh:
      return FollowSignals()

    ems_dct1 = self._raw[EMS_DCT1]
    ems_dct2 = self._raw[EMS_DCT2]
    esp2 = self._raw[ESP2]
    ems6 = self._raw[EMS6]
    ems1 = self._raw[EMS1]
    tcu3 = self._raw[TCU3]
    tcu1 = self._raw[TCU1]
    tcu2 = self._raw[TCU2]

    brake_act = (ems_dct2[0] >> 6) & 0x3
    # A light touch cancels the stock cruise while BRAKE_ACT still reads 0, so pressure counts too, above
    # the level the reading spikes to on its own.
    pressure = self._pressure_frames >= FollowParams.BRAKE_PRESSURE_FRAMES

    return FollowSignals(
      valid=True,
      brake=brake_act != BRAKE_ACT_RELEASED or pressure,
      gas=((ems6[7] >> 6) & 0x3) != 0,
      pedal_pct=ems_dct1[0] * 0.3906,
      rpm=(ems1[2] | (ems1[3] << 8)) * 0.25,
      turbine_rpm=(tcu2[5] | (tcu2[6] << 8)) * 0.25,
      gear=tcu2[1] & 0xF,
      target_gear=(tcu3[2] >> 4) & 0xF,
      sport=(tcu1[1] & 0xF) == TCU1_SPORT,
      long_accel=(esp2[2] | ((esp2[3] & 0x07) << 8)) * 0.01 - 10.23,
    )
