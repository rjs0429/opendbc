from opendbc.car.can_definitions import CanData
from opendbc.car.avante_md.values import CanBus

VSM1 = 0x164
VSM1_ACTIVE_CTR_MODE = 2
VSM1_STALE_NANOS = 50_000_000


def vsm1_checksum(dat: bytes | bytearray) -> int:
  checksum = 0
  for b in dat[:7]:
    checksum ^= b
  return checksum


def vsm1_checksum_valid(raw: bytes) -> bool:
  return len(raw) == 8 and vsm1_checksum(raw) == raw[7]


def vsm1_is_normal_state(raw: bytes) -> bool:
  return (vsm1_checksum_valid(raw) and
          raw[0] == 0x00 and
          raw[1] == 0x08 and
          raw[2] == 0x00 and
          raw[3] == 0x00 and
          raw[4] == 0x00 and
          raw[5] == 0x00 and
          (raw[6] & 0xF0) == 0x00)


def create_vsm1(raw_vsm1: bytes, apply_torque: int, enabled: bool) -> CanData:
  dat = bytearray(raw_vsm1[:8])

  raw_torque = max(0, min(4095, apply_torque + 2048))
  act = 1 if enabled else 0
  ctr_mode = VSM1_ACTIVE_CTR_MODE if enabled else 0

  dat[0] = raw_torque & 0xff
  dat[1] = ((raw_torque >> 8) & 0x0f) | (act << 4) | (ctr_mode << 5)
  dat[2] = 0
  dat[7] = vsm1_checksum(dat)

  return CanData(VSM1, bytes(dat), CanBus.EPS)
