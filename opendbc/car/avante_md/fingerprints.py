""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.avante_md.values import CAR
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu

FINGERPRINTS = {
  CAR.AVANTE_MD_2012: [{
    # Repeated in logs but not yet modeled in the DBC.
    0x2: 8,
    0x2C0: 8,
    0x38B: 8,
    0x5A0: 8,

    # DBC-backed frames observed in Avante MD logs.
    0x80: 8,
    0x81: 8,
    0xA0: 8,
    0xA1: 8,
    0x130: 8,
    0x131: 8,
    0x140: 8,
    0x153: 8,
    0x164: 8,
    0x165: 8,
    0x18F: 8,
    0x1F1: 8,
    0x220: 8,
    0x260: 8,
    0x2A0: 8,
    0x2B0: 5,
    0x316: 8,
    0x329: 8,
    0x350: 8,
    0x370: 8,
    0x382: 8,
    0x43F: 8,
    0x440: 8,
    0x4B1: 8,
    0x4F0: 8,
    0x545: 8,
    0x59B: 8,
    0x5E4: 3,
    0x690: 8,
  }],
}

FW_VERSIONS = {
  CAR.AVANTE_MD_2012: {
    (Ecu.engine, 0x7e0, None): [
      b'GIMD-BD-6QF16C00',
    ],
    (Ecu.eps, 0x7d4, None): [
      b'LMDPS',
      bytes.fromhex('4c4d445053084d5253430000000011010103014d44013a00ffffffffffffffffffffffffffffffffffffffffffffffff'),
    ],
  },
}
