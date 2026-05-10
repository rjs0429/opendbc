""" AUTO-FORMATTED USING opendbc/car/debug/format_fingerprints.py, EDIT STRUCTURE THERE."""
from opendbc.car.avante_md.values import CAR
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu

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
