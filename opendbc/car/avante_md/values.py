from dataclasses import dataclass, field

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts, SupportType
from opendbc.car.fw_query_definitions import FwQueryConfig, Request
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu


class CanBus:
  VEHICLE = 0
  EPS = 2


AVANTE_MD_DBC = "hyundai_avante_2012"


class CarControllerParams:
  # VSM1 command safety allows up to 8.0 Nm; controller is capped at 8.0 Nm (STEER_MAX=800).
  STEER_COMMAND_SIGN = 1  # VSM1 command torque is left-positive.
  STEER_MAX = 800
  STEER_DELTA_UP = 500
  STEER_DELTA_DOWN = 500
  STEER_DRIVER_ALLOWANCE = 200
  STEER_DRIVER_MULTIPLIER = 1
  STEER_DRIVER_FACTOR = 1
  STEER_STEP = 1  # 100 Hz

  def __init__(self, CP):
    pass


@dataclass
class AvanteMdCarDocs(CarDocs):
  package: str = "Custom VSM1 torque port"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))
  support_type: SupportType = SupportType.CUSTOM
  support_link: str | None = None


@dataclass
class AvanteMdPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: AVANTE_MD_DBC,
    Bus.adas: AVANTE_MD_DBC,
  })


class CAR(Platforms):
  AVANTE_MD_2012 = AvanteMdPlatformConfig(
    [AvanteMdCarDocs("Hyundai Avante 2012")],
    CarSpecs(mass=1525, wheelbase=2.70, steerRatio=14.65, tireStiffnessFactor=0.385),
  )


AVANTE_MD_ENGINE_VERSION_REQUEST = b'\x21\x91'
AVANTE_MD_ENGINE_VERSION_RESPONSE = b'\x61\x91'
AVANTE_MD_EPS_ID_REQUEST = b'\x21\x94'
AVANTE_MD_EPS_ID_RESPONSE = b'\x61\x94'
AVANTE_MD_EPS_VERSION_REQUEST = b'\x21\x98'
AVANTE_MD_EPS_VERSION_RESPONSE = b'\x61\x98'

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [AVANTE_MD_ENGINE_VERSION_REQUEST],
      [AVANTE_MD_ENGINE_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
    ),
    Request(
      [AVANTE_MD_EPS_ID_REQUEST],
      [AVANTE_MD_EPS_ID_RESPONSE],
      whitelist_ecus=[Ecu.eps],
    ),
    Request(
      [AVANTE_MD_EPS_VERSION_REQUEST],
      [AVANTE_MD_EPS_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.eps],
    ),
  ],
)

DBC = CAR.create_dbc_map()
