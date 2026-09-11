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


class CruiseParams:
  """Stock cruise control driven by ECO switch long press.

  Button semantics measured on the car: CruiseSwMain 0->1 is a latch toggle, SwState 2 is SET,
  and holding SET after engagement is a deceleration request, so every press is lamp-closed-loop
  with a hard cap.
  """
  # ECO switch hold that toggles cruise. Must exceed MomentaryButtonDoubleClick.DOUBLE_CLICK_INTERVAL
  # so a cruise hold can never also read as the lateral-control double click.
  HOLD_NANOS = 3_100_000_000

  MAIN_PRESS_MAX_NANOS = 1_500_000_000
  SET_PRESS_MAX_NANOS = 400_000_000
  SETTLE_NANOS = 600_000_000
  GAP_NANOS = 300_000_000
  MAX_PRESS_ATTEMPTS = 3

  SET_SPEED_MIN_KPH = 40.
  SET_SPEED_MAX_KPH = 120.
  SET_READY_DWELL_NANOS = 200_000_000
  SET_READY_TIMEOUT_NANOS = 60_000_000_000


class CarControllerParams:
  # VSM1 command safety allows up to 8.0 Nm; controller is capped at 8.0 Nm (STEER_MAX=800).
  STEER_COMMAND_SIGN = 1  # VSM1 command torque is left-positive.
  STEER_MAX = 800
  STEER_DELTA_UP = 20
  STEER_DELTA_DOWN = 20
  STEER_DRIVER_ALLOWANCE = 150
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
