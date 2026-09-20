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
  HOLD_NANOS = 2_500_000_000

  # Every press is a tap that is released as soon as its lamp answers, then a settle window before
  # the next attempt. MAIN settles longest: it is an edge toggle, so tapping again while its lamp is
  # still catching up would toggle it straight back off. Presses are long because the cluster keeps
  # sending its own released frame between ours, so the ECM only samples the button pressed part of
  # the time.
  MAIN_PRESS_NANOS = 1_500_000_000
  MAIN_SETTLE_NANOS = 1_000_000_000
  # SET stays shorter: holding it once cruise engages is a deceleration request.
  SET_PRESS_NANOS = 800_000_000
  SET_SETTLE_NANOS = 600_000_000
  GAP_NANOS = 300_000_000
  MAX_PRESS_ATTEMPTS = 6

  SET_SPEED_MIN_KPH = 40.
  SET_SPEED_MAX_KPH = 120.
  SET_READY_DWELL_NANOS = 200_000_000
  SET_READY_TIMEOUT_NANOS = 60_000_000_000

  # RES and SET- change the set speed by one step per tap and accelerate or coast while held, so
  # they are short open-loop taps. CANCEL only lowers output, so it may be held until its lamp answers.
  TAP_NANOS = 300_000_000
  TAP_GAP_NANOS = 500_000_000
  CANCEL_PRESS_NANOS = 500_000_000
  CANCEL_SETTLE_NANOS = 600_000_000
  CANCEL_MAX_ATTEMPTS = 3
  # CANCEL is not proven on this ECM. A coast whose CANCEL goes unanswered falls back to MAIN OFF; only
  # repeated, separate failures stop trying CANCEL, and then only for a while.
  CANCEL_FAILED_EPISODES = 3
  CANCEL_EPISODE_SPACING_NANOS = 20_000_000_000
  CANCEL_RETRY_NANOS = 300_000_000_000
  # SET lamp drops without a brake are treated as a pause, but not endlessly.
  UNEXPECTED_DROP_LIMIT = 3
  UNEXPECTED_DROP_WINDOW_NANOS = 60_000_000_000
  # A precondition blip shorter than this keeps the follow session.
  PRECOND_GRACE_NANOS = 500_000_000
  # While the stock cruise stays on after the buttons stopped answering, MAIN OFF is retried this often.
  FAULT_RETRY_NANOS = 10_000_000_000


class FollowParams:
  """Lead following on top of the stock cruise, planned by openpilot and executed with button taps.

  Set speeds are on the cluster scale, because that is what the driver reads and what SET captures.
  """
  CLUSTER_GAIN = 1.013
  CLUSTER_OFFSET_KPH = 0.55
  TAP_STEP_KPH = 2.12  # 2 km/h at the ECM
  MIN_SPEED_KPH = 40.
  # The ECM keeps its set speed at or above its own minimum, which sits above MIN_SPEED_KPH on the cluster.
  TAP_DOWN_MIN_KPH = 43.5
  # SET- taps the estimator judged unheard in a row before trimming down waits for the next capture.
  TAP_DOWN_MISS_LIMIT = 2
  # A tap is normally held back until the estimator has resolved the previous one, which costs several
  # seconds. An error this many steps wide is too far off to wait, and is closed by a burst of up to
  # PENDING_BURST_TAPS taps that the estimator then resolves together.
  PENDING_OVERRIDE_STEPS = 2
  PENDING_BURST_TAPS = 3
  # The upstream plan is meaningless for the first seconds after a capture: its MPC has just been reset.
  CAPTURE_HOLD_NANOS = 2_000_000_000
  RESYNC_MIN_SPEED_KPH = 44.
  # Coasting this slow, with no lead to fall back from, means the driver is handling the traffic.
  COAST_DISARM_SPEED_KPH = 30.

  TARGET_DEADBAND_KPH = 1.5
  TARGET_HOLD_NANOS = 1_500_000_000
  CLIMB_MIN_ACCEL = 0.0

  RECAPTURE_DELAY_NANOS = 1_500_000_000
  RECAPTURE_MIN_ACCEL = -0.05
  RECAPTURE_SPEED_MARGIN_KPH = 1.0
  RECAPTURE_RETRY_NANOS = 1_500_000_000
  RESYNC_SET_DELAY_NANOS = 300_000_000

  # Climb is held while any of these is true, and for GATE_CLEAR_NANOS after the last one clears.
  GATE_CLEAR_NANOS = 1_000_000_000
  DOWNSHIFT_HOLD_NANOS = 5_000_000_000
  UPHILL_GRADE_PCT = 3.0
  # Engine demand saturates near 47%, and the downshifts measured on the road start above 34%. Holding
  # the climb below that would hold it through ordinary cruising, which reads 20-40%.
  PEDAL_HOLD_PCT = 44.
  PEDAL_HOLD_HIGH_PCT = 46.
  PEDAL_HIGH_SPEED_KPH = 100.
  # The car cruises the highway in 5th as often as in 6th, around 2800 rpm, so only a genuine overrev
  # holds the climb.
  RPM_SOFT = 3000.
  LOCKUP_CHECK_MIN_KPH = 80.
  LOCKUP_SLIP_RPM = 60.
  # A RES that overshot the target and ended in a kickdown or overrev is taken back with one SET- tap.
  RPM_HARD = 3500.
  RES_UNDO_WINDOW_NANOS = 10_000_000_000

  # ESP2 longitudinal acceleration minus the wheel-speed derivative reads grade with a small positive bias.
  LONG_ACCEL_BIAS = 0.03
  GRADE_TAU = 1.0
  GRADE_MAX_PCT = 8.
  SIGNAL_STALE_NANOS = 250_000_000
  BRAKE_PRESSURE_BAR = 2.5
  BRAKE_PRESSURE_FRAMES = 1


class SetSpeedParams:
  PRIOR_ONE_STEP = 0.75
  PRIOR_MISSED = 0.24
  PRIOR_TWO_STEPS = 0.01
  OBS_SIGMA_KPH = 0.45
  CONFIRM_PROB = 0.9
  # A settled speed this far from every candidate is not trusted as evidence for any of them.
  AMBIGUOUS_KPH = 0.75
  WINDOW_NANOS = 3_000_000_000
  SETTLE_AFTER_TAP_NANOS = 2_500_000_000
  SETTLE_AFTER_CAPTURE_NANOS = 3_000_000_000
  VERIFY_TIMEOUT_NANOS = 60_000_000_000
  STEADY_WHEEL_RANGE_KPH = 0.6
  # The ECM holds its speed uphill but overruns downhill and sags for a while after, so steady windows
  # admit climbs, not descents or their aftermath.
  STEADY_MIN_GRADE_PCT = -2.3
  STEADY_MAX_GRADE_PCT = 5.
  DOWNHILL_RECOVERY_NANOS = 20_000_000_000
  TRACK_GAIN = 0.2
  MISMATCH_LIMIT = 3


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
