from dataclasses import dataclass


@dataclass(frozen=True)
class FollowCommand:
  """The follow plan openpilot hands to the car controller: target speed in m/s on the vEgo scale."""
  v_target: float
  a_target: float
  coast: bool
