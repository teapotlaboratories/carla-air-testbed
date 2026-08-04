"""Pure limit arithmetic for the offboard controller.

Separate from `offboard_node` so it can be tested with no ROS, no simulator and no GPU —
`offboard_node` imports rclpy and px4_msgs, which the offline suite deliberately cannot.
Nothing here may import either.

The rule these encode: **a limit must scale with the limits around it.** An absolute
constant is correct only for the one configuration it was tuned against, and this project
now has per-scenario `control:` overrides that move those configurations by an order of
magnitude.
"""
from __future__ import annotations

import math

#: Below this fraction of the commanded speed cap, horizontal motion is treated as noise
#: rather than travel, and the heading is left alone.
YAW_GATE_FRACTION = 0.1

#: Absolute floor, so a scenario that crawls still has a non-zero gate.
YAW_GATE_FLOOR_MPS = 0.02


def yaw_gate_threshold(max_speed_mps: float) -> float:
    """The horizontal speed above which a heading command is worth sending."""
    return max(YAW_GATE_FLOOR_MPS, YAW_GATE_FRACTION * float(max_speed_mps))


def should_command_yaw(vx: float, vy: float, max_speed_mps: float) -> bool:
    """Is the aircraft moving horizontally enough for its heading to mean anything?

    Uses the MAGNITUDE, not `abs(vx) + abs(vy)`. The L1 sum varies by sqrt(2) with heading
    for one speed, so an L1 gate is tighter for axis-aligned travel than for diagonal — the
    aircraft's willingness to turn depended on which way it was already pointing.

    Scaled to the cap, not absolute. The previous absolute 0.5 was unreachable for
    `street_level`, which overrides `max_speed_mps` to 0.5: due north — what its instruction
    asks for — produced exactly 0.500 and never cleared a `> 0.5` test, so the aircraft flew
    the street without ever turning to face along it, with the camera the model is scored on
    pointing wherever it happened to be left.
    """
    return math.hypot(float(vx), float(vy)) > yaw_gate_threshold(max_speed_mps)
