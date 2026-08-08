#!/usr/bin/env python3
"""The web console's flight and world commands, expressed as ROS 2 rather than sidecar RPC.

**R-03 step 2.** Step 1 moved video and telemetry onto topics; this moves the buttons. After it,
the only thing the console still needs the Unix socket for is the chase pane (step 4) and
process lifecycle, which can never be ROS calls (see R-03's carve-out).

## No friendly services

`examples/ros2_full_control.py:17-20` refuses to invent a `/testbed/takeoff` on purpose:
moving to hardware must mean deleting `carla_air_bridge` and starting `uxrce_dds_client`, not
rewriting every client. So the flight buttons send **PX4 messages on PX4 topics** — the same
ones a client written against a real Pixhawk would send — and the world buttons call the four
`/sim/*` services that already exist. Nothing new was added to the ROS surface for this.

## Semantics preserved, not improved

Three details of the socket behaviour are matched deliberately, because step 2 must move the
buttons without changing what they do:

* **`yaw` is ABSOLUTE, not relative.** The sidecar calls `rotateToYawAsync(deg)`, which is an
  absolute heading, so the page's ±30 buttons command *heading 30°*, not *turn by 30°*. That
  reads oddly next to the ↺/↻ glyphs, and it is not this change's job to fix — silently making
  it relative would alter behaviour under cover of a refactor.
* **`hold` is a zero-velocity command**, matching `ctrl.velocity(0,0,0)` on the bridge side.
* **The velocity lifetime is NOT ours to set.** `TrajectorySetpoint` has no duration field, so
  the bridge applies its own `setpoint_duration_s` parameter (0.5 s) and the page's
  `duration: 0.7` is simply not expressible. A nudge is therefore slightly shorter over ROS.
  That is the honest consequence of using the PX4 surface, and inventing a duration field to
  paper over it is exactly what the rule above forbids.

## NaN, not zero

Unused fields in a `TrajectorySetpoint` must be NaN. Zeros are a *command to hold still*, and
the bridge reads them as one: `_on_setpoint` takes velocity when every component is finite, so
a position command with `velocity = [0,0,0]` would be silently executed as "stop".
"""
from __future__ import annotations

import math

#: Methods the console can issue over ROS 2. Anything else stays on the socket — the chase
#: pane, and the sidecar-only diagnostics. Kept as data so `server.py` can route without
#: knowing what any of them mean.
ROS_METHODS = frozenset({
    "takeoff", "land", "hold", "velocity", "yaw",           # flight, on /fmu/in/*
    "reset", "spawn_traffic", "set_weather", "destroy_actors",  # world, on /sim/*
})

#: Commands that fly the aircraft, on `/fmu/in/*`.
FLIGHT_METHODS = frozenset({"takeoff", "land", "hold", "velocity", "yaw"})

#: What is refused while another node is publishing setpoints — everything that MOVES THE
#: AIRCRAFT, which is not the same set as "flight commands".
#:
#: `reset` is the one that is easy to get wrong, and this originally did. It is a `/sim/*`
#: service rather than a setpoint, so it looks like world control — but `/sim/reset_vehicle`
#: takes a `hold_ned` and flies the aircraft there, then runs the D-03 convergence loop for
#: several seconds. Relocating an aircraft out from under a running controller is strictly
#: worse than any velocity nudge this refuses.
#:
#: `spawn_traffic`, `set_weather` and `destroy_actors` stay allowed deliberately: they change
#: the world around a running episode, which is rude, but they never touch the aircraft, and a
#: guard that blocks harmless buttons trains people to work around it.
GUARDED_METHODS = FLIGHT_METHODS | frozenset({"reset"})

SETPOINT_TOPIC = "/fmu/in/trajectory_setpoint"


class Contested(RuntimeError):
    """Another node is publishing setpoints, so commanding from here would fight it.

    **Not an interlock.** The count is read and then the command is sent, so a controller that
    starts up in between still slips through. DDS discovery is far slower than that window, so
    closing it with a lock would buy nothing real — but do not mistake this for a guarantee.

    Not a failure to command — a refusal to. `examples/ros2_full_control.py` documents what
    happens otherwise: a takeoff commanded to NED 35 m reached 15.6 m while the aircraft flew
    where the autonomy loop wanted, **with no error anywhere to say why**. Two publishers on
    one setpoint topic do not produce an error, they produce a confusing flight.
    """


def setpoint_fields(velocity=None, position=None, yaw_deg=None):
    """The five array fields of a `TrajectorySetpoint`, with NaN everywhere unused.

    Pure, so the NaN discipline is testable without ROS. Returns a dict rather than a message
    because building the message needs `px4_msgs`, and this file must import on the 3.10 side
    for the offline suite.
    """
    nan3 = [math.nan] * 3
    return {
        "position": [float(v) for v in position] if position is not None else list(nan3),
        "velocity": [float(v) for v in velocity] if velocity is not None else list(nan3),
        "acceleration": list(nan3),
        "jerk": list(nan3),
        "yaw": math.radians(float(yaw_deg)) if yaw_deg is not None else math.nan,
        "yawspeed": math.nan,
    }


def plan(method, args):
    """What a console command becomes on the ROS surface, as plain data.

    Returns `("setpoint", fields)`, `("command", (px4_command_name, params))` or
    `("service", (service_name, args))`. Splitting the decision from the publishing is what
    lets the mapping be tested — including the sign conventions, which is where this would
    otherwise go quietly wrong.
    """
    a = dict(args or {})

    if method == "velocity":
        return "setpoint", setpoint_fields(
            velocity=(a.get("vx", 0.0), a.get("vy", 0.0), a.get("vz", 0.0)),
            yaw_deg=a.get("yaw_deg"))

    if method == "hold":
        # Zero velocity IS the command, and here the zeros are meant literally.
        return "setpoint", setpoint_fields(velocity=(0.0, 0.0, 0.0))

    if method == "yaw":
        # Absolute heading, holding position — see the module docstring.
        return "setpoint", setpoint_fields(velocity=(0.0, 0.0, 0.0), yaw_deg=a["deg"])

    if method == "takeoff":
        # The page sends NED (negative is up); MAVLink param7 is altitude ABOVE the origin and
        # the bridge negates it. Passing the NED value straight through would command a
        # takeoff to below the ground.
        ned = float(a.get("altitude_ned", -30.0))
        return "command", ("VEHICLE_CMD_NAV_TAKEOFF", {"param7": abs(ned)})

    if method == "land":
        return "command", ("VEHICLE_CMD_NAV_LAND", {})

    if method == "reset":
        hold = a.get("hold_ned") or [0.0, 0.0, -30.0]
        return "service", ("/sim/reset_vehicle",
                           {"hold_ned": [float(v) for v in hold],
                            "speed": float(a.get("speed", 10.0))})

    if method == "spawn_traffic":
        return "service", ("/sim/spawn_traffic", a)
    if method == "set_weather":
        return "service", ("/sim/set_weather", a)
    if method == "destroy_actors":
        return "service", ("/sim/destroy_actors", a)

    raise KeyError(f"{method!r} has no ROS 2 equivalent; it stays on the socket")


def others_publishing(total_publishers, we_publish=True):
    """How many nodes OTHER than this console are publishing setpoints.

    `count_publishers` includes our own publisher, and forgetting that would make the console
    permanently refuse to fly — a guard that never opens is as broken as one that never closes.
    """
    return max(0, int(total_publishers) - (1 if we_publish else 0))
