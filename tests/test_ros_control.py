#!/usr/bin/env python3
"""Every console button must mean the same thing on ROS 2 as it did on the socket.

    ./.venv/bin/python -m pytest tests/test_ros_control.py -q

R-03 step 2 moves the buttons off the sidecar RPC and onto PX4 messages and `/sim/*` services.
The danger is not that a button stops working — that is obvious the first time it is pressed.
It is that a button keeps working and means something slightly different: a sign flipped, a
relative command turned absolute, a NaN turned into a zero. Each of those flies the aircraft
somewhere nobody asked for, with nothing in any log to say why.

So this pins the mapping as data. No ROS, no simulator, no GPU.

R-03 step 2.
"""
from __future__ import annotations

import math
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "webui"))

import ros_control  # noqa: E402


# ------------------------------------------------------------------ the NaN discipline

def test_unused_setpoint_fields_are_nan_not_zero():
    """The single most dangerous confusion in this file.

    `bridge_node._on_setpoint` takes VELOCITY when every velocity component is finite, and only
    falls through to position otherwise. So a position command carrying `velocity = [0,0,0]`
    is not "go there, without a velocity preference" — it is executed as **stop**.
    """
    fields = ros_control.setpoint_fields(position=(1.0, 2.0, -30.0))
    assert all(math.isnan(v) for v in fields["velocity"]), (
        "a position setpoint carried finite zero velocity; the bridge would read it as 'stop' "
        "and the aircraft would never move")
    assert all(math.isnan(v) for v in fields["acceleration"])
    assert all(math.isnan(v) for v in fields["jerk"])
    assert math.isnan(fields["yawspeed"])


def test_an_omitted_yaw_is_nan_so_the_current_heading_is_kept():
    assert math.isnan(ros_control.setpoint_fields(velocity=(1.0, 0.0, 0.0))["yaw"])


# ------------------------------------------------------------------ sign conventions

def test_takeoff_converts_ned_to_mavlink_altitude():
    """The page sends NED (negative is UP). MAVLink `param7` is altitude ABOVE the origin, and
    `bridge_node` negates it again. Passing the NED value straight through would command a
    takeoff to 30 m BELOW the ground."""
    kind, (command, params) = ros_control.plan("takeoff", {"altitude_ned": -30.0})
    assert kind == "command" and command == "VEHICLE_CMD_NAV_TAKEOFF"
    assert params["param7"] == 30.0, "sent a negative altitude; the bridge would fly it into the map"


def test_takeoff_is_robust_to_a_positive_altitude_too():
    """The page sends `-Math.abs(alt)`, but a hand-written client may not. Either way the
    intent is 'up', and `abs()` is what the bridge does with it."""
    _, (_, params) = ros_control.plan("takeoff", {"altitude_ned": 30.0})
    assert params["param7"] == 30.0


def test_velocity_passes_ned_through_unaltered():
    _, fields = ros_control.plan("velocity", {"vx": 1.5, "vy": -2.0, "vz": 0.5})
    assert fields["velocity"] == [1.5, -2.0, 0.5]
    assert all(math.isnan(v) for v in fields["position"])


# ------------------------------------------------------------------ semantics preserved

def test_yaw_stays_absolute_and_is_sent_in_radians():
    """The sidecar calls `rotateToYawAsync(deg)`, which is an ABSOLUTE heading. The ↺/↻ buttons
    therefore command *heading 30 degrees*, not *turn by 30 degrees*.

    That reads oddly, and changing it is not this step's job: silently making it relative would
    alter what a button does under cover of a refactor. If it should be relative, that is a
    separate decision with its own entry.
    """
    _, fields = ros_control.plan("yaw", {"deg": 30.0})
    assert fields["yaw"] == pytest.approx(math.radians(30.0))
    assert fields["velocity"] == [0.0, 0.0, 0.0], "yaw must hold position while it turns"


def test_hold_is_a_literal_zero_velocity_command():
    """The one place zeros are meant. 'Send nothing' is what makes an aircraft run away."""
    _, fields = ros_control.plan("hold", {})
    assert fields["velocity"] == [0.0, 0.0, 0.0]
    assert math.isnan(fields["yaw"]), "hold must not also command a heading"


def test_reset_keeps_the_pages_ned_and_speed():
    kind, (service, args) = ros_control.plan("reset", {"hold_ned": [10.0, 20.0, -55.0], "speed": 10.0})
    assert kind == "service" and service == "/sim/reset_vehicle"
    assert args["hold_ned"] == [10.0, 20.0, -55.0] and args["speed"] == 10.0


@pytest.mark.parametrize("method,service", [
    ("spawn_traffic", "/sim/spawn_traffic"),
    ("set_weather", "/sim/set_weather"),
    ("destroy_actors", "/sim/destroy_actors"),
])
def test_world_commands_use_the_services_that_already_exist(method, service):
    """No new ROS surface was invented for step 2 — these four services predate it."""
    kind, (name, _) = ros_control.plan(method, {})
    assert kind == "service" and name == service


# ------------------------------------------------------------------ what stays on the socket

def test_a_method_with_no_ros_equivalent_is_refused_not_guessed():
    """`chase_jpeg` is the step-4 gap. Inventing a mapping would be worse than routing it back
    to the socket, which is what `server.py` does when this raises."""
    with pytest.raises(KeyError):
        ros_control.plan("chase_jpeg", {})


def test_the_routing_table_and_the_flight_subset_agree():
    assert ros_control.FLIGHT_METHODS <= ros_control.ROS_METHODS
    assert "set_weather" not in ros_control.FLIGHT_METHODS, (
        "changing the weather mid-episode is rude, not dangerous — refusing it while another "
        "node flies would block a harmless button for no reason")


# ------------------------------------------------------------------ the contention guard

class TestOthersPublishing:
    """`count_publishers` includes our own publisher. Forgetting that gives a guard that never
    opens, which is exactly as broken as one that never closes."""

    def test_only_us(self):
        assert ros_control.others_publishing(1) == 0

    def test_us_and_a_controller(self):
        assert ros_control.others_publishing(2) == 1

    def test_before_our_publisher_exists(self):
        assert ros_control.others_publishing(0) == 0

    def test_it_never_goes_negative(self):
        assert ros_control.others_publishing(0, we_publish=True) == 0
