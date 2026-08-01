#!/usr/bin/env python3
"""Lint the scenario definitions, and test the oracle that validates them.

A scenario with an unreachable goal produces a table full of plausible-looking failures and
no indication that the harness, not the model, is at fault. These checks are cheap, run
without a simulator, and catch the mistakes that are otherwise only visible after a
half-hour sweep:

* a goal further away than the episode's own travel budget can reach,
* a success radius tighter than the vehicle's ~4 m station-keeping,
* start or goal altitudes outside the range the controller will actually fly,
* a goal the aircraft is already standing on.

They cannot check whether the goal is inside a building — that needs the oracle and the
simulator. See `vlm_client/backends/oracle.py`.

    ./.venv/bin/python -m pytest tests/test_scenarios.py -q
"""
from __future__ import annotations

import math
import os
import sys

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "sim_bridge"))
sys.path.insert(0, os.path.join(ROOT, "sim_bridge", "carla_air"))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "vlm_client"))

SCENARIOS = os.path.join(ROOT, "ros2_ws", "src", "evaluation", "scenarios", "default.yaml")
PARAMS = os.path.join(ROOT, "ros2_ws", "src", "bringup", "config", "testbed.yaml")


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


SCEN = load(SCENARIOS)["scenarios"]
CTRL = load(PARAMS)["offboard_control"]["ros__parameters"]
ALL = [pytest.param(s, id=s["name"]) for s in SCEN]


def test_scenario_file_is_not_empty():
    assert len(SCEN) >= 1
    assert len({s["name"] for s in SCEN}) == len(SCEN), "duplicate scenario names"


@pytest.mark.parametrize("s", ALL)
def test_required_fields(s):
    for key in ("name", "instruction", "start_ned", "goal_ned"):
        assert key in s and s[key], f"{s.get('name')} missing {key}"
    assert len(s["start_ned"]) == 3 and len(s["goal_ned"]) == 3


@pytest.mark.parametrize("s", ALL)
def test_goal_is_reachable_within_the_step_budget(s):
    """max_steps x max_step_m has to exceed the straight-line distance, with margin.

    The controller flies a bounded step per VLM annotation, so an episode can physically
    cover at most that product — and only if every step points at the goal, which no real
    backend manages. Anything under 2x is a scenario that fails for arithmetic reasons.
    """
    dist = math.dist(s["start_ned"], s["goal_ned"])
    budget = s.get("max_steps", 30) * CTRL["max_step_m"]
    assert budget >= 2.0 * dist, (
        f"{s['name']}: {dist:.0f} m to goal but only {budget:.0f} m of travel budget "
        f"({s.get('max_steps', 30)} steps x {CTRL['max_step_m']} m)")


@pytest.mark.parametrize("s", ALL)
def test_timeout_allows_the_flight(s):
    """Wall-clock has to cover the distance at the controller's speed, plus VLM thinking.

    Sweeps run in real time — ClockSpeed > 1 desyncs CARLA from AirSim — so a timeout that
    is merely optimistic costs real minutes on every seed.
    """
    dist = math.dist(s["start_ned"], s["goal_ned"])
    flight_s = dist / CTRL["max_speed_mps"]
    assert s.get("timeout_s", 240.0) >= 1.5 * flight_s, (
        f"{s['name']}: {dist:.0f} m needs >= {flight_s:.0f} s of pure flight, "
        f"timeout is {s.get('timeout_s', 240.0):.0f} s")


@pytest.mark.parametrize("s", ALL)
def test_success_radius_is_above_the_station_keeping_floor(s):
    """The vehicle relaxes ~4 m after reaching a setpoint (tests/conformance/p10).

    A radius below that measures the controller's drift, not the model's navigation.
    """
    assert s.get("success_radius_m", 20.0) >= 8.0, (
        f"{s['name']}: success radius {s.get('success_radius_m')} m is inside the "
        "~4 m post-setpoint relaxation")


@pytest.mark.parametrize("s", ALL)
def test_altitudes_are_inside_the_controller_envelope(s):
    """NED: altitude is -z. A goal outside the clamp can never be reached."""
    lo, hi = CTRL["min_altitude_m"], CTRL["max_altitude_m"]
    for label, ned in (("start", s["start_ned"]), ("goal", s["goal_ned"])):
        alt = -ned[2]
        assert lo <= alt <= hi, (
            f"{s['name']} {label} altitude {alt:.0f} m is outside the controller's "
            f"[{lo}, {hi}] m clamp — it will be silently re-targeted")


@pytest.mark.parametrize("s", ALL)
def test_start_is_not_already_at_the_goal(s):
    dist = math.dist(s["start_ned"], s["goal_ned"])
    assert dist > s.get("success_radius_m", 20.0), (
        f"{s['name']}: starts {dist:.0f} m from goal, inside its own success radius")


@pytest.mark.parametrize("s", ALL)
def test_weather_preset_exists(s):
    from carla_air.world import World  # noqa: PLC0415 — needs the carla module

    assert s.get("weather", "ClearNoon") in World.WEATHER


# --------------------------------------------------------------------------- oracle


def _fake_image(w=640, h=480):
    import numpy as np

    return np.zeros((h, w, 3), dtype=np.uint8)


def _oracle():
    from vlm_client.backends.oracle import OracleBackend

    return OracleBackend()


def test_oracle_annotates_the_centre_when_the_goal_is_dead_ahead():
    o = _oracle()
    o.set_context(goal_ned=(100.0, 0.0, -50.0), cam_pos=(0.0, 0.0, -50.0),
                  cam_quat=(1.0, 0.0, 0.0, 0.0), hfov_deg=90.0)
    a = o.annotate(_fake_image(), "go to the goal")
    assert (a.u, a.v) == (320, 240)
    assert a.confidence == 1.0 and o.off_screen == 0


def test_oracle_points_left_when_the_goal_is_left():
    o = _oracle()
    o.set_context(goal_ned=(100.0, -40.0, -50.0), cam_pos=(0.0, 0.0, -50.0),
                  cam_quat=(1.0, 0.0, 0.0, 0.0), hfov_deg=90.0)
    a = o.annotate(_fake_image(), "go")
    assert a.u < 320, "a goal to port must annotate left of centre"


def test_oracle_reports_a_goal_behind_the_camera_rather_than_guessing():
    o = _oracle()
    o.set_context(goal_ned=(-100.0, 0.0, -50.0), cam_pos=(0.0, 0.0, -50.0),
                  cam_quat=(1.0, 0.0, 0.0, 0.0), hfov_deg=90.0)
    a = o.annotate(_fake_image(), "go")
    assert o.off_screen == 1
    assert "behind" in a.rationale


def test_oracle_declares_terminal_only_when_close():
    o = _oracle()
    o.set_context(goal_ned=(200.0, 0.0, -50.0), cam_pos=(0.0, 0.0, -50.0),
                  cam_quat=(1.0, 0.0, 0.0, 0.0), hfov_deg=90.0)
    assert not o.annotate(_fake_image(), "go").terminal
    o.set_context(goal_ned=(8.0, 0.0, -50.0), cam_pos=(0.0, 0.0, -50.0),
                  cam_quat=(1.0, 0.0, 0.0, 0.0), hfov_deg=90.0)
    assert o.annotate(_fake_image(), "go").terminal


def test_oracle_without_context_declines_rather_than_inventing_a_pixel():
    assert _oracle().annotate(_fake_image(), "go") is None


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
