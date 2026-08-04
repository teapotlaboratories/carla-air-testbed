#!/usr/bin/env python3
"""The controller's limits must stay reachable under every scenario's own overrides.

This exists because of a bug that no build and no unit test would have caught: the heading
gate was an absolute 0.5 m/s, `street_level` overrides `max_speed_mps` to 0.5, and travelling
due north — literally what that scenario's instruction asks for — produced exactly 0.500,
which is not `> 0.5`. The aircraft flew the street without ever turning to face along it, so
the camera the model is scored on pointed wherever it was last left.

Nothing errored. The only symptom was a navigation failure that looked like the model's.

    ./.venv/bin/python -m pytest tests/test_control_limits.py -q
"""
from __future__ import annotations

import math
import os
import sys

import pytest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "control"))

from control.limits import should_command_yaw, yaw_gate_threshold  # noqa: E402

SCENARIOS = os.path.join(ROOT, "ros2_ws", "src", "evaluation", "scenarios", "default.yaml")
CONFIG = os.path.join(ROOT, "configs", "testbed.yaml")


def _global_speed():
    with open(CONFIG) as fh:
        cfg = yaml.safe_load(fh) or {}
    return float(((cfg.get("graph") or {}).get("offboard_control") or {})
                 .get("max_speed_mps", 5.0))


def _scenarios():
    with open(SCENARIOS) as fh:
        return (yaml.safe_load(fh) or {}).get("scenarios") or []


def _speed_for(scen):
    """The cap this scenario will actually fly under, override included."""
    return float((scen.get("control") or {}).get("max_speed_mps", _global_speed()))


# --------------------------------------------------------------------- the gate itself

def test_gate_is_direction_independent():
    """Same speed, different heading, same verdict.

    The old gate summed |vx| + |vy|, which varies by sqrt(2) across headings — so whether the
    aircraft would turn depended on which way it was already pointing.
    """
    cap = 5.0
    speed = yaw_gate_threshold(cap) * 1.5
    verdicts = set()
    for deg in range(0, 360, 15):
        a = math.radians(deg)
        verdicts.add(should_command_yaw(speed * math.cos(a), speed * math.sin(a), cap))
    assert verdicts == {True}, "the gate's verdict changed with heading at constant speed"


def test_gate_still_rejects_noise():
    """The gate exists to stop a near-stationary wobble becoming a heading command."""
    for cap in (0.5, 5.0, 20.0):
        assert not should_command_yaw(0.001, 0.0, cap)


def test_gate_scales_with_the_cap():
    assert yaw_gate_threshold(5.0) > yaw_gate_threshold(0.5)
    assert yaw_gate_threshold(0.0) > 0.0, "a zero cap must not produce a zero threshold"


# ------------------------------------------------------- against the shipped scenarios

@pytest.mark.parametrize("scen", _scenarios(), ids=lambda s: s["name"])
def test_every_scenario_can_command_yaw_along_each_axis(scen):
    """At its own commanded speed, in the worst direction, the gate must still open.

    Axis-aligned travel is the worst case for the old L1 gate and the most likely heading in
    a city laid out on a grid — `street_level`'s instruction is 'fly north along this street'.
    """
    cap = _speed_for(scen)
    for name, (vx, vy) in {"north": (cap, 0.0), "east": (0.0, cap),
                           "south": (-cap, 0.0), "west": (0.0, -cap)}.items():
        assert should_command_yaw(vx, vy, cap), (
            f"{scen['name']}: flying {name} at its own cap of {cap} m/s would not command a "
            f"heading (threshold {yaw_gate_threshold(cap):.3f} m/s) — the camera would not "
            f"turn to face travel")


@pytest.mark.parametrize("scen", _scenarios(), ids=lambda s: s["name"])
def test_gate_survives_a_climb_or_descent(scen):
    """Vertical motion steals from the horizontal budget; the gate must tolerate that.

    A 60/40 split between horizontal and vertical is ordinary while the controller is
    closing altitude error, and it is what pushed the old gate below its threshold.
    """
    cap = _speed_for(scen)
    assert should_command_yaw(cap * 0.6, 0.0, cap), (
        f"{scen['name']}: a descent at its own cap of {cap} m/s suppresses heading control")
