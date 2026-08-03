#!/usr/bin/env python3
"""The world-control service definitions say what the handlers assume.

    ./.venv/bin/python -m pytest tests/test_interfaces.py -q

These are cheap type assertions, not behaviour tests, and they exist because a `.srv` field
kind is invisible at the call site. `float64[3]` and `float64[]` read almost identically and
behave completely differently: the fixed form is ALWAYS length 3, so a handler branching on
`len(...) == 0` has a dead branch, an unset request silently means the NED origin, and
assigning `[]` serialises to uninitialised memory instead of erroring. That shipped once and
was caught in review rather than by a test - this is the test.

Skipped when the ROS 2 environment is not sourced, so `pytest tests/` still passes on the
3.10 side where `interfaces` cannot be imported at all.
"""
from __future__ import annotations

import pytest

interfaces_srv = pytest.importorskip(
    "interfaces.srv", reason="ROS 2 workspace not sourced; run after scripts/build_ros.sh")


def fields(srv_part):
    return srv_part.get_fields_and_field_types()


def test_near_ned_is_unbounded_so_map_wide_is_expressible():
    """0 values means map-wide. A fixed `double[3]` cannot say that."""
    from interfaces.srv import SpawnTraffic
    assert fields(SpawnTraffic.Request)["near_ned"] == "sequence<double>", (
        "near_ned must be float64[] (unbounded). A fixed float64[3] is always length 3, so "
        "the map-wide branch in _srv_spawn_traffic becomes unreachable.")
    assert len(SpawnTraffic.Request().near_ned) == 0, (
        "an unset near_ned must mean map-wide, not the NED origin (offshore on Town10HD)")


def test_hold_ned_is_exactly_three():
    """A pose is always three numbers; here the fixed array is the right choice."""
    from interfaces.srv import ResetVehicle
    assert fields(ResetVehicle.Request)["hold_ned"] == "double[3]"


@pytest.mark.parametrize("name,expected", [
    ("ResetVehicle", {"success": "boolean", "position_ned": "double[3]", "message": "string"}),
    ("SetWeather", {"success": "boolean", "applied": "string", "message": "string"}),
    ("DestroyActors", {"success": "boolean", "destroyed": "int32", "message": "string"}),
])
def test_responses_carry_success_and_a_reason(name, expected):
    """Every world-control call can fail in a way the caller must see - that is why these
    are services rather than topics, and the response has to make it inspectable."""
    srv = getattr(interfaces_srv, name)
    assert fields(srv.Response) == expected


def test_spawn_traffic_reports_what_it_actually_got():
    """Asked-for and got differ routinely: the map refuses occupied spawn points."""
    from interfaces.srv import SpawnTraffic
    got = fields(SpawnTraffic.Response)
    assert got["spawned"] == "int32" and got["walkers_spawned"] == "int32"
    assert got["success"] == "boolean" and got["message"] == "string"


def test_destroy_actors_takes_no_arguments():
    """A filter would invite leaving half the traffic behind between runs, which is exactly
    the state that makes two episodes with the same seed diverge."""
    from interfaces.srv import DestroyActors
    assert fields(DestroyActors.Request) == {}
