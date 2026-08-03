#!/usr/bin/env python3
"""The obstacle geometry behind `scripts/survey_buildings.py`, without a simulator.

These are the functions that decide whether a scenario contains an obstacle at all, so a
bug here does not produce a wrong number — it produces a scenario that quietly measures
nothing, which is exactly the failure E-02 was opened to fix.

    ./.venv/bin/python -m pytest tests/test_survey.py -q
"""
from __future__ import annotations

import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from scripts.survey_buildings import (  # noqa: E402
    GROUND_NED_Z,
    column_roof,
    corners,
    is_free,
    route,
    segment_hits_box,
    solid_length,
)


def box(x0, y0, z0, x1, y1, z1, name="b"):
    return {"name": name, "min_ned": [x0, y0, z0], "max_ned": [x1, y1, z1],
            "roof_agl_m": GROUND_NED_Z - z0}


# A 10 m cube centred on the origin's x-axis at x = 45..55.
CUBE = box(45.0, -5.0, -5.0, 55.0, 5.0, 5.0)


# ---------- segment_hits_box ----------

def test_segment_straight_through_the_middle():
    r = segment_hits_box((0.0, 0.0, 0.0), (100.0, 0.0, 0.0), CUBE["min_ned"], CUBE["max_ned"])
    assert r is not None
    assert r[0] == pytest.approx(0.45)
    assert r[1] == pytest.approx(0.55)


def test_segment_that_stops_short_does_not_hit():
    """The box is beyond the far endpoint. A ray test would report a hit here; a segment
    test must not, or every scenario looks obstructed by something behind the goal."""
    assert segment_hits_box((0.0, 0.0, 0.0), (40.0, 0.0, 0.0),
                            CUBE["min_ned"], CUBE["max_ned"]) is None


def test_segment_passing_beside_the_box():
    assert segment_hits_box((0.0, 20.0, 0.0), (100.0, 20.0, 0.0),
                            CUBE["min_ned"], CUBE["max_ned"]) is None


def test_segment_passing_over_the_box():
    """NED z is down, so 'over' is more negative."""
    assert segment_hits_box((0.0, 0.0, -30.0), (100.0, 0.0, -30.0),
                            CUBE["min_ned"], CUBE["max_ned"]) is None


def test_inflate_turns_a_near_miss_into_a_hit():
    p0, p1 = (0.0, 7.0, 0.0), (100.0, 7.0, 0.0)
    assert segment_hits_box(p0, p1, CUBE["min_ned"], CUBE["max_ned"]) is None
    assert segment_hits_box(p0, p1, CUBE["min_ned"], CUBE["max_ned"], inflate=3.0) is not None


def test_axis_aligned_segment_inside_a_slab():
    """Zero-length component on an axis: the degenerate branch must not divide by it."""
    slab = box(-100.0, -100.0, -5.0, 100.0, 100.0, 5.0)
    r = segment_hits_box((0.0, 0.0, 0.0), (0.0, 50.0, 0.0), slab["min_ned"], slab["max_ned"])
    assert r == (0.0, 1.0)


# ---------- solid_length ----------

def test_overlapping_pieces_are_counted_once():
    """The bug this exists for: summing per-piece hits reported 143 m of obstacle on a 90 m
    line, because a procedural facade stacks trim and panels in the same volume."""
    stacked = [CUBE, box(45.0, -5.0, -5.0, 55.0, 5.0, 5.0, "trim"),
               box(46.0, -4.0, -4.0, 54.0, 4.0, 4.0, "window")]
    assert solid_length((0.0, 0.0, 0.0), (100.0, 0.0, 0.0), stacked) == pytest.approx(10.0)


def test_disjoint_pieces_add_up():
    two = [CUBE, box(70.0, -5.0, -5.0, 80.0, 5.0, 5.0, "second")]
    assert solid_length((0.0, 0.0, 0.0), (100.0, 0.0, 0.0), two) == pytest.approx(20.0)


def test_adjacent_pieces_merge_without_double_counting():
    touching = [CUBE, box(55.0, -5.0, -5.0, 65.0, 5.0, 5.0, "next")]
    assert solid_length((0.0, 0.0, 0.0), (100.0, 0.0, 0.0), touching) == pytest.approx(20.0)


def test_clear_line_measures_zero():
    assert solid_length((0.0, 40.0, 0.0), (100.0, 40.0, 0.0), [CUBE]) == 0.0


# ---------- is_free ----------

def test_point_inside_is_not_free():
    assert not is_free((50.0, 0.0, 0.0), [CUBE], 0.0)


def test_margin_keeps_a_nearby_point_from_counting_as_free():
    assert is_free((62.0, 0.0, 0.0), [CUBE], 5.0)
    assert not is_free((62.0, 0.0, 0.0), [CUBE], 10.0)


def test_free_with_no_geometry_at_all():
    assert is_free((0.0, 0.0, 0.0), [], 10.0)


# ---------- column_roof ----------

def test_column_roof_sees_pieces_above_the_flight_band():
    """A tower's upper floors are different pieces from the ones solid at flight altitude.
    Reading the roof off only the pieces a segment hit made a 154 m tower look 31 m tall."""
    lower = box(45.0, -5.0, GROUND_NED_Z - 50.0, 55.0, 5.0, GROUND_NED_Z - 40.0, "lower")
    upper = box(45.0, -5.0, GROUND_NED_Z - 150.0, 55.0, 5.0, GROUND_NED_Z - 140.0, "upper")
    assert column_roof([45.0, -5.0], [55.0, 5.0], [lower]) == pytest.approx(50.0)
    assert column_roof([45.0, -5.0], [55.0, 5.0], [lower, upper]) == pytest.approx(150.0)


def test_column_roof_ignores_geometry_outside_the_footprint():
    far = box(500.0, 500.0, GROUND_NED_Z - 200.0, 510.0, 510.0, GROUND_NED_Z - 190.0, "far")
    assert column_roof([45.0, -5.0], [55.0, 5.0], [far]) is None


# ---------- route ----------

def test_clear_route_is_the_straight_line():
    p0, p1 = (0.0, 0.0, 0.0), (0.0, 60.0, 0.0)
    length, pts = route(p0, p1, [], clearance=4.0, pad=40.0)
    assert length == pytest.approx(60.0, abs=2.0)
    assert len(corners(pts)) == 2  # start and end, no turns


def test_route_goes_around_a_blocking_box():
    """The straight line is blocked; a detour exists and must cost more than the straight
    line but not wildly more."""
    wall = box(-10.0, 25.0, -5.0, 10.0, 35.0, 5.0, "wall")
    p0, p1 = (0.0, 0.0, 0.0), (0.0, 60.0, 0.0)
    assert segment_hits_box(p0, p1, wall["min_ned"], wall["max_ned"]) is not None
    length, pts = route(p0, p1, [wall], clearance=4.0, pad=60.0)
    assert length is not None
    assert length > 60.0
    assert len(corners(pts)) > 2


def test_route_reports_unreachable_when_walled_in():
    """A box fully enclosing the start, within the search area."""
    walls = [
        box(-20.0, -20.0, -5.0, 20.0, -15.0, 5.0, "s"),
        box(-20.0, 15.0, -5.0, 20.0, 20.0, 5.0, "n"),
        box(-20.0, -20.0, -5.0, -15.0, 20.0, 5.0, "w"),
        box(15.0, -20.0, -5.0, 20.0, 20.0, 5.0, "e"),
    ]
    length, pts = route((0.0, 0.0, 0.0), (0.0, 60.0, 0.0), walls, clearance=1.0, pad=40.0)
    assert length is None
    assert pts == []


def test_route_ignores_geometry_above_and_below_the_flight_level():
    """The router works at one altitude. A slab the aircraft flies under must not block it."""
    overhead = box(-10.0, 25.0, -50.0, 10.0, 35.0, -40.0, "overhead")
    length, _ = route((0.0, 0.0, 0.0), (0.0, 60.0, 0.0), [overhead], clearance=4.0, pad=40.0)
    assert length == pytest.approx(60.0, abs=2.0)


def test_route_refuses_a_goal_inside_a_building():
    wall = box(-10.0, 55.0, -5.0, 10.0, 65.0, 5.0, "wall")
    length, _ = route((0.0, 0.0, 0.0), (0.0, 60.0, 0.0), [wall], clearance=1.0, pad=40.0)
    assert length is None


# ---------- corners ----------

def test_corners_keeps_only_the_turns():
    straight = [(0.0, float(i), 0.0) for i in range(10)]
    assert corners(straight) == [straight[0], straight[-1]]


def test_corners_survives_a_two_point_path():
    pts = [(0.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
    assert corners(pts) == pts


# ---------- the offset that caused E-02 ----------

def test_ned_altitude_is_not_agl():
    """The whole reason the old scenario never met a building: the AirSim origin on Town10HD
    sits 27.45 m above the street, so `z = -50` is 77.45 m AGL, not 50."""
    assert GROUND_NED_Z - (-50.0) == pytest.approx(77.45)
