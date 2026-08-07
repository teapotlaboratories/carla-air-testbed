#!/usr/bin/env python3
"""`reset()` must retry while it is helping, and stop when it is not.

    ./.venv/bin/python -m pytest tests/test_reset_convergence.py -q

The convergence loop has three outcomes and a silent failure mode either side of it. Stop
retrying too eagerly and a reset that would have converged reports a miss; stop too late and
an unreachable pose burns every attempt before saying so. Neither raises, both are only
visible in a number the caller may not check — which is exactly the shape that needs a test
rather than a careful reading.

Both behaviours were established against a live simulator (D-03, D-05) and that is the slow,
expensive way to keep them true. `Vehicle.__init__` takes a client, so a fake returning a
scripted sequence of positions answers the same questions in milliseconds and on every change.

Q-01. No simulator, no airsim server, no GPU.
"""
from __future__ import annotations

import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "sim_bridge"))

pytest.importorskip("airsim", reason="Vehicle imports airsim for its pose types")

from carla_air.vehicle import Vehicle  # noqa: E402

TARGET = (10.0, 20.0, -50.0)


class _V:
    def __init__(self, x, y, z):
        self.x_val, self.y_val, self.z_val = x, y, z


class _Q:
    w_val, x_val, y_val, z_val = 1.0, 0.0, 0.0, 0.0


class _Kin:
    def __init__(self, pos):
        self.position = _V(*pos)
        self.linear_velocity = _V(0.0, 0.0, 0.0)
        self.angular_velocity = _V(0.0, 0.0, 0.0)
        self.orientation = _Q()


class _State:
    landed_state = 0

    def __init__(self, pos):
        self.kinematics_estimated = _Kin(pos)


class _Collision:
    time_stamp = 0


class _Future:
    def join(self):
        return None


class FakeClient:
    """An AirSim client that reports a scripted sequence of positions.

    One entry per `moveToPositionAsync` — i.e. one per attempt of the convergence loop. The
    last entry repeats if the loop asks for more, which is what a floor looks like.
    """

    def __init__(self, positions):
        self.positions = list(positions)
        self.moves = 0
        self.poses_set = 0

    # -- what reset() calls -------------------------------------------------------------
    def cancelLastTask(self, *a, **k): pass
    def armDisarm(self, *a, **k): pass
    def enableApiControl(self, *a, **k): pass
    def reset(self, *a, **k): pass
    def simGetCollisionInfo(self, *a, **k): return _Collision()

    def simSetVehiclePose(self, *a, **k):
        self.poses_set += 1

    def moveToPositionAsync(self, *a, **k):
        self.moves += 1
        return _Future()

    def getMultirotorState(self, *a, **k):
        i = min(self.moves, len(self.positions)) - 1
        return _State(self.positions[max(i, 0)])


@pytest.fixture(autouse=True)
def _no_sleeping(monkeypatch):
    """reset() sleeps ~2.8 s per attempt. None of it is what is under test."""
    monkeypatch.setattr("carla_air.vehicle.time.sleep", lambda _s: None)


def _run(positions, **kw):
    c = FakeClient(positions)
    stages = []
    v = Vehicle(c)
    v.reset(hold_ned=TARGET, settle_s=0.0, on_stage=stages.append, **kw)
    return c, stages


def test_it_stops_as_soon_as_it_is_inside_tolerance():
    """The ordinary case: one hold is enough, so there is no second attempt."""
    c, stages = _run([TARGET])
    assert c.moves == 1, "commanded the hold more than once when the first one landed"
    assert not any("re-holding" in s for s in stages)


def test_it_retries_while_it_is_helping():
    """D-03: `join()` returns before the aircraft has arrived, so one attempt is not enough."""
    c, stages = _run([(10.0, 20.0, -30.0),      # 20 m out
                      (10.0, 20.0, -44.0),      # 6 m  — still outside 1.5
                      TARGET])                  # arrived
    assert c.moves == 3, f"gave up after {c.moves} attempts while it was still converging"
    assert any("converged after 3" in s for s in stages), stages


def test_it_stops_when_an_attempt_stops_helping():
    """D-05: a floor. Retrying cannot fix it, so it must not burn every attempt.

    3.3 m out and staying there is the measured street-level case — commanded to 3.5 m AGL the
    aircraft settles at 0.2 m and does not move however many times it is asked.
    """
    floor = (10.0, 20.0, -46.7)                 # 3.3 m out, forever
    c, stages = _run([floor] * 6)
    assert c.moves == 2, (
        f"made {c.moves} attempts against a floor; the point of RESET_MIN_IMPROVEMENT_M is "
        "that the second one proves the first was not a fluke and the rest are wasted")
    assert any("stalled" in s for s in stages), stages
    assert not any("NOT CONVERGED" in s for s in stages), (
        "reported a generic failure when it had a specific one to report")


def test_a_slow_but_real_improvement_is_not_mistaken_for_a_floor():
    """The other side of the guard, and the one that would fail silently.

    Improving by more than RESET_MIN_IMPROVEMENT_M each time is converging, slowly. Stopping
    there would report a miss for a reset that was about to succeed.
    """
    c, stages = _run([(10.0, 20.0, -44.0),      # 6.0 m out
                      (10.0, 20.0, -46.0),      # 4.0 — improved 2.0
                      (10.0, 20.0, -48.0),      # 2.0 — improved 2.0
                      TARGET])                  # arrived
    assert c.moves == 4, f"stopped after {c.moves} while still improving 2 m per attempt"
    assert not any("stalled" in s for s in stages), stages


def test_it_gives_up_loudly_rather_than_spinning():
    """Never reachable, and improving just enough each time to dodge the stall guard.

    The attempt cap is the backstop. It must report, not raise: a start pose that is metres out
    still flies, and failing the episode outright would be worse than flying it with the error
    recorded.
    """
    c, stages = _run([(10.0, 20.0, -20.0),      # 30 m
                      (10.0, 20.0, -25.0),      # 25
                      (10.0, 20.0, -30.0),      # 20
                      (10.0, 20.0, -35.0)])     # 15 — still out, attempts exhausted
    assert c.moves == Vehicle.RESET_ATTEMPTS
    assert any("NOT CONVERGED" in s for s in stages), stages


def test_the_caller_gets_the_real_position_even_when_it_did_not_converge():
    """Whatever happened, the returned state is where the aircraft actually is."""
    floor = (10.0, 20.0, -46.7)
    c = FakeClient([floor] * 6)
    got = Vehicle(c).reset(hold_ned=TARGET, settle_s=0.0)
    assert got["position"] == pytest.approx(list(floor)), (
        "returned something other than the measured position, so a caller cannot tell")
