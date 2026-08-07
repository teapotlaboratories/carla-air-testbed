#!/usr/bin/env python3
"""The console's ROS conversions must produce exactly what the page already reads.

    ./.venv/bin/python -m pytest tests/test_ros_source.py -q

R-03 step 1 moved the onboard view and the telemetry off the sidecar socket and onto
`/camera/rgb/image_raw` and `/fmu/out/vehicle_odometry`. The risk in that move is not that it
fails — a failure is visible immediately, the pane stays blank. The risk is that it *succeeds
differently*: a sign flipped, a key renamed, a `numpy.float32` reaching `json.dumps`. Those
produce a console that looks alive and reads wrong, which is worse than one that is plainly
broken.

So this pins the contract from both ends: the keys `webui/index.html` reads must be present,
and the values must survive JSON.

No ROS, no simulator, no GPU — `webui/ros_source.py` keeps its conversions free of `rclpy` and
`px4_msgs` precisely so this file can exist.

Q-01 conventions, R-03 step 1.
"""
from __future__ import annotations

import json
import math
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "webui"))

import ros_source  # noqa: E402

#: Every key `webui/index.html` reads off `/api/state`. Kept as data rather than assertions
#: scattered through the tests, because when the page starts reading a new one this list is
#: where the omission shows up.
PAGE_READS_STATE = ("position", "velocity", "yaw", "armed")
PAGE_READS_COLLISION = ("has_collided", "object_name")


def test_the_state_dict_has_every_key_the_page_reads():
    s = ros_source.odometry_to_state((1.0, 2.0, -3.0), (1.0, 0.0, 0.0, 0.0), (0.1, 0.2, 0.3))
    missing = [k for k in PAGE_READS_STATE if k not in s]
    assert not missing, f"index.html reads {missing} and the conversion does not produce it"


def test_the_collision_dict_has_every_key_the_page_reads():
    c = ros_source.collision_to_dict(True, "Building_17", (1.0, 2.0, 3.0))
    missing = [k for k in PAGE_READS_COLLISION if k not in c]
    assert not missing, f"index.html reads {missing} and the conversion does not produce it"


def test_numpy_values_survive_json():
    """The failure this catches, in the shape it actually arrives.

    A ROS message hands over `numpy.float32` arrays, not lists. `json.dumps` refuses them with
    `Object of type float32 is not JSON serializable` — a 500 from `/api/state` on every poll,
    and only once something is actually publishing, which is not when the code is written.
    """
    np = pytest.importorskip("numpy")
    s = ros_source.odometry_to_state(
        np.array([1.0, 2.0, -3.0], dtype=np.float32),
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        np.array([0.1, 0.2, 0.3], dtype=np.float32))
    json.dumps(s)                                     # must not raise
    assert all(type(v) is float for v in s["position"]), (
        "numpy scalars reached the payload; json.dumps happens to accept some of them and "
        "not others, so this must be forced rather than left to luck")


def test_ned_is_passed_through_untouched():
    """Positive NED z is BELOW the origin, and the origin is 27.45 m above the street.

    The page does that arithmetic itself and shows both altitudes. If this function ever
    "helpfully" negates z or subtracts the ground offset, the page would apply it twice — and
    confusing those two has already cost this project a broken scenario.
    """
    s = ros_source.odometry_to_state((10.0, -20.0, 5.5), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert s["position"] == [10.0, -20.0, 5.5], "the conversion altered NED coordinates"


@pytest.mark.parametrize("q,expected_deg", [
    ((1.0, 0.0, 0.0, 0.0), 0.0),
    ((math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)), 90.0),
    ((math.cos(-math.pi / 4), 0.0, 0.0, math.sin(-math.pi / 4)), -90.0),
])
def test_yaw_matches_the_sidecar_convention(q, expected_deg):
    """Same formula as `carla_air/frames.py`. A different convention here would make the
    heading disagree with every other number in the project by a sign or 90 degrees."""
    got = math.degrees(ros_source.quat_to_yaw(*q))
    assert got == pytest.approx(expected_deg, abs=1e-6)


def test_collision_reports_no_name_as_empty_not_none():
    """`index.html` does `collision.object_name || 'yes'`. A None would render as the string
    "None" in some paths; an empty string takes the fallback, which is what was intended."""
    c = ros_source.collision_to_dict(True, None)
    assert c["object_name"] == ""


class TestFreshness:
    """Never-seen and gone-stale are different problems with different remedies, and the
    status endpoint has to tell them apart: nothing published versus something stopped."""

    def test_never_seen(self):
        assert ros_source.freshness(None) == (None, False)

    def test_fresh(self):
        age, ok = ros_source.freshness(100.0, now=100.5, stale_after=2.0)
        assert ok and age == pytest.approx(0.5)

    def test_stale(self):
        age, ok = ros_source.freshness(100.0, now=103.0, stale_after=2.0)
        assert not ok and age == pytest.approx(3.0)

    def test_the_boundary_is_inclusive(self):
        """Exactly at the threshold is still fresh. Arbitrary, but it has to be one of them,
        and a sample that arrives exactly on the deadline is not a fault."""
        _, ok = ros_source.freshness(100.0, now=102.0, stale_after=2.0)
        assert ok


def test_the_module_imports_without_rclpy():
    """The point of the split. If `ros_source` ever grows a module-scope `import rclpy`, this
    whole file becomes unrunnable outside a sourced ROS environment — and so does the offline
    suite that R-07 exists to keep working."""
    assert "rclpy" not in sys.modules or ros_source.RosSource.import_error is None
    src = ros_source.__file__
    with open(src) as fh:
        module_scope = [ln for ln in fh if ln.startswith(("import ", "from "))]
    offenders = [ln.strip() for ln in module_scope
                 if any(pkg in ln for pkg in ("rclpy", "px4_msgs", "cv_bridge", "sensor_msgs"))]
    assert not offenders, f"ROS imports at module scope: {offenders}"
