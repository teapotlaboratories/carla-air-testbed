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

import types  # noqa: E402
import ros_source  # noqa: E402
from types import SimpleNamespace  # noqa: E402

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
    # No runtime check on sys.modules here: rclpy is never importable in the offline suite,
    # so any such assertion passes without testing anything. Reading the source is the check
    # that can actually fail.
    src = ros_source.__file__
    with open(src) as fh:
        module_scope = [ln for ln in fh if ln.startswith(("import ", "from "))]
    offenders = [ln.strip() for ln in module_scope
                 if any(pkg in ln for pkg in ("rclpy", "px4_msgs", "cv_bridge", "sensor_msgs"))]
    assert not offenders, f"ROS imports at module scope: {offenders}"


class TestTheChasePaneHoldsTheSensorUp:
    """R-03 step 4, console half.

    The bridge spawns the CARLA chase sensor while anything is subscribed, so on this side the
    SUBSCRIPTION is the thing being managed — not a stream, not a button. A subscription held
    for the console's lifetime would pin a full extra render pass up for as long as the console
    ran, which is precisely the cost the topic design exists to avoid. An idle console has to
    be invisible to the simulator.

    Driven against a fake node, because creating a real one needs rclpy and a graph. What is
    under test is the counting and the create/destroy pairing.
    """

    class _FakeNode:
        def __init__(self):
            self.created, self.destroyed = [], []

        def create_subscription(self, msg_type, topic, cb, qos):
            sub = f"sub:{topic}:{len(self.created)}"
            self.created.append(topic)
            return sub

        def destroy_subscription(self, sub):
            self.destroyed.append(sub)

    @pytest.fixture
    def src(self, monkeypatch):
        # `chase_acquire` imports CompressedImage at call time, and the 3.10 venv that runs
        # the documented offline suite has no sensor_msgs. Stubbed rather than skipped: what
        # is under test is the counting, and skipping would have meant these never ran in the
        # suite anyone actually invokes. `setitem` restores sys.modules afterwards.
        if "sensor_msgs.msg" not in sys.modules:
            pkg, mod = types.ModuleType("sensor_msgs"), types.ModuleType("sensor_msgs.msg")
            mod.CompressedImage = type("CompressedImage", (), {})
            pkg.msg = mod
            monkeypatch.setitem(sys.modules, "sensor_msgs", pkg)
            monkeypatch.setitem(sys.modules, "sensor_msgs.msg", mod)
        s = ros_source.RosSource()
        s._node = self._FakeNode()
        return s

    def test_opening_the_pane_subscribes(self, src):
        assert src._chase_sub is None, "subscribed before anyone opened the pane"
        src.chase_acquire()
        assert src._node.created == ["/camera/chase/image_raw/compressed"]

    def test_closing_the_last_pane_unsubscribes(self, src):
        src.chase_acquire()
        src.chase_release()
        assert src._chase_sub is None
        assert len(src._node.destroyed) == 1, "the subscription outlived its only viewer"

    def test_two_tabs_need_two_closes(self, src):
        """The first tab closing must not take the picture away from the second."""
        src.chase_acquire()
        src.chase_acquire()
        src.chase_release()
        assert src._chase_sub is not None and not src._node.destroyed
        src.chase_release()
        assert src._chase_sub is None and len(src._node.destroyed) == 1

    def test_reopening_subscribes_again(self, src):
        src.chase_acquire(); src.chase_release()
        src.chase_acquire()
        assert len(src._node.created) == 2, "the pane could not be reopened"

    def test_frames_are_passed_through_without_re_encoding(self, src):
        """The topic already carries JPEG. Unlike the onboard pane there is nothing to encode
        here — re-encoding would spend CPU on the machine that is also rendering, to produce
        the same picture."""
        src.chase_acquire()
        src._on_chase(SimpleNamespace(data=b"\xff\xd8jpeg-bytes"))
        assert src.latest_chase_jpeg() == b"\xff\xd8jpeg-bytes"

    def test_no_frame_yet_reads_as_none_not_as_an_error(self, src):
        src.chase_acquire()
        assert src.latest_chase_jpeg() is None

    def test_releasing_drops_the_stale_frame(self, src):
        """A reopened pane must not flash the last frame from the previous session — the
        aircraft has moved, and a stale picture that looks live is the kind of quiet wrongness
        this project keeps paying for."""
        src.chase_acquire()
        src._on_chase(SimpleNamespace(data=b"old"))
        src.chase_release()
        assert src.latest_chase_jpeg() is None
