#!/usr/bin/env python3
"""Everything checkable without a simulator, a GPU or a display.

The conformance suite needs 18 GB of Unreal and fifteen minutes of real-time flight. This
file needs neither, so it can run on every change: the wire protocol, the frame maths and
the grounding geometry are pure functions and a bug in any of them is a bug in every
episode.

    ./.venv/bin/python -m pytest tests/test_offline.py -q

Runs under the 3.10 client venv. The ROS 2 nodes are not importable here — they need Jazzy's
3.12 — which is precisely the split this testbed is built around; their logic lives in
`sim_bridge/carla_air/` on purpose so it can be tested here.
"""
from __future__ import annotations

import math
import os
import socket
import sys
import threading

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "sim_bridge"))

import protocol  # noqa: E402
from carla_air.frames import (  # noqa: E402
    DEFAULT_OFFSET,
    Intrinsics,
    calibrate,
    carla_to_ned,
    ned_to_carla,
    quat_to_matrix,
    quat_to_yaw,
    unproject,
)

# --------------------------------------------------------------------------- frames


def test_carla_ned_roundtrip():
    for pt in [(0, 0, 0), (-47.2, 13.8, 0.0), (120.5, -3.25, 12.0)]:
        assert ned_to_carla(*carla_to_ned(*pt)) == pytest.approx(pt, abs=1e-9)


def test_town10hd_offset_matches_the_measured_value():
    # Upstream's documented constants, re-measured in tests/conformance/p06.
    assert carla_to_ned(-47.2, 13.8, 0.0) == pytest.approx((125.0, -170.06, 27.45), abs=0.01)


def test_z_axis_flips():
    """CARLA is z-up, NED is z-down. Getting this wrong inverts every altitude."""
    _, _, up10 = carla_to_ned(0.0, 0.0, 10.0)
    _, _, ground = carla_to_ned(0.0, 0.0, 0.0)
    assert up10 < ground


def test_calibrate_recovers_a_known_offset():
    carla_pt = (10.0, -5.0, 2.0)
    ned_pt = carla_to_ned(*carla_pt)
    assert calibrate(carla_pt, ned_pt) == pytest.approx(DEFAULT_OFFSET, abs=1e-9)


# ----------------------------------------------------------------------- intrinsics


def test_intrinsics_are_a_pinhole_with_square_pixels():
    i = Intrinsics(640, 480, 90.0)
    assert i.fx == pytest.approx(320.0)     # 90 deg hfov ⇒ fx == width/2
    assert i.fy == pytest.approx(i.fx)
    assert (i.cx, i.cy) == (320.0, 240.0)


def test_depth_scale_between_rgb_and_depth_frames():
    """The whole reason depth is rendered smaller — see docs/architecture.md."""
    rgb = Intrinsics(640, 480, 90.0)
    depth = Intrinsics(320, 240, 90.0)
    assert rgb.scale_to(depth) == (0.5, 0.5)
    assert rgb.width / rgb.height == pytest.approx(depth.width / depth.height)


# ------------------------------------------------------------------------ grounding


IDENT = (1.0, 0.0, 0.0, 0.0)


def test_centre_pixel_unprojects_straight_ahead():
    i = Intrinsics(640, 480, 90.0)
    p = unproject(i.cx, i.cy, 50.0, i, (0.0, 0.0, -30.0), IDENT)
    assert p == pytest.approx((50.0, 0.0, -30.0), abs=1e-6)


def test_depth_is_planar_not_range():
    """A corner pixel must land FURTHER than `depth` metres away.

    AirSim's DepthPerspective is planar z-depth. Treating it as a range shortens every
    waypoint by sec(angle off-axis) — at the frame edge of a 90 deg lens that is 40%.
    """
    i = Intrinsics(640, 480, 90.0)
    origin = (0.0, 0.0, 0.0)
    p = unproject(0, 0, 50.0, i, origin, IDENT)
    assert math.dist(p, origin) > 50.0
    assert p[0] == pytest.approx(50.0)        # forward component IS the depth


def test_yaw_rotation_moves_the_waypoint():
    i = Intrinsics(640, 480, 90.0)
    facing_north = unproject(i.cx, i.cy, 20.0, i, (0.0, 0.0, -10.0), IDENT)
    q90 = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))   # yaw +90 deg
    facing_east = unproject(i.cx, i.cy, 20.0, i, (0.0, 0.0, -10.0), q90)
    assert facing_north == pytest.approx((20.0, 0.0, -10.0), abs=1e-6)
    assert facing_east == pytest.approx((0.0, 20.0, -10.0), abs=1e-6)


def test_pitched_camera_points_down_not_at_the_horizon():
    """The failure this guards against is silent: a yaw-only rotation puts every
    waypoint on the horizon even though the camera is looking at the ground."""
    i = Intrinsics(640, 480, 90.0)
    pitch = -0.5  # rad, camera nose-down
    q = (math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0)
    p = unproject(i.cx, i.cy, 50.0, i, (0.0, 0.0, -50.0), q)
    assert p[2] > -50.0, "a down-pitched camera must ground BELOW the aircraft"


def test_quat_helpers_agree():
    for yaw in (-2.0, -0.3, 0.0, 1.1, 3.0):
        q = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
        assert quat_to_yaw(*q) == pytest.approx(yaw, abs=1e-9)
        r = quat_to_matrix(*q)
        assert r[0][0] == pytest.approx(math.cos(yaw), abs=1e-9)


# ------------------------------------------------------------------------- protocol


def test_frame_roundtrip_over_a_real_socket():
    a, b = socket.socketpair()
    payload = {"id": 7, "method": "state", "args": {"x": 1.5, "flag": True}}
    protocol.send(a, payload)
    assert protocol.recv(b) == payload
    a.close()
    b.close()


def test_large_frames_survive_fragmentation():
    """Images are megabytes; a recv() that assumes one chunk silently truncates them."""
    import numpy as np

    a, b = socket.socketpair()
    img = np.arange(640 * 480 * 3, dtype=np.uint8).reshape(480, 640, 3)
    got = {}

    def reader():
        got["msg"] = protocol.recv(b)

    t = threading.Thread(target=reader)
    t.start()
    protocol.send(a, {"image": protocol.encode_image(img)})
    t.join(timeout=10)

    out = protocol.decode_image(got["msg"]["image"])
    assert out.shape == img.shape and (out == img).all()
    a.close()
    b.close()


def test_encode_image_passes_none_through():
    assert protocol.encode_image(None) is None
    assert protocol.decode_image(None) is None


def test_method_table_matches_the_server():
    """A method the client can name but the server cannot serve is a runtime-only bug."""
    import server  # noqa: PLC0415 — imported here so the module list is the live one

    for method in protocol.METHODS:
        assert hasattr(server.SimBridge, method), f"server has no handler for {method!r}"


def test_the_three_way_client_split_holds():
    """Telemetry, control and media must not share an AirSim client.

    Each collapse was measured. Capture in the telemetry set drops odometry from 14 Hz to
    1.5 Hz; control in the telemetry set caps it at 12.3 Hz because 20 Hz of state() and
    10 Hz of velocity() serialise through one RPC client.
    """
    import server  # noqa: PLC0415

    fast, control = server.SimBridge.FAST, server.SimBridge.CONTROL
    assert {"state", "collision"} <= fast, "telemetry reads must be on the fast path"
    assert {"velocity", "goto"} <= control, "control writes must be on the control path"
    assert "capture" not in fast and "capture" not in control, "capture must be on neither"
    assert not (fast & control), f"a method cannot be both: {fast & control}"


def test_chase_follow_does_not_share_a_client_with_telemetry():
    """The chase follower must not drive `airsim_client`.

    The locks in SimBridge guard dispatch CLASSES, not clients: `state` is FAST (fast_lock)
    while `reset` is neither FAST nor CONTROL (slow_lock), and both drive `airsim_client`.
    Two threads under two different locks can therefore write one msgpack-rpc socket at
    once. It surfaces from deep inside tornado as

        RuntimeError: Existing exports of data: object cannot be re-sized

    which names nothing about the real cause, and it cost a flight test to diagnose. The
    follow thread runs continuously at 20 Hz, so it is the one caller that genuinely races
    everything else — it gets its own connection.
    """
    import inspect  # noqa: PLC0415

    import server  # noqa: PLC0415

    # Strip the docstring: it names both locks in order to explain why neither is taken,
    # and matching prose rather than code is how an assertion starts lying.
    source = inspect.getsource(server.SimBridge._chase_follow)
    body = "\n".join(line for line in source.splitlines()
                     if not line.strip().startswith(("#", '"""')))
    body = body.split('"""')[-1]

    assert "self.vehicle" not in body, (
        "the chase follower is reading the shared telemetry vehicle; give it "
        "_chase_vehicle on its own client")
    assert "_chase_vehicle" in body
    # And it must not be routed through a dispatch lock it does not own.
    assert "with self.fast_lock" not in body and "with self.slow_lock" not in body


def test_media_never_shares_a_lock_with_blocking_commands():
    """Video must not queue behind `land`, `reset` or `goto`.

    Reported as "pressing land breaks the web stream". `land` blocks for the whole descent,
    and it held the same `slow_lock` that `view_jpeg` and `chase_jpeg` needed — so both
    streams froze for tens of seconds. Frames come off the media AirSim client or off CARLA
    and touch neither the telemetry nor the control socket, so they get their own lock.
    """
    import server  # noqa: PLC0415

    media = server.SimBridge.MEDIA
    assert {"capture", "view_jpeg", "chase_jpeg"} <= media
    # Disjoint from every other dispatch class, or it lands on the wrong lock.
    assert not (media & server.SimBridge.FAST)
    assert not (media & server.SimBridge.CONTROL)
    # And a blocking command must never be classed as media.
    for blocking in ("reset", "goto", "land", "spawn_traffic"):
        assert blocking not in media, f"{blocking} blocks; it cannot share the media lock"


def test_land_is_a_control_command_not_a_slow_one():
    """`land` drove the TELEMETRY client while `state` drove the same socket under a
    different lock — the two-locks-one-socket race that produced 'Existing exports of data'.
    It belongs on the control client, and therefore on the control path."""
    import server  # noqa: PLC0415

    assert "land" in server.SimBridge.CONTROL

    import inspect  # noqa: PLC0415
    body = inspect.getsource(server.SimBridge.land)
    body = body.split('"""')[-1]           # drop the docstring, which names both clients
    assert "self.vehicle" not in body, "land must not drive the telemetry client"
    assert "self.control" in body


def test_chase_methods_are_not_on_the_fast_or_control_paths():
    """Chase calls touch CARLA and spawn a connection; they belong on neither hot path."""
    import server  # noqa: PLC0415

    for method in ("chase_start", "chase_stop"):
        assert method not in server.SimBridge.FAST
        assert method not in server.SimBridge.CONTROL


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
