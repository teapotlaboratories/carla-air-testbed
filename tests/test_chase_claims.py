#!/usr/bin/env python3
"""One chase camera, two owners. R-03 step 4.

    ./.venv/bin/python -m pytest tests/test_chase_claims.py -q

No simulator, no CARLA, no GPU: `Bridge` is driven with its CARLA-touching parts replaced, so
what is under test is the CLAIM ARITHMETIC — who wants the sensor, and what happens when one
of them lets go.

The failure this guards is silent. `ChaseCamera.destroy()` calls `stop()`, which drains the
queue and closes the writer, so destroying the sensor under a running recording does not
corrupt the mp4 — it finalises one that is simply SHORT. A playable file that is quietly
incomplete is worse than a broken one, because nothing anywhere says so.
"""
from __future__ import annotations

import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "sim_bridge"))


class _FakeCamera:
    """Stands in for ChaseCamera. Records whether it was destroyed, and at what spec."""

    def __init__(self, width, height, fps):
        self.width, self.height, self.fps = int(width), int(height), float(fps)
        self.destroyed = False
        self.recording = None

    def start(self, path):
        self.recording = path

    def stop(self):
        self.recording = None
        return {"frames": 7, "dropped": 0}

    def destroy(self):
        self.destroyed = True

    def latest_jpeg(self, quality=75):
        return b"jpeg-bytes"


@pytest.fixture
def bridge(monkeypatch):
    """A real `SimBridge` with only the CARLA-facing pieces replaced.

    `SimBridge.__init__` connects to AirSim and CARLA, so it is bypassed rather than called —
    the claim arithmetic lives entirely in the chase methods and the attributes they read.
    """
    import server as sidecar        # sim_bridge/server.py

    b = sidecar.SimBridge.__new__(sidecar.SimBridge)
    b._chase = None
    b._chase_viewers = 0
    b._chase_recording = False
    # A stand-in for the shared follow thread. It must NOT be None: `chase_stop` only reaches
    # its park-the-follower branch when a thread exists, so leaving this None made the test
    # for that branch assert on code it never ran — caught by mutation-checking it.
    class _FakeThread:
        def __init__(self):
            self.joined = False

        def join(self, timeout=None):
            self.joined = True

    b._chase_thread = _FakeThread()
    b._chase_stop = __import__("threading").Event()
    b._rig = None

    made = []

    def fake_ensure(width, height, fps, distance, above):
        want = (int(width), int(height), float(fps))
        if b._chase is not None and (b._chase.width, b._chase.height, b._chase.fps) != want:
            b._chase.destroy()
            b._chase = None
        if b._chase is None:
            b._chase = _FakeCamera(*want)
            made.append(want)

    monkeypatch.setattr(b, "_ensure_chase", fake_ensure)
    monkeypatch.setattr(b, "_chase_defaults", lambda: {
        "width": 1280, "height": 720, "fps": 30.0, "distance": 14.0, "above": 6.0, "crf": 26})
    return b, made


def test_a_view_brings_the_camera_up_and_releasing_puts_it_away(bridge):
    b, _ = bridge
    assert b._chase is None
    b.chase_view(width=1280, height=720, fps=10.0)
    assert b._chase is not None and b._chase_viewers == 1
    cam = b._chase
    b.chase_release()
    assert b._chase is None and cam.destroyed, "the sensor outlived its only viewer"


def test_two_viewers_need_two_releases(bridge):
    """The count is not a boolean. Two subscribers on the topic, or a topic and a human
    poking the sidecar, must not have the first one to leave take the camera away."""
    b, _ = bridge
    b.chase_view(width=1280, height=720, fps=10.0)
    b.chase_view(width=1280, height=720, fps=10.0)
    cam = b._chase
    b.chase_release()
    assert b._chase is cam and not cam.destroyed, "the first release destroyed a shared camera"
    b.chase_release()
    assert b._chase is None and cam.destroyed


def test_releasing_a_view_never_touches_a_running_recording(bridge):
    """THE test. The subscriber goes away mid-recording and the mp4 must keep its source.

    Getting this wrong does not produce an error or a corrupt file — `destroy()` finalises
    the writer — it produces a playable recording that just stops early.
    """
    b, _ = bridge
    b.chase_start(path="/tmp/x.mp4", width=1920, height=1080, fps=20.0)
    b.chase_view(width=1280, height=720, fps=10.0)
    cam = b._chase
    b.chase_release()
    assert b._chase is cam and not cam.destroyed, (
        "the last subscriber leaving destroyed the sensor under a running recording — the "
        "mp4 would be finalised and silently short")
    assert cam.recording == "/tmp/x.mp4"


def test_a_viewer_adopts_the_running_recordings_spec(bridge):
    """The spec-conflict rule, decided 2026-08-11.

    A CARLA sensor's resolution and tick are fixed at spawn, so honouring both owners means
    respawning — which truncates the file being written. The recording wins; the viewer is
    told what it actually got.
    """
    b, made = bridge
    b.chase_start(path="/tmp/x.mp4", width=1920, height=1080, fps=20.0)
    spawned_for_recording = len(made)
    got = b.chase_view(width=1280, height=720, fps=10.0)
    assert len(made) == spawned_for_recording, "the viewer respawned the sensor mid-recording"
    assert got["size"] == [1920, 1080] and got["fps"] == 20.0, got
    assert got["adopted_recording_spec"] is True, "the caller was not told it was overridden"


def test_a_viewer_with_no_recording_gets_the_spec_it_asked_for(bridge):
    """The adoption must not leak into the ordinary case — otherwise the topic would quietly
    ignore its own configured resolution forever."""
    b, _ = bridge
    got = b.chase_view(width=640, height=480, fps=5.0)
    assert got["size"] == [640, 480] and got["fps"] == 5.0
    assert got["adopted_recording_spec"] is False


def test_stopping_a_recording_leaves_a_viewers_camera_alone(bridge):
    b, _ = bridge
    b.chase_view(width=1280, height=720, fps=10.0)
    b.chase_start(path="/tmp/x.mp4", width=1280, height=720, fps=10.0)
    cam = b._chase
    b.chase_stop()
    assert b._chase is cam and not cam.destroyed
    assert b._chase_recording is False


def test_stopping_a_recording_does_not_park_the_shared_follower_while_watched(bridge):
    """A bug older than this item, fixed by the same condition.

    `_chase_follow` drives the chase camera AND `self._rig` — every CARLA sensor in
    carla_sensors.yaml — from one pose read. Stopping it unconditionally meant ending any
    chase recording froze all of those sensors at the last pose, still publishing, from a
    camera that no longer followed the aircraft.
    """
    b, _ = bridge
    b.chase_view(width=1280, height=720, fps=10.0)
    b.chase_start(path="/tmp/x.mp4", width=1280, height=720, fps=10.0)
    b._chase_stop.clear()
    b.chase_stop()
    assert not b._chase_stop.is_set(), (
        "the shared follow thread was parked while a viewer was still watching — every CARLA "
        "sensor would stop following the aircraft too")


def test_releasing_when_nothing_was_taken_is_harmless(bridge):
    b, _ = bridge
    assert b.chase_release()["camera"] == "destroyed"
    assert b._chase_viewers == 0


def test_chase_jpeg_survives_a_release_racing_it(bridge):
    """`chase_jpeg` runs under media_lock while the claim methods mutate `_chase` under
    slow_lock. Reading the attribute once is what keeps a release between the None-check and
    the dereference from raising."""
    b, _ = bridge
    b.chase_view(width=1280, height=720, fps=10.0)
    assert b.chase_jpeg()["jpeg"] == b"jpeg-bytes"
    b.chase_release()
    assert b.chase_jpeg()["jpeg"] is None
