#!/usr/bin/env python3
"""A recording must last as long as the flight did, and must always close.

T-02's chase/onboard drift, and the three attempts that failed at it. Both causes are
reproduced here with synthetic frames — no simulator, no camera — which is the whole point:
every earlier attempt was made in flight, so each failure cost a whole episode and took a
day to see again.

    ./.venv/bin/python -m pytest tests/test_h264_timing.py -q
"""
from __future__ import annotations

import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "sim_bridge"))

av = pytest.importorskip("av", reason="PyAV is the H.264 path; mp4v fallback has no PTS")
import numpy as np                                                     # noqa: E402

from carla_air.h264 import VideoWriter                                 # noqa: E402

W, H = 160, 120


def _frames(n, rate):
    """`n` frames arriving at `rate` Hz — the real rate, not the nominal one."""
    return [(i / rate, np.full((H, W, 3), (i * 5) % 255, np.uint8)) for i in range(n)]


def _duration(path):
    c = av.open(path)
    try:
        return float(c.duration) / av.time_base if c.duration else 0.0
    finally:
        c.close()


def _count(path):
    c = av.open(path)
    try:
        return sum(1 for _ in c.decode(video=0))
    finally:
        c.close()


def test_the_file_lasts_as_long_as_the_flight(tmp_path):
    """THE regression. A source that cannot hold its nominal rate must still play real-time.

    Nominal 20 fps, actual 13.7 — the shape measured on the real camera (5.58 against a
    nominal 8). Stamped at the nominal rate the file would be 7.0 s for a 10.1 s flight,
    which is where the chase and onboard recordings drift apart.
    """
    path = str(tmp_path / "slow.mp4")
    w = VideoWriter(path, W, H, fps=20.0, crf=30, preset="ultrafast")
    if w.codec != "h264":
        pytest.skip("mp4v fallback: no PTS control")
    data = _frames(140, 13.7)
    for t, arr in data:
        w.write(arr, t_s=t)
    w.close()

    span = data[-1][0] - data[0][0]
    assert abs(_duration(path) - span) < 0.5, (
        f"file is {_duration(path):.2f}s for a {span:.2f}s flight — this is the drift T-02 is "
        "about; a nominal-rate file would be 7.0s")


def test_two_frames_in_one_millisecond_do_not_lose_the_recording(tmp_path):
    """Reproduces `av.error.ArgumentError ... returned 22`, raised by close().

    That is the failure recorded against the first two attempts, and it costs the whole file
    because it lands after everything is encoded.
    """
    path = str(tmp_path / "dup.mp4")
    w = VideoWriter(path, W, H, fps=20.0, crf=30, preset="ultrafast")
    if w.codec != "h264":
        pytest.skip("mp4v fallback")
    for i, t in enumerate([0.0, 0.05, 0.0500001, 0.1, 0.15, 0.2]):
        w.write(np.full((H, W, 3), i * 9, np.uint8), t_s=t)
    w.close()                                    # must not raise
    assert os.path.getsize(path) > 0
    assert _count(path) == 6, "a de-duplicated timestamp must not drop the frame"


def test_a_backwards_timestamp_does_not_lose_the_recording(tmp_path):
    """A stamp can step backwards across a reconnect. It must not cost the episode."""
    path = str(tmp_path / "back.mp4")
    w = VideoWriter(path, W, H, fps=20.0, crf=30, preset="ultrafast")
    if w.codec != "h264":
        pytest.skip("mp4v fallback")
    for i, t in enumerate([0.0, 0.05, 0.10, 0.08, 0.15, 0.20]):
        w.write(np.full((H, W, 3), i * 9, np.uint8), t_s=t)
    w.close()
    assert _count(path) == 6


def test_two_streams_of_one_flight_agree_on_length(tmp_path):
    """The actual complaint: 'the video is out of sync'.

    One flight, two recorders at different nominal AND different real rates — the chase at
    20/15.1 and the onboard at 8/5.58, both measured. Their files must come out the same
    length, because the flight was one flight.
    """
    span = 30.0
    out = {}
    for name, nominal, real in (("chase", 20.0, 15.1), ("onboard", 8.0, 5.58)):
        path = str(tmp_path / f"{name}.mp4")
        w = VideoWriter(path, W, H, fps=nominal, crf=30, preset="ultrafast")
        if w.codec != "h264":
            pytest.skip("mp4v fallback")
        n = int(span * real)
        for i in range(n):
            w.write(np.full((H, W, 3), (i * 3) % 255, np.uint8), t_s=i / real)
        w.close()
        out[name] = _duration(path)
    drift = abs(out["chase"] - out["onboard"])
    assert drift < 0.5, (
        f"chase {out['chase']:.2f}s vs onboard {out['onboard']:.2f}s — {drift:.2f}s apart over "
        f"{span:.0f}s of one flight")


def test_no_timestamps_still_works(tmp_path):
    """Callers that pass no `t_s` keep the old nominal-rate behaviour rather than breaking."""
    path = str(tmp_path / "nominal.mp4")
    w = VideoWriter(path, W, H, fps=10.0, crf=30, preset="ultrafast")
    if w.codec != "h264":
        pytest.skip("mp4v fallback")
    for i in range(20):
        w.write(np.full((H, W, 3), i * 11, np.uint8))
    w.close()
    assert _count(path) == 20
