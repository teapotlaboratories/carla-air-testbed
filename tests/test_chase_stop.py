#!/usr/bin/env python3
"""`ChaseCamera.stop()` must never block, however full its queue is.

D-04: `destroy: no response after 30.0s` with the simulator, sidecar and bridge all still
running and the socket present — a wedge invisible to `status.sh`. The SIGUSR1 stack dump
found `chase_stop` parked in `queue.put`, holding the sidecar's slow lock, with every other
slow-class call queued behind it forever.

The race: `stop()` sets `self._writer = None`, and `_drain` exits the moment it sees that —
so the sentinel that follows has no consumer. CARLA's sensor thread has meanwhile filled a
BOUNDED queue, so the put blocks with nobody to drain it. Intermittent, because it needs the
queue to be full at that instant, which happens after an episode long enough for the encoder
to fall behind.

No simulator, no CARLA, no encoder: the queue and the exit condition are the whole bug.

    ./.venv/bin/python -m pytest tests/test_chase_stop.py -q
"""
from __future__ import annotations

import os
import queue
import sys
import threading

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "sim_bridge"))


@pytest.fixture
def chase():
    """A ChaseCamera with its CARLA half never constructed — only stop() is under test."""
    from carla_air.chase import ChaseCamera
    c = ChaseCamera.__new__(ChaseCamera)
    c._lock = threading.Lock()
    c._queue = queue.Queue(maxsize=4)
    c._thread = None
    c._writer = None
    c.frames_written = 0
    c.frames_dropped = 0
    c.fps = 20.0
    return c


class _Writer:
    codec = "h264"

    def close(self):
        pass


def test_stop_returns_when_the_queue_is_full_and_nothing_drains_it(chase):
    """THE regression. Before the fix this blocks forever and takes the slow lock with it."""
    chase._writer = _Writer()
    while True:                       # fill it, exactly as the sensor thread would
        try:
            chase._queue.put_nowait((object(), 0.0))
        except queue.Full:
            break
    assert chase._queue.full()

    done = threading.Event()
    result = {}

    def call():
        result["got"] = chase.stop()
        done.set()

    threading.Thread(target=call, daemon=True).start()
    assert done.wait(timeout=10.0), (
        "stop() blocked on a full queue with no drain thread — this is D-04, and in the "
        "sidecar it holds the slow lock so destroy/spawn never return")
    assert result["got"]["frames"] == 0


def test_stop_is_idempotent(chase):
    assert chase.stop() == {"frames": 0, "dropped": 0}
    assert chase.stop() == {"frames": 0, "dropped": 0}


def test_stop_leaves_the_queue_empty_for_the_next_recording(chase):
    """A full queue of 1080p buffers held until the next start is pure resident memory."""
    chase._writer = _Writer()
    for _ in range(4):
        try:
            chase._queue.put_nowait((object(), 0.0))
        except queue.Full:
            break
    chase.stop()
    assert chase._queue.empty()


def test_a_live_drain_thread_still_gets_its_sentinel(chase):
    """The fix must not break the ordinary path: a running drain thread exits on the sentinel."""
    chase._writer = _Writer()
    seen = []

    def drain():
        while True:
            item = chase._queue.get()
            if item is None:
                seen.append("sentinel")
                return
            seen.append("frame")

    chase._thread = threading.Thread(target=drain, daemon=True)
    chase._thread.start()
    chase.stop()
    assert "sentinel" in seen, "the drain thread never received its stop sentinel"
