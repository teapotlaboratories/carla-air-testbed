#!/usr/bin/env python3
"""A reply nobody is waiting for must never become the next call's answer.

    ./.venv/bin/python -m pytest tests/test_rpc_correlation.py -q

This is the regression that killed a 40-episode sweep. `SimBridgeClient.call` used to send a
frame and block reading the next one, on the assumption that the next frame off the socket
was its own reply. The moment any caller gave up early - a slow `reset` against a 60 s socket
timeout - the abandoned reply stayed buffered, and every later call read the PREVIOUS call's
answer. Forever, with no resync. `state()` unpacked `reset()`'s reply, raised
`KeyError: 'position'`, and rclpy took the whole bridge node down with it.

The tests run against a fake sidecar over a real socketpair, so they exercise the actual
framing and the actual reader thread without needing carla, airsim, or a simulator.
See docs/rpc-path.html.
"""
from __future__ import annotations

import importlib.util
import os
import socket
import threading
import time

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLIENT_PY = os.path.join(ROOT, "ros2_ws", "src", "carla_air_bridge", "carla_air_bridge",
                         "client.py")


@pytest.fixture(scope="module")
def mod():
    """Load the client module directly - importing the ROS package would need a workspace."""
    os.environ.setdefault("TESTBED_PROTOCOL", os.path.join(ROOT, "sim_bridge", "protocol.py"))
    spec = importlib.util.spec_from_file_location("carla_air_bridge_client", CLIENT_PY)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


class FakeSidecar:
    """The server half of a socketpair, driven by the test."""

    def __init__(self, mod):
        self.protocol = mod.protocol
        self.srv, self.cli = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
        self.requests = []
        self._stop = threading.Event()

    def recv_request(self, timeout=5.0):
        self.srv.settimeout(timeout)
        req = self.protocol.recv(self.srv)
        self.requests.append(req)
        return req

    def reply(self, rid, result=None, ok=True, error="boom"):
        frame = {"id": rid, "ok": ok}
        if ok:
            frame["result"] = result
        else:
            frame["error"] = error
        self.protocol.send(self.srv, frame)

    def progress(self, rid, stage):
        self.protocol.send(self.srv, {"id": rid, "progress": stage})

    def close(self):
        for s in (self.srv, self.cli):
            try:
                s.close()
            except OSError:
                pass


def attach(mod, fake, patience=2.0):
    """A client wired to the fake, with its reader thread running."""
    c = mod.SimBridgeClient(path="/nonexistent", timeout=patience)
    c._sock = fake.cli
    c._closing = False
    c._dead = None
    c._reader = threading.Thread(target=c._read_loop, daemon=True)
    c._reader.start()
    return c


def test_an_abandoned_reply_does_not_become_the_next_answer(mod):
    """THE regression. Give up on call 1, then check call 2 gets call 2's result."""
    fake = FakeSidecar(mod)
    c = attach(mod, fake, patience=0.6)
    try:
        with pytest.raises(mod.SimBridgeError) as first:
            c.call("state")                      # never answered -> caller gives up
        assert "no reply and no progress" in str(first.value)

        # The request is sitting unread in the fake's buffer; read it now to learn its id.
        rid1 = fake.recv_request()["id"]
        fake.reply(rid1, {"position": [1.0, 2.0, 3.0]})   # the abandoned reply, arriving late
        time.sleep(0.2)                                   # let the reader consume and drop it

        threading.Thread(
            target=lambda: (fake.recv_request(), fake.reply(fake.requests[-1]["id"],
                                                            {"collided": False})),
            daemon=True).start()
        assert c.call("collision") == {"collided": False}, (
            "the second call received the first call's reply - the stream desynchronised")
        assert c.late_replies >= 1, "the late reply should have been counted and dropped"
    finally:
        c.close(); fake.close()


def test_progress_extends_patience_so_slow_is_not_dead(mod):
    """A call that keeps reporting may outlive the patience window many times over."""
    fake = FakeSidecar(mod)
    c = attach(mod, fake, patience=1.2)
    try:
        def sidecar():
            rid = fake.recv_request()["id"]
            for stage in ("sim-reset", "placing", "arming", "holding"):
                time.sleep(0.8)                  # each gap alone is within patience...
                fake.progress(rid, stage)        # ...and each frame resets the clock
            time.sleep(0.8)
            fake.reply(rid, {"position": [0.0, 0.0, -55.0]})

        threading.Thread(target=sidecar, daemon=True).start()
        t0 = time.monotonic()
        got = c.call("reset", hold_ned=[0.0, 0.0, -55.0])
        elapsed = time.monotonic() - t0

        assert got["position"] == [0.0, 0.0, -55.0]
        assert elapsed > 1.2, (
            f"call returned in {elapsed:.1f}s; it must outlive the {c.timeout}s patience "
            "because progress kept arriving")
    finally:
        c.close(); fake.close()


def test_silence_still_fails_promptly(mod):
    """The other half of the contract: no progress means no extension."""
    fake = FakeSidecar(mod)
    c = attach(mod, fake, patience=0.8)
    try:
        t0 = time.monotonic()
        with pytest.raises(mod.SimBridgeError):
            c.call("state")
        assert time.monotonic() - t0 < 3.0, "a wedged call must fail near its patience window"
    finally:
        c.close(); fake.close()


def test_a_dead_connection_fails_waiting_calls_rather_than_hanging(mod):
    """If the sidecar dies mid-call, the caller learns immediately - it used to wait out the
    full socket timeout, and every later call inherited a broken stream."""
    fake = FakeSidecar(mod)
    c = attach(mod, fake, patience=30.0)
    try:
        threading.Thread(target=lambda: (fake.recv_request(), time.sleep(0.2),
                                         fake.srv.close()), daemon=True).start()
        t0 = time.monotonic()
        with pytest.raises(mod.SimBridgeError):
            c.call("state")
        assert time.monotonic() - t0 < 5.0, "should fail on EOF, not wait out the patience"
    finally:
        c.close(); fake.close()


def test_errors_still_carry_the_remote_traceback(mod):
    """Behaviour that must survive the rewrite."""
    fake = FakeSidecar(mod)
    c = attach(mod, fake)
    try:
        threading.Thread(
            target=lambda: (fake.recv_request(),
                            fake.protocol.send(fake.srv, {"id": fake.requests[-1]["id"],
                                                          "ok": False, "error": "nope",
                                                          "traceback": "TB HERE"})),
            daemon=True).start()
        with pytest.raises(mod.SimBridgeError) as exc:
            c.call("state")
        assert "nope" in str(exc.value)
        assert exc.value.remote_traceback == "TB HERE"
    finally:
        c.close(); fake.close()
