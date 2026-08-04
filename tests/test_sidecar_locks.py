#!/usr/bin/env python3
"""Every sidecar method must dispatch under the lock that owns the client it touches.

    ./.venv/bin/python -m pytest tests/test_sidecar_locks.py -q

`sim_bridge/server.py` runs three AirSim connections behind three locks. The dispatcher
picks a lock from the METHOD NAME, not from what the method actually does - so a method can
hold one lock while writing a socket another lock is meant to protect. Two threads, one
msgpack-rpc connection.

This has bitten five times, and every symptom named something other than the cause:

    Existing exports of data: object cannot be re-sized     (from inside tornado)
    IOLoop is already running                               (on reset, mid-sweep)
    carla::client::TimeoutException -> terminate()          (the whole sidecar, twice)

The rule is mechanical, so it can be checked mechanically. This is that check. It parses the
source rather than importing it, because importing `server.py` needs `carla` and `airsim` -
which is exactly why this went unguarded for so long.
"""
from __future__ import annotations

import os
import re

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SERVER = os.path.join(ROOT, "sim_bridge", "server.py")

#: attribute -> the dispatch class whose lock protects its underlying AirSim client
OWNER = {"self.vehicle": "FAST", "self.control": "CONTROL", "self.camera": "MEDIA"}


@pytest.fixture(scope="module")
def src():
    with open(SERVER) as fh:
        return fh.read()


@pytest.fixture(scope="module")
def classes(src):
    out = {}
    for name in ("FAST", "CONTROL", "MEDIA"):
        m = re.search(rf"^    {name} = frozenset\(\{{(.*?)\}}\)", src, re.S | re.M)
        assert m, f"could not find the {name} frozenset in server.py"
        out[name] = set(re.findall(r'"([a-z_]+)"', m.group(1)))
    return out


def methods(src):
    """(name, body) for each method of SimBridge, private ones included."""
    return [(m.group(1), m.group(2)) for m in re.finditer(
        r"\n    def ([a-z_][a-z_0-9]*)\(self[^)]*\):(.*?)(?=\n    def |\nclass |\Z)", src, re.S)]


def test_every_method_dispatches_under_its_clients_lock(src, classes):
    offenders = []
    for name, body in methods(src):
        if name.startswith("_"):
            continue                      # not reachable through the dispatcher
        for attr, required in OWNER.items():
            if attr + "." not in body:
                continue
            actual = next((c for c in classes if name in classes[c]), "slow")
            if actual != required:
                offenders.append(
                    f"{name}() touches {attr}, so it must be in {required}; it is in {actual}")
    assert not offenders, (
        "lock class does not match the client the method drives:\n  "
        + "\n  ".join(offenders)
        + "\n\nTwo threads can then write one msgpack-rpc connection. See the note on "
          "CONTROL in server.py.")


def test_reset_is_a_control_operation(classes):
    """Specifically pinned: `reset` drove the telemetry client under slow_lock and killed a
    40-episode sweep. It commands the vehicle, so CONTROL is also right semantically."""
    assert "reset" in classes["CONTROL"]


def test_the_classes_do_not_overlap(classes):
    """A method in two classes would take whichever lock the dispatcher checks first, which
    is an ordering dependency nobody would think to look for."""
    seen = {}
    for cls, names in classes.items():
        for n in names:
            assert n not in seen, f"{n!r} is in both {seen[n]} and {cls}"
            seen[n] = cls
