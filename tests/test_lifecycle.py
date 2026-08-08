#!/usr/bin/env python3
"""The console's start/stop buttons must drive the deployment that is actually running.

    ./.venv/bin/python -m pytest tests/test_lifecycle.py -q

R-03 step 3. These three buttons are R-03's permanent carve-out — they can never be ROS calls,
because there is no graph before the simulator exists and the stop button's job is to destroy
the graph. What changes here is *which* processes they reach.

They always drove the host scripts. Against the containerised stack that made **Start actively
wrong**: `run_sim.sh` brings up a host-native `CarlaUE4`, so pressing Start while the stack is
running gives a second simulator competing for GPU 1 with the one being watched.

No Docker, no simulator, no graph — the decision is pure so it can be tested here.
"""
from __future__ import annotations

import importlib.util
import os
import sys
from types import SimpleNamespace

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "webui"))

import lifecycle  # noqa: E402


def test_the_stack_being_up_selects_the_container_deployment():
    assert lifecycle.deployment(stack_running=True) == lifecycle.CONTAINER


def test_no_stack_means_host():
    assert lifecycle.deployment(stack_running=False) == lifecycle.HOST


class _FakeRun:
    """Records what `Processes._run` was asked to execute, and answers with a chosen result."""

    def __init__(self, returncode=0):
        self.calls, self.returncode = [], returncode

    def __call__(self, argv, env=None, background=False):
        self.calls.append(list(argv))
        return SimpleNamespace(returncode=self.returncode, stdout="ok", stderr="")


@pytest.fixture
def procs(monkeypatch):
    """A real `Processes`, with only the subprocess layer replaced.

    These tests drive the SHIPPED methods. An earlier version asserted against a
    `lifecycle.scripts_for()` lookup table that `server.py` never called — three green tests
    measuring a parallel structure, which had already drifted from the code it claimed to
    describe. That table is gone.
    """
    # Loaded by PATH, not by name. `sim_bridge/server.py` is also called `server` and is on
    # sys.path via webui/server.py's own imports, so a plain `import server` picks up the
    # sidecar instead and fails with "module 'server' has no attribute 'Processes'".
    #
    # sys.path is snapshotted and restored: webui/server.py's module body inserts `webui/`
    # ahead of `sim_bridge/`, and leaving that in place made every OTHER test that imports
    # `server` pick up the console instead of the sidecar. Six unrelated failures, caused
    # entirely by this fixture.
    saved_path, saved_modules = list(sys.path), set(sys.modules)
    try:
        spec = importlib.util.spec_from_file_location(
            "webui_server_under_test", os.path.join(ROOT, "webui", "server.py"))
        webui_server = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(webui_server)
        p = webui_server.Processes()
        fake = _FakeRun()
        monkeypatch.setattr(p, "_run", fake)
        yield p, fake, webui_server
    finally:
        sys.path[:] = saved_path
        for name in set(sys.modules) - saved_modules:
            del sys.modules[name]


def test_start_against_a_running_stack_launches_nothing(procs, monkeypatch):
    """The defect this step exists to fix. `run_sim.sh` would bring up a host-native CarlaUE4
    beside the containerised one, competing for GPU 1 with the simulator being watched."""
    p, fake, _ = procs
    monkeypatch.setattr(p, "stack_running", lambda: True)
    result = p.start()
    assert result["deployment"] == "container"
    assert fake.calls == [], f"Start ran {fake.calls} while the stack was already up"


def test_stop_simulator_removes_the_container_when_the_stack_is_up(procs, monkeypatch):
    """`run_sim.sh --kill` matches a host process NAME, so against the stack it would report
    success while stopping nothing."""
    p, fake, _ = procs
    monkeypatch.setattr(p, "stack_running", lambda: True)
    assert p.stop_simulator()["deployment"] == "container"
    assert fake.calls[0][:3] == ["docker", "rm", "-f"], fake.calls


def test_stop_simulator_uses_the_host_script_without_a_stack(procs, monkeypatch):
    p, fake, _ = procs
    monkeypatch.setattr(p, "stack_running", lambda: False)
    assert p.stop_simulator()["deployment"] == "host"
    assert fake.calls[0][-1] == "--kill" and "run_sim.sh" in fake.calls[0][0]


def test_a_failed_docker_rm_is_reported_as_a_failure(procs, monkeypatch):
    """`stop.sh` announced success twice on 2026-08-03 while the simulator held 3.5 GB of
    VRAM, because nothing checked the result. Reintroducing that one file over would be worse
    than never having fixed it."""
    p, fake, _ = procs
    monkeypatch.setattr(p, "stack_running", lambda: True)
    fake.returncode = 1
    with pytest.raises(RuntimeError, match="STILL RUNNING"):
        p.stop_simulator()


class TestRefusingToDestroyItself:
    """With `--in-stack` the console's container joins the simulator container's namespaces, so
    stopping the simulator kills the console mid-request. The browser sees a dropped connection,
    not an error — destructive AND silent about it, which is the worst failure shape there is."""

    @pytest.mark.parametrize("action", ["stop_all", "stop_sim"])
    def test_stopping_is_refused_from_inside_the_stack(self, action):
        why = lifecycle.refusal(action, in_stack=True)
        assert why is not None, f"{action} from inside the stack would kill this server"
        assert "host" in why, "did not say where to run it instead"

    def test_starting_is_still_allowed_from_inside(self):
        """Only self-destruction is refused. A guard that blocks more than it must gets
        worked around, and then it protects nothing."""
        assert lifecycle.refusal("start", in_stack=True) is None

    @pytest.mark.parametrize("action", ["start", "stop_all", "stop_sim"])
    def test_nothing_is_refused_when_not_inside_the_stack(self, action):
        """The normal case, and the one a marker-file check would have broken: this project
        runs inside a podman container, so `/run/.containerenv` exists for every ordinary
        console too. Detection had to be explicit."""
        assert lifecycle.refusal(action, in_stack=False) is None


def test_in_stack_is_not_inferred_from_a_container_marker():
    """Pins the reasoning above against a well-meaning future simplification.

    Inferring it from `/run/.containerenv` or `/.dockerenv` looks tidier and is wrong here —
    both are present for the ordinary development environment on this machine, so the stop
    button would refuse always.
    """
    with open(os.path.join(ROOT, "webui", "server.py")) as fh:
        src = fh.read()
    assert 'os.environ.get("TESTBED_IN_STACK")' in src
    for marker in ("/run/.containerenv", "/.dockerenv"):
        assert f'exists("{marker}")' not in src, (
            f"in-stack inferred from {marker}; that is true for every console on this machine")


def test_being_inside_the_stack_is_proof_the_stack_is_running():
    """The bug that only the live `--in-stack` run exposed.

    `stack_running()` shells out to `docker ps`, and **the ROS image ships no docker binary**.
    So a console inside the stack got `OSError`, concluded there was no stack, took the host
    path, and offered to start a second simulator. Every unit test passed throughout, because
    they inject `stack_running` as a parameter instead of computing it — the detection was the
    one part never covered.
    """
    with open(os.path.join(ROOT, "webui", "server.py")) as fh:
        src = fh.read()
    body = src[src.index("def stack_running"):src.index("def _guard")]
    # Anchored on the CALL, not the word "docker" — which now appears in the docstring
    # explaining this very trap, and matched there on the first attempt at this test.
    assert "if self.IN_STACK:" in body, "the shortcut is gone"
    assert body.index("if self.IN_STACK:") < body.index("subprocess.run"), (
        "stack_running() shells out before checking IN_STACK; inside the stack there is no "
        "docker binary, so it would answer 'no stack' from inside the stack")
