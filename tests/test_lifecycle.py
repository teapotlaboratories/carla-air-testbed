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

import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "webui"))

import lifecycle  # noqa: E402


def test_the_stack_being_up_selects_the_container_deployment():
    assert lifecycle.deployment(stack_running=True) == lifecycle.CONTAINER


def test_no_stack_means_host():
    assert lifecycle.deployment(stack_running=False) == lifecycle.HOST


def test_start_never_runs_the_host_simulator_against_a_running_stack():
    """The defect this step exists to fix, stated as the thing that must not happen."""
    scripts = lifecycle.scripts_for(lifecycle.CONTAINER)
    assert "scripts/run_sim.sh" not in scripts["start"], (
        "Start would launch a host-native CarlaUE4 beside the containerised one, competing "
        "for GPU 1 with the simulator the operator is actually watching")
    assert scripts["start"][0] == "scripts/stack_up.sh"


def test_stop_simulator_uses_docker_against_a_stack():
    """`run_sim.sh --kill` matches a host process NAME, so against the stack it silently
    succeeds while stopping nothing — the worst outcome for a button."""
    assert lifecycle.scripts_for(lifecycle.CONTAINER)["stop_sim"][0] == "docker"
    assert lifecycle.scripts_for(lifecycle.HOST)["stop_sim"][0] == "scripts/run_sim.sh"


def test_stop_everything_is_the_same_script_either_way():
    """Deliberately not branched. `stop.sh --all` kills the host processes AND `docker rm -f`s
    the container, so one button already covers both — adding a branch would be a second
    implementation to drift."""
    assert (lifecycle.scripts_for(lifecycle.HOST)["stop_all"]
            == lifecycle.scripts_for(lifecycle.CONTAINER)["stop_all"]
            == ["scripts/stop.sh", "--all"])


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
