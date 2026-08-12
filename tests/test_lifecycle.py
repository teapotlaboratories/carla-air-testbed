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
        #: What `docker ps -a` reports. The teardown's verdict comes from ASKING whether the
        #: container is gone, not from a return code, so the fake has to answer that question.
        self.containers = ""       # what `docker ps` reports (running)
        self.containers_all = ""   # what `docker ps -a` reports (running + stopped)
        #: What a `docker rm` actually ACHIEVES, which is the whole subject of these tests.
        #: A fake that always answered the same thing before and after the removal could not
        #: tell "gone" from "still there", so every check would have been asserting on a
        #: constant.
        #:
        #:   "remove"    the container goes away, as it does when this works
        #:   "stop_only" it stops but is not removed - holds no VRAM, object still present
        #:   "nothing"   the removal does not take, and it is still running
        self.rm_effect = "remove"

    def __call__(self, argv, env=None, background=False):
        self.calls.append(list(argv))
        if argv[:2] == ["docker", "rm"]:
            name = argv[-1]
            if self.rm_effect in ("remove", "stop_only"):
                self.containers = " ".join(
                    c for c in self.containers.split() if c != name)
            if self.rm_effect == "remove":
                self.containers_all = " ".join(
                    c for c in self.containers_all.split() if c != name)
        # `docker ps` (running) and `docker ps -a` (exists) are asked separately now.
        if "ps" in argv:
            out = self.containers_all if "-a" in argv else self.containers
        else:
            out = "ok"
        return SimpleNamespace(returncode=self.returncode, stdout=out, stderr="")


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


def test_stop_simulator_removes_the_container_when_one_is_running(procs):
    """`run_sim.sh --kill` matches a host process NAME, so against the stack it would report
    success while stopping nothing.

    T-08: the lane is decided by the container actually RUNNING, not by `stack_running()`, so
    the fake states it rather than the answer being injected.
    """
    p, fake, _ = procs
    fake.containers = fake.containers_all = "carla-air-sim"
    assert p.stop_simulator()["deployment"] == "container"
    # Not calls[0] any more: T-06 put a graceful `docker stop` in front of the removal, and
    # T-08 put the state probe in front of that.
    assert ["docker", "rm", "-f"] in [c[:3] for c in fake.calls], fake.calls


def test_stop_simulator_uses_the_host_script_when_there_is_no_container(procs):
    p, fake, _ = procs
    result = p.stop_simulator()
    assert result["deployment"] == "host"
    kills = [c for c in fake.calls if c[-1] == "--kill" and "run_sim.sh" in c[0]]
    assert kills, fake.calls
    assert not any("removed a leftover" in d for d in result["detail"]), (
        "claimed to remove a container when there was none")


def test_a_container_that_survives_teardown_is_reported_as_a_failure(procs, monkeypatch):
    """`stop.sh` announced success twice on 2026-08-03 while the simulator held 3.5 GB of
    VRAM, because nothing checked. The verdict comes from asking whether the container is
    gone — not from the return code, which is a hint at best."""
    p, fake, _ = procs
    fake.containers = fake.containers_all = "carla-air-sim other-thing"
    fake.rm_effect = "nothing"      # the removal does not take
    with pytest.raises(RuntimeError, match="STILL RUNNING"):
        p.stop_simulator()


def test_a_stopped_but_unremoved_container_is_not_an_error(procs, monkeypatch):
    """Review of T-06 caught the check and the message disagreeing.

    `docker ps -a` answers "does this container object exist", but the message said STILL
    RUNNING. A stopped-but-present container holds no GPU memory — the simulator IS down — so
    raising would report a failure that did not happen. It is still worth SAYING, in the reply
    rather than as an error, because an unremoved container is a surprise.

    It must NOT claim the container blocks the next start: `run_sim_docker.sh` and
    `stack_up.sh` both `docker rm -f` before starting, so `carla-air-sim` self-heals. That
    justification was borrowed from `carla-air-webui`, which is a different name on the one
    start path with no pre-emptive removal.
    """
    p, fake, _ = procs
    fake.containers = fake.containers_all = "carla-air-sim"   # running when the button is hit
    fake.rm_effect = "stop_only"                              # it stops, but is not removed
    detail = " ".join(p.stop_simulator()["detail"])
    assert "NOT removed" in detail and "simulator is down" in detail
    assert "block the next start" not in detail, (
        "the reply claims a stale carla-air-sim blocks the next start; the start paths "
        "docker rm -f first, so that is not true of this container")


def test_the_teardown_asks_rather_than_trusting_the_return_code(procs, monkeypatch):
    """T-06. A non-zero `docker rm` with the container actually gone is a success, not a
    failure — `rm` complains about plenty of things that do not matter."""
    p, fake, _ = procs
    fake.containers = fake.containers_all = "carla-air-sim"
    fake.returncode = 1              # every command "fails" ...
    fake.rm_effect = "remove"        # ... but the container really does go away
    assert p.stop_simulator()["deployment"] == "container"


def test_sigterm_with_a_grace_period_comes_before_the_hammer(procs, monkeypatch):
    """T-06. `docker rm -f` alone is an immediate SIGKILL — harsher than the host path it sits
    beside, which escalates TERM/TERM/KILL because Unreal does not always go down on the
    first signal. Nobody decided to be less careful in containers; it just happened."""
    p, fake, _ = procs
    fake.containers = fake.containers_all = "carla-air-sim"
    p.stop_simulator()
    # Only the calls that CHANGE something. T-08 put a read-only `docker ps` probe in front of
    # the teardown, and a `ps` is not a failure to escalate gracefully.
    docker = [c for c in fake.calls if c and c[0] == "docker" and c[1] != "ps"]
    verbs = [c[1] for c in docker]
    assert verbs[0] == "stop", f"went straight to {verbs[0]!r} without a graceful stop"
    # Against `docker[0]`, not `fake.calls[0]` — the latter is the first call of ANY kind and
    # only happens to be this one, so it would keep passing if a probe were put in front.
    assert "-t" in docker[0], "no grace period given"
    assert verbs.index("stop") < verbs.index("rm")


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


class TestTheStopButtonAsksTheRightQuestion:
    """T-08. "Is the stack up" and "is there a container to clean up" are different questions.

    `stop_simulator` was gated on `stack_running()`, which asks `docker ps` — running only — so
    a container that existed but was STOPPED looked exactly like no container at all. The
    button took the host lane, ran `run_sim.sh --kill` against a host process that was not
    there, and reported success having done nothing about either half.

    Fixing `stack_running()` would have been wrong: it means "is the stack up", which is the
    right question for the **Start** button, and widening it would make Start believe a dead
    stack was alive. Hence a separate, finer probe.
    """

    def test_a_running_container_is_the_container_lane(self, procs):
        p, fake, _ = procs
        fake.containers = fake.containers_all = "carla-air-sim"
        assert p.sim_container_state() == lifecycle.CONTAINER_RUNNING

    def test_a_stopped_container_is_neither_lane_nor_nothing(self, procs):
        """The state the whole item is about, and the one that had no name before."""
        p, fake, _ = procs
        fake.containers = ""
        fake.containers_all = "carla-air-sim"
        assert p.sim_container_state() == lifecycle.CONTAINER_STOPPED

    def test_no_container_is_absent(self, procs):
        p, fake, _ = procs
        fake.containers = fake.containers_all = "something-else"
        assert p.sim_container_state() == lifecycle.CONTAINER_ABSENT

    def test_a_leftover_container_is_removed_and_said_out_loud(self, procs):
        """The regression itself. Before T-08 this returned a bare host success, with the
        container still sitting there and nothing in the reply mentioning it."""
        p, fake, _ = procs
        fake.containers = ""
        fake.containers_all = "carla-air-sim"
        result = p.stop_simulator()
        assert ["docker", "rm", "-f"] in [c[:3] for c in fake.calls], (
            "the leftover container was never removed")
        assert any("leftover" in d for d in result["detail"]), (
            f"the reply says nothing about the leftover container: {result['detail']}")

    def test_a_stopped_container_does_not_hide_a_running_host_simulator(self, procs):
        """The bug the obvious fix would have introduced, and the reason the sweep is on the
        HOST path rather than being a third lane.

        Routing a merely-present container to the container teardown would remove the leftover,
        report `deployment: container`, and never run `run_sim.sh --kill` — leaving a
        host-native simulator holding 3.3 GB of VRAM while the console said it had stopped the
        simulator. That is the 2026-08-03 incident with a new cause.
        """
        p, fake, _ = procs
        fake.containers = ""
        fake.containers_all = "carla-air-sim"
        result = p.stop_simulator()
        assert result["deployment"] == lifecycle.HOST, (
            "a stopped container was treated as a deployment, so the host simulator was "
            "never killed")
        assert [c for c in fake.calls if c[-1] == "--kill" and "run_sim.sh" in c[0]], (
            "run_sim.sh --kill was skipped, so a host simulator would still be running")

    def test_a_leftover_that_will_not_go_is_reported_rather_than_claimed_gone(self, procs):
        p, fake, _ = procs
        fake.containers = ""
        fake.containers_all = "carla-air-sim"
        fake.rm_effect = "nothing"
        detail = " ".join(p.stop_simulator()["detail"])
        assert "could not be removed" in detail, detail

    def test_no_docker_binary_falls_back_to_the_host_lane(self, procs):
        """A console started with `--in-stack` has no `docker` in its image. That path is
        refused before it reaches here, but the probe must not raise on the way to finding
        out — the honest default is the lane this console drove before containers existed.
        """
        p, fake, _ = procs

        def no_docker(argv, env=None, background=False):
            if argv and argv[0] == "docker":
                raise OSError("No such file or directory: 'docker'")
            return SimpleNamespace(returncode=0, stdout="ok", stderr="")

        p._run = no_docker
        assert p.sim_container_state() == lifecycle.CONTAINER_ABSENT
