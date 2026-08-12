#!/usr/bin/env python3
"""The console is the stack's OPT-IN fourth container. R-08.

    ./.venv/bin/python -m pytest tests/test_stack_console.py -q

No Docker, no simulator, no GPU. Two kinds of test here, and the split is deliberate:

1. **Behavioural, for the paths that exit during argument parsing.** Those are safe to
   execute — `--help` and a refused argument reach no `docker` command at all. The script is
   copied to a temporary directory first, exactly as `test_stop_args.py` does, so even a
   parse bug that fell through could not act on this checkout.

2. **Structural, for everything else.** `--config PATH --console` really does bring up a
   simulator, so no test may pass it — the same rule `--all` has in `test_stop_args.py`. The
   properties that matter about that path are pinned by reading the source instead, and the
   path itself is verified by running it against a real stack (see the R-08 worklog).

The invariant these exist to protect is not about containers at all: after a default bringup
`ros2 node list` must be `/carla_air_bridge` ALONE. The console is an `rclpy` node since R-03
step 1, so "on by default" and that invariant cannot both be true.
"""
from __future__ import annotations

import os
import re
import shutil
import subprocess

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STACK_UP = os.path.join(ROOT, "scripts", "stack_up.sh")
STATUS = os.path.join(ROOT, "scripts", "status.sh")
DOCKERFILE = os.path.join(ROOT, "docker", "webui.Dockerfile")
ENTRYPOINT = os.path.join(ROOT, "docker", "webui-entrypoint.sh")

pytestmark = pytest.mark.skipif(
    not os.path.exists(STACK_UP), reason="stack_up.sh is not in this checkout")


def _source(path):
    with open(path) as fh:
        return fh.read()


@pytest.fixture
def sandboxed(tmp_path):
    """A copy of stack_up.sh whose PROJ is a temporary directory.

    Only the parse-and-exit paths are ever run against it. It cannot find a config, a release
    or a `.venv` there, so even a fall-through would stop at the first check rather than
    starting anything.
    """
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    dst = scripts / "stack_up.sh"
    shutil.copy(STACK_UP, dst)
    os.chmod(dst, 0o755)
    return str(dst)


def _run(script, *args):
    return subprocess.run([script, *args], capture_output=True, text=True, timeout=60)


# ------------------------------------------------------------------ behavioural, parse-only

@pytest.mark.parametrize("flag", ["--help", "-h"])
def test_help_documents_the_console_flag(sandboxed, flag):
    r = _run(sandboxed, flag)
    assert r.returncode == 0, f"{flag} exited {r.returncode}"
    assert "--console" in r.stdout, "the console flag is undocumented"


def test_the_help_says_why_the_console_is_opt_in(sandboxed):
    """A flag whose reason is not written down gets flipped to a default by the next person.

    The reason is a rule, not a preference: the console is an rclpy node, so on-by-default
    makes `ros2 node list` two nodes.
    """
    out = _run(sandboxed, "--help").stdout
    assert "carla_air_bridge" in out, "the help does not say which invariant --console breaks"


def test_console_is_off_unless_asked_for():
    """The whole point of R-08. If this initialiser ever becomes 1, a default bringup starts
    an HTTP control surface and a second ROS node."""
    assert re.search(r"CONSOLE=0\b", _source(STACK_UP)), (
        "CONSOLE is no longer initialised to 0 — the console may now start by default")


def test_an_unknown_argument_is_still_refused(sandboxed):
    """`--consle` is the plausible typo. T-05's lesson was that a teardown script guessing at
    input costs whatever was running; the same applies to one that brings a simulator up."""
    r = _run(sandboxed, "--consle")
    assert r.returncode == 2, f"exited {r.returncode}, not 2"
    assert "unknown argument" in (r.stderr + r.stdout)


def test_console_without_a_config_starts_nothing(sandboxed):
    """`--console` is not a mode of its own: it decorates a bringup, which still requires an
    explicit config. Refusing here also proves parsing completes before anything is started."""
    r = _run(sandboxed, "--console")
    assert r.returncode == 2, f"exited {r.returncode}, not 2"
    assert "--config is required" in (r.stderr + r.stdout)


# ------------------------------------------------------------------ structural, because
# `--config PATH --console` brings up a real simulator.

def test_a_stale_console_container_is_replaced_not_inherited():
    """R-08's filing incident, pinned.

    `webui.sh --in-stack` died with `exit 125` (name already in use) while the PREVIOUS
    container kept answering on the port, so a fix that had landed appeared not to work. The
    removal must come BEFORE the run, or the same wrong diagnosis is available again.

    Unlike `carla-air-sim`, this name is not cleared by anything else on the way up:
    `run_sim_docker.sh` pre-removes the simulator and `stack_up.sh` pre-removes the bridge and
    the ROS container, but nothing pre-removed the console until R-08.
    """
    src = _source(STACK_UP)
    rm_at = src.index('docker rm -f "$WEBUI"')
    run_at = src.index('docker run -d --name "$WEBUI"')
    assert rm_at < run_at, (
        "the console container is started without clearing a stale one first — a leftover "
        "will keep serving old code and the start will fail with exit 125")


def test_the_console_joins_the_simulators_namespaces():
    """Not cosmetic. The stack shares one IPC namespace and Fast-DDS prefers shared memory, so
    a console outside it discovers the graph and then receives NOTHING — no error, just empty
    panes on topics publishing at 16 Hz. And `--network container:` is what puts its port on
    the host's loopback, since the simulator runs with `--network host`.
    """
    src = _source(STACK_UP)
    block = src[src.index('docker run -d --name "$WEBUI"'):]
    block = block[:block.index("\n\n")]
    assert '--network "container:$SIM"' in block, "the console does not join the sim's network"
    assert '--ipc "container:$SIM"' in block, (
        "the console does not join the sim's IPC namespace — it will receive no topic data")


def test_the_image_is_checked_before_anything_is_started():
    """Nothing builds `carla-air/webui:1` automatically, so "not built yet" is the ordinary
    first-run case. Discovering it at step 4 means a 55 s bringup, a stack now up, and one step
    failed — the config check refuses at the top for exactly this reason, and so does this."""
    src = _source(STACK_UP)
    check_at = src.index('docker image inspect "$WEBUI_IMAGE"')
    first_step_at = src.index('echo "=== 1/$STEPS  simulator ==="')
    assert check_at < first_step_at, (
        "the console image is checked after the simulator starts, so a missing image leaves a "
        "half-started stack behind")


def test_the_http_probe_is_skipped_when_the_address_is_not_known():
    """A check that cries wolf on its own documented usage is worse than no check.

    `TESTBED_CONSOLE_ARGS` can carry `--bind netbird` or `--port N` — the help gives the first
    as the example — and a probe hard-coded to 127.0.0.1:8080 would then fail against a
    perfectly healthy console, burn the full timeout and print a warning that is untrue. The
    container-exited check still runs in both cases; that is the failure that matters.
    """
    src = _source(STACK_UP)
    assert re.search(r'\[ -z "\$\{TESTBED_CONSOLE_ARGS:-\}" \].*probe=1', src), (
        "the HTTP probe no longer depends on the console arguments being the default ones")
    assert "command -v curl" in src, "the probe assumes curl is installed"


def test_status_reports_the_console_as_a_container():
    """`stop.sh` and `stack_up.sh --down` are different lanes with different teardowns, so a
    status screen that cannot say which one the console is in cannot say how to stop it."""
    assert "console container" in _source(STATUS), (
        "status.sh no longer distinguishes a containerised console from a host one")


def test_the_console_image_installs_nothing():
    """The ROS image already carries rclpy, cv_bridge and msgpack. A `pip install` appearing
    here would mean the console had grown a dependency the graph does not have — worth
    noticing rather than papering over, since the console must not need anything the
    interface does not already provide.
    """
    src = _source(DOCKERFILE)
    assert "carla-air/ros:1" in src, "the console image no longer builds on the ROS image"
    assert not re.search(r"^\s*RUN\s+.*(pip install|apt-get install)", src, re.M), (
        "the console image installs packages; it is meant to be a thin layer over the ROS "
        "image, and needing more means the console diverged from the graph")


# ------------------------------------------------------------------ behavioural, the entrypoint

def test_the_entrypoint_refuses_a_missing_workspace(tmp_path):
    """The failure mode this guards is SILENT. Without `interfaces`/`px4_msgs` the console does
    not crash — it falls back to the sidecar socket and opens the second AirSim capture that
    R-03 step 1 exists to remove, costing 24% of the camera rate instead of 6.6%.

    A quiet regression is worse than a refusal, so the entrypoint checks. Run here with
    TESTBED_PROJ pointing at a directory with no console in it, which is the cheap half of
    the same guard.
    """
    r = subprocess.run([ENTRYPOINT], capture_output=True, text=True, timeout=60,
                       env={**os.environ, "TESTBED_PROJ": str(tmp_path)})
    assert r.returncode != 0, "the entrypoint started with no webui/server.py present"
    assert "no webui/server.py" in r.stderr
