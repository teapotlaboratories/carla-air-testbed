#!/usr/bin/env python3
"""`stop.sh` must refuse an argument it does not understand, before it kills anything.

    ./.venv/bin/python -m pytest tests/test_stop_args.py -q

It used to accept anything and fall through to its default, so **`./scripts/stop.sh --help`
destroyed the ROS graph instead of printing help** — observed 2026-08-07 mid-measurement,
costing a bringup and a restart. A teardown script is the worst possible place to guess at a
typo: refusing costs one retry, guessing costs whatever was running.

## Why these tests are safe to run on a machine with a live simulator

Two independent guards, because a test that kills the developer's graph when it regresses is
worse than no test:

1. **The script is copied to a temporary directory first.** `PROJ` is derived from
   `BASH_SOURCE`, and every kill pattern is anchored to `PROJ` — so a copy under `/tmp` matches
   no real process even if it runs all the way through.
2. **Only the paths that exit during argument parsing are exercised.** `--help` and an unknown
   argument both return before the kill escalation and before the `rm -f` of the sidecar
   socket. `--all` is never passed: it would `pkill -x CarlaUE4-Linux-` for real, and no test
   is worth that.

The `--all` behaviour is pinned statically instead — see the structural tests at the bottom,
which encode the exact shape of the bug rather than its symptom.
"""
from __future__ import annotations

import os
import re
import shutil
import subprocess

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STOP = os.path.join(ROOT, "scripts", "stop.sh")

pytestmark = pytest.mark.skipif(not os.path.exists(STOP), reason="stop.sh is not in this checkout")

#: The line the script prints when it has actually done something. Its ABSENCE is what proves
#: an argument was refused rather than ignored.
DID_SOMETHING = "stopped: graph and sidecar"


@pytest.fixture
def sandboxed(tmp_path):
    """A copy of stop.sh whose PROJ is a temporary directory, so it can match nothing real."""
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    dst = scripts / "stop.sh"
    shutil.copy(STOP, dst)
    os.chmod(dst, 0o755)
    return str(dst)


def _run(script, *args):
    return subprocess.run([script, *args], capture_output=True, text=True, timeout=60)


# ------------------------------------------------------------------ must refuse, kill nothing

@pytest.mark.parametrize("flag", ["--help", "-h"])
def test_help_prints_help_and_stops_nothing(sandboxed, flag):
    r = _run(sandboxed, flag)
    assert r.returncode == 0, f"{flag} exited {r.returncode}"
    assert "SYNOPSIS" in r.stdout, "did not print usage"
    assert DID_SOMETHING not in (r.stdout + r.stderr), (
        f"{flag} TORE DOWN THE GRAPH instead of printing help — this is the bug")


@pytest.mark.parametrize("bad", ["--webui", "--al", "-a", "all", "--force"])
def test_an_unknown_argument_is_refused(sandboxed, bad):
    """`--webui` is the one that actually happened: a plausible flag that does not exist."""
    r = _run(sandboxed, bad)
    assert r.returncode == 2, f"{bad} exited {r.returncode}, expected 2"
    assert DID_SOMETHING not in (r.stdout + r.stderr), f"{bad} was ignored and it stopped things"


def test_the_refusal_says_nothing_was_stopped_and_how_to_proceed(sandboxed):
    """A guard that only says no leaves the operator wondering what it did to their graph."""
    r = _run(sandboxed, "--bogus")
    out = r.stdout + r.stderr
    assert "--bogus" in out, "did not name the offending argument"
    assert "Nothing was stopped" in out, "did not say the graph is untouched — the first thing "\
                                        "you want to know when a teardown script errors"
    assert "SYNOPSIS" in out, "did not show what it does accept"


def test_a_bad_argument_after_a_good_one_still_refuses(sandboxed):
    """`--all --bogus` must not perform the --all half and then complain. Parsing completes
    before anything is stopped, so a partially-valid command line stops nothing at all."""
    r = _run(sandboxed, "--all", "--bogus")
    assert r.returncode == 2
    assert DID_SOMETHING not in (r.stdout + r.stderr)


# ------------------------------------------------------------------ structural, because --all
# cannot be run: it would pkill a real CarlaUE4.

def _source():
    with open(STOP) as fh:
        return fh.read()


def test_all_is_not_read_from_a_positional_argument_any_more():
    """The original defect in its exact shape: `"${1:-}" = "--all"` tested THREE times, so
    `stop.sh --foo --all` silently did the default, and anything unrecognised did too."""
    assert '${1:-}' not in _source(), (
        "stop.sh still branches on $1 directly; an unrecognised first argument would fall "
        "through to the default again")


def test_parsing_happens_before_anything_is_killed():
    """The ordering IS the fix. The old script ran its kill escalation first and only looked
    at the arguments afterwards, by which point the graph was already down."""
    src = _source()
    parse_at = src.index("while [ $# -gt 0 ]")
    kill_at = re.search(r"^\s*for p in \$pids; do kill", src, re.M).start()
    assert parse_at < kill_at, "arguments are parsed after the first kill — the bug is back"


def test_all_is_honoured_consistently():
    """`ALL=1` used to be read by the container teardown and by nothing else, so
    `ALL=1 ./scripts/stop.sh` removed the container, left the host simulator up, and then
    reported "simulator left running". Three sites, one variable."""
    assert _source().count('[ "$ALL" -eq 1 ]') == 4, (
        "the handover to stack_up.sh, the simulator kill, the status message and the container "
        "teardown must all read the same flag, or they disagree about what --all meant")


def test_the_container_is_torn_down_before_the_process_escalation():
    """T-06, and the ordering is the whole fix.

    The container's CarlaUE4 runs as the invoking user on the host with `comm` exactly
    `CarlaUE4-Linux-`, so `pkill -x` reaches INSIDE the container. When the escalation ran
    first it killed the container's main process, the container dropped to `Exited`, and the
    teardown block below — guarded on "is it still running" — was skipped entirely. Measured
    2026-08-10 against a real stack: `Exited (143)`, never removed, nothing reported.

    An `alpine sleep` container cannot reproduce it, which is why the original real-container
    check passed. Nothing here fails if the two blocks are swapped back, so it is pinned.
    """
    src = _source()
    docker_at = src.index("docker stop -t 10")
    pkill_at = re.search(r'^\s*pkill "-\$sig" -x "CarlaUE4-Linux-"', src, re.M).start()
    assert docker_at < pkill_at, (
        "the pkill escalation runs before the container teardown again — it will kill the "
        "container's simulator first and the teardown block will never execute")


def test_the_container_teardown_acts_on_existence_not_on_running():
    """A container someone else already stopped still has to be REMOVED.

    Asking `docker ps` walks past a stopped-but-present container, which holds no VRAM but
    blocks the next start by that name — how a stale console container silently served old
    code on 2026-08-07.
    """
    guard = re.search(
        r"docker ps (-a )?--format '\{\{\.Names\}\}' 2>/dev/null \| grep -qx \"\$c\" \|\| continue",
        _source())
    assert guard is not None, "the container teardown lost its existence guard"
    assert guard.group(1) == "-a ", (
        "the teardown triggers on `docker ps` (running) again, so a stopped-but-present "
        "container is left behind to block the next start")


def test_the_container_lane_is_handed_over_before_anything_is_signalled():
    """T-07, and the ordering does two jobs at once.

    After a containerised bringup this script reported three processes that "survived TERM and
    KILL" — the sidecar and the ROS graph, running INSIDE `carla-air-bridge` and
    `carla-air-ros`, uid 0 on the host while this script is uid 1000. It could never have
    signalled them. It then said `simulator stopped` with both containers still up, because it
    only knew the name `carla-air-sim`.

    Handing over FIRST fixes both: the containers are gone before `targets()` runs, so there
    are no container processes left to misreport and no `sleep 2` wasted failing to signal
    them. Move the handover after the escalation and the false alarm comes straight back.
    """
    src = _source()
    handover_at = src.index("stack_up.sh\" --down")
    kill_at = re.search(r"^\s*for p in \$pids; do kill", src, re.M).start()
    assert handover_at < kill_at, (
        "the handover runs after the kill escalation — the container processes will be "
        "reported as stragglers this script cannot kill, exactly as before")


def test_the_handover_only_happens_for_all():
    """Plain `stop.sh` means "the graph and the sidecar, leaving the simulator up", and there
    is no containerised equivalent — they ARE containers, and `stack_up.sh --down` would take
    the simulator with them. Handing over unconditionally would turn the non-destructive
    default into a full teardown, which is the T-05 class of defect all over again."""
    src = _source()
    i = src.index("DOWN_FAILED=0")
    j = src.index("# Everything this project can start.", i)
    block = src[i:j]
    call = block.index('stack_up.sh" --down')
    guard = block.index('[ "$ALL" -eq 1 ]')
    assert guard < call, "stop.sh hands the stack over without checking --all"


def test_a_failed_handover_is_not_reported_as_success():
    """The handover happens before `rc` exists, so its verdict has to be carried across. A
    teardown that could not tear down must not exit 0 — that is the 2026-08-03 lesson."""
    src = _source()
    assert re.search(r'\[ "\$DOWN_FAILED" -eq 1 \] && rc=1', src), (
        "the handover's result is dropped, so a failed stack_up.sh --down would still exit 0")


def test_destructive_scope_never_comes_from_the_environment():
    """`ALL` is about as generic an environment variable name as exists.

    An earlier draft of this fix "unified" the flag by seeding it with `${ALL:-0}` so the old
    half-implemented env path kept working. That made things worse, not better: it turned an
    unrelated `export ALL=1` in some parent shell into "also SIGKILL the simulator". Whether
    the teardown escalates must come from the command line, never from ambient state.
    """
    assert "ALL=0" in _source(), "the flag is no longer initialised to a hard 0"
    assert "${ALL:-" not in _source(), (
        "stop.sh inherits ALL from the environment again — an unrelated export would silently "
        "escalate a graph teardown into stopping the simulator")
