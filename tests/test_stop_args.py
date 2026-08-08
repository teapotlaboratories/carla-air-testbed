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
    assert _source().count('[ "$ALL" -eq 1 ]') == 3, (
        "the simulator kill, the status message and the container teardown must all read the "
        "same flag, or they disagree about what --all meant")


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
