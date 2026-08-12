#!/usr/bin/env bash
# Stop THIS testbed's processes. Nothing else.
#
# Three hazards this script exists to avoid:
#
# 1. `pkill -f <pattern>` also matches the command line of the shell running it, so typing
#    these patterns inline SIGTERMs your own session (exit 144). Keeping them in a file
#    keeps them off the parent shell's command line.
# 2. drone-sim on this machine has packages called control, evaluation and vlm_client too.
#    Matching on node names would kill a running flight gate in the other project. So every
#    pattern below is anchored to THIS repository's install path.
#
#   ./scripts/stop.sh            # ROS graph + sidecar
#   ./scripts/stop.sh --all      # also the simulator
#   ./scripts/stop.sh --help     # this, and nothing else
#
# 3. **An argument this script does not recognise must not be ignored.** It used to accept
#    anything and fall through to the default, so `stop.sh --help` DESTROYED THE GRAPH instead
#    of printing help — observed 2026-08-07, mid-measurement, costing a restart. A teardown
#    script is exactly where a typo must fail closed: the cost of refusing a bad argument is
#    one retry, and the cost of guessing is whatever was running.
set -u
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
    cat <<'__HELP__'
stop.sh - stop THIS testbed's processes, and nothing else.

SYNOPSIS
  ./scripts/stop.sh [--all]

OPTIONS
  --all          also stop the simulator, including the containerised one
  -h, --help     this text

With no arguments: the ROS graph and the sidecar, leaving the simulator up.

CONTAINERS
  When a containerised stack is up, --all hands over to `stack_up.sh --down`, which owns
  that lane. One teardown command in both lanes, and no second copy of its sweep here.
  Without --all there is no containerised equivalent to perform - the graph and the
  sidecar ARE containers - so it says so rather than half-doing it.

Every pattern is anchored to this repository's install path, so a sibling project's
control/evaluation/vlm_client nodes are never matched. Exits non-zero if anything
survived TERM and KILL - it reports what it ACHIEVED, not what it attempted.
__HELP__
}

# Parsed BEFORE anything is killed. That ordering is the fix: the old script ran its kill
# escalation first and only looked at $1 afterwards, so an unrecognised argument had already
# taken the graph down by the time it was noticed.
#
# NOT seeded from the environment, and that is deliberate. `ALL=1` used to be read by the
# container teardown at the bottom and by nothing else, so `ALL=1 ./scripts/stop.sh` removed
# the container, left the host simulator running, and reported "simulator left running" — one
# of the three defects this change fixes.
#
# The tempting fix is to honour the variable everywhere. That is worse: `ALL` is about as
# generic an environment variable name as exists, nothing in this repository sets it, and
# inheriting it would mean an unrelated export in a parent shell silently escalates a teardown
# into SIGKILLing the simulator. Destructive scope comes from an explicit flag on the command
# line, never from ambient state.
ALL=0
while [ $# -gt 0 ]; do
    case "$1" in
        --all)     ALL=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *)
            echo "ERROR: unknown argument: $1" >&2
            echo "Nothing was stopped. Run --help to see what this accepts." >&2
            echo >&2
            usage >&2
            exit 2 ;;
    esac
done

# T-07. THE CONTAINER LANE HAS AN OWNER, AND IT IS stack_up.sh.
#
# Measured 2026-08-10: after a containerised bringup, `stop.sh --all` reported
#   WARNING: 3 process(es) survived TERM and KILL
# naming the sidecar and the ROS graph, and then said `simulator stopped` while
# `carla-air-bridge` and `carla-air-ros` were still up. Both halves were wrong. Those three
# processes run INSIDE those containers and are uid 0 on the host while this script is uid
# 1000, so TERM and KILL both bounce - no amount of retrying could ever have removed them.
# And this script only ever knew the name `carla-air-sim`, so the other two containers
# survived a teardown that announced success.
#
# Rule 1 says a flight test ends with `stop.sh --all` and `status.sh` showing every count at
# 0. Handing over is what keeps that ONE command true in both lanes. The alternative - teaching
# this script to sweep `carla-air-*` itself - puts `stack_up.sh --down`'s logic in two places,
# and this project has already deleted one parallel structure that drifted from the thing it
# claimed to describe.
#
# HANDED OVER FIRST, before the escalation below, and the order does the second half of the
# job: with the containers already gone, `targets()` finds no container processes to misreport
# as stragglers and the escalation wastes no `sleep 2` failing to signal them. The fix for the
# ownership and the fix for the false alarm are the same edit.
#
# Only for --all. Plain `stop.sh` means "the graph and the sidecar, leaving the simulator up",
# and there is no containerised equivalent: the graph and the sidecar ARE containers here, and
# `stack_up.sh --down` would take the simulator with them. So that case is told, not guessed at.
DOWN_FAILED=0
if command -v docker >/dev/null 2>&1 &&
   [ "$(docker ps --format '{{.Names}}' 2>/dev/null | grep -c '^carla-air-' || true)" -gt 0 ]; then
    if [ "$ALL" -eq 1 ]; then
        if [ -x "$PROJ/scripts/stack_up.sh" ]; then
            echo "a containerised stack is running — handing over to stack_up.sh --down"
            "$PROJ/scripts/stack_up.sh" --down || DOWN_FAILED=1
        else
            echo "WARNING: a containerised stack is running but scripts/stack_up.sh is not" >&2
            echo "         executable here, so its containers were left alone." >&2
            DOWN_FAILED=1
        fi
    else
        echo "note: a containerised stack is running. The graph and the sidecar are CONTAINERS" >&2
        echo "      here, so this script cannot stop them on their own — ./scripts/stack_up.sh" >&2
        echo "      --down stops the whole stack, simulator included." >&2
    fi
fi

# Everything this project can start.
#
# Matching on the command line alone is not enough, and that is not theoretical: the web
# console is normally started as `./.venv/bin/python webui/server.py`, whose command line
# contains no absolute path at all. A pattern anchored to $PROJ misses it; an unanchored
# pattern could match drone-sim, which lives in the SAME container and has packages called
# control, evaluation and vlm_client too.
#
# So ownership is PROVEN rather than guessed: a candidate is ours only if its command line
# mentions $PROJ *or* its working directory is inside $PROJ. Both come from /proc, so
# neither can be spoofed by a relative invocation.
NAMES="ros2_ws/install/"
NAMES="$NAMES|sim_bridge/server\.py|webui/server\.py"
NAMES="$NAMES|testbed\.launch\.py|vlm_navigation/vlm\.launch\.py|navigation/nav\.launch\.py"
# A trace recorder outlives the flight it was recording: it is started beside an episode
# and nothing stops it when the episode ends. Four orphaned in one session on 2026-08-04,
# each still subscribed and still writing. Matched on the OUTPUT PATH under this repo, not
# on "bag record", which would also match a sibling project and this script's own shell.
NAMES="$NAMES|bag record .*--output $PROJ/out|bag record .*--output out/"
NAMES="$NAMES|scripts/run_episode\.py|scripts/run_sweep\.sh"

is_ours() {
    local pid="$1" cwd
    tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -qF "$PROJ" && return 0
    cwd="$(readlink -f "/proc/$pid/cwd" 2>/dev/null)"
    case "$cwd" in "$PROJ"|"$PROJ"/*) return 0 ;; esac
    return 1
}

# Never kill ourselves or whatever launched us. Without this, stop.sh called from inside
# run_sweep.sh would kill the sweep that called it - and a shell that happened to match
# would take the operator's session with it, which is hazard 1 in the header.
mine() {
    local p=$$
    while [ "${p:-0}" -gt 1 ]; do
        echo "$p"
        p="$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')"
        [ -n "$p" ] || break
    done
}

targets() {
    local safe pid
    safe="|$(mine | tr '\n' '|')"
    ps -eo pid,cmd --no-headers |
        grep -E "$NAMES" |
        grep -v " grep " |
        grep -vE "stop\.sh" |
        awk '{print $1}' |
    while read -r pid; do
        case "$safe" in *"|$pid|"*) continue ;; esac
        is_ours "$pid" && echo "$pid"
    done
}

for sig in TERM TERM KILL; do
    pids="$(targets)"
    [ -z "${pids//[[:space:]]/}" ] && break
    for p in $pids; do kill "-$sig" "$p" 2>/dev/null; done
    sleep 2
done

rm -f /tmp/carla_air_testbed.sock

# The simulator gets the SAME escalation the graph gets, and then gets CHECKED.
#
# It used to get a single SIGTERM and no verification, while this script printed
# "stopped: ... simulator" regardless. Observed twice on 2026-08-03: the sidecar had already
# died, `stop.sh --all` reported success, and the simulator was still holding 3.5 GB of VRAM
# on GPU 1 until `run_sim.sh --kill` was run by hand. Unreal does not always go down on the
# first TERM, and a teardown script that reports what it ATTEMPTED rather than what it
# ACHIEVED is worse than no script - rule 1 of this project is that the machine is left
# clean, and this is what is supposed to guarantee it.
#
# `pkill -x` matches the process NAME, never a command line: see the header.
sim_alive() { pgrep -x "CarlaUE4-Linux-" > /dev/null 2>&1; }

rc=0
# The handover ran before `rc` existed, so its verdict is folded in here rather than being
# lost. A teardown that could not tear down must not exit 0.
[ "$DOWN_FAILED" -eq 1 ] && rc=1

# THE CONTAINER IS TORN DOWN FIRST, and the order is load-bearing rather than tidy.
#
# The containerised simulator (P-01) is not a process under $PROJ, so the path-scoped matching
# in `targets` cannot see it — `stop.sh --all` would report success with 3.3 GB of VRAM still
# held. It is matched by CONTAINER NAME, which is this project's own and cannot collide with
# the sibling project's `sim-unreal`.
#
# But `pkill -x "CarlaUE4-Linux-"` below CAN see it, and that is the trap. The container's
# CarlaUE4 runs as THIS user on the host (`docker top` shows the invoking user, not root) and
# its `comm` is exactly `CarlaUE4-Linux-` — the 15-character truncation `-x` matches. So if the
# escalation ran first it would kill the container's main process, the container would drop to
# `Exited`, and a guard asking "is it still RUNNING" would be false: the graceful stop, the
# removal and both checks below would never execute at all.
#
# That is not hypothetical. Measured 2026-08-10 against a real stack: the container was left
# `Exited (143)` and NOT removed, with nothing reported — holding no VRAM, but blocking the
# next start by name, which is exactly the stale container that silently served old code on
# 2026-08-07. Stopping the container before signalling processes is what keeps this reachable.
if [ "$ALL" -eq 1 ] && command -v docker >/dev/null 2>&1; then
    for c in ${TESTBED_SIM_CONTAINER:-carla-air-sim}; do
        # `docker ps -a`, not `docker ps`: act whenever the container EXISTS, not only while it
        # is still running. A container someone else already stopped still has to be removed,
        # and that is the case this script kept walking past.
        docker ps -a --format '{{.Names}}' 2>/dev/null | grep -qx "$c" || continue
        # SIGTERM with a grace period BEFORE the hammer, and then CHECK — the same
        # escalate-and-verify the host simulator gets below, and for the same reason.
        # `docker rm -f` alone is an immediate SIGKILL, which is harsher than the path it
        # replaced and told nobody whether it worked. The grace period is nearly free:
        # measured 2026-08-10, CarlaUE4 takes SIGTERM and exits in 1.2 s (code 143, not 137),
        # so the 10 s is a ceiling that is never reached rather than a delay anyone pays.
        docker stop -t 10 "$c" >/dev/null 2>&1 || true
        docker rm -f "$c" >/dev/null 2>&1 || true
        # Two DIFFERENT questions, and conflating them mislabels a harmless state.
        # "Still running" is the VRAM question rule 1 cares about, and the only one that
        # means rule 1 was not met. "Still exists" is a leftover object: the simulator IS
        # down, so it is reported without being called a failure of the teardown.
        #
        # It is NOT reported as blocking the next start, which is what an earlier draft of
        # this said. `run_sim_docker.sh` and `stack_up.sh` both `docker rm -f` before they
        # `docker run`, so this name self-heals. The container that really did block a start
        # was `carla-air-webui` on 2026-08-07, and `scripts/webui.sh` is the one start path
        # with no pre-emptive removal. Borrowing that incident's consequence for this name
        # would be an overclaim in the teardown's own output.
        if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$c"; then
            echo "WARNING: container $c is STILL RUNNING after stop and rm -f" >&2
            rc=1
        elif docker ps -a --format '{{.Names}}' 2>/dev/null | grep -qx "$c"; then
            echo "WARNING: container $c is stopped but NOT REMOVED — the simulator is down" >&2
            echo "         and it holds no GPU memory; the container object is still there." >&2
            rc=1
        else
            echo "stopped container $c"
        fi
    done
fi

if [ "$ALL" -eq 1 ]; then
    for sig in TERM TERM KILL; do
        sim_alive || break
        pkill "-$sig" -x "CarlaUE4-Linux-" 2>/dev/null || true
        sleep 2
    done
fi

sim_note="simulator left running"
if [ "$ALL" -eq 1 ]; then
    if sim_alive; then
        sim_note="SIMULATOR STILL RUNNING — it ignored TERM and KILL"
        rc=1
    else
        sim_note="simulator stopped"
    fi
fi

# TWO different populations, and calling both "stragglers" was the other half of T-07.
#
# "Survived TERM and KILL" says a process RESISTED. The container-internal ones never could
# have been signalled at all: they are uid 0 on the host while this script is uid 1000, so
# every signal bounced with EPERM. Reporting them as survivors sent someone looking for a
# wedged process that was never wedged.
#
# `kill -0` asks the precise question — could this script have acted on it? — rather than
# inferring containerhood from a uid or a namespace. Both still mean the machine is not clean,
# so both still set rc; only the sentence changes, because only the sentence was false.
survived=""; unreachable=""
for p in $(targets); do
    if kill -0 "$p" 2>/dev/null; then survived="$survived $p"; else unreachable="$unreachable $p"; fi
done
n_survived="$(echo $survived | wc -w)"
n_unreachable="$(echo $unreachable | wc -w)"

if [ "$n_survived" -gt 0 ]; then
    echo "WARNING: ${n_survived} process(es) survived TERM and KILL:" >&2
    ps -o pid,cmd --no-headers -p $survived 2>/dev/null | sed 's/^/  /' >&2
    rc=1
fi
if [ "$n_unreachable" -gt 0 ]; then
    echo "WARNING: ${n_unreachable} process(es) are inside containers this script cannot" >&2
    echo "         signal — not stragglers, and no amount of retrying would remove them." >&2
    echo "         ./scripts/stack_up.sh --down owns that lane:" >&2
    ps -o pid,cmd --no-headers -p $unreachable 2>/dev/null | sed 's/^/  /' >&2
    rc=1
fi

echo "stopped: graph and sidecar, ${sim_note} (${n_survived} stragglers, ${n_unreachable} in containers)"
exit $rc
