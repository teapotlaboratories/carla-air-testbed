#!/usr/bin/env bash
# Stop THIS testbed's processes. Nothing else.
#
# Two hazards this script exists to avoid:
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
set -u
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

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

if [ "${1:-}" = "--all" ]; then
    for sig in TERM TERM KILL; do
        sim_alive || break
        pkill "-$sig" -x "CarlaUE4-Linux-" 2>/dev/null || true
        sleep 2
    done
fi

left="$(targets | wc -l)"
sim_note="simulator left running"
rc=0
if [ "${1:-}" = "--all" ]; then
    if sim_alive; then
        sim_note="SIMULATOR STILL RUNNING — it ignored TERM and KILL"
        rc=1
    else
        sim_note="simulator stopped"
    fi
fi

if [ "$left" -gt 0 ]; then
    echo "WARNING: ${left} process(es) survived TERM and KILL:" >&2
    ps -o pid,cmd --no-headers -p $(targets | tr '\n' ' ') 2>/dev/null | sed 's/^/  /' >&2
    rc=1
fi

echo "stopped: graph and sidecar, ${sim_note} (${left} stragglers)"
exit $rc
