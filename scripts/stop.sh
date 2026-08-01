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

targets() {
    ps -eo pid,cmd --no-headers |
        grep -E "$PROJ/ros2_ws/install/|$PROJ/sim_bridge/server\.py|testbed\.launch\.py" |
        grep -v " grep " | awk '{print $1}'
}

for sig in TERM TERM KILL; do
    pids="$(targets)"
    [ -z "${pids//[[:space:]]/}" ] && break
    for p in $pids; do kill "-$sig" "$p" 2>/dev/null; done
    sleep 2
done

rm -f /tmp/carla_air_testbed.sock
left="$(targets | wc -l)"

if [ "${1:-}" = "--all" ]; then
    pkill -x "CarlaUE4-Linux-" 2>/dev/null || true
    echo "stopped: graph, sidecar, simulator (${left} stragglers)"
else
    echo "stopped: graph and sidecar, simulator left running (${left} stragglers)"
fi
