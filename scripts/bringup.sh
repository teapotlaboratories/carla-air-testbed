#!/usr/bin/env bash
# Start the whole testbed in dependency order and tear it down cleanly on exit.
#
#   ./scripts/bringup.sh                                  # geometric baseline backend
#   ./scripts/bringup.sh --backend mock --seed 3
#   ./scripts/bringup.sh --no-sim                         # simulator already running
#
# Order matters: the simulator must serve both RPC ports before the 3.10 sidecar can
# connect, and the sidecar must be listening before the ROS 2 bridge starts.
set -uo pipefail

PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOCKET="${TESTBED_SOCKET:-/tmp/carla_air_testbed.sock}"

# DDS domain isolation is NOT optional here.
#
# drone-sim runs on this same machine and publishes REAL PX4 topics — /fmu/out/vehicle_odometry
# among them — through a uXRCE-DDS agent on the default domain 0. This testbed publishes
# PX4-SHAPED topics with the same names on purpose. On domain 0 the two graphs merge:
# measured, a foreign publisher was pushing /fmu/out/vehicle_odometry at 125 Hz into this
# testbed while our own node published at 20 Hz, and every rate measured here was the sum.
# Worse, our /fmu/in/trajectory_setpoint is then visible to a real PX4 SITL instance.
#
# Anything that talks to this testbed must export the same domain. scripts/status.sh does.
export ROS_DOMAIN_ID="${TESTBED_ROS_DOMAIN_ID:-42}"
BACKEND="geometric"
INSTRUCTION="fly forward and stay clear of buildings"
START_SIM=1
EVAL=true

while [ $# -gt 0 ]; do
    case "$1" in
        --backend) BACKEND="$2"; shift 2 ;;
        --instruction) INSTRUCTION="$2"; shift 2 ;;
        --no-sim) START_SIM=0; shift ;;
        --no-eval) EVAL=false; shift ;;
        -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

mkdir -p "$PROJ/out"

# Clear any previous run FIRST. Without this a second bringup stacks a second graph on the
# first: ros2 node list still shows one of each name, but /fmu/in/trajectory_setpoint comes
# out at 20 or 30 Hz instead of 10 and two controllers fight over the aircraft. It is
# invisible unless you check the rate, which is exactly why it happened three times.
"$PROJ/scripts/stop.sh" > /dev/null 2>&1 || true

PIDS=()
cleanup() {
    echo
    echo "shutting down..."
    for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null || true; done
    sleep 2
    for pid in "${PIDS[@]:-}"; do kill -9 "$pid" 2>/dev/null || true; done
    rm -f "$SOCKET"
}
trap cleanup EXIT INT TERM

# ---- 1. simulator ----
if [ "$START_SIM" -eq 1 ]; then
    "$PROJ/scripts/run_sim.sh" || exit 1
elif ! ss -tln 2>/dev/null | grep -q ":41451 "; then
    echo "--no-sim given but nothing is listening on 41451" >&2
    exit 1
fi

# ---- 2. the Python 3.10 sidecar ----
# It owns the carla/airsim clients. It cannot be a ROS node: libcarla is cpython-310 and
# Jazzy is 3.12 — measured incompatible in both directions. See docs/architecture.md.
rm -f "$SOCKET"
"$PROJ/.venv/bin/python" "$PROJ/sim_bridge/server.py" --socket "$SOCKET" \
    > "$PROJ/out/sim_bridge.log" 2>&1 &
PIDS+=($!)

for _ in $(seq 1 60); do
    [ -S "$SOCKET" ] && break
    sleep 1
done
if [ ! -S "$SOCKET" ]; then
    echo "sim_bridge never created $SOCKET — see out/sim_bridge.log" >&2
    tail -20 "$PROJ/out/sim_bridge.log" >&2
    exit 1
fi
echo "sim_bridge up on $SOCKET"

# ---- 3. the ROS 2 graph ----
# ROS's setup scripts read unbound variables (AMENT_TRACE_SETUP_FILES and friends), so
# `set -u` has to come off for exactly as long as it takes to source them.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$PROJ/ros2_ws/install/setup.bash"
set -u
export TESTBED_PROTOCOL="$PROJ/sim_bridge/protocol.py"

echo "launching the ROS 2 graph (backend=$BACKEND)"
ros2 launch bringup testbed.launch.py \
    backend:="$BACKEND" \
    instruction:="$INSTRUCTION" \
    evaluation:="$EVAL" \
    socket_path:="$SOCKET" &
PIDS+=($!)

wait
