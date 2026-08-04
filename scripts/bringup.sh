#!/usr/bin/env bash
# Start the whole testbed in dependency order and tear it down cleanly on exit.
# Run `./scripts/bringup.sh --help` for usage.
#
# Order matters: the simulator must serve both RPC ports before the 3.10 sidecar can
# connect, and the sidecar must be listening before the ROS 2 bridge starts.
set -uo pipefail

PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOCKET="${TESTBED_SOCKET:-/tmp/carla_air_testbed.sock}"

usage() {
    cat <<'__HELP__'
bringup.sh - start the drone simulator and its ROS 2 graph.

This starts a SIMULATOR, not a VLM. Nothing it launches interprets a camera. You get a
quadrotor in a photorealistic city and a ROS 2 interface to it:

    /fmu/out/*      odometry, IMU, barometer, magnetometer, GPS
    /camera/*       rgb, depth, segmentation (+ camera_info)
    /sensors/*      semantic lidar
    /fmu/in/*       takeoff, land, position, velocity, attitude
    /sim/*          reset, spawn traffic, set weather, teardown (services)

SYNOPSIS
  ./scripts/bringup.sh --config PATH [--no-sim] [--no-eval]

REQUIRED
  --config PATH   the testbed config. There is NO default, on purpose: it decides the map,
                  the GPU, the camera resolutions, the sensor list and the GPS origin. A
                  silent fallback would bring up a simulator nobody chose, and every number
                  measured from it would belong to a configuration nobody recorded.

OPTIONS
  --no-sim        the simulator is already running; start only the sidecar and the graph
  --no-eval       skip the episode runner (no scenario scoring)
  -h, --help      this text

EXAMPLES
  # the usual thing
  ./scripts/bringup.sh --config configs/testbed.yaml

  # your own config, e.g. a different map or sensor set
  ./scripts/bringup.sh --config configs/my-survey-rig.yaml

  # reuse a simulator that is already up
  ./scripts/bringup.sh --config configs/testbed.yaml --no-sim

FLYING IT
  Once this is up, the aircraft is yours over ROS 2:

    python3 examples/ros2_full_control.py     takeoff -> waypoint -> attitude -> land
    python3 examples/ros2_world_control.py    traffic, weather, teleport, teardown

  For vision-language navigation - an EXAMPLE built on that interface, not part of the
  simulator - start it separately in another terminal:

    ./examples/vlm_navigation/run.sh --backend oracle

STOPPING
  Ctrl-C here tears down what this started. Then confirm nothing is left:

    ./scripts/stop.sh --all && ./scripts/status.sh
__HELP__
}

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
START_SIM=1
EVAL=true

CONFIG=""

# `shift 2` on a flag whose value is missing does NOT shift - it fails and returns 1 - so
# `while [ $# -gt 0 ]` spins forever on the same argument. `--config` with no value hung
# indefinitely until this guard existed. Every value-taking flag goes through need_val.
need_val() {
    if [ "$2" -lt 2 ]; then
        echo "ERROR: $1 needs a value" >&2
        echo "       e.g. --config configs/testbed.yaml" >&2
        exit 2
    fi
}

while [ $# -gt 0 ]; do
    case "$1" in
        # --flag=value as well as --flag value: both forms are muscle memory somewhere, and
        # rejecting one of them teaches nothing.
        --config=*)  CONFIG="${1#*=}"; shift ;;
        --config)    need_val "$1" $#; CONFIG="$2"; shift 2 ;;
        --no-sim)    START_SIM=0; shift ;;
        --no-eval)   EVAL=false; shift ;;
        # --backend / --instruction used to live here. They belong to the VLM example now;
        # accepted and redirected rather than failing with "unknown argument", because every
        # doc and muscle memory in this project still reaches for them.
        --backend=*|--instruction=*)
            echo "note: ${1%%=*} moved to the VLM example - bring this up, then run:" >&2
            echo "        ./examples/vlm_navigation/run.sh ${1%%=*} ${1#*=}" >&2
            shift ;;
        --backend|--instruction)
            need_val "$1" $#
            echo "note: $1 moved to the VLM example - bring this up, then run:" >&2
            echo "        ./examples/vlm_navigation/run.sh $1 $2" >&2
            shift 2 ;;
        -h|--help)   usage; exit 0 ;;
        --)          shift; break ;;
        *)  echo "unknown argument: $1" >&2; echo >&2; usage >&2; exit 2 ;;
    esac
done
[ $# -eq 0 ] || { echo "unexpected extra arguments: $*" >&2; exit 2; }

if [ -z "$CONFIG" ]; then
    echo "ERROR: --config is required." >&2
    echo >&2
    echo "  ./scripts/bringup.sh --config configs/testbed.yaml" >&2
    echo >&2
    echo "There is no default on purpose: the config decides the map, the GPU, the camera" >&2
    echo "resolutions, the sensor list and the GPS origin. See --help." >&2
    exit 2
fi
[ -f "$CONFIG" ] || { echo "ERROR: no such config: $CONFIG" >&2; exit 2; }
CONFIG="$(cd "$(dirname "$CONFIG")" && pwd)/$(basename "$CONFIG")"

mkdir -p "$PROJ/out"

# One source of truth. Rendering here rather than trusting a checked-in copy means editing
# configs/testbed.yaml is enough — nobody has to remember a build step.
"$PROJ/.venv/bin/python" "$PROJ/scripts/apply_config.py" --source "$CONFIG" --quiet || {
    echo "could not render $CONFIG" >&2; exit 1; }

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
    "$PROJ/scripts/run_sim.sh" --config "$CONFIG" || exit 1
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

# ROS-side (3.12) python packages — currently the anthropic SDK for the `claude` backend.
# Appended, not prepended: vendor/ must never shadow a ROS-supplied module.
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$PROJ/vendor/py312"

echo "launching the ROS 2 graph (simulator only — no VLM node)"
ros2 launch bringup testbed.launch.py \
    params:="$PROJ/ros2_ws/src/bringup/config/testbed.yaml" \
    evaluation:="$EVAL" \
    socket_path:="$SOCKET" &
PIDS+=($!)

wait
