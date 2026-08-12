#!/usr/bin/env bash
# Start the web console with its video and telemetry on ROS 2 topics.
#
#   ./scripts/webui.sh                       # on this host, joining the graph
#   ./scripts/webui.sh --in-stack            # inside a container that joins the stack
#   ./scripts/webui.sh --bind netbird        # reachable over the mesh, token generated
#   ./scripts/webui.sh --source socket       # the old path, second capture and all
#
# **Why this exists.** `webui/server.py` used to be started directly with the 3.10 interpreter,
# because everything it did went over the sidecar's Unix socket. Since R-03 step 1 the onboard
# view and the telemetry come from `/camera/rgb/image_raw` and `/fmu/out/vehicle_odometry`,
# which needs `rclpy` — so it needs the 3.12 side and a sourced workspace. Getting that wrong
# does not fail loudly on its own: the console simply falls back to the socket and opens the
# second camera capture this change exists to remove. Hence one script that sets it up.
#
# **The host is not automatically on the graph.** The stack's containers share one IPC
# namespace and Fast-DDS prefers shared memory, so a host process discovers the graph and then
# receives nothing — no error, just NO DATA on topics publishing at 16 Hz inside. The fix is
# `configs/dds/udp-only.xml`, applied below whenever the containerised stack is running.
# Measured: NO DATA without it, 13.9 Hz with it.
#
# `--in-stack` sidesteps that by running inside a container that joins the namespaces properly.
# The console is reachable on the same port either way, because `carla-air-sim` runs with
# `--network host` and every joiner shares that namespace.
set -uo pipefail
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM="${TESTBED_SIM_CONTAINER:-carla-air-sim}"

usage() {
    cat <<'__HELP__'
webui.sh - start the web console with video and telemetry on ROS 2.

SYNOPSIS
  ./scripts/webui.sh [--in-stack] [ARGS...]

OPTIONS
  --in-stack     run inside a container joining the stack, instead of on this host
  -h, --help     this text

Everything else is passed through to webui/server.py:
  --bind ADDR|netbird   address to listen on (default 127.0.0.1)
  --port N              default 8080
  --source ros|socket   where video and telemetry come from (default auto)
  --token TOKEN         require ?k=TOKEN; generated automatically off loopback

THEN
  open the URL it prints. /api/status reports which source is live and how old the
  last frame is, so a blank pane says why rather than just staying blank.
__HELP__
}

IN_STACK=0
ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --in-stack) IN_STACK=1; shift ;;
        # Set by the --in-stack branch below on the inner invocation. Not for humans: it
        # asserts "you are inside the stack", and claiming that falsely disables the stop
        # button for no reason.
        --inside-the-stack) export TESTBED_IN_STACK=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) ARGS+=("$1"); shift ;;
    esac
done

if [ "$IN_STACK" = "1" ]; then
    docker ps --format '{{.Names}}' | grep -qx "$SIM" || {
        echo "ERROR: the stack is not up — ./scripts/stack_up.sh --config configs/testbed.yaml" >&2
        exit 1; }
    # R-08 made `stack_up.sh --console` the managed way to do this. This path still works and
    # is kept for a console started against a stack that is already up, but it is unmanaged:
    # nothing else knows it exists.
    echo "note: --console on stack_up.sh is the managed equivalent of this" >&2
    # REPLACE a stale container rather than inheriting `exit 125`. This exact failure cost a
    # wrong diagnosis on 2026-08-07: the run died with "name already in use", the PREVIOUS
    # container kept answering on the port, and a fix that had landed appeared not to work.
    # `stack_run.sh` runs with --rm, so this only ever matters after an abnormal exit.
    docker rm -f carla-air-webui >/dev/null 2>&1 || true
    # The inner invocation is told it is inside the stack EXPLICITLY, via a flag rather than
    # by detection. /run/.containerenv is present for this whole project on this machine, so a
    # marker-file check cannot tell "console inside the stack" from "console in the ordinary
    # development environment" — and would refuse the stop button in the normal case.
    #
    # A flag rather than `--env` because stack_run.sh takes only -d and --name; anything else
    # is treated as the COMMAND, so `--env FOO=1` would have been run as a program.
    exec "$PROJ/scripts/stack_run.sh" --name carla-air-webui \
        scripts/webui.sh --inside-the-stack "${ARGS[@]+"${ARGS[@]}"}"
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
[ -f "$PROJ/ros2_ws/install/setup.bash" ] && {
    # shellcheck disable=SC1091
    source "$PROJ/ros2_ws/install/setup.bash"
}
set -u

# `interfaces` (Collision) and `px4_msgs` (VehicleOdometry, VehicleStatus) come from the
# workspace. Without it the console cannot subscribe and would fall back to the socket, which
# is the one outcome this script exists to prevent — so say so rather than starting degraded.
python3 -c 'import interfaces.msg, px4_msgs.msg' 2>/dev/null || {
    echo "ERROR: the ROS workspace is not built or not sourced —" >&2
    echo "       interfaces/px4_msgs are missing, so there is nothing to subscribe to." >&2
    echo "       ./scripts/build_ros.sh, then try again." >&2
    exit 1; }

# PyAV lives here for the ROS side on the host; harmless when unused.
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$PROJ/vendor/py312"

# Only when the stack is containerised. On a host-native bringup everything is in one IPC
# namespace already and the profile would be pointless indirection.
if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$SIM"; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$PROJ/configs/dds/udp-only.xml"
    echo "stack is containerised — using $FASTRTPS_DEFAULT_PROFILES_FILE"
    echo "  (without it this process discovers the graph and receives nothing)"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
exec python3 "$PROJ/webui/server.py" "${ARGS[@]+"${ARGS[@]}"}"
