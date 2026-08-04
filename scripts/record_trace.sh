#!/usr/bin/env bash
# Record what the aircraft actually did, as an MCAP bag.
#
#   ./scripts/record_trace.sh out/traces/run1          # start; Ctrl-C or stop.sh to end
#   ./scripts/record_trace.sh out/traces/run1 --camera # include the image topics (LARGE)
#
# Simulator observability, not episode scoring: this records the aircraft's state and the
# commands reaching it, so it works the same whatever is flying — the VLM example, your own
# navigation stack, or a hand-written setpoint publisher. It is deliberately NOT wired into
# the episode runner, which lives in examples/navigation and is out of scope (see
# .ai/AGENTS.md, "Scope").
#
# Cameras are excluded by default. RGB at 960x720 bgr8 and 8 Hz is ~16 MB/s uncompressed;
# a three-minute episode would be ~3 GB and swamp the thing you are trying to read.
set -uo pipefail
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
    cat <<'__HELP__'
record_trace.sh - record the aircraft's state and commands to an MCAP bag.

SYNOPSIS
  ./scripts/record_trace.sh OUTPUT_DIR [--camera] [--all]

ARGUMENTS
  OUTPUT_DIR    where the bag goes; created if missing, must not already contain one
  --camera      also record /camera/* (LARGE — roughly 16 MB/s at the shipped resolution)
  --all         record every topic, cameras included
  -h, --help    this text

WHAT IS RECORDED BY DEFAULT
  /fmu/out/vehicle_odometry        where the aircraft actually is
  /fmu/in/trajectory_setpoint      what it was told to do, at 10 Hz
  /control/waypoint                waypoints, with the bearing_only flag
  /control/active_target           what the controller was aiming at
  /control/arrived                 when it declared arrival
  /vlm/annotation                  the pixel, if something is annotating
  /camera/pose                     the pose the projection uses — without it a pose-lag
                                   hypothesis cannot be tested at all
  /camera/rgb/camera_info          intrinsics, so a trace is self-describing
  /episode/status /episode/result  episode framing, if the example is running
  /sim/collision                   what was hit

READING IT BACK
  ./scripts/analyse_trace.sh OUTPUT_DIR
__HELP__
}

TOPICS=(
    /fmu/out/vehicle_odometry
    /fmu/in/trajectory_setpoint
    /control/waypoint
    /control/active_target
    /control/arrived
    /vlm/annotation
    /camera/pose
    /camera/rgb/camera_info
    /episode/status
    /episode/result
    /sim/collision
)
CAMERA=(/camera/depth/image_raw /camera/rgb/image_raw)

OUT=""; WITH_CAMERA=0; ALL=0
while [ $# -gt 0 ]; do
    case "$1" in
        --camera)  WITH_CAMERA=1; shift ;;
        --all)     ALL=1; shift ;;
        -h|--help) usage; exit 0 ;;
        -*)        echo "unknown argument: $1" >&2; echo >&2; usage >&2; exit 2 ;;
        *)         [ -z "$OUT" ] || { echo "ERROR: only one output directory" >&2; exit 2; }
                   OUT="$1"; shift ;;
    esac
done
[ -n "$OUT" ] || { echo "ERROR: an output directory is required" >&2; echo >&2; usage >&2; exit 2; }
# rosbag2 refuses to overwrite, but it says so only after the flight has started. Check first.
[ -e "$OUT/metadata.yaml" ] && { echo "ERROR: $OUT already holds a bag" >&2; exit 2; }

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$PROJ/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${TESTBED_ROS_DOMAIN_ID:-42}"

if ! ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_odometry"; then
    echo "no /fmu/out/vehicle_odometry — is the simulator up?" >&2
    echo "  ./scripts/bringup.sh --config configs/testbed.yaml" >&2
    exit 1
fi

mkdir -p "$(dirname "$OUT")"
if [ "$ALL" -eq 1 ]; then
    echo "recording ALL topics -> $OUT"
    exec ros2 bag record --storage mcap --output "$OUT" --all
fi
[ "$WITH_CAMERA" -eq 1 ] && TOPICS+=("${CAMERA[@]}")
echo "recording ${#TOPICS[@]} topics -> $OUT"
# Missing topics are not fatal: /vlm/annotation and /episode/* only exist when the examples
# are running, and a trace of the simulator alone is a legitimate thing to want.
exec ros2 bag record --storage mcap --output "$OUT" "${TOPICS[@]}"
