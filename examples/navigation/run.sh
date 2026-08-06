#!/usr/bin/env bash
# Start the navigation and evaluation stack against an already-running simulator.
#
#   ./examples/navigation/run.sh                 # controller + episode runner + recorder
#   ./examples/navigation/run.sh --no-eval       # controller and recorder only
#   ./examples/navigation/run.sh --no-record     # no onboard video
#
# The simulator does not start this. Waypoint following and episode scoring are OUT OF SCOPE
# for this repository (see .ai/AGENTS.md, "Scope") — they are an example of what to build on
# the ROS 2 interface. Sourcing the workspace and setting ROS_DOMAIN_ID are the two things
# easy to forget, so they happen here.
#
# `scripts/run_episode.sh` needs this running: an episode has nothing to score if nothing is
# following waypoints.
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "$HERE/../.." && pwd)"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$PROJ/ros2_ws/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${TESTBED_ROS_DOMAIN_ID:-42}"

# PyAV lives in vendor/py312 for the ROS side, and the recorder needs it. Without this the
# writer falls back to mp4v — which cannot carry timestamps, so the recording plays at its
# nominal rate rather than real time, and cannot be played in a browser either. It fell back
# SILENTLY from 2026-08-04, when the recorder moved out of bringup (which does export this)
# into this example, until 2026-08-06. Appended, never prepended: vendor/ must not shadow a
# ROS-supplied module.
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$PROJ/vendor/py312"

LAUNCH_ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --no-eval)   LAUNCH_ARGS+=("evaluation:=false"); shift ;;
        --no-record) LAUNCH_ARGS+=("record:=false"); shift ;;
        --params|--scenarios)
            [ $# -ge 2 ] || { echo "$1 needs a value" >&2; exit 2; }
            LAUNCH_ARGS+=("${1#--}:=$2"); shift 2 ;;
        *:=*)        LAUNCH_ARGS+=("$1"); shift ;;
        -h|--help)   sed -n '2,13p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2
           echo "  try: --no-eval, --no-record, --params PATH, --scenarios PATH" >&2; exit 2 ;;
    esac
done

# Check the simulator is actually up before launching into a graph with nothing to talk to.
# Without this the controller starts, finds no odometry, and streams setpoints at a vehicle
# that does not exist - which looks like a controller bug rather than a missing simulator.
if ! ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_odometry"; then
    echo "no /fmu/out/vehicle_odometry — is the simulator up?" >&2
    echo "  ./scripts/bringup.sh --config configs/testbed.yaml" >&2
    exit 1
fi

exec ros2 launch "$HERE/nav.launch.py" "${LAUNCH_ARGS[@]}"
