#!/usr/bin/env bash
# Start the See-Point-Fly example against an already-running simulator.
#
#   ./examples/vlm_navigation/run.sh --backend oracle
#
# The simulator does not start this: it is an example of what to build on top, so it is
# launched separately and talks only to the public ROS 2 interface. Sourcing the workspace
# and setting ROS_DOMAIN_ID are exactly the two things easy to forget, so they happen here.
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

# Credentials, if there are any. The `claude` backend needs ANTHROPIC_API_KEY and refuses to
# start without it rather than failing on the first frame with the aircraft airborne.
#
# `.env` is git-ignored and mode 0600. It is sourced here rather than passed as a ROS
# parameter on purpose: parameters are readable from the graph with `ros2 param get` and are
# echoed into launch logs, so a key put there is visible to anything on the DDS domain.
if [ -f "$PROJ/.env" ]; then
    set -a
    # shellcheck disable=SC1091
    . "$PROJ/.env"
    set +a
fi
# The `claude` backend's SDK is a python 3.12 dependency; appended so vendor/ can never
# shadow a ROS-supplied module.
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$PROJ/vendor/py312"

if ! ros2 topic list 2>/dev/null | grep -q "/camera/rgb/image_raw"; then
    echo "no /camera/rgb/image_raw — is the simulator up?" >&2
    echo "  ./scripts/bringup.sh --config configs/testbed.yaml" >&2
    exit 1
fi

# `ros2 launch` takes name:=value. Every doc and every finger in this project types
# --backend, because that is what bringup.sh took for months, so translate rather than fail
# with an unrecognised-argument error that says nothing about the real syntax.
LAUNCH_ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --backend|--instruction|--params)
            [ $# -ge 2 ] || { echo "$1 needs a value" >&2; exit 2; }
            LAUNCH_ARGS+=("${1#--}:=$2"); shift 2 ;;
        *:=*) LAUNCH_ARGS+=("$1"); shift ;;
        -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2
           echo "  try: --backend oracle   (or backend:=oracle)" >&2; exit 2 ;;
    esac
done

exec ros2 launch "$HERE/vlm.launch.py" "${LAUNCH_ARGS[@]}"
