#!/usr/bin/env bash
# Run something against the containerised stack, in a container that joins it.
#
#   ./scripts/stack_run.sh -d examples/navigation/run.sh
#   ./scripts/stack_run.sh -d examples/vlm_navigation/run.sh --backend oracle
#   ./scripts/stack_run.sh scripts/run_episode.sh --scenario cross_the_plaza --seeds 1
#
# **Why this exists rather than just running it on the host.** The stack's containers share
# one IPC namespace, and Fast-DDS prefers shared memory over loopback UDP. A host process is
# in a different IPC namespace, so it discovers the graph and then receives nothing — measured:
# `ros2 topic hz` on the host reports NO DATA for topics publishing at 16 Hz inside. Joining
# the namespace is the fix, and it is also what "fully containerised" should mean.
#
# The repository is mounted at its own absolute path for the same reason stack_up.sh does it:
# `colcon build --symlink-install` fills the install tree with absolute symlinks.
set -uo pipefail
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM="${TESTBED_SIM_CONTAINER:-carla-air-sim}"
SOCKVOL=carla-air-run
SOCKPATH=/run/carla-air/sim.sock
IMAGE="${TESTBED_ROS_IMAGE:-carla-air/ros:1}"

usage() {
    cat <<'__HELP__'
stack_run.sh - run a project command inside the containerised stack.

SYNOPSIS
  ./scripts/stack_run.sh [-d] [--name NAME] COMMAND [ARGS...]

OPTIONS
  -d, --detach    run in the background (for the long-lived examples)
  --name NAME     container name; defaults to something derived from the command
  -h, --help      this text

COMMAND is a path relative to the repository root, e.g. examples/navigation/run.sh.

EXAMPLES
  ./scripts/stack_run.sh -d examples/navigation/run.sh
  ./scripts/stack_run.sh -d examples/vlm_navigation/run.sh --backend oracle
  ./scripts/stack_run.sh scripts/run_episode.sh --scenario cross_the_plaza --seeds 1
__HELP__
}

DETACH=0; NAME=""
while [ $# -gt 0 ]; do
    case "$1" in
        -d|--detach) DETACH=1; shift ;;
        --name) [ $# -ge 2 ] || { echo "--name needs a value" >&2; exit 2; }; NAME="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) break ;;
    esac
done
[ $# -ge 1 ] || { echo "ERROR: a command is required" >&2; echo >&2; usage >&2; exit 2; }

docker ps --format '{{.Names}}' | grep -qx "$SIM" || {
    echo "ERROR: the stack is not up — ./scripts/stack_up.sh --config configs/testbed.yaml" >&2
    exit 1; }

CMD="$1"; shift
[ -n "$NAME" ] || NAME="carla-air-run-$(basename "$CMD" | tr -c 'A-Za-z0-9_.-' '-' | tr -d '.')-$$"

# TESTBED_SOCKET so anything that talks to the sidecar finds it on the shared volume rather
# than at the /tmp default, which is per-container and would be empty here.
DOCKER_ARGS=(
    --rm --name "$NAME"
    --network "container:$SIM"
    --ipc "container:$SIM"
    -e "TESTBED_SOCKET=$SOCKPATH"
    -e "ROS_DOMAIN_ID=${TESTBED_ROS_DOMAIN_ID:-42}"
    -v "$PROJ:$PROJ"
    -v "$SOCKVOL:/run/carla-air"
    -w "$PROJ"
)
[ "$DETACH" -eq 1 ] && DOCKER_ARGS+=(-d) || DOCKER_ARGS+=(-i)

# Source ROS and the workspace before running. The .sh examples do it themselves, but a
# bare .py cannot, and `stack_run.sh examples/byo_agent.py` failing with
# "No module named 'rclpy'" is a gap in the runner rather than in the thing being run.
# Sourcing twice is harmless.
exec docker run "${DOCKER_ARGS[@]}" "$IMAGE" \
    "source /opt/ros/jazzy/setup.bash \
     && source $PROJ/ros2_ws/install/setup.bash \
     && exec $PROJ/$CMD $*"
