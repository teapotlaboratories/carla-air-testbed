#!/usr/bin/env bash
# Run scripts/run_episode.py with the ROS 2 environment it now needs.
#
#   ./scripts/run_episode.sh --scenario cross_the_plaza --seeds 1 2 3
#
# The script became a plain ROS 2 client when world control moved onto services, so it runs
# under ROS's python 3.12 rather than the 3.10 venv that owns the carla/airsim clients.
# Sourcing by hand and forgetting ROS_DOMAIN_ID lands you in the sibling project's graph,
# so this exists to make that impossible rather than merely documented.
set -uo pipefail
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ROS's setup scripts read unbound variables, so `set -u` comes off just long enough.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$PROJ/ros2_ws/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${TESTBED_ROS_DOMAIN_ID:-42}"
exec python3 "$PROJ/scripts/run_episode.py" "$@"
