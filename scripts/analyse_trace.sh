#!/usr/bin/env bash
# Read a flight trace written by record_trace.sh.
#
#   ./scripts/analyse_trace.sh out/traces/run1
#   ./scripts/analyse_trace.sh out/traces/a out/traces/b     # compare
#
# A wrapper because the analysis needs ROS's python 3.12 (rosbag2_py is a ROS package), not
# the 3.10 venv that owns the carla/airsim clients.
set -uo pipefail
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$PROJ/ros2_ws/install/setup.bash"
set -u
if [ "${1:-}" = "--split" ]; then shift; exec python3 "$PROJ/scripts/split_layers.py" "$@"; fi
exec python3 "$PROJ/scripts/analyse_trace.py" "$@"
