#!/usr/bin/env bash
# Build the ROS 2 workspace. px4_msgs comes from vendor/ via --base-paths so no symlinks
# or copies land in ros2_ws/src and build/install/log stay git-ignored.
set -euo pipefail
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJ/ros2_ws"

# ROS's setup scripts read unbound variables (AMENT_TRACE_SETUP_FILES and friends), so
# `set -u` has to come off for exactly as long as it takes to source them. Without this the
# build dies on line 8 of setup.bash with "AMENT_TRACE_SETUP_FILES: unbound variable" —
# but only from a shell that has never sourced ROS, which is every fresh install and no
# interactive session, so it hides well. `scripts/bringup.sh` guards the same way.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

if [ ! -d "$PROJ/vendor/px4_msgs" ]; then
    echo "vendor/px4_msgs missing — run scripts/fetch_vendor.sh first" >&2
    exit 1
fi

# px4_msgs first: interfaces does not depend on it, but every node does, and building it
# separately keeps a 2-minute C++ codegen out of the fast edit/build loop.
if [ ! -d install/px4_msgs ]; then
    colcon build --base-paths "$PROJ/vendor/px4_msgs" --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

# --symlink-install so editing a node's Python does not need a rebuild.
colcon build --symlink-install

echo
echo "built. source it with:"
echo "  source /opt/ros/jazzy/setup.bash && source $PROJ/ros2_ws/install/setup.bash"
