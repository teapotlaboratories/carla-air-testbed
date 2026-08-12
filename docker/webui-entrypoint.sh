#!/usr/bin/env bash
# The console's entrypoint inside the stack. R-08.
#
# Everything after `--` on the `docker run` command line is passed through to
# `webui/server.py`, so `--bind`, `--port`, `--source` and `--token` all still work.
#
# **Sourcing is the whole job.** `interfaces` (Collision) and `px4_msgs` (VehicleOdometry,
# VehicleStatus) come from the mounted workspace, not from the image — the image bakes the
# environment and mounts the code, exactly as the other three do. Without the workspace on
# the path the console does not fail: it falls back to the socket and opens the SECOND AirSim
# capture that R-03 step 1 exists to remove. A quiet 24% regression is worse than a crash, so
# this checks and refuses.
set -uo pipefail

# The repository is mounted at its own absolute path (see stack_up.sh for why that is not
# optional), and `-w` puts us there. TESTBED_PROJ is the override for anything that does not.
PROJ="${TESTBED_PROJ:-$PWD}"

if [ ! -f "$PROJ/webui/server.py" ]; then
    echo "ERROR: no webui/server.py under $PROJ" >&2
    echo "       The repository is mounted at its own absolute path and the container's" >&2
    echo "       working directory should be it. Set TESTBED_PROJ if it is somewhere else." >&2
    exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [ -f "$PROJ/ros2_ws/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$PROJ/ros2_ws/install/setup.bash"
fi
set -u

python3 -c 'import interfaces.msg, px4_msgs.msg' 2>/dev/null || {
    echo "ERROR: the ROS workspace is not built — interfaces/px4_msgs are missing, so the" >&2
    echo "       console would silently fall back to the sidecar socket and open a second" >&2
    echo "       AirSim capture. Build it first: ./scripts/build_ros.sh" >&2
    exit 1; }

# ROS_DOMAIN_ID is baked into the ROS image this is built FROM; restated only as a default so
# an explicit -e still wins. On domain 0 this project's PX4-shaped topics merge with the
# sibling project's real ones.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

exec python3 "$PROJ/webui/server.py" "$@"
