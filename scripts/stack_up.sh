#!/usr/bin/env bash
# The whole stack in containers: simulator, sidecar, ROS 2 graph.
#
#   ./scripts/stack_up.sh --config configs/testbed.yaml
#   ./scripts/stack_up.sh --down
#   ./scripts/stack_up.sh --status
#
# The container equivalent of scripts/bringup.sh. It keeps that script's two hard rules —
# the config is REQUIRED, and it refuses to report success without confirming hardware
# rendering — and adds a third that only matters here: every container is torn down together,
# because a leftover one holds 3.3 GB of VRAM exactly like a leftover process does.
#
# TOPOLOGY. The simulator owns the network namespace and the other two join it, so
# 127.0.0.1 means the same thing to all three and the sidecar reaches CARLA on :2000 and
# AirSim on :41451 exactly as it does on the host. The same trick the sibling project uses.
#
#   carla-air-sim         --network host          CarlaUE4, GPU
#   carla-air-bridge      --network container:sim sidecar, python 3.10
#   carla-air-ros         --network container:sim bridge node, python 3.12
#
# THE REPOSITORY IS MOUNTED AT ITS OWN ABSOLUTE PATH, not at /workspace. That looks odd and
# is deliberate: `colcon build --symlink-install` writes an install tree full of symlinks to
# absolute source paths, so mounting the repo anywhere else leaves every one of them dangling
# and the graph fails with
#     not found: ".../install/interfaces/share/interfaces/local_setup.bash"
#     Package 'bringup' not found
# Mounting at the identical path makes the host-built workspace load unchanged, which is also
# what lets a `colcon build` on either side be used by the other.
#
# The UDS socket between the sidecar and the ROS side lives on a shared VOLUME rather than in
# /tmp: mounting a volume over /tmp would shadow everything else that expects to write there,
# and the socket path is configurable precisely so it need not be /tmp.
set -uo pipefail
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

SIM="${TESTBED_SIM_CONTAINER:-carla-air-sim}"
BRIDGE=carla-air-bridge
ROS=carla-air-ros
SOCKVOL=carla-air-run
SOCKPATH=/run/carla-air/sim.sock

usage() {
    cat <<'__HELP__'
stack_up.sh - run the whole testbed in containers.

SYNOPSIS
  ./scripts/stack_up.sh --config PATH [--gpu N]
  ./scripts/stack_up.sh --down
  ./scripts/stack_up.sh --status

OPTIONS
  --config PATH   required; decides the map, GPU, cameras and GPS origin
  --gpu N         override simulator.gpu for one run
  --down          stop and remove every container in the stack
  --status        what is running, and what it holds on the GPU
  -h, --help      this text

WHAT IT DOES NOT START
  Waypoint following, episode scoring and the VLM are examples, not the simulator, and are
  started separately exactly as on the host. See examples/navigation and
  examples/vlm_navigation.
__HELP__
}

down() {
    # EVERY carla-air container, not just the three this script starts. The examples and any
    # one-shot run from stack_run.sh join the same namespaces and hold the same resources, and
    # a `--down` that leaves them behind is the containerised version of the leftover-graph
    # failure rule 1 exists for. Matched on this project's own name prefix, which cannot
    # collide with the sibling project's `sim-*`.
    local found=0
    for c in $(docker ps -a --format '{{.Names}}' 2>/dev/null | grep '^carla-air-' || true); do
        docker rm -f "$c" >/dev/null 2>&1 && { echo "  stopped $c"; found=1; }
    done
    [ "$found" -eq 1 ] || echo "  nothing was running"
    # Say what is left rather than assuming, exactly as stop.sh does.
    local left
    left="$(docker ps --format '{{.Names}}' 2>/dev/null | grep -c '^carla-air-' || true)"
    if [ "${left:-0}" -gt 0 ]; then
        echo "  WARNING: $left carla-air container(s) still running" >&2
    else
        nvidia-smi --query-gpu=index,name,memory.used --format=csv,noheader 2>/dev/null | sed 's/^/  gpu /'
    fi
}

status() {
    echo "== containers =="
    for c in "$SIM" "$BRIDGE" "$ROS"; do
        printf "  %-18s %s\n" "$c" \
            "$(docker ps --format '{{.Names}} {{.Status}}' 2>/dev/null | grep "^$c " | cut -d' ' -f2- || echo 'not running')"
    done
    echo "== gpu =="
    nvidia-smi --query-gpu=index,name,utilization.gpu,memory.used --format=csv,noheader 2>/dev/null | sed 's/^/  /'
}

CONFIG=""; GPU_ARG=""; ACTION=up
while [ $# -gt 0 ]; do
    case "$1" in
        --config) [ $# -ge 2 ] || { echo "--config needs a value" >&2; exit 2; }; CONFIG="$2"; shift 2 ;;
        --config=*) CONFIG="${1#*=}"; shift ;;
        --gpu)    [ $# -ge 2 ] || { echo "--gpu needs a value" >&2; exit 2; }; GPU_ARG="$2"; shift 2 ;;
        --down)   ACTION=down; shift ;;
        --status) ACTION=status; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown argument: $1" >&2; echo >&2; usage >&2; exit 2 ;;
    esac
done

case "$ACTION" in
    down)   down; exit 0 ;;
    status) status; exit 0 ;;
esac

[ -n "$CONFIG" ] || { echo "ERROR: --config is required (no default, on purpose)" >&2; exit 2; }
[ -f "$CONFIG" ] || { echo "ERROR: no such config: $CONFIG" >&2; exit 2; }

cfg() { "$PROJ/.venv/bin/python" -c "
import sys, yaml
d = yaml.safe_load(open('$CONFIG'))
for k in sys.argv[1].split('.'):
    d = (d or {}).get(k)
    if d is None: print(''); raise SystemExit
print(d)" "$1" 2>/dev/null; }

GPU="${GPU_ARG:-$(cfg simulator.gpu)}"; GPU="${GPU:-0}"

# DISCOVERY_SERVER=<host>:<port> makes every participant in the stack a discovery CLIENT of
# that server instead of relying on multicast, so a client off this machine can reach the
# graph. See scripts/discovery_server.sh. Unset means multicast, the default.
#
# ROS_SUPER_CLIENT is NOT optional, and leaving it out is what makes this look impossible: a
# plain discovery client is only told about participants it has already matched on a topic it
# subscribes to, and `ros2 topic echo` introspects the graph BEFORE it can subscribe, to
# resolve the message type. Without it you get "Could not determine the type for the passed
# topic" while the publisher is right there.
DS_ENV=()
if [ -n "${DISCOVERY_SERVER:-}" ]; then
    case "$DISCOVERY_SERVER" in
        *:*[!0-9]*|:*|*:) echo "ERROR: DISCOVERY_SERVER must be <host>:<port>, e.g. 10.0.0.5:11811 (got '$DISCOVERY_SERVER')" >&2; exit 2 ;;
        *:*) ;;
        *) echo "ERROR: DISCOVERY_SERVER must be <host>:<port>, e.g. 10.0.0.5:11811 (got '$DISCOVERY_SERVER')" >&2; exit 2 ;;
    esac
    DS_ENV=(-e "ROS_DISCOVERY_SERVER=$DISCOVERY_SERVER" -e "ROS_SUPER_CLIENT=true")
    echo "discovery server: $DISCOVERY_SERVER"
fi

echo "=== 1/3  simulator ==="
"$PROJ/scripts/run_sim_docker.sh" --config "$CONFIG" ${GPU_ARG:+--gpu "$GPU_ARG"} || {
    echo "the simulator did not come up — not starting the rest" >&2; exit 1; }

docker volume create "$SOCKVOL" >/dev/null 2>&1 || true
docker rm -f "$BRIDGE" "$ROS" >/dev/null 2>&1 || true

echo "=== 2/3  sidecar (python 3.10) ==="
docker run -d --name "$BRIDGE" \
    --network "container:$SIM" \
    -v "$PROJ:$PROJ" \
    -v "$SOCKVOL:/run/carla-air" \
    carla-air/sim-bridge:1 \
    "python3.10 $PROJ/sim_bridge/server.py --socket $SOCKPATH" >/dev/null || {
    echo "ERROR: could not start $BRIDGE" >&2; exit 1; }

for i in $(seq 1 30); do
    sleep 2
    docker exec "$BRIDGE" test -S "$SOCKPATH" 2>/dev/null && break
    if ! docker ps --format '{{.Names}}' | grep -qx "$BRIDGE"; then
        echo "ERROR: the sidecar exited:" >&2; docker logs "$BRIDGE" 2>&1 | tail -15 >&2; exit 1
    fi
done
docker exec "$BRIDGE" test -S "$SOCKPATH" 2>/dev/null || {
    echo "ERROR: the sidecar never created $SOCKPATH" >&2
    docker logs "$BRIDGE" 2>&1 | tail -15 >&2; exit 1; }
echo "  sidecar up on $SOCKPATH"

echo "=== 3/3  ROS 2 graph (python 3.12) ==="
docker run -d --name "$ROS" \
    --network "container:$SIM" \
    --ipc "container:$SIM" \
    ${DS_ENV[@]+"${DS_ENV[@]}"} \
    -v "$PROJ:$PROJ" \
    -v "$SOCKVOL:/run/carla-air" \
    carla-air/ros:1 \
    "source /opt/ros/jazzy/setup.bash \
     && source $PROJ/ros2_ws/install/setup.bash \
     && ros2 launch bringup testbed.launch.py \
          params:=$PROJ/ros2_ws/src/bringup/config/testbed.yaml \
          socket_path:=$SOCKPATH" >/dev/null || {
    echo "ERROR: could not start $ROS" >&2; exit 1; }

for i in $(seq 1 45); do
    sleep 2
    docker logs "$ROS" 2>&1 | grep -q "bridged to CARLA-Air" && break
    if ! docker ps --format '{{.Names}}' | grep -qx "$ROS"; then
        echo "ERROR: the ROS graph exited:" >&2; docker logs "$ROS" 2>&1 | tail -20 >&2; exit 1
    fi
done
docker logs "$ROS" 2>&1 | grep -m1 "bridged to CARLA-Air" | sed 's/^/  /' || {
    echo "ERROR: the bridge never connected" >&2; docker logs "$ROS" 2>&1 | tail -20 >&2; exit 1; }

echo
echo "stack up. Stop it with: ./scripts/stack_up.sh --down"
