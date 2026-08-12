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
#   carla-air-webui       --network container:sim web console, python 3.12   [--console only]
#
# The console is the odd one out and OPT-IN, which is a rule rather than taste: it is an
# `rclpy` node since R-03 step 1, so starting it by default would make `ros2 node list` two
# nodes rather than `/carla_air_bridge` alone. That invariant — "you bring the agent" — is one
# of the few things keeping this project's scope honest, and a flag is cheap.
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
WEBUI=carla-air-webui
WEBUI_IMAGE="${TESTBED_WEBUI_IMAGE:-carla-air/webui:1}"
SOCKVOL=carla-air-run
SOCKPATH=/run/carla-air/sim.sock

usage() {
    cat <<'__HELP__'
stack_up.sh - run the whole testbed in containers.

SYNOPSIS
  ./scripts/stack_up.sh --config PATH [--gpu N] [--console]
  ./scripts/stack_up.sh --down
  ./scripts/stack_up.sh --status

OPTIONS
  --config PATH   required; decides the map, GPU, cameras and GPS origin
  --gpu N         override simulator.gpu for one run
  --console       also start the web console as a fourth container (opt-in; see below)
  --down          stop and remove every container in the stack
  --status        what is running, and what it holds on the GPU
  -h, --help      this text

WHY --console IS OPT-IN
  Since R-03 step 1 the console is an rclpy node, so starting it by default would make
  `ros2 node list` two nodes instead of `/carla_air_bridge` alone — the "you bring the
  agent" invariant — and would bring up an HTTP control surface with every stack. It is
  a flag, and the default stays one node.

  It serves on http://127.0.0.1:8080. TESTBED_CONSOLE_ARGS passes arguments through to
  webui/server.py, e.g. TESTBED_CONSOLE_ARGS="--bind netbird" to reach it over the mesh.
  Build its image once with:
      docker build -f docker/webui.Dockerfile -t carla-air/webui:1 .

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
    for c in "$SIM" "$BRIDGE" "$ROS" "$WEBUI"; do
        printf "  %-18s %s\n" "$c" \
            "$(docker ps --format '{{.Names}} {{.Status}}' 2>/dev/null | grep "^$c " | cut -d' ' -f2- || echo 'not running')"
    done
    echo "== gpu =="
    nvidia-smi --query-gpu=index,name,utilization.gpu,memory.used --format=csv,noheader 2>/dev/null | sed 's/^/  /'
}

CONFIG=""; GPU_ARG=""; ACTION=up; CONSOLE=0
while [ $# -gt 0 ]; do
    case "$1" in
        --config) [ $# -ge 2 ] || { echo "--config needs a value" >&2; exit 2; }; CONFIG="$2"; shift 2 ;;
        --config=*) CONFIG="${1#*=}"; shift ;;
        --gpu)    [ $# -ge 2 ] || { echo "--gpu needs a value" >&2; exit 2; }; GPU_ARG="$2"; shift 2 ;;
        --console) CONSOLE=1; shift ;;
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

#: The console makes it four steps. Counted rather than hard-coded so the progress lines do
#: not quietly lie about how much is left when --console is given.
STEPS=3; [ "$CONSOLE" -eq 1 ] && STEPS=4

# Checked BEFORE anything starts, not at step 4. Nothing builds this image automatically, so
# "you have not built it yet" is the ordinary first-run case — and discovering it after a 55 s
# bringup, with a stack now up and one step failed, is a worse way to be told. Fail fast is
# the same instinct as stack_up.sh's config check: refuse at the top, or not at all.
if [ "$CONSOLE" -eq 1 ] && ! docker image inspect "$WEBUI_IMAGE" >/dev/null 2>&1; then
    echo "ERROR: --console needs the image $WEBUI_IMAGE, which is not built." >&2
    echo "       docker build -f docker/webui.Dockerfile -t $WEBUI_IMAGE ." >&2
    echo "       Nothing was started." >&2
    exit 1
fi

echo "=== 1/$STEPS  simulator ==="
"$PROJ/scripts/run_sim_docker.sh" --config "$CONFIG" ${GPU_ARG:+--gpu "$GPU_ARG"} || {
    echo "the simulator did not come up — not starting the rest" >&2; exit 1; }

docker volume create "$SOCKVOL" >/dev/null 2>&1 || true
docker rm -f "$BRIDGE" "$ROS" >/dev/null 2>&1 || true

echo "=== 2/$STEPS  sidecar (python 3.10) ==="
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

echo "=== 3/$STEPS  ROS 2 graph (python 3.12) ==="
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

if [ "$CONSOLE" -eq 1 ]; then
    echo "=== 4/$STEPS  web console ==="
    # REPLACE a stale container rather than failing on the name, and this is the specific
    # incident R-08 was filed for: `webui.sh --in-stack` died with `exit 125` (name already in
    # use), the PREVIOUS container kept answering on the port, and a fix that had landed
    # appeared not to work. A wrong diagnosis is a worse outcome than a loud failure, and this
    # is the one container in the stack whose name is not already cleared by run_sim_docker.sh
    # or by the bridge/ROS removal above.
    docker rm -f "$WEBUI" >/dev/null 2>&1 || true
    # Joins the simulator's namespaces exactly as the other two do. The simulator runs with
    # --network host, so binding 127.0.0.1 in here IS the host's loopback and the console is
    # reachable at localhost:8080 without publishing a port.
    docker run -d --name "$WEBUI" \
        --network "container:$SIM" \
        --ipc "container:$SIM" \
        ${DS_ENV[@]+"${DS_ENV[@]}"} \
        -e "TESTBED_SOCKET=$SOCKPATH" \
        -e "TESTBED_PROJ=$PROJ" \
        -v "$PROJ:$PROJ" \
        -v "$SOCKVOL:/run/carla-air" \
        -w "$PROJ" \
        "$WEBUI_IMAGE" \
        ${TESTBED_CONSOLE_ARGS:+$TESTBED_CONSOLE_ARGS} >/dev/null || {
        echo "ERROR: could not start $WEBUI" >&2; exit 1; }

    # Wait for it to SERVE, not merely to exist. The console falling back to the socket is a
    # quiet 24% regression rather than a crash, so the entrypoint refuses to start without the
    # workspace — which means a container that exited has something worth printing.
    #
    # The HTTP probe only runs when the address is actually known. `TESTBED_CONSOLE_ARGS` can
    # carry `--bind netbird` or `--port N` — the documented example does exactly that — and a
    # probe hard-coded to 127.0.0.1:8080 would then fail on a perfectly healthy console, burn
    # the full timeout, and print a warning that is simply untrue. A check that cries wolf on
    # its own documented usage is worse than no check. Same for a machine without `curl`.
    probe=0
    [ -z "${TESTBED_CONSOLE_ARGS:-}" ] && command -v curl >/dev/null 2>&1 && probe=1

    for i in $(seq 1 20); do
        sleep 1
        [ "$probe" -eq 1 ] && curl -sf -o /dev/null "http://127.0.0.1:8080/" 2>/dev/null && break
        # The failure that matters either way: the entrypoint refused, or the console died.
        if ! docker ps --format '{{.Names}}' | grep -qx "$WEBUI"; then
            echo "ERROR: the console exited:" >&2; docker logs "$WEBUI" 2>&1 | tail -15 >&2; exit 1
        fi
        [ "$probe" -eq 0 ] && [ "$i" -ge 3 ] && break
    done

    if [ "$probe" -eq 1 ]; then
        if curl -sf -o /dev/null "http://127.0.0.1:8080/" 2>/dev/null; then
            echo "  console up on http://127.0.0.1:8080"
        else
            echo "  WARNING: $WEBUI is running but did not answer on :8080 within 20 s" >&2
            docker logs "$WEBUI" 2>&1 | tail -10 >&2
        fi
    else
        # Say what was NOT checked rather than implying it was. The container is up and did not
        # exit; where it is listening is whatever the arguments asked for.
        if [ -n "${TESTBED_CONSOLE_ARGS:-}" ]; then
            why="TESTBED_CONSOLE_ARGS=$TESTBED_CONSOLE_ARGS decides the address"
        else
            why="no curl on this machine"
        fi
        echo "  console container up — not probed, $why"
    fi
fi

echo
echo "stack up. Stop it with: ./scripts/stack_up.sh --down"
if [ "$CONSOLE" -eq 0 ]; then
    echo "The console is opt-in: add --console, or ./scripts/webui.sh for the host lane."
fi
