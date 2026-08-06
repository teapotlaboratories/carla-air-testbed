#!/usr/bin/env bash
# A Fast-DDS discovery server, so a client off this machine can reach the graph.
#
#   ./scripts/discovery_server.sh                 # bind everything, port 11811
#   ./scripts/discovery_server.sh --port 11888
#   ./scripts/discovery_server.sh --bind netbird  # only the mesh address
#
# WHY. DDS finds peers by MULTICAST, and multicast does not cross a VPN, a routed subnet or
# the internet. A discovery server replaces that with plain unicast UDP: every participant
# announces itself to one known address, and anything else pointed at the same address gets
# the whole graph. Data still flows peer-to-peer afterwards — the server is a phone book, not
# a relay, so it never sees your camera frames.
#
# Then, on the machine running the simulator:
#
#   DISCOVERY_SERVER=<this-host>:11811 ./scripts/stack_up.sh --config configs/testbed.yaml
#
# and on the remote machine running your VLM:
#
#   export ROS_DISCOVERY_SERVER=<this-host>:11811
#   export ROS_SUPER_CLIENT=true
#   export ROS_DOMAIN_ID=42
#   python3 examples/byo_agent.py
#
# ROS_SUPER_CLIENT IS NOT OPTIONAL and leaving it out is the failure that makes this look
# impossible. A plain discovery client is only told about participants it has already matched
# on a topic it subscribes to. Graph introspection needs the whole picture — and
# `ros2 topic echo` introspects BEFORE it can subscribe, because it has to resolve the message
# TYPE from the graph. Without it you get "Could not determine the type for the passed topic"
# while the publisher is right there and healthy.
set -uo pipefail
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PORT=11811
BIND=0.0.0.0
SERVER_ID=0

usage() {
    cat <<'__HELP__'
discovery_server.sh - Fast-DDS discovery server for reaching the graph off this machine.

SYNOPSIS
  ./scripts/discovery_server.sh [--port N] [--bind ADDR|netbird] [--id N]

OPTIONS
  --port N       UDP port to listen on (default 11811)
  --bind ADDR    address to bind; `netbird` resolves the wt0 mesh address (default 0.0.0.0)
  --id N         discovery server id (default 0); only matters with several servers
  -h, --help     this text

THEN
  on this machine   DISCOVERY_SERVER=<host>:PORT ./scripts/stack_up.sh --config ...
  on the remote     export ROS_DISCOVERY_SERVER=<host>:PORT
                    export ROS_SUPER_CLIENT=true
                    export ROS_DOMAIN_ID=42
__HELP__
}

while [ $# -gt 0 ]; do
    case "$1" in
        --port) [ $# -ge 2 ] || { echo "--port needs a value" >&2; exit 2; }; PORT="$2"; shift 2 ;;
        --bind) [ $# -ge 2 ] || { echo "--bind needs a value" >&2; exit 2; }; BIND="$2"; shift 2 ;;
        --id)   [ $# -ge 2 ] || { echo "--id needs a value" >&2; exit 2; }; SERVER_ID="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown argument: $1" >&2; echo >&2; usage >&2; exit 2 ;;
    esac
done

case "$PORT" in ''|*[!0-9]*) echo "ERROR: --port must be numeric, got '$PORT'" >&2; exit 2 ;; esac

if [ "$BIND" = "netbird" ]; then
    BIND="$(ip -o -4 addr show wt0 2>/dev/null | awk '{print $4}' | cut -d/ -f1)"
    [ -n "$BIND" ] || { echo "ERROR: no wt0 interface — is NetBird up?" >&2; exit 1; }
    echo "binding to the NetBird address only: $BIND"
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

command -v fastdds >/dev/null || {
    echo "ERROR: the fastdds CLI is not on PATH — source /opt/ros/jazzy/setup.bash" >&2; exit 1; }

# What to tell people to point at. 0.0.0.0 is a bind address, not something a peer can dial.
ADVERTISE="$BIND"
if [ "$BIND" = "0.0.0.0" ]; then
    ADVERTISE="$(ip -o -4 addr show wt0 2>/dev/null | awk '{print $4}' | cut -d/ -f1)"
    [ -n "$ADVERTISE" ] || ADVERTISE="$(hostname -I 2>/dev/null | awk '{print $1}')"
    [ -n "$ADVERTISE" ] || ADVERTISE="<this-host>"
fi

cat <<EOF
discovery server listening on $BIND:$PORT (id $SERVER_ID)

  on this machine:
    DISCOVERY_SERVER=$ADVERTISE:$PORT ./scripts/stack_up.sh --config configs/testbed.yaml

  on the remote machine:
    export ROS_DISCOVERY_SERVER=$ADVERTISE:$PORT
    export ROS_SUPER_CLIENT=true
    export ROS_DOMAIN_ID=${TESTBED_ROS_DOMAIN_ID:-42}
    export FASTRTPS_DEFAULT_PROFILES_FILE=<repo>/configs/dds/udp-only.xml

The last line is only needed when the client shares a HOST with the stack but not its IPC
namespace — which is exactly the case when you test this locally before going remote.
Fast-DDS advertises a shared-memory locator to same-host peers, the client cannot use it
across the namespace, and the result is a graph that discovers cleanly and delivers nothing.
Measured: NO DATA without it, 17.9 Hz with it. A genuinely remote machine is never offered
that locator and does not need the profile — but it costs nothing to keep it set.

Verified on two DIFFERENT interfaces, because binding to 0.0.0.0 only means anything if a
second address actually works: over NetBird (100.127.184.189) odometry at 17.9 Hz, and over
the LAN (10.0.0.72) odometry at 16.1 Hz with the camera at 4.0 Hz. NOT verified from a
genuinely separate machine — there is only one host here — so the remaining unknown is
whether something upstream drops the traffic, not whether the configuration is right.

Ctrl-C to stop. Data flows peer-to-peer; this process only brokers introductions, so your
camera frames never pass through it.
EOF

exec fastdds discovery -i "$SERVER_ID" -l "$BIND" -p "$PORT"
