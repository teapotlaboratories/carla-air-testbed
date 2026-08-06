#!/usr/bin/env bash
# Start the CARLA-Air simulator inside a container, on the GPU.
#
#   ./scripts/run_sim_docker.sh --config configs/testbed.yaml
#   ./scripts/run_sim_docker.sh --config configs/testbed.yaml --stop
#
# The container equivalent of scripts/run_sim.sh, and it deliberately keeps that script's two
# hard rules: the config is REQUIRED (no silent default), and it refuses to report success
# without confirming hardware rendering. A simulator on the lavapipe software rasteriser is
# ~9x slower with the GPU idle and nothing errors — see .ai/AGENTS.md rule 5.
#
# P-01. The 18 GB release is mounted, never baked. See docker/sim.Dockerfile for why the
# image looks the way it does, and docs/worklog/2026-08-06-gpu-in-a-container.md for the four
# wrong answers that preceded the right one.
set -uo pipefail
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${TESTBED_SIM_CONTAINER:-carla-air-sim}"
IMAGE="${TESTBED_SIM_IMAGE:-carla-air/sim:v0.1.7}"

usage() {
    cat <<'__HELP__'
run_sim_docker.sh - run the simulator in a container, on the GPU.

SYNOPSIS
  ./scripts/run_sim_docker.sh --config PATH [--map NAME] [--gpu N]
  ./scripts/run_sim_docker.sh --stop

OPTIONS
  --config PATH   required; decides the map, GPU, camera resolutions and GPS origin
  --map NAME      override simulator.map for one run
  --gpu N         override simulator.gpu for one run
  --stop          stop and remove the container
  -h, --help      this text

WHY --gpus AND NOT --device
  Both work. --gpus '"device=nvidia.com/gpu=N"' is what the sibling project uses and what is
  documented on this machine; the inner quotes are required by Docker's parser for a CDI
  device name and are not decorative.
__HELP__
}

CONFIG=""; MAP_ARG=""; GPU_ARG=""; STOP=0
while [ $# -gt 0 ]; do
    case "$1" in
        --config) [ $# -ge 2 ] || { echo "--config needs a value" >&2; exit 2; }; CONFIG="$2"; shift 2 ;;
        --config=*) CONFIG="${1#*=}"; shift ;;
        --map)    [ $# -ge 2 ] || { echo "--map needs a value" >&2; exit 2; }; MAP_ARG="$2"; shift 2 ;;
        --gpu)    [ $# -ge 2 ] || { echo "--gpu needs a value" >&2; exit 2; }; GPU_ARG="$2"; shift 2 ;;
        --stop)   STOP=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown argument: $1" >&2; echo >&2; usage >&2; exit 2 ;;
    esac
done

if [ "$STOP" -eq 1 ]; then
    docker rm -f "$NAME" >/dev/null 2>&1 && echo "stopped $NAME" || echo "$NAME was not running"
    exit 0
fi
[ -n "$CONFIG" ] || { echo "ERROR: --config is required (no default, on purpose)" >&2; exit 2; }
[ -f "$CONFIG" ] || { echo "ERROR: no such config: $CONFIG" >&2; exit 2; }

cfg() { "$PROJ/.venv/bin/python" -c "
import sys, yaml
d = yaml.safe_load(open('$CONFIG'))
for k in sys.argv[1].split('.'):
    d = (d or {}).get(k)
    if d is None: print(''); raise SystemExit
print(d)" "$1" 2>/dev/null; }

MAP="${MAP_ARG:-$(cfg simulator.map)}"; MAP="${MAP:-Town10HD}"
GPU="${GPU_ARG:-$(cfg simulator.gpu)}"; GPU="${GPU:-0}"
RELEASE="$("$PROJ/scripts/release_path.sh")"
[ -d "$RELEASE" ] || { echo "ERROR: no release at $RELEASE — run ./scripts/install.sh" >&2; exit 1; }

# Render settings.json from the config first, exactly as run_sim.sh does, so the container
# can never start against a stale one.
"$PROJ/.venv/bin/python" "$PROJ/scripts/apply_config.py" --source "$CONFIG" --quiet || {
    echo "refusing to start: $CONFIG was rejected (see above)" >&2; exit 1; }

docker rm -f "$NAME" >/dev/null 2>&1 || true
mkdir -p "$PROJ/out"

echo "launching $MAP in $NAME on GPU $GPU (image $IMAGE)"
# --network host: the sidecar and the ROS graph reach the simulator on 127.0.0.1:2000 and
#   :41451, exactly as they do on the host.
# --ipc shareable: so the other containers can JOIN this IPC namespace, which is what lets
#   Fast-DDS use shared memory between them instead of loopback UDP. `--ipc host` is refused
#   by this daemon under rootless/nested Docker ("error mounting mqueue ... operation not
#   permitted"), so shareable + `--ipc container:` is the way, and is what the sibling
#   project does for the same reason.
# --gpus: the inner quotes are required. See --help.
docker run -d --name "$NAME" \
    --network host \
    --ipc shareable --shm-size=2g \
    --gpus "\"device=nvidia.com/gpu=$GPU\"" \
    -v "$RELEASE:/opt/carla-air" \
    -v "$PROJ/configs/sim/settings.json:/home/sim/Documents/AirSim/settings.json:ro" \
    "$IMAGE" \
    "/opt/carla-air/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping CarlaUE4 $MAP \
        -RenderOffScreen -carla-rpc-port=2000 -quality-level=Epic -unattended -nosound -stdout" \
    >/dev/null || { echo "ERROR: docker run failed" >&2; exit 1; }

echo "waiting for the RPC ports..."
for i in $(seq 1 60); do
    sleep 5
    if ss -tln 2>/dev/null | grep -q ":41451 "; then
        echo "ready after $((i * 5))s — CARLA :2000, AirSim :41451"
        break
    fi
    if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
        echo "ERROR: the container exited during startup:" >&2
        docker logs "$NAME" 2>&1 | tail -20 >&2
        exit 1
    fi
done

# The same guard run_sim.sh applies, for the same reason: a container that started is not a
# container that is rendering. Checked against the card actually requested.
sleep 3
vram="$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits -i "$GPU" 2>/dev/null | tr -d ' ')"
name="$(nvidia-smi --query-gpu=name --format=csv,noheader -i "$GPU" 2>/dev/null)"
if [ -n "$vram" ] && [ "$vram" -lt 1000 ]; then
    echo >&2
    echo "WARNING: GPU $GPU ($name) is only using ${vram} MiB — the simulator is" >&2
    echo "         almost certainly on the lavapipe SOFTWARE rasteriser, not the GPU." >&2
    echo "         docker logs $NAME | grep -i vulkan" >&2
    exit 1
fi
echo "GPU $GPU ($name): ${vram} MiB in use — hardware rendering confirmed"
