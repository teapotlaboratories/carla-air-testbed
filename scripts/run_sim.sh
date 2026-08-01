#!/usr/bin/env bash
# Start CARLA-Air v0.1.7 headless on this box, on the NVIDIA GPU, and wait until both RPC
# servers answer.
#
# Deliberately NOT a wrapper around upstream's CarlaAir.sh: that script launches
# `-windowed`, which needs a display this container does not have, and it spawns traffic
# through a conda python we do not use.
#
#   ./scripts/run_sim.sh [MAP]        default Town10HD
#   ./scripts/run_sim.sh --kill
set -euo pipefail

# Default matches scripts/fetch_release.sh, so the three scripts agree without
# configuration. Override with CARLAAIR_RELEASE (or CARLAAIR_HOME for the parent).
RELEASE="${CARLAAIR_RELEASE:-${CARLAAIR_HOME:-$(dirname "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)")/carla-air-release}/CarlaAir-v0.1.7}"
BIN="$RELEASE/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ICD="$PROJ/configs/vulkan/nvidia_icd.container.json"
MAP="${1:-Town10HD}"

# `pkill -f CarlaUE4-Linux-Shipping` also matches this script's own command line and kills
# the shell running it. Match on the process name instead.
if [ "$MAP" = "--kill" ]; then
    pkill -x "CarlaUE4-Linux-" 2>/dev/null || true
    echo "stopped"
    exit 0
fi

# Fail early and clearly if the release is not where we think. Without this the launch
# proceeds, nohup cannot exec a missing binary, and the startup watchdog reports it as a bad
# Vulkan ICD — which sent a debugging session down the wrong path.
if [ ! -x "$BIN" ]; then
    echo "ERROR: simulator binary not found at" >&2
    echo "         $BIN" >&2
    echo >&2
    if [ -z "${CARLAAIR_RELEASE:-}" ]; then
        echo "       CARLAAIR_RELEASE is not set, so this fell back to a default path." >&2
        echo "       Set it to your unpacked release (scripts/fetch_release.sh prints it):" >&2
        echo "         export CARLAAIR_RELEASE=/path/to/CarlaAir-v0.1.7" >&2
    else
        echo "       CARLAAIR_RELEASE is set to: $CARLAAIR_RELEASE" >&2
        echo "       but no simulator binary is under it. Check the path is the unpacked" >&2
        echo "       release directory itself, not its parent." >&2
    fi
    exit 1
fi

# ---- AirSim settings ----
# Without CaptureSettings per ImageType, only ImageType 0 honours the configured
# resolution: RGB comes back 1280x960 (4:3) while depth/segmentation fall back to AirSim's
# 256x144 (16:9) default, so a pixel index from the RGB frame reads the wrong place.
# configs/sim/settings.json keeps the aspect ratios equal and depth/seg small — see
# docs/architecture.md for why small matters (it is the float readback, not the GPU).
mkdir -p ~/Documents/AirSim
cp "$PROJ/configs/sim/settings.json" ~/Documents/AirSim/settings.json

# ---- Vulkan: put it on the actual GPU ----
#
# On a NATIVE Ubuntu install this whole block is a no-op: the nvidia-driver package ships
# an ICD whose library_path is correct, and the loader finds the GPU on its own.
#
# Inside a distrobox it is not. The ICD JSON is injected from the HOST, so on a Fedora host
# it says "/usr/lib64/libGLX_nvidia.so.0" — a path that does not exist in an Ubuntu
# container, where the driver is bind-mounted at /lib/x86_64-linux-gnu/. Both failure modes
# are silent:
#
#   VK_ICD_FILENAMES=<the broken ICD>  -> UE dies in Vulkan init. Exit 1, ZERO bytes of log,
#       no crash dump, nothing in Saved/Logs. It looks like a corrupt install.
#   VK_ICD_FILENAMES unset             -> the loader enumerates every ICD and silently falls
#       back to lavapipe, the LLVM SOFTWARE rasteriser. Everything "works" — on the CPU,
#       with the GPU at 0% and 111 MiB. Measured cost, RGB 640x480: 5.95 Hz vs 53.8 Hz.
#
# So: use the system ICD when it resolves, and only synthesise a corrected one when it does
# not. Regenerating from ldconfig rather than trusting a checked-in path keeps this working
# on a machine whose driver lives somewhere else.
icd_library() { python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['ICD']['library_path'])" "$1" 2>/dev/null; }

SYSTEM_ICD=/usr/share/vulkan/icd.d/nvidia_icd.x86_64.json
if [ -f "$SYSTEM_ICD" ] && [ -e "$(icd_library "$SYSTEM_ICD")" ]; then
    export VK_DRIVER_FILES="$SYSTEM_ICD"
    echo "vulkan: system NVIDIA ICD is valid"
else
    REAL_LIB="$(ldconfig -p 2>/dev/null | grep -m1 'libGLX_nvidia.so.0' | awk '{print $NF}')"
    if [ -z "$REAL_LIB" ] || [ ! -e "$REAL_LIB" ]; then
        echo "ERROR: no usable libGLX_nvidia.so.0 on this system. Install the NVIDIA driver," >&2
        echo "       or inside a distrobox check that the host driver is being mounted in." >&2
        exit 1
    fi
    mkdir -p "$(dirname "$ICD")"
    if [ "$(icd_library "$ICD" 2>/dev/null)" != "$REAL_LIB" ]; then
        printf '{\n    "file_format_version": "1.0.1",\n    "ICD": {\n        "library_path": "%s",\n        "api_version": "1.4.341"\n    }\n}\n' \
            "$REAL_LIB" > "$ICD"
        echo "vulkan: system ICD unusable — regenerated $ICD -> $REAL_LIB"
    else
        echo "vulkan: using corrected ICD -> $REAL_LIB"
    fi
    export VK_DRIVER_FILES="$ICD"
fi
unset VK_ICD_FILENAMES || true

pkill -x "CarlaUE4-Linux-" 2>/dev/null || true
sleep 4

mkdir -p "$PROJ/out"
setsid nohup "$BIN" CarlaUE4 "$MAP" \
    -RenderOffScreen \
    -carla-rpc-port=2000 \
    -quality-level=Epic \
    -unattended -nosound \
    > "$PROJ/out/sim.log" 2>&1 < /dev/null &
disown

echo "launching $MAP headless on the NVIDIA GPU (log: out/sim.log)"
for i in $(seq 1 60); do
    sleep 5
    if ss -tln 2>/dev/null | grep -q ":41451 "; then
        echo "ready after $((i * 5))s — CARLA :2000, AirSim :41451"

        # Guard against the silent software-rendering fallback. A GPU-backed UE4 loading
        # Town10HD sits around 3.3 GB of VRAM; lavapipe leaves it at ~111 MB.
        sleep 3
        vram="$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits -i 0 2>/dev/null | tr -d ' ')"
        if [ -n "$vram" ] && [ "$vram" -lt 1000 ]; then
            echo >&2
            echo "WARNING: GPU 0 is only using ${vram} MiB — the simulator is almost" >&2
            echo "         certainly on the lavapipe SOFTWARE rasteriser, not the RTX." >&2
            echo "         Check that $ICD points at a libGLX_nvidia.so.0 that exists." >&2
        else
            echo "GPU 0: ${vram} MiB in use — hardware rendering confirmed"
        fi
        exit 0
    fi
    if ! pgrep -x "CarlaUE4-Linux-" > /dev/null; then
        echo "ERROR: process died during startup. out/sim.log is probably empty —" >&2
        echo "       that is what a bad Vulkan ICD looks like. Check $ICD." >&2
        exit 1
    fi
done
echo "ERROR: timed out waiting for the AirSim RPC port" >&2
exit 1
