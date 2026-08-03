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

# One resolver for every script that needs the release. Override for a single run with
# CARLAAIR_RELEASE; a custom install location is remembered in .release-path instead.
RELEASE="$("$(dirname "${BASH_SOURCE[0]}")/release_path.sh")"
BIN="$RELEASE/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ICD="$PROJ/configs/vulkan/nvidia_icd.container.json"
# Defaults come from configs/testbed.yaml; the argument and the environment still win, so
# a one-off run needs no edit and a permanent change needs no flag.
cfg() { "$PROJ/.venv/bin/python" -c "
import sys, yaml
d = yaml.safe_load(open('$PROJ/configs/testbed.yaml'))
for k in sys.argv[1].split('.'):
    d = (d or {}).get(k)
    if d is None: print(''); raise SystemExit
print(d)" "$1" 2>/dev/null; }

MAP="${1:-$(cfg simulator.map)}"
MAP="${MAP:-Town10HD}"

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
# Render configs/testbed.yaml first, so settings.json is never stale relative to its source.
"$PROJ/.venv/bin/python" "$PROJ/scripts/apply_config.py" --quiet || {
    echo "could not render configs/testbed.yaml" >&2; exit 1; }
mkdir -p ~/Documents/AirSim
cp "$PROJ/configs/sim/settings.json" ~/Documents/AirSim/settings.json

# ---- GPS origin, chosen at start rather than baked in ----
#
# AirSim reads settings.json ONCE at startup, so the geodetic origin cannot be changed on a
# running simulator — this copy is the only place it can be set. Left unset, AirSim uses its
# own default (Redmond, Washington: 47.639686, -122.138289, observed), which is a perfectly
# good GPS *sensor* and meaningless *geolocation*, since the aircraft is flying in Town10HD.
#
#   TESTBED_ORIGIN_LAT=51.5 TESTBED_ORIGIN_LON=-0.12 ./scripts/run_sim.sh
#
# Patching rather than templating: settings.json is also read by humans, and a file full of
# ${PLACEHOLDERS} is worse to read than one that says what it means.
TESTBED_ORIGIN_LAT="${TESTBED_ORIGIN_LAT:-$(cfg simulator.gps_origin.lat)}"
TESTBED_ORIGIN_LON="${TESTBED_ORIGIN_LON:-$(cfg simulator.gps_origin.lon)}"
TESTBED_ORIGIN_ALT="${TESTBED_ORIGIN_ALT:-$(cfg simulator.gps_origin.alt)}"
if [ -n "${TESTBED_ORIGIN_LAT:-}" ] && [ "${TESTBED_ORIGIN_LAT}" != "None" ]; then
    LAT="${TESTBED_ORIGIN_LAT:-0}"; LON="${TESTBED_ORIGIN_LON:-0}"
    ALT="${TESTBED_ORIGIN_ALT:-0}"
    "$PROJ/.venv/bin/python" - "$LAT" "$LON" "$ALT" <<'PYEOF'
import json, os, sys
path = os.path.expanduser("~/Documents/AirSim/settings.json")
with open(path) as fh:
    cfg = json.load(fh)
cfg["OriginGeopoint"] = {"Latitude": float(sys.argv[1]),
                         "Longitude": float(sys.argv[2]),
                         "Altitude": float(sys.argv[3])}
with open(path, "w") as fh:
    json.dump(cfg, fh, indent=4)
print(f"gps origin: {sys.argv[1]}, {sys.argv[2]} at {sys.argv[3]} m")
PYEOF
else
    echo "gps origin: AirSim default (Redmond WA) - set TESTBED_ORIGIN_LAT/LON to override"
fi

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

# ---- which GPU ----
#
# TESTBED_GPU picks the card: unset (let the driver choose, historically device 0), an index
# as nvidia-smi reports it ("0", "1"), or a raw "vendor:device" pair.
#
# This goes through the Mesa device-select Vulkan layer, which despite the name is a generic
# loader layer and works with the proprietary NVIDIA driver. It is the mechanism that
# actually works here: this CARLA build carries no UE `-graphicsadapter` switch, and
# CUDA_VISIBLE_DEVICES does not affect Vulkan.
TESTBED_GPU="${TESTBED_GPU:-$(cfg simulator.gpu)}"
if [ -n "${TESTBED_GPU:-}" ] && [ "$TESTBED_GPU" != "None" ]; then
    if [ ! -e /usr/share/vulkan/implicit_layer.d/VkLayer_MESA_device_select.json ]; then
        echo "WARNING: TESTBED_GPU is set but the Mesa device-select layer is missing;" >&2
        echo "         the request will be ignored. Install mesa-vulkan-drivers." >&2
    fi
    case "$TESTBED_GPU" in
        *:*)
            sel="$TESTBED_GPU"
            ;;
        *)
            # nvidia-smi reports pci.device_id as 0xDDDDVVVV — device first, vendor second.
            raw="$(nvidia-smi --query-gpu=pci.device_id --format=csv,noheader -i "$TESTBED_GPU" 2>/dev/null | tr -d ' ' || true)"
            case "$raw" in
                0x[0-9A-Fa-f][0-9A-Fa-f][0-9A-Fa-f][0-9A-Fa-f][0-9A-Fa-f][0-9A-Fa-f][0-9A-Fa-f][0-9A-Fa-f]) ;;
                *)
                    echo "ERROR: no GPU with index '$TESTBED_GPU'." >&2
                    echo "       Available:" >&2
                    nvidia-smi --query-gpu=index,name --format=csv,noheader 2>/dev/null | sed 's/^/         /' >&2
                    exit 1
                    ;;
            esac
            raw="${raw#0x}"
            sel="$(printf '%s' "${raw:4:4}" | tr 'A-Z' 'a-z'):$(printf '%s' "${raw:0:4}" | tr 'A-Z' 'a-z')"
            ;;
    esac
    export MESA_VK_DEVICE_SELECT="$sel"
    # Remember the index so the post-startup VRAM check looks at the right card.
    case "$TESTBED_GPU" in *:*) : ;; *) TESTBED_GPU_INDEX="$TESTBED_GPU" ;; esac
    echo "gpu: requesting device $sel$([ "$sel" = "$TESTBED_GPU" ] || echo " (index $TESTBED_GPU)")"
fi

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

        # Guard against the silent software-rendering fallback, on the card we actually
        # asked for. A GPU-backed UE4 loading Town10HD sits around 3.3 GB; lavapipe leaves
        # the card untouched at ~110 MB. Checking a FIXED index here would be wrong as soon
        # as TESTBED_GPU points elsewhere — and worse than wrong, since another workload on
        # GPU 0 would make a software-rendering run look confirmed.
        sleep 3
        target="${TESTBED_GPU_INDEX:-0}"
        vram="$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits -i "$target" 2>/dev/null | tr -d ' ')"
        name="$(nvidia-smi --query-gpu=name --format=csv,noheader -i "$target" 2>/dev/null)"
        if [ -n "$vram" ] && [ "$vram" -lt 1000 ]; then
            echo >&2
            echo "WARNING: GPU $target ($name) is only using ${vram} MiB — the simulator is" >&2
            echo "         almost certainly on the lavapipe SOFTWARE rasteriser, not the GPU." >&2
            echo "         Check the Vulkan ICD, and that any TESTBED_GPU request was honoured." >&2
        else
            echo "GPU $target ($name): ${vram} MiB in use — hardware rendering confirmed"
        fi
        nvidia-smi --query-gpu=index,name,memory.used --format=csv,noheader 2>/dev/null |
            awk -F', ' '{printf "  GPU %s (%s): %s\n", $1, $2, $3}'
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
