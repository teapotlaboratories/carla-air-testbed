#!/usr/bin/env bash
# Client-side Python environment for CARLA-Air v0.1.7 — no conda.
#
# Upstream's env_setup/setup_env.sh requires miniconda and hard-fails without it.
# We do not need conda; we need *CPython 3.10*, because the shipped CARLA module is
# an ABI-tagged extension:
#
#   carla/libcarla.cpython-310-x86_64-linux-gnu.so
#
# Ubuntu 24.04 ships Python 3.12 and has no python3.10 in apt, so this script uses
# uv to fetch a standalone 3.10 and build the venv from it. Everything lands under
# the project (vendor/, .venv/) — nothing is installed into ~ or the system.
#
# Usage: bash scripts/setup_env.sh
set -euo pipefail

PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$PROJ/.venv"
UV="$PROJ/vendor/bin/uv"
# Same default as scripts/fetch_release.sh and scripts/run_sim.sh.
RELEASE_DIR="${CARLAAIR_RELEASE:-${CARLAAIR_HOME:-$(dirname "$PROJ")/carla-air-release}/CarlaAir-v0.1.7}"

# ---------- 1. uv, into the project ----------
if [ ! -x "$UV" ]; then
    mkdir -p "$PROJ/vendor/bin"
    curl -LsSf https://astral.sh/uv/install.sh |
        env UV_INSTALL_DIR="$PROJ/vendor/bin" INSTALLER_NO_MODIFY_PATH=1 sh
fi
export UV_PYTHON_INSTALL_DIR="$PROJ/vendor/python"

# ---------- 2. CPython 3.10 venv ----------
[ -d "$VENV" ] || "$UV" venv --python 3.10 "$VENV"

# ---------- 3. deps ----------
# Order matters. airsim 1.8.1 declares neither numpy nor setuptools as build
# dependencies but imports numpy at setup.py time, so a plain
# `uv pip install airsim` fails twice in a row under build isolation:
#   ModuleNotFoundError: No module named 'numpy'      (in airsim/utils.py)
#   ModuleNotFoundError: No module named 'setuptools'
# Install both first, then airsim with --no-build-isolation.
"$UV" pip install --python "$VENV/bin/python" \
    "numpy<2" opencv-python pygame Pillow setuptools wheel msgpack-rpc-python
"$UV" pip install --python "$VENV/bin/python" --no-build-isolation airsim

# numpy<2 is not cosmetic: airsim 1.8.1 predates the NumPy 2 ABI break and
# msgpack-rpc-python pins tornado 4.5.3, which is likewise 3.10-era.

# ---------- 4. the CARLA module from the release ----------
# Not the PyPI `carla` wheel — CARLA-Air is a fork with a different server ABI
# (client version aa9c92b). Upstream ships it as a tarball, not a wheel.
SITE="$($VENV/bin/python -c 'import site; print(site.getsitepackages()[0])')"
TARBALL="$RELEASE_DIR/env_setup/carla_python_module.tar.gz"
if [ -f "$TARBALL" ]; then
    rm -rf "$SITE/carla" "$SITE/carla.libs"
    tar xzf "$TARBALL" -C "$SITE"
else
    echo "WARN: $TARBALL not found — set CARLAAIR_RELEASE to the extracted release" >&2
fi

# ---------- 5. verify ----------
"$VENV/bin/python" - <<'PY'
import airsim, carla, cv2, numpy
print("python      ", __import__("sys").version.split()[0])
print("numpy       ", numpy.__version__)
print("carla       ", carla.Client("localhost", 9999).get_client_version())
print("airsim      ", airsim.__file__)
print("OK — client environment ready (server not required for this check)")
PY
