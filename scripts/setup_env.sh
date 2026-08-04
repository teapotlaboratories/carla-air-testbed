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
# One resolver, so this cannot disagree with run_sim.sh about where the release is.
RELEASE_DIR="$("$PROJ/scripts/release_path.sh")"

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
# PyYAML, PyAV and pytest are NOT optional extras, whatever their names suggest:
#
#   PyYAML  scripts/apply_config.py reads configs/testbed.yaml, and bringup.sh runs it
#           FIRST. Without this a fresh install cannot start the simulator at all.
#   av      sim_bridge/carla_air/h264.py. opencv-python ships no libx264 (GPL vs Apache),
#           so PyAV is what makes chase and episode recording work.
#   pytest  the command install.sh prints on success, and the one the README, the
#           quickstart and both guide tabs tell a new user to run next.
#
# All three were missing until 2026-08-03, and none of it showed here because this machine
# had acquired them outside the scripted path. Found by installing into a clean tree with
# an empty environment - see todo.md R-04.
"$UV" pip install --python "$VENV/bin/python" \
    "numpy<2" opencv-python pygame Pillow setuptools wheel msgpack-rpc-python \
    PyYAML av pytest
"$UV" pip install --python "$VENV/bin/python" --no-build-isolation airsim

# numpy<2 is not cosmetic: airsim 1.8.1 predates the NumPy 2 ABI break and
# msgpack-rpc-python pins tornado 4.5.3, which is likewise 3.10-era.

# ---------- 4. the CARLA client module ----------
# Not the PyPI `carla` wheel — CARLA-Air is a fork with a different server ABI
# (client version aa9c92b). Upstream ships it as a tarball, not a wheel.
#
# And NOT the copy inside the release archive, which is a broken build: its RPATH is
# hardcoded to the upstream author's own machine
#
#   RPATH: /home/lenovo/miniconda3/envs/carlaAir/lib
#
# and its carla.libs/ vendors only libboost_python310, expecting libpng16/libtiff5/libjpeg/
# libwebp/libzstd/liblzma/libjbig from a system that no longer exists. On Ubuntu 24.04
# libtiff5 is not even packaged, so it cannot import at all. This was found by containerising
# and is why setup_env.sh no longer reads $CARLAAIR_RELEASE/env_setup/.
#
# The same file in the project's GitHub repo is properly auditwheel-vendored — 7 bundled
# libs with mangled SONAMEs and a correct $ORIGIN rpath — and imports with no system
# dependencies at all. Pinned by commit and checksummed, because a raw URL follows a moving
# ref otherwise.
SITE="$($VENV/bin/python -c 'import site; print(site.getsitepackages()[0])')"
CARLAAIR_REPO_SHA=d70247b52043d6eadb849ea41cac861ad8567dba
CARLA_MODULE_MD5=a3fa4579f646f6ef1dcac5fa9e03c0b8
TARBALL="$PROJ/vendor/carla_python_module.tar.gz"

if [ ! -f "$TARBALL" ] || ! echo "$CARLA_MODULE_MD5  $TARBALL" | md5sum -c - >/dev/null 2>&1; then
    mkdir -p "$PROJ/vendor"
    curl -fsSL -o "$TARBALL" \
      "https://raw.githubusercontent.com/louiszengCN/CarlaAir/${CARLAAIR_REPO_SHA}/env_setup/carla_python_module.tar.gz"
    echo "$CARLA_MODULE_MD5  $TARBALL" | md5sum -c -
fi
rm -rf "$SITE/carla" "$SITE/carla.libs"
tar xzf "$TARBALL" -C "$SITE"

# ---------- 5. verify ----------
"$VENV/bin/python" - <<'PY'
import airsim, carla, cv2, numpy
print("python      ", __import__("sys").version.split()[0])
print("numpy       ", numpy.__version__)
print("carla       ", carla.Client("localhost", 9999).get_client_version())
print("airsim      ", airsim.__file__)
print("OK — client environment ready (server not required for this check)")
PY
