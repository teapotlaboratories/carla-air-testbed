#!/usr/bin/env bash
# Clone the pinned upstreams listed in .repos into vendor/ (git-ignored).
set -euo pipefail
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SHA=392e831c1f659429ca83902e66820d7094591410
DEST="$PROJ/vendor/px4_msgs"

if [ ! -d "$DEST/.git" ]; then
    git clone --filter=blob:none https://github.com/PX4/px4_msgs.git "$DEST"
fi
git -C "$DEST" fetch --depth 1 origin "$SHA" 2>/dev/null || git -C "$DEST" fetch origin
git -C "$DEST" checkout -q "$SHA"

# A pin that is not verified is a wish.
ACTUAL="$(git -C "$DEST" rev-parse HEAD)"
[ "$ACTUAL" = "$SHA" ] || { echo "px4_msgs sha mismatch: $ACTUAL != $SHA" >&2; exit 1; }
echo "px4_msgs at $SHA ($(ls "$DEST/msg" | wc -l) messages)"

# ---- python packages for the ROS side (3.12) ----
#
# The `claude` backend needs the anthropic SDK, and it runs inside the ROS graph — so it
# needs python 3.12, NOT the 3.10 `.venv` that owns the carla/airsim clients. Those two
# interpreters are the seam this whole project is built around; installing into the wrong
# one produces a ModuleNotFoundError that reads like a missing package.
#
# `--target vendor/py312` rather than `pip install --user`: the project rule is that nothing
# lands in ~ or the system. scripts/bringup.sh puts this directory on PYTHONPATH.
PY312="$PROJ/vendor/py312"
python3 -m pip install --quiet --upgrade --target "$PY312" 'anthropic>=0.70' 'av>=13' \
    || { echo "failed to install the anthropic SDK into $PY312" >&2; exit 1; }
VER="$(PYTHONPATH="$PY312" python3 -c 'import anthropic; print(anthropic.__version__)')"
# PyAV carries an FFmpeg with libx264 — opencv-python's bundled one has no GPL
# encoder, which is why recordings were MPEG-4 Part 2 and unplayable in a browser.
echo "anthropic $VER in vendor/py312 (python $(python3 -c 'import sys;print(".".join(map(str,sys.version_info[:2])))'))"
