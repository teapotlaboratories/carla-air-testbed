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
