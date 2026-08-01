#!/usr/bin/env bash
# Download and unpack the CARLA-Air v0.1.7 simulator (6.85 GB download, 18 GB unpacked).
#
#   ./scripts/fetch_release.sh [DEST]
#
# DEST defaults to $CARLAAIR_HOME, or a directory next to the repo. Put it on a large disk:
# this is by far the biggest thing in the project, and it does not belong on a small root
# filesystem.
#
# Use the Hugging Face library, not curl. Measured on this connection, plain curl against
# the CDN sustained ~800 KB/s and projected two hours; hf_hub_download pulled the same
# 6.85 GB in about 75 seconds because it fetches chunks in parallel.
set -euo pipefail

PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${1:-${CARLAAIR_HOME:-$(dirname "$PROJ")/carla-air-release}}"
REPO=tianlezeng/CarlaAIr-v0.1.7
ZIP=CarlaAir-v0.1.7.zip
EXPECTED_BYTES=6846384047
PY="$PROJ/.venv/bin/python"

[ -x "$PY" ] || { echo "run scripts/setup_env.sh first (no $PY)" >&2; exit 1; }

if [ -d "$DEST/CarlaAir-v0.1.7/CarlaUE4" ]; then
    echo "already unpacked: $DEST/CarlaAir-v0.1.7"
    echo "export CARLAAIR_RELEASE=$DEST/CarlaAir-v0.1.7"
    exit 0
fi

avail_gb=$(df -BG --output=avail "$(dirname "$DEST")" 2>/dev/null | tail -1 | tr -dc '0-9')
if [ -n "${avail_gb:-}" ] && [ "$avail_gb" -lt 30 ]; then
    echo "ERROR: only ${avail_gb} GB free at $(dirname "$DEST"); need ~30 GB" >&2
    echo "       (6.85 GB download + 18 GB unpacked). Pass a different DEST." >&2
    exit 1
fi

mkdir -p "$DEST/downloads"
"$PY" - "$REPO" "$ZIP" "$DEST/downloads" <<'PY'
import sys
try:
    from huggingface_hub import hf_hub_download
except ImportError:
    sys.exit("huggingface_hub missing — re-run scripts/setup_env.sh")
repo, name, dest = sys.argv[1:4]
print(f"downloading {name} (6.85 GB) ...", flush=True)
print(hf_hub_download(repo_id=repo, filename=name, local_dir=dest))
PY

ARCHIVE="$DEST/downloads/$ZIP"
actual=$(stat -c%s "$ARCHIVE")
[ "$actual" = "$EXPECTED_BYTES" ] || {
    echo "ERROR: $ZIP is $actual bytes, expected $EXPECTED_BYTES — download incomplete" >&2
    exit 1
}

echo "unpacking to $DEST (~2.5 min) ..."
unzip -q -o "$ARCHIVE" -d "$DEST"

BIN="$DEST/CarlaAir-v0.1.7/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
[ -x "$BIN" ] || { echo "ERROR: simulator binary missing after unpack: $BIN" >&2; exit 1; }

cat <<EOF

done. $(du -sh "$DEST/CarlaAir-v0.1.7" | cut -f1) unpacked.

Point the project at it (add to your shell profile to make it stick):

  export CARLAAIR_RELEASE=$DEST/CarlaAir-v0.1.7

The $(du -sh "$ARCHIVE" | cut -f1) archive in $DEST/downloads is no longer needed and can be deleted.
EOF
