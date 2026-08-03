#!/usr/bin/env bash
# The whole install, in one command.
#
#   ./scripts/install.sh                # release lands next to the repo
#   ./scripts/install.sh /mnt/big       # release lands there instead (needs ~30 GB)
#   ./scripts/install.sh --skip-release # everything except the 6.85 GB download
#
# Four steps, already ordered and already idempotent, so re-running after a failure resumes
# rather than starting over. What this adds beyond `a && b && c && d`:
#
#   * one failure path, naming the step and the log to read
#   * the release location is REMEMBERED. fetch_release.sh prints an
#     `export CARLAAIR_RELEASE=...` line that every later script needs; an install wrapper
#     that swallows it has silently broken the next command the user runs. Instead the path
#     is written to .release-path, which scripts/release_path.sh reads - so a custom install
#     location needs no shell-profile edit at all.
set -uo pipefail

PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VERSION="CarlaAir-v0.1.7"
DEST=""
SKIP_RELEASE=0

while [ $# -gt 0 ]; do
    case "$1" in
        --skip-release) SKIP_RELEASE=1; shift ;;
        -h|--help) sed -n '2,8p' "$0"; exit 0 ;;
        -*) echo "unknown option: $1" >&2; exit 2 ;;
        *) DEST="$1"; shift ;;
    esac
done

STEPS=4
[ "$SKIP_RELEASE" -eq 1 ] && STEPS=3
STARTED="$(date +%s)"
step_no=0

step() {
    step_no=$((step_no + 1))
    local label="$1"; shift
    local began; began="$(date +%s)"
    echo
    echo "=== [$step_no/$STEPS] $label ==="
    if ! "$@"; then
        echo >&2
        echo "INSTALL FAILED at step $step_no: $label" >&2
        echo "  command: $*" >&2
        echo "  re-run ./scripts/install.sh once fixed - completed steps are skipped." >&2
        exit 1
    fi
    echo "--- $label done in $(( $(date +%s) - began ))s"
}

echo "carla-air testbed - installing into $PROJ"

step "python 3.10 environment"  bash "$PROJ/scripts/setup_env.sh"

if [ "$SKIP_RELEASE" -eq 1 ]; then
    echo
    echo "=== skipping the simulator download (--skip-release) ==="
else
    if [ -n "$DEST" ]; then
        step "simulator release (6.85 GB -> 18 GB)" bash "$PROJ/scripts/fetch_release.sh" "$DEST"
    else
        step "simulator release (6.85 GB -> 18 GB)" bash "$PROJ/scripts/fetch_release.sh"
    fi

    # Remember where it went. Only when it is NOT the default, so a normal install leaves no
    # state file to go stale - release_path.sh already computes the default.
    default_parent="$(dirname "$PROJ")/carla-air-release"
    chosen="${DEST:-${CARLAAIR_HOME:-$default_parent}}/$VERSION"
    if [ "$chosen" != "$default_parent/$VERSION" ]; then
        printf '%s\n' "$chosen" > "$PROJ/.release-path"
        echo "    remembered in .release-path (no shell profile edit needed)"
    fi
fi

step "pinned upstreams (px4_msgs, anthropic SDK)" bash "$PROJ/scripts/fetch_vendor.sh"
step "ROS 2 workspace"                            bash "$PROJ/scripts/build_ros.sh"

RELEASE="$("$PROJ/scripts/release_path.sh")"
echo
echo "=============================================================="
echo "installed in $(( $(date +%s) - STARTED ))s"
echo "  release: $RELEASE"
if [ ! -x "$RELEASE/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" ]; then
    echo "           (not present - run without --skip-release to download it)"
fi
echo
echo "next:"
echo "  ./.venv/bin/python -m pytest tests/ -q     # no simulator needed"
echo "  ./scripts/bringup.sh --backend oracle"
echo "=============================================================="
