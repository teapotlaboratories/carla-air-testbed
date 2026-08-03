#!/usr/bin/env bash
# Print the path to the unpacked CARLA-Air release, and nothing else.
#
# Four scripts need this path and used to each inline the same fallback expression, which
# meant four places to keep in agreement. They agreed by luck rather than by construction.
#
# Precedence, most explicit first:
#
#   1. $CARLAAIR_RELEASE     an explicit override for one command, always wins
#   2. .release-path         written by scripts/install.sh when the release went somewhere
#                            other than the default, so a custom location survives WITHOUT
#                            the user having to edit a shell profile
#   3. $CARLAAIR_HOME        the parent directory, release appended
#   4. next to the repo      the default install location
#
# Printing rather than exporting is deliberate: an exported variable would need every caller
# to source this, and sourcing under `set -u` is how scripts here have broken before.
set -uo pipefail

PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VERSION="CarlaAir-v0.1.7"

if [ -n "${CARLAAIR_RELEASE:-}" ]; then
    printf '%s\n' "${CARLAAIR_RELEASE%/}"
elif [ -s "$PROJ/.release-path" ]; then
    # First non-empty line, so a stray trailing newline or a comment appended by hand does
    # not turn into a path that almost works.
    grep -m1 -v '^[[:space:]]*$' "$PROJ/.release-path" | sed 's:/*$::'
elif [ -n "${CARLAAIR_HOME:-}" ]; then
    printf '%s/%s\n' "${CARLAAIR_HOME%/}" "$VERSION"
else
    printf '%s/carla-air-release/%s\n' "$(dirname "$PROJ")" "$VERSION"
fi
