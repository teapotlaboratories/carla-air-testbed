#!/usr/bin/env bash
# Install this repository's git hooks.
#
#   ./scripts/install_hooks.sh
#
# Points `core.hooksPath` at .githooks/, so the hooks are versioned with the code rather than
# living untracked in .git/hooks where a fresh clone silently has none.
#
# What gets installed: a pre-commit hook that refuses code on the default branch. See
# .githooks/pre-commit for why that needed a mechanism rather than a firmer sentence.
set -uo pipefail
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJ" || exit 1

git rev-parse --git-dir >/dev/null 2>&1 || { echo "not a git repository" >&2; exit 1; }
git config core.hooksPath .githooks
chmod +x .githooks/* 2>/dev/null || true

echo "hooks installed — core.hooksPath = $(git config core.hooksPath)"
for h in .githooks/*; do
    [ -f "$h" ] && printf '  %-14s %s\n' "$(basename "$h")" "$([ -x "$h" ] && echo executable || echo 'NOT EXECUTABLE')"
done
echo
echo "verify with:  git config core.hooksPath"
