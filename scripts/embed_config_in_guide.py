#!/usr/bin/env python
"""Re-paste `configs/testbed.yaml` into the full-file listing in `docs/guide.html`.

    ./.venv/bin/python scripts/embed_config_in_guide.py          # update the listing
    ./.venv/bin/python scripts/embed_config_in_guide.py --check  # fail if it has drifted

The guide shows the complete config file, which makes it a copy, and copies rot silently.
`tests/test_config.py::test_the_guide_embeds_the_real_config_file` fails when it has, and
this is what fixes it - so nobody hand-edits 200 lines of YAML inside an HTML file.

Same reasoning as `apply_config.py`, pointed at a reader instead of a machine.
"""
from __future__ import annotations

import argparse
import html
import os
import re
import sys

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOURCE = os.path.join(PROJ, "configs", "testbed.yaml")
GUIDE = os.path.join(PROJ, "docs", "guide.html")

#: The four top-level sections, emboldened so the structure reads at a glance in a
#: 200-line listing. Everything else is left as typed.
SECTIONS = ("simulator", "sensors", "sidecar", "graph")


def tint(yaml_text):
    """YAML to HTML, dimming comments and emboldening the section keys.

    Not a YAML highlighter and not trying to be. It splits on the first `#`, which is
    correct only because no *value* in this file contains one - asserted below rather
    than assumed, since a URL or a colour would break it silently.
    """
    lines = []
    for raw in yaml_text.rstrip("\n").split("\n"):
        m = re.match(r"^([^#]*)(#.*)$", raw)
        code, comment = (m.group(1), m.group(2)) if m else (raw, "")
        assert "#" not in comment[1:] or comment.startswith("#"), raw
        out = html.escape(code)
        out = re.sub(rf"^({'|'.join(SECTIONS)})(:)$", r"<b>\1</b>\2", out)
        if comment:
            out += f'<span class="c">{html.escape(comment)}</span>'
        lines.append(out)
    return "\n".join(lines)


def render(page, yaml_text):
    """Return `page` with the listing's <pre> replaced. Raises if the block is gone."""
    m = re.search(r'(<details class="full">.*?<pre>)(.*?)(</pre>)', page, re.S)
    if not m:
        raise SystemExit("no <details class=\"full\"> listing in docs/guide.html")
    return page[:m.start(2)] + tint(yaml_text) + page[m.end(2):]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true",
                    help="exit non-zero if the guide has drifted from the config")
    args = ap.parse_args()

    with open(SOURCE, encoding="utf-8") as fh:
        yaml_text = fh.read()
    with open(GUIDE, encoding="utf-8") as fh:
        page = fh.read()

    updated = render(page, yaml_text)
    if args.check:
        if updated != page:
            print("STALE: docs/guide.html no longer matches configs/testbed.yaml",
                  file=sys.stderr)
            return 1
        print("the guide's config listing is up to date")
        return 0

    if updated == page:
        print("already up to date")
        return 0
    with open(GUIDE, "w", encoding="utf-8") as fh:
        fh.write(updated)
    print(f"updated the listing in {os.path.relpath(GUIDE, PROJ)} "
          f"({len(yaml_text.splitlines())} lines)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
