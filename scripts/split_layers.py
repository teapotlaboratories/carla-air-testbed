#!/usr/bin/env python3
"""Given a trace, say WHICH layer oscillated: the pixel, or the projection of it.

    ./scripts/analyse_trace.sh --split out/traces/run9      # via the wrapper

D-01's failing runs zigzag laterally around the goal line. That can come from either side of
the See-Point-Fly seam and the two need completely different owners:

  * the ANNOTATION alternates  -> whatever is choosing pixels is oscillating. Out of scope.
  * the annotation is steady but the WAYPOINT alternates -> the pixel-to-NED projection is
    oscillating: camera pose, intrinsics, or a stale-frame mismatch. Simulator-side, in scope.

Reports both series side by side, in the episode window, so the answer is read rather than
argued.
"""
from __future__ import annotations

import statistics
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from analyse_trace import read  # noqa: E402


def split(path):
    by = {}
    for topic, t, m in read(path):
        by.setdefault(topic, []).append((t, m))

    status = by.get("/episode/status", [])
    run = [t for t, m in status if getattr(m, "state", "") == "running"]
    lo = run[0] if run else None
    end = [t for t, m in status
           if getattr(m, "state", "") in ("success", "failure", "aborted") and (lo is None or t > lo)]
    hi = end[0] if end else None
    ok = lambda t: (lo is None or t >= lo) and (hi is None or t <= hi)   # noqa: E731

    ann = [(t, m) for t, m in by.get("/vlm/annotation", []) if ok(t)]
    wps = [(t, m) for t, m in by.get("/control/waypoint", []) if ok(t)]
    print(f"\n=== {path} ===   {len(ann)} annotations, {len(wps)} waypoints in the episode")
    if not ann or not wps:
        print("  nothing to split")
        return

    print(f"\n  {'#':>3} {'t(s)':>6}   {'pixel u,v':>12}   {'waypoint y (NED)':>17}  side")
    t0 = ann[0][0]
    us, ys = [], []
    for i, (t, a) in enumerate(ann[:30], 1):
        # the waypoint grounded from this annotation is the next one published
        w = next((m for wt, m in wps if wt >= t), None)
        y = w.position.y if w is not None else float("nan")
        us.append(float(a.u)); ys.append(y)
        side = "" if w is None else ("  LEFT" if y > -159.4 else "  right")
        print(f"  {i:3d} {t - t0:6.1f}   {a.u:5d},{a.v:5d}   {y:17.1f}{side}")

    def flips(seq, mid):
        s = [x > mid for x in seq if x == x]
        return sum(1 for a, b in zip(s, s[1:]) if a != b)

    umid = statistics.median(us)
    print(f"\n  pixel u:     median {umid:7.1f}   spread {max(us) - min(us):7.1f} px   "
          f"side-flips {flips(us, umid)}")
    yy = [y for y in ys if y == y]
    print(f"  waypoint y:  median {statistics.median(yy):7.1f}   "
          f"spread {max(yy) - min(yy):7.1f} m    side-flips {flips(yy, -159.4)}")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        split(p)
