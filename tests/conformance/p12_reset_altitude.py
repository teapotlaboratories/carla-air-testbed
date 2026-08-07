#!/usr/bin/env python3
"""P12 — does `reset()` reach the altitude it was given?

`examples/ros2_traffic_flyover.py` asked for NED z = -8.0 and the reset reported settling at
+7.5: **15.5 m low**, where the documented station-keeping tolerance is ~9 m and the same
day's episode resets came in at 4-6 m. One observation, filed as D-03, and this probe is what
turns it into a measurement.

Three candidate explanations, and the grid below separates them:

  * **altitude-dependent** — only the low hold misses, e.g. because ground effect, a collision
    volume, or a floor somewhere clamps it;
  * **speed-dependent** — the flyover resets at 8 m/s and `run_episode.py` at 10, and
    `moveToPositionAsync` returning early would show up as the slower one undershooting;
  * **variance** — it happens everywhere and the flyover was an unlucky sample.

**This calls the REAL `Vehicle.reset()`**, not a reimplementation of it. p11 mirrored the
reset by hand and spent an afternoon measuring its own copy; the fix is to import the thing
under test. Anything this reports is what a caller actually gets.

Run against a bare simulator with no ROS graph — a running offboard controller would fly the
aircraft between resets and there would be nothing left to attribute.

    ./scripts/run_sim.sh --config configs/testbed.yaml
    ./.venv/bin/python tests/conformance/p12_reset_altitude.py
"""
import math
import os
import statistics
import sys

import common

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "sim_bridge"))
from carla_air.vehicle import Vehicle  # noqa: E402

#: NED z, i.e. NEGATIVE is above the origin, and the origin sits 27.45 m above the street.
#: Chosen to span what the repository actually flies: episodes at -55, the city tour at -30,
#: the flyover at -8 (where the miss was seen), and street level at +23.95.
ALTITUDES = [-55.0, -30.0, -8.0, 23.95]
SPEEDS = [8.0, 10.0]
N = 4
#: The documented station-keeping floor. QUICKSTART calls a ~9 m start-pose error normal.
TOLERANCE_M = 9.0

#: What `Vehicle.reset()` is asked to converge to, overridable so D-05 can be answered:
#: does the converge loop CORRECT the street-level error, or exhaust its attempts against a
#: floor? A flat 6 m is sensible at 82 m AGL and meaningless at 3.5, but tightening it is only
#: worth doing if retrying actually helps.
#:
#:     ./.venv/bin/python tests/conformance/p12_reset_altitude.py --reset-tolerance 1.0
RESET_TOLERANCE = None
for _i, _a in enumerate(sys.argv):
    if _a == "--reset-tolerance" and _i + 1 < len(sys.argv):
        RESET_TOLERANCE = float(sys.argv[_i + 1])

p = common.Probe("p12_reset_altitude")
v = Vehicle(common.airsim_client())
_SHIPPED_TOLERANCE = Vehicle.RESET_TOLERANCE_M
if RESET_TOLERANCE is not None:
    # Applied to the class, because reset() reads it from there rather than taking it as an
    # argument — the point is to measure the SHIPPED code path with one constant changed.
    Vehicle.RESET_TOLERANCE_M = RESET_TOLERANCE
    p.note("reset tolerance overridden",
           f"{RESET_TOLERANCE} m (the shipped default is {_SHIPPED_TOLERANCE} m)")
p.metric("reset_tolerance_m", Vehicle.RESET_TOLERANCE_M)
p.metric("reset_attempts", Vehicle.RESET_ATTEMPTS)

rows = []
for speed in SPEEDS:
    for alt in ALTITUDES:
        errs, z_errs = [], []
        for _ in range(N):
            got = v.reset(hold_ned=(107.6, -159.4, alt), speed=speed)
            pos = [float(x) for x in got["position"]]
            errs.append(math.dist(pos, (107.6, -159.4, alt)))
            z_errs.append(pos[2] - alt)
        rows.append((speed, alt, errs, z_errs))
        agl = 27.45 - alt
        print(f"  speed {speed:4.1f}  z {alt:7.2f} ({agl:5.1f} m AGL)   "
              f"err mean {statistics.mean(errs):5.1f} worst {max(errs):5.1f} m   "
              f"z-err mean {statistics.mean(z_errs):+6.1f} m")

print()
worst = max(max(e) for _, _, e, _ in rows)
p.metric("worst_error_m", round(worst, 2))
p.metric("worst_error_by_altitude", {str(a): round(max(max(e) for s, aa, e, _ in rows if aa == a), 2)
                                     for a in ALTITUDES})
p.metric("worst_error_by_speed", {str(s): round(max(max(e) for ss, _, e, _ in rows if ss == s), 2)
                                  for s in SPEEDS})

p.check(f"every reset lands within {TOLERANCE_M:.0f} m of its commanded pose",
        worst <= TOLERANCE_M, f"worst {worst:.1f} m over {len(rows) * N} resets")

# Separate the three explanations rather than leaving a grid to be squinted at.
by_alt = {a: statistics.mean([x for s, aa, e, _ in rows if aa == a for x in e]) for a in ALTITUDES}
by_speed = {s: statistics.mean([x for ss, _, e, _ in rows if ss == s for x in e]) for s in SPEEDS}
spread_alt = max(by_alt.values()) - min(by_alt.values())
spread_speed = max(by_speed.values()) - min(by_speed.values())
p.metric("mean_error_by_altitude", {str(k): round(x, 2) for k, x in by_alt.items()})
p.metric("mean_error_by_speed", {str(k): round(x, 2) for k, x in by_speed.items()})

if worst <= TOLERANCE_M:
    p.note("no miss reproduced", "the flyover's 15.5 m did not recur in this grid — "
                                 "treat it as variance until it does")
elif spread_alt > 2 * spread_speed:
    p.note("the error tracks ALTITUDE", f"spread across altitudes {spread_alt:.1f} m "
                                        f"vs {spread_speed:.1f} m across speeds")
elif spread_speed > 2 * spread_alt:
    p.note("the error tracks SPEED", f"spread across speeds {spread_speed:.1f} m "
                                     f"vs {spread_alt:.1f} m across altitudes")
else:
    p.note("neither altitude nor speed dominates",
           f"altitude spread {spread_alt:.1f} m, speed spread {spread_speed:.1f} m — "
           "looks like variance, and the sample is small")

p.finish()
