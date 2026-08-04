#!/usr/bin/env python3
"""Read a flight trace back and say where the path went.

    ./scripts/analyse_trace.sh out/traces/run1
    ./scripts/analyse_trace.sh out/traces/a out/traces/b     # compare two runs

Runs under ROS's python 3.12, not the 3.10 venv: `rosbag2_py` is a ROS package. The wrapper
`analyse_trace.sh` sources the workspace.

Written for D-01 — the same seed producing 13 steps / 60 m or 25 steps / 125 m over the same
80 m journey, with nothing changed. A count of steps cannot distinguish "flew a longer route"
from "flew the same route twice", and that is the whole question, so this reports the shape:

  * path length against net displacement — the ratio is how much of the flying was wasted,
  * per-leg distance and heading, so a doubled leg or a reversal is visible,
  * where the aircraft turned round, measured as travel opposed to the goal direction,
  * waypoint arrivals and how far each waypoint was from the one before.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from collections import defaultdict

import rclpy.serialization
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


def read(path):
    """Every message in the bag, as (topic, t_seconds, msg), in time order."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                    output_serialization_format="cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        try:
            msg = rclpy.serialization.deserialize_message(data, get_message(types[topic]))
        except Exception:                                              # noqa: BLE001
            continue
        out.append((topic, stamp * 1e-9, msg))
    return out


def summarise(path):
    msgs = read(path)
    if not msgs:
        return None
    by_topic = defaultdict(list)
    for topic, t, m in msgs:
        by_topic[topic].append((t, m))

    # Window to the EPISODE, not the whole recording. A trace normally starts before the
    # reset, and the reset flies the aircraft across the map to the start pose — 200+ m of
    # travel that belongs to no episode and swamps every ratio below if it is counted.
    status = by_topic.get("/episode/status", [])
    running = [t for t, m in status if getattr(m, "state", "") == "running"]
    ended = [t for t, m in status if getattr(m, "state", "") in ("success", "failure", "aborted")]
    lo = running[0] if running else None
    # The END must come AFTER the start. /episode/status is latched, so a recording that
    # begins between two episodes receives the PREVIOUS episode's terminal status first —
    # taking ended[0] blindly gives hi < lo, an empty window, and "0 odometry samples" for a
    # bag that is perfectly good. Found by running this on six traces that all looked fine.
    hi = next((t for t in ended if lo is None or t > lo), None)
    window = ""
    if lo is not None:
        window = f"  (episode window {lo - by_topic['/fmu/out/vehicle_odometry'][0][0]:.1f}s .. " \
                 f"{'end' if hi is None else f'{hi - lo:.1f}s'})"

    def inside(t):
        return (lo is None or t >= lo) and (hi is None or t <= hi)

    odom = [(t, [float(v) for v in m.position])
            for t, m in by_topic["/fmu/out/vehicle_odometry"] if inside(t)]
    if len(odom) < 2:
        print(f"  {path}: only {len(odom)} odometry samples in the episode — nothing to say")
        return None

    t0 = odom[0][0]
    start, end = odom[0][1], odom[-1][1]

    # Path length, ignoring the sub-centimetre jitter that station-keeping produces at rest;
    # summing raw samples at 20 Hz turns a stationary hover into kilometres of "path".
    seg = [math.dist(a[1], b[1]) for a, b in zip(odom, odom[1:])]
    path_len = sum(d for d in seg if d > 0.01)
    net = math.dist(start, end)

    waypoints = [(t, m) for t, m in by_topic.get("/control/waypoint", []) if inside(t)]
    arrivals = [t for t, m in by_topic.get("/control/arrived", [])
                if inside(t) and getattr(m, "data", True)]
    bearing_only = sum(1 for _, m in waypoints if getattr(m, "bearing_only", False))

    # How much travel went AGAINST the net direction. A straight run is ~0; flying out and
    # back shows up here and nowhere else in the numbers we currently store.
    if net > 1e-6:
        u = [(end[i] - start[i]) / net for i in range(3)]
        back = sum(-d for d in
                   (sum((b[1][i] - a[1][i]) * u[i] for i in range(3)) for a, b in zip(odom, odom[1:]))
                   if d < -0.01)
    else:
        back = float("nan")

    print(f"\n=== {path} ==={window}")
    print(f"  duration        {odom[-1][0] - t0:8.1f} s   ({len(odom)} odometry samples)")
    print(f"  start           {[round(v, 1) for v in start]}")
    print(f"  end             {[round(v, 1) for v in end]}")
    print(f"  net displacement{net:8.1f} m")
    print(f"  path length     {path_len:8.1f} m")
    print(f"  wasted ratio    {path_len / net if net else float('nan'):8.2f}   "
          f"(1.0 = a straight line)")
    print(f"  travel AGAINST the net direction {back:6.1f} m")
    print(f"  waypoints       {len(waypoints):8d}   ({bearing_only} bearing-only)")
    print(f"  arrivals        {len(arrivals):8d}")

    if waypoints:
        print("\n  leg   t(s)   waypoint NED                 gap from previous")
        prev = None
        for i, (t, m) in enumerate(waypoints[:40], 1):
            p = m.position
            here = [p.x, p.y, p.z]
            gap = f"{math.dist(here, prev):6.1f} m" if prev else "     —"
            flag = " BEARING-ONLY" if getattr(m, "bearing_only", False) else ""
            print(f"  {i:3d}  {t - t0:6.1f}   "
                  f"[{here[0]:7.1f},{here[1]:8.1f},{here[2]:7.1f}]  {gap}{flag}")
            prev = here
        if len(waypoints) > 40:
            print(f"  ... {len(waypoints) - 40} more")

    return {"path": path_len, "net": net, "waypoints": len(waypoints),
            "arrivals": len(arrivals), "back": back}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("traces", nargs="+", help="bag directories written by record_trace.sh")
    args = ap.parse_args()

    got = []
    for t in args.traces:
        if not os.path.exists(os.path.join(t, "metadata.yaml")):
            print(f"  {t}: no metadata.yaml — not a bag", file=sys.stderr)
            continue
        s = summarise(t)
        if s:
            got.append((t, s))

    if len(got) >= 2:
        print("\n=== comparison ===")
        print(f"  {'trace':38s} {'path':>8s} {'net':>7s} {'ratio':>7s} {'back':>7s} {'wps':>5s}")
        for name, s in got:
            print(f"  {os.path.basename(name):38s} {s['path']:8.1f} {s['net']:7.1f} "
                  f"{s['path'] / s['net'] if s['net'] else float('nan'):7.2f} "
                  f"{s['back']:7.1f} {s['waypoints']:5d}")


if __name__ == "__main__":
    main()
