#!/usr/bin/env python
"""Ask CARLA where the buildings actually are, in the frame the scenarios are written in.

Runs under the 3.10 sidecar interpreter (`.venv/bin/python`) because it imports `carla`.
It only reads the world, so it is safe to run against a simulator that is already serving
an episode.

Four modes. All of them cache `out/buildings.json`, which `--cached` reuses so the rest can
run with no simulator at all:

    ./.venv/bin/python scripts/survey_buildings.py --check          # is a scenario real?
    ./.venv/bin/python scripts/survey_buildings.py --route          # is its goal reachable?
    ./.venv/bin/python scripts/survey_buildings.py --propose --span 110
    ./.venv/bin/python scripts/survey_buildings.py --top 20

`--check` is the one that matters for the backlog: it reports, per scenario, whether the
straight line from start to goal passes through a building. A scenario that advertises
obstacle avoidance and reports `clear` is measuring something other than its name.

`--route` is its counterpart, and exists because of a rule collision. `.ai/AGENTS.md` says to
validate a scenario with the `oracle` backend and treat an oracle failure as a broken
scenario — but the oracle flies straight at the goal, so a scenario built to block the
straight line fails it by construction. For those, `--route` proves reachability with A*
instead, and prices the detour.

**Altitudes read strangely until you hold the offset in your head.** The AirSim NED origin
on Town10HD sits 27.45 m ABOVE CARLA's ground plane, so NED z = -50 is 77.45 m above the
street, not 50. Every rooftop in this map is below that, which is the whole finding: the
original four scenarios were not badly sited, they were flying over the entire city.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim_bridge.carla_air.frames import DEFAULT_OFFSET, carla_to_ned  # noqa: E402

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(PROJ, "out")
SCENARIOS = os.path.join(PROJ, "ros2_ws", "src", "evaluation", "scenarios", "default.yaml")
PARAMS = os.path.join(PROJ, "ros2_ws", "src", "bringup", "config", "testbed.yaml")

# Ground level in NED. CARLA z=0 is the street, and the offset puts that at +27.45 NED.
GROUND_NED_Z = DEFAULT_OFFSET[2]


def _ctrl_min_altitude():
    """The controller's altitude floor, read from the same file `tests/test_scenarios.py`
    lints against so the two can never disagree."""
    import yaml
    with open(PARAMS) as f:
        return float(yaml.safe_load(f)["offboard_control"]["ros__parameters"]["min_altitude_m"])


CTRL_MIN_ALT_M = _ctrl_min_altitude()


def survey(host="127.0.0.1", port=2000, timeout=20.0):
    """Every building in the map as an axis-aligned box in NED metres.

    CARLA hands back an oriented box per object. Rotating the eight corners and taking their
    extent gives an AABB that is never smaller than the true footprint, so a segment that
    misses this box provably misses the building. It can be slightly generous on a rotated
    building, which is the harmless direction for siting an obstacle.
    """
    import carla

    client = carla.Client(host, port)
    client.set_timeout(timeout)
    world = client.get_world()
    objs = world.get_environment_objects(carla.CityObjectLabel.Buildings)

    out = []
    for o in objs:
        bb = o.bounding_box
        e = bb.extent
        # Local corners -> world, through the box's own rotation.
        c, s = math.cos(math.radians(bb.rotation.yaw)), math.sin(math.radians(bb.rotation.yaw))
        xs, ys, zs = [], [], []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    lx, ly, lz = sx * e.x, sy * e.y, sz * e.z
                    xs.append(bb.location.x + lx * c - ly * s)
                    ys.append(bb.location.y + lx * s + ly * c)
                    zs.append(bb.location.z + lz)

        lo = carla_to_ned(min(xs), min(ys), max(zs))  # max CARLA z -> min NED z
        hi = carla_to_ned(max(xs), max(ys), min(zs))
        out.append({
            "name": o.name,
            "id": o.id,
            "min_ned": [round(v, 2) for v in lo],
            "max_ned": [round(v, 2) for v in hi],
            "height_m": round(2 * e.z, 2),
            "roof_agl_m": round(GROUND_NED_Z - lo[2], 2),
            "footprint_m": [round(2 * e.x, 2), round(2 * e.y, 2)],
        })
    return out


def segment_hits_box(p0, p1, lo, hi, inflate=0.0):
    """Slab test. Returns the (t_enter, t_exit) sub-interval inside the box, or None.

    `inflate` widens the box in every axis — a clearance margin, so "the line passes 1 m
    from a wall" counts as a hit rather than a pass.
    """
    t0, t1 = 0.0, 1.0
    for i in range(3):
        a, b = lo[i] - inflate, hi[i] + inflate
        d = p1[i] - p0[i]
        if abs(d) < 1e-9:
            if not (a <= p0[i] <= b):
                return None
            continue
        ta, tb = (a - p0[i]) / d, (b - p0[i]) / d
        if ta > tb:
            ta, tb = tb, ta
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 > t1:
            return None
    return (t0, t1)


def obstructions(p0, p1, buildings, inflate=0.0):
    """Buildings the segment passes through, nearest first, with how much of it is inside."""
    hits = []
    length = math.dist(p0, p1)
    for b in buildings:
        r = segment_hits_box(p0, p1, b["min_ned"], b["max_ned"], inflate)
        if r is None:
            continue
        t0, t1 = r
        hits.append({
            "name": b["name"],
            "entry_m": round(t0 * length, 1),
            "through_m": round((t1 - t0) * length, 1),
            "roof_agl_m": b["roof_agl_m"],
            "min_ned": b["min_ned"],
            "max_ned": b["max_ned"],
        })
    return sorted(hits, key=lambda h: h["entry_m"])


def solid_length(p0, p1, buildings, inflate=0.0):
    """Metres of the segment actually inside geometry, counting overlap once.

    Summing per-piece hits instead double-counts badly: a procedural facade stacks windows,
    trims and wall panels in the same volume, and a naive sum reported 143 m of obstacle on a
    90 m line.
    """
    spans = []
    for b in buildings:
        r = segment_hits_box(p0, p1, b["min_ned"], b["max_ned"], inflate)
        if r is not None:
            spans.append(r)
    if not spans:
        return 0.0
    spans.sort()
    total, lo, hi = 0.0, spans[0][0], spans[0][1]
    for s, e in spans[1:]:
        if s > hi:
            total += hi - lo
            lo, hi = s, e
        else:
            hi = max(hi, e)
    return (total + hi - lo) * math.dist(p0, p1)


def is_free(point, buildings, margin):
    """True when no geometry comes within `margin` of the point, in any axis."""
    return not any(
        all(b["min_ned"][d] - margin <= point[d] <= b["max_ned"][d] + margin for d in range(3))
        for b in buildings
    )


def column_roof(lo_xy, hi_xy, buildings):
    """Highest geometry anywhere over a 2D footprint, in metres AGL.

    Taking the roof from only the pieces the segment hit underestimates it badly: those are
    the pieces solid at flight altitude, and a tower's upper floors are different pieces.
    That error made a 154 m skyscraper look 31 m tall.
    """
    top = None
    for b in buildings:
        if (b["min_ned"][0] <= hi_xy[0] and lo_xy[0] <= b["max_ned"][0]
                and b["min_ned"][1] <= hi_xy[1] and lo_xy[1] <= b["max_ned"][1]):
            top = b["min_ned"][2] if top is None else min(top, b["min_ned"][2])
    return None if top is None else GROUND_NED_Z - top


def route(p0, p1, buildings, clearance=8.0, cell=2.0, pad=200.0):
    """Shortest obstacle-free path between two NED points at a fixed altitude.

    A* on an 8-connected grid of the horizontal plane, cells blocked where any geometry
    solid at that altitude comes within `clearance`. Returns (length_m, waypoints) or
    (None, []) when the goal is unreachable.

    `pad` is how far outside the start/goal bounding box to search, and it is not a
    formality: at 80 m this reported `avoid_the_block` UNREACHABLE, because the detour needs
    to leave that box. A wrong "unreachable" reads exactly like a broken scenario, so the
    default is deliberately generous. Widen it before believing a negative result.

    This exists because of a rule collision. `.ai/AGENTS.md` says to validate a scenario with
    the `oracle` backend and treat an oracle failure as a broken scenario — but the oracle is
    a straight-line policy, so on a scenario that deliberately blocks the straight line it
    fails *by construction*. For those, reachability has to be proven geometrically instead,
    and this is that proof.
    """
    z = p0[2]
    band = [b for b in buildings if b["min_ned"][2] <= z <= b["max_ned"][2]]
    lo_x, hi_x = min(p0[0], p1[0]) - pad, max(p0[0], p1[0]) + pad
    lo_y, hi_y = min(p0[1], p1[1]) - pad, max(p0[1], p1[1]) + pad
    nx = int((hi_x - lo_x) / cell) + 1
    ny = int((hi_y - lo_y) / cell) + 1

    blocked = bytearray(nx * ny)
    for b in band:
        bx0 = int((b["min_ned"][0] - clearance - lo_x) / cell)
        bx1 = int((b["max_ned"][0] + clearance - lo_x) / cell)
        by0 = int((b["min_ned"][1] - clearance - lo_y) / cell)
        by1 = int((b["max_ned"][1] + clearance - lo_y) / cell)
        for ix in range(max(0, bx0), min(nx - 1, bx1) + 1):
            base = ix * ny
            for iy in range(max(0, by0), min(ny - 1, by1) + 1):
                blocked[base + iy] = 1

    def to_cell(p):
        return (int((p[0] - lo_x) / cell), int((p[1] - lo_y) / cell))

    start, goal = to_cell(p0), to_cell(p1)
    if blocked[start[0] * ny + start[1]] or blocked[goal[0] * ny + goal[1]]:
        return None, []

    import heapq
    diag = math.sqrt(2.0)
    steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, diag), (-1, 1, diag), (1, -1, diag), (1, 1, diag)]
    dist = {start: 0.0}
    prev = {}
    heap = [(0.0, start)]
    while heap:
        _, cur = heapq.heappop(heap)
        if cur == goal:
            break
        d = dist[cur]
        for dx, dy, w in steps:
            nxt = (cur[0] + dx, cur[1] + dy)
            if not (0 <= nxt[0] < nx and 0 <= nxt[1] < ny):
                continue
            if blocked[nxt[0] * ny + nxt[1]]:
                continue
            nd = d + w
            if nd < dist.get(nxt, 1e18):
                dist[nxt] = nd
                prev[nxt] = cur
                h = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(heap, (nd + h, nxt))
    if goal not in dist:
        return None, []

    path, cur = [], goal
    while cur != start:
        path.append(cur)
        cur = prev[cur]
    path.append(start)
    path.reverse()
    pts = [(lo_x + c[0] * cell, lo_y + c[1] * cell, z) for c in path]
    return dist[goal] * cell, pts


def corners(pts, tol=1.0):
    """Thin an A* path down to the points where it actually turns."""
    if len(pts) < 3:
        return pts
    out = [pts[0]]
    for a, b, c in zip(pts, pts[1:], pts[2:]):
        v1 = (b[0] - a[0], b[1] - a[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        if abs(v1[0] * v2[1] - v1[1] * v2[0]) > tol:
            out.append(b)
    out.append(pts[-1])
    return out


def load_scenarios():
    import yaml
    with open(SCENARIOS) as f:
        return yaml.safe_load(f)["scenarios"]


def cmd_top(buildings, n):
    tall = sorted(buildings, key=lambda b: -b["roof_agl_m"])[:n]
    print(f"{'roof AGL':>9}  {'height':>7}  {'footprint':>15}  {'centre NED (x, y)':>22}  name")
    print("-" * 96)
    for b in tall:
        cx = (b["min_ned"][0] + b["max_ned"][0]) / 2
        cy = (b["min_ned"][1] + b["max_ned"][1]) / 2
        fp = f"{b['footprint_m'][0]:.0f} x {b['footprint_m'][1]:.0f}"
        print(f"{b['roof_agl_m']:9.1f}  {b['height_m']:7.1f}  {fp:>15}  "
              f"{cx:10.1f}, {cy:10.1f}  {b['name'][:34]}")


def cmd_route(buildings, clearance):
    """Prove each scenario's goal is reachable, and say how far the detour costs.

    The straight line is tested in 3D first. Only a blocked scenario goes to A*, which routes
    in the horizontal plane at flight altitude — that is the right model for going *around*
    something and the wrong one for `rain_descent`, whose start and goal share an x and y and
    which the planar router would score as a zero-length path.
    """
    print(f"{clearance:.0f} m clearance from every wall\n")
    for s in load_scenarios():
        p0, p1 = tuple(s["start_ned"]), tuple(s["goal_ned"])
        straight = math.dist(p0, p1)

        if not obstructions(p0, p1, buildings, clearance):
            print(f"{s['name']:<20} {straight:6.1f} m — straight line is clear, no detour needed")
            continue

        if math.hypot(p1[0] - p0[0], p1[1] - p0[1]) < 1.0:
            print(f"{s['name']:<20} blocked on a vertical leg — a planar route cannot help; "
                  f"re-site it or change the altitude")
            continue

        length, pts = route(p0, p1, buildings, clearance)
        if length is None:
            print(f"{s['name']:<20} UNREACHABLE at {clearance:.0f} m clearance "
                  f"(try a larger --pad before believing this)")
            continue
        turns = corners(pts)
        print(f"{s['name']:<20} {length:6.1f} m vs {straight:6.1f} m straight "
              f"(+{length - straight:5.1f} m, {length / straight:.2f}x), {len(turns) - 2} turn(s)")
        if length / straight > 2.0:
            print(f"{'':<22}NOTE: a detour that large is probably a bigger task than the "
                  f"instruction describes")
    print()


def cmd_check(buildings, inflate):
    print(f"clearance margin: {inflate:.1f} m\n")
    for s in load_scenarios():
        p0, p1 = s["start_ned"], s["goal_ned"]
        agl = GROUND_NED_Z - p0[2]
        hits = obstructions(p0, p1, buildings, inflate)
        verdict = f"{len(hits)} building(s) in the way" if hits else "CLEAR — straight line solves it"
        print(f"{s['name']:<20} {math.dist(p0, p1):5.0f} m at {agl:5.1f} m AGL   {verdict}")
        for h in hits[:4]:
            print(f"{'':<22}+{h['entry_m']:6.1f} m  through {h['through_m']:5.1f} m  "
                  f"roof {h['roof_agl_m']:5.1f} m AGL  {h['name'][:36]}")
    print()


def cmd_propose(buildings, span, clearance, min_roof):
    """Search for start/goal pairs that a straight line genuinely cannot solve.

    Four conditions, and dropping any one of them produces a scenario that looks like an
    obstacle course and is not:

    * the obstacle is 25..55 m into the leg — closer and the aircraft starts at the wall,
      further and it reaches the goal before meeting anything;
    * 15..50 m of solid geometry on the line — enough to be unmistakable, not a map edge;
    * 12..40 m of lateral detour, so grazing a corner does not count;
    * a roof at least 15 m above flight altitude, so climbing is not the cheap answer.

    Altitudes are restricted to what the controller will actually fly. That clamp is on NED
    altitude, and the NED origin is 27.45 m above the street, so the real floor is 42.45 m
    AGL — a scenario written below it gets silently re-targeted.
    """
    lo_agl = GROUND_NED_Z + CTRL_MIN_ALT_M
    agls = [a for a in range(int(math.ceil(lo_agl)), 121, 5)]
    print(f"searching {len(agls)} altitudes from {lo_agl:.1f} m AGL "
          f"(the controller's {CTRL_MIN_ALT_M:.0f} m NED floor)\n")

    rows = []
    for agl in agls:
        z = GROUND_NED_Z - agl
        band = [b for b in buildings if b["min_ned"][2] <= z <= b["max_ned"][2]]
        if not band:
            continue
        for sx in range(70, 230, 10):
            for sy in range(-140, -330, -10):
                p0 = (float(sx), float(sy), z)
                if not is_free(p0, band, 12.0):
                    continue
                for axis in (0, 1):
                    for sign in (1, -1):
                        p1 = list(p0)
                        p1[axis] += sign * span
                        p1 = tuple(p1)
                        if not is_free(p1, band, 12.0):
                            continue
                        hits = obstructions(p0, p1, band, clearance)
                        if not hits:
                            continue
                        entry = hits[0]["entry_m"]
                        thru = solid_length(p0, p1, band, clearance)
                        if not (25 <= entry <= 55 and 15 <= thru <= 50):
                            continue
                        lat = 1 - axis
                        blo = [min(h["min_ned"][d] for h in hits) for d in range(2)]
                        bhi = [max(h["max_ned"][d] for h in hits) for d in range(2)]
                        detour = min(abs(bhi[lat] - p0[lat]), abs(p0[lat] - blo[lat]))
                        if not (12 <= detour <= 40):
                            continue
                        roof = column_roof(blo, bhi, buildings)
                        if roof is None or roof - agl < min_roof:
                            continue
                        rows.append((thru, entry, detour, roof, agl, p0, p1, hits))

    rows.sort(key=lambda r: (-r[0], -r[2]))
    print(f"{len(rows)} viable legs\n")
    print(f"{'thru':>6}{'entry':>6}{'detour':>7}{'roof':>8}{'AGL':>6}   start -> goal")
    seen = set()
    for thru, entry, detour, roof, agl, p0, p1, hits in rows:
        key = (round(p0[0] / 25), round(p0[1] / 25), p0[0] == p1[0])
        if key in seen:
            continue
        seen.add(key)
        print(f"{thru:6.1f}{entry:6.1f}{detour:7.1f}{roof:8.1f}{agl:6d}   "
              f"[{p0[0]:.1f}, {p0[1]:.1f}, {p0[2]:.2f}] -> [{p1[0]:.1f}, {p1[1]:.1f}, {p1[2]:.2f}]")
        print(f"{'':>33}   {hits[0]['name'][:48]}")
        if len(seen) >= 12:
            break


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--top", type=int, metavar="N", help="list the N tallest buildings")
    ap.add_argument("--check", action="store_true",
                    help="report which scenarios a straight line already solves")
    ap.add_argument("--propose", action="store_true",
                    help="suggest start/goal pairs that a straight line cannot solve")
    ap.add_argument("--route", action="store_true",
                    help="prove every scenario's goal is reachable, and cost the detour")
    ap.add_argument("--span", type=float, default=70.0, help="proposed leg length, metres")
    ap.add_argument("--clearance", type=float, default=3.0,
                    help="margin added to every box, metres")
    ap.add_argument("--min-roof", type=float, default=25.0,
                    help="ignore buildings whose roof is below this AGL, metres")
    ap.add_argument("--cached", action="store_true",
                    help="reuse out/buildings.json instead of querying the simulator")
    args = ap.parse_args()

    path = os.path.join(OUT, "buildings.json")
    if args.cached:
        with open(path) as f:
            buildings = json.load(f)
    else:
        buildings = survey()
        os.makedirs(OUT, exist_ok=True)
        with open(path, "w") as f:
            json.dump(buildings, f, indent=1)
    print(f"{len(buildings)} buildings  (ground is NED z = +{GROUND_NED_Z:.2f})\n")

    if args.top:
        cmd_top(buildings, args.top)
    if args.check:
        cmd_check(buildings, args.clearance)
    if args.route:
        cmd_route(buildings, max(args.clearance, 8.0))
    if args.propose:
        cmd_propose(buildings, args.span, args.clearance, args.min_roof)
    if not (args.top or args.check or args.route or args.propose):
        ap.print_help()


if __name__ == "__main__":
    main()
