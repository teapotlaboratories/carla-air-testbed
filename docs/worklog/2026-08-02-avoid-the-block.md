# 2026-08-02 — E-02: making `avoid_the_block` earn its name

Backlog item [E-02](../todo.md). The scenario advertised obstacle avoidance and measured
nothing of the kind: in the 40-episode sweep the oracle solved it in a median 9 steps on a
near-straight line. The task was to re-site it so the straight line genuinely intersects a
building, with the verification being that **the oracle's success rate should drop**.

Simulator on GPU 1 (`TESTBED_GPU=1`), 3271 MiB — hardware rendering confirmed at launch.

---

## 1. Why no building was ever in the path

Wrote `scripts/survey_buildings.py`, which pulls
`world.get_environment_objects(CityObjectLabel.Buildings)`, converts each oriented box to an
axis-aligned box in NED, and slab-tests the start→goal segment against every one of them.

`--check` against the four shipped scenarios:

```
cross_the_plaza         80 m at  82.5 m AGL   CLEAR — straight line solves it
follow_the_avenue       90 m at  72.5 m AGL   CLEAR — straight line solves it
rain_descent            45 m at 107.5 m AGL   CLEAR — straight line solves it
avoid_the_block         70 m at  77.5 m AGL   CLEAR — straight line solves it
```

**The cause is altitude, not siting.** All four fly at 72–107 m AGL. Only 29 of the 1301
building pieces in the scenario region are solid that high, and none of them is anywhere near
these legs. The aircraft was flying over the entire city.

This was invisible from the coordinates alone because **NED altitude is not AGL**. The AirSim
origin on Town10HD sits 27.45 m above the street, so the scenario's `z = -50` is 77.45 m over
the ground, not 50. Every one of these scenarios was written a full storey-count higher than
it reads.

## 2. What the map actually contains

13352 building entries — but they are *pieces* of procedural buildings
(`ProceduralBuilding_<N>_Inst_<i>_<j>`, 945 distinct prefixes), not whole buildings. That is
the right granularity for an occupancy test and the wrong one for anything reported to a
human, which cost two false starts:

- Ranking pieces by height put a **1.6 m tall panel at 207 m AGL** at the top of "tallest
  buildings". It is the top floor slab of a distant backdrop tower.
- Taking the obstacle roof from only the pieces the segment hit made a 154 m skyscraper look
  31 m tall — those pieces are the ones solid *at flight altitude*, and the floors above are
  different pieces. `column_roof()` now scans every piece overlapping the 2D footprint.

A third error worth recording: summing per-piece hit lengths reported **143 m of obstacle on
a 90 m line**. Procedural facades stack windows, trim and wall panels in the same volume.
`solid_length()` merges the intervals before measuring.

## 3. The obstacle

Grid search over legal altitudes, requiring all four of: contact 25–55 m into the leg, 15–50 m
of solid geometry, 12–40 m of lateral detour, and a roof ≥15 m above flight altitude. Dropping
any one of them yields something that looks like an obstacle course and is not — the early
candidates were mostly corner grazes with a 1.2 m detour, or roofs 1 m above the flight level
where a token climb solves it.

One clear winner: **`ProceduralBuilding_94`**, a 30 × 30 m tower, 310 pieces, roof 154.3 m AGL,
footprint NED x 195.4..225.3, y −303.7..−273.8.

```
leg             110 m, NED (210, -230) -> (210, -340)
first contact   41.8 m into the leg
solid depth     33.9 m of tower on the straight line
around it       14.6 m west, 15.3 m east
over it         154.3 m AGL
the goal        36.3 m past the far face; the 20 m success ball
                stops 16.3 m clear of the wall
```

The tower is a clean vertical prism, so those numbers hold at every altitude. **Which
altitude to fly it at is a separate question, and my first answer to it was wrong — see §7.**

## 4. A constraint the linter caught and I had not

First draft put the leg at 40 m AGL. `tests/test_scenarios.py` rejected it:

```
avoid_the_block start altitude 13 m is outside the controller's [15.0, 120.0] m clamp
```

The controller clamps **NED** altitude, and NED altitude is measured from an origin 27.45 m
above the street — so the legal band on this map is **42.45..147.45 m AGL**, not 15..120. This
is the same offset trap as §1 wearing a different hat, and it is very likely why the original
scenarios were written high in the first place.

~~Settled at 50 m AGL (NED z = −22.55), which is clear of the floor and still 100 m below the
tower's roof.~~ **Superseded — see §7.** 50 m AGL clears the linter and blocks the straight
line, and is still the wrong altitude.

Both facts are now comments in `default.yaml` next to the coordinates they constrain.

## 5. Direction

The old instruction read "fly north but go around the tall building". The old leg ran from
y = −159.4 to −229.4 — and in NED, y is **East**, so decreasing y is **west**. The instruction
had the heading wrong, on coordinates that never met a building anyway. The new instruction is
written from the camera's point of view rather than the compass, which is what a VLM can
actually act on:

> "fly to the far side of the tall tower ahead, going around it rather than through it"

## 6. First verification run, at 50 m AGL

Oracle, `avoid_the_block`, the same 5 seeds as E-01:

```
oracle  avoid_the_block  N=5  0 ok  0%  median final 70.7 m  max_steps x5
```

**5/5 → 0/5.** Final distances 70.6–71.3 m across all five seeds: the aircraft flies 41 m,
meets the tower's north face, and stalls there. E-02's stated criterion is met.

It is worth being precise about *how* it fails, because it is not a collision. Grounding
clamps the waypoint to the measured depth, so the oracle never drives into the wall — it
parks in front of it and burns all 40 steps. Zero collisions across the five episodes.

## 7. The rule collision, and why 50 m AGL was still wrong

`.ai/AGENTS.md` says to validate a scenario with the `oracle` backend and to read an oracle
failure as **a broken scenario, not a bad model**. That rule now points the wrong way: the
oracle is a straight-line policy, so on a scenario built to block the straight line it fails
by construction. Reachability needs a different proof.

So I added `--route`: A* on the horizontal plane at flight altitude, cells blocked where any
geometry solid at that altitude comes within a clearance margin. Its first answer was
`UNREACHABLE`, which was a bug in my own search box — an 80 m pad around the leg, inside which
the obstruction spans the full width. At a 150 m pad it resolves, and the answer is the
interesting part:

| altitude | solid on the line | shortest legal route | vs straight |
|---|---|---|---|
| 50 m AGL | 33.9 m | 348.2 m | **3.17x** |
| 90 m AGL | 33.9 m | 342.4 m | 3.11x |
| 110 m AGL | 33.9 m | 244.5 m | 2.22x |
| **120 m AGL** | **33.9 m** | **129.9 m** | **1.18x** |

The tower blocks the line at every one of those altitudes — that is why the 50 m version
passed every check I had written. What differs is *everything else*. At 50 m the neighbouring
high-rises are also solid and close ranks into a near-continuous wall, so the cheapest legal
route is 348 m and the real task is "go around the whole block". The instruction says go
around the tower. The scenario would have been measuring something its own wording does not
describe — a subtler version of the exact fault E-02 was opened to fix.

By 120 m AGL the neighbours have dropped below the flight level and **exactly one building**
is left on the line. Re-sited there:

```
in the way    1 building (ProceduralBuilding_94), 4 pieces
contact       41.8 m in
solid depth   33.9 m
around        14.6 m west / 15.3 m east; A* says 129.9 m vs 110 m, 1.18x
over          needs NED altitude 126.8 — ABOVE the controller's own 120 m clamp
```

That last line is the part I like. The vertical escape is not merely expensive, it is
**outside the envelope the controller will fly**, so it is closed by the same clamp that
caught my first draft — not by luck, and not by anything a model could talk its way past.

## 8. Verification at 120 m AGL

Both backends, the same five seeds as E-01:

```
backend    scenario            N  ok  rate  median final  failure modes
geometric  avoid_the_block     5   0    0%       249.4 m  max_steps x5
oracle     avoid_the_block     5   0    0%        70.6 m  max_steps x5
```

Per-seed oracle finals: 71.0, 70.6, 70.3, 70.0, 70.6 m — **a 1.0 m spread across five seeds**.
The aircraft flies 41 m, meets the tower's north face at y = −273.8, and holds there until the
step budget runs out. It is the same distance every time because it is the same wall every
time; the traffic and weather seeds do not move it.

**Zero collisions in all ten episodes.** Grounding clamps the waypoint to measured depth, so
the oracle never actually drives into the tower — it parks in front of it. Worth stating
plainly because "the obstacle works" and "the aircraft crashes" are different claims and only
the first one is true here.

`geometric` is unchanged in character — it wanders, ending 170–287 m out, and never had a
straight-line policy to break in the first place.

## 9. What this scenario now measures, and what it does not

It measures whether a policy can **route around a thing it can see**. The oracle cannot, by
construction, because it is handed the goal and steers at it. `geometric` cannot, because it
steers at open depth with no notion of the goal.

So `avoid_the_block` currently has **no backend that solves it**, and that is the intended
state: it is the first scenario where a real VLM has room to beat both baselines rather than
tie them. A model that annotates a pixel beside the tower gets a waypoint that goes around,
and `--route` proves the resulting path exists at 1.18x the straight-line distance.

What it does not measure: obstacle avoidance in the reflexive, depth-only sense. Nothing here
requires reacting to something that appears late. The tower is visible from the start pose.

The other three scenarios are still straight-line-solvable — `--check` says so — and are
deliberately left alone, because changing them would invalidate the E-01 baseline that V-01 is
meant to be compared against. Logged as E-02b.

