# Oracle validation, and the bearing-only fix it forced

**Date:** 2026-08-01
**Goal:** establish whether the four scenarios are navigable at all, so that any backend's
score becomes a measurement of the backend rather than of the harness.

---

## Result

| scenario | oracle | geometric |
|---|---|---|
| `cross_the_plaza` | **SUCCESS** — 18.6 m, 14 steps | FAILURE — 41.3 m, 25 steps |
| `follow_the_avenue` | **SUCCESS** — 19.6 m, 16 steps | FAILURE — 188.0 m, 30 steps |
| `rain_descent` | **SUCCESS** — 13.4 m, 8 steps | FAILURE — 71.8 m, 20 steps |
| `avoid_the_block` | **SUCCESS** — 18.2 m, 9 steps | FAILURE — 173.6 m, 30 steps |
| | **4/4** | **0/4** |

All four scenarios are navigable, and the harness reaches goals when something points at
them. A depth-following heuristic with no language understanding reaches none. That gap is
the space a real VLM has to fill, and it is wide enough to measure in.

Every failure was `max_steps`, not collision or timeout — the baseline does not crash, it
simply wanders. Both single-seed; these are ceiling/floor markers, not success rates.

## The bug the oracle found on its first run

The first oracle run **failed on a scenario that is provably navigable**, which is exactly
the situation the oracle exists to expose — except it exposed a defect in my own grounding
node rather than in the scenario.

Symptom: the aircraft flew a short way, stopped at `[119.5, -187.6, -49.9]` and never moved
again. The logs gave it away immediately:

```
controller targets issued:  1
invalid waypoints:          70
reason: "pixel is sky — no finite depth"
rationale: "goal at 74 m projects to (639,0), depth 6 m [off-screen, clamped]"
```

The goal sits at the **same altitude** as the aircraft, so the ray to it is horizontal and
lands on the horizon. Grounding rejected every such pixel for having no finite depth. That
produced a deadlock with no error anywhere in it:

> no waypoint → the aircraft stops → the camera never turns → the goal stays off-screen →
> every annotation is sky → no waypoint.

**A pixel with no depth is not useless — it still carries a bearing.** Discarding it means
the system can never turn toward anything distant, which is fatal for a navigation stack.
See-Point-Fly takes a bounded step along the bearing and re-observes; now so does this one.

### What changed

* `GroundedWaypoint` gained `bool bearing_only`. `valid=false` now means "nothing
  actionable at all"; sky is actionable.
* `grounding` places the waypoint at `bearing_only_range_m` (40 m) along the ray when depth
  is missing or past `max_range_m`, sets `bearing_only`, zeroes `depth`, and says so in
  `reason`.
* `control` skips `standoff_m` for bearing-only waypoints — there is no surface to stand off
  from, and at close range the standoff could cancel the step entirely and re-stall the loop.

Same scenario, same seed, after the fix: **SUCCESS, 18.6 m, 14 steps.**

## Caveat worth acting on

`avoid_the_block` — "fly north but go around the tall building, do not fly through it" —
was solved by the oracle in **9 steps on a near-straight line**. Nothing was in the way. The
scenario does not test what its name claims, so it currently measures the same thing as
`follow_the_avenue`. It needs its start and goal re-sited so a building genuinely blocks the
straight path, otherwise a model that ignores the instruction scores identically to one that
follows it.

More broadly, the oracle succeeding everywhere means **none of these scenarios currently
require obstacle avoidance**. That is fine for a first benchmark of goal-directed navigation
and a real limitation to state before reporting any VLM's number against them.
