# First real sweep — 40 seeded episodes, and the numbers finally have an N

**Date:** 2026-08-01
**Backlog item:** E-01
**Ran on:** RTX 5060 Ti (GPU 1), pinned with `TESTBED_GPU=1`, leaving the 3080 to other work.

Every result published before this was **one run per scenario per backend**. The README said
so, but that meant the repository claimed nothing measurable. This is the same grid with
5 seeds a cell.

---

## Result

| backend | scenario | N | success | rate | median final distance | failure modes |
|---|---|---|---|---|---|---|
| oracle | `cross_the_plaza` | 5 | 5 | **100%** | 18.2 m | — |
| oracle | `follow_the_avenue` | 5 | 5 | **100%** | 18.7 m | — |
| oracle | `rain_descent` | 5 | 5 | **100%** | 14.1 m | — |
| oracle | `avoid_the_block` | 5 | 5 | **100%** | 17.9 m | — |
| **oracle** | **all** | **20** | **20** | **100%** | 18.1 m | — |
| geometric | `cross_the_plaza` | 5 | 0 | 0% | 185.7 m | `max_steps` ×5 |
| geometric | `follow_the_avenue` | 5 | 0 | 0% | 187.0 m | `max_steps` ×5 |
| geometric | `rain_descent` | 5 | 0 | 0% | 81.4 m | `max_steps` ×5 |
| geometric | `avoid_the_block` | 5 | 0 | 0% | 175.4 m | `max_steps` ×5 |
| **geometric** | **all** | **20** | **0** | **0%** | 178.9 m | `max_steps` ×20 |

**Zero collisions in 40 episodes**, both backends.

## What holds up

**The harness is repeatable.** Twenty oracle episodes across four scenarios and five seeds
land between **13.7 m and 19.8 m** of the goal — a 6 m spread, entirely inside the 20 m
success radius, with a median of 18.1 m. That tightness is the useful part: it says the
~4 m post-setpoint relaxation and the seeded traffic do not push episodes over the line, so a
future backend's failures will be the backend's.

**Two code paths that had run once now have forty runs.** Bearing-only grounding and the
three-way AirSim client split both went in late in the previous session on a single episode
each. Neither produced a stall, a runaway or a dropped waypoint across the sweep.

**The floor is real.** A depth-following heuristic with no language understanding reaches
none of the goals, and always in the same way — `max_steps` in every one of twenty runs,
never a collision, never a timeout. It does not crash into things; it wanders. Median 28
steps against the oracle's 12.

## What the numbers do not say

**`rain_descent` is visibly easier for the baseline.** Its geometric median final distance is
**81.4 m** against 175–187 m everywhere else. Descending toward open ground is close to what
"steer at the most open depth column" already does, so the baseline drifts roughly the right
way without understanding the instruction. Worth remembering when a real model is scored:
that scenario has the smallest gap to close.

**None of the four scenarios require obstacle avoidance** (`E-02`). The oracle solves all of
them on essentially straight lines — `avoid_the_block` in a median 8 steps despite its name.
A 100% oracle rate here proves the harness reaches goals; it does not prove the scenarios are
hard.

**0% is a floor, not a calibrated baseline.** `geometric` ignores the instruction almost
entirely. The interesting comparison for a real VLM is not 0% but whether it beats the
oracle's *path*, and no scenario yet distinguishes "understood the instruction" from "flew
toward the goal".

## Method

```
TESTBED_GPU=1 ./scripts/run_sweep.sh          # 40 episodes, ~75 min wall clock
```

Sweeps run in real time; `ClockSpeed` cannot compress them because it accelerates AirSim
while CARLA stays at 1x and the two halves of the world desync.

`scripts/run_sweep.sh` restarts the ROS graph between backends (the backend is a launch
parameter), waits on `episode runner ready` rather than on the simulator alone, and collates
only episodes written **after the sweep started** — `out/episodes/` accumulates across runs,
and globbing all of it would have folded the earlier single-seed results into these rates.

Everything was stopped on exit, per the project rule.
