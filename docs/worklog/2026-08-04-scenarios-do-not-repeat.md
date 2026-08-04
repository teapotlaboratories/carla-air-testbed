# 2026-08-04 — the same seed does not give the same result

Found while verifying something else: moving `control` and `evaluation` out of `bringup`
(scope change, same day) needed an end-to-end check that flight still worked. It did — and
then it didn't, and then it did.

## The measurement

`cross_the_plaza`, **seed 1**, `oracle` backend, shipped defaults, **nothing changed between
runs**. Parameters read back off the node before the series: `max_speed_mps 5.0`,
`max_accel_mps2 2.5`, `max_yaw_rate_dps 45.0`.

| | steps | path |
|---|---|---|
| FAILURE `max_steps` | 25 | 124.5 m |
| FAILURE `max_steps` | 25 | 132.8 m |
| FAILURE `max_steps` | 25 | 126.3 m |
| SUCCESS 18.9 m | 14 | 65.4 m |
| SUCCESS 19.4 m | 13 | 59.8 m |
| SUCCESS 19.0 m | 13 | 60.9 m |
| FAILURE `max_steps` | 25 | 124.9 m |

**3 of 7.** The scenario is documented at **5/5** for this backend (E-01, reproduced in E-01b),
and the successful runs here match that documentation almost exactly — 13–14 steps, ~19 m
final, against a documented 18.6 m / 14 steps. The successes are not the anomaly.

## What is and is not established

**Not a regression from the day's changes.** That was the first hypothesis and it is wrong.
The identical configuration produced both outcomes, repeatedly, in both orders. Two candidate
causes were tested and cleared individually by A/B:

- velocity slew off (`max_accel_mps2 0`) — still failed, 132.8 m
- yaw slew off (`max_yaw_rate_dps 0`) — still failed, 126.3 m

and the shipped defaults then produced three consecutive successes before failing again.

**The distribution is bimodal, not noisy.** Two tight clusters with nothing between them:
13–14 steps and ~60 m of path, or 25 steps and ~125 m. The failing path is **almost exactly
twice** the succeeding one over an 80 m journey, and 25 is the scenario's `max_steps`, so the
failures are "ran out of budget", not "flew somewhere wrong". No collisions in any run.

**No cause identified.** The doubling invites a guess — a leg flown twice, an overshoot and
return, two consumers of one waypoint — and none of that is evidence. `out/episodes/*.json`
records `steps` as a COUNT, with no per-step trace, so the shape of the path cannot be
recovered from what is stored. That is the same gap that stopped the `bearing_only`
correlation two days ago, and it is now blocking a second investigation.

## Why this matters more than it looks

The scope agreed this morning makes **determinism and repeatability** a core concern of this
repository. A seeded episode that returns different answers on the same seed is a defect in
exactly the thing the project now says it is for.

It also puts a caveat under every number measured this way. E-01b's `oracle 14/19` and the
per-scenario table in `README.md` were single passes at N=5 per scenario. If one seed is 3/7
on repeat, then **5/5 was a sample, not a property**, and the honest reading of the existing
results is that they carry an unmeasured variance. The numbers are not withdrawn — they were
really measured — but they should not be quoted as reproducible until this is understood.

## Next, in order

1. **Record a per-step trace** (E-03, MCAP or simpler). Two investigations have now stalled on
   the same missing data. This should come before any attempt at a fix.
2. **Establish the actual rate** — one seed x N repeats per scenario, not N seeds x 1.
3. Only then look for the cause, with the trace in hand.

Deliberately not attempted today: a fix. There is nothing to fix yet, only something to
measure, and guessing at a cause with no trace is how this project got its one confidently
wrong conclusion.

---

## Update, same day: the trace exists, and it contradicts my hypothesis

Built E-03 as `scripts/record_trace.sh` (an MCAP bag of state + commands, cameras excluded —
RGB at 960x720/8 Hz is ~16 MB/s and would swamp a three-minute episode) and
`scripts/analyse_trace.sh`. Deliberately simulator-side rather than wired into the episode
runner, which now lives in `examples/navigation` and is out of scope.

Ten traced runs of `cross_the_plaza` seed 1, oracle, defaults. **9 succeeded, 1 failed** — so
today's overall rate on this seed is 12/17. The failure was caught with a trace.

### The comparison

| | net | path | ratio | travel against | waypoints |
|---|---|---|---|---|---|
| run8 SUCCESS | 60.6 m | 61.1 m | **1.01** | 0.0 m | 13 (**13** bearing-only) |
| run10 SUCCESS | 61.5 m | 61.9 m | **1.01** | 0.0 m | 12 (**12** bearing-only) |
| run9 FAILURE | 77.2 m | 124.1 m | **1.61** | **15.2 m** | 24 (**15** bearing-only) |

Two things fall out, and the second is the opposite of what I had been saying:

**The failure zigzags.** 15.2 m of travel directly against the net direction, where both
successes measure exactly 0.0. Its waypoints alternate either side of the goal line
(y = -159.4) by 15–20 m a step: -171.9, -156.9, -168.6, -156.3, -168.0, -153.5, -170.0,
-146.8, -172.0, -153.0, -175.2, -136.6, -172.2. The successes go straight in.

**Bearing-only correlates with SUCCESS, not failure.** Both successful runs are **100%**
bearing-only. The failure is 15 of 24 — it has *more* depth-valid waypoints, and its
depth-valid ones are the bad ones: `[351.2, -26.7, 39.0]` is a 220 m jump off the map, and
`[82.0, -142.8, 44.6]` is behind the start and 17 m **below street level** (the origin sits
27.45 m up, so z = +44.6 is underground). Physically impossible points, produced by the branch
that has a real depth reading.

I had been calling `bearing_only` the strongest remaining explanation for the navigation
failures, in the review, in the PR description and twice to the operator. **On this evidence
it is not.** The runs that were 100% bearing-only are the ones that worked.

### What is established, and what is not

Established: the failure is a lateral oscillation, not a longer route; it coincides with
depth-valid waypoints resolving to impossible positions; the successes reproduce the
documentation closely.

Not established: why. A zigzag that alternates sides is what you would get from a heading or
camera-pose lag in the projection, but the yaw slew was already cleared by A/B, and I am not
going to name a cause on one failing trace. **The next step is more failing traces, not a
theory** — one is a coincidence generator.

### Two bugs found by using the tools

- **The recorder orphans.** Four `ros2 bag record` processes survived their episodes in one
  session, still subscribed and still writing. `stop.sh` and `status.sh` now know about them,
  matched on the output path under this repo rather than on `bag record`, which would also
  match a sibling project — and matched this script's own shell when I first checked by hand.
- **The analysis windowed to nothing.** `/episode/status` is latched, so a recording that
  starts between episodes receives the *previous* episode's terminal status first; taking the
  first terminal message gave `hi < lo`, an empty window, and "0 odometry samples" for six
  perfectly good bags. The end is now the first terminal status *after* the start.

---

## The layer split, and where it actually leads

`scripts/analyse_trace.sh --split` prints the annotation pixel beside the waypoint it
produced, so the question "which side of the See-Point-Fly seam is oscillating" is read rather
than argued. Run on the traces already captured:

**Success (run8)** — the pixel settles and stays:

    #   t(s)      pixel u,v    waypoint y      side
    2    1.1     496,    0        -159.4
    5    4.3     480,   63        -159.3
   13   12.9     481,   63        -159.5
    pixel u: median 481.0, side-flips 2     waypoint y: spread 0.5 m

u ≈ 480 is dead centre of a 960-wide frame, and the waypoint sits on the goal line
(y = -159.4) to within half a metre for the whole flight.

**Failure (run9)** — the pixel slams between the frame edges:

    5    4.4     272,    0        -168.6  right
    6    5.4     685,    0        -156.3  LEFT
   14   14.1     959,    0        -136.6  LEFT
   17   17.3       0,    0        -189.8  right
    pixel u: median 715.0, spread 959 px, side-flips 11   waypoint y: spread 182.3 m

0 and 959 are exactly the bounds of a 960 px frame, so the annotation is being **clamped to
the border** — the thing it wants to point at is outside the field of view, and it alternates
which edge it gives up on.

**So the projection is not the problem.** Grounding faithfully converts a swinging pixel into
a swinging waypoint. The oscillation is upstream, in the annotation, which is out of scope.

### But the reason the goal leaves the frame is in scope

Yaw at the moment each episode starts, and over the following seconds:

| trace | | yaw at start | then |
|---|---|---|---|
| run5 | SUCCESS | 20.9° | 1.3, 3.5, 1.9, 2.9 |
| run6 | SUCCESS | 20.9° | 1.8, -0.1, 0.0, 0.1 |
| run8 | SUCCESS | 25.7° | -3.9, -0.7, -0.3, -0.2 |
| run10 | SUCCESS | 27.8° | 2.3, 0.5, 0.2, 0.3 |
| **run9** | **FAILURE** | 17.6° | **8.5, -19.9, 18.3** |

Two things, and both matter:

1. **The failing run's heading oscillates ±20° and never converges.** Every successful run
   settles to within a degree or two of zero inside two seconds. That is what pushes the goal
   out of frame, which is what makes the annotation clamp.
2. **The starting heading is not repeatable at all.** 17.6°, 20.9°, 20.9°, 25.7°, 27.8° across
   runs that are supposed to be identical — same scenario, same seed, same everything. And
   `Vehicle.reset()` explicitly commands `airsim.Quaternionr(0, 0, 0, 1)`, i.e. **yaw zero**.
   The aircraft is not where the reset says it put it.

(2) is a **simulator repeatability defect and squarely in scope**: a seeded run that starts
from a different attitude every time is not seeded. Filed as **D-02**.

### What is NOT established

That (2) causes (1). A heading loop through the navigation stack — waypoint sets velocity,
velocity sets yaw, yaw aims the camera, camera picks the pixel, pixel makes the waypoint — can
oscillate from any perturbation, and a 10° difference in initial attitude is only one
candidate perturbation. Proving it needs runs where the starting attitude is actually fixed,
which is what D-02 is for. Recording the correlation, not a mechanism.

---

## D-02 measured: nothing holds the heading after a reset

`tests/conformance/p11_reset_attitude.py`, 10 iterations against a bare simulator — no ROS
graph, so nothing else is touching the aircraft.

**Established, and repeatable:** the airframe rotates a median of **98.6°** (worst 105.0°)
during the 2 s settle at the end of `reset()`. Every iteration, same direction, same
magnitude. Nothing is holding heading once `moveToPositionAsync` returns, so the attitude the
aircraft presents to the camera when the episode starts is whatever it happened to rotate to.

That is a sufficient explanation for the trace observation — episode-start yaw of 17.6°, 20.9°,
20.9°, 25.7°, 27.8° across five identical runs — without needing anything else to be wrong.
In an episode the offboard controller then pulls the heading back, which is why four of those
five settle to ~0° within two seconds; the aircraft simply starts each episode pointing
somewhere different.

**Not established: which stage loses it.** The probe's own attribution line says the teleport,
reporting ~-115° after `simSetVehiclePose`. But the identical call issued once from an idle
aircraft was measured delivering **exactly 0.00°** and holding it for two seconds:

    +0.00s after simSetVehiclePose: yaw -177.60      <- the STALE pose, not yet applied
    +0.05s                          yaw    0.00
    +2.00s                          yaw    0.00

So the teleport demonstrably can work. Something about the preceding state decides whether it
does, and swapping the call order between two variants inverted the result — which is the
signature of an experiment measuring itself. The probe's stage attribution is marked untrusted
in its own docstring rather than quietly relied on.

**Two dead ends worth recording**, both mine:

- I first blamed `vehicle_name`: the probe omitted it while `reset()` passes `"SimpleFlight"`.
  Testing both showed the empty name working and the explicit one failing — the opposite of
  the hypothesis, and on re-running, order-dependent rather than name-dependent. Not the cause.
- Before that I had assumed the traces' 17-28° spread meant the teleport was inaccurate. The
  drift measurement says the teleport may be fine and the aircraft simply rotates afterwards,
  which is a different bug with a different fix.

**Next:** find what holds heading through the settle. `moveToPositionAsync`'s default
`YawMode` is rate control at 0 deg/s, which asks for no active heading hold at all — passing
an explicit `YawMode(is_rate=False, yaw_or_rate=0)` is the first thing to try. That is a
one-line change to `Vehicle.reset()` and it is testable with this probe as it stands, since
the drift metric is the part that measures cleanly.

---

## The YawMode fix: D-02 solved, D-01 untouched

`moveToPositionAsync`'s default is `YawMode(is_rate=True, yaw_or_rate=0.0)`. That reads as
"hold zero" and means the opposite — **rate** control commanding zero rate, i.e. do not
actively drive yaw at all. The airframe keeps whatever angular momentum it arrives with.

A/B over 10 resets each, same simulator, nothing else running:

| | worst drift | median drift |
|---|---|---|
| AirSim default | **65.2°** | 0.5° |
| explicit `YawMode(is_rate=False, yaw_or_rate=0.0)` | **0.9°** | 0.3° |

Applied to `Vehicle.reset()`. **D-02 is fixed** — and note the median: most resets were always
fine, and it is the occasional 65–105° one that did the damage. That intermittency is why it
survived this long, and it matched D-01's intermittency closely enough to look causal.

### It is not causal

`cross_the_plaza` seed 1, oracle, ten runs with the fix in place: **8/10**, against 12/17
before. P(≥8 of 10 | true rate still 0.706) = **0.40**. The sample cannot tell those apart, and
nothing suggests an improvement.

**So D-01 is still open and the heading hypothesis is refuted.** The two failures had the same
signature as before — `max_steps` at 25, ~40 m short — so whatever oscillates is still
oscillating with the starting attitude nailed down.

That is the fourth hypothesis this session that did not survive contact with a measurement:
`bearing_only`, projection lag, teleport inaccuracy, and now reset attitude. Each was
plausible, each was tested, each was wrong. The one thing that has held up throughout is the
observation itself — the failure is an annotation oscillating against the frame edges, and the
loop it sits in (waypoint → velocity → yaw → camera → pixel → waypoint) is closed.

**The fix stays regardless.** A reset that does not deliver the attitude it commands is a
defect on its own terms, in scope, and now measured and closed. It simply is not D-01's cause.

### What I would do next, and what I would not

Not: another single-cause hypothesis. Four in a row have failed, and the pattern suggests the
oscillation is a property of the closed loop rather than of any one component — which would
also explain why disabling the velocity slew, the yaw slew, and now the attitude perturbation
each changed nothing.

Instead: capture traces for several failures and check whether the oscillation ever starts
from a *quiet* state, or only ever from a large initial heading error. If it can start from
quiet, it is self-sustaining and the loop gain is the thing to look at. That is a
navigation-stack property and out of scope for this repository — which would make the right
outcome for D-01 "documented, bounded, and not this repository's to fix", with the simulator's
own contribution (D-02) already closed.
