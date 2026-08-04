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
