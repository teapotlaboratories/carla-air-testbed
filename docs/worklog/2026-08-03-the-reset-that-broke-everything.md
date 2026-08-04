# 2026-08-03 — One AirSim call was breaking everything

Backlog [E-06](../todo.md), and the re-baseline [E-01b](../todo.md) it unblocked.

The 40-episode sweep had not run end to end since 2026-08-01. Every attempt died partway
with the sidecar gone and `[Errno 32] Broken pipe` on the ROS side. This is how it was found,
including the three wrong hypotheses.

---

## 1. Three wrong guesses, each cheap to disprove

The failure looked like load, so the first hypotheses were the two heaviest things added
recently. Both were wrong, and testing them one at a time is what made that clear:

| run | change | result |
|---|---|---|
| A | baseline, 5 seeds | died at seed 2 |
| C | `--no-chase` (no H.264 recording) | died at seed 2 — **worse**, so not recording |
| D | semantic lidar disabled | died at seed 2 — not that either |

Contention was the third guess: the operator's GPU 0 workload was at 88% during the first
sweep. Also wrong — the simulator renders on GPU 1 and its VRAM was never the issue.

## 2. The minimal reproduction is what mattered

Run E: **six resets, nothing else running.** No VLM, no episode, no offboard target — just
`reset` repeated with only the bridge's own polling alongside.

    reset 1:  26.8 s   ok
    reset 2:  60.1 s   failed
    reset 3:  HUNG

That killed the "it's the sweep" framing entirely. The sweep was never the problem; it was
just the first thing to call `reset` forty times.

## 3. Four real bugs, none of them the cause

Each was found while chasing this, each was independently worth fixing, and **none of them
fixed it**:

- **`reset` raced telemetry on one msgpack-rpc connection.** It drove `self.vehicle` (the
  telemetry client) under `slow_lock` while FAST `state` drove the same socket under
  `fast_lock`. The FIFTH instance of *lock classes guard dispatch, not sockets*, so the rule
  became a test — `tests/test_sidecar_locks.py` parses `server.py` and asserts every method
  dispatches under the lock owning the client it touches. It immediately found **two more**
  latent instances (`describe`, `ground`).
- **Timer callbacks guarded the RPC and then indexed the reply outside the guard.** One
  error-shaped dict raised `KeyError: 'position'`; rclpy does not catch callback exceptions,
  so `bridge_node` exited(1) mid-run.
- **A 60 s socket timeout with no resync.** When a caller gave up, the reply still arrived
  and sat unread, so every later call on that connection read the PREVIOUS call's answer.
  Permanently. That is what produced the `KeyError`, the broken pipes, and the cascade across
  all four world services. Fixed with a reader thread and id correlation
  (`tests/test_rpc_correlation.py`, 5 tests; the old design **hangs** against them).
- **`destroy_all` was fire-and-forget**, so a following `spawn_traffic` attached walker
  controllers to half-reaped walkers and CARLA threw an uncaught C++ exception.

## 4. The progress instrumentation is what named the culprit

The async work added progress frames so a caller could tell slow from dead. That is what
found it. `reset` announces `"sim-reset"`, calls `client.reset()`, then announces
`"placing"`. On the second reset the first frame arrived at t=0 and **nothing followed for
60 s**.

Execution was inside one AirSim call. Not the flight back, not CARLA, not the socket layer.

## 5. The fix: stop calling it

`simSetVehiclePose` already did the positioning. `client.reset()` remained only for three
side effects, and each had a cheaper equivalent:

| side effect | replacement |
|---|---|
| cancel in-flight commands | `cancelLastTask()` |
| drop API control | `armDisarm(False)` + `enableApiControl(False)` |
| clear the latched collision flag | **a collision epoch** |

The collision one is the interesting change. AirSim latches `has_collided` until a full sim
reset — which made a minute-long call load-bearing **for scoring**. Snapshotting `time_stamp`
at reset and reporting only newer collisions removes the dependency, and is better anyway: an
explicit per-vehicle epoch beats a side effect of a global operation.

Take the epoch AFTER settling. A hard reset restarts sim time, so an epoch captured before it
would sit in the future and mask every real collision that followed.

    E  fly-in, polling live      26.8 -> 60.1 -> hung        0/6, sidecar died
    F  polling suspended         60, 59, 30, 27, 32, 27 s    4/6
    G  teleport                  5.5, 5.6, 60, 60, fail      2/6, sidecar died
    H  no client.reset()         2.8 2.9 2.8 2.8 2.9 2.9 s   6/6, zero deaths

Verified end to end: clean after reset, flew the aircraft into the ground and it was detected
as `Road_Road_Town10HD19`, then a soft reset cleared it — no `client.reset()` anywhere.

## 6. The re-baseline, and what it does not say

40/40 episodes, zero sidecar deaths, zero collisions. geometric **0/20**, exactly as before.
oracle **14/19** against 15/20.

The headline reproduces: decoupling the VLM did not change flight behaviour.

**But these numbers supersede 2026-08-01 rather than confirming it.** Start-pose error fell
from ~9 m to ~3 m, because the aircraft is placed rather than flown in and settled. Two
anomalies are recorded rather than smoothed: `avoid_the_block` produced 4 results not 5 (so
the denominator is 19, and the missing seed wants explaining), and `rain_descent` went
5/5 -> 4/5 with `model_declared_done` — most likely the tighter starts showing up first in
the scenario with the smallest margin.

## 7. What this cost, and the lesson

Most of a day, and three plausible hypotheses that were all wrong. What eventually worked was
the boring thing: **strip everything away until one call is left, then instrument that call
rather than reasoning about it.** The four bugs found on the way were real and are all fixed,
but none of them were the answer — and each one felt like it might be, which is exactly what
made the detour expensive.

Everything stopped afterwards and independently confirmed: all ten process counts 0, GPU 1
back to 33 MiB.
