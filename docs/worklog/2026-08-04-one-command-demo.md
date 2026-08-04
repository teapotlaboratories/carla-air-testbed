# 2026-08-04 — one command from nothing to a video

Goal: collapse the five-command, three-terminal sequence that produces a demo video into
something runnable without a checklist. Filed as T-04 in `docs/todo.md` — **after** the script
was written, which is the wrong order and is noted there rather than backdated.

## What the sequence was

```
./scripts/bringup.sh --config configs/testbed.yaml       # terminal 1
./examples/vlm_navigation/run.sh --backend claude        # terminal 2
./scripts/run_episode.sh --scenario street_level --seeds 5   # terminal 3
./.venv/bin/python scripts/combine_views.py <chase> <onboard> <out> <depth>
./scripts/stop.sh --all
```

The fifth is the one that gets skipped, and skipping it is the failure rule 1 was written
about: a leftover graph stacks on the next bringup, two controllers fight over the aircraft,
and `ros2 node list` still looks correct.

## Decisions

- **Tear down from a trap, not from the last line.** `trap cleanup EXIT INT TERM`, so a
  Ctrl-C mid-flight still stops the simulator. A cleanup that only runs on the happy path
  would have left the exact state rule 1 forbids, in the exact case people hit it.
- **Wait on the rendering line, not on a timer.** The loop blocks until
  `hardware rendering confirmed` appears in the bringup log and then prints it. A fixed sleep
  would happily hand back a lavapipe run, which is the trap rule 5 exists for — 9x slower with
  no error anywhere.
- **Set no `ROS_DOMAIN_ID`.** `bringup.sh:77`, `examples/vlm_navigation/run.sh:20` and
  `scripts/run_episode.sh:21` each export `${TESTBED_ROS_DOMAIN_ID:-42}` already. A second
  place to set it is a second place for it to be wrong, and getting it wrong merges this
  project's PX4-shaped topics with the sibling project's real ones.
- **Parse the episode id out of the run output** rather than globbing for the newest file in
  `out/chase/`. A stale file from a previous run is newer than nothing, and globbing would
  composite the wrong flight without saying so.
- **Depth is optional.** Inset only when `graph.recorder.record_depth` is on *and* the file is
  non-empty, so turning depth off does not fail the combine step.

## Run, 2026-08-04

`./scripts/demo.sh --scenario street_level --backend claude --seed 5`

```
GPU 1 (NVIDIA GeForce RTX 5060 Ti): 3342 MiB in use — hardware rendering confirmed
chase: 703 frames (35s), 0 dropped
-> FAILURE (model_declared_done)  106.5 m from goal, 8 steps
real rates: chase 20.03 fps (file 20), onboard 7.70 fps (file 8)
playback scale: chase x0.998, onboard x1.040
video: out/demo/street_level-s5-a18931.mp4   1920x1080, 703 frames, 35.1s, 16.3 MB
stopped: graph and sidecar, simulator stopped (0 stragglers)
```

`status.sh` afterwards: every count 0, GPU 1 back to 33 MiB, no sim ports.

**The 0/1 is not a navigation result.** One seed, and it matches the street-level failures
already on record. It says the wrapper works and nothing about the model; a behaviour claim
needs a rate over N seeds.

## Defect found while auditing this against `.ai/AGENTS.md`

The script ran `stop.sh --all` but not `status.sh`, so it implemented **half** of rule 1 — the
half that reports what was *signalled*, not the half that reports what is actually *left*.
Those disagree exactly when something ignores TERM, which is the case worth catching. Fixed:
cleanup now runs `status.sh`, greps for any non-zero count, and prints a warning naming the
survivors instead of a clean-looking summary. Verified both branches — the current clean
output does not trigger it, a synthetic `vlm example 2` line does.

## Two more bugs, found by actually interrupting a run

The `status.sh` fix above was written but not *executed* — so it got tested the way rule 6
asks: start a real run, SIGINT it mid-flight, watch what happens. Both of these were invisible
until then.

**1. The interrupt tore down and then kept going.** `trap cleanup EXIT INT TERM` runs cleanup
on SIGINT and *returns to the next line*. Observed: the stack came down, `verified clean`
printed, and the script walked straight into step 3 and tried to fly an episode against a
simulator that no longer existed — failing with `no episode id in the run output` and running
the whole teardown a second time on EXIT.

Fixed by splitting the traps: `EXIT` does the work, `INT`/`TERM` only print and `exit 130`,
which then reaches EXIT. Cleanup is additionally guarded by a `CLEANED` flag so it cannot run
twice by any other path. Re-tested: exit status 130, `stopping everything` appears exactly
once, step 3 never starts.

**2. `status.sh` was checking the wrong GPU.** While the simulator sat at 3848 MiB on GPU 1 —
correct, hardware-rendered — `status.sh` printed:

```
WARNING: simulator running but GPU 0 under 1 GB — probably SOFTWARE rendering.
```

`scripts/status.sh:59` queried `-i 0` unconditionally, while the shipped config is
`simulator.gpu: 1`. This is precisely the mistake `run_sim.sh:312` documents next to its own
copy of the check: *"Checking a FIXED index here would be wrong as soon as TESTBED_GPU points
elsewhere."* `run_sim.sh` got it right via `TESTBED_GPU_INDEX`; `status.sh` was never updated
to match.

Wrong in **both** directions, and the second is the dangerous one:

- On this machine it fired on every correct run. A warning that cries wolf is one nobody reads.
- Reverse the GPUs — sim on 0, operator's UnrealEditor on 1 — and it stays *silent* through a
  genuine lavapipe run, because GPU 0 would be busy. That is exactly the failure rule 5 exists
  for, the one that went undetected through an entire build.

Fixed: resolve `TESTBED_GPU`, else `simulator.gpu` from the config, else 0; skip the check for
a raw `vendor:device` selector, which has no index to query. Verified with the simulator up on
GPU 1 — no warning, and the per-GPU table still prints.

## Still open

- **Chase/onboard sync** (T-02) is unresolved and the composited output inherits it.
  `combine_views.py` rescales from the `.timing.json` sidecars — measured 0.00 s offset on this
  run — but four earlier attempts failed and this path has not been shown to hold across runs.
- The wrapper does not lower `min_altitude_m`; it relies on the per-scenario `control:`
  overrides in `ros2_ws/src/evaluation/scenarios/default.yaml`. A scenario without them flies
  at the global floor.

## Review of the branch, and two fixes (2026-08-04)

`/review` over the 13 commits found five issues. Two were worth fixing before merge.

**1. Per-scenario control overrides leaked on any early failure.** `run_episode.py` applied
them at line 206; the restore sat in the `finally` of a `try` that did not open until line
293. Between them: `raise SystemExit` on a failed reset, `return None` on a refused episode,
and three RPC calls that can raise on timeout. Any of those returned with the overrides still
set on `/offboard_control`, which outlives the process — and `run_sweep.sh` loops scenarios
inside one bringup, so the next scenario inherited them. Including `min_altitude_m: -24.0`,
the floor whose entire job is stopping a waypoint flying the aircraft into the ground.

Everything from the reset onward is now inside the try that restores. Checked structurally
with an AST walk: of the raise/return sites after the apply, none remain outside the guard. A
failed restore now prints a warning rather than leaving the next episode quietly unmeasurable.

**2. The heading gate was unreachable at street level.** `offboard_node.py` gated yaw on
`abs(v[0]) + abs(v[1]) > 0.5`. Two things wrong at once:

- The L1 sum varies by sqrt(2) with heading for one speed, so the gate was tighter for
  axis-aligned travel than diagonal — whether the aircraft would turn depended on which way
  it was already pointing.
- 0.5 was absolute, and `street_level` overrides `max_speed_mps` to 0.5. Due north — what its
  instruction literally asks for — gives exactly 0.500, which is not `> 0.5`. Any descent put
  it lower. **The aircraft flew the street without ever turning to face along it**, with the
  camera the model is scored on pointing wherever it was last left.

Now `hypot(vx, vy) > max(0.02, 0.1 * max_speed_mps)`. At the shipped 5 m/s that is 0.5 m/s —
the strict end of what the old test did, so benchmark behaviour is unchanged; at street level
it is 0.05 m/s. The arithmetic moved to `control/limits.py`, which imports no ROS, so
`tests/test_control_limits.py` can check it against every shipped scenario without a
simulator. 15 tests; confirmed they FAIL against the old gate before being trusted.

### Verified in flight, not just in the suite

Graph up on GPU 1 at 3264 MiB, `street_level` seed 5, geometric backend.

- **Overrides restored.** Read back after the episode with the node still alive:
  `max_speed_mps 5.0`, `max_yaw_rate_dps 45.0`, `min_altitude_m 15.0`, `max_step_m 20.0` —
  all four back to global, having been 0.5 / 5.625 / -24.0 / 6.0 during.
- **Heading is now commanded.** 1440 samples off `/fmu/in/trajectory_setpoint`, **835 distinct
  headings**. Under the old gate, due-north travel at 0.5 m/s produced none at all.

### Noted, not fixed

Two of 1439 ticks show a heading step far above any configured cap (largest 119.4 deg in one
tick). That is consistent with the documented `self._last_yaw = None` reset on hold and on
arrival, which restarts the ramp and permits one uncapped snap per waypoint — intended per the
comment, but it means the yaw cap is not enforced across arrivals, and this scenario has 50 of
them. **Not confirmed**: `ros2 topic echo` carries no timestamps, so episode-window samples
cannot be separated from the pre- and post-episode ones taken under the global 45 deg/s cap,
and the remaining 881 apparent exceedances of the street_level cap are most likely that. Worth
a measurement that logs time alongside yaw before anything is concluded.

The episode itself failed with a collision at 50 steps, against `model_declared_done` at 8 in
the earlier claude run. Different backends, so the two are not comparable and neither is a
result.

## The remaining three review findings (2026-08-04)

**3. A stale reader could condemn the next connection.** `client.py` `connect()` cleared
`_closing` before starting the new reader, so the ordering `close()` -> `connect()` -> old
reader finally wakes from its socket error left the old reader seeing `_closing == False`,
concluding the connection had died, and calling `_fail_all()` on a fresh, healthy connection
belonging to somebody else. Guarded with a generation number: `_read_loop` carries the
generation it was started for, and `_fail_all` ignores a verdict from any generation that is
no longer current.

Latent, not live — `bridge_node` connects each of its four clients exactly once and nothing
reconnects, which is the only reason this never fired. Two tests in
`tests/test_rpc_correlation.py` now cover it, and the second one matters as much as the first:
the guard must not stop the *current* reader reporting a genuine death. Confirmed against the
old behaviour before being trusted — a stale wake-up set `_dead` and left `connected=False`.

**4. `chase_start` treated a legitimate 0 as "unset".** `above = float(above) or
float(d["above"])` meant a caller asking for a chase camera level with the aircraft silently
got the config's 6.0 m instead, with nothing logged. Now `None` is the sentinel for
`distance`/`above`, which a caller cannot legitimately mean. `width`/`height`/`fps` keep 0,
where it genuinely cannot collide with a real value and where `ChaseRecording.srv` already
documents it that way. Reachable only from a direct RPC caller — the ROS service does not
expose `distance` or `above` at all.

**5. A typo'd override key was skipped in silence.** The scenario read as configured and flew
at the global limits, reported as though it were the intended flight. Both failure paths now
warn and name what the episode will actually fly at: an unknown or non-numeric parameter, and
one the node rejects.

**Unexercised:** finding 5's warning is a print on a path that only a malformed scenario
reaches, and it has no test — extracting it into something testable without ROS would mean a
new module for three lines, which is more surgery than the finding is worth. The code parses
and every name in it is in scope; that is all that has been checked.

Findings 3 and 4 need no flight: neither is on the control path, both are covered offline, and
the packages rebuild and import cleanly under ROS. Suite: 156 passed, 1 skipped.
