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
