# 2026-08-07 — the tests that were never written, and what the containers did to the console

Six commits. Two PRs merged (#3 D-05, #4 Q-01), the branch backlog emptied, and a day that
ended somewhere it did not start: a backlog item that had been deferred as tidiness turned out
to be the only route out of a dead end.

**Written at the end of the day, not as it happened** — the same failure the 2026-08-06 log
records about itself. Twice in a row is a pattern, not a slip, and it is recorded here rather
than quietly fixed by backdating. What is lost is the ordering of the dead ends; what survives
is the findings, because those were measured and the measurements are still on disk.

## D-05 — the reset tolerance, tightened

Shipped as `f598823`. `RESET_TOLERANCE_M` 6.0 → 1.5, with `RESET_MIN_IMPROVEMENT_M` 0.3 as a
stall guard and `RESET_ATTEMPTS` 4 as the backstop. The guard is the interesting half: at
street level the aircraft is commanded to 3.5 m AGL, settles at 0.2, and **does not move
however many times it is asked**. Retrying a floor is not converging, and burning four attempts
against one wastes ~11 s per episode to reach the same answer.

So the loop now stops for two different reasons and says which:

- `stalled at 3.3 m out; further attempts are not improving it` — a floor, reported specifically
- `NOT CONVERGED: 15.0 m from the commanded pose` — attempts exhausted while still improving

That distinction is the whole point. A generic failure message would have made D-05 and D-03
look like the same bug.

## Q-01 — the two things that shipped untested

`a5536cf`. Both the D-05 convergence loop and the `pre-commit` hook shipped with no test, and
both were verified by hand in a throwaway repo — twice during review, once after a fix. That is
the slow way, and it is how the hook's deletion gap survived the first review: `--diff-filter=ACMR`
omits `D`, so `git rm scripts/foo.sh` on `main` went straight through.

17 tests, no simulator, no GPU: 6 against a `FakeClient` returning scripted positions, 11
against a temporary git repository.

### The diagnostic was measuring stale bytecode

Mutation testing is only worth running if a mutation actually reaches the interpreter. Several
of mine did not. **CPython invalidates `.pyc` on `(mtime, size)`** — so a mutation that changes
a constant *without changing the file's byte length*, applied within the same second as the
last write, leaves the cached bytecode valid and the test suite runs the **original code**. The
mutations "passed" and I nearly recorded that the tests were weaker than they are.

Fourth self-inflicted diagnostic failure in this project — after `ss` not being able to see a
UDP client, a `/proc` scan SIGTERMing its own shell, and a probe looking in the wrong Vulkan ICD
directory. The pattern is always the same: **the tool was not measuring what I believed it was
measuring**, and nothing errored.

## The PR #4 review found one real thing

`01958ad`. The hook tests build a throwaway repo and set `user.email` / `user.name` in it — but
`git commit` also reads the developer's **global** config. On a machine with
`commit.gpgsign = true` set globally, every commit the fixture makes dies with
`gpg failed to sign the data` and **all eleven tests go red saying nothing about the hook**.

Reproduced by pointing `GIT_CONFIG_GLOBAL` at a config with signing on: red before, green after
adding `commit.gpgsign=false` to the fixture. A test that fails for a reason unrelated to its
subject is worse than no test, because it trains you to ignore it.

Also checked and clean: the fake covers every client method `reset()` calls (so a new call
raises `AttributeError` rather than passing on a stale surface), the `_no_sleeping` monkeypatch
removes only fixed settling waits nothing under test depends on, and `importorskip("airsim")`
was not silently swallowing the file.

## The web console, and a decision that reversed

The day's real finding, and it came from a question about a diagram rather than from a test.

### It cannot reach the containerised stack at all

`webui/server.py` is in no image, is started by no script, and looks for
`/tmp/carla_air_testbed.sock` while the containerised stack serves `/run/carla-air/sim.sock` on
volume `carla-air-run`. Setting `TESTBED_SOCKET` does not fix it: that volume's host mountpoint,
`/var/lib/docker/volumes/carla-air-run/_data`, is **`Permission denied`** to this user under the
rootless daemon. Measured, not assumed.

So the console is not merely on the wrong interface. Since 2026-08-06 it is on **no** interface
to the containerised stack.

### R-03 was three times smaller than its own entry said

The allowlist exposes all 30 sidecar methods to the browser, but the console only *calls*
**14** — nine from the page, five from the server. Mapping those against the ROS surface:

| | |
|---|---|
| already on ROS 2 | **13** — odometry, collision, four `/sim/*` services, setpoints for flight, the RGB topic, the chase-recording service |
| genuine gap | **1** — `chase_jpeg`; the chase camera has no topic |

`examples/ros2_full_control.py` is the proof for the flight half, and a hard one: it flies
takeoff, land, velocity and yaw over `rclpy` + `px4_msgs` while **importing nothing from the
testbed**. So R-03 is not "grow the ROS surface to match the console". It is "make the console
use what already works".

### Why the deferral was reversed

Deferred 2026-08-03 on the grounds that nothing depends on the console. **That is still true**,
and it is not what changed. What changed is the cost of *waiting*:

- Porting to `rclpy` is now the **cheapest** route to a containerised console — an `rclpy` node
  needs no socket, so both blockers above stop existing rather than getting fixed.
- It removes the second AirSim capture, which is the entire reason the console carries a
  do-not-run-during-a-scored-episode warning. One capture fanned out by DDS means a scored run
  becomes watchable live — the use case the console was built for and currently cannot serve.

Recorded as R-03 with a four-step sequence, each step shippable alone, plus the rule that the
port must **not** invent a `/testbed/takeoff` to make itself easy. `ros2_full_control.py:17-20`
refuses one on purpose: moving to hardware must mean deleting `carla_air_bridge`, not rewriting
every client.

And one thing worth writing down before the port rather than discovering during it: **the
carve-out is permanent**. Start and stop can never be ROS calls — there is no graph before the
simulator exists, and the stop button's job is to destroy the graph. The honest claim afterwards
is *"flying is ROS; lifecycle is not"*, never *"the only interface is ROS 2"*.

Two reports written: `docs/webui-architecture.html` (what it is now) and
`docs/webui-ros2-plan.html` (what to do), committed as `ab01f12`. Neither has been rendered —
there is no browser on this box — and both say so.

## Step 1 landed the same day, and the measurement corrected the plan

`webui/ros_source.py` + `scripts/webui.sh`: onboard video and telemetry off the socket and onto
the topics. Measured against a live graph, three drift-controlled A/B pairs each way:

| console | camera rate | cost |
|---|---|---|
| running, not streaming | 6.39 – 6.66 Hz | baseline |
| streaming, socket | 5.060 Hz | −24.0% |
| streaming, ROS | 5.968 Hz | −6.6% |

**Two things I got wrong, in order.**

1. **The plan predicted the rate would be *unchanged*** on ROS — one capture fanned out by DDS,
   costing nothing. It is not: re-encoding to JPEG costs ~6.6% on the machine that is also
   rendering. Corrected in `todo.md` with the old criterion struck through.
2. **Then the review caught the baseline label.** I wrote "console closed". The console process
   was running through *both* conditions; only the HTTP stream opened and closed. On ROS an idle
   console still converts every frame, so its idle cost sits inside the baseline — while an idle
   socket console makes no calls at all. **The comparison is biased toward ROS and −6.6% is a
   floor, not a total.** A true no-console baseline is still unmeasured.

The first attempt at the measurement showed a 10% drop that was **inside the baseline's own
drift** — 6.36 → 6.03 Hz between two consecutive idle runs. Alternating the conditions is what
made the number mean anything. Two overclaims in one day, both caught by looking again rather
than by anything failing.

Also found while measuring: the stream served 12 fps from a 6.4 Hz source, so half the JPEG
encodes were the same frame twice. And `scripts/stop.sh` ignores unrecognised arguments and runs
its destructive default anyway — `stop.sh --help` stops the graph instead of printing help. It
cost a restart mid-measurement; filed, not fixed here.

## Process

- **The commit window held.** Both of this week's violations came from checking the clock in the
  *same* command block as the commit; checking it in a separate call is what fixed it, and it
  worked again today.
- **The branch rule held.** PRs #3 and #4 both went branch → review → merge → delete. Four
  branches cleaned up, remote pruned, no open PRs. That is the first full day since the rule was
  hardened on 2026-08-06 with no violation of it.
- **Doc-only went straight to `main`**, correctly, and the hook allowed it — which is the
  exemption working, not the rule failing.
