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

## T-05 — the teardown script obeyed an argument it did not understand

`./scripts/stop.sh --help` **tore down the ROS graph** instead of printing help. Found the hard
way, mid-measurement: I wanted to stop only the console, guessed at a `--webui` flag, and it
stopped the graph. The script tested `"${1:-}" = "--all"` in three separate places and had no
other argument handling, so anything unrecognised fell through to the default teardown.

Three defects, one shape — the kill escalation ran *before* the arguments were read; `--all`
only worked in position 1; and `ALL=1` was honoured by the container teardown and nothing else,
so it removed the container, left the simulator running, and reported "simulator left running".

**And my first fix was worse than the bug.** I "unified" the flag by seeding it from
`${ALL:-0}` so the old environment path kept working everywhere. That turns an unrelated
`export ALL=1` in some parent shell into "also SIGKILL the simulator", and `ALL` is about as
generic a variable name as exists. Destructive scope must come from an explicit flag on the
command line, never from ambient state. Caught by reviewing my own diff before merging — the
third overclaim-or-hazard in two days found by looking again rather than by anything failing.

13 tests, and they were **mutation-checked**: all fail against the pre-fix script, all pass
after, and the newest one fails against the `${ALL:-0}` draft specifically. Safe to run beside
a live simulator by construction — the script is copied to a temp directory so its
`PROJ`-anchored patterns match nothing real, and only the paths that exit *during parsing* run.
No test passes `--all`; it would `pkill` a real `CarlaUE4`.

## Step 2 — the buttons, and a guard with a hole in it

`webui/ros_control.py`: every console button onto the ROS surface that already existed —
`TrajectorySetpoint` and `VehicleCommand` for flight, the four `/sim/*` services for the world.
**Nothing was added to the ROS surface**, which was the constraint that mattered. Flown against
a live graph: takeoff −30 NED landed at −30.41, four velocity nudges moved it 6.3 m north, yaw
90 reached +82°, `set_weather` round-tripped.

The design decision was contention. Two publishers on `/fmu/in/trajectory_setpoint` produce
**no error at all** — `examples/ros2_full_control.py` measured a takeoff to NED 35 m arriving at
15.6 m while the aircraft flew where the autonomy loop wanted. The console now **refuses** (HTTP
409, naming how many nodes it declines to fight) rather than seizing control the way that
example does. After step 1 its job is watching a run. Measured both directions: `contested`
0 → 1, flight 409, `set_weather` still 200, then 1 → 0 and flight allowed again once the rival
exited — a guard that never opens is as broken as one that never closes.

### The guard had a hole, and the test documented it as a decision

Review caught that **`reset` bypassed it**. `reset` is a `/sim/*` service, so it sorted with
world control and the guard — applied to `FLIGHT_METHODS` — skipped it. But
`/sim/reset_vehicle` takes a `hold_ned` and **flies the aircraft there**, then runs the D-03
convergence loop for several seconds. Relocating an aircraft out from under a running controller
is strictly worse than any velocity nudge the guard did refuse.

*"Flight command"* and *"moves the aircraft"* are different sets. That is the whole lesson, and
it is a classification error rather than an implementation one — both of this PR's findings
were.

**The test that should have caught it encoded the omission instead.** It asserted only that
`set_weather` was excluded, which reads as "the exclusions were considered" when exactly one of
them had been. There is now a table naming why each unguarded method is safe, so adding one
without deciding fails rather than passing quietly.

### Semantics preserved rather than improved

Moving a button must not change what it does, so: `yaw` stays **absolute** (the sidecar calls
`rotateToYawAsync`, so ↺/↻ command *heading 30°*, not *turn by 30°* — odd, but a separate
decision); `hold` stays a literal zero-velocity command; and the velocity lifetime stays the
bridge's, because `TrajectorySetpoint` has no duration field and inventing one is what the
no-friendly-services rule forbids.

## Three reviews, three real findings

Worth stating plainly at the end of the day. Every `/review` this week found something that
mattered, and **twice the defect was in code I had just written to prevent that class of
defect** — the `${ALL:-0}` seeding that made a teardown escalate, and the guard that let `reset`
through. Nothing failed to make either visible; both took a second look.

## Process

- **The commit window held.** Both of this week's violations came from checking the clock in the
  *same* command block as the commit; checking it in a separate call is what fixed it, and it
  worked again today.
- **The branch rule held.** PRs #3 and #4 both went branch → review → merge → delete. Four
  branches cleaned up, remote pruned, no open PRs. That is the first full day since the rule was
  hardened on 2026-08-06 with no violation of it.
- **Doc-only went straight to `main`**, correctly, and the hook allowed it — which is the
  exemption working, not the rule failing.
