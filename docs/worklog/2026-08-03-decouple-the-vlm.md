# 2026-08-03 — Decoupling the VLM from the simulator

Backlog [R-02](../todo.md). The repositioning proper: this becomes a drone simulator that a
VLM can be pointed at, rather than a VLM testbed that happens to contain a simulator.

---

## 1. The launch file was not the coupling

The plan said "remove `vlm_client` and `grounding` from the default launch and move their
config out". Reading the actual subscriptions before writing any code changed that:

    control/offboard_node   subscribes to   /vlm/grounded_waypoint

A drone simulator whose **controller** takes its "go here" input from a `/vlm/` topic is not
decoupled, however few VLM nodes happen to be running. Someone writing a non-VLM planner
would have had to publish to `/vlm/...` to make the aircraft move, and would have concluded —
correctly — that this is a VLM testbed.

So the first change was the rename to `/control/waypoint`, across `grounding` (publisher),
`control` and `episode_runner` (subscribers), `scripts/status.sh` and three docs. Only then
does removing nodes from a launch file mean anything.

## 2. I changed my mind about `evaluation`, in writing, before acting

The R-02 entry said `episode_runner` should move to the example with the VLM. Wrong:
`episode_runner` starts and stops episodes and scores distance-to-goal from odometry. None of
that is VLM work — it is generic scenario running, and a non-VLM user wants it. What is
actually VLM-specific is `vlm_client` (makes annotations) and `grounding` (annotation to
waypoint). The line ended up:

| core | `carla_air_bridge`, `control`, `evaluation` (recorder + episode_runner) |
| example | `vlm_client`, `grounding` |

`episode_runner` and `recorder` keep optional `/vlm/annotation` subscriptions, for step
counting and the video overlay. They stay empty without a VLM, which is what an optional
input should do.

## 3. Logical decoupling, not physical relocation

The entry said the two packages "become `examples/vlm_navigation/`". They did not move out of
`ros2_ws/src/` — a second colcon workspace would add a build step and a sourcing step for no
gain the verify criteria can see. What moved is what a user actually experiences:

* the **launch**: `examples/vlm_navigation/vlm.launch.py` + `run.sh`, started separately;
* the **config**: `examples/vlm_navigation/config/vlm.yaml`, out of `configs/testbed.yaml`;
* the **topic namespace**, above.

`configs/testbed.yaml` no longer contains an Anthropic model name. That was the tell: someone
installing a drone simulator was reading `claude_model: claude-opus-5` in its config file.

Stating the deviation rather than quietly redefining the item: if physical relocation is
wanted, it is a further step and a smaller one.

## 4. Argument compatibility, deliberately

`bringup.sh --backend oracle` was in every doc, every worklog and everyone's muscle memory
for months. It now **warns and redirects** rather than failing with "unknown argument":

    note: --backend moved to the VLM example — bring this up, then run:
            ./examples/vlm_navigation/run.sh --backend oracle

`run.sh` also translates `--backend X` into the `backend:=X` that `ros2 launch` actually
wants. I found that one the honest way: the first run of the example died with
`ros2: error: unrecognized arguments: --backend oracle`, because I had passed `"$@"` straight
through.

## 5. Verified

Bare `./scripts/bringup.sh`, then `ros2 node list`:

    /carla_air_bridge  /episode_runner  /offboard_control  /recorder

No `vlm_client`, no `grounding`. Then `./examples/vlm_navigation/run.sh --backend oracle` adds
exactly those two, and `ros2 param get /grounding camera_pitch_deg` returns -28.6 **from the
example's own config file**.

End to end with the two halves started separately:

    cross_the_plaza seed=1 -> SUCCESS  18.0 m from goal, 14 steps

against a documented 18.6 m / 14 steps. The decoupling did not move the numbers.

**Not done:** the full E-01 sweep. 40 episodes is ~2 h of wall clock because this runs in real
time, and one seed reproducing the baseline exactly is good evidence rather than proof. It is
recorded as owed in the backlog rather than quietly skipped.

Everything stopped: all counts 0, GPU 1 back to 33 MiB.

---

## 6. The E-01 re-sweep failed, twice, and the machine is clean

Ran R-07 (`pytest.ini` disabling the seven ROS pytest plugins — the documented command now
works in a ROS-sourced shell), smoke-tested the rewritten `run_sweep.sh` on 1x1x1, and
started the full 40-episode sweep.

**It died both times**, the same way:

    terminate called after throwing an instance of 'carla::client::TimeoutException'

Attempt 1 got 2 of 20 oracle episodes, then died on the third `reset` — and the sweep kept
going for 18 more episodes against a dead sidecar, writing empty logs. Attempt 2, with the
CARLA timeout raised from 30 s to 120 s, died the same way, which rules out "the timeout was
too tight" as the cause: the simulator is wedging for more than two minutes.

Both fixes stayed in, because both were independently right — 30 s was too tight regardless,
and a sweep that cannot tell a dead sidecar from a slow one is a sweep that wastes an hour
proving nothing.

**I stopped rather than restarting a third time.** The operator was asleep, two attempts had
failed, and their own GPU 0 workload was at 88% with a load average near 4 — a third
unattended run would most likely have burned the night and produced another set of empty
logs. Recorded as E-06 with the leads: contention, whatever the third reset does that the
first two do not, and whether `--no-chase` survives.

Everything stopped and independently confirmed: all node counts 0, no simulator ports, no
sidecar socket, GPU 1 back to 33 MiB and GPU 0 back to its idle 114 MiB.
