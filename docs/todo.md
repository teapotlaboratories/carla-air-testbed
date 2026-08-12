# Backlog

Open work, with the reason and how each will be verified. The `.ai/AGENTS.md` "plan first"
rule points here: non-trivial work gets an entry before it gets code, and the entry is marked
done when it lands.

Status: **open** · **next** (agreed, not started) · **blocked** · **done**

---

## Scope — read this before picking anything up

Agreed 2026-08-04. **The product is the simulator**: a faithful world, faithful sensors, a
ROS 2 interface that behaves the way real hardware would, and runs that repeat. **Navigation
and VLM work is out of scope** and belongs in `examples/`, as a consumer of the public
interface. Full statement in
[`.ai/AGENTS.md`](../.ai/AGENTS.md#scope--what-this-repository-is-for).

The test: **could a user with a completely different navigation stack still want it?**

Every open item, classified. Out-of-scope items are **not** deleted — they are real work, they
were reasoned about, and the reasoning is worth keeping. They are simply not what this
repository is for, and should not be picked up here.

| item | | |
|---|---|---|
| `R-05` headless or windowed from the config | **in** | rendering path; every user needs it |
| `R-06` camera resolution and the 4:3 constraint | **in** | sensor contract; `fov` is horizontal, so aspect changes what a pixel means |
| `S-03` segmentation published but disabled | **in** | a sensor that ships switched off |
| `T-02` H.264 recording and cheaper live streams | **in** | capture path, and the chase/onboard sync is still unresolved |
| `E-03` an MCAP bag per episode | **in** | recording the simulator's own behaviour is fidelity work, not scoring |
| `P-01` containerise the stack | **in**, blocked | deployment; blocked on non-nested Docker |
| `R-03` web console talks ROS 2 only | **in**, planned | proves the interface is sufficient — and is now how the console reaches the containers at all |
| `T-03` pedestrians spawn but do not move | **in**, **and already fixed** | world fidelity; status below is stale, the fix landed 2026-08-03 |
| `V-01b` local vLLM on GPU 1 | **out** | model choice |
| `E-02b` the other scenarios have nothing in the way | **out** | scenario design as a policy challenge |
| `E-04` anchor scenarios to real map features | **out** | same |
| the `bearing_only` blind step | **out** | grounding — pixel + depth → NED is a client concern |
| the camera-pitch decision | **out** *as framed* | it was framed as "what does the model see"; **in** if reframed as a sensor-calibration question |

**Two things this contradicts in the current tree**, both known and neither urgent:

- `bringup.sh` starts `control` (waypoint following: standoff, step capping, altitude floor,
  yaw and velocity slew) and `evaluation` (episode scoring, recording). Under this scope both
  are examples. `examples/ros2_full_control.py` already flies the aircraft through
  `/fmu/in/trajectory_setpoint` importing nothing from the project, so `control` is not
  load-bearing. **No new navigation behaviour should be added to either.**
- `interfaces/` is mixed. `ResetVehicle`, `SpawnTraffic`, `SetWeather`, `SetCameraPose`,
  `DestroyActors`, `ChaseRecording` are simulator contracts. `Annotation2D`,
  `GroundedWaypoint`, `EpisodeStatus`, `EpisodeResult` are navigation types living here for
  historical reasons.

**Decided 2026-08-04: the target is SENSORS AND WORLD, not flight stack.** CARLA-Air contains
no PX4 — `/fmu/*` is a shim over AirSim SimpleFlight, with no EKF2, failsafes, arming logic or
lockstep. That is accepted rather than a gap to close; flight-controller fidelity belongs to
`drone-sim`. Fidelity work here means camera geometry and intrinsics, sensor noise and rates,
lidar, traffic and pedestrian behaviour, weather, and repeatability.

**Recent work that would be out of scope under this rule**, recorded so the boundary is not
retroactively flattering: the velocity and yaw slew, the yaw-gate fix, per-scenario controller
overrides, and the `bearing_only` analysis. All landed before 2026-08-04.

---

## Repositioning — a drone simulator first, a VLM testbed second

Agreed 2026-08-03. The project's product is **a ROS 2-driven drone simulator**; VLM navigation
becomes the flagship *example* of what you can build on it, not the thing it is. Everything in
this section serves that, and the recommended order is at the end.

The line that decides every question here: **the only interface to the aircraft and the world
is ROS 2.** If something needs the sidecar socket, it is not finished.

### R-01 · Put world control on ROS 2 — **done** *(2026-08-03)*

The keystone, and the item that is not obvious from the outside. [S-04](#sensors) closed
*aircraft* commanding — takeoff, land, position, velocity, attitude all have `/fmu/in/*`
topics. **World** control has no ROS surface at all. `sim_bridge/server.py` exposes ~30 RPC
methods; these are the ones a scenario needs and ROS cannot reach:

| method | what it does | why it must be on ROS |
|---|---|---|
| `reset` | teleport and hold at an NED pose | every example starts with one |
| `spawn_traffic` | vehicles + walkers | scenario setup |
| `set_weather` | CARLA weather preset | scenario setup |
| `destroy_actors` | clear traffic between runs | scenario teardown |
| `chase_view` / `chase_jpeg` | the exterior follow camera | the web console's second video pane |

> **Corrected before starting:** `collision` was in this table and should not have been.
> `/sim/collision` (`Bool`) and `/sim/traffic_stats` (`String`) are already published at 1 Hz
> from `_tick_world` (`bridge_node.py:503`). I listed them from reading the RPC surface
> instead of the publisher list. Read state is therefore already on ROS; what is missing is
> the *commands* above and the chase camera.

Consequences, both of which are real work rather than plumbing:

- **The chase camera needs a ROS surface — but a narrower one than this said.** *(Revised
  2026-08-03, when R-03 was deferred.)* Originally "it must become an `image_transport`
  publisher". That was driven entirely by the web console's second video pane. Chase
  *recording* writes its own mp4 inside the sidecar (`chase_start` / `chase_stop`) and never
  streams frames anywhere, so what `run_episode.py` actually needs is those two calls as
  services. The video topic is deferred with R-03.
- **Services or topics, decided per method.** `reset` and `spawn_traffic` are request/response
  with a meaningful failure — services. `collision` is state — a topic. Guessing uniformly
  either way will be wrong for half of them.

`ros2_ws/src/interfaces` already exists for exactly this kind of definition — four messages
and one service (`SetEpisode.srv`) today, all of them VLM/evaluation-shaped, which is itself a
hint that the world-control definitions were never written.

- **Verify:** a node importing only `rclpy` and the interface package resets the aircraft,
  spawns traffic, sets weather, subscribes to the chase camera and reads collisions. Then
  **`scripts/run_episode.py` is rewritten to use it** — that script is the honest test,
  because it is the heaviest current user of the socket (`scripts/run_episode.py:95-118`).

**Landed 2026-08-03 — the four services.** `ros2_ws/src/interfaces/srv/` gains
`ResetVehicle`, `SpawnTraffic`, `SetWeather` and `DestroyActors`, served by the bridge on
`/sim/*`. Services as agreed, and **not** PX4 messages: nothing on a real Pixhawk teleports an
airframe or spawns pedestrians, so borrowing `VehicleCommand` would imply a portability these
calls do not have. `/fmu/*` is what survives the move to hardware, `/sim/*` is what does not.

`examples/ros2_world_control.py` is the proof — imports `rclpy`, `interfaces` and `std_msgs`,
nothing else. Against a live simulator, **8/8 checks passed**:

| check | result |
|---|---|
| `destroy_actors` | destroyed 0, then 26 on teardown |
| `set_weather ClearNoon` | applied |
| `set_weather NoSuchWeather` | **rejected**, valid list returned — not a silent fallback |
| `reset_vehicle` | settled `[66.4, -106.0, -60.9]`, 8.8 m from commanded |
| `spawn_traffic` | 12 vehicles, 7 of 8 walkers |
| `/sim/collision` | `has_collided=False` |
| `/sim/traffic_stats` | `spawned=12 moving=12 walkers=7` |

**Two things only the run could tell me:**

- **`reset` blocks for 16.2 s**, not the ~5 s I wrote in the comments — 3 s of AirSim reset,
  the flight itself, then a 2 s settle. A client on rclpy's default 5 s timeout would report a
  failure that did not happen. Every comment now carries the measurement.
- **The blocking call does not stall telemetry**, which was the design risk. Measured
  *during* that 16.2 s reset: `/fmu/out/vehicle_odometry` held **19.9 Hz** against a 20 Hz
  target. That is what the fourth `SimBridgeClient` connection and the fifth executor thread
  bought; on `self.sim` it would have stalled odometry and the world tick for the whole call.

Also fixed on the way: `client.spawn_traffic` silently dropped `near_ned` and `radius_m`, so
every ROS-side spawn would have been map-wide — a scenario asking for busy streets getting an
empty city, with nothing to indicate it.

**Finished the same day.** Two more services (`SetCameraPose`, `ChaseRecording`) and a
`Collision` message replacing the bare `Bool` on `/sim/collision` — the flag alone had people
searching video for a building the log already knew the name of.

`scripts/run_episode.py` is now **a plain ROS 2 client**. It no longer opens the socket, and
it no longer shells out through `bash -lc` to `ros2 service call` / `ros2 param set` either;
those became a service call and a `SetParameters` client. It runs under ROS's 3.12 via
`scripts/run_episode.sh`. That was the real test of the surface, and it passed:

    cross_the_plaza seed=1 -> SUCCESS 19.4 m, 13 steps
    cross_the_plaza seed=2 -> SUCCESS 19.4 m, 13 steps

against a documented baseline of 18.6 m / 14 steps. Chase recording still works through the
new service (452 frames, 0 dropped).

**Three bugs the port exposed, none of them in the port:**

- **`destroy_all` was fire-and-forget.** It used `apply_batch`, so destruction was still in
  flight when the next `spawn_traffic` attached walker controllers to half-reaped walkers.
  CARLA threw `set_actor_simulate_physics: Actor could not be found in the registry` as an
  uncaught C++ `std::runtime_error`, which calls `terminate()` and **took the whole sidecar
  down**. The old script was slow enough between the two calls to hide it; a ROS client is
  not. Now `apply_batch_sync`, and three back-to-back destroy/spawn cycles survive.
- **`set_camera_pose` was in no lock class** — the *fourth* instance of "lock classes guard
  dispatch, not sockets". It drives the media AirSim client but took `slow_lock`, so it ran
  concurrently with `capture` on one msgpackrpc socket, failed with `IOLoop is already
  running`, and **silently did not apply the camera pitch** — on a measurement surface every
  scored episode depends on. Now in `MEDIA`.
- **`near_ned` falls back to map-wide silently** when the radius holds too few spawn points
  (`world.py:83`). Measured, that is 20 cars within 60 m of the aircraft versus 5. The
  service now reports it: *"NOT clustered: only 0 spawn points within 30 m … fell back to
  map-wide"*.

**Deferred with [R-03](#r-03--the-web-console-talks-ros-2-only--planned-revised-2026-08-07):** the
chase camera still has no video *topic*. Nothing needs one now that recording is a service —
the web console was the only consumer.

### R-02 · Decouple the VLM from the core — **done** *(2026-08-03)*

Today `vlm_client` and `grounding` are launched unconditionally
(`ros2_ws/src/bringup/launch/testbed.launch.py:50-58`) and their settings are core config —
`graph.vlm_client` and `graph.grounding` in `configs/testbed.yaml`, including six
`claude_*` keys. Someone who wants a drone simulator is reading an Anthropic model name in
the config file of the thing they installed.

**The move:**

- `graph.vlm_client` and `graph.grounding` leave `configs/testbed.yaml` entirely. The example
  carries its own config.
- The default launch starts the bridge and the controller. Nothing else.
- `vlm_client` + `grounding` become `examples/vlm_navigation/`, run **after** the simulator is
  up, against the ROS 2 interface only — the same rule `examples/ros2_full_control.py` already
  follows.

**Refined 2026-08-03, before writing any code.** Reading the actual subscriptions changed
what this item is. The launch file is the *visible* coupling; the real one is the **topic
namespace**:

    control/offboard_node  subscribes to  /vlm/grounded_waypoint

A drone simulator whose controller takes its "go here" input from a `/vlm/` topic is not
decoupled, however few VLM nodes are running. So the first move is a rename to a neutral
`/control/waypoint`, and only then does removing the nodes from the launch mean anything.
Touches `grounding` (publisher), `control` and `episode_runner` (subscribers),
`scripts/status.sh`, and two HTML docs.

**And I have changed my mind about `evaluation`.** The entry below says `episode_runner`
should go with the example. It should not: it starts and stops episodes and scores
distance-to-goal from odometry, which is generic scenario running, not VLM work. What is
VLM-specific is `vlm_client` (makes annotations) and `grounding` (annotation to waypoint).
So the line is:

| core | `carla_air_bridge`, `control`, `evaluation` (recorder + episode_runner) |
| example | `vlm_client`, `grounding` |

`episode_runner` keeps an optional `/vlm/annotation` subscription for step counting, and the
recorder keeps one for its overlay. Both simply stay empty without a VLM, which is the
correct behaviour for an optional input rather than a coupling.

**Two decisions this forces, and neither is cosmetic:**

- **Where does `evaluation` go?** `episode_runner` and the scenario scoring exist to score
  *VLM* episodes. But `recorder` (flight video) is generally useful, and a
  non-VLM user still wants "fly this and tell me if it worked". My reading: `recorder`
  stays core, `episode_runner` + `scenarios/` go with the example. Worth disagreeing with.
- **`grounding` is not VLM-specific.** It turns a pixel and a depth frame into an NED
  waypoint — useful to anyone doing vision. It may deserve to stay core as a *library* while
  only its node moves.

**This is the item most likely to break something quietly**, because the launch file, the
config, the params renderer, `run_sweep.sh`, `run_conformance.sh` and the E-01 baselines all
assume the current layout. The 40-episode results must reproduce afterwards.

- **Verify:** `./scripts/bringup.sh` with no arguments brings up a simulator with **no VLM
  node running** and `ros2 node list` proves it; then the example is started separately and
  `cross_the_plaza` scores what it scored before. Re-run the E-01 sweep, not one seed.

**Done.** `ros2 node list` after a bare `./scripts/bringup.sh`:

    /carla_air_bridge  /episode_runner  /offboard_control  /recorder

No `vlm_client`, no `grounding`. Then `./examples/vlm_navigation/run.sh --backend oracle`
adds exactly those two, reading `examples/vlm_navigation/config/vlm.yaml` — confirmed live
with `ros2 param get /grounding camera_pitch_deg` returning -28.6 from the example's own
file, not the simulator's.

**The topic rename was the substance.** `/vlm/grounded_waypoint` -> `/control/waypoint`
across `grounding`, `control`, `episode_runner`, `status.sh` and three docs. Removing nodes
from a launch file is cosmetic while the controller's "go here" input still sits in a `/vlm/`
namespace.

`--backend` and `--instruction` are gone from `bringup.sh`, but **accepted and redirected**
rather than rejected — every doc and every finger in this project reached for them for
months, so an "unknown argument" error would have been the wrong answer. `run.sh` likewise
translates `--backend X` into the `backend:=X` that `ros2 launch` actually wants.

`configs/testbed.yaml` no longer contains an Anthropic model name, which was the tell.

**Scored, decoupled:** `cross_the_plaza` seed 1 -> **SUCCESS 18.0 m, 14 steps**, against a
documented baseline of 18.6 m / 14 steps.

- **Still owed:** the full E-01 sweep. **Attempted twice on 2026-08-03 and it failed both
  times** — see E-06 below. One seed reproducing the baseline exactly remains good evidence
  and not proof.

### Q-01 · Two PRs shipped logic with no test — **done** *(2026-08-06)*

Both covered, 17 tests, no simulator:

- `tests/test_reset_convergence.py` (6) — a fake AirSim client returning a scripted sequence
  of positions, so the convergence loop's three outcomes are checked in milliseconds instead
  of against a live aircraft.
- `tests/test_precommit_hook.py` (11) — a temporary repository per test, so it touches neither
  this checkout nor its git config.

**Verified by mutation, because a passing test proves nothing until it fails against the bug:**

| reverted | tests failing |
|---|---|
| the stall guard removed (the D-05 fix) | 1 |
| stall guard too eager, 0.3 → 5.0 | 1 |
| tolerance back to 6.0 (pre-D-05) | 3 |
| no retry at all (pre-D-03) | 4 |
| hook's `--diff-filter=ACMR` restored (the deletion gap) | 1 |

**And the mutation testing itself was wrong the first time**, which is worth more than the
tests. Both mutations replaced a constant with one of the *same byte length* — `0.3`→`5.0`,
`1.5`→`6.0` — and the writes landed in the same second, so Python's `(mtime, size)` bytecode
check saw the cached `.pyc` as valid and kept running the old module. Two mutation results and
the "restored" run were all measuring stale bytecode. Caught only because the restored tree
still failed a test it had just passed. **Clear `__pycache__` between mutations**; a
same-length edit defeats the cache check.



Raised by `/review` on PR #2 and again on PR #3. Both landed branches whose whole substance is
a piece of pure logic, and neither has a test, in a repository with eleven test files and a
stated rule that pure logic goes in `tests/`.

| what | why it is testable offline |
|---|---|
| `.githooks/pre-commit` path matching | a temp repo, stage a file, assert the exit code. Already done by hand four times; never automated |
| `Vehicle.reset()` convergence loop | `Vehicle.__init__` takes a client, so a fake returning a scripted sequence of positions covers converges / stalls / exhausts-attempts with no simulator |

The second is the one that matters. `RESET_MIN_IMPROVEMENT_M` decides when to stop retrying,
and its failure mode is silent — stop too early and a reset that would have converged reports
a miss instead; too late and street level burns four attempts. Both were verified by hand
against a live simulator, which is the slow, expensive way to learn something a fake client
answers in milliseconds.

`tests/test_chase_stop.py` exists because a deadlock survived only-in-flight testing, and it
was written two days before both of these. The pattern is worth naming rather than repeating.

- **Verify:** each test fails against the code as it was before its fix, the way
  `test_control_limits.py` and `test_h264_timing.py` were checked before being trusted.
- **Not urgent.** Both behaviours are verified and recorded; this is about them staying
  verified.

### R-03 · The web console talks ROS 2 only — **planned** *(revised 2026-08-07)*

`webui/server.py` dispatches sidecar RPC methods over the Unix socket
(`webui/server.py:328-332`) and imports no ROS at all. Every button — velocity, yaw, hold,
land, takeoff — bypasses the interface this project claims is the interface.

**Revised 2026-08-07, and the revision changes the answer.** This was deferred on 2026-08-03
as a tidiness item with nothing depending on it. Two things have since made that reading
wrong, and a third made the estimate wrong:

1. **The containers arrived (2026-08-06) and stranded the console.** It is in no image, no
   script starts it, and it cannot reach the containerised sidecar at all: it looks for
   `/tmp/carla_air_testbed.sock` (`sim_bridge/protocol.py:23`) while the stack serves
   `/run/carla-air/sim.sock` on volume `carla-air-run` (`scripts/stack_up.sh:40`), whose host
   mountpoint is `Permission denied` under the rootless daemon *(checked 2026-08-07)*. So the
   current design is not merely untidy, it is a **dead end** — and porting to `rclpy` is now
   the *cheapest* route to a containerised console, cheaper than joining namespaces with the
   3.10 process, because an `rclpy` node needs no socket at all.
2. **The socket path is why the console contends with the graph.** It opens a *second* AirSim
   capture, which is the whole reason it carries a do-not-run-during-a-scored-episode warning
   (`webui/server.py:26-29`). Subscribing to `/camera/rgb/image_raw` means one capture serves
   the agent and the console both — so a scored run becomes watchable live, which is the use
   case the console was built for and currently cannot serve.
3. **The job is much smaller than this entry implied.** The allowlist exposes all 30 sidecar
   methods, but the console only *uses* **14** — nine from the page, five from the server.
   Thirteen of those already exist on the ROS surface:

| what the console calls | on ROS 2 today |
|---|---|
| `state` | `/fmu/out/vehicle_odometry` |
| `collision` | `/sim/collision` |
| `reset` | `/sim/reset_vehicle` |
| `spawn_traffic`, `set_weather`, `destroy_actors` | the three matching `/sim/*` services |
| `takeoff`, `land`, `hold`, `velocity`, `yaw` | `/fmu/in/trajectory_setpoint` + `VehicleCommand` |
| `view_jpeg` | `/camera/rgb/image_raw` |
| `chase_view` | `/sim/chase_recording` |
| **`chase_jpeg`** | **nothing — the one genuine gap** |

`examples/ros2_full_control.py` is the existence proof for row five, and a hard one: it flies
takeoff, land, velocity and yaw over `rclpy` + `px4_msgs` while **importing nothing from the
testbed**. So this item is not "grow the ROS surface to match the console". It is "make the
console use what already works".

**Do not invent friendly services to make the port easy.** `examples/ros2_full_control.py:17-20`
refuses `/testbed/takeoff` on purpose: moving to hardware must mean deleting `carla_air_bridge`
and starting `uxrce_dds_client`, not rewriting clients. If porting a button tempts you to add
one, the shim is what broke.

**One honest carve-out, and it is permanent.** *Start the simulator* and *stop everything*
cannot be ROS calls — there is no graph to call into before the simulator exists, and the stop
button's whole job is to destroy the graph. Those stay process management, and keep shelling
out to the project's own scripts so the Vulkan ICD repair, the GPU pin and the path-scoped
process matching are not reimplemented. The claim afterwards is **"flying is ROS; lifecycle is
not"** — not "the only interface is ROS 2", which will never be true of this component.

**Sequence.** Each step is independently shippable and leaves the console usable:

1. **Console becomes an `rclpy` node** in the ROS container. Onboard pane from
   `/camera/rgb/image_raw`, telemetry from `/fmu/out/vehicle_odometry` and `/sim/collision`.
   Control stays on the socket. *This is the step that closes the container gap.*
2. **Control moves to ROS** — `TrajectorySetpoint` / `VehicleCommand` for flight, the four
   existing `/sim/*` services for the world.
3. **Start/stop repoint** at `stack_up.sh` / `docker stop`.
4. **Chase pane: decide, do not assume.** It is the only thing needing new plumbing (an
   `image_transport` pipeline nobody else consumes). Ship one pane first and answer it with
   the console in hand.

**Step 1 landed 2026-08-07, and the measurement corrected the plan.** `webui/ros_source.py`
subscribes to `/camera/rgb/image_raw`, `/fmu/out/vehicle_odometry`, `/fmu/out/vehicle_status`
and `/sim/collision`; `scripts/webui.sh` starts it with the environment set up. Measured
against a live graph, watching the camera topic while a browser streams — three
drift-controlled A/B pairs each way, because a first attempt showed a 10% drop that turned out
to be **within the baseline's own drift** and proved nothing:

| console | camera rate | cost |
|---|---|---|
| running, **not streaming** | 6.39 – 6.66 Hz | baseline |
| streaming, **socket** | 5.060 Hz | **−24.0%** |
| streaming, **ROS** | 5.968 Hz | **−6.6%** |

**The baseline is the console *running but not streaming*, not absent.** On ROS an idle console
still subscribes and runs `imgmsg_to_cv2` on every frame, so that cost sits **inside** the
baseline; on the socket an idle console issues no RPCs at all, so its idle cost is ~0. The
comparison is therefore biased in ROS's favour: **−6.6% is a floor on the ROS console's total
cost, not the total.** The direction survives — 24% is large — but a true no-console baseline
is **unmeasured**.


- **The verify criterion below was wrong and is corrected.** It said the rate would be
  *unchanged*, on the reasoning that one capture fanned out by DDS costs nothing. It is not
  unchanged: **re-encoding to JPEG costs ~6.6%**, on the machine that is also rendering. The
  right claim is **3.6× cheaper, not free** — the difference between a console you must close
  before a scored run and one you can leave open. Every user-facing string now states the
  number instead of the word "safe".
- Found while measuring: the stream served 12 fps from a 6.4 Hz source, so **half the JPEG
  encodes were the same frame again**. Now encoded at most once, keyed on arrival stamp.

**Step 2 landed 2026-08-07.** `webui/ros_control.py` maps every console button onto the ROS
surface that already existed — `TrajectorySetpoint` and `VehicleCommand` for flight, the four
`/sim/*` services for the world. **Nothing was added to the ROS surface for it.** Verified
against a live graph:

| button | over ROS | result |
|---|---|---|
| takeoff −30 NED | `VEHICLE_CMD_NAV_TAKEOFF` param7 = 30 | NED z 28.91 → **−30.41** |
| velocity ×4 north | `TrajectorySetpoint.velocity` | north 6.5 → **12.8 m** |
| yaw 90 | setpoint, absolute heading | yaw −89 → **+82°** |
| set_weather | `/sim/set_weather` | `success: true, applied: HardRainSunset` |

**Review of the step-2 PR found the guard had a hole**, and it is worth recording because the
shape recurs: `reset` is a `/sim/*` service, so it sorted with world control and the guard
skipped it — but `/sim/reset_vehicle` takes a `hold_ned` and **flies the aircraft there**, then
runs the D-03 convergence loop for several seconds. Relocating an aircraft out from under a
running controller is strictly worse than the velocity nudges the guard did refuse. *"Flight
command"* and *"moves the aircraft"* are different sets and the guard needs the second one.
The test that should have caught it instead **encoded the omission**: it asserted only that
`set_weather` was excluded. There is now a table naming why each unguarded method is safe, so
adding one without deciding is a failure rather than an oversight.

**The contention decision, and it is enforced:** a second publisher on
`/fmu/in/trajectory_setpoint` makes the console **refuse to fly** — HTTP 409, naming how many
nodes it is declining to fight — while world commands stay allowed. Measured with a rival
publisher: `contested: 0 → 1`, flight `409`, `set_weather` still `200`, then `1 → 0` and flight
allowed again when the rival exited. The alternative (seizing control, as
`examples/ros2_full_control.py` does) is a large side effect for a button press; after step 1
the console's job is watching a run, so it declines instead. Two publishers on one setpoint
topic produce **no error at all**, just an aircraft that goes somewhere neither asked for —
that example measured a takeoff to NED 35 m arriving at 15.6 m.

Three semantics preserved rather than improved, because step 2 must move the buttons without
changing what they do: **`yaw` is absolute** (the sidecar calls `rotateToYawAsync`), `hold` is a
literal zero-velocity command, and **the velocity lifetime is not ours to set** —
`TrajectorySetpoint` has no duration field, so the bridge's `setpoint_duration_s` (0.5 s)
applies and the page's `duration: 0.7` is simply not expressible. Inventing a duration field to
paper over that is what the no-friendly-services rule forbids.

- **Verify:** after step 2, `grep -c 'socket' webui/server.py` reaches zero for the control and
  video paths, and every control still works over NetBird. Onboard video comes from a ROS
  topic. ~~The camera rate the agent sees is unchanged with the console open~~ — **superseded
  2026-08-07**: the rate is *materially reduced but not eliminated*, and what the step must
  produce is the measured pair above, not a null result.
  **Met, with one honest qualification:** in ROS mode no control or video goes over the socket.
  Four `CONTROL.call` sites remain and all four are legitimate — two are the socket-mode
  *fallback* for state and collision, one is the socket-mode command fallback for methods with
  no ROS equivalent, and one is `chase_view`, which is step 4. A literal zero is only reachable
  by deleting the fallback, which would make the console unusable without a graph.

> **Superseded in part.** The 2026-08-03 deferral is kept below because its *reasoning* about
> the chase camera still holds — but its conclusion ("nothing depends on the console, so this
> can wait indefinitely") no longer does. The containers changed the cost of waiting.

> **Deferred at the operator's request, 2026-08-03.** The console is a convenience for
> looking at the simulator by hand; nothing depends on it. Two consequences worth stating
> rather than discovering later:
>
> - **The chase camera does not need a ROS topic yet.** It was on R-01's list *only* because
>   this item needed a second video pane. Chase recording writes its own mp4 inside the
>   sidecar via `chase_start`/`chase_stop` and never streams frames to ROS, so R-01 can be
>   finished with two more services instead of an `image_transport` pipeline nobody is
>   consuming. The topic comes back when this item does. *(Still true, and step 4 above is
>   where it gets answered.)*
> - **The console keeps talking to the socket until then**, so "the only interface is ROS 2"
>   has a known, documented exception rather than being quietly untrue. *(Still true, and now
>   also the reason it cannot reach the containerised stack.)*

### R-04 · One command to install — **done** *(2026-08-03)*

`./scripts/install.sh` runs all four steps with a step counter, per-step timing, and one
failure path that names the step and says re-running resumes. `--skip-release` omits the
6.85 GB download; a positional argument puts the release somewhere with room.

**The export was the real problem, and it is gone rather than automated.** Four scripts each
inlined the same `${CARLAAIR_RELEASE:-${CARLAAIR_HOME:-<default>}}` expression — they agreed
by luck, not by construction, and `run_sweep.sh` hard-failed without the variable set.
`scripts/release_path.sh` is now the single resolver, with an explicit precedence:

    1. $CARLAAIR_RELEASE   an override for one command
    2. .release-path       written by install.sh, git-ignored
    3. $CARLAAIR_HOME
    4. next to the repo

So a non-default install location survives with **no shell-profile edit at all**, and
`fetch_release.sh` no longer prints an export line for the user to copy.

**Verified**, not assumed:

- `install.sh --skip-release` on this tree: 3/3 steps green in 8 s (everything already
  satisfied), then 117 tests pass. Run twice — idempotent.
- The failure path exercised directly with a failing step: prints
  `INSTALL FAILED at step 1`, names the command, exits 1, does not continue.
- The resolver at all four precedence levels, each returning the expected path.
- `bash -n` on all five touched scripts.

**Found while doing it, and worth recording:** `CARLAAIR_RELEASE` was **unset** in a fresh
shell on this machine, and the built-in default points at a directory that does not exist —
the real release is on the external drive. Every script that needed it was one un-exported
variable away from failing, and `.release-path` is what fixes that here.

**Clean-tree install verified 2026-08-03, and it was broken.** Simulated a fresh checkout by
copying every file `git` would carry (147 of them - tracked plus untracked-not-ignored, so
no `.venv`, no `vendor/`, no build output) into an empty directory, then running
`install.sh --skip-release` under `env -i` with nothing inherited.

**Three dependencies were missing from `setup_env.sh`**, and none of it showed on this
machine because it had acquired them outside the scripted path:

| missing | consequence on a fresh machine |
|---|---|
| **PyYAML** | `apply_config.py` cannot read `configs/testbed.yaml`, and `bringup.sh` runs it FIRST — **the simulator could not start at all** |
| **pytest** | the command `install.sh` prints on success, and that the README, quickstart and both guide tabs tell a new user to run next, failed with `No module named pytest` |
| **av** (PyAV) | `h264.py` — opencv ships no libx264 (GPL vs Apache), so chase and episode recording were dead |

Fixed in `setup_env.sh`. Re-verified on the same clean tree: `pytest tests/ -q` gives
**125 passed, 1 skipped**, and `apply_config.py` renders.

- **Still unverified:** the 6.85 GB download itself (`--skip-release` was used, since the
  release is already on disk). Everything downstream of it now is.

### R-07 · The offline suite dies in a ROS-sourced shell — **done** *(2026-08-03)*

`./.venv/bin/python -m pytest tests/ -q` is the documented command, and it fails with
`ModuleNotFoundError: No module named 'lark'` whenever the shell has sourced ROS — which is
every shell that has just run `bringup.sh`.

**Diagnosed, so this is a fix and not an investigation.** It is not our code: sourcing ROS
puts Jazzy's 3.12 site-packages on `PYTHONPATH`, the 3.10 pytest then autoloads the pytest
plugins ROS registers there (`launch_testing` and friends), and those import `launch`, which
wants `lark`, which the 3.10 venv does not have. `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` makes it
pass. Collection never starts, so a `conftest.py` cannot help — it runs too late.

Pre-existing, not introduced by the ROS-services work; found while adding
`tests/test_interfaces.py`. The error names a package nobody has heard of and points at
`/opt/ros`, which is exactly the shape of misleading failure this project keeps paying for.

- **Verify:** the documented command passes both in a clean shell and in one that has
  sourced ROS.

**Done.** A `pytest.ini` at the repo root disables the seven plugins ROS registers
(`ament_copyright`, `ament_flake8`, `ament_lint`, `ament_pep257`, `ament_xmllint`,
`launch_ros`, `launch_testing`). It has to live there rather than in a `conftest.py`, because
plugin loading happens *before* collection.

Verified in both shells with the documented command verbatim:

    clean shell   117 passed, 1 skipped
    ROS sourced   124 passed          (the 3.10 interpreter can reach `interfaces` via the
                                       leaked PYTHONPATH, so test_interfaces runs too)

The skip is `tests/test_interfaces.py`, which `importorskip`s the generated ROS messages —
correct behaviour, not a gap.

### R-05 · Headless or windowed, from the config — **half done, windowed blocked** *(2026-08-03)*

`scripts/run_sim.sh:194` hardcodes `-RenderOffScreen`. A `simulator.display:` key selects
`headless` (current behaviour, video via the web console) or `windowed` (an actual Unreal
window on the operator's screen).

Small change, but with a **verification risk worth stating up front**: this development
container has no display, and `run_sim.sh:6` records that `-windowed` needs one. So the
windowed path may only be testable from the host, and that is an "ask the operator first"
action under the environment rule rather than something to just try.

Note the interaction with the GPU pin — a window has to open on a display that GPU 1 can
actually drive, and GPU 0 is the operator's card.

- **Verify:** both values start a working simulator; headless is unchanged and windowed
  produces a visible window without breaking the VRAM check.

**The option exists; the windowed path does not work yet.** `simulator.display:
headless | windowed`, with `run_sim.sh --display MODE` overriding for one run. Headless is
unchanged and re-verified — ready in 5 s, 3240 MiB, hardware rendering confirmed.

**Tested on a VIRTUAL screen rather than the operator's desktop**, which is what made it safe
to try at all: `Xvfb :99 -screen 0 1280x720x24`, then `DISPLAY=:99 … --display windowed`.
Result:

- the process starts and **correctly pins to GPU 1** (1225 MiB against pid 176661, so the
  GPU selection logic survives the mode change);
- but it **never serves :2000 or :41451**, holds only ~1.2 GB against the ~3.3 GB a loaded
  Town10HD needs, and writes a **0-byte `out/sim.log`** — this project's documented signature
  for "rendering setup failed".

Diagnosis: Xvfb is a plain software framebuffer with no Vulkan WSI, so the NVIDIA ICD has
nowhere to create a swapchain. Nothing on this box bridges that gap — no VirtualGL, no
`xf86-video-dummy` (there are no xorg driver modules at all).

**Options, none of them free:**

1. `DISPLAY=:0` against the operator's real desktop. Likely to work, but the window opens on
   a screen driven by GPU 0 while the simulator renders on GPU 1, and it is a visible,
   disruptive thing to do to someone's session. **Needs asking, every time.**
2. ~~Install VirtualGL into the container, which bridges GPU rendering to a virtual X
   screen.~~ **Ruled out 2026-08-03 — do not retry.** VirtualGL 3.1.4 ships exactly three
   interposers (`libvglfaker.so`, `-opencl`, `-nodl`) and **all of them fake GLX/OpenGL**.
   There is no Vulkan faker. Meanwhile the simulator binary is Vulkan-only:

       strings CarlaUE4-Linux-Shipping | grep -c VulkanRHI   ->  108
       strings CarlaUE4-Linux-Shipping | grep -c OpenGLDrv   ->    0

   So VirtualGL would sit in a code path this binary never enters. (Upstream's launcher
   scripts do mention `-opengl4`, which is what makes this look plausible from the outside —
   but the shipped binary has no OpenGL RHI compiled in, so the flag has nothing to select.)
   Checked before installing anything; the download was deleted unused.
3. Leave windowed unsupported and say so in the config comment rather than shipping a mode
   that fails with an empty log — which is precisely the failure this project has burned the
   most time on.

Currently option 3 is the state of the tree: the key exists, `windowed` refuses cleanly when
`DISPLAY` is unset and prints the Xvfb recipe, but that recipe does not actually produce a
working simulator here.

**With option 2 ruled out, the real choice is 1 or 3.** Everything that could carry a Vulkan
surface here belongs to the operator's session — `/tmp/.X11-unix/X0` and
`/run/user/1000/wayland-0` are theirs — so any working `windowed` mode means rendering into
the operator's desktop, and that is a per-run decision they have to make, not a default this
project can ship. The `Xvfb` recipe in `run_sim.sh --help` should be corrected or removed:
right now it suggests something that does not work.

### R-06 · Camera resolution, configurable — **done** *(2026-08-04)*

**Decision: keep the constraint, make the failure loud and early** (option 1 below). The
aspect-ratio rule was enforced only by `tests/test_config.py`, which meant a config that
would silently mis-map every waypoint could still start a simulator any time the suite was
not run — and this project has now shipped two bugs of exactly that shape (dead
`sidecar.chase_camera` config, and `ros2-api.html` publishing 640x480 intrinsics against a
960x720 buffer). A rule only a test knows is a rule that reaches a flight.

`scripts/apply_config.py` gained `validate()`, called before anything is rendered, and
`run_sim.sh` refuses to launch on a non-zero exit. Measured on a 16:9 RGB beside 4:3 depth:

    ERROR: <config> is not usable:
      * camera buffers must all share one aspect ratio, because `fov` is HORIZONTAL:
               depth 160x120 (1.333), rgb 1280x720 (1.778), segmentation 320x240 (1.333)
             ... a pixel index from the RGB frame reads the wrong place in depth — with no
             error, on every waypoint.

It also rejects non-positive dimensions, an fov outside 1-180 degrees, missing or
non-numeric fields, and no cameras at all — the same class of thing, all silent today.
Six tests in `tests/test_config.py`; suite is 161 passed, 1 skipped.

Option 2 (lift the constraint by computing each buffer's vertical FOV and scaling correctly)
is **not** taken. It is more flexible and it touches the one code path where a silent error
costs a whole flight, which is a poor trade for a constraint nobody has asked to break.

---

*(original entry)*

### R-06 · Camera resolution, configurable — superseded by the decision above

`simulator.cameras` in `configs/testbed.yaml` already sets width, height and FOV for all
three buffers, renders into AirSim's `CaptureSettings`, and the intrinsics are **derived**
rather than fixed (`sim_bridge/carla_air/frames.py:62-84` computes fx/fy/cx/cy from width and
HFOV) with `grounding` reading them off `/camera/rgb/camera_info`
(`ros2_ws/src/grounding/grounding/grounding_node.py:128`). So changing the resolution should
already work end to end.

What is left is the constraint, and it is a real one: **all three buffers must share an aspect
ratio**, because `fov` is horizontal, so equal FOV at different aspects covers different
*vertical* fields and `scale_to()` then maps an RGB pixel onto the wrong depth pixel —
silently, on every waypoint. A test enforces it today.

Two options, and this is a decision rather than code:

1. **Keep the constraint**, and make the failure loud and early — a clear error at config
   render time naming the offending buffer, instead of a test failure.
2. **Lift it**, by computing each buffer's vertical FOV and scaling correctly. More flexible,
   and it removes a footgun; also touches the one code path where a silent error costs a
   whole flight.

- **Verify:** run an episode at 1280x960 and confirm the grounding residual is unchanged
  (~3 m on a 64 m ray) — the number that would move if the depth hop broke.

### Recommended order

1. ~~**R-04**~~ — **done 2026-08-03.**
2. ~~**R-01**~~ — **done 2026-08-03**, `run_episode.py` included.
3. ~~**R-02**~~ — **done 2026-08-03**. The repositioning is complete apart from R-03, which was re-planned on 2026-08-07 once the containers stranded the console.
4. **R-07**, **R-05**, **R-06** — small and independent, in whatever order suits.
4. **R-05** and **R-06** any time — small, independent, and neither blocks anything.

Deliberately **not** started until the above lands: the camera-pitch decision, and V-01's
Claude flights. Both are VLM work, and R-02 moves the code they live in.

---

## Sensors

### S-01 · Bridge GPS to a ROS 2 topic — **done** *(2026-08-02)*

**Decided:** ship AirSim's default origin, and make it settable at simulator start rather than
baked in. `scripts/run_sim.sh` already copies `configs/sim/settings.json` into place, so that
copy is the hook — `TESTBED_ORIGIN_LAT/LON/ALT` patch an `OriginGeopoint` into it at launch.
With nothing set, AirSim uses its own default and behaviour is unchanged.

Probed live rather than assumed: GPS answers today with `47.639686, -122.138289`, fix 3,
eph/epv 0.10 — Redmond, Washington, exactly as this entry predicted.

_Original entry follows._



`Vehicle.state()` returns position, velocity, angular velocity, orientation, yaw, landed and
armed, and stops there. A grep for `gps|geo_point|latitude|longitude` across `sim_bridge/`,
`ros2_ws/src/` and `configs/` matches nothing, so there is no geodetic position anywhere in
the graph.

Upstream already has it: AirSim's `getMultirotorState()` carries a `gps_location` GeoPoint,
and `simGetGroundTruthEnvironment()` adds air pressure, temperature and gravity.
`px4_msgs` already builds both `SensorGps` and `VehicleGlobalPosition`, so nothing new needs
generating.

**The part that needs a decision, not just code.** `configs/sim/settings.json` sets no
`OriginGeopoint`, so AirSim falls back to its built-in default — Redmond, Washington
(`47.63982902, -122.13757430`, observed). The aircraft is flying in CARLA's Town10HD, which
has no relationship to those coordinates. As shipped, GPS would be a synthetic offset from an
arbitrary origin: usable as a stand-in for a GPS *sensor* (fix quality, noise, dropout),
useless as geolocation. Pinning a deliberate `OriginGeopoint` for Town10HD makes NED↔geodetic
consistent, and is the same hook the reference plan wants for Cesium real-world maps.

- Add `gps` and optionally `environment` to `Vehicle.state()`; publish `SensorGps` (and/or
  `VehicleGlobalPosition`) from the bridge.
- Decide the origin: keep AirSim's default and label it fictitious, or pin one for Town10HD.
- **Verify:** round-trip a known NED point through the geodetic conversion and back to
  <1 m; confirm the topic tracks the aircraft during a flight rather than sitting static.

### S-02 · Bridge IMU, barometer, magnetometer — **done** *(2026-08-02)*

Probed on the running simulator: `settings.json` declares **no sensors at all**, so AirSim
created its multirotor defaults, and IMU, barometer, magnetometer and GPS have been answering
all along while publishing nowhere.

They are real instruments, not ground truth — two IMU reads 0.4 s apart differ by 1.0e-1, so
the noise model is active. That makes them usable for exercising estimator behaviour rather
than merely reporting state.

`px4_msgs` already builds every message needed, so nothing new is generated:

| source | topic | message |
|---|---|---|
| IMU | `/fmu/out/sensor_combined` | `SensorCombined` |
| barometer + environment | `/fmu/out/vehicle_air_data` | `VehicleAirData` |
| magnetometer | `/fmu/out/vehicle_magnetometer` | `VehicleMagnetometer` |
| GPS | `/fmu/out/sensor_gps` | `SensorGps` |

One RPC call returns all four — four separate calls would quadruple the round trips for data
that is read from the same client in the same millisecond.

- **Verify:** publish rates hold under a flight, and the image path does not regress —
  re-measure RGB+depth afterwards.

### S-02b · LiDAR via CARLA, and a config-driven sensor list — **done** *(2026-08-02)*

**Chosen:** `sensor.lidar.ray_cast_semantic` — a point cloud where every point carries the
object id and semantic tag of what it hit. Strictly more than depth for obstacle work, and
the natural instrument for the E-02 avoidance scenario.

**The sensor list becomes configuration, not code.** A YAML list declares which CARLA sensors
to spawn, their attributes, and their offset from the aircraft; the sidecar reads it at startup
and spawns what is enabled. Adding a radar or an event camera later is then a config edit, not
a patch.

> **Superseded — see C-01.** This shipped as its own file, `configs/sim/carla_sensors.yaml`,
> on the argument that AirSim's `settings.json` is read by the simulator at launch while these
> are CARLA actors spawned afterwards, so one file would imply one reader. That argument was
> about *rendering*, and it confused rendering with *authoring*: a reader can still get its own
> generated file. The sensor list now lives in the `sensors:` section of `configs/testbed.yaml`
> alongside the AirSim ones, with a `source:` field carrying the distinction that used to be
> carried by which file you were in.

Following reuses what the chase camera proved: a free-floating actor whose transform is
rewritten each tick from the aircraft's NED pose. One follow thread drives the chase camera
and every configured sensor from a single pose read, rather than one thread each.

**Verified.** PointCloud2 on `/sensors/lidar/points` at 10.6 Hz with semantic tags intact
(Building 34.9%, Vegetation 2.2%, Static 0.8% over the plaza) and object ids preserved. Cost to
the image path: **none measurable** — 6.494 Hz with lidar on against 6.324 off, run-to-run
variance larger than the effect, versus 28% for the AirSim sensors at 20 Hz. The CARLA route
was the right call.

**`points_per_second` is rays CAST, not points returned**, and the binding constraint is
`range` combined with `lower_fov`. Measured:

| position | AGL | points/msg |
|---|---|---|
| plaza | 107 m | 390 |
| plaza | 67 m | 735 |
| plaza | 47 m | **1584** |
| offshore, no buildings | 67 m | 443 |

The geometry explains it exactly. With `lower_fov: -50` and a level sensor, the steepest
available ray is 50 degrees below horizontal, so the slant range to the ground is
`AGL / sin(50)`. Against `range: 80`:

    107 m AGL -> 140 m slant   ground unreachable
     67 m AGL ->  87 m slant   ground unreachable
     47 m AGL ->  61 m slant   ground in range

**The ground is only visible below about 61 m AGL.** Above that the lidar sees buildings and
nothing else — which is fine for obstacle avoidance and useless for terrain. Worth knowing
before trusting it in a scenario: `avoid_the_block` flies at **120 m AGL**, where this sensor
sees the tower and no ground at all.

**Two robustness fixes came out of this.** At 120000 points/second CARLA's own RPC began
timing out at 30 s and `carla::client::TimeoutException` escaped as an uncaught C++ exception
that terminated the sidecar mid-flight; the config is now 16 channels at 30000 points/second,
and `CarlaSensorRig.follow` catches broadly rather than `RuntimeError` — which would never
have caught it.

**Known limitation:** `drain()` is destructive, so the lidar supports exactly ONE consumer. A
second reader steals arcs from the first rather than seeing the same data.

- **Still open:** whether a wider `lower_fov` (or pitching the sensor down) is a better answer
  than a longer `range` for terrain work. Both cost rays.

`getLidarData` throws an RPC error today, and the reason is configuration rather than absence:
AirSim auto-creates only IMU, barometer, magnetometer and GPS for a multirotor. LiDAR and the
distance sensor exist only if declared in a `Sensors` block in `settings.json`, which is read
**at startup** — so enabling it costs a simulator restart, and the point cloud then travels the
AirSim RPC path that is already this project's bottleneck for depth.

**CARLA offers a way around that.** This build exposes 25 sensors including
`sensor.lidar.ray_cast` and `sensor.lidar.ray_cast_semantic`. The chase camera already proved
the pattern — a free-floating CARLA sensor that follows the aircraft by writing a transform
each tick — and CARLA sensors render in-process and push asynchronously, so they never queue
behind a `simGetImages` call.

- **Verify:** points per second delivered, and whether RGB+depth capture regresses. Measure
  both routes before choosing; the AirSim one may still win on fidelity to a real airframe.

### T-03 · Pedestrians spawn but do not move — **done** *(2026-08-03)*

> Marked open until 2026-08-04 although the fix had landed: walkers are steered directly from
> `tick_watchdog` because `controller.ai.walker` is inert in this build (0/8 moved under it,
> 6/6 under `WalkerControl`). See `sim_bridge/carla_air/world.py`.

Spotted by the operator in a 40 s recording: 35 pedestrians in frame, not one of them
walking. Cars drive normally.

**Measured, not inferred.** `/sim/traffic_stats` now reports `walkers_moving` and
`controllers` precisely so this is countable:

    spawned=5 moving=4 walkers=24 walkers_moving=0 controllers=24

Twenty-four walkers, twenty-four AI controllers, **zero movement over 8+ seconds**.

**Ruled out, each by measurement:**

| suspect | finding |
|---|---|
| controllers never attached | `controllers=24` — one per walker |
| `start()` / `go_to_location()` never called | both are called for every controller (`world.py`) |
| max speed left unset | **added** `set_max_speed(1.0-1.7 m/s)`, which CARLA's own `generate_traffic.py` does and this did not. Made no difference. |
| navigation mesh missing from the build | `Content/Carla/Maps/Nav/Town10HD.bin` is present |
| `get_random_location_from_navigation()` failing | returns varied valid points — 24 walkers spawned at 24 of them |
| velocity reporting broken for kinematic actors | switched the metric to **displacement** between calls. Still zero, so they are genuinely stationary. |

**Prime remaining suspect: the client/server version mismatch.** The sidecar warns on every
connect and it has been treated as noise:

    Client API version     = aa9c92b
    Simulator API version  = adaf011-dirty

CARLA-Air is a fork. If its server's walker-controller RPC has drifted from the shipped
client, `start()` and `go_to_location()` would be accepted and do nothing — which is exactly
the observed behaviour, and would explain why every client-side check passes.

**Why it matters beyond the footage.** `traffic_walkers` is a scenario parameter, and every
episode has been requesting 19-20 of them. Stationary pedestrians are street furniture: they
change what the camera sees but not how the scene behaves, so any claim that a scenario has
"live traffic" is currently only half true. The E-01b numbers are unaffected (nothing was
scored on pedestrian motion) but the scenario descriptions overstate what is happening.

- **Next:** try a walker driven directly by `apply_batch`/`WalkerControl` instead of the AI
  controller. If manual control moves them, the AI controller is the broken half and the
  navigation server is the thing to look at; if it does not, walker physics in this build is.
- **Verify:** `walkers_moving` is a substantial fraction of `walkers` for 30 s, and it is
  visible in a recording.

### S-03 · Segmentation is published but disabled — **done, now on by default** *(2026-08-04)*

**Decision: ship it on.** A simulator should publish its sensors. Switching one off to protect
a navigation loop is a navigation decision, and navigation is out of scope — so the default
belongs on, with a flag for anyone who wants the throughput back.

Re-measured rather than trusting the "~77 ms per capture" note, A/B by toggling
`publish_segmentation` at runtime on the shipped config:

| | rgb | depth | segmentation |
|---|---|---|---|
| off | 5.54 Hz | 8.04 Hz | — |
| on | 5.92 Hz | 6.75 Hz | 6.86 Hz |

**~16% off the depth rate, RGB unaffected.** Data verified good: 7 distinct classes over the
plaza at 320x240 — road, buildings, vegetation, vehicles. (A first check showed only 2, which
was the aircraft still sitting at the AirSim origin, offshore, looking at sea and sky. Placing
it over the city was the difference, not any change to the sensor.)

**A faster machine will not buy the 16% back**, and that is worth stating because it is
counter-intuitive: this path is **transport-bound, not render-bound**. The cost is marshalling
the buffer through msgpack-rpc across the 3.10/3.12 interpreter seam, and the GPU never
touches it — `docs/architecture.md` measured a 640x480 depth grab at ~3.2 s *with or without a
GPU*. Only a smaller buffer helps. The same reason RGB went 640x480 -> 960x720 earlier today
with no rate change at all.

- **Consequence, flagged not hidden:** the flight loop now gets depth at ~6.8 Hz instead of
  ~8.0. E-01b's numbers were measured with segmentation off, which is one more reason its
  table should not be quoted as reproducible.
- `tests/test_config.py` caught `docs/guide.html` embedding a now-stale copy of the config,
  which is the guard working as designed.

---

## Evaluation

### E-01c · Re-baseline after D-03 — **done** *(2026-08-05)*

E-01b was measured while `reset()` was landing the aircraft up to 32 m below its commanded
altitude (D-03), so every episode in it started somewhere the scenario had not asked for. Re-run
in full after the fix: 40 episodes, 5 seeds x 4 scenarios x 2 backends, zero collisions.

| backend | scenario | E-01b (2026-08-03) | E-01c (2026-08-05) |
|---|---|---|---|
| oracle | `cross_the_plaza` | 5/5 | **5/5** · 18.3 m |
| oracle | `follow_the_avenue` | 5/5 | **5/5** · 18.3 m |
| oracle | `rain_descent` | **4/5** | **5/5** · 14.6 m |
| oracle | `avoid_the_block` | 0/5 | 0/5 · 69.9 m *(by design)* |
| oracle | **all** | **14/19** | **15/20** |
| geometric | **all** | 0/20 | **0/20** — exact |

Two things the fix bought, both small and both real:

- **`rain_descent` is back to 5/5.** E-01b's loss was `model_declared_done` — the oracle
  stopping short, which is what a wrong start altitude produces.
- **The denominator is 20 again.** E-01b lost an episode to a missed deadline, hence 14/19.
  Nothing was lost this time.

`geometric` reproduces **exactly** at 0/20, every failure `max_steps`, which is the control:
the fix did not simply make everything easier.

- **Caveat kept:** one pass per cell. The non-determinism itself is fixed and measured —
  `cross_the_plaza` seed 1 went from 12/17 on repeat to 16/16 — but a single cell here is
  still a sample, not a rate.
- Raw: `out/sweep-20260805-080540/summary.json`.

### E-01b · Re-baseline after the reset change — **done** *(2026-08-03)*

The first 40-episode sweep to run end to end since 2026-08-01. Zero sidecar deaths, zero node
deaths, zero collisions. `out/sweep-20260803-173125/summary.json`.

| | 2026-08-01 | 2026-08-03 |
|---|---|---|
| geometric | 0/20 | **0/20** — exact |
| oracle | 15/20 | **14/19** |

Oracle per scenario: `cross_the_plaza` 5/5 (median 18.4 m), `follow_the_avenue` 5/5 (19.3 m),
`rain_descent` 4/5, `avoid_the_block` 0/4 — the last being the scenario working, since a
154 m tower sits on the straight line.

**The headline reproduces:** the oracle solves the three open scenarios, the geometric
baseline solves none, nothing collides. Decoupling the VLM (R-02) did not change flight
behaviour.

**THESE NUMBERS SUPERSEDE 2026-08-01 AND ARE NOT COMPARABLE WITH IT.** The reset change moved
the measurement surface: start-pose error dropped from ~9 m to ~3 m, because the aircraft is
now placed rather than flown in and settled. Do not quote the two sets side by side.

**Two anomalies, recorded rather than smoothed:**

- **`avoid_the_block` produced 4 results, not 5.** Seed 5 started and no result landed before
  the deadline, so the oracle denominator is 19. Worth finding before anyone trusts a count
  from that scenario.
- **`rain_descent` went 5/5 -> 4/5**, failure mode `model_declared_done` — the oracle stopped
  early rather than timing out. Most likely the tighter start poses, showing up first in the
  scenario with the smallest margin (median final 14.8 m against a 20 m radius).

### E-01 · Turn single-seed markers into success rates — **done** *(2026-08-01)*, superseded by E-01b

40 episodes, 5 seeds x 4 scenarios x 2 backends, on the 5060 Ti. **oracle 20/20 (100%),
geometric 0/20 (0%), zero collisions.** Every geometric failure was `max_steps` — it wanders
rather than crashing. Oracle final distances span 13.7–19.8 m, so the harness is not the
noise floor. `scripts/run_sweep.sh` reproduces it; see the 2026-08-01 sweep worklog.

Also exercised, forty times each, two paths that had previously run once: bearing-only
grounding and the three-way AirSim client split. Neither stalled.

**The `avoid_the_block` row of that sweep no longer stands** — E-02 re-sited that scenario on
2026-08-02 and the oracle now scores 0/5 on it by design. The other three scenarios are
unchanged and their numbers still hold.

### E-02 · `avoid_the_block` does not test what its name claims — **done** *(2026-08-02)*

Re-sited against a real building footprint. **The oracle went 5/5 → 0/5**, which was the
agreed signal that the obstacle is real.

The cause was not siting but **altitude**: all four scenarios flew at 72–107 m AGL, above
every rooftop in Town10HD. NED altitude is not AGL — the AirSim origin sits 27.45 m above the
street, so the old `z = -50` was 77.45 m over the ground. The aircraft was flying over the
entire city.

Now: NED (210, −230) → (210, −340) at 120 m AGL, blocked by `ProceduralBuilding_94`, a 30 × 30 m
tower with a 154.3 m roof. Contact 41.8 m in, 33.9 m of solid on the line, ~15 m of lateral
detour, and **going over needs NED altitude 126.8 — above the controller's own 120 m clamp**,
so the vertical escape is closed by the envelope rather than by luck.

Tooling: `scripts/survey_buildings.py` — `--check` (does a straight line already solve this
scenario?), `--route` (A* proof that the goal is reachable, and what the detour costs),
`--propose` (search for legs a straight line cannot solve), `--top`. 23 offline tests.

Note the rule collision this created, now written into
[`.ai/AGENTS.md`](../.ai/AGENTS.md#verifying-changes): the oracle is a straight-line policy,
so it fails an obstacle scenario *by construction* and can no longer serve as that scenario's
validator. `--route` is the replacement proof.

### V-01b · Local vLLM on GPU 1 — **out of scope** *(2026-08-04)*

> Model choice. Kept for the reasoning, not as work to pick up here — see the scope table
> at the top of this file.

The other half of the V-01 fork. It needs no API credentials and no per-call cost, which is
the deciding factor if the operator has a Claude.ai subscription rather than API access —
those are separately billed products (see V-01).

**GPU 1 (RTX 5060 Ti, 16 GB) is idle by design** — the simulator renders on it only during a
sweep, and the project rule reserves it for inference. 16 GB fits a quantised 7B-class VLM
comfortably.

The backend contract is the same narrow one, so this is a sibling of `backends/claude.py`
rather than a rewrite: image and instruction in, pixel out. Two things carry over directly and
are worth keeping — a **schema-constrained reply** (vLLM supports guided decoding, so the
pixel can still arrive as an integer rather than something regexed out of prose), and
**clamping `u`/`v` on our side**, since a schema cannot express numeric bounds.

- Serving has to be up before anything works — unlike the API backend, that is a second
  process to supervise and a second thing that can be down mid-sweep.
- **Verify:** same 5 seeds x 4 scenarios as E-01, plus p50/p95 decision latency. A local model
  changes the latency budget in both directions — no network round trip, but no server-side
  batching either — so re-check that a decision still fits the 7.5 s per-step budget before
  reading anything into the success rate.

### E-06 · The sidecar dies mid-sweep — **done** *(2026-08-03)*

Two attempts at the 40-episode sweep on 2026-08-03, both killed the same way:

    terminate called after throwing an instance of 'carla::client::TimeoutException'
      what():  time-out of 30000ms while waiting for the simulator

CARLA raises this from a C++ thread where nothing catches it, so it calls `terminate()` and
takes the entire sidecar with it. The ROS side sees only `[Errno 32] Broken pipe`, which
names neither the cause nor the process that died.

**Attempt 1** reached 2 of 20 oracle episodes before dying on the third `reset`, then ground
on for 18 more episodes against a dead sidecar producing empty log files.
**Attempt 2**, with the timeout raised 30 s -> 120 s, still died — so the simulator is wedging
for over two minutes, and this is not a too-tight timeout.

**Two fixes landed anyway, because both were real:**

- `SimBridge.CARLA_TIMEOUT_S` is 120 s (was 30 s). Verified the constructor takes it —
  `main()` passes no override. It did not prevent the crash, but 30 s was too tight
  regardless: a healthy call never reaches the ceiling, so raising it costs nothing.
- `run_sweep.sh` now checks the sidecar socket before each scenario and abandons that
  backend loudly instead of producing an hour of empty logs.

**Diagnosed 2026-08-03.** Six controlled runs, one variable at a time:

| run | change | result |
|---|---|---|
| A | baseline, 5 seeds | died at seed 2 — `reset: IOLoop is already running` |
| B | `reset` on its own client | seeds 1-3 passed, seed 4 timed out |
| C | `--no-chase` | died at seed 2 — **worse**, so recording is not the cause |
| D | lidar disabled | died at seed 2 — not the cause either |
| E | **six resets, nothing else running** | 26.8 s -> 60.1 s -> **hung** |
| F | E again, polling suspended during reset | no hang; resets 3-6 steady at ~30 s |

**Run E is the finding.** No VLM, no episode, no offboard target — just `reset` repeated, with
only this bridge's own polling alongside it. AirSim's `reset()` tears the vehicle down and
rebuilds it, and every RPC arriving during that window competes with it: 20 Hz odometry,
8 Hz images, 5 Hz sensors, 10 Hz lidar, 1 Hz world tick. The sweep was never the problem; it
was just the first thing to call `reset` forty times.

**Three fixes, each verified:**

1. **`reset` was racing telemetry on one msgpack-rpc connection** — it drove `self.vehicle`
   (the telemetry client) under `slow_lock` while FAST `state` drove the same socket under
   `fast_lock`. Now on `self.control`, in the CONTROL class. This is the **fifth** instance of
   "lock classes guard dispatch, not sockets", so the rule is now a test —
   `tests/test_sidecar_locks.py` parses `server.py` and asserts every method dispatches under
   the lock owning the client it touches. It immediately found **two more latent instances**
   (`describe` and `ground` both drive the media client from the slow class); `ground` takes a
   real depth capture, so that one was a live race.
2. **The bridge now suspends its own polling while a reset is in flight** (`_resetting`,
   cleared in `finally` so a failed reset cannot mute the graph permanently). This is what
   removed the hang.
3. **A malformed reply killed the whole node.** Every timer callback guarded its RPC and then
   indexed the reply *outside* the guard, so one error-shaped dict raised `KeyError:
   'position'` — and rclpy does not catch callback exceptions, so the executor propagated it
   and `bridge_node` exited(1) mid-run. The guards now cover the unpacking too.

**Residual, and the reason this is still open:**

- The **first one or two resets after bringup still fail** with a ~60 s timeout, then it
  settles. Something is not ready when the graph reports ready.
- A reset costs **~30 s steady-state**, which at 40 episodes is 20 minutes of pure setup.
- There is a **60 s** timeout in play that is neither the 120 s CARLA ceiling nor anything in
  this repo — most likely msgpack-rpc's own default. Worth finding.

**Narrowed to one call, 2026-08-03.** After the async-correlation work (below), the progress
instrumentation named the slow step. `reset` announces `"sim-reset"`, calls AirSim's
`client.reset()`, then announces `"placing"`. On the second reset the first frame arrives at
t=0 and **nothing follows for 60 s** — so execution is inside that one AirSim call.

    reset 1:   5.5 s   ok
    reset 2:  60.0 s   no progress after "sim-reset"
    reset 3:  60.0 s   same, then the sidecar died

**AirSim's `reset()` is pathologically slow on repeat.** Not the flight back (we teleport
now), not CARLA, not the socket layer.

**The way out is probably to stop calling it.** `simSetVehiclePose` already does the
positioning. `reset()` remains only for three side effects, and each may have a cheaper
equivalent:

| side effect | possible replacement |
|---|---|
| clears the latched collision flag | unknown — needs checking, and it is the load-bearing one |
| cancels in-flight commands | a `hold()` / `cancelLastTask()` before repositioning |
| disarms and drops API control | explicit `armDisarm(False)` + `enableApiControl(False)` |

If all three can be done without `reset()`, the pathological call leaves the episode path
entirely rather than being waited on. **That is the next experiment**, and it is cheap: six
resets with `reset()` removed, watching whether the collision flag survives a crash.

**What the async work fixed along the way** (kept, and worth keeping regardless):
`bridge_node` deaths went to **zero**; failures are named (`no reply and no progress for
60s`) rather than `KeyError`/`Broken pipe`; a dead sidecar is detected in **0.0-0.8 s**
instead of a full timeout; and stream desync is impossible by construction. Five offline
regression tests in `tests/test_rpc_correlation.py`, and the old design **hangs** against
them. Written up in [`rpc-path.html`](rpc-path.html).

**SOLVED. `client.reset()` was the entire problem.**

`reset` no longer calls it. What it was actually being used for is done explicitly:

| side effect | replacement |
|---|---|
| cancel in-flight commands | `cancelLastTask()` |
| drop API control so the teleport is not fought | `armDisarm(False)` + `enableApiControl(False)` |
| clear the latched collision flag | **a collision epoch** |

The collision one is the substantive change. AirSim latches `has_collided` until a full sim
reset, which made that minute-long call load-bearing **for scoring**. `Vehicle` now snapshots
`time_stamp` at reset and reports a collision only if newer. That is better than clearing:
the epoch is explicit and per-vehicle, not a side effect of a global operation. The epoch is
taken AFTER settling, because a hard reset restarts sim time and an epoch captured before it
would sit in the future and mask every real collision.

The old path survives as `hard=True`, for when the simulator is already misbehaving.

**The same six-reset harness across the whole investigation:**

| run | setup | resets | outcome |
|---|---|---|---|
| E | fly-in, polling live | 26.8s -> 60.1s -> **hung** | 0/6, sidecar died |
| F | polling suspended during reset | 60, 59, 30, 27, 32, 27s | 4/6 |
| G | teleport instead of flying | 5.5, 5.6, 60, 60, fail, fail | 2/6, sidecar died |
| **H** | **no `client.reset()`** | **2.8, 2.9, 2.8, 2.8, 2.9, 2.9s** | **6/6**, zero deaths |

**~10x faster and stable.** Collision epoch verified end to end: clean after reset, flew the
aircraft into the ground and it was detected as `Road_Road_Town10HD19`, then a soft reset
cleared it — all without `client.reset()`.

**Everything else found on the way stayed**, because each was independently real: `reset`
moved off the telemetry client (the 5th "lock classes guard dispatch, not sockets"), polling
suspended during reset, timer callbacks guarding their unpacking, and the async RPC
correlation. None of them fixed this on their own; the last one is what produced the
progress instrumentation that named the culprit.

**Not yet known:**

- **Contention is a suspect, not a conclusion.** Both runs happened while the operator's
  GPU 0 workload sat at 86-88% with a load average around 4. The simulator renders on GPU 1
  and was unaffected on VRAM, but CARLA's RPC is CPU-side.
- **What the third reset does differently** from the first two. It is reproducible at that
  point, which is a strong lead.
- Whether a `--no-chase` sweep survives — 30 fps of 720p H.264 encoding runs on the same box
  and is the most obvious load that a single-episode smoke test does not exercise.

- **Verify:** 40 episodes complete with no `terminate` in `out/sim_bridge.log`, and the
  result matches the E-01 baseline (oracle 15/20, geometric 0/20, zero collisions).

### E-02b · The other three scenarios still have nothing in the way — **out of scope** *(2026-08-04)*

> Scenario design as a policy challenge. Kept for the reasoning; not work for this
> repository. It remains true that three of the four benchmark scenarios are
> straight-line solvable, which is worth knowing when reading any score from them.

`--check` reports `cross_the_plaza`, `follow_the_avenue` and `rain_descent` as CLEAR: a
straight line solves all three, and the 100% oracle rate on them measures the harness, not the
scenarios. That is honest as a baseline and thin as a benchmark.

~~Deliberately left alone for now — changing them would invalidate the E-01 numbers~~
**Unparked 2026-08-03.** That hold was on the E-01 numbers, and E-01b has just replaced them
with a fresh sweep. Changing the scenarios now costs one re-run, not a lost baseline — so
this is the item that actually moves the research question, since three of four scenarios are
still solvable in a straight line and therefore test the harness rather than navigation.

- **Verify:** same as E-02 — the oracle's rate drops, and `--route` shows a detour ratio
  under ~2x so the scenario still matches its own instruction.

### S-04 · Close the ROS command surface: takeoff, land, attitude, waypoint — **done** *(2026-08-03)*

Auditing what a ROS-only client can actually do today found three gaps. Sensors are fully
readable, but **commanding is not** — takeoff and land exist only as sidecar RPC methods
reachable from the 3.10 side, and there is no attitude channel at all. A node written against
this graph could read everything and could not take off.

| capability | before | after |
|---|---|---|
| takeoff / land | sidecar RPC only, invisible to ROS | `/fmu/in/vehicle_command` |
| roll / pitch / yaw | **absent** | `/fmu/in/vehicle_attitude_setpoint` |
| waypoint | `/vlm/grounded_waypoint`, the VLM's own channel | `/fmu/in/trajectory_setpoint` with a position |
| sensors | already complete | unchanged |

**PX4 messages, not invented ones.** `VehicleCommand` with `VEHICLE_CMD_NAV_TAKEOFF` and
`NAV_LAND` is exactly how a real Pixhawk is commanded over uXRCE-DDS, and
`VehicleAttitudeSetpoint` carries a quaternion `q_d` rather than three Euler angles for the
same reason. Inventing a friendlier `/testbed/takeoff` would break the property this whole
shim exists for: that a node written here ports to hardware by deleting one node.

- **Verify:** a standalone ROS 2 node — no sidecar import, no carla, no airsim — takes off,
  flies a waypoint, holds an attitude, lands, and prints every sensor stream. If it needs
  anything from the 3.10 side, the surface is not closed.

**Done, and the verification is the interesting part.** `examples/ros2_full_control.py`
imports only `rclpy`, `px4_msgs` and `sensor_msgs`, and does all five. Two things only a real
run could have found:

- **The autonomy loop fights a manual command.** A takeoff to 35 m settled at 15.6 m, because
  `offboard_control` is still publishing its own setpoints at 10 Hz and wins. The example now
  disables that node via `SetParameters` on entry and restores it on exit. Nothing about the
  message surface was wrong; the *graph* was.
- **Attitude signs were inverted for two of three axes.** Commanded roll +12 deg came back
  +12.0, pitch +15 came back **-15.0**, yaw +40 came back **-40.0**. Fixed in
  `Vehicle.attitude()` by negating pitch and yaw, with the measurement in the comment.

I had claimed this working off a log tail before either was found. It was not; the operator
asking "is that working?" is what produced the re-test. **Read the numbers, not the fact that
output appeared.**

The tested surface is written up in [`docs/ros2-api.html`](ros2-api.html) — every command and
sensor with its message type, generated only after the calls above were run.

### C-01 · One config file instead of three — **done** *(2026-08-03)*

"Where do I set X" currently needs a read of three scripts. Settings live in
`configs/sim/settings.json` (AirSim's own schema), `configs/sim/carla_sensors.yaml` (ours) and
`ros2_ws/src/bringup/config/testbed.yaml` (ROS's schema) — and **two of those formats are not
ours to change**: the first is read by the CarlaUE4 binary, the second by rclpy's parameter
system.

So: one **source**, `configs/testbed.yaml`, rendered into the formats each reader demands by
`scripts/apply_config.py`. The machinery is half there already — `run_sim.sh` copies
`settings.json` into place and patches it, and the launch file already accepts `params:=`.

**Sensors are the sharpest case.** They are split across two files today purely by which
simulator provides them, which is an implementation detail leaking into the user's head. In
the unified file they are one `sensors:` list with a `source:` field, and the generator routes
each entry.

**Sections are named for WHEN a setting takes effect**, not for which file it lands in:

| section | changing it costs |
|---|---|
| `simulator:` | a simulator restart, ~60 s |
| `sidecar:` | a sidecar restart, ~5 s |
| `graph:` | nothing — live via `ros2 param set` |

That distinction is load-bearing and a flat file would hide it. This project has been bitten
repeatedly by exactly that shape of flattening: `min_altitude_m: 15` reads as 15 m AGL and is
42.45; `points_per_second` reads as points and is rays.

**Done.** `configs/testbed.yaml` is the source; `scripts/apply_config.py` renders
`configs/sim/settings.json` and `ros2_ws/src/bringup/config/testbed.yaml`, both with a DO-NOT-EDIT
header. `run_sim.sh` and `bringup.sh` render before every start, so editing the source is
enough and nobody has to remember a build step. Argument and environment overrides still win,
so a one-off run needs no edit.

Verified with a full bringup and `TESTBED_GPU` deliberately unset: GPU 1 selected from the
config, hardware rendering confirmed, lidar spawned from the unified `sensors:` list
(5022 measurements), all five sensor topics live, ROS parameters matching the source.

`configs/sim/carla_sensors.yaml` is gone — its contents are the `source: carla` entries.

> **Corrected 2026-08-03 — this entry overclaimed, and the flaw survived it.** C-01 said three
> config files became one. It actually left **four**: the renderer wrote to a
> `configs/generated/params.yaml` of its own invention, while
> `ros2_ws/src/bringup/config/testbed.yaml` — the old ROS parameter file — stayed in the repo,
> stayed git-tracked, stayed installed into the package share, and remained the **default value
> of the launch file's `params` argument** (`testbed.launch.py:23`). `bringup.sh` passed
> `params:=` explicitly so the normal path was correct and nothing ever looked wrong; a bare
> `ros2 launch bringup testbed.launch.py` silently read the stale copy. They had already
> diverged — the stale one was missing `recorder.crf: 26`.
>
> Fixed by rendering into the path the launch file already defaults to and deleting
> `configs/generated/` entirely, so there is exactly one parameter file and the bare launch
> command is correct. **Found by verifying the doc, not the code** — the check "does every path
> this document names still exist" is what turned it up, three days after the entry claimed
> done.

13 offline tests, including one that regenerates into memory and diffs, so a stale generated
file fails a test rather than a flight. The rest encode constraints the format cannot: all
camera buffers share one aspect ratio, ClockSpeed is 1.0, the altitude clamp sits above the
NED ground, an unset GPS origin is omitted rather than written as {0,0} (which AirSim would
treat as the Atlantic), and `ros_domain_id` never leaks into the ROS parameter file.

## Tooling

### R-08 · The console becomes a fourth container, opt-in — **done** *(2026-08-10)*

**Decided by the operator 2026-08-10: opt-in.** The console is not part of the stack today and
should be. It is in no image, `stack_up.sh` starts nothing for it, and `--in-stack` merely
borrows the ROS image to spin up an unmanaged `carla-air-webui` beside the three real
containers. Two rough edges follow from nobody owning its lifecycle, and both were hit while
verifying R-03 step 3:

- ~~`stack_up.sh --down` leaves `carla-air-webui` running — it has to be removed by hand.~~
  **Wrong, and corrected 2026-08-10 by testing it.** `down()` has swept every `^carla-air-`
  container since the original containerisation commit (`d5ed0ff`), which includes this one.
  Measured: with the console up, `--down` printed `stopped carla-air-webui` and left nothing
  in `docker ps -a`. The entry asserted a defect that was never there — filed from reasoning
  about the code rather than from running it, which is the failure mode rule 6 exists for.
  The *second* bullet was real, and is the one that mattered.
- A **stale container silently served old code**. `webui.sh --in-stack` failed with `exit 125`
  (name already in use) and the previous container kept answering on the port, so a fix that
  had landed appeared not to work. That cost a wrong diagnosis before the container was found.

**Opt-in, not on by default, and the reason is a rule rather than taste.** `CLAUDE.md` states
that after `bringup.sh`, `ros2 node list` is `/carla_air_bridge` **alone** — the "you bring the
agent" invariant. Since R-03 step 1 the console *is* an `rclpy` node, so starting it by default
makes that two nodes and brings up an HTTP control surface with every stack. That invariant is
one of the few things keeping the scope honest, and a flag is cheap.

- `docker/webui.Dockerfile` — thin, `FROM carla-air/ros:1`, its own entrypoint. The ROS image
  already carries `rclpy`, `cv_bridge` and `msgpack`, so nothing new is installed.
- `stack_up.sh --console` starts `carla-air-webui` as a managed fourth container.
- `stack_up.sh --down` removes it, and starting replaces a stale one rather than failing on the
  name.
- `status.sh` counts it as a container rather than a host process.
- `webui.sh --in-stack` becomes redundant for the stack case; `webui.sh` stays for the host lane.

**Not in scope: retiring the host lane.** "Purely container-based" could also mean deleting
`bringup.sh`, `run_sim.sh` and the `.venv`. That is a much larger decision and a separate item —
the host lane is what every measurement this week was taken against, and it is the fallback when
Docker is unavailable. Containers should become the *default* path, not the only one.

- **Verify:** `stack_up.sh --config ... --console` brings up four containers and the console is
  reachable; `--down` leaves none; starting twice in a row replaces the container rather than
  serving the old one; and without `--console`, `ros2 node list` inside the stack is
  `/carla_air_bridge` alone — the invariant this design exists to preserve, checked rather than
  assumed.

**Done 2026-08-10. All four verify criteria met against a real stack**, not a stand-in:

| criterion | result |
|---|---|
| default bringup keeps the invariant | 3 containers, `ros2 node list` = **`/carla_air_bridge` alone**, nothing on :8080 |
| `--console` brings up four | 4 containers, `GET /` → **200**, `ros2 node list` = bridge **+ `/carla_air_webui`** |
| starting twice replaces a stale one | container `41bb69a400e4` → **`e076e91a0675`**, one container by that name, still serving 200 |
| `--down` leaves none | `stopped carla-air-webui` … and `docker ps -a` shows **no** `carla-air-*` |

**The third row was set up as the actual incident, not as a happy path.** The console container
was deliberately left *stopped but present* — `Exited (137)`, holding the name — before the
second run, because that is the shape that produced the wrong diagnosis on 2026-08-07. A run
that only ever replaces a *running* container would not have exercised it.

**And the console is genuinely on ROS inside the container**, which is the thing most worth
checking because failing it is silent: `/api/status` reported `source: ros` with video 0.16 s
old and state 0.03 s old. The failure mode is not a crash — without `interfaces`/`px4_msgs` the
console falls back to the sidecar socket and opens the second AirSim capture R-03 step 1 exists
to remove, turning a 6.6% cost into 24%. The image's entrypoint therefore *refuses* rather than
degrading, and that refusal has its own test.

**`status.sh` reports both rows, and that is deliberate.** A containerised console shows
`web console 1` *and* `console container 1`, because the process row greps the process table
and this machine's table includes container processes. One console in a container reads 1 and
1; two host consoles read 2 and 0. Narrowing the process row to exclude containers was
rejected: a status screen that checks *less* than `stop.sh` removes is how an orphaned console
listened on the mesh for days.

- **13 tests** in `tests/test_stack_console.py`, no Docker or GPU. Behavioural for the paths
  that exit during parsing; **structural** for the rest, because `--config PATH --console`
  brings up a real simulator and no test may pass it — the same rule `--all` has in
  `test_stop_args.py`. Five were mutation-checked: console-on-by-default, the removed
  pre-emptive `docker rm -f`, dropping `--ipc container:`, moving the image check back after
  step 1, and making the HTTP probe unconditional each turn exactly one test red.

**Self-review before proposing it found three, and one broke the documented example.** The
health probe curled `127.0.0.1:8080` unconditionally while `TESTBED_CONSOLE_ARGS` is documented
— with `--bind netbird` as *the* example — as the way to change that address, so the documented
usage would have made a healthy console burn a 20 s timeout and then print a warning that was
false. The probe now runs only when the address is known, and says what it did not check
otherwise; the container-exited check runs either way. The image check moved above step 1,
because nothing builds that image automatically and finding out at step 4 means a bringup spent
and a half-started stack. `curl` is no longer assumed present.

### T-08 · The console's stop button is gated by "running", not "exists" — **open** *(filed 2026-08-10)*

Found reviewing PR #9 (T-06), and it is the same defect one level up from the one that PR
fixes.

`stop_simulator()` takes the container path only when
`lifecycle.deployment(self.stack_running())` returns `CONTAINER`, and `stack_running()`
(`webui/server.py`) asks `docker ps` — **running containers only**. So when `carla-air-sim`
exists but is *stopped* — a crashed simulator, a `docker stop` by hand — the console falls
through to the HOST branch, runs `run_sim.sh --kill` against a host process that is not there,
and returns `{"stopped": "simulator"}`. Success, having done nothing, with the container object
still present.

T-06's whole thesis is that *running* and *exists* are different questions. It answered that
correctly **inside** the container branch while the check that decides whether to enter the
branch still conflates them.

**The obvious fix is wrong and that is why this is filed rather than patched.** Making
`stack_running()` ask `docker ps -a` would tell the **Start** button that a stopped stack is
up — the console would then refuse to start anything, or aim at a deployment that is not
running. `stack_running()` means "is the stack up", and a stopped container is not an up stack.
The sweep belongs in `stop_simulator` as a question asked *independently* of which deployment
was selected: whatever the deployment, if a `carla-air-*` container object is lying around,
say so.

- **Verify:** with `carla-air-sim` present but stopped, the console's stop button reports the
  container's actual state rather than reporting a host-path success; and the Start button
  still treats the stack as down.

### T-07 · `stop.sh --all` does not clean up the container lane — **open** *(filed 2026-08-10)*

Found while verifying T-06 against a real stack, and left unfixed on purpose because it is a
scope question rather than a bug in a line of code.

Rule 1 says every flight test ends with `./scripts/stop.sh --all` and then `status.sh` showing
every count at 0. In the container lane it does not deliver that. Measured 2026-08-10, after
`stack_up.sh --config …` and then `stop.sh --all`:

    WARNING: 3 process(es) survived TERM and KILL:
       …/sim_bridge/server.py --socket /run/carla-air/sim.sock
       …/ros2 launch bringup testbed.launch.py …
       …/carla_air_bridge/bridge_node …
    stopped: graph and sidecar, simulator stopped (3 stragglers)

    docker ps  →  carla-air-ros    Up
                  carla-air-bridge Up

Two separate wrongnesses:

- **The three "stragglers" can never be killed and are not stragglers.** They are the sidecar
  and the ROS graph running *inside* `carla-air-bridge` and `carla-air-ros`. They are uid **0**
  on the host while `stop.sh` runs as uid 1000, so TERM and KILL both bounce — no amount of
  retrying will remove them. The script escalates, fails, names them, and sets `rc=1`.
- **Two containers are left running.** `stop.sh` only knows the name `carla-air-sim`, so the
  other two survive a teardown that reports `simulator stopped`. They hold no VRAM once the sim
  container is gone (their namespaces are broken with it) — but they are exactly the leftover
  graph rule 1 exists to prevent, and they will collide by name on the next `stack_up.sh`.

**The question is which script owns the container lane**, and that is why this is not a one-line
fix. `stack_up.sh --down` already tears the whole stack down correctly, and duplicating its
`carla-air-*` sweep into `stop.sh` puts the same logic in two places. The options are roughly:
teach `stop.sh --all` to delegate to `stack_up.sh --down` when a stack is present; widen its
container matching to the `carla-air-` prefix as `stack_up.sh` does; or state in rule 1 that the
container lane's teardown is `stack_up.sh --down` and make `stop.sh` say so when it detects one.
The last is the smallest and the most honest, but it changes a rule rather than a script.

- **Verify:** after whichever fix, `stop.sh --all` following a containerised bringup leaves no
  `carla-air-*` container in `docker ps -a`, reports no straggler it cannot act on, and
  `status.sh` shows every count 0 with GPU 1 at ~33 MiB.

### T-06 · Container teardown SIGKILLs where the host path escalates — **open** *(filed 2026-08-08)*

`stop.sh` gives the host simulator `TERM → TERM → KILL` and then **verifies it is gone**, because
Unreal does not always go down on the first TERM and because this script once reported success
twice while 3.5 GB of VRAM was still held (2026-08-03). The container path added in R-03 step 3
does none of that: `webui/server.py` runs `docker rm -f`, which is an immediate SIGKILL plus
removal, and `stop.sh` itself does the same for `carla-air-sim`.

That is harsher and less careful than the path it replaced, for no reason anyone wrote down. It
is not obviously *wrong* — the container is being destroyed either way — but "less careful than
the host path" is a claim nobody made deliberately, and the host path's care was bought with a
real incident.

- **Do:** `docker stop` (SIGTERM, grace period, then kill) before `docker rm`, and verify the
  container is actually gone rather than reporting the attempt. The verification half matters
  more than the signal half — that is exactly what the 2026-08-03 incident was.
- **Do not** bother preserving the container to restart it faster. `carla-air-bridge` and
  `carla-air-ros` join the sim container's network and IPC namespaces, so once it goes away —
  stopped *or* removed — the other two are broken and need recreating. The namespace coupling
  makes the sim container's lifecycle the whole stack's lifecycle.

**Measured 2026-08-10, and the open question is closed: the grace period costs nothing.** The
fix shipped with `docker stop -t 10` verified only against an `alpine sleep` container, which
dies on TERM instantly, so whether CarlaUE4 consumed the full 10 s was unmeasured. Against a
real stack on GPU 1:

| | |
|---|---|
| `docker stop -t 10 carla-air-sim` | **1.175 s** |
| exit code | **143** (128 + SIGTERM), not 137 (SIGKILL) |
| GPU 1 VRAM | 4006 MiB → 32 MiB |

143 is the whole answer: 137 would mean the grace period expired and Docker escalated. Unreal
took the TERM and left in about a tenth of the budget. Two structural facts checked rather than
assumed, either of which would have made the number meaningless — **PID 1 is the Unreal binary
itself**, because `bash -lc` `exec`s a lone simple command rather than interposing a shell that
would eat the signal; and **SIGTERM is caught, not ignored** (`SigCgt: …400144ff`, bit 14 set).
So the host path's "Unreal does not always go down on the first TERM" does **not** transfer to
the container.

**And the second trial found the fix was unreachable.** Running the *shipped* path rather than
the command in isolation — `stop.sh --all` against a real stack — left `carla-air-sim` in
`Exited (143)` and **never removed**, with none of the three new messages printed.

`pkill -x "CarlaUE4-Linux-"` (`scripts/stop.sh`, the escalation) reaches **inside** the
container: its CarlaUE4 runs as the invoking user on the host (`docker top`, UID column) and its
`comm` is exactly `CarlaUE4-Linux-` — the 15-character truncation `-x` matches. That ran first,
so by the time the container block was reached the container had exited, and its guard asked
*is it running*. The graceful stop, the removal and both verifications were **dead code in the
container lane**, reachable only when `pkill` finds nothing — precisely what an `alpine`
container reproduces, which is why the original real-container check passed.

The state it left is the one R-08 was filed about: stopped, holding no VRAM, **blocking the next
start by name**.

- **Fixed 2026-08-10.** The container block moves *ahead* of the `pkill` escalation, and its
  guard becomes `docker ps -a` (exists) rather than `docker ps` (running). Verified against a
  real stack: `stopped container carla-air-sim` printed, container absent from `docker ps -a`,
  VRAM 3744 → 32 MiB, and `docker events` showing `kill → stop → die exitCode=143 → destroy`.
  Two structural tests pin both halves, mutation-checked against the pre-fix script.

**Not a reset path, and worth stating because the two get conflated.** Resetting the simulator
never needs the container touched. The ladder is `/sim/reset_vehicle` (seconds, what episodes
use), then `/sim/destroy_actors` + `spawn_traffic` for world state, then AirSim's global
`client.reset()` — which `vehicle.py` records as costing about a minute, and which this project
deliberately engineered away from by comparing collision timestamps against a per-vehicle epoch
instead of relying on the latched flag only a global reset clears. Container removal is teardown,
not reset.

### T-05 · `stop.sh` obeyed arguments it did not understand — **fixed** *(2026-08-07)*

`./scripts/stop.sh --help` **tore down the ROS graph** instead of printing help. The script
tested `"${1:-}" = "--all"` in three separate places and had no other argument handling at all,
so anything unrecognised — a typo, a guessed flag, `--help` — fell through to the default
teardown. Hit while measuring R-03 step 1: I guessed at a `--webui` flag to stop only the
console, and it stopped the graph, costing a bringup and a restart mid-measurement.

Three defects, one shape:

- **The kill escalation ran before the arguments were read.** By the time an unrecognised flag
  could have been noticed, the graph was already down. Parsing now completes first, so a
  partially valid command line (`--all --bogus`) stops *nothing*.
- **`--all` only worked in position 1.** `stop.sh --foo --all` silently did the default.
- **`ALL=1` was read by the container teardown and by nothing else**, so
  `ALL=1 ./scripts/stop.sh` removed the container, left the host simulator running, and then
  reported `simulator left running`. One flag now, read in all three places — and **not**
  inherited from the environment. The first draft of this fix "unified" it by seeding from
  `${ALL:-0}`, which is worse than the bug: `ALL` is about as generic an environment variable
  name as exists, so an unrelated export in a parent shell would silently escalate a graph
  teardown into SIGKILLing the simulator. Destructive scope comes from an explicit flag, never
  from ambient state. Caught reviewing my own diff before the merge.

This is the class of bug the script exists to prevent — it is the teardown path, where rule 1
says the machine is left clean, and it was guessing at input instead of refusing it.

- **Verify:** 12 tests in `tests/test_stop_args.py`, and they were **checked against the
  original**: all 12 fail on the pre-fix script, all 12 pass after. They are safe to run beside
  a live simulator by construction — the script is copied to a temp directory so its
  `PROJ`-anchored patterns match nothing real, and only the paths that exit *during parsing*
  are executed. `--all` is never passed by a test; it would `pkill` a real `CarlaUE4`, so it is
  pinned structurally instead.

### T-02 · H.264 recording, and cheaper live streams — **done** *(2026-08-06)*

**The sync is fixed.** Three previous attempts failed in flight, each costing a recording, and
the reasons are now measured rather than guessed — reproduced with synthetic frames in
`tests/test_h264_timing.py`, no simulator involved, which is why they survived three tries
made only in flight.

Two causes, both real:

1. **`stream.time_base` alone does nothing.** libav overwrites it while muxing, so hand-set
   PTS end up read against a different base. Measured on a 10.15 s synthetic clip: stream
   time_base only produced a **507 s** file. `frame.time_base` must be set on every frame;
   with it the same clip comes out at 10.20 s.
2. **PTS must be strictly increasing.** Two frames in the same millisecond, or one stamp
   stepping backwards, both raise exactly the recorded failure —
   `av.error.ArgumentError: Invalid argument ... returned 22` — from `close()`, i.e. after the
   whole episode is encoded. Real capture stamps do both routinely. A monotonic guard is what
   makes the approach survivable, and its absence is what beat the first two attempts.

`combine_views.py` no longer rescales a file that is already real-time; it detects that from
the container duration against the recorded span, so old recordings still get the old
correction.

**And a regression this uncovered, which had been silently degrading every episode recording
for two days.** `examples/navigation/run.sh` did not export `vendor/py312`, so when the
recorder moved out of `bringup.sh` (which does) on 2026-08-04, `import av` failed on the ROS
side and `VideoWriter` fell back to **mp4v** — no timestamps, nominal-rate playback, and a file
no browser can play. Silently. The fallback now says so on stderr; a silent fallback on a
measurement path is a bug in the fallback, not in the environment.

Verified on a real flight, all three streams:

| stream | duration | codec | real span | would have been |
|---|---|---|---|---|
| chase | 30.10 s | h264 | 30.05 s | 30.10 s |
| onboard | 26.41 s | h264 | 26.28 s | **15.50 s** |
| depth | 26.41 s | h264 | 26.28 s | **15.50 s** |

The onboard stream was playing 1.7x too fast. That is the drift, and it is gone.

**Why there was no H.264.** Not a missing feature — a licensing boundary. `opencv-python`
bundles its *own* FFmpeg (avcodec 59.37.100) built without `libx264`, because x264 is GPL and
the wheel ships under Apache/MIT. The system has `libx264.so.164` and avcodec 60 installed;
OpenCV simply does not link them. **PyAV** bundles an FFmpeg that does — `libx264`, `libx265`,
`libvpx-vp9` and `h264_nvenc`. Installed into `.venv` and `vendor/py312`, no system change and
no permission needed.

**Measured three times, because the first two benchmarks lied.** The number depends almost
entirely on how much the scene moves, and both of my first attempts measured an easier
workload than the real one:

| benchmark | mp4v | h264 crf26 | gain | why it was wrong |
|---|---|---|---|---|
| frames decoded from an existing mp4v file | 57.9 KB/f | 27.5 | 2.1x | mp4v had already discarded the detail x264 would have had to encode |
| raw frames, **static** camera | 53.7 KB/f | 12.2 | 4.4x | a fixed camera lets inter-frame prediction do nearly all the work |
| **raw frames, camera moving** | **81.9 KB/f** | **36.4** | **2.25x** | this is the chase camera's actual workload |

On the real workload, per 1102-frame episode at 1920x1080:

| encoder | KB/frame | per episode |
|---|---|---|
| cv2 `mp4v` (was) | 81.9 | 88 MB |
| h264 crf23 | 53.4 | 57 MB |
| **h264 crf26 (chosen)** | **36.4** | **39 MB** |
| h264 crf28 | 28.4 | 31 MB |
| h264 crf30 | 22.4 | 24 MB |

CRF is the only lever worth touching — `preset slow` beat `medium` by 2% for 40% of the encode
speed. `h264_nvenc` was no smaller than x264 and would load the GPU rendering the simulator,
so **CPU x264**: ~140 fps encode against a 10 fps capture is ample headroom.

**Size is the lesser win. `mp4v` (MPEG-4 Part 2) does not play in browsers** — H.264 does,
which is what makes a recording viewable in the console instead of only after a download.

- **Verify:** an episode produces playable H.264 for both views; `ffprobe`-equivalent confirms
  the codec; file sizes drop as measured; the recorder still drops frames rather than
  buffering under backlog, and still closes cleanly on a killed sweep.

### T-01 · A web console: start the sim, watch both cameras, fly it by hand — **done** *(2026-08-02)*

Everything here is currently driven by a scripted episode. There is no way to just *look* at
the simulator, or to fly somewhere and see what the camera sees — which is exactly what
scenario design needs, and what turned three flight-test failures into log archaeology.

A single page at `http://localhost:8080`:

- **start / stop** the simulator and the 3.10 sidecar, with live status
- **two live views** — the drone's own camera (what a model would be scored on) and the HD
  chase camera following it
- **manual control** — translate, climb/descend, yaw, hold, reset to a point, land
- **world controls** — spawn clustered traffic, set weather

**Design constraints, each with a reason:**

- **Stdlib `ThreadingHTTPServer`, no new dependency.** `tornado` is importable but only as a
  transitive dependency of `msgpackrpc`, which runs its own IOLoop for the AirSim
  connection. Building the web server on the same library invites the class of bug that cost
  a flight test this morning.
- **MJPEG (`multipart/x-mixed-replace`), not WebRTC or websockets.** It renders in a plain
  `<img>`, needs no client-side decoding, and both sources are ~10 Hz — the frame rate a
  video codec would be buying back does not exist here.
- **Its own UDS connections to the sidecar, one per concern** — streaming and control must
  not share a socket. Same lesson as the chase follower.
- **It captures directly from AirSim, so it contends with the ROS graph.** The image path is
  this project's bottleneck; a web client capturing at 10 Hz while an episode runs would
  halve the rate the model sees. The console is for manual flying and inspection with the
  graph **down**, and must say so rather than silently degrading a scored run.

**Built.** `webui/server.py` + `webui/index.html`. Measured: onboard stream 6.0 Hz, chase
stream 10.0 Hz, both over NetBird; simulator starts and stops from the page; `reset` flies the
aircraft and telemetry follows.

**Remote access** — `--bind netbird` resolves the `wt0` address from `ip` and binds *only*
that interface, so the console is not also exposed on `docker0` or `tap0`. Off loopback a
token is generated automatically and required on every `/api` and `/stream` request; the URL
carrying it is printed and written to `out/webui-url.txt` (mode 0600). Verified: 401 without a
token, 401 with a wrong one, 200 with the right one, including on the streams. `--no-token`
exists for the deliberate case and says what it costs.

The token travels as `?k=` rather than a header because MJPEG is rendered by an `<img>`, and
an `<img>` cannot send an Authorization header.

**Controls** are both in the sidebar and as a touch-friendly pad overlaid on the chase view
(toggleable), so the page is usable from a tablet or phone over the mesh. Velocity commands
carry their own duration — a press is a nudge and the aircraft stops itself. A page that
*must* deliver a "stop" is one dropped request away from an aircraft that never gets one.

Two stops: **Stop simulator** (`run_sim.sh --kill`, matches the process name) and **Stop
everything** (`stop.sh --all`).

### E-05 · Record every flight test, from both views — **done** *(2026-08-02)*

> **Filed after the fact, which is the wrong order.** The "plan first" rule at the top of this
> file exists so a reader can see what was intended before it was built; writing the entry
> afterwards turns it into a changelog. Recorded here rather than quietly backdated.

An episode used to leave a JSON and nothing to look at. `141.9 m from goal` says nothing about
*why*, and the first real VLM flight made that expensive: diagnosing a 40 m descent meant
reading the offboard node's target log and reconstructing the geometry by hand. Two recorders
now run, and the failure was obvious on video within seconds.

| | resolution | shows |
|---|---|---|
| `out/videos/<episode_id>.mp4` | camera-native (640x480) | what the model was shown, with its annotation crosshair, confidence, latency and its own rationale |
| `out/chase/<episode_id>.mp4` | **1920x1080** | the aircraft in the world, from an exterior camera that follows it |

Both are on by default — `record:=false` for the onboard one, `--no-chase` for the exterior.

**Neither may cost a flight, and neither may perturb the measurement.** The onboard recorder
*subscribes* to `/camera/rgb/image_raw` rather than capturing its own frames, so it adds no
simulator load and records exactly the pixels that were scored. The chase camera is a **CARLA
sensor**, not an AirSim capture — AirSim's image path is the project's bottleneck and a 1080p
grab there would contend with the frames the model needs; a CARLA `sensor.camera.rgb` renders
in the same UE4 process and pushes frames asynchronously. Measured: 1102 frames at 1920x1080,
**0 dropped**, with the model flying normally.

- **Verify:** an episode produces both files; they play; the chase file's `dropped` count is 0;
  a killed sweep still leaves the final episode playable. All confirmed.
- **Known limit:** there is no H.264 encoder on this box (no system ffmpeg, and OpenCV's build
  has no libx264), so recording falls back to `mp4v` at roughly 3-4x the size. ~80 MB per
  episode across both views, ~1.6 GB for a 20-episode sweep. Installing ffmpeg in the
  container would fix it and is a container change, so it has not been done.

### T-04 · One command from nothing to a video — **done** *(2026-08-04)*

> **Filed after the fact.** Same wrong order as E-05, and recorded rather than backdated: the
> script was written and run before this entry existed.

Producing one demo video took five commands across three terminals — `bringup.sh`,
`vlm_navigation/run.sh`, `run_episode.sh`, `combine_views.py`, `stop.sh --all` — and the
*fifth* is the one that gets forgotten, which is exactly the failure rule 1 exists to prevent.
A leftover graph stacks on the next bringup and two controllers fight over the aircraft while
`ros2 node list` still looks correct.

`scripts/demo.sh` runs the sequence and tears it down from an `EXIT INT TERM` trap, so a
Ctrl-C mid-flight still stops the simulator instead of leaving 3.3 GB of VRAM held. It waits
on the `hardware rendering confirmed` line rather than sleeping a fixed interval, so it cannot
silently hand you a software-rasterised run (rule 5). It sets no `ROS_DOMAIN_ID` of its own —
each child already exports `${TESTBED_ROS_DOMAIN_ID:-42}`, and a second place to set it is a
second place for it to be wrong.

- **Verify:** one end-to-end run producing a playable file, then `status.sh` clean. Done
  2026-08-04: `street_level` seed 5, claude backend → `out/demo/street_level-s5-a18931.mp4`,
  1920x1080, 703 frames, 35.1 s, 16.3 MB, chase 0 dropped. Afterwards all process counts 0,
  GPU 1 back to 33 MiB, no sim ports.
- **Not a navigation result.** That run scored 0/1 (`model_declared_done`, 106.5 m short after
  8 steps) — one seed, consistent with the street-level failures already recorded, and not a
  success rate. It says the wrapper works, nothing about the model.
- **Known limit:** the composited output inherits the unresolved chase/onboard sync (T-02);
  `combine_views.py` rescales from the `.timing.json` sidecars, which reduces the drift but has
  not been shown to remove it.

Auditing this against `.ai/AGENTS.md` and then interrupting a live run turned up three defects,
all of them in the teardown path rule 1 is about:

- **`demo.sh` ran `stop.sh --all` but not `status.sh`** — the half that reports what was
  *signalled*, not what is actually *left*. Those disagree exactly when something ignores TERM.
- **SIGINT tore the stack down and then continued**, flying step 3 against a simulator that no
  longer existed and running the teardown twice. `EXIT` now does the work; `INT`/`TERM` exit
  130 and fall through to it, with a `CLEANED` guard.
- **`status.sh` checked GPU 0 unconditionally** while the shipped config renders on GPU 1, so
  it warned SOFTWARE RENDERING on every correct run — and, with the cards reversed, would have
  stayed silent through a real one. It now resolves the same target `run_sim.sh` does. Details
  in `docs/worklog/2026-08-04-one-command-demo.md`.

### D-01 · The same seed does not give the same result — **closed as out of scope** *(2026-08-04)*

> **This closure was PREMATURE and is under review** *(2026-08-04, same day)*. See D-03: the
> aircraft does not reliably start an episode at the altitude the scenario commands — measured
> across ten real episodes, four began 18-37 m low. **The single failing run of that batch,
> `run9`, was the worst outlier at +37.5 m**, starting 45 m above the street instead of 82.5.
> A start pose that far off changes what the camera sees, and "the goal is outside the field
> of view" is exactly the mechanism recorded below.
>
> **Re-measured after D-03 was fixed: 16 of 16** (9, then 7 more while verifying D-04), with
> start-pose errors of 1.4-5.7 m where four of the previous ten had started 18-37 m low.
> P(16 of 16 | the pre-fix rate of 20/27) = **0.008**. At n=9 this was suggestive; at n=16 it
> is hard to explain as a streak, and D-03 looks like it was the substance of D-01.
>
> Still not a closed case: all 16 are **one scenario and one seed**. `avoid_the_block` and
> `follow_the_avenue` have not been re-run at all, and the E-01b table was measured against
> the broken reset. Nobody should quote 16/16 as *the* rate — it is the rate for
> `cross_the_plaza` seed 1.

**Closed, not fixed, and not because it stopped mattering.** The mechanism was found and it
lives in the navigation stack, which the scope agreed the same day puts outside this
repository. The simulator's own contribution to it was D-02, and that is fixed.

**What anyone building a navigation stack here needs to know**, because it will happen to them
too: the control path is a closed loop — waypoint sets velocity, velocity sets heading, heading
aims the camera, the camera decides the next annotation, the annotation becomes the next
waypoint. With enough gain that loop oscillates. When it does, the aircraft's heading swings,
the goal leaves the field of view, and whatever is annotating gets clamped to the frame border
— measured at exactly u=0 and u=959 on a 960 px frame, alternating sides. The result is a
lateral zigzag at roughly twice the necessary path length, ending in `max_steps`.

Rate on the reference case: **~70-80%** on a scenario documented at 100%
(12/17, then 8/10 after D-02). Bimodal — a run either goes straight in at ~13 steps or
zigzags to 25. Nothing in between.

**Four hypotheses were tested and refuted**, each by measurement, and they are worth listing
because each looked convincing:

| hypothesis | how it died |
|---|---|
| `bearing_only` flies blind | the successful runs were **100%** bearing-only; the failure had *more* depth-valid waypoints |
| the projection lags the camera pose | `--split` showed the annotation pixel oscillating and the projection following it faithfully |
| the velocity or yaw slew drives it | each disabled by A/B, no change |
| reset leaves an inconsistent attitude | fixed as D-02, attitude now repeatable to <1°, rate unchanged (p=0.40) |

Disabling each component individually changed nothing, which is what a self-sustaining loop
looks like rather than a faulty part.

- **Not withdrawn from the docs.** The README results table keeps its caveat: those numbers
  were single passes and carry an unmeasured variance.
- **If it is ever picked up**, it belongs with `examples/navigation/`, and the question to ask
  is whether the oscillation can start from a quiet state or only from a large initial error.
  The tooling to answer that exists: `scripts/record_trace.sh`, `scripts/analyse_trace.sh`,
  and `--split`.
- Full evidence: `docs/worklog/2026-08-04-scenarios-do-not-repeat.md`.

### D-01 (original entry, kept for the reasoning) *(2026-08-04)*

`cross_the_plaza`, seed 1, `oracle`, shipped defaults, nothing changed between runs:
**3 successes and 4 failures out of 7.** Documented as 5/5. The successes reproduce the
documentation closely (13-14 steps, ~19 m final, against 18.6 m / 14 steps); the failures all
hit `max_steps` at 25 with roughly twice the path length over the same 80 m journey. Bimodal,
not noisy. No collisions in any run.

Not caused by the day's changes — the identical configuration produced both outcomes in both
orders, and the velocity and yaw slews were each cleared by A/B.

**This is now the most important open item**, because the scope agreed 2026-08-04 makes
determinism and repeatability what this repository is *for*. It also puts a caveat under every
result measured at N=5 x 1 seed, including E-01b and the README table: if one seed is 3/7 on
repeat, 5/5 was a sample rather than a property.

- **E-03 landed 2026-08-04** (`scripts/record_trace.sh`, `scripts/analyse_trace.sh`) and the
  first failing trace is captured. The failure is a **lateral oscillation**, not a longer
  route: 15.2 m of travel against the net direction where successes measure 0.0, waypoints
  alternating 15-20 m either side of the goal line. **Bearing-only correlates with SUCCESS**
  — both successes were 100% bearing-only, the failure 15 of 24, and its depth-valid
  waypoints resolve to impossible points (220 m off the map; 17 m below street level). That
  contradicts what this file and the PR previously called the likely cause.
- Rate now **12/17** on this seed across the day.
- **The oscillation is in the ANNOTATION, not the projection** — `analyse_trace.sh --split`
  shows the failing run's pixel slamming between u=0 and u=959, the exact bounds of a 960 px
  frame, so the annotation is being clamped because its target is outside the field of view.
  Successes hold u≈480, dead centre. Grounding is faithfully projecting a swinging pixel.
- The goal leaves the frame because the **heading oscillates ±20°** and never converges in the
  failing run, where every success settles to ~0° within two seconds.
- **D-02 was fixed and D-01 did not move** (8/10 vs 12/17, p=0.40). The starting attitude is
  now repeatable to under a degree and the failures are unchanged, so the perturbation
  hypothesis is refuted. Four single-cause hypotheses have now failed: `bearing_only`,
  projection lag, teleport inaccuracy, reset attitude. The remaining reading is that the
  oscillation is a property of the **closed loop** — waypoint → velocity → yaw → camera →
  pixel → waypoint — which is navigation-stack behaviour and **out of scope** here. If that
  holds, the right outcome for D-01 is documented and bounded rather than fixed.
- Superseded, kept for the record: `out/episodes/*.json` stores `steps` as a count
  with no per-step trace, so the shape of the doubled path cannot be recovered from what is
  recorded. Two investigations have now stalled on this same gap.
- **Verify:** one seed x 10 repeats per scenario, reported as a rate with the spread, before
  and after any fix. Not N seeds x 1.
- **Do not guess at the cause** until there is a trace. Evidence in
  `docs/worklog/2026-08-04-scenarios-do-not-repeat.md`.

### D-03 · `reset()` lands the aircraft BELOW its commanded altitude — **fixed** *(2026-08-04)*

Started as one observation from `ros2_traffic_flyover.py`. Measured with
`tests/conformance/p12_reset_altitude.py` — 32 resets through the **real** `Vehicle.reset()`,
against a bare simulator with no ROS graph — and it is systematic, not variance:

| commanded | AGL | error @ 8 m/s | error @ 10 m/s |
|---|---|---|---|
| z = -55.0 | 82.5 m | mean 14.7, worst 25.2 | mean 21.1, worst **32.2** |
| z = -30.0 | 57.5 m | mean 15.8, worst 21.7 | mean 20.1, worst 30.5 |
| z = -8.0 | 35.5 m | mean 13.3, worst 21.8 | mean 20.8, worst 28.4 |
| z = +23.95 | **3.5 m** | mean **3.3**, worst 3.5 | mean **3.5**, worst 4.2 |

Against a documented tolerance of ~9 m. Three things fall out:

- **The z-error is always POSITIVE** — +9.1 to +18.5 m — and positive NED z is *downward*.
  The aircraft consistently ends up **below** where it was told to be. This is a sag, not
  scatter.
- **It tracks altitude, not speed** (spread 14.5 m across altitudes vs 4.6 m across speeds),
  and the one accurate row is street level — the only altitude with no room to fall.
- **Faster is worse**: 10 m/s misses by more than 8 m/s at every altitude. Consistent with
  `moveToPositionAsync` declaring arrival on a lookahead that scales with velocity, so a
  faster command gives up further out.

The likely shape, **not yet proven**: `simSetVehiclePose` places the aircraft, then it is
unpowered through the 0.5 s sleep and the arm, falls, and `moveToPositionAsync(...).join()`
returns before it has climbed back. At 3.5 m AGL there is nowhere to fall to.

**Resolved 2026-08-04: real episodes are affected.** The apparent contradiction was my own
sampling — I quoted a single 4.1 m episode log against eight probe samples. Two checks
settled it:

- **The ROS service path reproduces it.** Same grid through `/sim/reset_vehicle` with 6 s
  between resets, i.e. exactly what an episode does: 16.0 / 15.4 / 13.6 m mean at -55 / -30 /
  -8, and 3.3 m at street level. Not a probe artefact, not back-to-back resets.
- **The traces say so directly.** Aircraft z at the moment ten real episodes began, against a
  commanded -55.0: -3.6, +2.1, +2.7, +3.1, +3.2, +3.5, **+18.0, +19.4, +22.8, +37.5**.
  Mean +10.9 m, worst +37.5 m. **Bimodal** — six starts inside 4 m, four between 18 and 37 m
  low.

**Fixed by converging rather than trusting one `join()`.** `moveToPositionAsync().join()`
returns when SimpleFlight decides it has arrived, which is not the same as being there.
`reset()` now commands the hold, checks where the aircraft actually ended up, and re-commands
up to `RESET_ATTEMPTS` times until it is within `RESET_TOLERANCE_M` (6 m) — reporting through
`on_stage` each time, and saying so loudly rather than raising if it cannot converge.

Same grid, 32 resets, after the fix:

| commanded | before (worst) | after (worst) |
|---|---|---|
| z = -55.0 | 32.2 m | **3.3 m** |
| z = -30.0 | 30.5 m | **1.7 m** |
| z = -8.0 | 28.4 m | **1.6 m** |
| z = +23.95 | 4.2 m | 3.3 m (was already fine) |

And on the real episode path, start-pose error across nine episodes: **5.7, 3.2, 1.5, 1.4,
4.9, 3.6, 1.7, 4.1, 2.8 m** — where four of the previous ten started 18–37 m low.

- **Cost:** a reset that needs a second or third attempt takes longer. Episodes went from
  roughly 60 s to roughly 130 s wall clock in this batch, which matters for sweep budgets.
- **Verify a fix by:** the same grid, and by whichever explanation accounts for the episode
  logs too.
- **Consequence if it holds:** every episode starts up to 18 m below its scenario's stated
  altitude, which makes start poses — and anything measured from them — not what the scenario
  says.

### D-03 (original single observation, kept for the record)

`examples/ros2_traffic_flyover.py` commands `hold_ned` z = **-8.0** and the reset reports
settling at **+7.5** — 15.5 m low, i.e. 20 m above the street where 35.5 m was asked for.
The documented tolerance is ~9 m (`QUICKSTART.md`: "a ~9 m start-pose error is normal", the
station-keeping floor), and today's episode resets at other altitudes came in well inside it:

    commanded z  23.9  ->  18.2   error 5.7 m
    commanded z -55.0  -> -51.9   error 4.1 m
    commanded z  -8.0  ->  +7.5   error 15.5 m   <- this one

**One observation, and deliberately not chased.** It is the same family as D-02 — what
`reset()` actually delivers versus what it commands — and it is in scope for the same reason.
But a single sample cannot distinguish an altitude-dependent effect, a speed-dependent one
(the flyover resets at 8 m/s, `run_episode` at 10), or ordinary variance, and this project has
spent a day on hypotheses that did not survive measurement.

- **Verify:** reset to a range of altitudes, N repeats each, and report the error against
  commanded as a function of altitude and approach speed. `tests/conformance/p11_reset_attitude.py`
  is the shape to copy — it already resets in a loop and reports a spread.
- **Note:** `reset()` was changed the same day (D-02's yaw hold). Whether that is related is
  unknown; there is no before-measurement at this altitude to compare against.

### D-05 · A fixed reset tolerance is wrong at street level — **done** *(2026-08-06)*

**Answered: it is a FLOOR, not a miss**, which is why this was filed rather than fixed on the
spot. `p12` gained `--reset-tolerance` so the question could be settled rather than argued.

Asked to converge to **1 m** instead of the shipped 6:

| commanded | AGL | at 6 m tolerance | at 1 m tolerance |
|---|---|---|---|
| z = -55.0 | 82.5 m | 1.5 m mean | **0.60 m** |
| z = -30.0 | 57.5 m | 1.4 m mean | **0.57 m** |
| z = -8.0 | 35.5 m | 1.3 m mean | **0.59 m** |
| z = +23.95 | **3.5 m** | 3.29 m | **3.29 m — unmoved** |

Two things fall out, and they pull in opposite directions:

- **The loop was stopping early for no reason.** Every altitude with air beneath it reaches
  sub-metre when asked to. 6 m was not buying anything except an earlier exit.
- **Street level cannot be reached at all.** 3.3 m, identical across 8 resets at both speeds,
  unmoved by retrying. And the number says what is happening: commanded to **3.5 m AGL** the
  aircraft settles at **0.2 m AGL** — it is on the ground. The 3.5 m is never held.

So tightening alone would have been wrong exactly as suspected: it would make every
street-level reset burn four attempts and then report failure.

**Fixed with both halves.** `RESET_TOLERANCE_M` 6.0 → **1.5**, and a new
`RESET_MIN_IMPROVEMENT_M` (0.3 m): an attempt that does not improve the miss by at least that
much is treated as converged-as-far-as-it-goes. Retry while it helps, stop when it does not.
No altitude special-casing — the loop measures whether it is making progress rather than being
told where progress is possible.

Verified in the real path, not inferred:

    street level   holding -> re-holding (3.3 m out) -> stalled at 3.3 m out;
                   further attempts are not improving it        4.8 s
    35 m AGL       holding -> re-holding (15.5 m out) -> converged after 2   9.1 s

Shipped defaults re-measured across 32 resets: 1.0-1.2 m mean at altitude, worst 1.3 m, and
street level exits early instead of burning attempts.

**Consequence for `street_level`, and it is not a reset bug.** That demonstration asks to fly
at 3.5 m AGL and the aircraft cannot be *placed* there — it rests on the ground and climbs
from there once the controller takes over. Whether SimpleFlight refuses to hover that low, or
a collision volume stops it, is unmeasured. The scenario is a demonstration and is not scored,
so nothing measured depends on it, but its premise is weaker than it reads.

### D-05 (original entry, kept for the reasoning) *(2026-08-05)*

`RESET_TOLERANCE_M` is a flat 6 m, which is sensible at 82 m AGL and meaningless at 3.5 m.
`examples/ros2_street_level.py` asks for z = 23.95 (3.5 m AGL) and was placed at 27.0 — **0.5 m
AGL**, 3 m low, or 86% of the entire flight altitude gone. The D-03 converge loop never
retried, because 3 m is inside tolerance.

Not caused by the D-03 fix: `p12_reset_altitude.py` measured street level at 3.3 m error both
before and after, consistently. What the fix did was make every *other* altitude accurate
enough that this one now stands out.

- **Open question, and the reason this is filed rather than fixed:** whether the converge loop
  *can* correct it. The street-level error was identical across 8 resets before and after the
  fix, which looks like a floor rather than a miss — possibly ground effect, a collision
  volume, or SimpleFlight refusing to descend the last few metres. Tightening the tolerance
  without knowing that would just make every street-level reset burn `RESET_ATTEMPTS` and then
  report failure.
- **Verify:** `p12` with a tolerance of 1 m and street level in the grid — does it converge, or
  exhaust its attempts at the same 3.3 m?
- **Consequence today:** anything flying near the ground starts lower than it asked for.
  `street_level` is a demonstration and is not scored, so nothing measured depends on this yet.

### D-04 · The sidecar wedges on `destroy` after several episodes — **fixed** *(2026-08-04)*

Twice in one session, after roughly four to nine consecutive episodes, `/sim/destroy_actors`
stopped answering and every following episode died the same way:

    RuntimeError: destroy: no response after 30.0s

`status.sh` shows the simulator, the sidecar and the bridge all still running with the socket
present — so it is a wedge, not a crash, and nothing in the process counts reveals it. The
first occurrence followed a run killed mid-flight, which would explain it; the second did not,
which does not.

This is squarely in scope: a simulator that stops answering after N episodes cannot host a
sweep, and E-06 closed a *different* death (an uncaught CARLA `TimeoutException` killing the
sidecar) that this is not.

**Root cause: `ChaseCamera.stop()` deadlocks on its own bounded queue**, and does it while
holding the sidecar's slow lock. Found with a SIGUSR1 stack dump added for the purpose:

    threading.py:320 in wait
    queue.py:140 in put           <- blocked putting into a FULL queue
    chase.py:136 in stop
    server.py:428 in chase_stop

with a second thread parked in `_handle`'s `with lock:` — that is `destroy`, waiting forever.

The race: `stop()` sets `self._writer = None`, and `_drain` **exits the moment it sees that**,
so the sentinel offered on the next line has no consumer. CARLA's sensor thread has meanwhile
been filling a bounded queue at the chase frame rate, so a busy episode ends with it full and
the `put` never returns. Intermittent because it needs the queue full at that instant, which
is why it took four to nine episodes — long enough for the encoder to fall behind.

`stop()` no longer blocks: the sentinel goes in with `put_nowait`, making room by dropping a
frame if it must, and the queue is drained afterwards rather than left holding 1080p buffers.
It also says so if the drain thread will not exit, instead of returning counts from a
recording still being written.

- **Verified:** 7 consecutive episodes with no wedge, where it previously died at the 4th.
  `tests/test_chase_stop.py` covers it with no simulator — 4 tests, and the old code **hangs**
  them, which is the proof they bite.
- **Kept:** `_install_stack_dumper()` in the sidecar. `kill -USR1 <pid>` dumps every thread's
  stack. It found this in one shot after two sessions of guessing, and a wedge is invisible to
  `status.sh` — the simulator, sidecar and bridge were all still running with the socket
  present.
- **My hypothesis was wrong first:** I blamed the 1 Hz world tick starving the slow lock.
  Six spawn/destroy cycles against a live tick came back at 1.2 s and 0.0 s with no
  contention. The dump beat the reasoning.

### D-02 · `reset()` commands yaw zero and does not get it — **done** *(2026-08-04)*

`Vehicle.reset()` places the aircraft with `airsim.Quaternionr(0, 0, 0, 1)` — yaw zero.
Measured off the odometry at the moment five identical episodes began: **17.6°, 20.9°, 20.9°,
25.7°, 27.8°**. Same scenario, same seed, same everything.

A seeded run that starts from a different attitude every time is not seeded, and this is the
kind of repeatability the scope agreed 2026-08-04 makes the point of the repository.

It is also the leading suspect for D-01: the one failing run of the five had the *lowest*
starting yaw and then oscillated ±20° without ever converging, while all four successes
settled to within a degree of zero inside two seconds. **That correlation is not a mechanism**
— a heading loop through the navigation stack can oscillate from any perturbation — and
separating the two is exactly why this is filed on its own.

- **Measured 2026-08-04** with `tests/conformance/p11_reset_attitude.py`, 10 iterations
  against a bare simulator: the airframe rotates a **median 98.6° (worst 105.0°) during the
  2 s settle**, every iteration. Nothing holds heading once `moveToPositionAsync` returns, so
  the attitude at episode start is whatever it drifted to. That alone explains the 17-28°
  spread seen in the traces.
- **Not established: which stage loses it.** The probe blames the teleport, but the same
  `simSetVehiclePose` call issued once from an idle aircraft delivers exactly 0.00° and holds
  it. Order-dependent, so the probe's stage attribution is marked untrusted in its own
  docstring.
- **Fixed 2026-08-04.** `moveToPositionAsync`'s default is `YawMode(is_rate=True,
  yaw_or_rate=0.0)` — *rate* control commanding zero rate, which is "do not drive yaw", not
  "hold yaw at zero". `Vehicle.reset()` now passes `YawMode(is_rate=False, yaw_or_rate=0.0)`.
  A/B over 10 resets each: **worst drift 65.2° → 0.9°**, median 0.5° → 0.3°.
- **It did NOT fix D-01.** Ten runs of `cross_the_plaza` seed 1 with the fix: 8/10, against
  12/17 before. P(≥8 of 10 | rate unchanged) = 0.40 — the sample cannot tell them apart. The
  heading-perturbation hypothesis is refuted; the fix stands on its own merits.
- **Verify:** starting yaw across 10 identical resets, reported as a spread. It should be a
  degree or two, not ten.
- Evidence: `docs/worklog/2026-08-04-scenarios-do-not-repeat.md`.

### E-03 · Record an MCAP bag per episode — **done** *(2026-08-04)*

Landed as `scripts/record_trace.sh` (MCAP via rosbag2) and `scripts/analyse_trace.sh`, plus
`--split` for separating an annotation from the projection of it. Simulator-side rather than
wired into the episode runner, which is an example now.

It paid for itself the day it landed. Three things came out of traces that no amount of
reading found: the failing runs oscillate laterally rather than taking a longer route;
`bearing_only` correlates with SUCCESS, not failure — the opposite of what had been recorded
in two places; and the aircraft was starting up to 37 m below its commanded altitude, which
became D-03.

Cameras are excluded by default — RGB at 960x720 and 8 Hz is ~16 MB/s, so a three-minute
episode would be ~3 GB — with `--camera` and `--all` for when that is what you want.

*(Partly overtaken by E-05: "a failed episode leaves a JSON and nothing to look at" is no
longer true. What a bag still adds over video is **replayability** — feeding the recorded
topics back through `grounding` to check it produces the same waypoints, which no video can
do.)*

A failed episode currently leaves a JSON and nothing to look at. Recording the annotation,
grounded-waypoint, odometry and camera topics would make failures diagnosable rather than
merely counted.

- **Verify:** a bag replays and the grounding node produces the same waypoints from it.
  Note `.gitignore` already excludes `*.mcap` and `rosbag2_*/`.

### E-04 · Anchor scenarios to real map features — **out of scope** *(2026-08-04)*

> Same: scenario design. Kept for the reasoning.

Start and goal coordinates were hand-picked from one spawn point. They lint clean and the
oracle proves them navigable, but nothing ties them to junctions, plazas or landmarks a
natural-language instruction could sensibly refer to. That matters once a real VLM reads the
instruction.

---

## VLM

### V-01 · First real backend — **out of scope** *(2026-08-02; closed 2026-08-07)*

> **Closed by the 2026-08-04 scope decision, recorded 2026-08-07.** This said *"built,
> awaiting a flight test"* for three days after model choice went out of scope, so it was
> promising a flight test nobody intends to run. The backend still exists and still works
> — it is an `examples/` concern now, not a backlog item. Kept below for the design
> decisions, which are about the *contract* a backend must satisfy and are still true.

Decision made: **Claude API**, not local vLLM. `ros2_ws/src/vlm_client/vlm_client/backends/claude.py`
implements the same narrow contract as the baselines — one BGR frame and one instruction in,
one pixel out — so its score is comparable with theirs. Registered as `claude` in `BACKENDS`;
run it with `./scripts/bringup.sh --backend claude`.

Design decisions worth keeping:

- **Structured outputs**, not prose parsing. `output_config.format` pins the reply to a JSON
  schema, so the pixel arrives as an integer. The schema cannot express numeric bounds, so
  `u`/`v` are clamped on our side.
- **`effort: low` by default.** 40 steps in 300 s is 7.5 s per decision; a higher effort can
  spend that on one call and turn every episode into a timeout that measures the budget
  rather than the navigation. Raise `claude_effort` when measuring quality.
- **Adaptive thinking stays on.** Disabling it is legal at low effort, but on this model a
  thinking-disabled reply can leak internal tags — with a schema in play that is a parse
  failure, not a cosmetic one. Buy latency with `effort` instead.
- **The API key is not a ROS parameter.** It comes from `ANTHROPIC_API_KEY` in the launching
  shell; parameters are readable from the graph and land in launch logs, which this repo
  commits.
- **The SDK is a python 3.12 dependency** and lives in `vendor/py312` (installed by
  `scripts/fetch_vendor.sh`, put on `PYTHONPATH` by `scripts/bringup.sh`) — *not* in the 3.10
  `.venv` that owns the carla/airsim clients. Installing it into the wrong interpreter
  produces a `ModuleNotFoundError` that reads like a missing package.

22 offline tests cover the request shape and reply handling against a stubbed SDK — no
network, no key. See `tests/test_claude_backend.py`.

- **Still to do:** the flight test — **blocked on API credentials.** Checked 2026-08-02: this
  machine has a Claude.ai subscription (Claude Code's `~/.claude/.credentials.json`) and no
  API credential. Those are different products — the subscription covers claude.ai and Claude
  Code, the API is billed separately with its own credits, and Claude Code's OAuth token has
  the wrong audience and scopes for the SDK. The backend accepts `ANTHROPIC_API_KEY`,
  `ANTHROPIC_AUTH_TOKEN`, or an `ant auth login` profile, and fails at construction naming all
  three if it finds none.
- **If paying per call is not wanted**, the other half of this fork is still open and needs no
  API credentials — see V-01b.
- **Verify:** success rate over the same 5 seeds x 4 scenarios as E-01, plus p50/p95 decision
  latency (the backend tallies both, and logs them on shutdown). The bar to clear is
  `geometric` on the three open scenarios; `avoid_the_block` is the one where it has room to
  beat the oracle too, since a straight-line policy cannot solve it (see E-02).

---

## Packaging

### P-02 · Reach the graph from another machine — **done** *(2026-08-06)*

> **Filed after the fact.** Same wrong order as E-05 and T-04, and recorded rather than
> backdated: `scripts/discovery_server.sh`, `examples/byo_agent.py` and `docs/containers.html`
> were all written before this entry existed.

Containerising the stack made a question urgent that the host setup never raised: how does a
client that is not on this machine reach the graph at all? DDS finds peers by **multicast**,
which does not cross a VPN, a routed subnet or the internet, so the default configuration
cannot work off-box however the firewall is set.

`scripts/discovery_server.sh` runs a Fast-DDS discovery server — every participant announces
itself to one known address over unicast, and anything pointed at the same address gets the
whole graph. Data still flows peer-to-peer, so the server never carries camera frames.
`stack_up.sh` and `stack_run.sh` both honour `DISCOVERY_SERVER=<host>:<port>` and validate it,
because a malformed value reaches Fast-DDS as a silently-ignored address and an ignored
discovery setting presents as an empty graph.

**Verified on two different interfaces**, since binding to `0.0.0.0` proves nothing on its own:

| via | address | measured |
|---|---|---|
| NetBird | `100.127.184.189:11811` | odometry 17.9 Hz |
| LAN | `10.0.0.72:11811` | odometry 16.1 Hz, camera 4.0 Hz |

**Two things that cost the debugging, both now in the script's own output:**

- **`ROS_SUPER_CLIENT=true` is not optional.** A plain discovery client is only told about
  participants it has already matched on a subscribed topic, and `ros2 topic echo` introspects
  the graph *before* it can subscribe, to resolve the message type. Without it you get
  `Could not determine the type for the passed topic` while the publisher is right there.
- **Discovery and transport are separate problems.** With the discovery server alone the
  client discovered cleanly and received **nothing**: Fast-DDS advertises a shared-memory
  locator to same-host peers and a client outside the IPC namespace cannot use it.
  `configs/dds/udp-only.xml` alongside it gives 17.9 Hz. A genuinely remote machine is never
  offered that locator — but the local rehearsal of a remote setup is, which is exactly when
  someone hits it.

- **Not verified:** connecting from a genuinely separate machine. There is one host here, so
  what remains unknown is whether something upstream drops the traffic, not whether the
  configuration is right.
- **Deliberately not addressed:** bandwidth. `/camera/rgb/image_raw` is **133 Mbit/s** raw at
  the shipped resolution. The operator's call was to ship the transport and ignore that; the
  sidecar already has JPEG encoding (`view_jpeg`), so a compressed image topic is small work
  if a link ever complains.

**Also landed alongside, and also filed late:**

- `examples/byo_agent.py` — a bring-your-own-agent template importing nothing from this
  project but the message package. It documents the three insertion points (a pixel on
  `/vlm/annotation`, a point on `/control/waypoint`, raw setpoints on
  `/fmu/in/trajectory_setpoint`) and the two traps that bite: PX4 topics are
  BEST_EFFORT + TRANSIENT_LOCAL so a RELIABLE subscriber silently gets nothing, and an
  annotation must carry the IMAGE's stamp rather than `now` or grounding pairs it with the
  wrong depth frame.
- `--backend none` on the VLM example, so the grounding layer can run **without** a shipped
  backend competing for the same topic. Verified: annotations at 6.05 Hz became waypoints at
  6.09 Hz.
- `docs/containers.html` — what runs in each container, where the data flows, and how to fly
  it from outside, with diagrams of the namespaces and the data path.

### P-01 · Containerise the stack — **UNBLOCKED** *(2026-08-06)*

**Hardware Vulkan works in a nested container on this machine.** Verified from a plain
`ubuntu` base, on GPU 1:

    deviceType = PHYSICAL_DEVICE_TYPE_DISCRETE_GPU
    deviceName = NVIDIA GeForce RTX 5060 Ti
    driverName = NVIDIA

The recipe, and it is short:

```bash
docker run --gpus '"device=nvidia.com/gpu=1"' <image>   # the inner quotes are required
```

```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
      libvulkan1 libxext6 libx11-6 libglvnd0 libgl1 libegl1 \
 && mkdir -p /usr/lib64 \
 && ln -sf /lib/x86_64-linux-gnu/libGLX_nvidia.so.0 /usr/lib64/libGLX_nvidia.so.0
```

Confirmed on **both** `ubuntu:22.04` (the sidecar's cpython-3.10 base) and `ubuntu:24.04`
(ROS Jazzy), so neither image is constrained by it.

**Two causes, and the second is the one nobody had:**

1. CDI injects an ICD pointing at `/usr/lib64/libGLX_nvidia.so.0` — the Fedora-family host
   path — which does not resolve in an Ubuntu container. The same bug `run_sim.sh` already
   works around in the distrobox. Necessary to fix, **not sufficient**.
2. `libGLX_nvidia.so.0` is a GLVND **vendor** library. Without `libGLX.so.0` / `libEGL.so.1`
   present it loads but exposes no entry points, which surfaces as
   `Could not get 'vkCreateInstance' via 'vk_icdGetInstanceProcAddr'` and a silent fall back
   to llvmpipe.

**Ruled out along the way**, each by changing one variable: the `--gpus` vs `--device` form
(identical), `NVIDIA_DRIVER_CAPABILITIES` (no effect — the CDI spec is static), the Ubuntu
base and loader version (identical either way, so the original "22.04 → 24.04 still fails"
was true and beside the point), the GPU (0 and 1 behave the same), and bind-mounting
`libnvidia-api.so.1`.

Found by diffing a known-good container against a failing one after the operator supplied a
measured account of the working `drone-sim` renderer — 58 libraries against 48, and the ten
extra were all `libGLX*`/`libEGL*`. Full account:
`docs/worklog/2026-08-06-gpu-in-a-container.md`.

**Done the same day: the simulator runs containerised, end to end.**
`docker/sim.Dockerfile` + `scripts/run_sim_docker.sh`, verified rather than assumed:

| check | result |
|---|---|
| ports serving | CARLA :2000 and AirSim :41451, ready in 40 s |
| VRAM on the requested card | **3339 MiB on GPU 1**, against ~3.3 GB native |
| the process holding it | `CarlaUE4-Linux-Shipping` at 3324 MiB — the containerised one |
| sustained load | GPU 1 at 42-43%, GPU 0 flat at 0% |
| software fallback | **0** mentions of llvmpipe/lavapipe/swiftshader |
| the existing stack against it | `bringup.sh --no-sim` bridged normally; odometry 16.2 Hz, camera 5.9 Hz |
| **a scored episode** | `cross_the_plaza` seed 1, **SUCCESS 18.0 m in 13 steps**, reset error 1.1 m, chase 301 frames 0 dropped |

18.0 m / 13 steps is the documented baseline exactly, so the containerised simulator is not
merely running — it produces the same result.

- **The release is mounted, never baked.** It is a licensed 18 GB binary drop that changes
  independently of this repository.
**All three containerised the same day, and a scored episode flown entirely in containers.**

    ./scripts/stack_up.sh --config configs/testbed.yaml
    ./scripts/stack_run.sh -d examples/navigation/run.sh
    ./scripts/stack_run.sh -d examples/vlm_navigation/run.sh --backend oracle
    ./scripts/stack_run.sh scripts/run_episode.sh --scenario cross_the_plaza --seeds 1
    ./scripts/stack_up.sh --down

| container | image | what it is |
|---|---|---|
| `carla-air-sim` | `carla-air/sim:v0.1.7` | CarlaUE4 on the GPU, owns the network and IPC namespaces |
| `carla-air-bridge` | `carla-air/sim-bridge:1` | the 3.10 sidecar — Ubuntu 22.04 ships CPython 3.10 natively, so no uv and no standalone interpreter build |
| `carla-air-ros` | `carla-air/ros:1` | the 3.12 graph, on `ros:jazzy-ros-base` |

Result: **SUCCESS 19.0 m in 13 steps**, reset error 1.1 m, chase 300 frames 0 dropped —
the documented baseline.

**Three things that had to be got right, each found by it failing:**

- **`--ipc shareable` on the simulator.** The joiners use `--ipc container:`, and `--ipc host`
  is refused by this daemon under rootless/nested Docker. Without it the ROS container will
  not start at all.
- **The repository is mounted at its OWN absolute path**, not at `/workspace`.
  `colcon build --symlink-install` fills the install tree with absolute symlinks, so any other
  mount point leaves them dangling: `Package 'bringup' not found`.
- **`python3-msgpack` in the ROS image.** `sim_bridge/protocol.py` is imported by the ROS-side
  client and imports it at module scope; without it the bridge node dies with a bare
  `ModuleNotFoundError` naming nothing about the seam.

**A consequence worth knowing before reaching for the host.** The containers share one IPC
namespace and Fast-DDS prefers shared memory, so a HOST process discovers the graph and then
receives nothing — measured: `ros2 topic hz` on the host reported NO DATA for topics
publishing at 16 Hz inside. That is what `stack_run.sh` is for. It is not a bug to fix by
disabling shared memory; it is what containerising the graph means.

- **Still mounted, not baked:** the 18 GB release and the repository itself. The environment
  is the slow, stable part and belongs in an image; the code changes every few minutes. A
  hermetic image that bakes the source is a reasonable follow-up and is not what makes this
  reproducible today.
- **`stack_up.sh --down` removes every `carla-air-*` container**, not only the three it starts,
  and reports what it left. The examples and one-shot runs hold the same resources, and a
  teardown that misses them is the containerised form of the leftover-graph failure rule 1
  exists for.
- **`stop.sh --all` and `status.sh` now know about the container.** Without that, rule 1 had a
  hole: `stop.sh` would report success while the container held 3.3 GB of VRAM.
- **Not yet measured:** whether the container costs throughput. Odometry read 16.2 Hz against
  19.5 native and the camera 5.9 against 8.1, but each is a single sample taken while three
  `ros2 topic hz` calls competed, so it is not a comparison. Worth a proper A/B before anyone
  quotes a container overhead.
- Unchanged and still true: `sim-bridge` and the `ros:jazzy-ros-base` images already built and
  ran. The simulator was the only one blocked.

### P-01 · Containerise the stack — original entry, blocked on non-nested Docker

Deferred 2026-08-01, not abandoned. The work was reverted from the tree; this entry keeps
what was learned so it resumes from here rather than from scratch.

**Unblocks when Docker runs on the host instead of inside the distrobox** — one driver hop
instead of two. Nothing about the compose design needs revisiting; only the GPU path failed.

Three images were built and two of them worked: `sim-bridge` (Ubuntu 22.04 ships CPython
3.10 natively, so no uv needed) ran the full 50-test offline suite, and a `ros:jazzy-ros-base`
image built every message and loaded the cross-interpreter seam under 3.12. The compose
topology was sound — `sim` owning the network namespace, `shm_size: 2gb` for Fast-DDS, the
18 GB release mounted read-only rather than baked.

**What killed it: GPU passthrough does not survive nesting here.** Docker runs *inside* the
`drone-sim` distrobox, so the driver has to traverse host → distrobox → container. Compute
survives that (`nvidia-smi` works in the nested container); Vulkan does not:

```
vk_icdGetInstanceProcAddr -> no vkCreateInstance  ->  ERROR_INCOMPATIBLE_DRIVER
                                                      falls back to llvmpipe
```

Ruled out, each by measurement rather than reasoning:

| tried | result |
|---|---|
| CDI `gpu=1` vs `gpu=all` | identical failure — not device selection |
| Image base 22.04 → 24.04 (loader 1.3.204 → 1.3.275) | still fails |
| Regenerate the CDI spec with `nvidia-ctk` | **byte-identical output** — already its best |
| Bypass CDI: bind-mount all 60 driver libs + 7 device nodes | same failure |
| `--privileged` + `nvidia-caps`, `/proc/driver/nvidia` verified present | same failure |
| podman in the distrobox | podman 4.9 cannot parse the CDI 0.7.0 spec |
| Host podman via `distrobox-host-exec` | unreachable — no `org.freedesktop.Flatpak` portal on the session bus in a detached screen session |

The root asymmetry is visible in the specs: Docker reads `/etc/cdi-local` (61 driver libs, no
`libGLX_nvidia.so.0`), while the host's `/etc/cdi` has 75 including it. The nested spec is
generated *inside* a distrobox whose own driver tree is itself a bind-mount.

**Two of the intermediate diagnoses were wrong** and are recorded so they are not repeated: a
`__malloc_hook` undefined-symbol trace looked like a glibc mismatch but both sides are glibc
2.39 (those are libnvidia-glcore's optional X-server hooks), and a symbol check run with `nm`
in an image that has no binutils returned a confidently meaningless answer.

**What to redo when it unblocks.** The three Dockerfiles, entrypoints and compose file were
deleted; they are recoverable from this description plus the git history of the branch they
were never committed to — in practice, expect to rewrite them. Worth keeping in mind:

- `sim-bridge` on Ubuntu 22.04 needs no uv at all — that release ships CPython 3.10 natively,
  which is the ABI the CARLA client is tagged for.
- The client module must come from the project's GitHub repo at a pinned commit, **not** the
  release archive (see the broken-RPATH finding in the 2026-08-01 worklog).
- `ros2` on `ros:jazzy-ros-base`, with `px4_msgs` cloned at `392e831c` and built before the
  workspace so a source edit does not invalidate its 2-minute codegen.
- The offline suite belongs in the **sim-bridge** image, not the ROS one: it imports
  `server.py`, which imports carla and airsim.
- `sim` owns the network namespace; the others join it. `shm_size: 2gb` on that service only.
- Mount the 18 GB release read-only; nothing writes inside it.

**Meanwhile everything stays inside the distrobox, which is where it already works.**

## Housekeeping

### H-03 · `stop.sh --all` reported success without stopping the simulator — **done** *(2026-08-03)*

The teardown script gave the ROS processes a TERM/TERM/KILL escalation and the simulator a
**single TERM with no wait and no check**, then printed `stopped: graph, sidecar, simulator`
unconditionally.

Observed twice while debugging E-06: the sidecar had already died, `stop.sh --all` claimed
success, and the simulator was still holding **3.5 GB of VRAM on GPU 1** until
`run_sim.sh --kill` was run by hand. Unreal does not always go down in the instant between
the `pkill` and the `echo`.

**This is rule 1's enforcement mechanism**, so a version that reports what it *attempted*
rather than what it *achieved* is worse than no script — every "everything stopped" in this
project's worklogs rests on it.

Now: the same TERM/TERM/KILL escalation for the simulator, a `pgrep -x` check afterwards,
an honest message (`simulator stopped` vs `SIMULATOR STILL RUNNING`), stragglers listed with
their command lines, and a **non-zero exit** if anything survived — so a caller can tell.

Verified against the exact failing case — simulator up with no sidecar and no graph:
3264 MiB -> 33 MiB in 2.0 s, `rc=0`, message truthful.

**And it was stopping less than it claimed in a second way.** Auditing what "everything"
covered found four things it had never matched: the VLM example's `ros2 launch` (its *nodes*
matched, its launcher did not), the **web console** — which can start a simulator by itself —
`run_episode`, and `run_sweep`, which brings the simulator back up between backends.

Command-line patterns alone could not fix it: the console is normally started as
`./.venv/bin/python webui/server.py`, whose command line contains no absolute path, and an
unanchored pattern would reach drone-sim in the same container. **Ownership is now proven
rather than guessed** — a candidate is ours only if `/proc/<pid>/cmdline` mentions `$PROJ` or
`/proc/<pid>/cwd` is inside it. Neither can be spoofed by a relative invocation, and neither
can match a project living somewhere else.

`stop.sh` also skips its own process ancestry, so a `stop.sh` called from inside
`run_sweep.sh` no longer kills the sweep that called it — and can never take out the
operator's shell.

`status.sh` now reports the same set, because a status that checks less than stop removes
will report "clean" while something is still up.

**It found a real orphan immediately:** a web console from an earlier session, started with
`--bind` for the mesh, still listening days later. `status.sh` had shown every count 0 that
whole time.

Full-stack verification — simulator, sidecar, core graph, VLM example and web console all
running, 11 processes: **one `stop.sh --all`, 4.2 s, everything gone.** No CarlaUE4, no web
port, no sim port, GPU 1 back to 33 MiB, `rc=0`.

### H-01 · Maintainer email — **done** *(2026-08-01)*

Aligned to `aldwin@hermanudin.com` across 13 package files to match the repo's git identity,
before the first public push. `package.xml` maintainer is a public-facing field, so the
moment before publishing was the cheap time to do it.

### H-02 · No remote — **done** *(2026-08-01)*

Published to `teapotlaboratories/carla-air-testbed`, public, matching the sibling
`drone-sim`. The repo is named for what the work became rather than the local directory
name it started as.

---

## Done

- Testbed built: 3.10 sidecar + 7 ROS 2 packages, PX4-shaped topics *(2026-08-01)*
- Hardware rendering fixed — the simulator had been on a software rasteriser for an entire
  build *(2026-08-01)*
- DDS domain isolated from `drone-sim`, which had been corrupting every rate measured
  *(2026-08-01)*
- Depth resolution tuned: RGB+depth 4.0 → 19.5 Hz, metric precision intact *(2026-08-01)*
- Oracle backend + scenario linter; all four scenarios validated navigable *(2026-08-01)*
- 50 offline tests, runnable with no simulator, GPU or display *(2026-08-01)*
- README, quick start, architecture doc, HTML guide, three worklogs *(2026-08-01)*
