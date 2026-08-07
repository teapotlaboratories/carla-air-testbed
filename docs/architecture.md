# Architecture

A VLM navigation testbed over ROS 2, on CARLA-Air. Two processes, one ROS 2 graph, and a
deliberate seam between them.

> **This document describes what runs today.** The project is *a ROS 2-driven drone
> simulator*; VLM navigation is an example on top of it — see
> [`todo.md` → Repositioning](todo.md#repositioning--a-drone-simulator-first-a-vlm-testbed-second).
> `bringup.sh` starts the bridge, the controller, the episode runner and the recorder, and
> **no node that interprets a camera**. One thing is still on the wrong side of that line:
> the **web console** commands the aircraft over the sidecar socket instead of ROS 2
> (R-03, re-planned 2026-08-07 — the containers stranded it, so the port is now also how it
> reaches the stack at all).

```
┌─ Python 3.10 ────────────────┐          ┌─ Python 3.12 / ROS 2 Jazzy ──────────────────┐
│                              │          │                                              │
│  CARLA-Air (UE4, headless)   │          │   carla_air_bridge                           │
│    :2000  CARLA RPC          │          │     /fmu/out/vehicle_odometry      14 Hz     │
│    :41451 AirSim RPC         │          │     /camera/rgb/image_raw         2.7 Hz     │
│         ▲                    │          │     /camera/depth/image_raw                  │
│         │ carla + airsim     │          │           │                                  │
│  sim_bridge/server.py        │          │     /sim/reset_vehicle  (services)           │
│    ├─ telemetry AirSim client│◄────────►│     /sim/spawn_traffic · set_weather         │
│    ├─ media AirSim client    │  UDS +   │           │                                  │
│    ├─ control AirSim client  │ msgpack  │           ▼                                  │
│    └─ world AirSim client    │  four    │   control ── /fmu/in/trajectory_setpoint 10Hz │
│         Vehicle · Camera     │  conns   │        ▲  │                                  │
│         World  (traffic,     │          │        │  ▼                                  │
│                weather)      │          │   evaluation ── /episode/{status,result}      │
└──────────────────────────────┘          └────────┼─────────────────────────────────────┘
                                                   │  /control/waypoint
                                          ┌────────┼─────────────────────────────────────┐
                                          │  examples/vlm_navigation  (NOT the simulator) │
                                          │   vlm_client ─ /vlm/annotation ─ grounding    │
                                          └──────────────────────────────────────────────┘
```

## Why two processes

Not a preference — a measured wall. The CARLA-Air client is an ABI-tagged
`libcarla.cpython-310` extension; ROS 2 Jazzy ships Python 3.12. Neither interpreter can
load the other's C extensions:

```
import carla under ROS 2 python3.12  →  ModuleNotFoundError: No module named 'carla.libcarla'
import rclpy under the 3.10 venv     →  _rclpy_pybind11.cpython-310-...so isn't present
```

Upstream's own ROS 2 example works only because Humble is *also* 3.10 — the `PYTHONPATH`
trick it documents has that unstated precondition. Three ways out were available: install
Humble alongside (a second ROS distro, and nodes that cannot be shared with the Jazzy work
in `drone-sim`), put frames through shared memory, or accept one serialisation hop. We took
the hop. At 640×480 it costs about a millisecond against a capture that costs 500.

`sim_bridge/protocol.py` is imported **by both interpreters** — the same file, not a copy.
A wire format that drifts between the two halves of a bridge is the classic way to lose a
day.

## Why three connections

The first version used one connection and one AirSim client. Image capture takes ~250 ms,
telemetry 0.2 ms, so every odometry read queued behind the current capture. Splitting it
took three rounds, each fixing a different cause:

| | `/fmu/out/vehicle_odometry` |
|---|---|
| one connection, one AirSim client | 1.5 Hz |
| + separate media client | ~12 Hz |
| + separate control client | ~12 Hz (no change — see below) |
| + schedule-anchored rate guard | **19.5 Hz** against a 20 Hz target |

The sidecar holds three AirSim clients — telemetry, control, media — each behind its own
lock, one thread per connection; the bridge node opens one connection for each with
separate callback groups. `tests/test_offline.py` asserts the three sets stay disjoint.

**The last row was the real one, and it was self-inflicted.** A naive rate guard
(`next = now + period`) re-anchors on the moment the callback happened to run, so ordinary
scheduling jitter is added to every interval and compounds — a 20 Hz timer settled at
11.8 Hz. `_advance()` anchors on the previous *deadline* instead, and resyncs only when it
falls more than one period behind. The guard is still needed: rclpy replays missed timer
firings as a burst after a long blocking call, and the sidecar blocks for seconds on
`reset`/`goto`/`spawn_traffic`.

## DDS domain isolation is mandatory here

This testbed publishes **PX4-shaped topics on purpose**, and `drone-sim` on the same machine
publishes **real PX4 topics** through a uXRCE-DDS agent. Both default to `ROS_DOMAIN_ID=0`,
so the two graphs merge:

```
$ ros2 topic info /fmu/out/vehicle_odometry --verbose
Publisher count: 2
  Node name: carla_air_bridge                 <- ours, 20 Hz
  Node name: _CREATED_BY_BARE_DDS_APP_        <- drone-sim's XRCE agent, ~125 Hz
```

Every rate measured before this was discovered was the **sum of two aircraft in two
simulators**. Worse in the other direction: our `/fmu/in/trajectory_setpoint` was visible to
a live PX4 SITL instance.

`scripts/bringup.sh`, `scripts/status.sh` and `scripts/run_episode.py` all export
`ROS_DOMAIN_ID=42` (override with `TESTBED_ROS_DOMAIN_ID`). **Anything else that talks to
this testbed must set it too**, or it will silently attach to the wrong graph.

For the same reason `scripts/stop.sh` matches only processes under this repository's install
path: drone-sim has packages called `control`, `evaluation` and `vlm_client` too, and a
name-based `pkill` there would kill a running flight gate in the other project.

## Why the topics are PX4-shaped

The simulator has **no PX4 in it** — no MAVLink, no uORB, no lockstep. The flight controller
is AirSim's SimpleFlight. Publishing `/fmu/out/vehicle_odometry` anyway is a deliberate
shim, and it buys one thing: the VLM, grounding, control and evaluation nodes are written
against the same interface a real Pixhawk 6C exposes over uXRCE-DDS. Moving to hardware
means deleting `carla_air_bridge` and starting `uxrce_dds_client` — not rewriting the stack.

`px4_msgs` is pinned to the same SHA as `drone-sim` (`392e831c`, `release/1.16`) so the two
projects speak identical definitions.

**What the shim does not give you**, and must not be reported as if it did:

- `timestamp` is host time, not a PX4 boot clock synced over `timesync_status`. Latency
  numbers measure this bridge, not a flight controller.
- `arming_state` follows AirSim's API-control flag. No failsafe state machine runs behind
  it, so nothing here exercises PX4's arming logic.
- `TrajectorySetpoint` acceleration and jerk are ignored; SimpleFlight has nowhere to put
  them.
- QoS matches PX4's (best-effort, transient-local, depth 5) on purpose, so a node that
  works here does not fail on hardware over a QoS mismatch.

## Configuration: one source, two rendered files

`configs/testbed.yaml` is the only file anyone edits. Two of the three formats involved are
not ours to change — AirSim's `settings.json` is read by the CarlaUE4 binary at launch, and
the parameter file is read by rclpy's parameter system — so the unification happens one level
up: one source, rendered by `scripts/apply_config.py` into what each reader insists on.

```
configs/testbed.yaml  ─┬─▸  configs/sim/settings.json                AirSim's schema
                       └─▸  ros2_ws/src/bringup/config/testbed.yaml  rclpy's schema
```

Both carry a DO-NOT-EDIT header. `run_sim.sh` and `bringup.sh` render before every start, and
`apply_config.py --check` regenerates into memory and diffs, so drift fails a test rather than
a flight.

**The ROS target is the path the launch file already defaults to**, and that is not
incidental. Rendering anywhere else leaves a second parameter file that
`ros2 launch bringup testbed.launch.py` silently prefers over the generated one — which is
exactly what happened for three days after the unification claimed to be done, and is why the
render target is a package path rather than a tidier `configs/generated/`.

Sections are named for **when a change takes effect** — `simulator:` costs a ~60 s restart,
`sensors:`/`sidecar:` ~5 s, `graph:` is live via `ros2 param set` — because that is the
distinction that costs time, and a flat file hides it.

## What a ROS 2 client can reach

A node importing only `rclpy`, `px4_msgs` and `sensor_msgs` can fly the aircraft and read
every instrument. `examples/ros2_full_control.py` does exactly that and is the executable
version of this table.

| | topic | message |
|---|---|---|
| **takeoff / land** | `/fmu/in/vehicle_command` | `VehicleCommand` (`NAV_TAKEOFF` 22, `NAV_LAND` 21) |
| **position** | `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint`, position set, velocity NaN |
| **velocity** | `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint`, velocity set — velocity wins over position |
| **attitude** | `/fmu/in/vehicle_attitude_setpoint` | `VehicleAttitudeSetpoint`, quaternion `q_d` |
| odometry | `/fmu/out/vehicle_odometry` | `VehicleOdometry` |
| IMU | `/fmu/out/sensor_combined` | `SensorCombined` |
| barometer | `/fmu/out/vehicle_air_data` | `VehicleAirData` |
| magnetometer | `/fmu/out/vehicle_magnetometer` | `VehicleMagnetometer` |
| GPS | `/fmu/out/sensor_gps` | `SensorGps` |
| semantic LiDAR | `/sensors/lidar/points` | `PointCloud2`, with object id and tag per point |
| cameras | `/camera/{rgb,depth}/image_raw` | `Image` + `CameraInfo` |

**Two traps that only a real run exposes**, both found that way:

- **The autonomy loop outranks a manual command.** `offboard_control` publishes its own
  setpoints at 10 Hz and wins: a takeoff commanded to 35 m settled at 15.6 m until the
  example disabled that node via `SetParameters` and restored it on exit. Nothing about the
  message was wrong; the graph was fighting the command.
- **Zeros are not "no opinion".** A `TrajectorySetpoint` with zeroed velocity means *hold
  still*, not *ignore velocity*. Unused fields must be NaN.

**What is NOT reachable from ROS 2**, and is the subject of R-01: world control. `reset`,
`spawn_traffic`, `set_weather`, `destroy_actors`, `collision` and the chase camera exist only
as sidecar RPC methods, which is why `scripts/run_episode.py` and `webui/server.py` both still
open the Unix socket directly.

## The loop

See-Point-Fly's shape is a slow generator over a fast controller, and every node boundary
here exists to keep those two decoupled.

1. **`carla_air_bridge`** publishes frames and odometry.
2. **`vlm_client`** hands a frame and an instruction to a backend and publishes an
   `Annotation2D` — a *pixel*. The model never sees a pose, a map or metres. Frames arriving
   during inference are **dropped, not queued**: a VLM answering a four-second-old frame is
   worse than one answering the current frame late.
3. **`grounding`** turns that pixel into a world point using the depth frame whose stamp
   matches the annotated image, so a slow backend is still grounded correctly. Sky grounds
   to `valid=false` rather than being dropped — "the model pointed at the sky" is a result
   the episode log needs.
4. **`control`** streams `/fmu/in/trajectory_setpoint` at 10 Hz toward a **bounded step**
   along the ray, then re-observes. It never stops sending: a gap with no setpoint is
   exactly when the vehicle runs away.
5. **`evaluation`** scores the episode and writes one JSON per run.

### The three geometry facts everything rests on

- **Use the camera pose, not the vehicle pose.** The camera is gimbal-less but its pitch is
  real; a yaw-only rotation silently puts every waypoint on the horizon.
- **`DepthPerspective` is planar z-depth, not ray length.** Treating it as a range shortens
  every waypoint by sec(angle off-axis) — 40% at the edge of a 90° lens.
- **The grounded point is on a surface.** Commanding the full displacement is commanding a
  controlled flight into a building; the conformance suite did exactly that once and hit
  `BP_Block13NY_Top_C_1024`. Hence `standoff_m` and `max_step_m`.

Measured end to end: annotate a pixel at 64.3 m, fly 20 m along the ray, depth at that pixel
reads 47.1 m against 44.1 m predicted. That ~3 m residual is the accuracy budget everything
downstream inherits.

## Rendering: put it on the actual GPU

**The system NVIDIA Vulkan ICD on this box is broken, and the failure is silent.**
`/usr/share/vulkan/icd.d/nvidia_icd.x86_64.json` was injected from the Fedora host and keeps
the host's library path — `/usr/lib64/libGLX_nvidia.so.0` — which does not exist inside this
Ubuntu container (the driver is at `/lib/x86_64-linux-gnu/`). Two failure modes follow, and
the second one is the expensive one:

| `VK_ICD_FILENAMES` / `VK_DRIVER_FILES` | what happens |
|---|---|
| the system NVIDIA ICD | UE dies in Vulkan init. Exit 1, **zero bytes of log**, no crash dump. Looks like a corrupt install. |
| unset | loader falls back to **lavapipe, the LLVM software rasteriser**. Everything works — on the CPU, with the RTX 3080 at 111 MiB and 0%. |
| `configs/vulkan/nvidia_icd.container.json` | correct: 3.3 GB VRAM, hardware rendering |

The silent fallback ran undetected through the entire first build of this testbed. It was
found by checking `nvidia-smi` during a capture loop (0% utilisation, 111 MiB) and confirmed
in `/proc/<pid>/maps`: **zero `/dev/nvidia*` file descriptors and `libvulkan_lvp.so`
mapped**, with the process pegging 2.5 CPU cores at idle.

`scripts/run_sim.sh` exports the corrected ICD and **checks VRAM after startup**, warning if
GPU 0 is under 1 GB — because nothing else about software rendering announces itself.

## Resolution, and why depth is smaller than RGB

Only `ImageType 0` honours the configured resolution; everything else falls back to
AirSim's 256×144 default — a *different aspect ratio*, so even scaling the pixel index is
wrong. But the obvious fix, giving every buffer RGB resolution, is unusable — and **not for
the reason it first appears**:

| config | software (lavapipe) | NVIDIA GPU | speedup |
|---|---|---|---|
| RGB 640×480 | 5.95 Hz | **53.8 Hz** | **9.0×** |
| RGB + depth 320×240 | 2.05 Hz | **3.98 Hz** | 1.9× |
| RGB + depth + seg 320×240 | 1.43 Hz | **1.96 Hz** | 1.4× |
| RGB + depth **640×480** | 0.29 Hz | 0.31 Hz | **1.05× — no change** |
| RGB + depth + seg 640×480 | 0.14 Hz | 0.15 Hz | 1.07× |

Read the last two rows: a 640×480 depth grab costs ~3.2 s **whether or not there is a GPU**.
Rendering is not what makes depth expensive. The cost is the `pixels_as_float=True`
readback — 307k float32 values marshalled through msgpack-rpc into a Python list — and the
GPU does not touch it.

So the two levers are different things:
* **RGB throughput is rendering-bound.** Hardware rendering buys 9×.
* **Depth and segmentation throughput are transport-bound.** Only shrinking them helps.

Hence `simulator.cameras` in `configs/testbed.yaml`: RGB 640×480, depth 160×120,
segmentation 320×240 — 4:3
throughout so the pixel scale is an exact 2:1, and small enough that the readback stays
affordable. The requirement is **equal aspect ratio, not equal resolution**.

## Speed: real-time exactly, and no way to go faster

Measured with the full graph running — captures, control and all:

```
wall 20.020 s | CARLA clock 20.017 s (RTF 1.000) | AirSim clock 20.019 s (RTF 1.000) | 60.0 FPS
```

Both clocks track wall time exactly, so nothing here is running in slow motion, and episode
timings mean what they say. The simulator is also **not** hardware-limited: GPU 0 sits around
30% of a 10 GB card and the process uses 2.8 of 24 CPU cores. The 60 FPS is UE's own default
frame smoothing, not a bottleneck.

**That headroom cannot be turned into faster sweeps.** AirSim exposes `ClockSpeed` for
faster-than-real-time physics; setting it to 3.0 does exactly what it says, and only to half
the world:

| clock | ClockSpeed 1.0 | ClockSpeed 3.0 |
|---|---|---|
| AirSim (the aircraft) | RTF 1.00 | **RTF 3.00** |
| CARLA (traffic, pedestrians, weather) | RTF 1.00 | **RTF 1.00** |
| UE render | 60 FPS | 60 FPS |

The drone's physics accelerates and the ground world does not, so the two halves of
CARLA-Air's "one world" desync — a car that took 4 s to cross a junction now takes 12 s of
drone time. For air-ground scenarios, which is the entire reason to use this simulator, that
makes results meaningless. The ROS 2 graph is wall-clock driven too, so the VLM would also
get a third of the sim-time budget to think in.

`simulator.clock_speed` stays at 1.0. **N-seed sweeps here run in real
time, and that is the throughput ceiling** — budget roughly (episode timeout x seeds) of
wall clock. Faster-than-real-time evaluation needs a lane where one clock drives everything;
Gazebo does this and CARLA-Air cannot.

## Known limits of the substrate

These are properties of CARLA-Air, not of this code. Each is worked around in exactly one
place so the workaround cannot drift.

| Behaviour | Where it is handled |
|---|---|
| Vehicle runs away after `reset()` — 7 m/s climb in one config, sink in another | `Vehicle.reset()` always leaves it holding a setpoint |
| Station-keeping is loose: ~4 m relaxation after reaching a setpoint | `arrival_radius_m: 2.0`; scenarios use a 20 m success radius |
| Traffic-manager vehicles stall constantly (4–11 of 15 moving; watchdog restarts 4–9 per tick) | `World.tick_watchdog()`, called at 1 Hz by the bridge |
| Pedestrians need an AI controller and a destination | `World.spawn_traffic()` |
| AirSim NED origin on Town10HD is **offshore** | `frames.carla_to_ned()`; scenarios are in NED |
| NED altitude is **not** AGL — the origin sits 27.45 m above the street, so `z = -50` is 77.45 m over the ground. Every original scenario flew above the whole city and no "obstacle" was ever in the path | `scripts/survey_buildings.py --check` reports it per scenario |
| The controller's `[15, 120] m` altitude clamp is in NED, so the real floor is **42.45 m AGL** | `tests/test_scenarios.py` lints it; `--propose` only searches legal altitudes |
| Every camera buffer must share ONE aspect ratio: `FOV_Degrees` is horizontal, so a 16:9 RGB buffer against a 4:3 depth buffer covers a different vertical field and `frames.scale_to()` silently samples the wrong depth pixel. 16:9 would also narrow VFOV 73.7deg -> 58.7deg, pushing the level-flight row from 13.7% of the frame to 1.5% | `simulator.cameras` in `configs/testbed.yaml`, all 4:3; `tests/test_config.py` enforces it |
| ROS-side python packages must go to **3.12**, not the 3.10 `.venv` — `pip install anthropic` into the wrong interpreter fails as a `ModuleNotFoundError` that reads like a missing package | `scripts/fetch_vendor.sh` installs into `vendor/py312`; `bringup.sh` appends it to `PYTHONPATH` |
| System NVIDIA Vulkan ICD has a host-only library path: pinning it kills the sim, leaving it unset silently falls back to CPU rendering | `scripts/run_sim.sh` exports `configs/vulkan/nvidia_icd.container.json` and checks VRAM after startup |
| `pkill -f` matches the calling shell's own command line | `scripts/stop.sh` |

Full evidence for each: [`worklog/2026-07-31-carla-air-probe-run.md`](worklog/2026-07-31-carla-air-probe-run.md).

## Adding a VLM backend

Implement `vlm_client.backends.base.VlmBackend` — one method, image and instruction in,
pixel out — and register it in `BACKENDS` in `vlm_node.py`. Four ship today:

- `mock` — seeded random pixels. The floor.
- `scripted` — a fixed pixel sequence. The regression backend: a change in episode outcome
  is unambiguously a change in the stack, not the model.
- `geometric` — steers toward the most open column in the depth band, with a left/right
  keyword bias. **The baseline a real VLM has to beat to be worth its latency.**
- `claude` — the Anthropic API, and the first real model in the loop. Sees the same frame
  and instruction as the three above, which is what makes its score comparable. Three
  things about it are worth knowing before reading a number from it:
  - The reply is **schema-constrained** (`output_config.format`), so the pixel arrives as an
    integer rather than something parsed out of prose. The schema cannot express numeric
    bounds, so `u`/`v` are clamped on our side.
  - It runs at **`effort: low`** by default. This is a control loop: 40 steps in 300 s is
    7.5 s per decision, and a higher effort can spend that on one call — which turns every
    episode into a timeout that measures the budget, not the navigation.
  - Its SDK is a **python 3.12** dependency in `vendor/py312`, *not* the 3.10 `.venv`. That
    is the same interpreter seam as everything else here, wearing a different hat.

  The key comes from `ANTHROPIC_API_KEY` and is deliberately not a ROS parameter — those are
  readable from the graph and are written into launch logs, which this repo commits.

And one that is **not a backend to compare against**:

- `oracle` — projects the episode goal into the image and annotates that pixel, i.e. flies
  straight at it. It is handed the goal and camera pose through a side channel, so it
  deliberately breaks the contract above and must never appear in a table beside a real
  model. Its job is to answer a question no other backend can: *is this scenario navigable
  at all?* An oracle failure means the scenario is broken and every number ever collected
  from it is noise. Run it on any new scenario before trusting a single result.

A backend that needs more than `(image, instruction)` is not answering the same question as
the others, and the comparison stops being fair.
