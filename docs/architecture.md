# Architecture

A VLM navigation testbed over ROS 2, on CARLA-Air. Two processes, one ROS 2 graph, and a
deliberate seam between them.

```
┌─ Python 3.10 ────────────────┐          ┌─ Python 3.12 / ROS 2 Jazzy ──────────────────┐
│                              │          │                                              │
│  CARLA-Air (UE4, headless)   │          │   carla_air_bridge                           │
│    :2000  CARLA RPC          │          │     /fmu/out/vehicle_odometry      14 Hz     │
│    :41451 AirSim RPC         │          │     /camera/rgb/image_raw         2.7 Hz     │
│         ▲                    │          │     /camera/depth/image_raw                  │
│         │ carla + airsim     │          │           │                                  │
│  sim_bridge/server.py        │          │           ▼                                  │
│    ├─ telemetry AirSim client│◄────────►│   vlm_client   ── /vlm/annotation   0.9 Hz    │
│    └─ media AirSim client    │  UDS +   │           │       (pluggable backend)         │
│         Vehicle · Camera     │ msgpack  │           ▼                                  │
│         World  (traffic,     │  two     │   grounding ── /vlm/grounded_waypoint         │
│                weather)      │  conns   │           │       (pixel + depth → NED)       │
│                              │          │           ▼                                  │
└──────────────────────────────┘          │   control ── /fmu/in/trajectory_setpoint 10Hz │
                                          │           │                                  │
                                          │   evaluation ── /episode/{status,result}      │
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

Hence `configs/sim/settings.json`: RGB 640×480 with depth and segmentation at 320×240 — 4:3
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

`ClockSpeed` stays at 1.0 in `configs/sim/settings.json`. **N-seed sweeps here run in real
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
| System NVIDIA Vulkan ICD has a host-only library path: pinning it kills the sim, leaving it unset silently falls back to CPU rendering | `scripts/run_sim.sh` exports `configs/vulkan/nvidia_icd.container.json` and checks VRAM after startup |
| `pkill -f` matches the calling shell's own command line | `scripts/stop.sh` |

Full evidence for each: [`worklog/2026-07-31-carla-air-probe-run.md`](worklog/2026-07-31-carla-air-probe-run.md).

## Adding a VLM backend

Implement `vlm_client.backends.base.VlmBackend` — one method, image and instruction in,
pixel out — and register it in `BACKENDS` in `vlm_node.py`. Three ship today:

- `mock` — seeded random pixels. The floor.
- `scripted` — a fixed pixel sequence. The regression backend: a change in episode outcome
  is unambiguously a change in the stack, not the model.
- `geometric` — steers toward the most open column in the depth band, with a left/right
  keyword bias. **The baseline a real VLM has to beat to be worth its latency.**

And one that is **not a backend to compare against**:

- `oracle` — projects the episode goal into the image and annotates that pixel, i.e. flies
  straight at it. It is handed the goal and camera pose through a side channel, so it
  deliberately breaks the contract above and must never appear in a table beside a real
  model. Its job is to answer a question no other backend can: *is this scenario navigable
  at all?* An oracle failure means the scenario is broken and every number ever collected
  from it is noise. Run it on any new scenario before trusting a single result.

A backend that needs more than `(image, instruction)` is not answering the same question as
the others, and the comparison stops being fair.
