# carla-air testbed

A **drone simulator with a ROS 2 interface**, built on CARLA-Air v0.1.7: a quadrotor in a
photorealistic city with live traffic and weather, **headless, on one GPU, with no
containers**. Fly it, sense from it, and script scenarios against it — all over ROS 2.

Your nodes talk to `/fmu/out/*` and `/fmu/in/*` exactly as they would to a real Pixhawk 6C,
so they port to hardware by deleting one node rather than rewriting the stack.

```
sim_bridge (py3.10) ──UDS/msgpack──> carla_air_bridge ──> control ──> /fmu/in/trajectory_setpoint
  carla + airsim                      /fmu/out/*  odometry, IMU, baro, mag, GPS, lidar
  traffic + weather                   /camera/*   rgb · depth · segmentation
                                      /sim/*      reset · traffic · destroy · weather · camera · chase (services)
```

**Vision-language navigation is one thing you can build on it**, not what it is. The
See-Point-Fly loop — *frame → 2D annotation → 3D displacement → velocity setpoint* — ships as
[`examples/vlm_navigation/`](examples/vlm_navigation/), started separately and talking only to
the interface above. Skip it and the simulator is unchanged.

**Scope.** The product is the simulator: a faithful world, faithful sensors, and a ROS 2
interface that behaves the way real hardware would. **Navigation is out of scope** — waypoint
following, obstacle avoidance, pixel-to-world grounding, prompt and model choice, and
benchmark scores for a policy are all things you build *on* this, and they live in
`examples/`. The test for any change: *could a user with a completely different navigation
stack still want it?* Some navigation code still ships here for historical reasons and is
marked as such in [`docs/todo.md`](docs/todo.md).

Two processes because it has to be: the CARLA-Air client is an ABI-tagged `cpython-310`
extension and ROS 2 Jazzy is 3.12, so neither interpreter can load the other's C extensions.

**➡️ New here? [`QUICKSTART.md`](QUICKSTART.md) — install and a scored flight in ~40 minutes.**

---

## Prerequisites

| | |
|---|---|
| **OS** | Ubuntu 24.04 (Noble) — ROS 2 Jazzy has no packages for other releases |
| **GPU** | NVIDIA, Vulkan-capable, **≥ 6 GB VRAM** (the sim uses ~3.3 GB on Town10HD) |
| **Driver** | 535+, `nvidia-smi` working |
| **Disk** | ~30 GB for the simulator, ~1 GB for the project |
| **RAM** | 16 GB minimum |
| **Display** | none — everything is `-RenderOffScreen` |

No Docker, no conda, no display server. It runs on **native Ubuntu and inside a distrobox**;
see [Native Ubuntu vs this distrobox](#native-ubuntu-vs-this-distrobox).

```bash
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions \
                 ros-jazzy-cv-bridge git curl unzip
```

## Install

```bash
git clone <this repo> carla-air_testing && cd carla-air_testing

./scripts/install.sh               # ~10 min, resumable, one command
                                   #   venv -> simulator -> px4_msgs -> colcon build
                                   # pass a directory to put the 18 GB release elsewhere;
                                   # it is remembered, no shell-profile export needed

./.venv/bin/python -m pytest tests/ -q     # 156 passed, 1 skipped — no sim needed
```

## Run an example

```bash
# terminal 1 — the simulator and its ROS 2 graph (~90 s). No VLM node runs.
./scripts/bringup.sh --config configs/testbed.yaml

# at this point you can already fly it from your own code:
#   python3 examples/ros2_full_control.py    takeoff -> waypoint -> attitude -> land
#   python3 examples/ros2_world_control.py   traffic, weather, teleport, teardown

# terminal 2 — OPTIONAL: the See-Point-Fly example, on top of that interface
./examples/vlm_navigation/run.sh --backend oracle

# terminal 3 — one scored episode (needs terminal 2: it scores what produces waypoints)
./scripts/run_episode.sh --scenario cross_the_plaza --seeds 1
#   -> SUCCESS  18.0 m from goal, 14 steps

# always, when the test is done
./scripts/stop.sh --all && ./scripts/status.sh
```

Results land in `out/episodes/*.json`; a sweep also writes `out/sweep-<scenario>.json`.

Other entry points:

```bash
./scripts/bringup.sh --config configs/testbed.yaml                             # the simulator
./examples/vlm_navigation/run.sh --backend geometric   # the no-language baseline
./.venv/bin/python scripts/record_flight.py 60 # -> out/flight.mp4
./scripts/run_conformance.sh                   # is the simulator still behaving? ~15 min
./scripts/status.sh --rates                    # what is running, and how fast

# Does a scenario actually contain an obstacle, or does a straight line solve it?
./.venv/bin/python scripts/survey_buildings.py --check
./.venv/bin/python scripts/survey_buildings.py --propose --span 110
```

## Results

**Oracle 14/19, geometric 0/20** over the three open scenarios plus the blocked one. The
harness reaches goals when something points at them, a depth-following heuristic with no
language understanding reaches none, and one scenario now defeats both. That gap is the space
a VLM has to fill.

Re-measured **2026-08-03 over 40 seeded episodes** — 5 seeds x 4 benchmark scenarios x 2
backends, zero collisions. This supersedes an earlier 50-episode run; both are in
[`docs/todo.md`](docs/todo.md) under `E-01b`, with the difference explained rather than overwritten.

| scenario | straight line? | oracle (N=5) | geometric (N=5) |
|---|---|---|---|
| `cross_the_plaza` | solves it | **5/5** | 0/5 |
| `follow_the_avenue` | solves it | **5/5** | 0/5 |
| `rain_descent` | solves it | **4/5** | 0/5 |
| `avoid_the_block` | **blocked by a tower** | 0/5 | 0/5 |

`rain_descent` slipped from 5/5 to 4/5, failure mode `model_declared_done` — the oracle stopped
short. One episode also missed the deadline, which is why the oracle denominator is 19 rather
than 20. Neither is a model result; both are the harness, and both are recorded rather than
rounded away.

`avoid_the_block` is **meant** to defeat the oracle. The oracle is handed the goal and steers
straight at it, so a scenario built to block the straight line fails it by construction —
`survey_buildings.py --route` proves a way around exists at 1.18x the direct distance. The
other three remain straight-line-solvable, which is honest as a baseline and thin as a
benchmark — see `E-02b` in [`docs/todo.md`](docs/todo.md).

Two further scenarios ship as **demonstrations, not benchmarks**: `busy_street` and
`street_level`, the latter flying at 3.5 m AGL with per-scenario controller overrides. They are
not scored above and no result from them should be read as one.

Measured with the full graph running, **2026-08-04**, at the shipped 960x720 camera:

| | |
|---|---|
| `/fmu/out/vehicle_odometry` | 19.5 Hz |
| `/camera/rgb/image_raw` | 8.1 Hz — 960x720, `bgr8` |
| `/camera/depth/image_raw` | 7.9 Hz — 160x120, `32FC1` |
| `/fmu/in/trajectory_setpoint` | 10.0 Hz |
| Grounding accuracy | ~3 m residual on a 64 m ray |
| Real-time factor | **1.000** on both clocks, 60 FPS |

The camera rate did not fall when the buffer went from 640x480 to 960x720 — 2.25x the pixels
at the same 8 Hz. The bottleneck is the readback and transport, not the render.

Evidence: [`docs/worklog/`](docs/worklog/).

## Documentation

| | |
|---|---|
| [`docs/ros2-api.html`](docs/ros2-api.html) | **Commanding the aircraft from ROS 2** — five commands, twelve sensor streams, message types and code. Every figure measured against the running simulator. |
| [`docs/dataflow.html`](docs/dataflow.html) | **How the data moves** — every protocol hop from UE4 render target to velocity setpoint, and why each one is there. |
| [`docs/rpc-path.html`](docs/rpc-path.html) | **The sidecar RPC path** — where a call lives, how it flows, and how one slow reply desynchronised the stream permanently. Backlog E-06, fixed. |
| [`docs/guide.html`](docs/guide.html) | This README and the quick start, rendered as one page. |
| [`docs/architecture.md`](docs/architecture.md) | What runs where, the measured numbers, and the traps that cost time. |
| [`docs/todo.md`](docs/todo.md) | The backlog: what is open, why, and how each item will be verified. |
| [`docs/worklog/`](docs/worklog/) | Dated accounts of what was tried, what was measured, and what turned out to be wrong. |

The HTML pages are self-contained — no CDN, no fonts, no scripts beyond a theme toggle — so
they open from disk and survive being emailed.

**Want to fly it from your own code?** Start with
[`examples/ros2_full_control.py`](examples/ros2_full_control.py). It takes off, flies a
waypoint, holds a velocity, commands an attitude, lands, and prints every sensor — and imports
nothing from this project, which is the test that the ROS surface stands on its own.

## Layout

```
sim_bridge/            python 3.10, no ROS — owns the carla + airsim clients
  carla_air/           frames · vehicle · camera · world  (every workaround lives here once)
  protocol.py          the wire format, imported by BOTH interpreters
  server.py            UDS server, 4 AirSim clients + a CARLA sensor rig

ros2_ws/src/
  interfaces/          Annotation2D · GroundedWaypoint · EpisodeStatus · EpisodeResult
  carla_air_bridge/    the only node that knows the simulator exists
  vlm_client/          backends: mock · scripted · geometric · oracle · claude
  grounding/           pixel + depth → world NED  (the See-Point-Fly transform)
  control/             offboard setpoint streamer
  evaluation/          seeded episode runner and scoring
  bringup/             launch + config/testbed.yaml (GENERATED from configs/testbed.yaml)

configs/               testbed.yaml (THE source) · sim/settings.json (generated)
                       · vulkan/nvidia_icd.container.json
scripts/               install (one command) · setup · fetch_release · fetch_vendor
                       · build · release_path · run_sim · bringup
                       · stop · status · run_episode · run_sweep · run_conformance
                       · demo (one command to a video) · combine_views
                       · record_flight · survey_buildings
tests/                 offline · scenarios · survey · claude_backend · config · interfaces
                       · sidecar_locks · rpc_correlation · control_limits
                       (156 passed, 1 skipped — no sim, GPU or display) · conformance/
examples/              ros2_full_control.py — fly it from plain ROS 2, no project imports
                       ros2_world_control · ros2_traffic_flyover · ros2_city_tour
                       ros2_street_level · vlm_navigation/ (the optional VLM)
docs/                  ros2-api.html · dataflow.html · rpc-path.html · guide.html
                       architecture · references · worklog · todo
```

Nothing installs into `~` or the system: `vendor/` holds uv, the standalone CPython,
`px4_msgs` and the ROS-side (3.12) python packages in `vendor/py312`; `.venv/` holds the
3.10 packages; the 18 GB simulator lives wherever `CARLAAIR_RELEASE` points.

## Building on it

**Anything that can speak ROS 2.** The aircraft takes `TrajectorySetpoint` (position or
velocity), `VehicleAttitudeSetpoint` and `VehicleCommand` for takeoff/land; the world takes
six services on `/sim/*` — reset, traffic, destroy, weather, camera pose, chase recording. `examples/ros2_full_control.py` and `examples/ros2_world_control.py`
are complete working clients that import nothing from this project.

### The vision-language example

If what you want *is* a VLM navigator, `examples/vlm_navigation/` is a working one and the
extension point is one method — image and instruction in, pixel out — registered in `BACKENDS`
in `vlm_client/vlm_node.py`. The model never sees a pose, a map or metres; all the geometry
happens downstream in `grounding`, which is what keeps backends swappable and the comparison
fair.

Five ship today: `mock` (seeded random), `scripted` (fixed pixels, for regression),
`geometric` (steers to the most open depth column — the baseline to beat), `oracle`
(**a diagnostic, not a competitor** — it is handed the goal, so it validates scenarios and
must never be reported beside a real model's score), and `claude` (the Anthropic API).

`claude` needs credentials in the environment — never a ROS parameter, since those are
readable from the graph and land in launch logs. Either works:

```bash
export ANTHROPIC_API_KEY=sk-ant-...   # from console.anthropic.com
ant auth login                        # or an OAuth profile, if the account has an API org

# terminal 1
./scripts/bringup.sh --config configs/testbed.yaml

# terminal 2
./examples/vlm_navigation/run.sh --backend claude
```

> **A Claude.ai Pro/Max subscription does not cover this.** The subscription is for claude.ai
> and Claude Code; the API is billed separately with its own credits. `~/.claude/.credentials.json`
> is Claude Code's own OAuth token — different audience and scopes — and cannot be used here.
> The backend checks for all three credential sources at construction and says so if it finds
> none, rather than failing on the first frame with the aircraft already airborne.

Its SDK is a **python 3.12** dependency (the ROS side), installed by `fetch_vendor.sh` into
`vendor/py312` — not into the 3.10 `.venv` that owns the carla/airsim clients. Defaults live
in `configs/testbed.yaml`: `claude-opus-5` at `effort: low`, because 40 steps in 300 s
leaves 7.5 s per decision and a higher effort can spend that on a single call. The backend
tallies call count, token spend and p50/p95 decision latency, and logs them on shutdown.

## Native Ubuntu vs this distrobox

Everything here runs **natively** — no Docker, no conda, no display server. It was developed
inside a distrobox on a Fedora host, but **native Ubuntu 24.04 is the simpler target**: one
workaround exists purely because of the distrobox and disappears without it.

- **Vulkan ICD (container-only).** distrobox injects the *host's* ICD JSON, so on a Fedora
  host it names `/usr/lib64/libGLX_nvidia.so.0`, a path that does not exist in an Ubuntu
  container. On native Ubuntu the `nvidia-driver` package ships a correct one.
  `scripts/run_sim.sh` uses the system ICD when it resolves and synthesises a corrected one
  only when it does not, so it works either way with no configuration.
- **Everything else is portable.** The Python 3.10 venv is needed because *Ubuntu 24.04*
  ships 3.12, not because of the container. `ROS_DOMAIN_ID=42` exists to avoid a sibling
  project's ROS graph on this machine — harmless, and worth keeping if you run anything else
  on ROS 2. The `pkill -f` guardrails are universal shell behaviour.

Paths that are machine-specific (`CARLAAIR_RELEASE`, `TESTBED_ROS_DOMAIN_ID`,
`CARLAAIR_HOME`) are all environment variables with defaults.

## What it is and is not good for

**Good for:** developing and benchmarking a VLM navigation client against a photoreal city
with dense traffic, weather and segmentation ground truth. The grounding transform closes,
episodes are repeatable to 0.04 m of trajectory divergence, and it runs on one workstation
GPU with no build step.

**Not good for sim-to-real.** CARLA-Air contains **no PX4** — zero occurrences of `px4`,
`mavlink`, `uxrce` or `/fmu/` in the upstream repository. The `/fmu/*` topics here are a
shim over AirSim SimpleFlight: you get portability of *your* nodes, not a flight controller.
Failsafes, lockstep, EKF2 and arming logic need a different lane.

**Sweeps run in real time.** Both clocks hold RTF 1.000, and faster-than-real-time is not
available — AirSim's `ClockSpeed` accelerates the aircraft while CARLA's world stays at 1×,
so the two halves desync. Budget `timeout × seeds` of wall clock.

The substrate has sharp edges, all measured, each worked around in exactly one place: the
vehicle runs away after `reset()` unless commanded, station-keeping is loose to ~4 m,
traffic-manager vehicles stall constantly, and the AirSim origin on Town10HD is offshore.
[`docs/architecture.md`](docs/architecture.md) lists them with the code that handles each.
