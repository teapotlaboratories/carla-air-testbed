# carla-air testbed

A **VLM navigation testbed over ROS 2**, built on CARLA-Air v0.1.7. It reproduces the
See-Point-Fly loop — *frame → 2D annotation → 3D displacement → velocity setpoint* — against
a photorealistic city with live traffic and weather, **headless, on one GPU, with no
containers**.

The autonomy nodes talk to `/fmu/out/*` and `/fmu/in/*` exactly as they would to a real
Pixhawk 6C, so they port to hardware by deleting one node rather than rewriting the stack.

```
sim_bridge (py3.10) ──UDS/msgpack──> carla_air_bridge ──> vlm_client ──> grounding ──> control
  carla + airsim                      /fmu/out/*           /vlm/          /vlm/         /fmu/in/
  traffic + weather                   /camera/*            annotation     grounded_wp   trajectory_setpoint
```

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
see [Running outside a container](#running-outside-a-container).

```bash
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions \
                 ros-jazzy-cv-bridge git curl unzip
```

## Install

```bash
git clone <this repo> carla-air_testing && cd carla-air_testing

bash scripts/setup_env.sh          # ~2 min   CPython 3.10 venv (uv, no conda)
bash scripts/fetch_release.sh      # ~5 min   6.85 GB simulator -> 18 GB unpacked
bash scripts/fetch_vendor.sh       # ~30 s    px4_msgs pinned to 392e831c
bash scripts/build_ros.sh          # ~3 min   colcon: px4_msgs + 7 packages

export CARLAAIR_RELEASE=<path fetch_release.sh printed>   # add to your shell profile
./.venv/bin/python -m pytest tests/ -q                     # 50 passed, no sim needed
```

## Run an example

```bash
# terminal 1 — simulator + the whole ROS 2 graph (~90 s to be ready)
./scripts/bringup.sh --backend oracle

# terminal 2 — one scored episode
./.venv/bin/python scripts/run_episode.py --scenario cross_the_plaza --seeds 1
#   -> SUCCESS  18.6 m from goal, 14 steps

# always, when the test is done
./scripts/stop.sh --all && ./scripts/status.sh
```

Results land in `out/episodes/*.json`; a sweep also writes `out/sweep-<scenario>.json`.

Other entry points:

```bash
./scripts/bringup.sh --backend geometric      # the no-language baseline
./.venv/bin/python scripts/record_flight.py 60 # -> out/flight.mp4
./scripts/run_conformance.sh                   # is the simulator still behaving? ~15 min
./scripts/status.sh --rates                    # what is running, and how fast
```

## Results

**Oracle 4/4, geometric 0/4** — the scenarios are navigable, the harness reaches goals when
something points at them, and a depth-following heuristic with no language understanding
reaches none. That gap is the space a VLM has to fill.

| scenario | oracle | geometric |
|---|---|---|
| `cross_the_plaza` | **SUCCESS** 18.6 m, 14 steps | FAILURE 41.3 m |
| `follow_the_avenue` | **SUCCESS** 19.6 m, 16 steps | FAILURE 188.0 m |
| `rain_descent` | **SUCCESS** 13.4 m, 8 steps | FAILURE 71.8 m |
| `avoid_the_block` | **SUCCESS** 18.2 m, 9 steps | FAILURE 173.6 m |

Measured with the full graph running:

| | |
|---|---|
| `/fmu/out/vehicle_odometry` | 20.0 Hz |
| `/camera/{rgb,depth}/image_raw` | 7.8 Hz |
| `/vlm/annotation` | 0.9 Hz — a slow model is the design point |
| `/fmu/in/trajectory_setpoint` | 10.0 Hz |
| Grounding accuracy | ~3 m residual on a 64 m ray |
| Real-time factor | **1.000** on both clocks, 60 FPS |

Evidence: [`docs/worklog/`](docs/worklog/).

## Layout

```
sim_bridge/            python 3.10, no ROS — owns the carla + airsim clients
  carla_air/           frames · vehicle · camera · world  (every workaround lives here once)
  protocol.py          the wire format, imported by BOTH interpreters
  server.py            UDS server, 3 AirSim clients, thread per connection

ros2_ws/src/
  interfaces/          Annotation2D · GroundedWaypoint · EpisodeStatus · EpisodeResult
  carla_air_bridge/    the only node that knows the simulator exists
  vlm_client/          backends: mock · scripted · geometric · oracle
  grounding/           pixel + depth → world NED  (the See-Point-Fly transform)
  control/             offboard setpoint streamer
  evaluation/          seeded episode runner and scoring
  bringup/             launch + one parameter file

configs/               sim/settings.json · vulkan/nvidia_icd.container.json
scripts/               setup · fetch_release · fetch_vendor · build · run_sim · bringup
                       · stop · status · run_episode · run_conformance · record_flight
tests/                 test_offline.py + test_scenarios.py (50, no sim) · conformance/ (needs sim)
docs/                  architecture · references · worklog
```

Nothing installs into `~` or the system: `vendor/` holds uv, the standalone CPython and
`px4_msgs`; `.venv/` the packages; the 18 GB simulator lives wherever `CARLAAIR_RELEASE`
points.

## Adding a VLM backend

Implement one method — image and instruction in, pixel out — and register it in `BACKENDS`
in `vlm_client/vlm_node.py`. The model never sees a pose, a map or metres; all the geometry
happens downstream in `grounding`, which is what keeps backends swappable and the comparison
fair.

Four ship today: `mock` (seeded random), `scripted` (fixed pixels, for regression),
`geometric` (steers to the most open depth column — the baseline to beat), and `oracle`
(**a diagnostic, not a competitor** — it is handed the goal, so it validates scenarios and
must never be reported beside a real model's score).

## Running outside a container

This was developed inside a distrobox on a Fedora host, but **native Ubuntu 24.04 is the
simpler target** — one workaround exists purely because of the container and disappears
without it.

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

## What this testbed is and is not good for

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
