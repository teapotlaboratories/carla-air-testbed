# Quick start

From nothing to a scored autonomous flight. **~40 minutes**, most of it the 6.85 GB
download and the 18 GB unpack.

If something goes wrong, jump to [Troubleshooting](#troubleshooting) — the three failures
you are most likely to hit are all silent, and all listed there.

---

## 0. Prerequisites

| | |
|---|---|
| **OS** | Ubuntu 24.04 (Noble). ROS 2 Jazzy has no packages for other releases. |
| **GPU** | NVIDIA, Vulkan-capable, **≥ 6 GB VRAM**. The simulator uses ~3.3 GB on Town10HD. |
| **Driver** | 535+, with `nvidia-smi` working. |
| **Disk** | ~30 GB for the simulator, ~1 GB for the project. |
| **RAM** | 16 GB minimum. |
| **Display** | **None.** Everything runs headless via `-RenderOffScreen`. |

You do **not** need Docker, conda, or a display server.

```bash
# ROS 2 Jazzy + build tools
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions \
                 ros-jazzy-cv-bridge git curl unzip

# sanity check — all three must succeed
nvidia-smi                                   # driver alive
source /opt/ros/jazzy/setup.bash && ros2 --help >/dev/null && echo "ros2 ok"
ls /usr/share/vulkan/icd.d/nvidia_icd*.json  # a Vulkan ICD exists
```

## 1. Install — one command

```bash
git clone <this repo> carla-air_testing && cd carla-air_testing
./scripts/install.sh                       # ~10 min
```

Four steps, printed as it goes: the CPython 3.10 venv (~2 min, uv, no conda), the 6.85 GB
simulator unpacked to 18 GB (~5 min), the pinned upstreams — `px4_msgs` at `392e831c` plus the
anthropic SDK (~40 s) — and the colcon build of 7 packages (~3 min).

**It is resumable.** Every step is idempotent, so if one fails, fix it and re-run: completed
steps are skipped and you land back where you stopped.

Put the simulator somewhere with room, and it is remembered — **no `export` to copy into your
shell profile:**

```bash
./scripts/install.sh /mnt/big-disk         # needs ~30 GB free
```

```bash
./scripts/install.sh --skip-release        # everything but the download
```

Verify without touching the simulator:

```bash
./.venv/bin/python -m pytest tests/ -q     # expect: 161 passed, 1 skipped
```

Why a separate Python: the CARLA-Air client is an ABI-tagged `cpython-310` extension and
ROS 2 Jazzy is 3.12. Neither interpreter can load the other's C extensions, so the project
is deliberately two processes. See [`docs/architecture.md`](docs/architecture.md).

### In a hurry? One command

If all you want is to watch a VLM fly and get a video out of it, skip to the end:

```bash
./scripts/demo.sh                      # street_level, claude backend, seed 5
./scripts/demo.sh --scenario cross_the_plaza --backend geometric --seed 2
```

It brings the simulator up, starts the VLM, flies one episode, combines the chase camera with
the drone's own view and the depth buffer into `out/demo/<episode>.mp4`, and **stops
everything on the way out** — including on Ctrl-C. Roughly 5 minutes end to end.

The rest of this guide is the same thing taken apart, which is what you want the first time
something does not work.

## 2. Start the simulator

**Terminal 1.** This is the whole product: a drone, a city, and a ROS 2 interface.

```bash
./scripts/bringup.sh --config configs/testbed.yaml
```

Wait for these three lines (~90 s, mostly Unreal loading the map):

```
GPU 1 (NVIDIA GeForce RTX 5060 Ti): 3342 MiB in use — hardware rendering confirmed
sim_bridge up on /tmp/carla_air_testbed.sock
[bridge_node-1] bridged to CARLA-Air — map=Town10HD hfov=89.9deg
```

> **`3342 MiB` matters.** If it says a few hundred MiB, you are rendering on the CPU —
> see [Troubleshooting](#troubleshooting).

The index is whichever card `simulator.gpu` asked for — `1` in the shipped config, because
GPU 0 is the workstation's display card. **On a single-GPU machine set `simulator.gpu: 0`**
(or `null` to let the driver choose) before the first run.

**Nothing is flying yet, and nothing is looking at the camera.** What you have is four nodes:

```bash
ros2 node list        # needs ROS_DOMAIN_ID=42
# /carla_air_bridge  /offboard_control  /episode_runner  /recorder
```

That is already enough to fly it from your own code — takeoff, waypoints, attitude, and every
sensor, all over ROS 2:

```bash
python3 examples/ros2_full_control.py     # takeoff -> waypoint -> attitude -> land
python3 examples/ros2_world_control.py    # traffic, weather, teleport, teardown
```

**If that is what you came for, you are done.** Steps 3 and 4 are one example of what to
build on top.

## 3. Start the VLM engine — optional

**Terminal 2.** The See-Point-Fly loop: look at the frame, point at a pixel, turn it into a
waypoint. It is an *example*, not part of the simulator — it starts separately and talks only
to the public ROS 2 interface, exactly as your own navigation code would.

```bash
./examples/vlm_navigation/run.sh --backend oracle
```

This adds two nodes and nothing else:

```
/vlm_client   frame + instruction -> a pixel        (/vlm/annotation)
/grounding    pixel + depth       -> an NED point   (/control/waypoint)
```

Backends: `oracle` (handed the goal — a diagnostic, not a competitor), `geometric` (steers
toward the most open depth column), `claude`, `mock`, `scripted`. Config is
[`examples/vlm_navigation/config/vlm.yaml`](examples/vlm_navigation/config/vlm.yaml) — the
simulator's own config knows nothing about any of it.

**Skip this step entirely** if you are driving the aircraft yourself. Nothing in step 2
depends on it.

## 4. Fly a scored episode

**Terminal 3.** Needs step 3 running — an episode scores whatever is producing waypoints, so
with no engine the aircraft simply sits there.

```bash
./scripts/run_episode.sh --scenario cross_the_plaza --seeds 1
```

Expected output:

```
=== cross_the_plaza seed=1 ===
  traffic: 30 vehicles, 19 walkers
  start pose: [112.7, -167.1, -55.8] (commanded [107.6, -159.4, -55.0], error 9.3 m)
  episode running [cross_the_plaza-s1-48ec6f] — 'fly across the open plaza and stop above the far side'
    alt  54.8 m  pos [122.8, -173.4, -54.8]
    alt  54.9 m  pos [146.5, -168.3, -54.9]
    alt  55.0 m  pos [170.3, -163.1, -55.0]
  -> SUCCESS  18.0 m from goal, 14 steps

=== cross_the_plaza: 1/1 succeeded (100%) ===
```

The result lands in `out/episodes/<episode_id>.json`. Run it again with different `--seeds`
or `--scenario` — steps 2 and 3 stay up, and that is the point of the split: 90 s of Unreal
loading once, not once per episode.

> **Why a wrapper and not `./.venv/bin/python` here?** `run_episode.py` is a plain ROS 2
> client — it drives the simulator entirely through services and topics, so it runs under
> ROS's python 3.12 rather than the 3.10 venv that owns the carla/airsim clients.
> `run_episode.sh` sources the workspace and sets `ROS_DOMAIN_ID` so neither can be
> forgotten.

**A ~9 m start-pose error is normal.** The aircraft relaxes about 4 m after reaching a
setpoint; that is the station-keeping floor, which is why success radii are 20 m.

## 5. Stop — every time

```bash
./scripts/stop.sh --all
./scripts/status.sh          # every count 0, the sim's GPU back to idle (tens of MiB)
```

**This is a project rule, not tidiness.** The simulator holds 3.3 GB of VRAM while idle, and
a leftover graph *stacks* on the next bringup — two controllers then fight over the aircraft
while `ros2 node list` still looks correct. See
[`.ai/AGENTS.md`](.ai/AGENTS.md#1-stop-every-process-when-a-flight-test-is-done).

## 6. Change something

Everything is configured in **one file**: [`configs/testbed.yaml`](configs/testbed.yaml). Its
three sections are named for **when a change takes effect**, which is the difference that
actually costs you time:

| section | what lives there | cost to change |
|---|---|---|
| `simulator:` | map, GPU, camera buffers, GPS origin, clock speed | restart the simulator, ~60 s |
| `sensors:` | every sensor on the aircraft, both simulators | restart the sidecar, ~5 s |
| `sidecar:` | the chase camera | restart the sidecar, ~5 s |
| `sidecar.traffic:` | cars, pedestrians, walking speed | **live** — re-read on every 1 Hz tick |
| `graph:` | rates, backend, controller limits, recording | **live** — `ros2 param set` |

Try it — put the aircraft somewhere else in the world:

```yaml
# configs/testbed.yaml
simulator:
  gps_origin:
    lat: 51.5074        # London instead of AirSim's default (Redmond, Washington)
    lon: -0.1278
    alt: 11.0
```

```bash
./scripts/bringup.sh --config configs/testbed.yaml                          # renders the config, then starts
./examples/vlm_navigation/run.sh --backend geometric
ros2 topic echo /fmu/out/sensor_gps --once    # now reports a London coordinate
```

### Traffic, and a config you can edit while it runs

`sidecar.traffic:` is re-read on every spawn *and* on every 1 Hz steering tick, so editing it
changes pedestrians that are **already walking**, within a second, with no restart:

```yaml
sidecar:
  traffic:
    vehicles: 15
    walkers: 10
    radius_m: 70.0          # cluster radius around the spawn point
    walker_speed_min: 1.0   # m/s — a stroll
    walker_speed_max: 1.7   # a brisk walk
    walker_arrive_m: 3.0    # how close counts as arrived
    walker_roam_m: 80.0     # how far the next destination may be
```

Verify it live — set both speeds to `0.0`, save, and watch:

```bash
ros2 topic echo /sim/traffic_stats     # walkers_moving drops to 0 within ~1 s
```

Counts here are **defaults**. A scenario's `traffic_vehicles` / `traffic_walkers` and an
explicit `/sim/spawn_traffic` call both override them.

> **Ask for more vehicles than the radius can hold and you get map-wide traffic.** Town10HD
> has ~45 spawn points within 70 m of the plaza; request 60 and the sidecar falls back to
> spreading them across the whole map — and says so in the service reply. An aircraft over an
> empty street usually means this.

### Adding a sensor

All of them are one list, whichever simulator provides them. Flip `enabled` and restart:

```yaml
sensors:
  - name: lidar
    source: carla                  # pushed from inside UE4 - costs the image path nothing
    enabled: true
    blueprint: sensor.lidar.ray_cast_semantic
    topic: /sensors/lidar/points
    offset: {x: 0.0, y: 0.0, z: -0.4}   # metres, body FRD; negative z is ABOVE
    attributes: {channels: "16", range: "80.0", points_per_second: "30000"}

  - name: radar
    source: carla
    enabled: false                 # <- flip this
    blueprint: sensor.other.radar
```

**`source` is not cosmetic.** `airsim` sensors are polled over RPC on the same path as image
capture, and that costs measurable frame rate — 5 Hz costs 1.7%, 20 Hz costs 28%. `carla`
sensors are pushed asynchronously from inside the simulator and cost nothing measurable. The
comments in the file carry the numbers.

`radar`, `events` (an event camera) and `instances` (instance segmentation) are already
written out and disabled, so enabling one is a one-word edit.

### Two files you must not edit

`configs/sim/settings.json` and `ros2_ws/src/bringup/config/testbed.yaml` are **generated** — AirSim's
schema is read by the simulator binary and the parameter file by rclpy, and neither will
accept a unified file. Both carry a DO-NOT-EDIT header, `run_sim.sh` and `bringup.sh` re-render
them on every start, and a test fails if they drift from their source.

---

## What you just ran

`oracle` is not a VLM. It is handed the goal and flies straight at it, so it answers one
question: **is this scenario navigable at all?** On the three open scenarios it is the ceiling
at 5/5 each, while the `geometric` depth-following baseline scores 0/5 — and that gap is the
space a real model has to fill.

It scores **0/5 on `avoid_the_block`**, which is that scenario working as intended: there is a
154 m tower sitting on the straight line, and a straight-line policy cannot get past it. For a
scenario like that the oracle cannot be the validator, so reachability is proven geometrically
instead — `./.venv/bin/python scripts/survey_buildings.py --route` finds a way around at 1.18x
the direct distance.

Try the baseline and see the contrast:

```bash
./scripts/stop.sh
# terminal 1
./scripts/bringup.sh --config configs/testbed.yaml

# terminal 2
./examples/vlm_navigation/run.sh --backend geometric
# then, in terminal 2:
./scripts/run_episode.sh --scenario cross_the_plaza --seeds 1
```

Other things to try:

```bash
# one scenario, three seeds (six scenarios ship; four are scored, two are demos)
./scripts/run_episode.sh --scenario follow_the_avenue --seeds 1 2 3

# record a flight video (out/flight.mp4)
./.venv/bin/python scripts/record_flight.py 60

# the same, but with the chase camera and depth composited in, and cleaned up afterwards
./scripts/demo.sh --scenario follow_the_avenue --backend geometric

# is the simulator still behaving? ~15 min. p06, p07 and p09 are EXPECTED to fail.
./scripts/run_conformance.sh
```

Backends live in `ros2_ws/src/vlm_client/vlm_client/backends/`. Adding one means
implementing a single method — image and instruction in, pixel out — and registering it in
`BACKENDS`.

---

## Troubleshooting

**The simulator starts but everything is slow, and `run_sim.sh` reports a few hundred MiB
of VRAM.**
You are on the **lavapipe software rasteriser**, not the GPU — RGB capture drops from
53.8 Hz to 5.95 Hz. Nothing errors. `run_sim.sh` detects this and regenerates a corrected
Vulkan ICD automatically; if it still happens, check `ldconfig -p | grep libGLX_nvidia` finds
a library that exists. Common inside a distrobox, where the ICD is injected from the host.

**The simulator exits immediately, `out/sim.log` is empty, no crash dump.**
That is what a broken Vulkan ICD looks like. Same fix as above. It is not a corrupt install.

**`bringup.sh` says it cannot reach `sim_bridge`.**
The sidecar could not connect to the simulator. Check `out/sim_bridge.log`, and that ports
2000 and 41451 are listening (`ss -tln | grep -E '2000|41451'`).

**Topic rates look doubled or tripled; two controllers seem to be fighting.**
A previous graph is still running. `./scripts/stop.sh` then bring up again. `status.sh`
shows a count above 1 for the stacked packages.

**`ros2 topic list` shows nothing.**
Set the domain: everything here runs on `ROS_DOMAIN_ID=42`, to stay out of the way of other
ROS graphs on the same machine.

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
```

**A code change has no effect.**
`colcon --symlink-install` still needs a rebuild for `ament_python` packages — the symlink
is made at build time. Re-run `scripts/build_ros.sh`.

**A config change has no effect.**
Three possibilities, in order of likelihood. You edited a **generated** file
(`configs/sim/settings.json` or `ros2_ws/src/bringup/config/testbed.yaml`) instead of
`configs/testbed.yaml` — the next start overwrites it. Or the setting lives in `simulator:`
and needs the simulator restarted, not just the graph. Or an environment variable is
overriding it: `TESTBED_GPU`, `TESTBED_ORIGIN_LAT/LON/ALT` and the `bringup.sh` flags all win
over the file, deliberately, so a one-off run needs no edit.

```bash
./.venv/bin/python scripts/apply_config.py --check   # are the generated files current?
```

**`import carla` fails.**
Use `./.venv/bin/python`, not the system one. The client module is `cpython-310`; the system
Python on Ubuntu 24.04 is 3.12 and physically cannot load it.
