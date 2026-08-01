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

## 1. Install — four commands

```bash
git clone <this repo> carla-air_testing && cd carla-air_testing

bash scripts/setup_env.sh          # ~2 min   CPython 3.10 venv (uv, no conda)
bash scripts/fetch_release.sh      # ~5 min   6.85 GB simulator -> 18 GB unpacked
bash scripts/fetch_vendor.sh       # ~30 s    px4_msgs pinned to 392e831c
bash scripts/build_ros.sh          # ~3 min   colcon: px4_msgs + 7 packages
```

`fetch_release.sh` prints an `export CARLAAIR_RELEASE=…` line at the end. **Run it, and put
it in your shell profile** — every other script reads it.

```bash
export CARLAAIR_RELEASE=/path/it/printed/CarlaAir-v0.1.7
```

Verify without touching the simulator:

```bash
./.venv/bin/python -m pytest tests/ -q     # expect: 50 passed
```

Why a separate Python: the CARLA-Air client is an ABI-tagged `cpython-310` extension and
ROS 2 Jazzy is 3.12. Neither interpreter can load the other's C extensions, so the project
is deliberately two processes. See [`docs/architecture.md`](docs/architecture.md).

## 2. Run the example

**Terminal 1 — bring up the simulator and the whole ROS 2 graph:**

```bash
./scripts/bringup.sh --backend oracle
```

Wait for these three lines (~90 s, mostly Unreal loading the map):

```
GPU 0: 3342 MiB in use — hardware rendering confirmed
sim_bridge up on /tmp/carla_air_testbed.sock
[bridge_node-1] bridged to CARLA-Air — map=Town10HD hfov=89.9deg
```

> **`3342 MiB` matters.** If it says a few hundred MiB, you are rendering on the CPU —
> see [Troubleshooting](#troubleshooting).

**Terminal 2 — fly a scored episode:**

```bash
cd carla-air_testing
./.venv/bin/python scripts/run_episode.py --scenario cross_the_plaza --seeds 1
```

Expected output:

```
=== cross_the_plaza seed=1 ===
  traffic: {'vehicles': 15, 'walkers': 9}
  start pose: [113.0, -167.1, -55.7] (commanded [107.6, -159.4, -55.0], error 9.4 m)
  episode running [cross_the_plaza-s1-331669] — 'fly across the open plaza and stop above the far side'
    alt  54.4 m  pos [122.1, -173.7, -54.4]
    alt  54.6 m  pos [145.8, -168.5, -54.6]
    alt  54.9 m  pos [169.6, -163.3, -54.9]
  -> SUCCESS  18.6 m from goal, 14 steps

=== cross_the_plaza: 1/1 succeeded (100%) ===
```

The result lands in `out/episodes/<episode_id>.json`.

**A ~9 m start-pose error is normal.** The aircraft relaxes about 4 m after reaching a
setpoint; that is the station-keeping floor, which is why success radii are 20 m.

## 3. Stop — every time

```bash
./scripts/stop.sh --all
./scripts/status.sh          # every count 0, GPU back to ~110 MiB
```

**This is a project rule, not tidiness.** The simulator holds 3.3 GB of VRAM while idle, and
a leftover graph *stacks* on the next bringup — two controllers then fight over the aircraft
while `ros2 node list` still looks correct. See
[`.ai/AGENTS.md`](.ai/AGENTS.md#1-stop-every-process-when-a-flight-test-is-done).

---

## What you just ran

`oracle` is not a VLM. It is handed the goal and flies straight at it, so it answers one
question: **is this scenario navigable at all?** Its 4/4 is the ceiling; the `geometric`
depth-following baseline scores 0/4, and that gap is the space a real model has to fill.

Try the baseline and see the contrast:

```bash
./scripts/stop.sh
./scripts/bringup.sh --backend geometric
# then, in terminal 2:
./.venv/bin/python scripts/run_episode.py --scenario cross_the_plaza --seeds 1
```

Other things to try:

```bash
# all four scenarios, three seeds each
./.venv/bin/python scripts/run_episode.py --scenario follow_the_avenue --seeds 1 2 3

# record a flight video (out/flight.mp4)
./.venv/bin/python scripts/record_flight.py 60

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

**`import carla` fails.**
Use `./.venv/bin/python`, not the system one. The client module is `cpython-310`; the system
Python on Ubuntu 24.04 is 3.12 and physically cannot load it.
