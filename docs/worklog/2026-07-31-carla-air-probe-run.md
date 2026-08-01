# CARLA-Air v0.1.7 — bring-up and probe run

**Date:** 2026-07-31 → 2026-08-01
**Box:** Ubuntu 24.04 container, RTX 3080 (10 GB) + RTX 5060 Ti (16 GB), 24 cores, 62 GB RAM,
NVIDIA 610.43.03, ROS 2 Jazzy installed on the host container.
**Scope:** test whether CARLA-Air can host the VLM-simulation goal from
[`../references/01_sim_stack_architecture.md`](../references/01_sim_stack_architecture.md).
Explicitly **not** building the stack, and explicitly **not** containerised.

---

## What was actually run

Upstream ships a 6.85 GB prebuilt binary on Hugging Face. Everything below is that
binary — `CarlaUE4-Linux-Shipping`, UE4.26 lineage, CARLA 0.9.16 fork — running headless
with `-RenderOffScreen`, driven from a Python 3.10 client environment.

| | |
|---|---|
| Release | `tianlezeng/CarlaAIr-v0.1.7` → `CarlaAir-v0.1.7.zip`, 6,846,384,047 bytes |
| Unpacked | 18 GB at `/var/mnt/…/Developments/projects/carla-air_testing/CarlaAir-v0.1.7` |
| Server version | `adaf011-dirty` (client module reports `aa9c92b` — skew is expected upstream) |
| Map | Town10HD, 155 spawn points |
| Probes | `probes/p01`–`p10`, ~15 min of real-time flight |

**Download note.** Plain `curl` off the HF CDN sustained ~800 KB/s and projected two hours.
`huggingface_hub.hf_hub_download` pulled the same 6.85 GB in ~75 seconds. Use the library,
not curl.

---

## The four things that cost time

### 1. The NVIDIA Vulkan ICD is broken here — and the "fix" made it worse

**Superseded 2026-08-01. The original conclusion in this section was wrong and the
correction is the single most valuable finding in this document.**

What was originally recorded: pinning `VK_ICD_FILENAMES` to the NVIDIA ICD killed the
simulator (0/6 starts, exit 1, **zero bytes** of log, no crash dump), while leaving it unset
worked (6/6). The recommendation was therefore "leave it unset".

That recommendation forced **software rendering**, and it went undetected through the entire
first build of the testbed.

The root cause is one line. `/usr/share/vulkan/icd.d/nvidia_icd.x86_64.json` was injected
into this container from the Fedora host and keeps the *host's* library path:

```json
"library_path": "/usr/lib64/libGLX_nvidia.so.0"     // does not exist in this container
```

In this Ubuntu 24.04 container the driver is at `/lib/x86_64-linux-gnu/libGLX_nvidia.so.0`.
So the loader cannot dlopen the NVIDIA ICD at all:

* **pinned to it** → no usable driver → UE dies during Vulkan init, silently;
* **unset** → the loader enumerates every ICD and falls back to **lavapipe, the LLVM
  software rasteriser**. Everything runs. On the CPU.

Confirmed three ways rather than inferred:

```
$ nvidia-smi   (during a capture loop)     GPU 0: 0 % utilisation, 111 MiB
$ ls -l /proc/<sim>/fd | grep -c nvidia    0
$ grep -oE '.*(nvidia|lvp).*\.so' /proc/<sim>/maps
      /usr/lib/x86_64-linux-gnu/libvulkan_lvp.so      <- lavapipe
$ ps -o %cpu -p <sim>                      251 %  (at idle)
```

The fix is a corrected ICD, `configs/vulkan/nvidia_icd.container.json`, exported as
`VK_DRIVER_FILES`. After it: **3.3 GB VRAM, hardware rendering**.

| capture | software (lavapipe) | NVIDIA RTX 3080 | speedup |
|---|---|---|---|
| RGB 640x480 | 5.95 Hz | **53.8 Hz** | **9.0x** |
| RGB + depth 320x240 | 2.05 Hz | 3.98 Hz | 1.9x |
| RGB + depth + seg 320x240 | 1.43 Hz | 1.96 Hz | 1.4x |
| RGB + depth **640x480** | 0.29 Hz | 0.31 Hz | **1.05x** |

The last row is the one that reframes the rest of this document: a 640x480 depth grab costs
~3.2 s *whether or not a GPU is involved*. Depth and segmentation are bound by the
`pixels_as_float=True` readback — 307k float32 values marshalled through msgpack-rpc — not
by rendering. **RGB throughput is rendering-bound; depth throughput is transport-bound.**
Every multi-buffer number originally recorded here was measured on the CPU, but the shape of
the resolution trade-off survives the correction for that reason.

`scripts/run_sim.sh` now checks VRAM after startup and warns below 1 GB, because nothing
about the software fallback announces itself.

### 2. After `reset()`, the vehicle does not hold station — and how it fails varies

`probes/p09_hover_hold.py`. Armed, under API control, no command issued, the vehicle does
not stay where it was put. **The direction is not stable between sessions:**

- one configuration reproduced a constant **+7.06 m/s climb that never stops**, four times
  in a row — one session reached **-1566 m NED** before anyone looked;
- a later session (after the `settings.json` change and a restart) instead **sank** to
  z = +5.1 m and sat there, climb rate -0.0 m/s.

The invariant across every session is the useful part: **it does not hold, and an explicit
setpoint fixes it.** Station-keeping after a setpoint is loose rather than rigid — 0.10 m
over 9 s in one run, **6.72 m in another**, consistent with the 4 m post-`join()` relaxation
p10 measures.

The 7 m/s climb was initially misread as render-thread contention, because it first showed
up as a 40 m "hover drift under image load". The A/B in p09 kills that reading: idle and
image-hammering gave the same 7.06 m/s.

It matters more here than it would elsewhere. A See-Point-Fly-style loop is *slow* — the
VLM generator runs well under 1 Hz, and every gap between waypoints is a window with no
active setpoint. Any episode-reset path must command a setpoint immediately after
`reset()`; nothing upstream says so.

### 3. Only `ImageType 0` honours the configured resolution — and "just match them" is a trap

Shipped `settings.json` gives `CaptureSettings` for ImageType 0 only. Result: RGB at
1280x960 (4:3), and depth / segmentation / surface-normals silently at AirSim's **256x144**
(16:9) default. A VLM annotates a pixel in the RGB frame; reading depth at that same index
reads a different part of the scene, or throws — and because the aspect ratios differ, even
scaling the index is wrong.

The obvious fix — give every ImageType the same resolution — works and is unusable.
Multi-buffer grab cost, measured over the city, is wildly superlinear in the depth and
segmentation resolution:

| depth / seg resolution | RGB alone | RGB + depth | RGB + depth + seg |
|---|---|---|---|
| 256x144 (shipped default, wrong aspect) | 8.96 Hz | 232 ms · 4.30 Hz | 280 ms · 3.57 Hz |
| 320x240 (**chosen**) | 5.95 Hz | 488 ms · 2.05 Hz | 698 ms · 1.43 Hz |
| 640x480 | 5.65 Hz | 3.4 s · 0.29 Hz | 6.9 s · 0.14 Hz |
| 960x720 (all seven types) | 5.38 Hz | **15.4 s · 0.06 Hz** | **33.5 s · 0.03 Hz** |

RGB-only cost barely moves; the extra buffers are what explode. Confirmed as
config-dependent and not scene-dependent: at 960x720 the RGB+depth grab took 15.4 s both at
the offshore origin and over the city.

So the requirement is **not** equal resolution, it is **equal aspect ratio** — that makes
the RGB→depth pixel scale exact while letting depth stay small and cheap.
`configs/settings.json` ships RGB 640x480 with depth and segmentation at 320x240, all 4:3,
an exact 2:1 scale. `p05` checks the aspect ratio rather than the resolution, and
`depth_at()` applies the scale.

### 4. `pkill -f CarlaUE4-Linux-Shipping` kills the shell that runs it

`-f` matches against full command lines, and the launching script's own command line
contains the string. Three debugging cycles were lost to shells dying with exit 144
(128+SIGTERM). Use `pkill -x "CarlaUE4-Linux-"` — process name, not command line.

---

## Results against the VLM goal

### It works: the See-Point-Fly transform closes

`probes/p05_pixel_to_waypoint.py` runs SPF's core claim with no VLM in the loop — pick a
pixel, read its depth, unproject through the camera pose, fly a bounded step along the ray,
and check the world agreed:

```
target pixel (144,601)   depth 72.8 m
commanded step           20.0 m along the ray
arrival error            1.67 m
depth at that pixel now  50.5 m measured vs 52.6 m predicted
collision                clean
```

A 2 m residual on a 73 m ray. Swapping the hand-picked pixel for a model-picked one changes
nothing else in the loop — this is the finding that matters, because it means the simulator
is not the obstacle to reproducing SPF.

The camera pose, not the body pose, has to drive the unprojection; the camera is
gimbal-less and `simSetCameraPose` pitch is real. A yaw-only rotation puts the waypoint on
the horizon.

### Perception is good enough

| | |
|---|---|
| RGB throughput | 5.95 Hz at 640x480 (8.96 Hz at 1280x960) |
| RGB + depth | 2.05 Hz at 640x480 / 320x240 |
| RGB + depth + segmentation | 1.43 Hz — see the resolution table above, this is a tuned number |
| Segmentation classes over the city | 15-21 distinct — **not** the all-black failure Cosys-AirSim has on UE5.5 |
| Depth | metric, plausible; `65504.0` (float16 max) marks sky |
| Extra buffers | DepthVis, SurfaceNormals, Infrared, DisparityNormalized all present |

Comfortably above what a sub-1 Hz VLM needs.

### Control is good enough

| | |
|---|---|
| Velocity command → motion | 317 ms mean |
| 10 Hz streamed `moveByVelocityAsync` | tracks; 47 m → 1.2 m in 8.7 s, no stall |
| `rotateToYawAsync(90)` | 93.5° |
| `moveToPositionAsync().join()` | returns *at* the target, 0.26–0.64 m |
| …then relaxes | **~4 m mean, 5.8 m worst**, and holds there |

The post-join relaxation is the number to design around: waypoint accuracy is ~4 m, not
sub-metre. Fine against AerialVLN's 20 m success radius, but any episode must log the
*measured* pose, never the commanded one.

### Repeatability is usable for seeded evaluation

Same open-loop command sequence, twice from reset: **0.04 m max trajectory divergence** on
the clean run (0.33 m on an earlier one). Not bit-exact, but tight enough for
success-rate-over-N-seeds work.

CARLA synchronous mode can be enabled and the drone keeps flying while CARLA is
tick-driven (9.09 m in 3 s of ticks) — but that means AirSim is **not** stepped by CARLA's
tick. There is no joint lockstep; the drone runs on its own clock regardless.

### Air-ground really is one world

One UE4 process serves both RPC servers — verified as one PID owning both :2000 and :41451.

- One `world.set_weather(HardRainSunset)` call on the CARLA side changes the drone's own
  camera: mean pixel delta 48.1, brightness 157.7 → 149.4.
- 20 segmentation classes in the nadir view over the city with traffic below.
- The frame offset is real and is the documented trap. Upstream's Town10HD constants
  (`airsim_x = carla_x + 172.20`, `airsim_y = carla_y - 183.86`, `airsim_z = -carla_z +
  27.45`) check out. **The AirSim NED origin on Town10HD is offshore** — fly to raw CARLA
  x/y and you get open water, which is exactly what the first p03/p06 runs captured.

### Two things that do not work

**Traffic manager vehicles stall intermittently.** Two runs of the same probe disagreed
sharply: **4 of 15** vehicles moving >1 m in 6 s (median 0.14 m) on one, **11 of 15**
(median 30.2 m) on the next. So it mostly works and sometimes does not, which is the worst
shape for a benchmark. This is not a probe artefact — upstream's own `auto_traffic.py`
ships a `health_check_vehicles()` watchdog that re-enables autopilot on any vehicle with
`speed < 0.01`, i.e. upstream hit the same thing and papered over it. Dense scripted
traffic is CARLA-Air's whole differentiator over Cosys-AirSim, so that watchdog needs
porting before any air-ground scenario is trustworthy.

Pedestrians additionally need `controller.ai.walker` controllers plus `go_to_location`;
spawning a walker alone gives a statue. Spawning vehicles and calling `set_autopilot()`
afterwards also leaves them parked — use the batched
`SpawnActor().then(SetAutopilot(...))` form.

**ROS 2 Jazzy cannot host the clients.** Hard ABI wall, both directions:

```
carla extension:  libcarla.cpython-310-x86_64-linux-gnu.so
ROS 2 Jazzy:      Python 3.12.3

import carla under ROS 2 python3.12  →  ModuleNotFoundError: No module named 'carla.libcarla'
import rclpy under the 3.10 venv     →  _rclpy_pybind11.cpython-310-...so isn't present
```

Upstream's ROS 2 example works only because Humble is also 3.10 — the PYTHONPATH trick it
documents has that unstated precondition. On Jazzy the clients and the ROS 2 graph must be
separate processes with an IPC hop between them. That is the same conclusion Lane C reached
from the other direction, and it is the single biggest structural cost of this lane.

---

## Environment, built without conda

Upstream's `env_setup/setup_env.sh` hard-fails without miniconda. Conda is not the
requirement — **CPython 3.10 is**, because the shipped module is ABI-tagged
`cpython-310`. Ubuntu 24.04 has no python3.10 in apt, so `scripts/setup_env.sh` uses uv to
fetch a standalone 3.10 into `vendor/`. Nothing lands in `~` or the system.

`airsim` 1.8.1 declares neither numpy nor setuptools as build dependencies while importing
numpy at `setup.py` time, so it fails twice under build isolation. Install `numpy<2` and
`setuptools` first, then `airsim --no-build-isolation`.

---

## Verdict for the plan

The reference doc rates CARLA-Air as the option to reach for "when a scenario needs dense,
scripted urban traffic + pedestrians beneath the drone". On the evidence here that is
backwards for the *VLM* goal specifically:

- the **VLM loop itself is the strongest part** — frame grab, segmentation, metric depth,
  pixel→3D grounding and streamed velocity control all work today, headless, on one GPU,
  with no build step and no container;
- the **dense traffic that justified the entry is the least dependable part** — between 4
  and 11 of 15 vehicles driving depending on the run, with upstream shipping a watchdog
  for exactly this;
- and **PX4 is simply absent**. Not partial, absent: zero occurrences of `px4`, `mavlink`,
  `uxrce` or `/fmu/` anywhere in the repository. The flight stack is AirSim SimpleFlight.
  Requirement #2 of the plan — PX4 SITL with the real `/fmu/*` topics — cannot be met here
  at all, so nothing tested in this lane transfers to a Pixhawk without a rewrite of the
  control path.

So: a good, cheap **VLM perception-and-grounding testbed** that can be stood up in an
afternoon, and a poor **sim-to-real** lane. Useful as a place to develop and benchmark the
VLM client against a photoreal city while the PX4-bearing lane is built elsewhere.
