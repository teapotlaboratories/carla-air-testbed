# From probe scratchpad to testbed — and the four things that were measuring the wrong thing

**Date:** 2026-08-01
**Box:** Ubuntu 24.04 distrobox on a Fedora host, RTX 3080 (10 GB) + RTX 5060 Ti (16 GB),
24 cores, 62 GB RAM, NVIDIA 610.43.03, ROS 2 Jazzy.
**Scope:** turn the probe suite into a working VLM navigation testbed over ROS 2, then find
out why its numbers were wrong.

The headline is not the build. It is that **four separate things were corrupting every
performance measurement taken in this project**, including measurements already written down
as fact, and that three of the four were self-inflicted.

---

## What was built

Three design decisions were the operator's, taken before any code:

| decision | chosen | rejected |
|---|---|---|
| Crossing the Python version boundary | 3.10 sidecar + Unix socket | ROS 2 Humble in a 3.10 env; shared memory for frames |
| ROS 2 topic shape | PX4-shaped (`/fmu/out/*`, `/fmu/in/*`) | AirSim-shaped; neutral names + adapters |
| First VLM backend | pluggable interface, mocks first | Claude API; local vLLM |

The resulting split, which is forced rather than preferred:

```
[py3.10] sim_bridge ──UDS/msgpack──> [py3.12] carla_air_bridge → vlm_client → grounding → control
  carla + airsim                       /fmu/out/*                                /fmu/in/trajectory_setpoint
```

`tests/conformance/p07_ros2_interop.py` measured the wall in both directions:

```
import carla under ROS 2 python3.12  ->  ModuleNotFoundError: No module named 'carla.libcarla'
import rclpy under the 3.10 venv     ->  _rclpy_pybind11.cpython-310-...so isn't present
```

Seven ROS 2 packages (`interfaces`, `carla_air_bridge`, `vlm_client`, `grounding`,
`control`, `evaluation`, `bringup`), named to match `drone-sim` so nodes can move between
the projects, with `px4_msgs` pinned to the same SHA (`392e831c`).

First end-to-end result, before any of the corrections below: the geometric backend flew
**152 m in 90 s with no collisions**, and a scored episode wrote an `EpisodeResult`. The
loop worked. The numbers describing it did not.

---

## Correction 1 — the simulator was rendering on the CPU

**This supersedes the "leave `VK_ICD_FILENAMES` unset" recommendation in
`2026-07-31-carla-air-probe-run.md`, which was written here and was actively harmful.** That
section is marked superseded in place.

Prompted by a question about whether the GPU was the limiting factor, utilisation was
sampled during a capture loop instead of assumed:

```
GPU 0 during capture:  0 % utilisation, 111 MiB
/proc/<sim>/fd       :  zero /dev/nvidia* handles
/proc/<sim>/maps     :  /usr/lib/x86_64-linux-gnu/libvulkan_lvp.so     <- lavapipe
ps -o %cpu           :  251 %  (at idle)
```

The simulator had been running on **lavapipe, the LLVM software rasteriser**, for the entire
build. Both RTX cards were idle throughout.

Root cause is one line. `/usr/share/vulkan/icd.d/nvidia_icd.x86_64.json` is injected into
the container from the Fedora host and keeps the host's path:

```json
"library_path": "/usr/lib64/libGLX_nvidia.so.0"     // absent in this Ubuntu container
```

`dpkg -S` confirms the library at `/lib/x86_64-linux-gnu/libGLX_nvidia.so.0` is **not from a
package** — distrobox bind-mounts the host driver there. So the loader cannot open the
NVIDIA ICD at all, and both outcomes are silent:

| `VK_ICD_FILENAMES` | result |
|---|---|
| the system NVIDIA ICD | UE dies in Vulkan init. Exit 1, **zero bytes** of log, no crash dump, nothing in `Saved/Logs`. Looks like a corrupt install. **0/6 starts.** |
| unset | loader enumerates every ICD and falls back to lavapipe. Everything works, on the CPU. **6/6 starts.** |
| every `icd.d/*.json` colon-joined | starts (this is what upstream's `CarlaAir.sh` builds, so upstream was never at fault) |

The earlier session saw 0/6 versus 6/6 and concluded "leave it unset". It started reliably
*because* it was on the CPU.

**Fix:** `scripts/run_sim.sh` uses the system ICD when its `library_path` resolves and
synthesises a corrected one from `ldconfig` when it does not, then **checks VRAM after
startup and warns below 1 GB** — because nothing else about the software path announces
itself. Verified both branches, including deliberately corrupting the ICD back to the Fedora
path and watching it repair itself.

**On native Ubuntu this cannot happen**: the `nvidia-driver` package ships a correct ICD.
This is the only distrobox-caused workaround in the project.

---

## Correction 2 — depth is transport-bound, not GPU-bound

With hardware rendering restored, the remaining cost was decomposed per buffer rather than
guessed at:

| single buffer | cost |
|---|---|
| `Scene` uint8 640x480 | 27.7 ms |
| `Segmentation` uint8 320x240 | 16.6 ms |
| `DepthVis` uint8 256x144 | 16.6 ms |
| **`DepthPerspective` float 320x240** | **243.3 ms** |
| `DepthPerspective` PNG 320x240 | 31.7 ms |

And the split inside that 243 ms:

```
raw RPC round trip              245.9 ms
python list -> numpy conversion   1.2 ms   (76800 floats)
```

So the cost is entirely server-side/on the wire in the `pixels_as_float=True` path — msgpack
encoding a float array element by element — and the client is innocent.

**Dead end, ruled out by measurement:** the PNG variant is 7.7× faster, so it was tested for
fidelity. It comes back **8-bit with 140 distinct levels and correlation 0.9442** against
metric depth, non-linear. Unusable for grounding. Recorded here so nobody re-tries it.

The lever that works is resolution, and it is confirmed *not* to be a rendering cost:

| config | software | NVIDIA GPU | speedup |
|---|---|---|---|
| RGB 640x480 | 5.95 Hz | **53.8 Hz** | **9.0x** |
| RGB + depth 320x240 | 2.05 Hz | 3.98 Hz | 1.9x |
| RGB + depth **640x480** | 0.29 Hz | 0.31 Hz | **1.05x — no change** |

A 640x480 depth grab costs ~3.2 s with or without a GPU. Dropping depth to **160x120**,
which preserves float32 metric precision entirely and only reduces spatial resolution, took
the RGB+depth pair from **4.0 Hz to 19.5 Hz (4.9x)**. `configs/sim/settings.json` ships RGB
640x480 with depth 160x120 and segmentation 320x240 — all 4:3, so the pixel scale into the
depth frame is exact.

**RGB throughput is rendering-bound; depth throughput is transport-bound.** They are
different problems and only one of them is fixed by a GPU.

---

## Correction 3 — a second simulator was publishing onto our topics

`/fmu/out/vehicle_odometry` measured **125 Hz against a 20 Hz timer**, with unique
timestamps and no duplicates. It survived a rate guard, a rebuild, a parameter change to
2 Hz, and a verified build marker in the running code. Median gap between published stamps
was exactly 8.00 ms.

The node was not the source:

```
$ ros2 topic info /fmu/out/vehicle_odometry --verbose
Publisher count: 2
  Node name: carla_air_bridge                 <- ours, 20 Hz
  Node name: _CREATED_BY_BARE_DDS_APP_        <- drone-sim's uXRCE-DDS agent, ~125 Hz
```

`drone-sim` runs on this machine and publishes **real** PX4 topics through a uXRCE-DDS
agent. This testbed publishes **PX4-shaped** topics with the same names, on purpose. Both
default to `ROS_DOMAIN_ID=0`, so the two graphs merged.

Consequences, both bad:

- every rate measured in this project before this point was **the sum of two aircraft in two
  simulators** — including numbers already written into docs;
- our `/fmu/in/trajectory_setpoint` at 10 Hz was visible to a **live PX4 SITL instance**.

**Fix:** `bringup.sh`, `status.sh` and `run_episode.py` all export `ROS_DOMAIN_ID=42`
(override `TESTBED_ROS_DOMAIN_ID`).

**Second, separate hazard found while fixing it:** `drone-sim` has packages named `control`,
`evaluation` and `vlm_client` too, so the name-based `pkill` in `stop.sh` could have killed a
running flight gate in the sibling project. `stop.sh` now matches only this repository's
absolute install path.

---

## Correction 4 — the rate guard caused the problem it was added to solve

With the domain isolated, odometry read **11.8 Hz against a 20 Hz timer**. Transport was
ruled out first — `state()` over the socket measured **median 0.2 ms, p90 0.3 ms**, a
~5000 Hz ceiling — so the loss was local.

The cause was a guard added earlier in the session to suppress rclpy replaying missed timer
firings as a burst:

```python
self._next_odom = now + period      # re-anchors on when the callback HAPPENED to run
```

Re-anchoring on `now` adds each firing's lateness to the next interval, and the error
compounds. `_advance()` anchors on the previous **deadline** and resyncs only when more than
one period behind, keeping the burst protection:

```python
return deadline + period if (now - deadline) < period else now + period
```

**11.8 Hz → 19.5 Hz.** Median gap between published timestamps: 50.06 ms.

The sidecar also gained a third AirSim client. One client shared by 20 Hz of telemetry and
10 Hz of setpoints capped odometry at 12.3 Hz; telemetry, control and media now each have
their own, behind their own lock, one thread per connection. `tests/test_offline.py` asserts
the three method sets stay disjoint — the test caught the split when it was made, which is
what it is for.

| | `/fmu/out/vehicle_odometry` |
|---|---|
| one connection, one AirSim client | 1.5 Hz |
| + separate media client | ~12 Hz |
| + separate control client | ~12 Hz (no change on its own) |
| + schedule-anchored guard | **19.5–20.0 Hz** |

---

## Speed: real-time exactly, and no way to go faster

Measured with the full graph running:

```
wall 20.02 s | CARLA RTF 1.000 | AirSim RTF 1.000 | 60.0 FPS
```

Not hardware-limited either — GPU 0 sits around 30 % of a 10 GB card and the process uses
2.8 of 24 cores. But that headroom **cannot** be spent on faster sweeps. AirSim's
`ClockSpeed` was tested at 3.0:

| clock | ClockSpeed 1.0 | ClockSpeed 3.0 |
|---|---|---|
| AirSim (the aircraft) | RTF 1.00 | **RTF 3.00** |
| CARLA (traffic, pedestrians, weather) | RTF 1.00 | **RTF 1.00** |

The aircraft accelerates and the ground world does not, so the two halves of CARLA-Air's
"one world" desync — a car taking 4 s to cross a junction now takes 12 s of drone time. For
air-ground scenarios, which is the entire reason to choose this simulator, that makes results
meaningless. The ROS 2 graph is wall-clock driven too, so the VLM would also get a third of
the sim-time budget.

`ClockSpeed` stays at 1.0. **N-seed sweeps run in real time and that is the throughput
ceiling** — budget roughly `timeout x seeds` of wall clock.

---

## Bugs in this project's own code, found late

Recorded because each was invisible until something unrelated exposed it:

- **A helper method inserted mid-`__init__`** silently truncated the constructor. The node
  started with **no publishers, no subscriptions and no timers**, logged no error, and
  reported healthy. Found only because topic rates were zero.
- **`capture()` lost a key** the client still read — `KeyError: 'state'` at runtime, not
  build time.
- **The episode harness matched results by scenario+seed prefix**, so re-running a seed
  returned the *previous* run's JSON instantly. It looked entirely plausible. Now matched by
  episode id.
- **The reset did not own the aircraft.** The controller kept streaming toward the previous
  episode's waypoint during reset; a reset to `(107.6, -159.4, -55.0)` landed at
  `(-14.2, -18.4, +5.1)`. The controller is now disabled during reset and drops stale
  targets.
- **`status.sh` reported one of every node while the stack was down** — `pgrep -f` was
  matching the invoking shell's own command line. It now counts by absolute install path and
  excludes its own process ancestry. Validated with a fake process at a package path.
- **A string replace that did not match, reported as success** because the script printed
  unconditionally. Replacements are now asserted.

`pkill -f <pattern>` killing the shell that runs it (exit 144) cost several cycles across
the session before it was understood, and is now a rule.

---

## Hardening and tooling added

- `scripts/stop.sh` — path-scoped, cannot reach `drone-sim`; `scripts/status.sh` — processes,
  GPU, sockets, optional topic rates; `bringup.sh` now clears a previous run first, because
  stacked graphs happened three times.
- `scripts/fetch_release.sh` — the install was previously prose. Downloads via
  `hf_hub_download` (**75 s** versus plain curl's sustained ~800 KB/s and projected two
  hours), verifies the byte count, unpacks, checks the binary.
- `tests/test_offline.py` + `tests/test_scenarios.py` — **50 tests, no simulator, GPU or
  display**. Includes a scenario linter (goal within the step budget, timeout feasible,
  success radius above the ~4 m station-keeping floor, altitudes inside the controller
  clamp).
- `.ai/` rewritten for this project; `CLAUDE.md` added at the repo root, without which
  nothing in `.ai/` was being loaded at all.
- `.gitignore` widened — notably `.env` / `*.key`, ahead of the API key the next backend will
  carry. Verified the broader rules drop nothing: tracked set 96 files before and after.

---

## State at the end of the session

| | |
|---|---|
| `/fmu/out/vehicle_odometry` | 20.002 Hz |
| `/fmu/in/trajectory_setpoint` | 10.001 Hz |
| `/camera/depth/image_raw` | 7.949 Hz |
| `/vlm/annotation` | 0.901 Hz |
| Real-time factor | 1.000 both clocks, 60 FPS |
| Offline tests | 50 passing |
| Rendering | RTX 3080, 3.3 GB, hardware confirmed |

## Unverified / open

- **Nothing is committed.** ~96 files in the index, zero commits.
- **`avoid_the_block` does not test what its name claims** — the oracle solved it in 9 steps
  on a near-straight line, so no building was in the path. None of the four scenarios
  currently require obstacle avoidance.
- **All backend results are single-seed.** No success *rates* yet; the 4/4 and 0/4 are
  ceiling and floor markers only.
- **No real VLM backend exists.** The next fork is Claude API versus local vLLM on the idle
  RTX 5060 Ti — which is genuinely free, so the reference plan's worry about UE and a model
  contending for one card does not apply on this machine.
- **The bearing-only grounding path and the three-way client split have run in one session
  only.** They are not yet exercised across a multi-seed sweep.
