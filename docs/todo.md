# Backlog

Open work, with the reason and how each will be verified. The `.ai/AGENTS.md` "plan first"
rule points here: non-trivial work gets an entry before it gets code, and the entry is marked
done when it lands.

Status: **open** · **next** (agreed, not started) · **blocked** · **done**

---

## Sensors

### S-01 · Bridge GPS to a ROS 2 topic — **open**

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

### S-02 · Bridge IMU, and decide about LiDAR — **open**

Same shape as S-01: AirSim exposes IMU, barometer and magnetometer, and CARLA has its own
sensor suite. None is bridged because nothing in the loop needed it. IMU is the cheap and
obviously useful one (it is a real sensor stream at high rate); LiDAR is a much larger
payload on a transport that is already the bottleneck for depth, so it needs a throughput
measurement before it is promised.

- **Verify:** publish rate holds under a flight, and the image path does not regress —
  re-measure RGB+depth after adding it.

### S-03 · Segmentation is published but disabled — **open**

`publish_segmentation: false` in `configs/.../testbed.yaml`. It works (15–21 classes measured)
but costs ~77 ms per capture, so it is off for the flight loop. Either leave it as an
analysis-only flag and say so in the docs, or find out whether a smaller segmentation buffer
makes it affordable to leave on.

---

## Evaluation

### E-01 · Turn single-seed markers into success rates — **next**

Every backend result in the README and worklogs is **one seed**. Oracle 4/4 and geometric 0/4
are ceiling and floor markers, not rates, and nothing published should be read as a success
rate until this runs.

- 5 seeds × 4 scenarios × {oracle, geometric}. ~90 min of wall clock; sweeps run in real
  time and `ClockSpeed` cannot help (see the 2026-08-01 worklog).
- Also exercises two code paths that have only ever run in one session: bearing-only
  grounding and the three-way AirSim client split.
- **Verify:** per-scenario success rate with N, plus the failure-mode breakdown, written to
  `out/sweep-*.json` and summarised in a worklog.

### E-02 · `avoid_the_block` does not test what its name claims — **open**

The oracle solved it in **9 steps on a near-straight line**, so no building was ever in the
path. It currently measures the same thing as `follow_the_avenue` while advertising obstacle
avoidance. More broadly, **none of the four scenarios require avoiding anything.**

- Query CARLA for real building footprints and re-site start/goal so the straight line
  genuinely intersects one.
- **Verify:** the naive oracle's success rate *drops* on it — that is the signal the
  obstacle is real. It would also be the first scenario where depth-following could
  plausibly beat a straight-line policy.

### E-03 · Record an MCAP bag per episode — **open**

A failed episode currently leaves a JSON and nothing to look at. Recording the annotation,
grounded-waypoint, odometry and camera topics would make failures diagnosable rather than
merely counted.

- **Verify:** a bag replays and the grounding node produces the same waypoints from it.
  Note `.gitignore` already excludes `*.mcap` and `rosbag2_*/`.

### E-04 · Anchor scenarios to real map features — **open**

Start and goal coordinates were hand-picked from one spawn point. They lint clean and the
oracle proves them navigable, but nothing ties them to junctions, plazas or landmarks a
natural-language instruction could sensibly refer to. That matters once a real VLM reads the
instruction.

---

## VLM

### V-01 · First real backend — **blocked on a decision**

Four backends exist, none is a model: `mock`, `scripted`, `geometric`, `oracle`. The
interface is deliberately narrow — image and instruction in, pixel out — so a real backend is
a single method plus a `BACKENDS` entry.

The fork:

| | |
|---|---|
| **Claude API** | no serving setup, no VRAM budgeting; network-dependent, per-call cost across sweeps |
| **local vLLM** | free to run, and **GPU 1 (RTX 5060 Ti, 16 GB) is completely idle** — the reference plan's worry about UE and a model contending for one card does not apply on this machine; needs a server up before anything works |

Do E-01 first so there is a baseline worth comparing against.

- **Verify:** success rate over the same seeds as E-01, plus p50/p95 decision latency. The
  bar to clear is `geometric`, not zero.

---

## Packaging

### P-01 · Containerise the stack — **blocked on non-nested Docker**

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

### H-01 · Maintainer email is inconsistent — **open**

13 package files carry `aldwinakbar@gmail.com`; the repo's git identity is
`aldwin@hermanudin.com`. Both are the owner's, so nothing is leaked, but the `package.xml`
maintainer field is public-facing if this is ever published. Left alone deliberately —
rewriting contact details is the owner's call.

### H-02 · No remote — **open**

Six commits on `main`, no remote configured, nothing pushed.

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
