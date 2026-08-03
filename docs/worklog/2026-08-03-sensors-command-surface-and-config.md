# 2026-08-03 — Sensors on ROS, the command surface closed, and one config file

Backlog items [S-01, S-02, S-02b](../todo.md#sensors), [S-04](../todo.md) and
[C-01](../todo.md). Spans 2026-08-02 and 2026-08-03.

> **Written retroactively, and that is a rule broken.** The `.ai/AGENTS.md` worklog rule says
> "as you go", appended at each finding. This was appended after the work landed, so the
> ordering of findings below is reconstructed from the todo entries and the code rather than
> recorded live. The findings themselves are all from measured output; the *timeline* is the
> part that is weaker than it should be. Same for the plan-first rule on S-04 and C-01, whose
> backlog entries were written alongside the code rather than before it.

---

## 1. The sensors were already there

Probed on a running simulator rather than reasoned about: `configs/sim/settings.json` declared
**no sensors at all**, and IMU, barometer, magnetometer and GPS answered anyway. AirSim
auto-creates those four for a multirotor. They had been answering for the life of the project
and publishing nowhere.

They are instruments, not ground truth — two IMU reads 0.4 s apart differed by 1.0e-1, so the
noise model is live. `px4_msgs` already builds every message needed, so nothing was generated:

| source | topic | message |
|---|---|---|
| IMU | `/fmu/out/sensor_combined` | `SensorCombined` |
| barometer + environment | `/fmu/out/vehicle_air_data` | `VehicleAirData` |
| magnetometer | `/fmu/out/vehicle_magnetometer` | `VehicleMagnetometer` |
| GPS | `/fmu/out/sensor_gps` | `SensorGps` |

One RPC call returns all four. Four calls would quadruple the round trips for data read from
the same client in the same millisecond.

**GPS answered `47.639686, -122.138289`** — Redmond, Washington, AirSim's built-in default,
with no relationship to Town10HD. Shipped as-is and made settable rather than pinned, because
a fictitious origin is a perfectly good GPS *sensor* and a meaningless *position*, and which
of those a user wants is their call.

**The cost is real and it lands on the image path.** These are polled over RPC, the same path
as `simGetImages`: 5 Hz costs **1.7%** of image rate, 20 Hz costs **28%**. Measured.

## 2. LiDAR: the AirSim route was the wrong one

`getLidarData` throws. Not a bug — LiDAR and the distance sensor exist only if declared in a
`Sensors` block in `settings.json`, read **at startup**. So enabling it costs a simulator
restart *and* puts the point cloud on the RPC path that section 1 just showed is the
bottleneck.

CARLA has 25 sensors including `sensor.lidar.ray_cast_semantic`, and the chase camera had
already proved the pattern: a free-floating actor whose transform is rewritten each tick from
the aircraft's NED pose. CARLA sensors render in-process and **push**, so they never queue
behind a `simGetImages` call.

Cost to the image path: **none measurable** — 6.494 Hz with lidar on against 6.324 off, which
is inside run-to-run variance. Against 28% for the AirSim route at 20 Hz. The detour was the
right call.

### `points_per_second` is rays CAST, not points returned

| position | AGL | points/msg |
|---|---|---|
| plaza | 107 m | 390 |
| plaza | 67 m | 735 |
| plaza | 47 m | **1584** |
| offshore, no buildings | 67 m | 443 |

I first assumed the missing returns were rays escaping into the sky. **That was wrong.** The
binding constraint is `range` against `lower_fov`: with `lower_fov: -50` the steepest ray is
50 degrees below horizontal, so the slant range to the ground is `AGL / sin(50)`, and against
`range: 80` the ground is unreachable above about **61 m AGL**.

    107 m AGL -> 140 m slant   ground unreachable
     67 m AGL ->  87 m slant   ground unreachable
     47 m AGL ->  61 m slant   ground in range

Fine for obstacle work, useless for terrain. Worth knowing before trusting it in a scenario:
`avoid_the_block` flies at 120 m AGL, where this sees the tower and no ground at all.

**The first lidar measurement was invalid** and nearly went in the log: the aircraft was at
NED z **-781 m** left over from a previous run. Check where the aircraft *is* before believing
what a sensor on it says.

**Two robustness fixes.** At 120000 points/second CARLA's own RPC began timing out at 30 s,
and `carla::client::TimeoutException` escaped as an uncaught C++ exception that **terminated
the sidecar mid-flight**. Config is now 16 channels at 30000 points/second, and
`CarlaSensorRig.follow` catches broadly — `except RuntimeError` would never have caught it.

**Known limitation:** `drain()` is destructive. Exactly one consumer; a second reader steals
arcs from the first rather than seeing the same data.

## 3. A ROS-only client could read everything and could not take off

Auditing the command surface found takeoff and land existed **only as sidecar RPC methods**,
reachable from the 3.10 side, and there was no attitude channel at all.

Closed with PX4 messages, not invented ones — `VehicleCommand` with `NAV_TAKEOFF` / `NAV_LAND`
is how a real Pixhawk is commanded over uXRCE-DDS, and `VehicleAttitudeSetpoint` carries a
quaternion `q_d` rather than three Euler angles for the same reason. A friendlier
`/testbed/takeoff` would break the one property this shim exists for: that a node written here
ports to hardware by deleting one node.

### I claimed this worked before it did

I reported the example verified off a `tail -55` that showed output appearing. The operator
asked "is that working?" and the re-test found two things:

- **The autonomy loop wins.** A takeoff commanded to 35 m settled at **15.6 m**, because
  `offboard_control` keeps publishing its own setpoints at 10 Hz. The message surface was
  fine; the *graph* was fighting the command. The example now disables that node via
  `SetParameters` on entry and restores it on exit.
- **Attitude signs were inverted on two axes of three.** Commanded against measured:

      roll  +12 deg -> +12.0    correct
      pitch +15 deg -> -15.0    inverted
      yaw   +40 deg -> -40.0    inverted

  Fixed in `Vehicle.attitude()` by negating pitch and yaw, with that table in the comment.

Neither was findable from a log tail. **Output appearing is not a measurement.** This is the
"measure, do not assume" rule failing in its most seductive form — the run *did* run.

`docs/ros2-api.html` was written only after those calls were re-run, and documents the tested
surface rather than the intended one.

## 4. C-01: one config file

"Where do I set X" needed a read of three scripts, across `configs/sim/settings.json`
(AirSim's schema), `configs/sim/carla_sensors.yaml` (ours) and
`ros2_ws/src/bringup/config/testbed.yaml` (rclpy's schema). **Two of those formats are not
ours to change** — one is read by the CarlaUE4 binary, the other by the ROS parameter system.

So the unification happens one level up: one source, `configs/testbed.yaml`, rendered by
`scripts/apply_config.py` into what each reader insists on. `run_sim.sh` and `bringup.sh`
render before every start, so editing the source is enough and nobody has to remember a build
step.

**Sections are named for WHEN a change takes effect**, not for which file it lands in —
`simulator:` ~60 s, `sidecar:`/`sensors:` ~5 s, `graph:` live. That distinction is load-bearing
and a flat file would hide it. This project has been bitten repeatedly by exactly that shape of
flattening: `min_altitude_m: 15` reads as 15 m AGL and is 42.45; `points_per_second` reads as
points and is rays.

**Sensors were the sharpest case, and I had argued the other way three days earlier.** S-02b
gave CARLA sensors their own file, reasoning that `settings.json` is read by the simulator at
launch while CARLA actors are spawned afterwards, so one file would imply one reader. That
argument was about **rendering** and I mistook it for an argument about **authoring** — a
reader can still get its own generated file. The list is now one `sensors:` section with a
`source: airsim|carla` field carrying the distinction that used to be carried by which file
you were in. `configs/sim/carla_sensors.yaml` is gone; the S-02b entry is marked superseded in
place.

**Two generated files carry a DO-NOT-EDIT header**, and `--check` regenerates into memory and
diffs, so drift fails a test rather than a flight.

One trap found while writing the renderer: an unset GPS origin must be **omitted**, not
written as `{0, 0}`. AirSim treats a present `OriginGeopoint` as authoritative, so zeros would
silently move the aircraft to the Atlantic instead of falling back to its default. There is a
test for it.

Verified with a full bringup and `TESTBED_GPU` deliberately unset: GPU 1 selected from the
config, hardware rendering confirmed, lidar spawned from the unified list (5022 measurements),
all five sensor topics live, ROS parameters matching the source. 13 offline tests; 116 total.

## 5. Docs

`QUICKSTART.md` and `docs/guide.html` gained a "Change something" section built on the unified
file — the section table with its restart costs, a GPS-origin worked example, and the
`source:` distinction with the measured costs attached, since that is the field most likely to
be set carelessly.

Three stale numbers fixed while in there: the test count (103 -> 116), and the bringup example
still showing `GPU 0:` and a `~110 MiB` idle figure. The simulator renders on **GPU 1** here
(GPU 0 is the operator's display card, and idles at 112 MiB where GPU 1 idles at 33), and
`run_sim.sh` prints the card it actually used, name included. The quickstart now says which
index it will be and what a single-GPU machine should set instead.

## 6. C-01 had left a fourth config file, and the doc pass is what found it

Checking that every path the docs name still exists turned up a live trap.

`ros2_ws/src/bringup/config/testbed.yaml` — the *old* ROS parameter file — was still in the
repo, still git-tracked, still installed into the package share, and still the **default value
of the launch file's `params` argument** (`testbed.launch.py:23`). Meanwhile the renderer wrote
to a `configs/generated/params.yaml` of its own invention. That is four config files, not one,
and C-01 claimed the opposite.

It never looked wrong because `bringup.sh` passes `params:=` explicitly, so the normal path
read the generated file. A bare `ros2 launch bringup testbed.launch.py` read the stale one.
They had already diverged — the stale copy was missing `recorder.crf: 26`.

Fixed by pointing `PARAMS_OUT` at the path the launch file already defaults to and deleting
`configs/generated/` entirely. One parameter file, and the bare launch command is now correct
by construction rather than by remembering a flag.

**The generalisable bit:** the unification was verified by *running* it — full bringup, GPU
selected from config, sensors live, parameters matching. All true, and all blind to a second
file nobody was reading on that path. "It works when I run it" does not cover paths you do not
run. The C-01 entry is corrected in place rather than edited to look right.

## 7. R-04: one command to install

`./scripts/install.sh` — four steps, a step counter, per-step timing, one failure path that
names the step and says re-running resumes.

The interesting half was **not** the wrapper. `fetch_release.sh` ended by printing an
`export CARLAAIR_RELEASE=…` line for the user to paste into a shell profile, and four scripts
each inlined the same fallback expression to guess the path otherwise. They agreed by luck
rather than by construction, and `run_sweep.sh` hard-failed when the variable was unset.

`scripts/release_path.sh` is now the only resolver: `$CARLAAIR_RELEASE`, then `.release-path`
(written by the installer, git-ignored), then `$CARLAAIR_HOME`, then next to the repo. The
export step is *removed*, not automated.

**And it was already broken here.** `CARLAAIR_RELEASE` is unset in a fresh shell on this
machine, and the built-in default points at a directory that does not exist — the real release
is on the external drive. Every script needing it was one un-exported variable from failing.
Writing `.release-path` fixed the working tree as a side effect of fixing the design.

Verified: 3/3 steps green and 117 tests passing, run twice for idempotency; the failure path
exercised with a deliberately failing step (`INSTALL FAILED at step 1`, exit 1, no
continuation); the resolver at all four precedence levels; `bash -n` on all five touched
scripts. **Not** verified: the clean-clone path including the 6.85 GB download — only a fresh
machine proves that.
