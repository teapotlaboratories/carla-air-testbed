# 2026-08-03 — World control on ROS 2, and a one-command install

Backlog [R-04](../todo.md) (done) and [R-01](../todo.md) (half done). Written as the work
happened, unlike the previous entry.

---

## 1. R-04: the export was the problem, not the four commands

Merging `setup_env`/`fetch_release`/`fetch_vendor`/`build_ros` into `scripts/install.sh` is a
wrapper. The part worth writing down is what the wrapper exposed.

`fetch_release.sh` ended by printing `export CARLAAIR_RELEASE=…` for the user to paste into a
shell profile, and **four separate scripts each inlined the same fallback expression** to guess
the path when it was unset. They agreed by copy-paste, not by construction, and `run_sweep.sh`
hard-failed without the variable.

`scripts/release_path.sh` is now the single resolver — `$CARLAAIR_RELEASE`, then
`.release-path` (written by the installer, git-ignored), then `$CARLAAIR_HOME`, then next to
the repo. The export step is **removed**, not automated.

**It was already broken on this machine.** `CARLAAIR_RELEASE` is unset in a fresh shell here,
and the built-in default points at a directory that does not exist — the release is on the
external drive. Every script needing it was one un-exported variable from failing, and nothing
had noticed because every previous session happened to export it by hand.

## 2. C-01 had left a fourth config file

Found by checking that every path the docs name still exists — not by running anything.

`ros2_ws/src/bringup/config/testbed.yaml` was still tracked, still installed into the package
share, and still the **default value of the launch file's `params` argument**, while the
renderer wrote to a `configs/generated/params.yaml` of its own invention. `bringup.sh` passes
`params:=` explicitly, so the normal path was correct and nothing ever looked wrong; a bare
`ros2 launch bringup testbed.launch.py` read the stale copy, which was already missing
`recorder.crf: 26`.

Fixed by rendering into the path the launch file already defaults to and deleting
`configs/generated/`. **C-01 was verified by running it**, and running it never touched the
bare-launch path. "It works when I run it" does not cover paths you do not run.

## 3. R-01: four services, and a design risk that had to be measured

Agreed with the operator: services, not topics, because each of these has a failure the caller
must see — an unknown weather preset, a map that refused half the spawn points, a reset that
could not reach its pose. A topic drops those and the scenario scores a number that means
nothing.

`ResetVehicle`, `SpawnTraffic`, `SetWeather`, `DestroyActors` on `/sim/*`.

**Not PX4 messages, deliberately.** Nothing on a real Pixhawk teleports an airframe or spawns
pedestrians, so borrowing `VehicleCommand` here would imply a portability these calls do not
have. The rule stays legible: `/fmu/*` is what survives the move to hardware, `/sim/*` is what
does not.

### The table in the plan was wrong before I started

R-01 listed `collision` as unreachable from ROS. It is not — `/sim/collision` and
`/sim/traffic_stats` have been published at 1 Hz from `_tick_world` all along. I had written
the list from reading the sidecar's RPC surface instead of the bridge's publisher list.
Corrected in the entry before writing any code.

### `reset` blocks for 16.2 s, not 5

I wrote "~5 s" in three comments from the shape of the code — `reset()`, a `sleep(3.0)`, a
`moveToPositionAsync().join()`, a 2 s settle. Measured on a ~60 m move: **16.2 s**. A client
using rclpy's default 5 s service timeout would report a failure that did not happen. Every
comment now carries the measurement instead of the guess.

### The blocking call does not stall telemetry — measured, because it easily could have

This was the real design risk. A 16 s blocking call on `self.sim` would stall odometry and the
world tick for its whole duration. So the services got a **fourth `SimBridgeClient`
connection** and the executor went from 4 threads to 5 — the same "one connection per concern"
reasoning that already governs telemetry / media / control.

Measured *during* a reset: `/fmu/out/vehicle_odometry` held **19.898 Hz** against a 20 Hz
target. The split works.

### Two bugs the run found

- **`self.clients` is a read-only property on rclpy's `Node`.** Assigning to it raises
  `AttributeError` at construction. Renamed to `srv_clients`. A trivial bug, and it would have
  shipped in an *example* — the file whose entire job is to be copied.
- **`client.spawn_traffic` silently dropped `near_ned` and `radius_m`.** Every ROS-side spawn
  would have been map-wide: of 155 spawn points on Town10HD only ~45 lie within 60 m of a
  typical start, so a scenario asking for busy streets would have got an empty city with
  nothing to indicate it.

8/8 checks passed against a live simulator, including the negative one — a bogus weather preset
is rejected with the valid list, not quietly turned into sunshine.

## 4. What R-01 still owes

- **The chase camera has no ROS topic.** Still `chase_jpeg` over the socket, so R-03 (the web
  console on ROS only) stays blocked.
- **`scripts/run_episode.py` still opens the socket directly.** Rewriting it onto these
  services is the honest end of R-01: it is the heaviest user of the RPC surface, so if it can
  be ported, the surface is real.

Everything stopped afterwards: all counts 0, GPU 1 back to 33 MiB.
