#!/usr/bin/env python3
"""Set the world up for an episode, then hand scoring to the ROS 2 episode runner.

    ./scripts/run_episode.sh --scenario cross_the_plaza --seeds 1 2 3

The ROS 2 graph must already be running (`./scripts/bringup.sh`).

**This is now a plain ROS 2 client, and that is the point of it.** It used to run on the
Python 3.10 side and open the sim_bridge socket directly, because resetting the aircraft,
spawning traffic and setting weather had no ROS surface — and it shelled out through
`bash -lc` to `ros2 service call` and `ros2 param set` for the parts that did. All of that is
gone: every interaction here is a service call, a parameter set or a subscription, so this
script is now exactly what any user's own harness would look like. If it ever needs the
socket again, the ROS surface has a hole in it.

Two consequences worth knowing:

* It runs under **ROS 2's python (3.12)**, not `./.venv/bin/python`. Use
  `scripts/run_episode.sh`, which sources the workspace and picks the interpreter.
* The division of labour is unchanged: this script owns the world, the ROS 2 graph owns the
  flight, `evaluation/episode_runner` owns the score. No component does two of those.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import yaml

try:
    import rclpy
    from rcl_interfaces.srv import SetParameters
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy)
    from px4_msgs.msg import VehicleOdometry

    from interfaces.msg import Collision
    from interfaces.srv import (ChaseRecording, DestroyActors, ResetVehicle, SetCameraPose,
                                SetEpisode, SetWeather, SpawnTraffic)
except ImportError as exc:                                             # pragma: no cover
    raise SystemExit(
        f"{exc}\n\n"
        "This is a ROS 2 client now and needs the workspace sourced:\n"
        "    source /opt/ros/jazzy/setup.bash\n"
        "    source ros2_ws/install/setup.bash\n"
        "    export ROS_DOMAIN_ID=42\n"
        "Or use the wrapper, which does all three: ./scripts/run_episode.sh") from exc

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENARIOS = os.path.join(PROJ, "ros2_ws", "src", "evaluation", "scenarios", "default.yaml")

#: `reset` flies the aircraft to the start pose and settles — 16.2 s measured for a ~60 m
#: move — and traffic spawning is slower still on a busy map. Neither fits a default timeout.
SLOW_CALL_S = 120.0

#: PX4 publishes BEST_EFFORT + TRANSIENT_LOCAL. A RELIABLE subscriber receives nothing, and
#: receives it silently: no error, no warning, just a topic that never fires.
PX4_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=5)


def load_scenario(name):
    with open(SCENARIOS) as f:
        doc = yaml.safe_load(f)
    for s in doc["scenarios"]:
        if s["name"] == name:
            return s
    raise SystemExit(f"unknown scenario {name!r}; have "
                     f"{[s['name'] for s in doc['scenarios']]}")


class EpisodeDriver(Node):
    """Everything this script needs from the graph, and nothing else."""

    def __init__(self):
        super().__init__("run_episode")
        # NOT `self.clients` — rclpy's Node owns that name as a read-only property.
        self.srv = {
            "destroy": self.create_client(DestroyActors, "/sim/destroy_actors"),
            "weather": self.create_client(SetWeather, "/sim/set_weather"),
            "traffic": self.create_client(SpawnTraffic, "/sim/spawn_traffic"),
            "reset": self.create_client(ResetVehicle, "/sim/reset_vehicle"),
            "camera": self.create_client(SetCameraPose, "/sim/set_camera_pose"),
            "chase": self.create_client(ChaseRecording, "/sim/chase_recording"),
            "episode": self.create_client(SetEpisode, "/episode/set"),
        }
        self.odom = None
        self.collision = None
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry",
                                 self._on_odom, PX4_QOS)
        self.create_subscription(Collision, "/sim/collision", self._on_collision, 5)

    def _on_odom(self, msg):
        self.odom = msg

    def _on_collision(self, msg):
        self.collision = msg

    def wait_for_graph(self, timeout_s=60.0):
        missing = [n for n, c in self.srv.items()
                   if not c.wait_for_service(timeout_sec=timeout_s)]
        if missing:
            raise SystemExit(
                f"these services never appeared: {', '.join(missing)}\n"
                "Is the graph up (./scripts/bringup.sh) and ROS_DOMAIN_ID=42?")

    def call(self, name, request, timeout_s=30.0):
        future = self.srv[name].call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done():
            raise RuntimeError(f"{name}: no response after {timeout_s}s")
        return future.result()

    def pump(self, seconds):
        """Spin so the subscriptions actually receive while we wait."""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def get_param(self, node_name, name, timeout_s=15.0):
        """The node's current value, so an override can be undone exactly."""
        from rcl_interfaces.srv import GetParameters
        cli = self.create_client(GetParameters, f"{node_name}/get_parameters")
        try:
            if not cli.wait_for_service(timeout_sec=timeout_s):
                return None
            req = GetParameters.Request(names=[name])
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_s)
            if not fut.done() or not fut.result().values:
                return None
            v = fut.result().values[0]
            return v.double_value if v.type == 3 else (
                v.integer_value if v.type == 2 else None)
        finally:
            self.destroy_client(cli)

    def set_param(self, node_name, name, value, timeout_s=15.0):
        """Replaces a `bash -lc … ros2 param set` subprocess with the service it wrapped."""
        cli = self.create_client(SetParameters, f"{node_name}/set_parameters")
        try:
            if not cli.wait_for_service(timeout_sec=timeout_s):
                return False
            req = SetParameters.Request()
            req.parameters = [Parameter(name=name, value=value).to_parameter_msg()]
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
            return bool(future.done() and future.result().results[0].successful)
        finally:
            self.destroy_client(cli)


def result_for(episode_id):
    """The result file for exactly this episode, or None if it has not landed yet.

    By id, never by a scenario+seed prefix: that would return the PREVIOUS run of the same
    seed the instant it is checked, and a sweep would report stale results that look
    entirely plausible.
    """
    path = os.path.join(PROJ, "out", "episodes", f"{episode_id}.json")
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


def run_one(drv: EpisodeDriver, scen, seed, camera_pitch, chase=True):
    print(f"\n=== {scen['name']} seed={seed} ===", flush=True)

    # World first, aircraft second: traffic spawning moves actors around, and the reset must
    # be the last thing that touches the vehicle before the clock starts.
    drv.call("destroy", DestroyActors.Request())

    r = drv.call("weather", SetWeather.Request(preset=scen.get("weather", "ClearNoon")))
    if not r.success:
        raise SystemExit(f"weather: {r.message}")

    # Cluster the traffic around where the aircraft actually starts. Map-wide spawning put
    # ~4 of 15 cars and ~2 of 10 pedestrians within 60 m of the start; the camera only ever
    # sees the neighbourhood, so the rest was scenery for nobody.
    treq = SpawnTraffic.Request()
    treq.vehicles = int(scen.get("traffic_vehicles", 15))
    treq.walkers = int(scen.get("traffic_walkers", 10))
    treq.near_ned = [float(v) for v in scen["start_ned"]]
    treq.radius_m = float(scen.get("traffic_radius_m", 150.0))
    r = drv.call("traffic", treq, timeout_s=SLOW_CALL_S)
    print(f"  traffic: {r.spawned} vehicles, {r.walkers_spawned} walkers"
          f"{'  (' + r.message + ')' if r.message else ''}", flush=True)
    if not r.success:
        raise SystemExit(f"traffic: {r.message}")

    # Per-scenario controller overrides, applied before the flight and restored after.
    #
    # `max_speed_mps` and friends are GLOBAL parameters while `max_steps` and `timeout_s` are
    # per-scenario, so tuning the controller for one scenario silently reaches into every
    # other one - and did: slowing a street-level DEMO to 0.5 m/s pushed four benchmark
    # scenarios past their timeouts and would have doubled the E-01b sweep to ~4 h. A demo
    # should not be able to invalidate a measurement.
    overrides = scen.get("control") or {}
    applied = {}
    for name, value in overrides.items():
        before = drv.get_param("/offboard_control", name)
        if before is not None and drv.set_param("/offboard_control", name, float(value)):
            applied[name] = before
    if applied:
        shown = ", ".join(f"{k}={overrides[k]} (was {applied[k]})" for k in applied)
        print(f"  control: {shown}", flush=True)

    start = [float(v) for v in scen["start_ned"]]
    # The reset must own the aircraft exclusively. The offboard controller streams setpoints
    # at 10 Hz and will happily drag the vehicle toward the previous episode's waypoint while
    # reset is trying to fly it to the start.
    if not drv.set_param("/offboard_control", "enabled", False):
        print("  WARNING: could not disable offboard_control; the reset may be fought",
              flush=True)
    time.sleep(0.5)

    # reset leaves the aircraft HOLDING a setpoint, never merely armed. Skipping that is how
    # a session ends up at -1566 m NED.
    rreq = ResetVehicle.Request()
    rreq.hold_ned = start
    rreq.speed = 10.0
    r = drv.call("reset", rreq, timeout_s=SLOW_CALL_S)
    if not r.success:
        raise SystemExit(f"reset: {r.message}")

    cam = SetCameraPose.Request()
    cam.xyz = [0.5, 0.0, 0.1]
    cam.pitch = float(camera_pitch)
    got = drv.call("camera", cam)
    if not got.success:
        print(f"  WARNING: camera pose not applied ({got.message})", flush=True)

    pos = [float(v) for v in r.position_ned]
    err = math.dist(pos, start)
    print(f"  start pose: {[round(v, 1) for v in pos]} "
          f"(commanded {[round(v, 1) for v in start]}, error {err:.1f} m)", flush=True)
    if err > 15.0:
        print("  WARNING: reset did not reach the start — episode will not be comparable",
              flush=True)

    drv.set_param("/offboard_control", "enabled", True)

    ep = SetEpisode.Request()
    ep.scenario = scen["name"]
    ep.seed = int(seed)
    ep.instruction = scen.get("instruction", "")
    ep.start = True
    t_episode = time.time()
    r = drv.call("episode", ep, timeout_s=60.0)
    if not r.accepted:
        print(f"  episode/set refused: {r.message}", flush=True)
        return None
    episode_id = r.episode_id
    print(f"  episode running [{episode_id}] — {ep.instruction!r}", flush=True)

    # Exterior HD view, following the aircraft. A spectator, scored on nothing, so it must
    # never be able to fail a flight — worst case you lose the video, not the run.
    chase_path = None
    if chase:
        try:
            os.makedirs(os.path.join(PROJ, "out", "chase"), exist_ok=True)
            chase_path = os.path.join(PROJ, "out", "chase", f"{episode_id}.mp4")
            creq = ChaseRecording.Request()
            creq.start, creq.path = True, chase_path
            # 0 = use sidecar.chase_camera from the config. A scenario may still pin its
            # own values, but it no longer has to restate the defaults to get them.
            creq.width = int(scen.get("chase_width", 0))
            creq.height = int(scen.get("chase_height", 0))
            creq.fps = float(scen.get("chase_fps", 0.0))
            t_chase = time.time()
            got = drv.call("chase", creq, timeout_s=60.0)
            if not got.success:
                raise RuntimeError(got.message)
            # Write down how much later the chase started than the episode. Without it the
            # two recordings cannot be aligned afterwards: the chase both STARTS later and
            # STOPS later, so their durations differ by (tail - head) and neither term is
            # recoverable from the files. Two unknowns, one equation.
            with open(chase_path.replace(".mp4", ".sync.json"), "w") as fh:
                json.dump({"episode_id": episode_id,
                           "chase_start_after_episode_s": round(t_chase - t_episode, 3)}, fh)
        except Exception as exc:                                       # noqa: BLE001
            print(f"  chase camera unavailable ({exc}); flying without it", flush=True)
            chase_path = None

    try:
        deadline = time.time() + float(scen.get("timeout_s", 240.0)) + 30.0
        while time.time() < deadline:
            drv.pump(5.0)
            if drv.odom is not None:
                p = [float(v) for v in drv.odom.position]
                hit = ""
                if drv.collision is not None and drv.collision.has_collided:
                    hit = f"  COLLIDED: {drv.collision.object_name or 'unnamed actor'}"
                print(f"    alt {abs(p[2]):5.1f} m  pos {[round(v, 1) for v in p]}{hit}",
                      flush=True)
            result = result_for(episode_id)
            if result:
                return result
        print("  runner did not report a result before the deadline", flush=True)
        return None
    finally:
        # Restore the controller BEFORE anything else can use it. A demo's slow speed
        # leaking into the next scenario of a sweep is exactly the failure these overrides
        # exist to prevent.
        for name, value in applied.items():
            drv.set_param("/offboard_control", name, value)

        # In `finally` so a timeout or an exception still closes the file. Without this, the
        # video of the run that went wrong is the one left unplayable.
        if chase_path:
            try:
                got = drv.call("chase", ChaseRecording.Request(start=False), timeout_s=60.0)
                print(f"    chase: {got.frames} frames ({got.seconds:.0f}s), "
                      f"{got.dropped} dropped -> {chase_path}", flush=True)
            except Exception as exc:                                   # noqa: BLE001
                print(f"    chase stop failed: {exc}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--seeds", type=int, nargs="+", default=[1])
    ap.add_argument("--camera-pitch", type=float, default=-0.5,
                    help="radians; must match grounding's camera_pitch_deg")
    ap.add_argument("--no-chase", action="store_true",
                    help="skip the exterior HD chase-camera recording")
    args = ap.parse_args()

    scen = load_scenario(args.scenario)
    rclpy.init()
    drv = EpisodeDriver()
    results = []
    try:
        drv.wait_for_graph()
        for seed in args.seeds:
            r = run_one(drv, scen, seed, args.camera_pitch, chase=not args.no_chase)
            if r:
                results.append(r)
                print(f"  -> {'SUCCESS' if r['success'] else 'FAILURE (' + r['failure_mode'] + ')'}"
                      f"  {r['final_distance_m']:.1f} m from goal, {r['steps']} steps",
                      flush=True)
    finally:
        try:
            drv.call("destroy", DestroyActors.Request(), timeout_s=30.0)
        except Exception:                                              # noqa: BLE001
            pass
        drv.destroy_node()
        rclpy.try_shutdown()

    if results:
        ok = sum(1 for r in results if r["success"])
        print(f"\n=== {args.scenario}: {ok}/{len(results)} succeeded "
              f"({100.0 * ok / len(results):.0f}%) ===")
        # A success rate over N seeds, never a single pass.
        out = os.path.join(PROJ, "out", f"sweep-{args.scenario}.json")
        with open(out, "w") as f:
            json.dump({"scenario": args.scenario, "seeds": args.seeds,
                       "success_rate": ok / len(results), "episodes": results}, f, indent=2)
        print(f"wrote {out}")


if __name__ == "__main__":
    main()
