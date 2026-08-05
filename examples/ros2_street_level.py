#!/usr/bin/env python3
"""Fly the streets at twice head height, and record both views.

    ./scripts/bringup.sh --config configs/testbed.yaml     # terminal 1
    python3 examples/ros2_street_level.py                  # terminal 2 (ROS 2 python)

    out/street/<stamp>-onboard.mp4   the drone camera, with a HUD
    out/street/<stamp>-chase.mp4     the exterior follow camera

**3.5 m above the road**, which is NED z = +23.95: the origin sits 27.45 m above the street,
so a positive z is BELOW it. Everything else in this project flies at 42-120 m AGL.

Two things this cannot use, and it is worth knowing why:

* **`offboard_control` is disabled for the flight.** Its floor is `min_altitude_m: 15.0` NED
  = 42.45 m AGL, twelve times this altitude. The floor is not a bug - it is what stops a
  waypoint on the ground flying the aircraft into it - so rather than lower it globally,
  this streams its own setpoints and puts the controller back afterwards.
* **No VLM.** The route is precomputed. At 3.5 m the streets here have room for about 2 m of
  clearance either side, and a model that picks one bad pixel hits a wall rather than drifting
  - `busy_street` already shows it flying off the map with far more room. The HUD the episode
  recorder draws needs an EpisodeStatus, so this draws its own.

The route comes from `survey_buildings.route()` - A* around real building geometry at this
altitude. Half the legs of the intended loop came back UNREACHABLE: at street level the gaps
are barely wider than the aircraft, which is itself the interesting measurement.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Image
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleOdometry

from interfaces.msg import Collision
from interfaces.srv import ChaseRecording, DestroyActors, ResetVehicle, SpawnTraffic

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJ, "sim_bridge", "carla_air"))
from h264 import VideoWriter  # noqa: E402

PX4_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=5)

ALT = 23.95              # NED, positive = below the origin. 27.45 - 23.95 = 3.5 m AGL.
ARRIVE_M = 4.0
LEG_TIMEOUT_S = 30.0
GROUND_OFFSET = 27.45    # origin height above the street, from docs/architecture.md


class StreetLevel(Node):
    def __init__(self):
        super().__init__("ros2_street_level")
        self.srv = {
            "reset": self.create_client(ResetVehicle, "/sim/reset_vehicle"),
            "traffic": self.create_client(SpawnTraffic, "/sim/spawn_traffic"),
            "destroy": self.create_client(DestroyActors, "/sim/destroy_actors"),
            "chase": self.create_client(ChaseRecording, "/sim/chase_recording"),
        }
        self.cv = CvBridge()
        self.writer = None
        self.frames = 0
        self.pos = None
        self.hit = None
        self.leg = ""
        self._sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint",
                                         PX4_QOS)
        self._mode = self.create_publisher(OffboardControlMode,
                                           "/fmu/in/offboard_control_mode", PX4_QOS)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry",
                                 self._on_odom, PX4_QOS)
        self.create_subscription(Collision, "/sim/collision", self._on_hit, 5)
        self.create_subscription(Image, "/camera/rgb/image_raw", self._on_image, 5)

    def _on_odom(self, m):
        self.pos = [float(v) for v in m.position]

    def _on_hit(self, m):
        self.hit = m.object_name if m.has_collided else None

    def _on_image(self, msg):
        if self.writer is None:
            return
        f = self.cv.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._hud(f)
        self.writer.write(f)
        self.frames += 1

    def _hud(self, f):
        """The episode recorder's HUD needs an EpisodeStatus. This flight has none."""
        h, w = f.shape[:2]
        s = h / 480.0
        agl = (GROUND_OFFSET - self.pos[2]) if self.pos else float("nan")
        lines = [f"street level   {self.leg}",
                 f"AGL {agl:5.1f} m" + (f"   pos [{self.pos[0]:6.1f}, {self.pos[1]:7.1f}]"
                                        if self.pos else "")]
        if self.hit:
            lines.append(f"CONTACT: {self.hit}")
        pad, lh = int(6 * s), int(20 * s)
        box = lh * len(lines) + pad
        panel = f[0:box].copy(); panel[:] = (0, 0, 0)
        cv2.addWeighted(panel, 0.55, f[0:box], 0.45, 0, f[0:box])
        for i, t in enumerate(lines):
            cv2.putText(f, t, (int(8 * s), pad + lh * i + lh - int(6 * s)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5 * s, (235, 235, 235),
                        max(1, int(round(s))), cv2.LINE_AA)

    def call(self, name, req, timeout=180.0):
        fut = self.srv[name].call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            raise RuntimeError(f"{name}: no reply after {timeout}s")
        return fut.result()

    def offboard(self, on):
        cli = self.create_client(SetParameters, "/offboard_control/set_parameters")
        try:
            # Short: `offboard_control` is in examples/navigation and bringup.sh has not
            # started it since 2026-08-04, so absence is the ordinary case and waiting 15 s
            # for it adds 15 s to every run. If it IS running it must be disabled — it holds
            # a 15 m NED floor and streams at 10 Hz, and it wins every conflict.
            if not cli.wait_for_service(timeout_sec=1.0):
                return False
            r = SetParameters.Request()
            r.parameters = [Parameter(name="enabled", value=on).to_parameter_msg()]
            fut = cli.call_async(r)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
            return bool(fut.done())
        finally:
            self.destroy_client(cli)

    def fly_to(self, x, y, label):
        self.leg = label
        end = time.time() + LEG_TIMEOUT_S
        while time.time() < end:
            now = int(time.time() * 1e6)
            m = OffboardControlMode(); m.position = True; m.timestamp = now
            self._mode.publish(m)
            s = TrajectorySetpoint(); s.timestamp = now
            s.position = [float(x), float(y), float(ALT)]
            s.velocity = [float("nan")] * 3
            if self.pos is not None:
                # Face the way we are going: the camera is fixed at -28.6 deg and cannot pan,
                # so heading is the only control over what the recording shows.
                s.yaw = math.atan2(y - self.pos[1], x - self.pos[0])
            self._sp.publish(s)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pos and math.dist(self.pos[:2], (x, y)) <= ARRIVE_M:
                return True
        return False


def main():
    with open(os.path.join(PROJ, "out", "street-route.json")) as fh:
        route = [tuple(p) for p in json.load(fh)]

    stamp = time.strftime("%Y%m%d-%H%M%S")
    outdir = os.path.join(PROJ, "out", "street")
    os.makedirs(outdir, exist_ok=True)
    chase_path = os.path.join(outdir, f"{stamp}-chase.mp4")
    onboard_path = os.path.join(outdir, f"{stamp}-onboard.mp4")

    rclpy.init()
    n = StreetLevel()
    try:
        for k in n.srv:
            n.srv[k].wait_for_service(timeout_sec=45.0)
        n.call("destroy", DestroyActors.Request())

        t = SpawnTraffic.Request()
        t.vehicles, t.walkers = 40, 40
        t.near_ned = [route[0][0], route[0][1], ALT]
        t.radius_m = 70.0
        got = n.call("traffic", t)
        print(f"  traffic: {got.spawned} vehicles, {got.walkers_spawned} walkers", flush=True)

        n.offboard(False)
        time.sleep(0.5)

        r = ResetVehicle.Request()
        r.hold_ned = [route[0][0], route[0][1], ALT]
        r.speed = 6.0
        res = n.call("reset", r)
        print(f"  placed at {[round(v,1) for v in res.position_ned]} "
              f"= {GROUND_OFFSET - res.position_ned[2]:.1f} m AGL", flush=True)

        c = ChaseRecording.Request()
        c.start, c.path = True, chase_path
        c.width, c.height, c.fps = 1280, 720, 30.0
        # Closer and lower than the default: at 3.5 m a 14 m/6 m chase looks down at rooftops.
        if not n.call("chase", c).success:
            chase_path = None

        n.writer = VideoWriter(onboard_path, 960, 720, fps=8.0, crf=23)
        print(f"  flying {len(route)} waypoints at {GROUND_OFFSET - ALT:.1f} m AGL", flush=True)
        for i, (x, y) in enumerate(route[1:], 1):
            ok = n.fly_to(x, y, f"wp {i}/{len(route) - 1}")
            if not ok:
                print(f"    wp {i}: timed out", flush=True)
            if n.hit:
                print(f"    wp {i}: CONTACT {n.hit}", flush=True)

    finally:
        writer, n.writer = n.writer, None
        if writer is not None:
            writer.close()
        try:
            if chase_path:
                g = n.call("chase", ChaseRecording.Request(start=False), timeout=60.0)
                print(f"  chase:   {g.frames} frames, {g.dropped} dropped", flush=True)
            print(f"  onboard: {n.frames} frames", flush=True)
            n.call("destroy", DestroyActors.Request(), timeout=60.0)
            n.offboard(True)
        except Exception:                                              # noqa: BLE001
            import traceback; traceback.print_exc()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
