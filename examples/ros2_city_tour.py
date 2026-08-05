#!/usr/bin/env python3
"""Fly a route across the city with traffic and pedestrians alive underneath, and record it.

    ./scripts/bringup.sh --config configs/testbed.yaml     # terminal 1
    python3 examples/ros2_city_tour.py                     # terminal 2 (ROS 2 python)

A demonstration, not a scored episode: there is no goal and nothing is measured. It exists to
show the world running — cars driving, pedestrians walking — from both the exterior chase
camera and the drone's own.

    out/tour/<stamp>-chase.mp4     exterior, following the aircraft, 1280x720 H.264
    out/tour/<stamp>-onboard.mp4   the drone camera

Imports `rclpy`, `interfaces`, `px4_msgs` and `sensor_msgs`. Nothing from the 3.10 side.

**Traffic is spawned MAP-WIDE here**, which is the opposite of what a scenario wants. A
scenario clusters its fleet around one start so the camera sees a busy neighbourhood; a tour
crosses several neighbourhoods, so clustering would leave most of the route empty. Passing an
empty `near_ned` is what asks for map-wide — see the note in SpawnTraffic.srv.
"""
from __future__ import annotations

import math
import os
import sys
import time

import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Image
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleOdometry

from interfaces.srv import ChaseRecording, DestroyActors, ResetVehicle, SpawnTraffic

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJ, "sim_bridge", "carla_air"))
from h264 import VideoWriter  # noqa: E402

PX4_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=5)

#: NED. -30 is 57.5 m above the street: high enough to see down a block, low enough that a
#: car is a car. Scenarios fly at 72-107 m AGL, where traffic is confetti.
ALT = -30.0

#: A circuit through populated parts of Town10HD. Every one of these was probed for spawn
#: density before it went in the list - five of seven candidates accepted 40 vehicles inside
#: 80 m, and these are those five, ordered into a loop that does not double back.
ROUTE = [
    (107.6, -159.4),      # the plaza
    (150.0, -100.0),
    (250.0, -160.0),
    (210.0, -230.0),
    (60.0, -200.0),
    (107.6, -159.4),      # home
]

ARRIVE_M = 12.0
LEG_TIMEOUT_S = 45.0


class Tour(Node):
    def __init__(self):
        super().__init__("ros2_city_tour")
        self.srv = {
            "reset": self.create_client(ResetVehicle, "/sim/reset_vehicle"),
            "traffic": self.create_client(SpawnTraffic, "/sim/spawn_traffic"),
            "destroy": self.create_client(DestroyActors, "/sim/destroy_actors"),
            "chase": self.create_client(ChaseRecording, "/sim/chase_recording"),
        }
        self.bridge = CvBridge()
        self.writer = None
        self.frames = 0
        self.pos = None
        self._sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint",
                                         PX4_QOS)
        self._mode = self.create_publisher(OffboardControlMode,
                                           "/fmu/in/offboard_control_mode", PX4_QOS)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry",
                                 self._on_odom, PX4_QOS)
        self.create_subscription(Image, "/camera/rgb/image_raw", self._on_image, 5)

    def _on_odom(self, msg):
        self.pos = [float(v) for v in msg.position]

    def _on_image(self, msg):
        if self.writer is None:
            return
        self.writer.write(self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"))
        self.frames += 1

    def wait(self, timeout=45.0):
        missing = [k for k, c in self.srv.items() if not c.wait_for_service(timeout_sec=timeout)]
        if missing:
            raise SystemExit(f"missing services: {', '.join(missing)}\n"
                             "Is the simulator up?  ./scripts/bringup.sh --config configs/testbed.yaml")

    def call(self, name, req, timeout=180.0):
        f = self.srv[name].call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=timeout)
        if not f.done():
            raise RuntimeError(f"{name}: no reply after {timeout}s")
        return f.result()

    def offboard(self, on):
        cli = self.create_client(SetParameters, "/offboard_control/set_parameters")
        try:
            # Short: `offboard_control` is in examples/navigation and bringup.sh has not
            # started it since 2026-08-04, so absence is the ordinary case and waiting 15 s
            # for it adds 15 s to every run. If it IS running it must be disabled — it holds
            # a 15 m NED floor and streams at 10 Hz, and it wins every conflict.
            if not cli.wait_for_service(timeout_sec=1.0):
                return False
            req = SetParameters.Request()
            req.parameters = [Parameter(name="enabled", value=on).to_parameter_msg()]
            f = cli.call_async(req)
            rclpy.spin_until_future_complete(self, f, timeout_sec=15.0)
            return bool(f.done())
        finally:
            self.destroy_client(cli)

    def fly_to(self, x, y, z, label):
        """Stream a position setpoint until we arrive or the leg times out.

        Yaw is set toward the destination, so the drone camera looks WHERE IT IS GOING. The
        camera is fixed at -28.6 deg pitch and cannot be panned in flight, so the heading is
        the only control over what the onboard recording actually shows.
        """
        deadline = time.time() + LEG_TIMEOUT_S
        while time.time() < deadline:
            now = int(time.time() * 1e6)
            m = OffboardControlMode(); m.position = True; m.timestamp = now
            self._mode.publish(m)
            s = TrajectorySetpoint(); s.timestamp = now
            s.position = [float(x), float(y), float(z)]
            s.velocity = [float("nan")] * 3
            if self.pos is not None:
                s.yaw = math.atan2(y - self.pos[1], x - self.pos[0])
            self._sp.publish(s)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pos is not None and math.dist(self.pos[:2], (x, y)) <= ARRIVE_M:
                print(f"    {label}: arrived, {math.dist(self.pos[:2], (x, y)):.1f} m",
                      flush=True)
                return True
        d = math.dist(self.pos[:2], (x, y)) if self.pos else float("nan")
        print(f"    {label}: timed out {d:.1f} m short", flush=True)
        return False


def main():
    stamp = time.strftime("%Y%m%d-%H%M%S")
    outdir = os.path.join(PROJ, "out", "tour")
    os.makedirs(outdir, exist_ok=True)
    chase_path = os.path.join(outdir, f"{stamp}-chase.mp4")
    onboard_path = os.path.join(outdir, f"{stamp}-onboard.mp4")

    rclpy.init()
    n = Tour()
    try:
        n.wait()
        n.call("destroy", DestroyActors.Request())

        print("spawning traffic MAP-WIDE (a tour crosses neighbourhoods)", flush=True)
        t = SpawnTraffic.Request()
        t.vehicles, t.walkers = 90, 70
        t.near_ned = []                      # empty = map-wide, see SpawnTraffic.srv
        t.radius_m = 0.0
        got = n.call("traffic", t)
        print(f"  {got.spawned} vehicles, {got.walkers_spawned} walkers", flush=True)

        n.offboard(False)
        time.sleep(0.5)

        start = ROUTE[0]
        print(f"placing at {start} / {ALT} NED ({abs(ALT) + 27.45:.0f} m above the street)",
              flush=True)
        r = ResetVehicle.Request()
        r.hold_ned = [start[0], start[1], ALT]
        r.speed = 10.0
        res = n.call("reset", r)
        if not res.success:
            raise SystemExit(f"reset: {res.message}")

        c = ChaseRecording.Request()
        c.start, c.path = True, chase_path
        c.width, c.height, c.fps = 1280, 720, 30.0
        if not n.call("chase", c).success:
            chase_path = None
            print("  chase camera unavailable; onboard only", flush=True)

        n.writer = VideoWriter(onboard_path, 640, 480, fps=8.0, crf=23)
        print(f"touring {len(ROUTE) - 1} legs", flush=True)
        for i, (x, y) in enumerate(ROUTE[1:], 1):
            print(f"  leg {i}/{len(ROUTE) - 1} -> ({x}, {y})", flush=True)
            n.fly_to(x, y, ALT, f"leg {i}")

    finally:
        # Detach before closing: everything below spins the node, and a queued image
        # callback writing into a closed VideoWriter raises from inside PyAV.
        writer, n.writer = n.writer, None
        if writer is not None:
            writer.close()
        try:
            if chase_path:
                got = n.call("chase", ChaseRecording.Request(start=False), timeout=60.0)
                print(f"  chase:   {got.frames} frames, {got.dropped} dropped", flush=True)
            print(f"  onboard: {n.frames} frames", flush=True)
            n.call("destroy", DestroyActors.Request(), timeout=60.0)
            n.offboard(True)
        except Exception:                                              # noqa: BLE001
            import traceback
            traceback.print_exc()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
