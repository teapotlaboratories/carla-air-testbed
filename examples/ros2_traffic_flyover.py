#!/usr/bin/env python3
"""Put the aircraft over a busy street and record both views — from plain ROS 2.

    ./scripts/bringup.sh --config configs/testbed.yaml      # terminal 1
    python3 examples/ros2_traffic_flyover.py                # terminal 2 (ROS 2 python)

Not a scored episode and not a benchmark: this exists to *show the actors*. It spawns a
dense fleet in a tight radius, places the aircraft low enough that cars and pedestrians are
recognisable rather than specks, and records:

    out/traffic/<stamp>-chase.mp4     exterior, following the aircraft, 1280x720 H.264
    out/traffic/<stamp>-onboard.mp4   what the drone's own camera sees

Imports are `rclpy`, `interfaces`, `px4_msgs` and `sensor_msgs`. Nothing from this project's
3.10 side — the same rule the other examples follow.

**Altitude is the whole trick.** The NED origin sits 27.45 m above the street, so an
altitude in NED is not an altitude above ground: `z = -8` is 35.4 m AGL, not 8. Scenarios fly
at 72-107 m AGL, which is above most of Town10HD and why traffic looks like confetti in
episode footage. This goes much lower, which means `offboard_control` has to be told to stop
pushing the aircraft back up to its 15 m NED floor (42.45 m AGL).
"""
from __future__ import annotations

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
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint

from interfaces.srv import ChaseRecording, DestroyActors, ResetVehicle, SpawnTraffic

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PROJ, "sim_bridge", "carla_air"))
from h264 import VideoWriter  # noqa: E402  — shared with the sidecar, see recorder_node

PX4_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=5)

#: The plaza. Measured densest of seven candidates probed on Town10HD: it is the only one
#: that accepted 40 vehicles AND 30 of 30 walkers inside an 80 m radius.
CENTRE = (107.6, -159.4)
#: NED. -8.0 is 35.4 m above the street; low enough that a person is a person.
ALTITUDE_NED = -8.0
SECONDS = 40.0


class Flyover(Node):
    def __init__(self):
        super().__init__("ros2_traffic_flyover")
        self.srv = {
            "reset": self.create_client(ResetVehicle, "/sim/reset_vehicle"),
            "traffic": self.create_client(SpawnTraffic, "/sim/spawn_traffic"),
            "destroy": self.create_client(DestroyActors, "/sim/destroy_actors"),
            "chase": self.create_client(ChaseRecording, "/sim/chase_recording"),
        }
        self.bridge = CvBridge()
        self.frames = 0
        self.writer = None
        self._sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint",
                                         PX4_QOS)
        self._mode = self.create_publisher(OffboardControlMode,
                                           "/fmu/in/offboard_control_mode", PX4_QOS)
        self.create_subscription(Image, "/camera/rgb/image_raw", self._on_image, 5)

    # ---------------------------------------------------------------- plumbing

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
        """The controller holds a 15 m NED floor. Below that it fights us."""
        cli = self.create_client(SetParameters, "/offboard_control/set_parameters")
        try:
            if not cli.wait_for_service(timeout_sec=15.0):
                return False
            req = SetParameters.Request()
            req.parameters = [Parameter(name="enabled", value=on).to_parameter_msg()]
            f = cli.call_async(req)
            rclpy.spin_until_future_complete(self, f, timeout_sec=15.0)
            return bool(f.done())
        finally:
            self.destroy_client(cli)

    def _on_image(self, msg):
        if self.writer is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.writer.write(frame)
        self.frames += 1

    def hold(self, ned, seconds, yaw_rate=0.0):
        """Stream a position setpoint ourselves, since the controller is disabled."""
        end = time.time() + seconds
        while time.time() < end:
            now = int(time.time() * 1e6)
            m = OffboardControlMode(); m.position = True; m.timestamp = now
            self._mode.publish(m)
            s = TrajectorySetpoint(); s.timestamp = now
            s.position = [float(ned[0]), float(ned[1]), float(ned[2])]
            s.velocity = [float("nan")] * 3
            if yaw_rate:
                s.yawspeed = float(yaw_rate)
            self._sp.publish(s)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    stamp = time.strftime("%Y%m%d-%H%M%S")
    outdir = os.path.join(PROJ, "out", "traffic")
    os.makedirs(outdir, exist_ok=True)
    chase_path = os.path.join(outdir, f"{stamp}-chase.mp4")
    onboard_path = os.path.join(outdir, f"{stamp}-onboard.mp4")

    rclpy.init()
    n = Flyover()
    try:
        n.wait()
        print("clearing any leftover actors", flush=True)
        n.call("destroy", DestroyActors.Request())

        print(f"spawning traffic around {CENTRE} within 60 m", flush=True)
        t = SpawnTraffic.Request()
        # 40, not 60: the map has only ~45 spawn points inside this radius, and asking for
        # more makes the sidecar fall back to MAP-WIDE - which spreads the fleet across
        # Town10HD and leaves the aircraft over an empty street. The service reports that
        # fallback now (see todo.md R-01), which is how this was caught on the first run.
        t.vehicles, t.walkers = 40, 40
        t.near_ned = [CENTRE[0], CENTRE[1], ALTITUDE_NED]
        t.radius_m = 70.0
        got = n.call("traffic", t)
        print(f"  {got.spawned} vehicles, {got.walkers_spawned} walkers"
              f"{'  (' + got.message + ')' if got.message else ''}", flush=True)

        # Take the controller off the aircraft BEFORE the reset: its 15 m NED floor would
        # drag us back up to 42 m AGL, which is the altitude that makes traffic unreadable.
        n.offboard(False)
        time.sleep(0.5)

        print(f"placing the aircraft at {ALTITUDE_NED} NED "
              f"({abs(ALTITUDE_NED) + 27.45:.1f} m above the street)", flush=True)
        r = ResetVehicle.Request()
        r.hold_ned = [CENTRE[0], CENTRE[1], ALTITUDE_NED]
        r.speed = 8.0
        res = n.call("reset", r)
        if not res.success:
            raise SystemExit(f"reset: {res.message}")
        print(f"  settled at {[round(v, 1) for v in res.position_ned]}", flush=True)

        c = ChaseRecording.Request()
        c.start, c.path = True, chase_path
        c.width, c.height, c.fps = 1280, 720, 30.0
        if not n.call("chase", c).success:
            print("  chase camera unavailable; continuing with onboard only", flush=True)
            chase_path = None

        n.writer = VideoWriter(onboard_path, 640, 480, fps=8.0, crf=23)
        print(f"recording {SECONDS:.0f}s — slow yaw to sweep the street", flush=True)
        # A slow yaw rather than a translation: the point is to see the actors move under a
        # stationary camera, not to cover ground.
        n.hold([CENTRE[0], CENTRE[1], ALTITUDE_NED], SECONDS, yaw_rate=0.16)

    finally:
        # Detach BEFORE closing. Everything below spins the node, which delivers image
        # callbacks that were already queued - and writing to a closed VideoWriter raises
        # `'NoneType' object has no attribute 'encode'` from inside PyAV, which names
        # nothing about the actual race.
        writer, n.writer = n.writer, None
        if writer is not None:
            writer.close()
        try:
            if chase_path:
                got = n.call("chase", ChaseRecording.Request(start=False), timeout=60.0)
                print(f"  chase:   {got.frames} frames, {got.dropped} dropped -> {chase_path}",
                      flush=True)
            print(f"  onboard: {n.frames} frames -> {onboard_path}", flush=True)
            n.call("destroy", DestroyActors.Request(), timeout=60.0)
            n.offboard(True)
        except Exception:                                              # noqa: BLE001
            import traceback
            print("  cleanup failed:", flush=True)
            traceback.print_exc()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
