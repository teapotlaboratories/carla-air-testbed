#!/usr/bin/env python3
"""Bring your own agent: a template for wiring YOUR model to this simulator.

    # in the containerised stack
    ./scripts/stack_up.sh --config configs/testbed.yaml
    ./scripts/stack_run.sh -d examples/navigation/run.sh
    ./scripts/stack_run.sh examples/byo_agent.py

    # or on the host, with shared memory off so the graph is reachable
    export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/configs/dds/udp-only.xml
    export ROS_DOMAIN_ID=42
    python3 examples/byo_agent.py

This imports NOTHING from this project. That is the point: the simulator's interface is
ROS 2, and anything that can speak it can fly the aircraft. Copy this file out of the
repository and it still works.

---------------------------------------------------------------------------------------
THREE PLACES TO PLUG IN, and the choice is how much geometry you want to own.

  1. publish /vlm/annotation          — a PIXEL. You look at the frame and say "there".
                                        The project's grounding turns that into a 3D point
                                        and its controller flies to it. Least work; you
                                        never touch metres or frames.
                                        NEEDS: examples/navigation + the grounding node.

  2. publish /control/waypoint        — a POINT IN NED METRES. You do your own
                                        pixel-to-world, the project's controller flies it.
                                        NEEDS: examples/navigation.

  3. publish /fmu/in/trajectory_setpoint — RAW SETPOINTS at 10 Hz. You own guidance
                                        entirely and need nothing from examples/ at all.
                                        This is the only option that talks purely to the
                                        simulator. See examples/ros2_full_control.py.

This file implements (1) and shows (2) and (3) in `send_waypoint_ned` and
`send_setpoint_ned`, which are left unused so you can delete what you do not want.
---------------------------------------------------------------------------------------
"""
from __future__ import annotations

import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, Image

# The project's message package. It is the ONE import that is not stock ROS, and it is here
# because a pixel annotation is not a standard type. Options (2) and (3) below need
# `interfaces` and `px4_msgs` respectively; option (3) needs nothing from this project.
from interfaces.msg import Annotation2D, GroundedWaypoint
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleOdometry

#: PX4 topics are BEST_EFFORT + TRANSIENT_LOCAL. A RELIABLE subscriber silently receives
#: nothing from them — no error, no warning, just an empty callback that never fires. This
#: is the single most common way to wire up a client that "does not work".
PX4_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=5)


class ByoAgent(Node):
    def __init__(self):
        super().__init__("byo_agent")

        # ---- what you READ ------------------------------------------------------------
        # The camera. Images are plain sensor_msgs/Image in bgr8; decode with cv_bridge or
        # numpy as below. `camera_info` carries the REAL intrinsics — read them from the
        # topic, never hard-code them. They follow simulator.cameras.rgb in the config, and
        # a stale hard-coded copy is how this project shipped a doc that was 1.5x wrong.
        self.create_subscription(Image, "/camera/rgb/image_raw", self.on_frame, 1)
        self.create_subscription(CameraInfo, "/camera/rgb/camera_info", self.on_info, 5)
        # Where the aircraft is. NED metres, and note the PX4 QoS above.
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry",
                                 self.on_odom, PX4_QOS)

        # ---- what you WRITE -----------------------------------------------------------
        self.pub_annotation = self.create_publisher(Annotation2D, "/vlm/annotation", 5)
        self.pub_waypoint = self.create_publisher(GroundedWaypoint, "/control/waypoint", 5)
        self.pub_setpoint = self.create_publisher(TrajectorySetpoint,
                                                  "/fmu/in/trajectory_setpoint", PX4_QOS)
        self.pub_mode = self.create_publisher(OffboardControlMode,
                                              "/fmu/in/offboard_control_mode", PX4_QOS)

        self.info = None
        self.odom = None
        self.frames = 0
        self.get_logger().info("byo_agent up — waiting for frames on /camera/rgb/image_raw")

    # ------------------------------------------------------------------ inputs

    def on_info(self, msg: CameraInfo):
        # k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]. Measured on this simulator at 960x720:
        # fx 480.81, cx 480.0, cy 360.0 — and they change with the configured resolution.
        self.info = msg

    def on_odom(self, msg: VehicleOdometry):
        self.odom = [float(v) for v in msg.position]

    def on_frame(self, msg: Image):
        """One camera frame. THIS is where your model goes."""
        self.frames += 1
        # bgr8 -> HxWx3 uint8, without cv_bridge so this file stays copyable.
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

        # ------------------------------------------------------------------------------
        # YOUR MODEL HERE. In: `frame`. Out: a pixel to fly toward.
        #
        # The placeholder below picks the centre column, slightly above the middle — i.e.
        # "straight ahead". Replace it with whatever your model returns.
        # ------------------------------------------------------------------------------
        u = msg.width // 2
        v = int(msg.height * 0.45)
        confidence = 0.0                      # 0 when a backend cannot say
        rationale = "placeholder: straight ahead"

        self.send_annotation(msg, u, v, confidence, rationale)

        if self.frames % 20 == 0:
            self.get_logger().info(
                f"{self.frames} frames — {msg.width}x{msg.height} {msg.encoding}, "
                f"last pixel ({u},{v})")

    # ------------------------------------------------------------------ outputs

    def send_annotation(self, image_msg, u, v, confidence=0.0, rationale=""):
        """OPTION 1 — a pixel. Grounding turns it into a point; the controller flies it.

        The header stamp must be the stamp of the IMAGE this annotates, not `now`.
        Grounding matches the annotation against the depth frame nearest that time, and a
        `now` stamp silently pairs your pixel with the wrong depth.
        """
        a = Annotation2D()
        a.header = image_msg.header
        a.u, a.v = int(u), int(v)
        a.confidence = float(confidence)
        a.rationale = str(rationale)
        a.backend = "byo"
        a.terminal = False                     # True when you believe the task is done
        self.pub_annotation.publish(a)

    def send_waypoint_ned(self, x, y, z):
        """OPTION 2 — a point in NED metres. You did your own pixel-to-world.

        NED: +x north, +y east, +z DOWN. The origin sits 27.45 m above the street on
        Town10HD, so z = -55 is 82.5 m above ground and a POSITIVE z is below the origin.
        Unused here; wired so you can delete option 1 and call this instead.
        """
        w = GroundedWaypoint()
        w.header.stamp = self.get_clock().now().to_msg()
        w.position = Point(x=float(x), y=float(y), z=float(z))
        w.valid = True
        w.bearing_only = False                 # True = direction is real, range is a guess
        w.reason = "byo agent"
        self.pub_waypoint.publish(w)

    def send_setpoint_ned(self, x, y, z, yaw=None):
        """OPTION 3 — a raw setpoint. Needs nothing from examples/.

        Must be published CONTINUOUSLY, ~10 Hz, alongside OffboardControlMode. A gap in the
        stream is not "hold position", it is "no active setpoint" — and the aircraft has
        been measured climbing at 7 m/s until someone noticed. See
        examples/ros2_full_control.py for the full pattern.
        """
        now = int(self.get_clock().now().nanoseconds / 1000)
        m = OffboardControlMode()
        m.position = True
        m.timestamp = now
        self.pub_mode.publish(m)

        s = TrajectorySetpoint()
        s.timestamp = now
        s.position = [float(x), float(y), float(z)]
        s.velocity = [float("nan")] * 3
        if yaw is not None:
            s.yaw = float(yaw)
        self.pub_setpoint.publish(s)


def main():
    rclpy.init()
    node = ByoAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
