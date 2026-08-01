#!/usr/bin/env python3
"""Turn the CARLA-Air simulator into a PX4-shaped ROS 2 graph.

This node is the **only** place that knows the simulator exists. Everything downstream —
the VLM client, the grounding node, the controller, the episode runner — talks to
`/fmu/out/*` and `/fmu/in/*` exactly as it would against a real Pixhawk 6C. Moving to
hardware means deleting this node and starting `uxrce_dds_client`, not rewriting the stack.

**This is deliberately a shim, and the shim is not free.** CARLA-Air has no PX4 in it at
all — no MAVLink, no uORB, no lockstep; the flight controller is AirSim's SimpleFlight. So
the PX4 semantics below are *emulated*, and three of them are worth knowing before trusting
a number that came out of here:

* `vehicle_odometry.timestamp` is host time, not a PX4 boot clock synced over
  `timesync_status`. Latency figures across this bridge measure the bridge, not a flight
  controller.
* `vehicle_status.arming_state` follows AirSim's API-control flag. There is no failsafe
  state machine behind it, so nothing here exercises PX4's arming logic.
* `TrajectorySetpoint` velocity is forwarded to `moveByVelocityAsync`. Position setpoints
  are honoured via `moveToPosition`; acceleration and jerk fields are ignored, because
  SimpleFlight has nowhere to put them.

**Three connections to the sidecar, not one.** Image capture takes ~500 ms; telemetry takes
~5 ms. Sharing one connection puts every odometry read behind the current capture and the
measured `/fmu/out/vehicle_odometry` rate collapses from 20 Hz to 1.5 Hz — which starves the
10 Hz controller of fresh position and makes it steer on stale data. Splitting control off
as well was worth another step: sharing one AirSim client between 20 Hz of telemetry and
10 Hz of setpoints capped odometry at 12.3 Hz. So `self.sim` is telemetry and world,
`self.media` is image capture, `self.ctrl` is setpoint forwarding, and the sidecar backs
each with its own AirSim client.

The odometry rate is what the sim can actually serve, not PX4's 100 Hz.
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from .client import SimBridgeClient, SimBridgeError

# PX4's uXRCE-DDS bridge publishes best-effort with a small keep-last depth, and real
# subscribers are written against that. Matching it here means a node that works in this
# testbed does not silently fail on hardware because of a QoS mismatch.
PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


def _advance(deadline: float, now: float, period: float) -> float:
    """Next deadline, anchored to the schedule rather than to `now`.

    Re-anchoring on `now` looks equivalent and is not: the timer fires a little late, the
    next deadline is pushed out by that lateness, and the error compounds. Measured, a
    20 Hz timer guarded that way settled at 11.8 Hz. Anchoring on the previous deadline
    holds the cadence; falling more than one period behind (a long blocking call) resyncs
    to now instead of replaying the backlog as a burst.
    """
    return deadline + period if (now - deadline) < period else now + period


class CarlaAirBridge(Node):
    def __init__(self):
        super().__init__("carla_air_bridge")

        self.declare_parameter("socket_path", "/tmp/carla_air_testbed.sock")
        self.declare_parameter("odometry_rate_hz", 20.0)
        self.declare_parameter("image_rate_hz", 4.0)
        self.declare_parameter("publish_depth", True)
        self.declare_parameter("publish_segmentation", False)
        self.declare_parameter("frame_id", "drone")
        self.declare_parameter("setpoint_duration_s", 0.5)

        self._frame_id = self.get_parameter("frame_id").value
        self._cv = CvBridge()
        self._seq = 0
        self._last_camera = None
        self._offboard_active = False
        # rclpy timers accumulate missed firings and replay them back-to-back once a long
        # callback returns. The sidecar blocks for SECONDS on reset/goto/spawn_traffic, so
        # without a guard the odometry timer bursts afterwards — measured at 110 Hz against
        # a 20 Hz setting, 1107 unique timestamps in 10 s. That burst is not free telemetry:
        # every firing is a blocking RPC that competes with the image path for the sidecar.
        # Collapse a backlog to a single firing.
        self._next_odom = 0.0
        self._next_image = 0.0


        socket_path = self.get_parameter("socket_path").value
        self.sim = SimBridgeClient(socket_path)      # telemetry + world
        self.media = SimBridgeClient(socket_path)    # image capture only
        self.ctrl = SimBridgeClient(socket_path)     # setpoint forwarding only
        try:
            self.sim.connect()
            self.media.connect()
            self.ctrl.connect()
        except OSError as exc:
            self.get_logger().error(
                f"cannot reach sim_bridge at {socket_path}: {exc}. "
                "Start it with: ./.venv/bin/python sim_bridge/server.py")
            raise
        info = self.sim.describe()
        self.get_logger().info(
            f"bridged to CARLA-Air — map={info['map']} hfov={info['camera']['hfov_deg']:.1f}deg "
            f"server={info['carla_server_version']}")

        # ---- PX4-shaped outputs ----
        self.pub_odom = self.create_publisher(VehicleOdometry, "/fmu/out/vehicle_odometry", PX4_QOS)
        self.pub_local = self.create_publisher(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", PX4_QOS)
        self.pub_status = self.create_publisher(VehicleStatus, "/fmu/out/vehicle_status", PX4_QOS)

        # ---- sensors (ordinary ROS types; nothing PX4 about a camera) ----
        self.pub_rgb = self.create_publisher(Image, "/camera/rgb/image_raw", 5)
        self.pub_info = self.create_publisher(CameraInfo, "/camera/rgb/camera_info", 5)
        self.pub_depth = self.create_publisher(Image, "/camera/depth/image_raw", 5)
        self.pub_seg = self.create_publisher(Image, "/camera/segmentation/image_raw", 5)
        # The camera's world pose, straight from simGetCameraInfo. Publishing it makes the
        # bridge the single source of truth: before this, `grounding` and the oracle each
        # reconstructed it from odometry plus their own camera_pitch_deg parameter, so a
        # pitch change in one place silently put every waypoint somewhere else.
        self.pub_campose = self.create_publisher(PoseStamped, "/camera/pose", 5)

        # ---- testbed-side signals the sim can answer but a Pixhawk cannot ----
        self.pub_collision = self.create_publisher(Bool, "/sim/collision", 5)
        self.pub_traffic = self.create_publisher(String, "/sim/traffic_stats", 5)

        # ---- PX4-shaped inputs ----
        setpoint_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self._on_setpoint, PX4_QOS,
            callback_group=setpoint_group)
        self.create_subscription(
            OffboardControlMode, "/fmu/in/offboard_control_mode", self._on_offboard, PX4_QOS,
            callback_group=setpoint_group)

        # One callback group per channel. Without this the executor serialises the timers
        # in a single thread and the two connections buy nothing.
        odo_hz = float(self.get_parameter("odometry_rate_hz").value)
        img_hz = float(self.get_parameter("image_rate_hz").value)
        self.create_timer(1.0 / odo_hz, self._tick_odometry,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0 / img_hz, self._tick_images,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0, self._tick_world,
                          callback_group=MutuallyExclusiveCallbackGroup())

    # ------------------------------------------------------------------ outputs

    def _stamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _tick_odometry(self):
        now = time.monotonic()
        if now < self._next_odom:
            return
        self._next_odom = _advance(
            self._next_odom, now, 1.0 / float(self.get_parameter("odometry_rate_hz").value))
        try:
            s = self.sim.state()
        except (SimBridgeError, OSError) as exc:
            self.get_logger().warn(f"state() failed: {exc}", throttle_duration_sec=5.0)
            return

        now = self._stamp_us()
        p, v, q = s["position"], s["velocity"], s["orientation"]

        odom = VehicleOdometry()
        odom.timestamp = now
        odom.timestamp_sample = now
        odom.pose_frame = VehicleOdometry.POSE_FRAME_NED
        odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        odom.position = np.array(p, dtype=np.float32)
        odom.q = np.array(q, dtype=np.float32)
        odom.velocity = np.array(v, dtype=np.float32)
        odom.angular_velocity = np.array(s["angular_velocity"], dtype=np.float32)
        self.pub_odom.publish(odom)

        loc = VehicleLocalPosition()
        loc.timestamp = now
        loc.timestamp_sample = now
        loc.xy_valid = loc.z_valid = loc.v_xy_valid = loc.v_z_valid = True
        loc.x, loc.y, loc.z = (float(c) for c in p)
        loc.vx, loc.vy, loc.vz = (float(c) for c in v)
        loc.heading = float(s["yaw"])
        self.pub_local.publish(loc)

        st = VehicleStatus()
        st.timestamp = now
        st.arming_state = (VehicleStatus.ARMING_STATE_ARMED if s["armed"]
                           else VehicleStatus.ARMING_STATE_DISARMED)
        st.nav_state = (VehicleStatus.NAVIGATION_STATE_OFFBOARD if self._offboard_active
                        else VehicleStatus.NAVIGATION_STATE_AUTO_LOITER)
        self.pub_status.publish(st)

    def _tick_images(self):
        now = time.monotonic()
        if now < self._next_image:
            return
        self._next_image = _advance(
            self._next_image, now, 1.0 / float(self.get_parameter("image_rate_hz").value))
        want_depth = bool(self.get_parameter("publish_depth").value)
        want_seg = bool(self.get_parameter("publish_segmentation").value)
        try:
            cap = self.media.capture(rgb=True, depth=want_depth, segmentation=want_seg)
        except (SimBridgeError, OSError) as exc:
            self.get_logger().warn(f"capture() failed: {exc}", throttle_duration_sec=5.0)
            return

        stamp = self.get_clock().now().to_msg()
        imgs = cap["images"]
        self._last_camera = cap["camera"]

        cam = cap["camera"]
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            float(c) for c in cam["position"])
        qw, qx, qy, qz = (float(c) for c in cam["orientation"])
        pose.pose.orientation.w, pose.pose.orientation.x = qw, qx
        pose.pose.orientation.y, pose.pose.orientation.z = qy, qz
        self.pub_campose.publish(pose)

        rgb = imgs.get("rgb")
        if rgb is not None:
            msg = self._cv.cv2_to_imgmsg(rgb, encoding="bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            self.pub_rgb.publish(msg)
            self.pub_info.publish(self._camera_info(rgb.shape, stamp))

        depth = imgs.get("depth")
        if depth is not None:
            # 32FC1 metres, PX4-irrelevant but the ROS convention. AirSim marks sky with
            # float16 max (65504.0); republish it as +inf so consumers can use isfinite().
            d = depth.astype(np.float32).copy()
            d[d >= 65504.0 * 0.99] = np.inf
            msg = self._cv.cv2_to_imgmsg(d, encoding="32FC1")
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            self.pub_depth.publish(msg)

        seg = imgs.get("segmentation")
        if seg is not None:
            msg = self._cv.cv2_to_imgmsg(seg, encoding="bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            self.pub_seg.publish(msg)

    def _camera_info(self, shape, stamp) -> CameraInfo:
        h, w = shape[0], shape[1]
        hfov = math.radians(self._last_camera["hfov_deg"])
        fx = (w / 2.0) / math.tan(hfov / 2.0)
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self._frame_id
        info.width, info.height = int(w), int(h)
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, w / 2.0, 0.0, fx, h / 2.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, w / 2.0, 0.0, 0.0, fx, h / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _tick_world(self):
        try:
            col = self.sim.collision()
            self.pub_collision.publish(Bool(data=bool(col["has_collided"])))
            # Traffic stalls intermittently (4/15 vs 11/15 across two runs); upstream ships
            # a watchdog for it and so do we. Without this an episode's "urban traffic" can
            # quietly become a car park.
            self.sim.watchdog()
            stats = self.sim.traffic_stats()
            self.pub_traffic.publish(String(
                data=f"spawned={stats['spawned']} moving={stats['moving']} "
                     f"walkers={stats['walkers']}"))
        except (SimBridgeError, OSError) as exc:
            self.get_logger().warn(f"world tick failed: {exc}", throttle_duration_sec=10.0)

    # ------------------------------------------------------------------- inputs

    def _on_offboard(self, msg: OffboardControlMode):
        self._offboard_active = bool(msg.velocity or msg.position)

    def _on_setpoint(self, msg: TrajectorySetpoint):
        """Forward a PX4 setpoint to SimpleFlight.

        Velocity wins when both are finite: the SPF inner loop streams velocity, and a
        position field left at NaN is PX4's way of saying "ignore me".
        """
        duration = float(self.get_parameter("setpoint_duration_s").value)
        yaw_deg = math.degrees(msg.yaw) if math.isfinite(msg.yaw) else None
        try:
            if all(math.isfinite(c) for c in msg.velocity):
                self.ctrl.velocity(msg.velocity[0], msg.velocity[1], msg.velocity[2],
                                   duration=duration, yaw_deg=yaw_deg)
            elif all(math.isfinite(c) for c in msg.position):
                self.ctrl.call("velocity", vx=0.0, vy=0.0, vz=0.0, duration=0.1, yaw_deg=None)
                self.ctrl.goto(msg.position[0], msg.position[1], msg.position[2])
            else:
                self.get_logger().warn("setpoint had neither finite velocity nor position",
                                       throttle_duration_sec=5.0)
        except (SimBridgeError, OSError) as exc:
            self.get_logger().error(f"setpoint rejected: {exc}", throttle_duration_sec=2.0)

    def destroy_node(self):
        try:
            self.sim.close()
            self.media.close()
            self.ctrl.close()
        finally:
            super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = CarlaAirBridge()
    # 4 threads: odometry, images, world tick, and the setpoint subscription.
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
