#!/usr/bin/env python3
"""Turn a 2D annotation into a point in the world. The See-Point-Fly transform.

This is the load-bearing geometry of the whole testbed, and the conformance suite measures
it end to end: annotate a pixel at 64.3 m, fly 20 m along the ray, and the depth at that
pixel reads 47.1 m against 44.1 m predicted. A ~3 m residual on a 64 m ray is the accuracy
budget everything downstream inherits.

Three things here are not obvious and all three were found the hard way:

* **Use the camera pose, not the vehicle pose.** The camera is gimbal-less but its pitch is
  real, and a yaw-only rotation silently puts every waypoint on the horizon.
* **`DepthPerspective` is planar z-depth, not ray length.** The ray is built as
  (depth, right, down) scaled by normalised pixel offsets, so |ray| > depth. Treating depth
  as a range shortens every waypoint by a factor of sec(angle from the optical axis).
* **Sky is a legitimate answer.** A pixel with no finite depth grounds to nothing. It is
  published with `valid=false` rather than dropped, because "the model pointed at the sky"
  is a result the episode log needs, not an error to swallow.

The node re-grounds against the depth frame whose stamp matches the annotation's, so a slow
backend annotating an old frame is still grounded correctly.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from interfaces.msg import Annotation2D, GroundedWaypoint
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


def quat_to_matrix(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class GroundingNode(Node):
    def __init__(self):
        super().__init__("grounding")

        self.declare_parameter("camera_pitch_deg", -28.6)
        self.declare_parameter("camera_offset_body", [0.5, 0.0, 0.1])
        self.declare_parameter("depth_history", 12)
        self.declare_parameter("max_range_m", 200.0)
        # How far along the ray to place a waypoint we have no depth for. Short on purpose:
        # it is a step in a direction, not a claim about the world, and the controller will
        # re-observe when it gets there.
        self.declare_parameter("bearing_only_range_m", 40.0)

        self._cv = CvBridge()
        self._depths: deque = deque(maxlen=int(self.get_parameter("depth_history").value))
        self._info: CameraInfo | None = None
        self._odom: VehicleOdometry | None = None
        self._cam_pose: PoseStamped | None = None

        self.pub = self.create_publisher(GroundedWaypoint, "/control/waypoint", 5)
        self.create_subscription(Annotation2D, "/vlm/annotation", self._on_annotation, 5)
        self.create_subscription(Image, "/camera/depth/image_raw", self._on_depth, 5)
        self.create_subscription(CameraInfo, "/camera/rgb/camera_info", self._on_info, 5)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry",
                                 self._on_odom, PX4_QOS)
        # Preferred over reconstructing the pose from odometry + camera_pitch_deg: this is
        # the pose simGetCameraInfo actually reports. The parameter stays only as a fallback
        # for replaying bags recorded before the bridge published this topic.
        self.create_subscription(PoseStamped, "/camera/pose", self._on_cam_pose, 5)

    # ------------------------------------------------------------------ inputs

    def _on_info(self, msg: CameraInfo):
        self._info = msg

    def _on_cam_pose(self, msg: PoseStamped):
        self._cam_pose = msg

    def _on_odom(self, msg: VehicleOdometry):
        self._odom = msg

    def _on_depth(self, msg: Image):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._depths.append((t, self._cv.imgmsg_to_cv2(msg, desired_encoding="32FC1")))

    def _depth_nearest(self, stamp):
        """The depth frame closest in time to the annotated image."""
        if not self._depths:
            return None, None
        t = stamp.sec + stamp.nanosec * 1e-9
        best_t, best = min(self._depths, key=lambda kv: abs(kv[0] - t))
        return best, abs(best_t - t)

    # ----------------------------------------------------------------- the work

    def _on_annotation(self, ann: Annotation2D):
        out = GroundedWaypoint()
        out.header = ann.header
        out.source = ann
        out.valid = False
        out.position = Point()

        if self._info is None or self._odom is None:
            out.reason = "waiting for camera_info / odometry"
            self.pub.publish(out)
            return

        depth, dt = self._depth_nearest(ann.header.stamp)
        if depth is None:
            out.reason = "no depth frame yet"
            self.pub.publish(out)
            return

        w, h = self._info.width, self._info.height
        dh, dw = depth.shape
        # Depth is rendered smaller than RGB on purpose — matching resolutions costs 15.4 s
        # per capture. The aspect ratios are equal, so this scale is exact.
        du = min(dw - 1, int(ann.u * dw / w))
        dv = min(dh - 1, int(ann.v * dh / h))
        d = float(depth[dv, du])

        max_range = float(self.get_parameter("max_range_m").value)
        bearing_only = (not math.isfinite(d)) or d > max_range
        if bearing_only:
            # Keep the direction, drop the range. Rejecting these outright deadlocks the
            # loop: the drone stops, so the camera never turns, so the goal stays off-screen,
            # so every annotation is sky. Measured — the oracle produced 70 consecutive
            # rejections and the aircraft sat motionless for the whole episode.
            why = "sky" if not math.isfinite(d) else f"depth {d:.0f} m beyond {max_range:.0f} m"
            d = float(self.get_parameter("bearing_only_range_m").value)

        fx = self._info.k[0]
        cx, cy = self._info.k[2], self._info.k[5]
        ray_cam = np.array([d, (ann.u - cx) / fx * d, (ann.v - cy) / fx * d])

        if self._cam_pose is not None:
            # The measured camera pose. No assumption about pitch, mounting offset or how
            # the bridge composed them — all three were places the two nodes could disagree.
            cp, cq = self._cam_pose.pose.position, self._cam_pose.pose.orientation
            cam_pos = np.array([cp.x, cp.y, cp.z])
            r = quat_to_matrix(cq.w, cq.x, cq.y, cq.z)
            pose_src = "measured"
        else:
            # Fallback: reconstruct from odometry and the configured pitch. Correct only
            # while camera_pitch_deg here matches what actually got set on the camera.
            q = self._odom.q
            r_body = quat_to_matrix(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
            pitch = math.radians(float(self.get_parameter("camera_pitch_deg").value))
            r_pitch = np.array([
                [math.cos(pitch), 0.0, math.sin(pitch)],
                [0.0, 1.0, 0.0],
                [-math.sin(pitch), 0.0, math.cos(pitch)],
            ])
            r = r_body @ r_pitch
            body_offset = np.array(
                list(self.get_parameter("camera_offset_body").value), dtype=float)
            cam_pos = np.array([float(c) for c in self._odom.position]) + r_body @ body_offset
            pose_src = "ESTIMATED from odometry + camera_pitch_deg"

        point = cam_pos + r @ ray_cam

        out.position = Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
        out.bearing_only = bearing_only
        out.depth = 0.0 if bearing_only else float(d)
        out.range = float(np.linalg.norm(point - cam_pos))
        out.valid = True
        out.reason = (f"depth frame {dt * 1000:.0f} ms from the annotation; "
                      f"camera pose {pose_src}"
                      + (f"; BEARING ONLY ({why}), stepping {d:.0f} m along the ray"
                         if bearing_only else ""))
        self.pub.publish(out)


def main(argv=None):
    rclpy.init(args=argv)
    node = GroundingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
