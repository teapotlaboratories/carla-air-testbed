#!/usr/bin/env python3
"""Run a VLM backend against the camera stream and publish 2D annotations.

The node exists to keep the model *off* the flight path. It subscribes to images, calls a
backend, and publishes `interfaces/Annotation2D` — nothing else. It never computes a
velocity, never sees a pose, and never blocks the controller: inference runs on its own
callback group so a backend that takes four seconds slows the annotation rate and nothing
else. That is the whole reason the loop tolerates a slow model.

Backpressure is deliberate. Frames that arrive while inference is running are **dropped,
not queued** — a VLM answering a four-second-old frame is worse than one answering the
current frame late, and a queue guarantees the former.
"""
from __future__ import annotations

import threading
import time

import rclpy
from cv_bridge import CvBridge
from interfaces.msg import Annotation2D
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .backends.base import Annotation
from .backends.claude import ClaudeBackend
from .backends.mock import GeometricBackend, MockBackend, ScriptedBackend
from .backends.oracle import OracleBackend

BACKENDS = {
    "mock": MockBackend,
    "scripted": ScriptedBackend,
    "geometric": GeometricBackend,
    # oracle is a DIAGNOSTIC, not a competitor: it is handed the goal through a side
    # channel and so cannot be compared with backends that only see the image. Use it to
    # decide whether a scenario is navigable at all. See backends/oracle.py.
    "oracle": OracleBackend,
    # The first real model. Sees one frame and one instruction, exactly like the baselines
    # above, which is what makes its score comparable with theirs.
    "claude": ClaudeBackend,
    # Real backends register here. Each must implement backends.base.VlmBackend and must
    # not need anything beyond (image, instruction) — see that module for why.
}


class VlmNode(Node):
    def __init__(self):
        super().__init__("vlm_client")

        self.declare_parameter("backend", "geometric")
        self.declare_parameter("seed", 0)
        self.declare_parameter("instruction", "fly forward and stay clear of buildings")
        self.declare_parameter("min_period_s", 1.0)
        self.declare_parameter("history_length", 5)
        self.declare_parameter("scripted_pixels", [0.5, 0.65])
        # Only the oracle needs this; it comes from simGetCameraInfo and is stable per run.
        self.declare_parameter("camera_hfov_deg", 89.9)
        # Claude backend. The API key is NOT here on purpose — it comes from the
        # ANTHROPIC_API_KEY environment variable, because parameters are readable from the
        # graph and get dumped into launch logs. See backends/claude.py.
        self.declare_parameter("claude_model", "claude-opus-5")
        # `low` because this is a control loop: 40 steps in 300 s is 7.5 s per decision, and
        # a higher effort can spend that on one call. Raise it to measure quality, not rate.
        self.declare_parameter("claude_effort", "low")
        self.declare_parameter("claude_max_tokens", 16000)
        self.declare_parameter("claude_timeout_s", 60.0)
        self.declare_parameter("claude_jpeg_quality", 90)
        self.declare_parameter("claude_fallbacks", True)

        name = self.get_parameter("backend").value
        if name not in BACKENDS:
            raise RuntimeError(f"unknown backend {name!r}; have {sorted(BACKENDS)}")
        self.backend = self._build(name)
        self.get_logger().info(f"VLM backend: {self.backend.describe()}")

        self._cv = CvBridge()
        self._instruction = self.get_parameter("instruction").value
        self._min_period = float(self.get_parameter("min_period_s").value)
        self._history: list[Annotation] = []
        self._busy = threading.Lock()
        self._last_call = 0.0
        self._latest_depth = None
        self._cam_pose = None       # from /camera/pose — the bridge is the source of truth
        self._goal = None           # from /episode/goal
        self.dropped = 0

        infer_group = MutuallyExclusiveCallbackGroup()
        self.pub = self.create_publisher(Annotation2D, "/vlm/annotation", 5)
        self.create_subscription(Image, "/camera/rgb/image_raw", self._on_image, 1,
                                 callback_group=infer_group)
        self.create_subscription(Image, "/camera/depth/image_raw", self._on_depth, 1)
        self.create_subscription(String, "/vlm/instruction", self._on_instruction, 5)

        # Oracle inputs. Subscribed unconditionally so switching backend needs no relaunch.
        self.create_subscription(PoseStamped, "/camera/pose", self._on_cam_pose, 5)
        goal_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                              history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(PointStamped, "/episode/goal", self._on_goal, goal_qos)

    def _build(self, name):
        if name == "mock":
            return MockBackend(seed=int(self.get_parameter("seed").value))
        if name == "scripted":
            flat = list(self.get_parameter("scripted_pixels").value)
            pairs = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
            return ScriptedBackend(pairs, loop=True)
        if name == "claude":
            return ClaudeBackend(
                model=self.get_parameter("claude_model").value,
                effort=self.get_parameter("claude_effort").value,
                max_tokens=int(self.get_parameter("claude_max_tokens").value),
                jpeg_quality=int(self.get_parameter("claude_jpeg_quality").value),
                timeout_s=float(self.get_parameter("claude_timeout_s").value),
                fallbacks=bool(self.get_parameter("claude_fallbacks").value),
            )
        return BACKENDS[name]()

    # ------------------------------------------------------------------ inputs

    def _on_instruction(self, msg: String):
        self._instruction = msg.data
        self._history.clear()
        self.get_logger().info(f"instruction: {msg.data!r}")

    def _on_cam_pose(self, msg: PoseStamped):
        q = msg.pose.orientation
        p = msg.pose.position
        self._cam_pose = ((p.x, p.y, p.z), (q.w, q.x, q.y, q.z))

    def _on_goal(self, msg: PointStamped):
        self._goal = (msg.point.x, msg.point.y, msg.point.z)
        self.get_logger().info(f"episode goal: {[round(c, 1) for c in self._goal]}")

    def _on_depth(self, msg: Image):
        self._latest_depth = self._cv.imgmsg_to_cv2(msg, desired_encoding="32FC1")

    def _on_image(self, msg: Image):
        now = time.time()
        if now - self._last_call < self._min_period:
            return
        # Drop rather than queue: see the module docstring.
        if not self._busy.acquire(blocking=False):
            self.dropped += 1
            return
        try:
            self._last_call = now
            image = self._cv.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if isinstance(self.backend, GeometricBackend):
                self.backend.set_depth(self._latest_depth)
            elif isinstance(self.backend, OracleBackend):
                if self._goal is None or self._cam_pose is None:
                    self.get_logger().warn(
                        "oracle has no goal or camera pose yet — start an episode",
                        throttle_duration_sec=10.0)
                    return
                cam_pos, cam_quat = self._cam_pose
                self.backend.set_context(self._goal, cam_pos, cam_quat,
                                         float(self.get_parameter("camera_hfov_deg").value))

            t0 = time.perf_counter()
            ann = self.backend.annotate(image, self._instruction, self._history[-int(
                self.get_parameter("history_length").value):])
            latency = time.perf_counter() - t0

            if ann is None:
                self.get_logger().info("backend declined to answer this frame")
                return
            self._history.append(ann)
            self.pub.publish(self._to_msg(ann, msg, latency))
        except Exception as exc:  # noqa: BLE001 — a bad backend must not kill the node
            self.get_logger().error(f"backend raised: {exc}")
        finally:
            self._busy.release()

    def _to_msg(self, ann: Annotation, image_msg: Image, latency: float) -> Annotation2D:
        m = Annotation2D()
        # Stamp of the IMAGE, not of now — downstream needs to know which frame this
        # annotation belongs to in order to ground it against the matching depth.
        m.header = image_msg.header
        m.u, m.v = int(ann.u), int(ann.v)
        m.confidence = float(ann.confidence)
        m.instruction = self._instruction
        m.rationale = ann.rationale
        m.terminal = bool(ann.terminal)
        m.backend = self.backend.name
        m.latency_s = float(latency)
        return m


def main(argv=None):
    rclpy.init(args=argv)
    node = VlmNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Log the backend's own tally on the way out. For a paid backend this is where the
        # call count, token spend and decision latency actually surface — without it a
        # sweep's cost is invisible until the invoice.
        node.get_logger().info(f"backend on shutdown: {node.backend.describe()}")
        node.backend.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
