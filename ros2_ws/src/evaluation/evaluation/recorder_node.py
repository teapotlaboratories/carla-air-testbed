"""Record every episode to MP4, with the model's own annotation drawn on the frame.

An episode used to leave a JSON and nothing to look at. A failure was a number — `141.9 m
from goal` — and working out *why* meant reading the offboard node's target log and
reconstructing the geometry in your head. The first real VLM flight made the cost of that
obvious: the aircraft descended 40 m onto the controller's altitude floor and circled there
for twenty steps, and the reason (the model was aiming at ground features, which ground to
points on the ground) took a log dig to see. On video it would have been the first thing
anyone noticed.

**This subscribes rather than captures.** The frames come off `/camera/rgb/image_raw`, which
the bridge already publishes — so recording adds no load to the simulator, cannot contend
with the bridge's own capture, and shows exactly the pixels the model was given. A recorder
that pulled its own frames from AirSim would be a fourth client on a transport that is
already the bottleneck, and it would be recording a *different* view from the one being
scored.

Nothing here may break a flight. Every callback is wrapped: a recorder that throws must cost
you a video, never an episode.
"""
from __future__ import annotations

import os
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from interfaces.msg import Annotation2D, EpisodeStatus
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

# Shared with the 3.10 side, exactly like frames.py: pure encoding, no ROS and no carla.
#
# Found by walking UP from this file rather than by counting dirname() levels. Under
# `colcon --symlink-install` this module is imported from ros2_ws/build/..., not from src/,
# so a fixed count calibrated on the source tree resolves to the wrong directory and the node
# dies at import with "No module named 'h264'" — which reads like a missing dependency.
def _find_carla_air():
    override = os.environ.get("TESTBED_CARLA_AIR")
    if override:
        return override
    here = os.path.abspath(__file__)
    for _ in range(10):
        here = os.path.dirname(here)
        candidate = os.path.join(here, "sim_bridge", "carla_air")
        if os.path.isdir(candidate):
            return candidate
    return None


_SIM_BRIDGE = _find_carla_air()
if _SIM_BRIDGE and _SIM_BRIDGE not in sys.path:
    sys.path.insert(0, _SIM_BRIDGE)
from h264 import VideoWriter  # noqa: E402

# The bridge publishes /fmu/out/* best-effort; a RELIABLE subscription silently receives
# nothing from a BEST_EFFORT publisher, which would leave the HUD's altitude field empty
# with no error anywhere.
_SENSOR_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)

_TERMINAL = {"success", "failure", "aborted"}

# The overlay's font and line sizes were chosen against a 480-line frame; every HUD
# dimension is scaled from this so the layout holds at any capture or output size.
_HUD_REFERENCE_HEIGHT = 480.0


class RecorderNode(Node):
    def __init__(self):
        super().__init__("recorder")
        self.declare_parameter("record_dir", "out/videos")
        self.declare_parameter("fps", 8.0)          # matches the measured RGB rate
        self.declare_parameter("draw_overlay", True)
        # A recorder is not worth a flight. If encoding is broken, say so once and fly on.
        self.declare_parameter("record_enabled", True)
        # Output height in lines, or 0 to write whatever the camera publishes. The camera
        # now captures 1440x1080, so the default is a no-op resize and the video carries
        # real detail rather than an upscale. Lower it to shrink the files.
        self.declare_parameter("output_height", 1080)
        # Pillarbox to this width for a 16:9 container (1920 gives true 1080p). 0 keeps the
        # camera's own 4:3, which at 1080 lines is 1440x1080 — 1080p without black bars.
        self.declare_parameter("pad_to_width", 0)
        # x264 constant-rate-factor. 23 is near-transparent, 26 is 2.1x smaller and still
        # good, 28 is 2.9x. Measured on real 1080p footage; see docs/todo.md T-02.
        self.declare_parameter("crf", 26)

        self._dir = self.get_parameter("record_dir").value
        if not os.path.isabs(self._dir):
            self._dir = os.path.join(os.getcwd(), self._dir)
        self._fps = float(self.get_parameter("fps").value)
        self._crf = int(self.get_parameter("crf").value)
        self._overlay = bool(self.get_parameter("draw_overlay").value)
        self._enabled = bool(self.get_parameter("record_enabled").value)
        self._out_h = int(self.get_parameter("output_height").value)
        self._pad_w = int(self.get_parameter("pad_to_width").value)

        self._cv = CvBridge()
        self._writer = None
        self._path = None
        self._frames = 0
        self._broken = False
        self._scale_factor = 1.0
        self._hud_scale = 1.0
        self._pad_left = 0

        self._episode = None        # EpisodeStatus, latest
        self.declare_parameter("record_depth", False)
        self._record_depth = bool(self.get_parameter("record_depth").value)
        self.declare_parameter("depth_max_m", 200.0)
        #: Clip depth here before colourising. AirSim writes sky as a huge
        #: value, so without a clip the whole city lands in the bottom few
        #: percent of the range and the frame reads as black.
        self._depth_max = float(self.get_parameter("depth_max_m").value)
        self._depth_writer = None
        self._depth_t0 = None
        self._annotation = None     # Annotation2D, latest
        self._t0 = None             # stamp of the first frame of this episode
        self._altitude = None

        self.create_subscription(Image, "/camera/rgb/image_raw", self._on_image, 1)
        self.create_subscription(Annotation2D, "/vlm/annotation", self._on_annotation, 5)
        if self._record_depth:
            self.create_subscription(Image, "/camera/depth/image_raw",
                                     self._on_depth, 5)
        self.create_subscription(EpisodeStatus, "/episode/status", self._on_status, 5)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry",
                                 self._on_odom, _SENSOR_QOS)

        if self._enabled:
            os.makedirs(self._dir, exist_ok=True)
            self.get_logger().info(f"recording episodes to {self._dir}")
        else:
            self.get_logger().info("recording disabled (record_enabled:=false)")

    # ------------------------------------------------------------------ inputs

    def _on_depth(self, msg: Image):
        """Record the depth buffer as a colourised video alongside the RGB one.

        Depth is 32FC1 METRES, and AirSim marks sky as a very large value rather than NaN -
        so a naive normalise maps the whole city into the bottom few percent of the range
        and the result is a black frame with a white sky. Clipping at `max_range_m` first is
        what makes it readable.

        This is what `grounding` actually consumes: the model picks a pixel, and the depth at
        that pixel is what turns it into a distance. A flight that went somewhere odd is much
        easier to explain with this beside the RGB.
        """
        if not self._record_depth or self._broken:
            return
        episode = self._episode
        if episode is None or episode.state != "running":
            return
        try:
            d = self._cv.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            d = np.nan_to_num(np.asarray(d, dtype=np.float32), nan=self._depth_max,
                              posinf=self._depth_max)
            norm = np.clip(d, 0.0, self._depth_max) / self._depth_max
            colour = cv2.applyColorMap((255 * (1.0 - norm)).astype(np.uint8), cv2.COLORMAP_TURBO)
            if self._depth_writer is None:
                path = os.path.join(self._dir, f"{episode.episode_id}-depth.mp4")
                self._depth_writer = VideoWriter(path, colour.shape[1], colour.shape[0],
                                                 fps=self._fps, crf=self._crf)
                self.get_logger().info(f"recording depth -> {path}")
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self._depth_t0 is None:
                self._depth_t0 = stamp
            self._depth_writer.write(colour, t_s=max(0.0, stamp - self._depth_t0))
        except Exception as exc:                                       # noqa: BLE001
            self.get_logger().warn(f"depth recording stopped: {exc}")
            self._depth_writer = None
            self._record_depth = False

    def _on_annotation(self, msg: Annotation2D):
        self._annotation = msg

    def _on_odom(self, msg: VehicleOdometry):
        self._altitude = -float(msg.position[2])

    def _on_status(self, msg: EpisodeStatus):
        previous = self._episode
        self._episode = msg
        # Close on the terminal transition, not on the next episode's first frame — otherwise
        # the last few seconds of a run (the part that explains the failure) land in the
        # following episode's file, or in no file at all.
        if msg.state in _TERMINAL:
            self._close()
        elif previous is not None and previous.episode_id != msg.episode_id:
            self._close()

    def _on_image(self, msg: Image):
        if not self._enabled or self._broken:
            return
        try:
            frame = self._cv.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            episode = self._episode
            if episode is None or episode.state != "running":
                return
            canvas = self._scale(frame)
            if self._writer is None:
                self._open(episode, canvas.shape)
            if self._writer is None:
                return
            # Time each frame by the IMAGE's own stamp, not by a nominal rate. The camera
            # publishes at whatever the capture path can manage - 7.08 Hz at 640x480, 5.58
            # at 960x720 - while the writer's `fps` is a fixed 8.0. Writing at the nominal
            # rate made the file play at 8.0/5.58 = 1.43x real speed, so a 66 s episode
            # became a 46 s video that drifted steadily out of step with the chase camera.
            #
            # The message stamp, not `now()`: it is when the FRAME was taken, and this
            # callback can run well after that.
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if self._t0 is None:
                self._t0 = stamp
            # Scale first, decorate second: drawing on the source and then resizing would
            # blur the HUD along with the image.
            self._writer.write(self._decorate(canvas, episode) if self._overlay else canvas,
                               t_s=max(0.0, stamp - self._t0))
            self._frames += 1
        except Exception as exc:  # noqa: BLE001 — never take an episode down
            self._broken = True
            self.get_logger().error(f"recording stopped after an error: {exc}")

    # ------------------------------------------------------------------ scaling

    def _scale(self, frame):
        """Source frame -> output canvas, preserving the camera's aspect ratio.

        INTER_CUBIC rather than the default INTER_LINEAR, for the case where the output is
        larger than the capture. Upscaling buys sharpness, never detail — no interpolation
        invents what the source did not record.

        `self._scale_factor` is recorded so the annotation crosshair lands on the same
        feature it did at 480p.
        """
        h, w = frame.shape[:2]
        if self._out_h <= 0 or self._out_h == h:
            self._scale_factor = 1.0
            out = frame
        else:
            self._scale_factor = self._out_h / h
            out = cv2.resize(frame, (round(w * self._scale_factor), self._out_h),
                             interpolation=cv2.INTER_CUBIC)
        # HUD size follows the OUTPUT height, not the upscale factor. These are different
        # numbers the moment the camera itself captures at 1080 — the upscale is then 1.0
        # while the canvas is still 2.25x the height the overlay was designed against, and
        # tying text to the upscale would render 480p-sized captions on a 1080p frame.
        self._hud_scale = max(0.7, out.shape[0] / _HUD_REFERENCE_HEIGHT)
        if self._pad_w > out.shape[1]:
            # Pillarbox, never letterbox: padding the sides keeps every source line, and the
            # HUD banners still sit against the frame edges where they are readable.
            pad = self._pad_w - out.shape[1]
            left = pad // 2
            out = cv2.copyMakeBorder(out, 0, 0, left, pad - left,
                                     cv2.BORDER_CONSTANT, value=(0, 0, 0))
            self._pad_left = left
        else:
            self._pad_left = 0
        return out

    # ------------------------------------------------------------------ writing

    def _open(self, episode: EpisodeStatus, shape):
        self._t0 = None
        height, width = shape[:2]
        name = episode.episode_id or "episode"
        self._path = os.path.join(self._dir, f"{name}.mp4")
        # H.264 via PyAV. OpenCV's bundled FFmpeg has no libx264 (GPL vs the wheel's
        # licence), so cv2.VideoWriter can only emit mp4v — which no browser will play.
        try:
            writer = VideoWriter(self._path, width, height, fps=self._fps,
                                 crf=int(self.get_parameter("crf").value))
        except Exception as exc:  # noqa: BLE001
            self._broken = True
            self.get_logger().error(f"could not open {self._path} for writing: {exc}")
            return
        self._writer = writer
        self._frames = 0
        note = "" if self._scale_factor == 1.0 else f" (upscaled x{self._scale_factor:.2f})"
        self.get_logger().info(f"recording {name} -> {self._path} @ {width}x{height}{note}")

    def _close(self):
        self._close_depth()
        if self._writer is None:
            return
        codec = getattr(self._writer, "codec", "?")
        self._writer.close()
        seconds = self._frames / self._fps if self._fps else 0.0
        self.get_logger().info(
            f"wrote {self._path} ({self._frames} frames, {seconds:.1f}s, {codec})")
        self._writer = None
        self._path = None

    # ------------------------------------------------------------------ overlay

    def _decorate(self, frame, episode: EpisodeStatus):
        """Draw what the model was told, what it answered, and where that put the aircraft.

        The altitude line is deliberately prominent: the first real VLM flight failed by
        descending, and a HUD that does not show height would have hidden it just as well as
        the JSON did.
        """
        height, width = frame.shape[:2]
        ann = self._annotation
        s = self._hud_scale

        if ann is not None:
            # The annotation is in SOURCE pixels; map it onto the canvas or the crosshair
            # drifts off the feature the model was actually pointing at.
            u = int(round(ann.u * self._scale_factor)) + self._pad_left
            v = int(round(ann.v * self._scale_factor))
            # Sized to survive being scaled down. This view is routinely inset into a
            # 1080p chase frame at a third of its width, and at the old 14 px arms the
            # marker - the one thing the whole HUD exists to show - vanished.
            arm, gap = max(14, int(34 * s)), max(4, int(9 * s))
            thick = max(2, int(round(4 * s)))
            # Black underlay first: yellow on a sunlit road is invisible, and this frame is
            # the evidence for what the model was pointing at.
            for colour, extra in (((0, 0, 0), 2), ((0, 255, 255), 0)):
                t = thick + extra
                cv2.line(frame, (u - arm, v), (u - gap, v), colour, t)
                cv2.line(frame, (u + gap, v), (u + arm, v), colour, t)
                cv2.line(frame, (u, v - arm), (u, v - gap), colour, t)
                cv2.line(frame, (u, v + gap), (u, v + arm), colour, t)
                cv2.circle(frame, (u, v), max(16, int(30 * s)), colour, max(1, t // 2))

        lines = [
            f"{episode.episode_id}   step {episode.step}",
            f"dist {episode.distance_to_goal:6.1f} m"
            + (f"   ALT {self._altitude:5.1f} m" if self._altitude is not None else ""),
        ]
        if ann is not None:
            lines.append(f"{ann.backend}  ({ann.u},{ann.v})  conf {ann.confidence:.2f}"
                         f"  {ann.latency_s:.1f}s")
        if episode.collided:
            lines.append("COLLIDED")

        self._banner(frame, lines, s, top=True)

        if ann is not None and ann.rationale:
            self._banner(frame, self._wrap(ann.rationale, width, s), s, top=False)
        return frame

    @staticmethod
    def _wrap(text, width_px, s, max_lines=3):
        """Wrap to the frame width by MEASURING the rendered text, not estimating it.

        An estimate of characters-per-line has to assume an average glyph width, and that
        assumption only holds at the font scale it was tuned for — at 2.25x it overflowed the
        frame and the rationale ran off the right edge. `getTextSize` asks the renderer that
        will actually draw it, so the wrap is correct at any scale.
        """
        budget = width_px - int(24 * s)
        font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.45 * s, max(1, int(round(s)))
        lines, current = [], ""
        for word in text.split():
            candidate = f"{current} {word}".strip()
            if cv2.getTextSize(candidate, font, scale, thick)[0][0] <= budget or not current:
                current = candidate
            else:
                lines.append(current)
                current = word
                if len(lines) == max_lines:
                    return lines
        if current:
            lines.append(current)
        return lines[:max_lines]

    @staticmethod
    def _banner(frame, lines, s, top):
        if not lines:
            return
        pad, line_h = int(6 * s), int(18 * s)
        scale, thick = 0.45 * s, max(1, int(round(s)))
        box = line_h * len(lines) + pad
        height = frame.shape[0]
        y0 = 0 if top else height - box
        # Blend rather than fill: the frame under the text is the evidence, and a solid bar
        # would hide whatever the model was looking at near the edge.
        panel = frame[y0:y0 + box].copy()
        panel[:] = (0, 0, 0)
        cv2.addWeighted(panel, 0.55, frame[y0:y0 + box], 0.45, 0, frame[y0:y0 + box])
        for i, text in enumerate(lines):
            cv2.putText(frame, text, (int(8 * s), y0 + pad + line_h * i + line_h - int(6 * s)),
                        cv2.FONT_HERSHEY_SIMPLEX, scale, (235, 235, 235), thick, cv2.LINE_AA)

    def _close_depth(self):
        if self._depth_writer is not None:
            self._depth_writer.close()
            self._depth_writer = None
            self._depth_t0 = None

    def destroy_node(self):
        # A killed graph is the normal end of a sweep. Without this the final episode's file
        # has no moov atom and will not play — the run you most want to watch.
        self._close()
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
