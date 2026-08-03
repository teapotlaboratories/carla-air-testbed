"""An HD chase camera on the world, following the aircraft — the view from outside.

This is **not** the drone's sensor. `Camera` in this package captures what the model is shown
and what it is scored on, and that stays at 640x480 because it is a measurement surface: its
resolution changes the model's token cost, its aspect ratio has to match the depth buffer, and
its field of view sets where level flight lands in the frame. Anything that alters it alters
the experiment.

This is the opposite — a spectator, cinematic, changeable at will, scored on nothing. It shows
the aircraft *in* the city rather than the city as the aircraft sees it, which is the view you
want when explaining a flight to someone who was not watching it.

**It is a CARLA sensor, not an AirSim capture.** That is the whole reason it is affordable.
AirSim's image path is RPC request/response and is already this project's bottleneck — a
1080p grab on it would contend directly with the frames the model needs. A CARLA
`sensor.camera.rgb` renders inside the same UE4 process and pushes frames out asynchronously
on its own tick, so it never queues behind a `simGetImages` call. Measured free-floating at
1920x1080: 9.7 Hz, and `set_transform` while streaming is free.

The sensor is spawned unattached — CARLA has no actor for an AirSim vehicle to attach to — so
following is done by writing a transform each tick from the aircraft's NED pose.
"""
from __future__ import annotations

import math
import queue
import threading

import numpy as np

import carla

from .frames import ned_to_carla
from .h264 import VideoWriter


class ChaseCamera:
    """A free-floating CARLA camera that trails the aircraft and records to MP4.

    Frames never touch the caller's thread and never block CARLA's. The sensor callback does
    one reshape and a non-blocking queue put; a writer thread owns the encoder. If encoding
    falls behind, frames are **dropped rather than buffered** — a recorder that grows without
    bound would eventually take the simulator down with it, and a dropped frame costs a
    fraction of a second of video.
    """

    def __init__(self, world, width=1920, height=1080, fov=90.0, fps=10.0,
                 distance=14.0, above=6.0, smoothing=0.25, queue_size=8):
        self._world = world
        self.width, self.height, self.fps = int(width), int(height), float(fps)
        self.distance, self.above = float(distance), float(above)
        # 0 = snap to the target pose, 1 = never move. A little lag reads as a camera
        # operator following the aircraft instead of being welded to it.
        self.smoothing = min(0.95, max(0.0, float(smoothing)))

        blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        blueprint.set_attribute("image_size_x", str(self.width))
        blueprint.set_attribute("image_size_y", str(self.height))
        blueprint.set_attribute("fov", str(float(fov)))
        # Without sensor_tick the camera renders every simulation tick, which is far more
        # frames than a video needs and pure cost.
        blueprint.set_attribute("sensor_tick", f"{1.0 / self.fps:.4f}")

        self._sensor = world.spawn_actor(blueprint, carla.Transform())
        self._sensor.listen(self._on_image)

        self._queue: queue.Queue = queue.Queue(maxsize=int(queue_size))
        self._writer = None
        self._thread = None
        self._lock = threading.Lock()
        self._pose = None            # smoothed (x, y, z, yaw_deg, pitch_deg) in CARLA
        self.frames_written = 0
        self.frames_dropped = 0
        self.latest = None           # newest BGR frame, for live viewing

    # ------------------------------------------------------------------ following

    def follow(self, ned_xyz, yaw_rad):
        """Place the camera behind and above the aircraft, looking down at it.

        `ned_xyz` and `yaw_rad` are the aircraft's own pose. NED x is north and y is east,
        and after `ned_to_carla` those map onto CARLA's x and y directly, so the CARLA yaw is
        just the aircraft's yaw in degrees.
        """
        n, e, d = float(ned_xyz[0]), float(ned_xyz[1]), float(ned_xyz[2])
        # Behind along the heading, and above — NED z is DOWN, so "up" subtracts.
        cam_ned = (n - self.distance * math.cos(yaw_rad),
                   e - self.distance * math.sin(yaw_rad),
                   d - self.above)
        x, y, z = ned_to_carla(*cam_ned)
        yaw_deg = math.degrees(yaw_rad)
        # Tilt down by however much the offset raised us, so the aircraft stays centred.
        pitch_deg = -math.degrees(math.atan2(self.above, max(1e-3, self.distance)))

        if self._pose is None:
            self._pose = (x, y, z, yaw_deg, pitch_deg)
        else:
            k = self.smoothing
            px, py, pz, pyaw, ppitch = self._pose
            # Interpolate yaw the short way round, or the camera whips through 360 degrees
            # every time the aircraft crosses north.
            dyaw = (yaw_deg - pyaw + 180.0) % 360.0 - 180.0
            self._pose = (px + (x - px) * (1 - k), py + (y - py) * (1 - k),
                          pz + (z - pz) * (1 - k), pyaw + dyaw * (1 - k),
                          ppitch + (pitch_deg - ppitch) * (1 - k))

        cx, cy, cz, cyaw, cpitch = self._pose
        self._sensor.set_transform(carla.Transform(
            carla.Location(x=cx, y=cy, z=cz),
            carla.Rotation(pitch=cpitch, yaw=cyaw, roll=0.0)))

    # ------------------------------------------------------------------ recording

    def start(self, path, crf=26):
        with self._lock:
            if self._writer is not None:
                self.stop()
            writer = VideoWriter(path, self.width, self.height, fps=self.fps, crf=crf)
            self._writer = writer
            self.frames_written = self.frames_dropped = 0
            self._thread = threading.Thread(target=self._drain, daemon=True)
            self._thread.start()
        return path

    def stop(self):
        with self._lock:
            writer, self._writer = self._writer, None
        if writer is None:
            return {"frames": 0, "dropped": 0}
        self._queue.put(None)                      # sentinel: drain and exit
        if self._thread is not None:
            self._thread.join(timeout=10.0)
            self._thread = None
        codec = getattr(writer, "codec", "?")
        writer.close()
        return {"frames": self.frames_written, "dropped": self.frames_dropped,
                "seconds": round(self.frames_written / self.fps, 1), "codec": codec}

    def _on_image(self, image):
        """CARLA's sensor thread. Do as little as possible here."""
        try:
            # CARLA hands over BGRA; the encoder wants BGR. The copy is required — raw_data
            # is a view onto a buffer CARLA reuses for the next frame.
            frame = np.frombuffer(image.raw_data, dtype=np.uint8)
            frame = frame.reshape((image.height, image.width, 4))[:, :, :3].copy()
        except Exception:  # noqa: BLE001 — a recorder must never break the simulation
            self.frames_dropped += 1
            return

        # Always publish the newest frame, whether or not anything is recording: the web
        # console streams from here, and it must not have to start a file to see a picture.
        # A plain assignment is enough — readers want the latest frame, never a history, and
        # a lock would put a reader on CARLA's sensor thread.
        self.latest = frame

        if self._writer is None:
            return
        try:
            self._queue.put_nowait(frame)
        except queue.Full:
            self.frames_dropped += 1

    def latest_jpeg(self, quality=80):
        """The most recent frame as JPEG bytes, or None if nothing has arrived yet.

        Encoding happens on the caller's thread on purpose — a slow consumer then slows only
        itself, never the sensor callback or the recorder.
        """
        import cv2

        frame = self.latest
        if frame is None:
            return None
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
        return buf.tobytes() if ok else None

    def _drain(self):
        while True:
            frame = self._queue.get()
            if frame is None:
                break
            writer = self._writer
            if writer is None:
                break
            try:
                writer.write(frame)
                self.frames_written += 1
            except Exception:  # noqa: BLE001
                self.frames_dropped += 1

    # ------------------------------------------------------------------ teardown

    def destroy(self):
        try:
            self.stop()
        finally:
            try:
                self._sensor.stop()
            finally:
                self._sensor.destroy()
