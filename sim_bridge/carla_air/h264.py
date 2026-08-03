"""H.264 video writing, shared by both recorders and therefore by both interpreters.

Like `frames.py`, this file is imported from the 3.10 sidecar **and** the 3.12 ROS side. It
depends only on `av` and numpy — nothing from carla, airsim or rclpy — so it can be.

**Why this exists at all.** `cv2.VideoWriter` on this box can only produce `mp4v`
(MPEG-4 Part 2). Not a missing feature: `opencv-python` bundles its own FFmpeg built without
`libx264`, because x264 is GPL and the wheel ships under Apache/MIT. The system does have
`libx264.so.164`; OpenCV just does not link it. PyAV bundles an FFmpeg that does.

That matters for two reasons, and the second is the bigger one:

* **Size.** Measured on 200 real 1920x1080 frames: `mp4v` 11.6 MB, x264 crf26 5.5 MB — 2.1x,
  or a 64 MB episode becoming 30 MB.
* **`mp4v` does not play in a browser.** H.264 does. A recording that a browser can play is
  one the web console can show; an mp4v file has to be downloaded and opened in something
  else first.

CRF is the only lever worth touching. `preset slow` beat `medium` by 2% for 40% of the encode
speed, and `h264_nvenc` was no smaller than x264 while adding load to the GPU that renders the
simulator. So: CPU x264, `medium`, and encoding runs at ~78 fps against a 10 fps capture.
"""
from __future__ import annotations

from fractions import Fraction

import numpy as np

try:
    import av
    HAVE_AV = True
except ImportError:                      # pragma: no cover - environment, not logic
    HAVE_AV = False


class VideoWriter:
    """H.264 when PyAV is available, `mp4v` when it is not.

    The fallback is deliberate. A fresh clone that has not run `scripts/fetch_vendor.sh` yet
    should still record something watchable rather than crash mid-flight — losing the codec
    is worth far less than losing the run.
    """

    def __init__(self, path, width, height, fps=10.0, crf=26, preset="medium"):
        self.path, self.width, self.height = path, int(width), int(height)
        self.fps = float(fps)
        self.frames = 0
        self._container = None
        self._stream = None
        self._cv = None

        if HAVE_AV:
            self._container = av.open(path, "w")
            # PyAV wants a rational frame rate; a float raises deep inside av.utils with
            # "'float' object has no attribute 'numerator'", which names nothing useful.
            # Fraction also keeps a non-integer rate like 7.5 exact.
            rate = Fraction(self.fps).limit_denominator(1000)
            stream = self._container.add_stream("libx264", rate=rate)
            stream.width, stream.height = self.width, self.height
            # yuv420p rather than a higher chroma format: it is the only pixel format every
            # browser and player decodes. yuv444p encodes fine here and then fails to play
            # in exactly the place this was meant to be viewable.
            stream.pix_fmt = "yuv420p"
            stream.options = {"crf": str(int(crf)), "preset": str(preset)}
            self._stream = stream
            self.codec = "h264"
        else:
            import cv2
            self._cv = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"),
                                       self.fps, (self.width, self.height))
            if not self._cv.isOpened():
                raise RuntimeError(f"could not open {path} for writing")
            self.codec = "mp4v"

    def write(self, frame_bgr):
        if self._cv is not None:
            self._cv.write(frame_bgr)
            self.frames += 1
            return
        # PyAV wants a contiguous array; a slice of a larger buffer (which is what the BGRA
        # -> BGR crop produces) is not, and raises rather than silently misreading.
        if not frame_bgr.flags["C_CONTIGUOUS"]:
            frame_bgr = np.ascontiguousarray(frame_bgr)
        packets = self._stream.encode(av.VideoFrame.from_ndarray(frame_bgr, format="bgr24"))
        for packet in packets:
            self._container.mux(packet)
        self.frames += 1

    def close(self):
        """Flush and finalise. Safe to call twice — a killed sweep may reach it twice."""
        if self._cv is not None:
            self._cv.release()
            self._cv = None
            return self.frames
        if self._container is None:
            return self.frames
        try:
            # Without this the encoder's internal buffer is discarded and the trailing frames
            # never reach the file.
            for packet in self._stream.encode():
                self._container.mux(packet)
        finally:
            self._container.close()
            self._container = None
            self._stream = None
        return self.frames

    @property
    def opened(self):
        return self._container is not None or self._cv is not None
