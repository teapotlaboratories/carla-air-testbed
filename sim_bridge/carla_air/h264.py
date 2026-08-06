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

#: Set once the fallback has been announced, so a 20 Hz recorder does not print per frame.
_WARNED_FALLBACK = False


def _warn_fallback(path):
    """Say out loud that this recording is degraded.

    The fallback exists so a fresh clone still records something, and that is worth keeping.
    What is not worth keeping is doing it in silence: mp4v carries no timestamps, so the file
    plays at its nominal rate instead of real time, and no browser will play it.

    That happened for two days without anyone noticing. The recorder moved out of `bringup.sh`
    into `examples/navigation/` on 2026-08-04 and the example did not export
    `vendor/py312`, so `import av` failed on the ROS side and every episode recording quietly
    became mp4v at the wrong length. A silent fallback on a measurement path is a bug in the
    fallback, not in the environment.
    """
    global _WARNED_FALLBACK
    if _WARNED_FALLBACK:
        return
    _WARNED_FALLBACK = True
    import sys
    print(f"WARNING: PyAV not importable — recording {path} as mp4v.\n"
          f"         The file will play at its NOMINAL rate, not real time, and browsers "
          f"cannot play it.\n"
          f"         Add vendor/py312 to PYTHONPATH (ROS side) or run scripts/fetch_vendor.sh.",
          file=sys.stderr, flush=True)


class VideoWriter:
    """H.264 when PyAV is available, `mp4v` when it is not.

    The fallback is deliberate. A fresh clone that has not run `scripts/fetch_vendor.sh` yet
    should still record something watchable rather than crash mid-flight — losing the codec
    is worth far less than losing the run.
    """

    #: The timebase every frame is stamped in. A millisecond is fine for video — 1 ms is
    #: 1/33rd of a frame at 30 fps — and coarse enough that the monotonic guard almost never
    #: has to nudge anything.
    TIME_BASE = Fraction(1, 1000)

    def __init__(self, path, width, height, fps=10.0, crf=26, preset="medium"):
        self.path, self.width, self.height = path, int(width), int(height)
        self.fps = float(fps)
        self.frames = 0
        self._t_first = self._t_last = None
        self._last_pts = None
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
            stream.time_base = self.TIME_BASE
            stream.codec_context.time_base = self.TIME_BASE
            self._stream = stream
            self.codec = "h264"
        else:
            _warn_fallback(path)
            import cv2
            self._cv = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"),
                                       self.fps, (self.width, self.height))
            if not self._cv.isOpened():
                raise RuntimeError(f"could not open {path} for writing")
            self.codec = "mp4v"

    def write(self, frame_bgr, t_s=None):
        """Encode one frame. `t_s` is its time in SECONDS since the recording started.

        Pass it whenever the source rate is not the nominal `fps`, which for a ROS image
        topic is always. Writing frames at a fixed nominal rate makes the video play at
        `fps / actual_rate` times real speed - measured here at 8.0 / 5.58 = **1.43x too
        fast**, which is why a 66 s episode became a 46 s file and drifted several seconds
        out of step with the chase camera over one clip.

        With `t_s` the frame carries a real presentation timestamp and the file is
        real-time regardless of what the source did.
        """
        if self._cv is not None:
            # The mp4v fallback has no PTS control; it is a fallback, not a measurement path.
            self._cv.write(frame_bgr)
            self.frames += 1
            return
        # PyAV wants a contiguous array; a slice of a larger buffer (which is what the BGRA
        # -> BGR crop produces) is not, and raises rather than silently misreading.
        if not frame_bgr.flags["C_CONTIGUOUS"]:
            frame_bgr = np.ascontiguousarray(frame_bgr)
        vframe = av.VideoFrame.from_ndarray(frame_bgr, format="bgr24")

        # Stamp the frame with WHEN IT HAPPENED, so the file is real-time regardless of
        # what the source managed. Three earlier attempts at this failed and cost whole
        # recordings; the reasons are now measured rather than guessed, and both are handled
        # here.
        #
        # 1. `stream.time_base` ALONE IS NOT ENOUGH. libav overwrites it while muxing, so the
        #    pts values end up read against a different base. Measured on a 10.15 s synthetic
        #    clip: stream time_base only -> a 507 s file. `frame.time_base` must be set on
        #    every frame; with it, the same clip comes out at 10.20 s.
        # 2. PTS MUST BE STRICTLY INCREASING. Two frames landing in the same millisecond, or
        #    one timestamp stepping backwards, both produce exactly the failure recorded
        #    before — `av.error.ArgumentError: Invalid argument ... returned 22`, raised by
        #    close() when the flush packets are muxed, i.e. after the whole episode is
        #    recorded. Real capture stamps do both. The guard below is what makes this safe.
        #
        # See tests/test_h264_timing.py, which reproduces both failures without a simulator —
        # which is why they survived three attempts made only in flight.
        if t_s is not None:
            if self._t_first is None:
                self._t_first = float(t_s)
            self._t_last = float(t_s)
            pts = int(round((float(t_s) - self._t_first) / self.TIME_BASE))
            if self._last_pts is not None and pts <= self._last_pts:
                pts = self._last_pts + 1        # never go backwards, never repeat
            vframe.pts = pts
            vframe.time_base = self.TIME_BASE
            self._last_pts = pts

        for packet in self._stream.encode(vframe):
            self._container.mux(packet)
        self.frames += 1

    def _write_timing(self):
        """Record what real span this file covers, beside it.

        `frames / fps` is the file's PLAYBACK length; `span` is how long the flight actually
        took. When a source cannot sustain its nominal rate - the camera at 5.58 Hz against a
        nominal 8, the chase at ~15 against 20 - those differ, and that difference is what
        makes two recordings of one flight drift apart.
        """
        if self._t_first is None or self._t_last is None:
            return
        try:
            import json
            with open(self.path + ".timing.json", "w") as fh:
                json.dump({"frames": self.frames, "nominal_fps": float(self.fps),
                           "span_s": round(self._t_last - self._t_first, 3)}, fh)
        except OSError:
            pass                              # a missing sidecar must never fail a recording

    def close(self):
        """Flush and finalise. Safe to call twice — a killed sweep may reach it twice."""
        self._write_timing()
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
