#!/usr/bin/env python
"""Composite the exterior chase and the drone's own view into one file.

    ./.venv/bin/python scripts/combine_views.py <chase.mp4> <onboard.mp4> <out.mp4> [depth.mp4]

The chase is the canvas; the model's inputs are inset over it. They are not the same kind of
evidence - the chase shows where the aircraft went, the onboard shows what the model saw and
said - so scaling them side by side would imply they are peers.

**The two are only approximately in sync.** They are separate recorders started a second or
so apart, with no shared clock — the chase runs at 30 fps from a CARLA sensor, the onboard at
8 fps off the ROS image topic. Frames are matched by presentation timestamp with the latest
onboard frame held between updates, which is right to within a frame or two. Do not use this
file to argue about causality at sub-second resolution; use the episode JSON.
"""
import json
import os
import sys

import av
import numpy as np

INSET_W, MARGIN, BORDER = 860, 28, 3


def _panel(img, w, h):
    return av.VideoFrame.from_ndarray(img, format="bgr24") \
             .reformat(width=w, height=h, format="bgr24").to_ndarray(format="bgr24")


def main(chase_path, onboard_path, out_path, depth_path=None):
    chase = av.open(chase_path)
    onboard = av.open(onboard_path)
    cs, os_ = chase.streams.video[0], onboard.streams.video[0]

    out = av.open(out_path, "w")
    st = out.add_stream("libx264", rate=cs.average_rate)
    st.width, st.height, st.pix_fmt = cs.codec_context.width, cs.codec_context.height, "yuv420p"
    st.options = {"crf": "23", "preset": "veryfast"}

    # SYNC BY THE END, not the start. The two recorders are started at different moments -
    # the episode (and with it the onboard recorder) begins before run_episode calls
    # chase_recording - but both are stopped together in the same `finally`. Aligning the
    # starts therefore drifts by however long that gap was; aligning the ends does not.
    # The offset written by run_episode when it started the chase. Preferred over guessing
    # from durations, which cannot work: the chase both starts later and stops later than the
    # onboard recorder, so their difference mixes two unknowns.
    # Each stream's PLAYBACK length divided by the real span it covers. A source that
    # cannot sustain its nominal rate produces a file shorter than the flight, and the two
    # recordings then drift apart even though they started together - which is what "the
    # video is out of sync" has meant every time. Rescaling here is safe: nothing can break
    # a recording that has already been written.
    def real_rate(path, stream):
        try:
            with open(path + ".timing.json") as fh:
                t = json.load(fh)
            if t.get("span_s", 0) > 1.0 and t.get("frames", 0) > 1:
                return t["frames"] / t["span_s"], t["span_s"]
        except (OSError, ValueError, KeyError):
            pass
        return None, None

    c_rate, c_span = real_rate(chase_path, cs)
    o_rate, o_span = real_rate(onboard_path, os_)
    c_nom = float(cs.average_rate or 30)
    o_nom = float(os_.average_rate or 8)

    def scale_for(container, stream, rate, nom, span, label):
        """How much faster this file plays than reality.

        Since 2026-08-06 the writer stamps every frame with when it happened, so a new file
        is ALREADY real-time and rescaling it would break what it fixed. Detect that from the
        file rather than from a flag: if the container's own duration already matches the span
        recorded beside it, there is nothing to correct.

        Files recorded before that still need the old correction, and there are plenty of them
        in `out/`, so both paths stay.
        """
        if not rate:
            return 1.0
        dur = float(container.duration) / 1_000_000 if container.duration else 0.0
        if span and dur and abs(dur - span) < max(0.5, 0.05 * span):
            print(f"  {label}: already real-time ({dur:.1f}s for a {span:.1f}s span) — "
                  f"no rescale")
            return 1.0
        return nom / rate

    c_scale = scale_for(chase, cs, c_rate, c_nom, c_span, "chase")
    o_scale = scale_for(onboard, os_, o_rate, o_nom, o_span, "onboard")
    if c_rate or o_rate:
        print(f"  real rates: chase {c_rate or c_nom:.2f} fps (file {c_nom:.0f}), "
              f"onboard {o_rate or o_nom:.2f} fps (file {o_nom:.0f})")
        print(f"  playback scale: chase x{c_scale:.3f}, onboard x{o_scale:.3f}")

    lead = 0.0
    sync = chase_path.replace(".mp4", ".sync.json")
    if os.path.exists(sync):
        with open(sync) as fh:
            lead = float(json.load(fh).get("chase_start_after_episode_s", 0.0))
        print(f"  measured offset: onboard leads the chase by {lead:.2f}s")
    else:
        print("  no .sync.json beside the chase file — assuming the two start together, "
              "which is wrong by a second or two")

    inset_h = int(INSET_W * os_.codec_context.height / os_.codec_context.width)
    dep_h = int(INSET_W * 0.5 * 3 / 4)
    ob_iter = onboard.decode(os_)
    ob_frame = next(ob_iter, None)
    ob_img = None

    # The depth buffer is 160x120 - deliberately small, because depth is transport-bound
    # rather than render-bound (see docs/architecture.md). It is upscaled here to sit beside
    # the RGB inset; it will look blocky, and that is what the model's geometry is actually
    # computed from.
    dep = av.open(depth_path) if depth_path else None
    ds = dep.streams.video[0] if dep else None
    dep_iter = dep.decode(ds) if dep else iter(())
    dep_frame = next(dep_iter, None)
    dep_img = None
    n = 0

    for frame in chase.decode(cs):
        t_file = float(frame.pts * cs.time_base) if frame.pts is not None else 0.0
        t = t_file * c_scale                  # chase file time -> REAL seconds since start
        # Advance the onboard stream to the newest frame at or before this instant.
        while ob_frame is not None and ob_frame.pts is not None and \
                float(ob_frame.pts * os_.time_base) * o_scale <= t + lead:
            ob_img = ob_frame.to_ndarray(format="bgr24")
            ob_frame = next(ob_iter, None)

        while dep_frame is not None and dep_frame.pts is not None and \
                float(dep_frame.pts * ds.time_base) * o_scale <= t + lead:
            dep_img = dep_frame.to_ndarray(format="bgr24")
            dep_frame = next(dep_iter, None)

        canvas = frame.to_ndarray(format="bgr24")
        if dep_img is not None:
            dw = int(INSET_W * 0.5)
            small = _panel(dep_img, dw, dep_h)
            y1, x1 = MARGIN, canvas.shape[1] - dw - MARGIN
            canvas[y1 - BORDER:y1 + dep_h + BORDER, x1 - BORDER:x1 + dw + BORDER] = 235
            canvas[y1:y1 + dep_h, x1:x1 + dw] = small
            cv_label(canvas, "depth", x1 + 8, y1 + dep_h - 10)
        if ob_img is not None:
            small = _panel(ob_img, INSET_W, inset_h)
            y1 = canvas.shape[0] - inset_h - MARGIN
            x1 = canvas.shape[1] - INSET_W - MARGIN
            canvas[y1 - BORDER:y1 + inset_h + BORDER, x1 - BORDER:x1 + INSET_W + BORDER] = 235
            canvas[y1:y1 + inset_h, x1:x1 + INSET_W] = small

        for p in st.encode(av.VideoFrame.from_ndarray(canvas, format="bgr24")):
            out.mux(p)
        n += 1

    for p in st.encode():
        out.mux(p)
    out.close(); chase.close(); onboard.close()
    if dep:
        dep.close()
    print(f"  {n} frames -> {out_path}")


def cv_label(img, text, x, y):
    import cv2
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (235, 235, 235), 2, cv2.LINE_AA)


if __name__ == "__main__":
    main(*sys.argv[1:5])
