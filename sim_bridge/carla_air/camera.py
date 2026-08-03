"""Frame capture and the pixel→world grounding transform.

The capture cost is dominated by how many buffers are requested and at what size, and it is
wildly superlinear **in the number of large buffers**: with depth and segmentation rendered at
RGB resolution a single RGB+depth grab takes **15.4 s**.

The RGB buffer alone is far cheaper than that warning suggests. Raising it 640x480 -> 1440x1080
(2026-08-02, for 1080p recordings) is 5x the pixels and cost only **7.8 -> 7.15 Hz**, because
depth stayed at 160x120 and segmentation at 320x240. It is the *extra* buffers that must stay
small, not RGB.

**All three must keep the same 4:3 aspect.** `FOV_Degrees` is horizontal, so two buffers with
matching HFOV and different aspects cover different *vertical* fields — and `frames.scale_to()`
would then map an RGB pixel onto the wrong depth pixel, silently, on every waypoint. This is
why the camera is 1440x1080 rather than the more obvious 1920x1080.
"""
from __future__ import annotations

import numpy as np

import airsim

from .frames import Intrinsics, unproject

SKY_DEPTH = 65504.0  # float16 max — AirSim's "nothing here" marker


class Camera:
    def __init__(self, client: airsim.MultirotorClient, name: str = "0",
                 vehicle: str = "SimpleFlight"):
        self._c = client
        self.name = name
        self.vehicle = vehicle

    # ---------- geometry ----------

    def info(self):
        i = self._c.simGetCameraInfo(self.name, vehicle_name=self.vehicle)
        p, q = i.pose.position, i.pose.orientation
        return {
            "hfov_deg": i.fov,
            "position": [p.x_val, p.y_val, p.z_val],
            "orientation": [q.w_val, q.x_val, q.y_val, q.z_val],
        }

    def set_pose(self, xyz=(0.5, 0.0, 0.1), pitch=0.0, roll=0.0, yaw=0.0):
        """Point the camera. Pitch is real — the grounding transform must use camera pose."""
        self._c.simSetCameraPose(
            self.name,
            airsim.Pose(airsim.Vector3r(*xyz), airsim.to_quaternion(pitch, roll, yaw)),
            vehicle_name=self.vehicle,
        )

    # ---------- capture ----------

    def capture(self, rgb=True, depth=True, segmentation=False):
        """Grab the requested buffers in one round trip. Returns numpy arrays."""
        req, order = [], []
        if rgb:
            req.append(airsim.ImageRequest(self.name, airsim.ImageType.Scene, False, False))
            order.append("rgb")
        if depth:
            req.append(airsim.ImageRequest(self.name, airsim.ImageType.DepthPerspective, True, False))
            order.append("depth")
        if segmentation:
            req.append(airsim.ImageRequest(self.name, airsim.ImageType.Segmentation, False, False))
            order.append("segmentation")

        out = {}
        for key, r in zip(order, self._c.simGetImages(req, vehicle_name=self.vehicle)):
            if r.height == 0:
                out[key] = None
                continue
            if key == "depth":
                out[key] = np.array(r.image_data_float, dtype=np.float32).reshape(r.height, r.width)
            else:
                out[key] = np.frombuffer(r.image_data_uint8, dtype=np.uint8).reshape(
                    r.height, r.width, 3)
        return out

    # ---------- grounding ----------

    def ground(self, u: int, v: int, rgb_shape, depth, info=None):
        """A pixel in the RGB frame → a world NED point. The See-Point-Fly transform.

        `u, v` index the **RGB** frame; depth is smaller, so the index is scaled. Returns
        None when the pixel lands on sky, which is a legitimate VLM answer and must not be
        flown to.
        """
        info = info or self.info()
        h, w = rgb_shape[0], rgb_shape[1]
        dh, dw = depth.shape
        du = min(dw - 1, int(u * dw / w))
        dv = min(dh - 1, int(v * dh / h))
        d = float(depth[dv, du])
        if not np.isfinite(d) or d >= SKY_DEPTH * 0.99:
            return None

        intr = Intrinsics(width=w, height=h, hfov_deg=info["hfov_deg"])
        point = unproject(u, v, d, intr, info["position"], info["orientation"])
        return {"point": list(point), "depth": d, "pixel": [int(u), int(v)]}
