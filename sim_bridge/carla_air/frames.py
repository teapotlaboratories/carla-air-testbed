"""Coordinate frames and camera geometry.

Three frames are in play and two of them are traps.

**CARLA** is x-forward / y-right / z-**up**, metres at the Python API. **AirSim NED** is
x/y/z-**down**, metres, with its origin at the AirSim PlayerStart — which on Town10HD is
**offshore**, so flying to a raw CARLA x/y puts the aircraft over open water. **PX4/ROS 2**
downstream is also NED, and matches AirSim directly, which is the one piece of luck here.

The CARLA→AirSim offsets are upstream's, re-measured in the conformance suite
(`tests/conformance/p06_air_ground_sync.py`) rather than trusted. They are per-map: change
the map and they must be recalibrated with `calibrate()`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# Town10HD, from upstream COORDINATE_SYSTEMS.md, confirmed by p06.
OFFSETS = {
    "Town10HD": (172.20, -183.86, 27.45),
    "Town10HD_Opt": (172.20, -183.86, 27.45),
}
DEFAULT_OFFSET = OFFSETS["Town10HD"]


def carla_to_ned(x: float, y: float, z: float, offset=DEFAULT_OFFSET):
    """CARLA location (metres, z-up) → AirSim/PX4 NED (metres, z-down)."""
    return (x + offset[0], y + offset[1], -z + offset[2])


def ned_to_carla(x: float, y: float, z: float, offset=DEFAULT_OFFSET):
    """AirSim/PX4 NED → CARLA location."""
    return (x - offset[0], y - offset[1], -(z - offset[2]))


def calibrate(carla_xyz, ned_xyz):
    """Derive the offset from one point observed in both frames.

    Read the drone's pose from CARLA and from AirSim at the same instant and pass both;
    the result replaces the table entry for a new map.
    """
    cx, cy, cz = carla_xyz
    nx, ny, nz = ned_xyz
    return (nx - cx, ny - cy, nz + cz)


def quat_to_matrix(w: float, x: float, y: float, z: float):
    """Unit quaternion → 3x3 rotation matrix, as nested lists (no numpy here)."""
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def quat_to_yaw(w: float, x: float, y: float, z: float) -> float:
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole model derived from AirSim's horizontal FOV. Square pixels, so fx == fy."""

    width: int
    height: int
    hfov_deg: float

    @property
    def fx(self) -> float:
        return (self.width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)

    @property
    def fy(self) -> float:
        return self.fx  # square pixels

    @property
    def cx(self) -> float:
        return self.width / 2.0

    @property
    def cy(self) -> float:
        return self.height / 2.0

    def scale_to(self, other: "Intrinsics"):
        """Pixel scale from this frame to `other` — for the RGB→depth index hop.

        Depth and segmentation are rendered smaller than RGB on purpose: matching them to
        RGB resolution costs 15.4 s per grab (see the worklog). `configs/sim/settings.json`
        keeps the aspect ratios equal so this scale is exact.
        """
        return other.width / self.width, other.height / self.height


def project(point_ned, intr: Intrinsics, cam_pos, cam_quat):
    """World NED point -> (u, v, depth). The exact inverse of `unproject`.

    Returns None when the point is behind the camera. Pixels outside the frame are returned
    unclipped: a caller wanting "the closest visible pixel" needs to know the target is
    off-screen and by how much, which clipping would hide.

    This exists for the oracle backend — the only way to tell a bad scenario from a bad
    model is to have something that provably flies to the goal.
    """
    r = quat_to_matrix(*cam_quat)
    d = [point_ned[i] - cam_pos[i] for i in range(3)]
    # world -> camera is the transpose of camera -> world; rotation matrices are orthonormal,
    # so column i of r dotted with d gives the camera-frame component directly.
    fwd = sum(r[i][0] * d[i] for i in range(3))
    right = sum(r[i][1] * d[i] for i in range(3))
    down = sum(r[i][2] * d[i] for i in range(3))
    if fwd <= 1e-6:
        return None
    return (intr.cx + right / fwd * intr.fx,
            intr.cy + down / fwd * intr.fy,
            fwd)


def unproject(u: float, v: float, depth_m: float, intr: Intrinsics, cam_pos, cam_quat):
    """Pixel + *perspective* depth → world NED point.

    AirSim's `DepthPerspective` is planar z-depth, not ray length, which is why the camera
    ray is built as (depth, right, down) and scaled by the normalised pixel offsets rather
    than by a unit vector.

    `cam_pos`/`cam_quat` must come from `simGetCameraInfo`, **not** from the vehicle pose:
    the camera is gimbal-less but `simSetCameraPose` pitch is real, and a yaw-only rotation
    silently puts the waypoint on the horizon.
    """
    ray = (
        depth_m,
        (u - intr.cx) / intr.fx * depth_m,
        (v - intr.cy) / intr.fx * depth_m,
    )
    r = quat_to_matrix(*cam_quat)
    return tuple(cam_pos[i] + sum(r[i][j] * ray[j] for j in range(3)) for i in range(3))
