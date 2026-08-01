#!/usr/bin/env python3
"""P03 — are RGB / depth / segmentation actually usable, or just present?

Cosys-AirSim on UE5.5 renders segmentation and annotation buffers all black
(Cosys-Lab/Cosys-AirSim#135), and segmentation ground truth is load-bearing for
VLM grounding work. CARLA-Air is a different lineage (UE4.26), so the same
question has to be asked here rather than assumed. "Present" is not the check —
"carries information" is: non-degenerate variance for RGB, a plausible metric
range for depth, and more than one class id for segmentation.
"""
import time

import cv2
import numpy as np

import airsim
import common

p = common.Probe("p03_modalities")
a = common.airsim_client()
a.reset()
time.sleep(3)
a.enableApiControl(True)
a.armDisarm(True)
a.moveToPositionAsync(0, 0, -40, 8).join()   # command a setpoint first — see p09
_, ned = common.goto_city(a, common.carla_world()[1], altitude=-45.0)
p.note("positioned over a CARLA spawn point", f"AirSim NED {[round(v,1) for v in ned]}")
a.simSetCameraPose("0", airsim.Pose(airsim.Vector3r(0.5, 0, 0.1), airsim.to_quaternion(-0.9, 0, 0)))
time.sleep(2)

req = [
    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
    airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False),
    airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False),
    airsim.ImageRequest("0", airsim.ImageType.SurfaceNormals, False, False),
    airsim.ImageRequest("0", airsim.ImageType.DepthVis, False, False),
]
r = a.simGetImages(req)

# ---- RGB ----
rgb = np.frombuffer(r[0].image_data_uint8, dtype=np.uint8).reshape(r[0].height, r[0].width, 3)
cv2.imwrite(f"{common.OUT}/p03_rgb.png", rgb)
p.metric("rgb_shape", f"{r[0].width}x{r[0].height}")
p.metric("rgb_std", round(float(rgb.std()), 2))
p.check("RGB is not a flat frame", rgb.std() > 10, f"std={rgb.std():.1f}")

# ---- depth ----
d = np.array(r[1].image_data_float, dtype=np.float32).reshape(r[1].height, r[1].width)
finite = d[np.isfinite(d)]
p.metric("depth_min_m", round(float(finite.min()), 2))
p.metric("depth_median_m", round(float(np.median(finite)), 2))
p.metric("depth_p99_m", round(float(np.percentile(finite, 99)), 2))
p.check("depth has a plausible metric range", 0.1 < np.median(finite) < 5000, f"median {np.median(finite):.1f} m")
alt = abs(a.getMultirotorState().kinematics_estimated.position.z_val)
# camera looks forward, so the nadir-ish bottom rows should be roughly the AGL
bottom = np.median(d[-40:, r[1].width // 2 - 20 : r[1].width // 2 + 20])
p.metric("altitude_m", round(alt, 1))
p.metric("depth_bottom_center_m", round(float(bottom), 1))
cv2.imwrite(f"{common.OUT}/p03_depth.png", cv2.applyColorMap(np.clip(d / 100 * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_JET))

# ---- segmentation ----
seg = np.frombuffer(r[2].image_data_uint8, dtype=np.uint8).reshape(r[2].height, r[2].width, 3)
cv2.imwrite(f"{common.OUT}/p03_seg.png", seg)
classes = np.unique(seg.reshape(-1, 3), axis=0)
p.metric("segmentation_distinct_colors", len(classes))
p.check("segmentation is not all black", seg.max() > 0, f"max={seg.max()}")
p.check("segmentation separates >2 classes", len(classes) > 2, f"{len(classes)} distinct colours")

# ---- extras ----
for idx, name in ((3, "surface_normals"), (4, "depth_vis")):
    ok = r[idx].height > 0
    p.check(f"{name} available", ok, f"{r[idx].width}x{r[idx].height}")

p.note("images written", f"{common.OUT}/p03_*.png")
common.land(a)
p.finish()
