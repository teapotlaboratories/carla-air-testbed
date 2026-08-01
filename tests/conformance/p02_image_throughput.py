#!/usr/bin/env python3
"""P02 — how fast can we pull frames out of the drone camera?

This is the input side of the VLM loop. See-Point-Fly runs the VLM at ~0.3-1 Hz
and a geometric controller underneath at 10-50 Hz, so the frame-grab path only
has to clear a few Hz — but if `simGetImages` blocks the render thread for
hundreds of milliseconds it also stalls the flight physics, which is the thing
worth measuring.
"""
import time

import numpy as np

import common

p = common.Probe("p02_image_throughput")
a = common.airsim_client()
a.reset()
time.sleep(3)
a.enableApiControl(True)
a.armDisarm(True)
# Must command something before measuring anything: after reset() the vehicle
# does not hold altitude, it climbs at ~7 m/s until it is given a setpoint.
# See p09 — benchmarking without this measures a runaway, not a frame rate.
a.moveToPositionAsync(0, 0, -20, 6).join()
time.sleep(2)

import airsim


def bench(label, requests, n=20):
    lat = []
    ok = 0
    for _ in range(n):
        t0 = time.perf_counter()
        r = a.simGetImages(requests)
        lat.append((time.perf_counter() - t0) * 1000.0)
        if r and r[0].height > 0:
            ok += 1
    lat = np.array(lat)
    p.metric(f"{label}_hz", round(1000.0 / lat.mean(), 2))
    p.metric(f"{label}_ms_mean", round(float(lat.mean()), 1))
    p.metric(f"{label}_ms_p95", round(float(np.percentile(lat, 95)), 1))
    p.check(f"{label}: all {n} grabs returned an image", ok == n, f"{ok}/{n}")
    return lat.mean()


scene = [airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)]
rgbd = [
    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
    airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False),
]
triple = rgbd + [airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)]

m_scene = bench("rgb_1280x960", scene)
bench("rgb_plus_depth", rgbd)
bench("rgb_depth_seg", triple)

p.check("RGB alone clears 5 Hz (VLM input budget)", 1000.0 / m_scene >= 5.0,
        f"{1000.0/m_scene:.1f} Hz")

# Does grabbing frames stall the physics? Command a hover and watch position
# drift while hammering the image API.
a.moveToPositionAsync(0, 0, -20, 4).join()
time.sleep(1)
p0 = a.getMultirotorState().kinematics_estimated.position
t0 = time.time()
grabs = 0
while time.time() - t0 < 10.0:
    a.simGetImages(triple)
    grabs += 1
p1 = a.getMultirotorState().kinematics_estimated.position
drift = ((p1.x_val - p0.x_val) ** 2 + (p1.y_val - p0.y_val) ** 2 + (p1.z_val - p0.z_val) ** 2) ** 0.5
p.metric("grabs_in_10s_while_hovering", grabs)
p.metric("hover_drift_m_under_image_load", round(drift, 2))
p.check("hover holds under sustained image load (<3 m drift)", drift < 3.0, f"{drift:.2f} m")

common.land(a)
p.finish()
