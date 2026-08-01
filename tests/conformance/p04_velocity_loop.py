#!/usr/bin/env python3
"""P04 — the action side of the VLM loop: closed-loop velocity commands.

See-Point-Fly turns each VLM annotation into a 3D displacement and streams it as
a velocity/position command while the VLM thinks about the next one. That means
the interesting properties are not "does moveByVelocity exist" but:

  * command → motion latency (how stale is the drone's response),
  * whether a *streamed* command at 10 Hz is tracked or fought by the
    controller's own timeout behaviour,
  * closed-loop position error when we drive it to a target ourselves.

`duration` on moveByVelocityAsync must exceed the resend period or the vehicle
stalls between commands; this probe uses 0.5 s at 10 Hz, the standard recipe.
"""
import math
import time

import numpy as np

import airsim
import common

p = common.Probe("p04_velocity_loop")
a = common.airsim_client()
a.reset()
time.sleep(3)
a.enableApiControl(True)
a.armDisarm(True)
a.moveToPositionAsync(0, 0, -30, 8).join()   # command a setpoint first — see p09
time.sleep(1.5)

# ---- command → motion latency ----
lat = []
for _ in range(5):
    a.moveByVelocityAsync(0, 0, 0, 0.2).join()
    time.sleep(0.6)
    v0 = a.getMultirotorState().kinematics_estimated.linear_velocity
    t0 = time.perf_counter()
    a.moveByVelocityAsync(4.0, 0, 0, 2.0)
    hit = None
    while time.perf_counter() - t0 < 2.0:
        v = a.getMultirotorState().kinematics_estimated.linear_velocity
        if v.x_val > 1.0:
            hit = (time.perf_counter() - t0) * 1000.0
            break
    if hit:
        lat.append(hit)
    a.moveByVelocityAsync(0, 0, 0, 0.5).join()
    time.sleep(0.8)

p.check("velocity command produces motion", len(lat) == 5, f"{len(lat)}/5 attempts responded")
if lat:
    p.metric("cmd_to_1mps_ms_mean", round(float(np.mean(lat)), 1))
    p.metric("cmd_to_1mps_ms_max", round(float(np.max(lat)), 1))
    p.check("responds within 500 ms", np.mean(lat) < 500, f"{np.mean(lat):.0f} ms")

# ---- streamed 10 Hz velocity, the SPF inner loop shape ----
a.moveToPositionAsync(0, 0, -30, 6).join()
time.sleep(1.5)
start = a.getMultirotorState().kinematics_estimated.position
target = np.array([start.x_val + 40.0, start.y_val + 25.0, -30.0])
track = []
t0 = time.time()
period = 0.1
while time.time() - t0 < 25.0:
    k = a.getMultirotorState().kinematics_estimated
    cur = np.array([k.position.x_val, k.position.y_val, k.position.z_val])
    err = target - cur
    dist = float(np.linalg.norm(err))
    track.append((time.time() - t0, dist))
    if dist < 1.5:
        break
    v = err / dist * min(6.0, dist)
    a.moveByVelocityAsync(float(v[0]), float(v[1]), float(v[2]), 0.5)
    time.sleep(period)

k = a.getMultirotorState().kinematics_estimated.position
final = np.array([k.x_val, k.y_val, k.z_val])
final_err = float(np.linalg.norm(target - final))
p.metric("streamed_cmds_sent", len(track))
p.metric("time_to_target_s", round(track[-1][0], 1))
p.metric("final_position_error_m", round(final_err, 2))
p.check("10 Hz streamed velocity reaches the target (<2 m)", final_err < 2.0, f"{final_err:.2f} m")
p.check("no stall — kept closing the whole way",
        track[-1][1] < track[0][1] * 0.1, f"{track[0][1]:.1f} m → {track[-1][1]:.1f} m")

# ---- yaw control, needed to point the camera where the VLM looked ----
a.rotateToYawAsync(90, timeout_sec=10).join()
time.sleep(1)
q = a.getMultirotorState().kinematics_estimated.orientation
yaw = math.degrees(math.atan2(2 * (q.w_val * q.z_val + q.x_val * q.y_val),
                              1 - 2 * (q.y_val ** 2 + q.z_val ** 2)))
p.metric("yaw_after_rotate_to_90_deg", round(yaw, 1))
p.check("yaw command lands within 10 deg", abs(((yaw - 90 + 180) % 360) - 180) < 10, f"{yaw:.1f} deg")

common.land(a)
p.finish()
