#!/usr/bin/env python3
"""P05 — the See-Point-Fly transform, end to end, with no VLM in the way.

SPF's whole claim is that AVLN reduces to 2D spatial grounding: the VLM annotates
a pixel, and the system turns that pixel into a 3D displacement command. Whether
a simulator can host SPF therefore comes down to one question this probe answers
directly:

    pick a pixel → read its depth → unproject to a world point → fly there →
    is the drone where the pixel said it would be?

If that closes to within a couple of metres, the VLM is a drop-in: swapping a
hand-picked pixel for a model-picked one changes nothing else in the loop. If it
does not close, no amount of VLM quality rescues it.

Camera model: AirSim gives a horizontal FOV; the image is rendered with square
pixels, so fx = fy = (W/2) / tan(hfov/2). Camera frame is x-forward, y-right,
z-down (NED-aligned), which is why the unprojection below maps
(depth, right, down) straight onto body axes with no axis swap.
"""
import math
import time

import cv2
import numpy as np

import airsim
import common

p = common.Probe("p05_pixel_to_waypoint")
a = common.airsim_client()
a.reset()
time.sleep(3)
a.enableApiControl(True)
a.armDisarm(True)
a.moveToPositionAsync(0, 0, -40, 8).join()   # command a setpoint first — see p09
import common as _c
_, ned = _c.goto_city(a, _c.carla_world()[1], altitude=-55.0)
p.note("positioned over a CARLA spawn point", f"AirSim NED {[round(v,1) for v in ned]}")
a.rotateToYawAsync(0, timeout_sec=10).join()
a.simSetCameraPose("0", airsim.Pose(airsim.Vector3r(0.5, 0, 0.1), airsim.to_quaternion(-0.5, 0, 0)))
time.sleep(2)

info = a.simGetCameraInfo("0")
hfov = math.radians(info.fov)
p.metric("camera_hfov_deg", round(info.fov, 2))

r = a.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
    airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False),
])
W, H = r[0].width, r[0].height
DW, DH = r[1].width, r[1].height
rgb = np.frombuffer(r[0].image_data_uint8, dtype=np.uint8).reshape(H, W, 3)
depth = np.array(r[1].image_data_float, dtype=np.float32).reshape(DH, DW)

# Shipped settings.json configures CaptureSettings for ImageType 0 only, so RGB
# comes back at 1280x960 (4:3) while depth/segmentation fall back to AirSim's
# 256x144 (16:9) default. A VLM annotates a pixel in the RGB frame; reading depth
# at that index indexes a different part of the scene — or throws.
#
# Equal resolution is NOT the fix: rendering depth at RGB resolution costs 15 s
# per grab (see the worklog). What must match is the *aspect ratio*, so the
# pixel scale below is exact and depth can stay small and cheap.
p.metric("depth_resolution", f"{DW}x{DH}")
p.check("RGB and depth share an aspect ratio (exact pixel scaling)",
        abs(W / H - DW / DH) < 0.01,
        f"rgb {W}x{H} ({W/H:.3f}) vs depth {DW}x{DH} ({DW/DH:.3f})")


def depth_at(u, v):
    """depth at an RGB-frame pixel, through the resolution mismatch."""
    return float(depth[min(DH - 1, int(v * DH / H)), min(DW - 1, int(u * DW / W))])

fx = (W / 2.0) / math.tan(hfov / 2.0)
fy = fx
cx, cy = W / 2.0, H / 2.0
p.metric("fx_px", round(fx, 1))

pos = np.array([a.getMultirotorState().kinematics_estimated.position.x_val,
                a.getMultirotorState().kinematics_estimated.position.y_val,
                a.getMultirotorState().kinematics_estimated.position.z_val])

# Use the *camera* pose, not the body pose: the camera is pitched down here, and
# a yaw-only rotation would silently put the waypoint on the horizon.
cam_pos = np.array([info.pose.position.x_val, info.pose.position.y_val, info.pose.position.z_val])
q = info.pose.orientation


def quat_to_R(q):
    w, x, y, z = q.w_val, q.x_val, q.y_val, q.z_val
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


R = quat_to_R(q)
p.metric("camera_pitch_deg", round(math.degrees(math.asin(max(-1.0, min(1.0, -R[2, 0])))), 1))


def unproject(u, v, d):
    """pixel + perspective depth → world NED point, through the camera pose."""
    ray_cam = np.array([d, (u - cx) / fx * d, (v - cy) / fy * d])  # fwd, right, down
    return cam_pos + R @ ray_cam


# Choose a target pixel the way SPF would: something with a finite, mid-range
# depth, off-centre so the unprojection is actually exercised in x and y.
best = None
for u in range(int(W * 0.15), int(W * 0.85), 17):
    for v in range(int(H * 0.15), int(H * 0.85), 17):
        d = depth_at(u, v)
        if 30.0 < d < 150.0:
            score = abs(u - cx) + abs(v - cy)
            if best is None or score > best[0]:
                best = (score, u, v, d)

p.check("found a mid-range pixel to ground on", best is not None,
        "" if best else "no pixel with 30-150 m depth in view")
if best is None:
    common.land(a)
    p.finish()

_, u, v, d = best
p.metric("target_pixel", f"({u},{v})")
p.metric("target_pixel_depth_m", round(d, 2))

target = unproject(u, v, d)
# SPF does not fly the whole way to an annotation — it takes a bounded step along
# the displacement and re-observes. Doing the same here keeps the test honest:
# the pixel grounds on a *surface*, so commanding the full ray means commanding a
# controlled flight into a building (verified: it collided with
# BP_Block13NY_Top_C_1024 when this probe did exactly that).
ray = target - pos
ray_len = float(np.linalg.norm(ray))
STEP = min(20.0, 0.4 * ray_len)
stop = pos + ray / ray_len * STEP
p.metric("ray_length_m", round(ray_len, 1))
p.metric("commanded_step_m", round(STEP, 1))
p.metric("unprojected_world_target_NED", [round(float(x), 2) for x in target])
p.metric("commanded_stop_point_NED", [round(float(x), 2) for x in stop])

vis = rgb.copy()
cv2.circle(vis, (u, v), 12, (0, 0, 255), 3)
cv2.line(vis, (int(cx), int(cy)), (u, v), (0, 255, 255), 2)
cv2.imwrite(f"{common.OUT}/p05_annotated.png", vis)

# Fly it with streamed velocity — the SPF inner loop, not a position setpoint.
t0 = time.time()
while time.time() - t0 < 30.0:
    k = a.getMultirotorState().kinematics_estimated.position
    cur = np.array([k.x_val, k.y_val, k.z_val])
    err = stop - cur
    dist = float(np.linalg.norm(err))
    if dist < 1.0:
        break
    vel = err / dist * min(5.0, dist)
    a.moveByVelocityAsync(float(vel[0]), float(vel[1]), float(vel[2]), 0.5)
    time.sleep(0.1)
a.moveByVelocityAsync(0, 0, 0, 1.0).join()
time.sleep(1.5)

k = a.getMultirotorState().kinematics_estimated.position
final = np.array([k.x_val, k.y_val, k.z_val])
err = float(np.linalg.norm(stop - final))
p.metric("arrival_error_m", round(err, 2))
p.check("flew the pixel-derived step (<2.5 m error)", err < 2.5, f"{err:.2f} m")
travelled = float(np.linalg.norm(final - pos))
p.metric("distance_travelled_m", round(travelled, 2))

# And the real closing question: is the thing we pointed at now in front of us,
# at roughly the distance geometry predicted?
r2 = a.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False)])
d2 = np.array(r2[0].image_data_float, dtype=np.float32).reshape(r2[0].height, r2[0].width)
# The pixel we grounded on should now be STANDOFF metres away. Sample the same
# pixel: the camera pose has not changed, so the target sits near where it was.
dv, du = int(v * d2.shape[0] / H), int(u * d2.shape[1] / W)
at_pixel = float(np.median(d2[max(0, dv - 4):dv + 4, max(0, du - 4):du + 4]))
p.metric("depth_at_target_pixel_after_flight_m", round(at_pixel, 2))
predicted = d - travelled
p.metric("predicted_depth_after_step_m", round(predicted, 2))
# This is the whole grounding claim in one number: if the pixel really denoted a
# point in space, closing `travelled` metres along the ray must reduce the depth
# at that pixel by the same amount.
p.check("depth at the grounded pixel fell by the distance flown (±15%)",
        abs(at_pixel - predicted) < max(4.0, 0.15 * d),
        f"{at_pixel:.1f} m measured vs {predicted:.1f} m predicted (was {d:.1f} m)")

time.sleep(0.5)
collision = a.simGetCollisionInfo()
p.check("no collision during the grounded flight", not collision.has_collided,
        collision.object_name if collision.has_collided else "clean")

common.land(a)
p.finish()
