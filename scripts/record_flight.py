#!/usr/bin/env python3
"""Record a test flight to MP4 — evidence that a probe run actually flew.

Flies a short scripted tour over Town10HD with CARLA traffic spawned underneath
and writes the drone camera to out/flight.mp4, with a HUD showing NED position,
speed and the active leg. Not a probe: it asserts nothing, it just produces
something watchable.

    ./.venv/bin/python scripts/record_flight.py [seconds]

Encoding goes through OpenCV's bundled FFmpeg (mp4v); there is no system ffmpeg
on this box.
"""
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "tests", "conformance"))

import airsim  # noqa: E402
import carla  # noqa: E402
import common  # noqa: E402

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
FPS = 8.0

c, w = common.carla_world()
a = common.airsim_client()

# ---- traffic underneath, so the video shows a live city ----
bp = w.get_blueprint_library()
tm = c.get_trafficmanager()
car_bps = [b for b in bp.filter("vehicle.*") if int(b.get_attribute("number_of_wheels")) == 4]
batch = []
for i, sp in enumerate(w.get_map().get_spawn_points()[:25]):
    b = car_bps[i % len(car_bps)]
    b.set_attribute("role_name", "autopilot")
    batch.append(carla.command.SpawnActor(b, sp).then(
        carla.command.SetAutopilot(carla.command.FutureActor, True, tm.get_port())))
vehicle_ids = [r.actor_id for r in c.apply_batch_sync(batch, False) if not r.error]
print(f"spawned {len(vehicle_ids)} vehicles")

w.set_weather(carla.WeatherParameters.ClearNoon)

a.reset()
time.sleep(3)
a.enableApiControl(True)
a.armDisarm(True)
a.moveToPositionAsync(0, 0, -40, 8).join()          # setpoint first — see p09
sp, _ = common.goto_city(a, w, altitude=-70.0, speed=15.0)
a.simSetCameraPose("0", airsim.Pose(airsim.Vector3r(0.5, 0, 0.1), airsim.to_quaternion(-0.6, 0, 0)))
time.sleep(2)

k = a.getMultirotorState().kinematics_estimated.position
base = np.array([k.x_val, k.y_val])
LEGS = [
    ("cruise north", (base[0] + 90, base[1], -70)),
    ("descend + turn", (base[0] + 90, base[1] + 70, -45)),
    ("low pass", (base[0], base[1] + 70, -35)),
    ("climb home", (base[0], base[1], -70)),
]

writer, leg_i, leg_started = None, 0, time.time()
a.moveToPositionAsync(*LEGS[0][1], 8.0)
t0 = time.time()
frames = 0
try:
    while time.time() - t0 < DURATION:
        r = a.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
        if not r or r[0].height == 0:
            continue
        img = np.frombuffer(r[0].image_data_uint8, dtype=np.uint8).reshape(r[0].height, r[0].width, 3)
        img = np.array(img, dtype=np.uint8, copy=True)

        st = a.getMultirotorState().kinematics_estimated
        p_, v_ = st.position, st.linear_velocity
        speed = (v_.x_val ** 2 + v_.y_val ** 2 + v_.z_val ** 2) ** 0.5
        hud = [
            f"CARLA-Air v0.1.7  Town10HD   t={time.time()-t0:5.1f}s",
            f"NED  x{p_.x_val:8.1f}  y{p_.y_val:8.1f}  z{p_.z_val:8.1f}   alt {abs(p_.z_val):5.1f} m",
            f"speed {speed:4.1f} m/s   leg: {LEGS[leg_i][0]}   traffic: {len(vehicle_ids)} vehicles",
        ]
        cv2.rectangle(img, (0, 0), (img.shape[1], 78), (0, 0, 0), -1)
        for i, line in enumerate(hud):
            cv2.putText(img, line, (12, 24 + i * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

        if writer is None:
            writer = cv2.VideoWriter(f"{common.OUT}/flight.mp4",
                                     cv2.VideoWriter_fourcc(*"mp4v"), FPS,
                                     (img.shape[1], img.shape[0]))
        writer.write(img)
        frames += 1

        # advance the tour
        cur = np.array([p_.x_val, p_.y_val, p_.z_val])
        if np.linalg.norm(cur - np.array(LEGS[leg_i][1])) < 6.0 or time.time() - leg_started > DURATION / len(LEGS):
            leg_i = (leg_i + 1) % len(LEGS)
            leg_started = time.time()
            a.moveToPositionAsync(*LEGS[leg_i][1], 8.0)
finally:
    if writer:
        writer.release()
    common.land(a)
    c.apply_batch([carla.command.DestroyActor(i) for i in vehicle_ids])
    time.sleep(1)

print(f"wrote {common.OUT}/flight.mp4 — {frames} frames @ {FPS} fps ({frames/FPS:.0f} s)")
