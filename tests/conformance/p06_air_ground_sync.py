#!/usr/bin/env python3
"""P06 — is it really one world? CARLA actors and weather, seen from the drone.

This is CARLA-Air's differentiator over Cosys-AirSim: dense scripted traffic and
pedestrians under the aircraft, from CARLA's own traffic manager, with no bridge.
The claim only means something if the CARLA-side actors are alive and visible to
the AirSim-side camera, and if a CARLA weather call changes what the drone sees.

Measured, not asserted:
  * the CARLA→AirSim frame offset, re-derived here rather than taken from
    upstream's docs (fly to raw CARLA x/y and you end up in the sea — the AirSim
    NED origin on Town10HD is offshore),
  * whether traffic-manager vehicles actually drive,
  * whether ClearNoon vs HardRainSunset changes the drone's own frame.

Traffic is spawned the way upstream's auto_traffic.py does it — batched
SpawnActor().then(SetAutopilot(...)) — because spawning first and calling
set_autopilot() afterwards leaves the vehicles parked.
"""
import time

import cv2
import numpy as np

import airsim
import carla
import common

p = common.Probe("p06_air_ground_sync")
c, w = common.carla_world()
a = common.airsim_client()
a.reset()
time.sleep(3)
a.enableApiControl(True)
a.armDisarm(True)
a.moveToPositionAsync(0, 0, -40, 8).join()   # setpoint first — see p09

bp = w.get_blueprint_library()
spawn_points = w.get_map().get_spawn_points()
vehicle_ids, walker_ids, controller_ids = [], [], []
try:
    # ---- ground traffic, upstream's batch pattern ----
    tm = c.get_trafficmanager()
    tm.set_global_distance_to_leading_vehicle(2.5)
    car_bps = [b for b in bp.filter("vehicle.*") if int(b.get_attribute("number_of_wheels")) == 4]
    batch = []
    for i, sp in enumerate(spawn_points[:15]):
        b = car_bps[i % len(car_bps)]
        b.set_attribute("role_name", "autopilot")
        batch.append(carla.command.SpawnActor(b, sp).then(
            carla.command.SetAutopilot(carla.command.FutureActor, True, tm.get_port())))
    for r in c.apply_batch_sync(batch, False):
        if not r.error:
            vehicle_ids.append(r.actor_id)
    p.check("CARLA traffic spawned", len(vehicle_ids) >= 5, f"{len(vehicle_ids)} vehicles")

    # ---- pedestrians need an AI controller, or they just stand there ----
    walker_bps = bp.filter("walker.pedestrian.*")
    wbatch = []
    for i in range(15):
        loc = w.get_random_location_from_navigation()
        if loc is None:
            continue
        wbatch.append(carla.command.SpawnActor(walker_bps[i % len(walker_bps)], carla.Transform(loc)))
    for r in c.apply_batch_sync(wbatch, True):
        if not r.error:
            walker_ids.append(r.actor_id)
    ctrl_bp = bp.find("controller.ai.walker")
    for r in c.apply_batch_sync(
        [carla.command.SpawnActor(ctrl_bp, carla.Transform(), wid) for wid in walker_ids], True
    ):
        if not r.error:
            controller_ids.append(r.actor_id)
    for cid in controller_ids:
        ctrl = w.get_actor(cid)
        ctrl.start()
        ctrl.go_to_location(w.get_random_location_from_navigation())
    p.check("CARLA pedestrians spawned and walking", len(controller_ids) >= 3,
            f"{len(walker_ids)} walkers / {len(controller_ids)} controllers")

    time.sleep(4)

    # ---- re-derive the frame offset instead of trusting the doc ----
    ref_id = vehicle_ids[0]
    loc = w.get_actor(ref_id).get_location()
    p.metric("reference_vehicle_carla_xyz", [round(loc.x, 1), round(loc.y, 1), round(loc.z, 1)])
    target_ned = common.carla_to_airsim(loc)
    p.metric("same_point_in_airsim_ned", [round(v, 1) for v in target_ned])

    a.moveToPositionAsync(float(target_ned[0]), float(target_ned[1]), -60.0, 12.0).join()
    time.sleep(2)
    k = a.getMultirotorState().kinematics_estimated.position
    xy_err = ((k.x_val - target_ned[0]) ** 2 + (k.y_val - target_ned[1]) ** 2) ** 0.5
    p.check("drone reached the transformed CARLA point", xy_err < 3.0, f"{xy_err:.2f} m")

    # point the camera down
    a.simSetCameraPose("0", airsim.Pose(airsim.Vector3r(0.5, 0, 0.1), airsim.to_quaternion(-1.35, 0, 0)))
    time.sleep(2)

    def grab(tag):
        r = a.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
            airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False),
        ])
        rgb = np.frombuffer(r[0].image_data_uint8, dtype=np.uint8).reshape(r[0].height, r[0].width, 3)
        seg = np.frombuffer(r[1].image_data_uint8, dtype=np.uint8).reshape(r[1].height, r[1].width, 3)
        cv2.imwrite(f"{common.OUT}/p06_{tag}_rgb.png", rgb)
        cv2.imwrite(f"{common.OUT}/p06_{tag}_seg.png", seg)
        return rgb, seg

    clear_rgb, clear_seg = grab("clear")
    classes = len(np.unique(clear_seg.reshape(-1, 3), axis=0))
    p.metric("seg_classes_in_downward_view", classes)
    p.check("the view under the drone is a populated scene, not empty ground", classes >= 4,
            f"{classes} segmentation classes")

    # ---- does the ground traffic actually drive? ----
    before = {i: w.get_actor(i).get_location() for i in vehicle_ids}
    time.sleep(6)
    moved = []
    for i in vehicle_ids:
        af = w.get_actor(i).get_location()
        bf = before[i]
        moved.append(((af.x - bf.x) ** 2 + (af.y - bf.y) ** 2) ** 0.5)
    p.metric("vehicles_moved_gt_1m_in_6s", f"{sum(1 for m in moved if m > 1.0)}/{len(moved)}")
    p.metric("median_vehicle_distance_m_in_6s", round(float(np.median(moved)), 2))
    p.check("traffic manager is driving the vehicles",
            sum(1 for m in moved if m > 1.0) >= max(2, len(moved) // 3),
            f"{sum(1 for m in moved if m > 1.0)} of {len(moved)} moved >1 m")

    # ---- weather: one CARLA call, does the AirSim camera see it? ----
    w.set_weather(carla.WeatherParameters.HardRainSunset)
    time.sleep(5)
    rain_rgb, _ = grab("rain")
    delta = float(np.abs(clear_rgb.astype(np.int16) - rain_rgb.astype(np.int16)).mean())
    p.metric("mean_abs_pixel_delta_clear_vs_rain", round(delta, 2))
    p.metric("brightness_clear", round(float(clear_rgb.mean()), 1))
    p.metric("brightness_rain", round(float(rain_rgb.mean()), 1))
    p.check("CARLA weather changes the AirSim drone camera", delta > 5.0, f"delta {delta:.1f}")
    w.set_weather(carla.WeatherParameters.ClearNoon)

    p.note("frames written", f"{common.OUT}/p06_*.png")
finally:
    a.simSetCameraPose("0", airsim.Pose(airsim.Vector3r(0.5, 0, 0.1), airsim.to_quaternion(0, 0, 0)))
    common.land(a)
    # Destroy by id in one batch. Calling .destroy() on a handle CARLA has already
    # reaped throws a C++ std::runtime_error that escapes as `terminate` and
    # core-dumps the interpreter, so never hold live actor handles past the probe.
    for cid in controller_ids:
        try:
            w.get_actor(cid).stop()
        except Exception:
            pass
    try:
        c.apply_batch([carla.command.DestroyActor(i)
                       for i in controller_ids + walker_ids + vehicle_ids])
        time.sleep(1.5)
    except Exception as e:
        p.note("batch destroy raised", str(e))

p.finish()
