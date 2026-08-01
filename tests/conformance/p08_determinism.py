#!/usr/bin/env python3
"""P08 — is a VLM benchmark run repeatable here?

AerialVLN/OpenFly-style evaluation reports a success rate over N seeded episodes.
That only means anything if the same commands produce the same trajectory. CARLA
has synchronous mode with a fixed delta; AirSim has its own clock and lockstep
settings. In CARLA-Air they share one process, and nobody upstream documents what
happens when you tick one and fly the other.

So: run the same open-loop command sequence twice from a reset and compare the
trajectories, in async mode and again with CARLA in synchronous mode. Also record
whether enabling CARLA sync mode stalls the AirSim vehicle — which is the failure
that would make seeded evaluation impossible.
"""
import time

import numpy as np

import carla
import common

p = common.Probe("p08_determinism")
c, w = common.carla_world()
a = common.airsim_client()

CMDS = [(5.0, 0.0, -1.0), (0.0, 5.0, 0.0), (-4.0, 0.0, 1.0), (0.0, -4.0, 0.0)]


def run_episode():
    a.reset()
    time.sleep(3)
    a.enableApiControl(True)
    a.armDisarm(True)
    a.moveToPositionAsync(0, 0, -30, 8).join()   # setpoint first — see p09
    time.sleep(1.5)
    traj = []
    for vx, vy, vz in CMDS:
        t0 = time.time()
        while time.time() - t0 < 3.0:
            a.moveByVelocityAsync(vx, vy, vz, 0.5)
            k = a.getMultirotorState().kinematics_estimated.position
            traj.append((k.x_val, k.y_val, k.z_val))
            time.sleep(0.1)
    a.moveByVelocityAsync(0, 0, 0, 1.0).join()
    return np.array(traj)


def compare(t1, t2, label):
    n = min(len(t1), len(t2))
    d = np.linalg.norm(t1[:n] - t2[:n], axis=1)
    p.metric(f"{label}_endpoint_divergence_m", round(float(np.linalg.norm(t1[-1] - t2[-1])), 3))
    p.metric(f"{label}_mean_trajectory_divergence_m", round(float(d.mean()), 3))
    p.metric(f"{label}_max_trajectory_divergence_m", round(float(d.max()), 3))
    return float(d.max())


settings = w.get_settings()
p.metric("initial_sync_mode", settings.synchronous_mode)

# ---- async (as shipped) ----
t1 = run_episode()
t2 = run_episode()
max_async = compare(t1, t2, "async")
p.check("async: repeated identical command sequence stays within 1 m",
        max_async < 1.0, f"max divergence {max_async:.2f} m")

# ---- CARLA synchronous mode ----
try:
    s = w.get_settings()
    s.synchronous_mode = True
    s.fixed_delta_seconds = 0.05
    w.apply_settings(s)
    p.note("CARLA sync mode enabled", "fixed_delta_seconds=0.05")

    # does the AirSim vehicle still move when only CARLA is being ticked?
    a.reset()
    time.sleep(2)
    a.enableApiControl(True)
    a.armDisarm(True)
    start = a.getMultirotorState().kinematics_estimated.position
    a.moveByVelocityAsync(4.0, 0, 0, 3.0)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        w.tick()
        time.sleep(0.02)
    end = a.getMultirotorState().kinematics_estimated.position
    moved = ((end.x_val - start.x_val) ** 2 + (end.y_val - start.y_val) ** 2) ** 0.5
    p.metric("airsim_motion_under_carla_sync_m", round(moved, 2))
    p.check("AirSim vehicle still flies while CARLA is tick-driven", moved > 2.0,
            f"{moved:.2f} m in 3 s of ticks")
finally:
    s = w.get_settings()
    s.synchronous_mode = False
    s.fixed_delta_seconds = None
    w.apply_settings(s)
    p.note("CARLA sync mode restored to async")

common.land(a)
p.finish()
