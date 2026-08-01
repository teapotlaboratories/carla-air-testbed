#!/usr/bin/env python3
"""P09 — does the vehicle hold station when nothing is commanding it?

This probe exists because of a failure the other probes tripped over. A VLM in
the loop is *slow*: See-Point-Fly's generator runs at well under 1 Hz, and every
gap between waypoints is a window with no active setpoint. A simulator whose
vehicle does not hold station in that window cannot host the loop at all — the
aircraft is somewhere else by the time the VLM answers.

Measured behaviour on CARLA-Air v0.1.7:

  * straight after `reset()`, armed and under API control, the vehicle climbs at
    a constant ~7 m/s and never stops — it reached -1566 m NED in one session
    before anyone noticed,
  * after any explicit position/velocity command completes, hover is rock solid
    (sub-0.3 m over 15 s),
  * grabbing images has nothing to do with it, which is worth stating because it
    looked exactly like render-thread contention until the A/B was run.

So the workaround is a one-liner — command a setpoint immediately after reset —
but it has to be in every episode-reset path, and nothing upstream says so.
"""
import time

import airsim
import common

p = common.Probe("p09_hover_hold")
a = common.airsim_client()


def climb_rate(seconds=9.0, grab=False):
    scene = [airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)]
    z0 = a.getMultirotorState().kinematics_estimated.position.z_val
    t0 = time.time()
    while time.time() - t0 < seconds:
        if grab:
            a.simGetImages(scene)
        else:
            time.sleep(0.25)
    z1 = a.getMultirotorState().kinematics_estimated.position.z_val
    return (z0 - z1) / (time.time() - t0), z1  # +ve = climbing


# ---- A: fresh reset, no command ever issued ----
a.reset()
time.sleep(3)
a.enableApiControl(True)
a.armDisarm(True)
rate_idle, z_idle = climb_rate()
p.metric("climb_rate_after_reset_no_command_mps", round(rate_idle, 2))
p.metric("z_after_9s_idle_m", round(z_idle, 1))
p.check("holds altitude after reset with no command", abs(rate_idle) < 0.5,
        f"{rate_idle:+.2f} m/s — uncommanded vertical runaway")

# ---- B: same, but hammering the image API (is it render contention?) ----
a.reset()
time.sleep(3)
a.enableApiControl(True)
a.armDisarm(True)
rate_grab, _ = climb_rate(grab=True)
p.metric("climb_rate_after_reset_while_grabbing_mps", round(rate_grab, 2))
p.check("image load is NOT the cause (A and B agree within 1 m/s)",
        abs(rate_grab - rate_idle) < 1.0,
        f"idle {rate_idle:+.2f} vs grabbing {rate_grab:+.2f} m/s")

# ---- C: after an explicit setpoint, does hover hold? ----
a.moveToPositionAsync(0, 0, -25, 8).join()
time.sleep(2)
k0 = a.getMultirotorState().kinematics_estimated.position
rate_cmd, _ = climb_rate(grab=True)
k1 = a.getMultirotorState().kinematics_estimated.position
drift = ((k1.x_val - k0.x_val) ** 2 + (k1.y_val - k0.y_val) ** 2 + (k1.z_val - k0.z_val) ** 2) ** 0.5
p.metric("climb_rate_after_setpoint_mps", round(rate_cmd, 2))
p.metric("total_drift_after_setpoint_m", round(drift, 2))
p.check("holds station once a setpoint has been issued", drift < 1.0, f"{drift:.2f} m over 9 s")

p.note("workaround", "issue a position setpoint immediately after every reset(); "
                     "never let an episode start rely on default hover")
common.land(a)
p.finish()
