#!/usr/bin/env python3
"""P10 — does `.join()` mean "arrived", and does it stay arrived?

Every episode script in this stack is written as
`moveToPositionAsync(...).join()` followed by "now observe", so the question is
where the vehicle actually is at that moment. P06 saw a 10.4 m miss checking
position two seconds after a join, which looked like join() returning early.

It is not. Measured over five legs, join() returns *at* the target — 0.26 to
0.64 m error. What happens next is the problem: over the following four seconds
the vehicle relaxes roughly 3-4 m away from the commanded point and then holds
station there. So the waypoint accuracy of this stack is about 4 m, not
sub-metre, and an observation taken "after the drone settles" is taken from a
different place than one taken the instant join() returns.

That is livable for AVLN-style evaluation (AerialVLN scores success at 20 m) but
it has to be a known quantity, and any episode that logs a pose must log the
measured pose rather than the commanded one.
"""
import time

import numpy as np

import common

p = common.Probe("p10_join_convergence")
a = common.airsim_client()
a.reset()
time.sleep(3)
a.enableApiControl(True)
a.armDisarm(True)
a.moveToPositionAsync(0, 0, -40, 8).join()   # setpoint first — see p09
time.sleep(2)

LEGS = [(60, 0, -40), (60, 60, -40), (0, 60, -60), (0, 0, -40), (120, 0, -50)]
SPEED = 10.0
errors, settled = [], []
for tx, ty, tz in LEGS:
    k = a.getMultirotorState().kinematics_estimated.position
    leg = float(np.linalg.norm(np.array([tx, ty, tz]) - np.array([k.x_val, k.y_val, k.z_val])))
    t0 = time.time()
    a.moveToPositionAsync(tx, ty, tz, SPEED).join()
    t_join = time.time() - t0
    k = a.getMultirotorState().kinematics_estimated.position
    at_join = float(np.linalg.norm(np.array([tx, ty, tz]) - np.array([k.x_val, k.y_val, k.z_val])))
    # how far out is it once it has actually stopped?
    time.sleep(4)
    k = a.getMultirotorState().kinematics_estimated.position
    after = float(np.linalg.norm(np.array([tx, ty, tz]) - np.array([k.x_val, k.y_val, k.z_val])))
    errors.append(at_join)
    settled.append(after)
    p.note(f"leg {leg:5.1f} m", f"join returned after {t_join:4.1f} s at {at_join:5.2f} m error; "
                                f"settled to {after:4.2f} m after 4 more s")

p.metric("error_at_join_max_m", round(float(np.max(errors)), 2))
p.metric("error_at_join_mean_m", round(float(np.mean(errors)), 2))
p.metric("error_after_4s_settle_max_m", round(float(np.max(settled)), 2))
p.metric("post_join_relaxation_mean_m", round(float(np.mean(np.array(settled) - np.array(errors))), 2))
p.check("join() returns with the vehicle at the target (<2 m)", max(errors) < 2.0,
        f"worst {max(errors):.2f} m")
p.check("post-join relaxation stays under 6 m", max(settled) < 6.0,
        f"worst {max(settled):.2f} m — waypoint accuracy floor, not a transient")

common.land(a)
p.finish()
