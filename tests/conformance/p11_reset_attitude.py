#!/usr/bin/env python3
"""P11 — does `reset()` actually deliver the attitude it commands?

`Vehicle.reset()` places the aircraft with `airsim.Quaternionr(0, 0, 0, 1)` — yaw
zero — and then issues a `moveToPositionAsync` so the drone is left holding a
setpoint rather than falling. Traces taken on 2026-08-04 say the aircraft is not
where the reset claims to have put it: five supposedly identical episodes began
at 17.6, 20.9, 20.9, 25.7 and 27.8 degrees of yaw.

That is a repeatability defect on its own — a seeded run that starts from a
different attitude each time is not seeded — and it is the leading suspect for
D-01, where the one failing run of five had the lowest starting yaw and then
oscillated +/-20 degrees without ever converging.

This probe separates the two places the commanded attitude can be lost:

  * after `simSetVehiclePose`   — did the teleport take?
  * after `moveToPositionAsync` — did flying to the (already reached) hold pose
    turn the airframe?

If the first is clean and the second is not, the fix is in how the hold is
commanded. If the first is already dirty, the pose is not being applied at all
and the settle time is the thing to look at.

**Read the stage attribution with suspicion — it is not yet trustworthy.** Run in a
loop this probe reports the teleport as the losing stage (~-115 deg after
`simSetVehiclePose`), but the same call issued once from an idle aircraft was
measured delivering exactly 0.00 deg and holding it for two seconds. Something
about the preceding state changes the outcome and this probe does not yet isolate
it. What the probe DOES establish, repeatably, is the drift metric below.

Run with the simulator up and NOTHING else: this drives AirSim directly, so a
running offboard controller would fight it and make the numbers meaningless.

    ./scripts/run_sim.sh --config configs/testbed.yaml
    ./.venv/bin/python tests/conformance/p11_reset_attitude.py
"""
import math
import statistics
import time

import airsim
import common

N = 10
HOLD = (107.6, -159.4, -55.0)     # cross_the_plaza's start, where D-01 was measured
SPEED = 10.0
SETTLE_S = 2.0
#: A degree or two is the noise floor of a settling multirotor. Ten is a different
#: starting condition, and that is what the traces showed.
TOLERANCE_DEG = 3.0


NAME = "SimpleFlight"


def yaw_deg(client):
    q = client.simGetVehiclePose(vehicle_name=NAME).orientation
    w, x, y, z = q.w_val, q.x_val, q.y_val, q.z_val
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


p = common.Probe("p11_reset_attitude")
c = common.airsim_client()

after_pose, after_hold, after_settle = [], [], []

for i in range(N):
    # Mirrors Vehicle.reset() exactly — same order, same calls. A probe that resets
    # differently from the code under test measures the probe.
    c.cancelLastTask(vehicle_name=NAME)
    c.armDisarm(False, NAME)
    c.enableApiControl(False, NAME)
    # Long enough for the airframe to STOP before it is teleported. 0.3 s (what reset()
    # uses) leaves it still rotating from the previous command, and a pose applied to a
    # spinning body was measured not to take at all.
    time.sleep(1.5)

    c.simSetVehiclePose(
        airsim.Pose(airsim.Vector3r(*HOLD), airsim.Quaternionr(0, 0, 0, 1)), True,
        vehicle_name=NAME)
    time.sleep(0.5)
    after_pose.append(yaw_deg(c))

    c.enableApiControl(True, NAME)
    c.armDisarm(True, NAME)
    c.moveToPositionAsync(HOLD[0], HOLD[1], HOLD[2], SPEED, vehicle_name=NAME).join()
    after_hold.append(yaw_deg(c))

    time.sleep(SETTLE_S)
    after_settle.append(yaw_deg(c))

    print(f"  [{i + 1:2d}/{N}] after pose {after_pose[-1]:7.2f}   "
          f"after hold {after_hold[-1]:7.2f}   settled {after_settle[-1]:7.2f}")


def report(label, series):
    spread = max(series) - min(series)
    p.metric(f"{label}_spread_deg", round(spread, 2))
    p.metric(f"{label}_worst_abs_deg", round(max(abs(v) for v in series), 2))
    p.metric(f"{label}_stdev_deg", round(statistics.pstdev(series), 2))
    return spread


print()
sp_pose = report("after_pose", after_pose)
sp_hold = report("after_hold", after_hold)
sp_settle = report("after_settle", after_settle)

p.check("simSetVehiclePose delivers yaw 0",
        max(abs(v) for v in after_pose) <= TOLERANCE_DEG,
        f"worst |yaw| {max(abs(v) for v in after_pose):.2f} deg over {N} resets")
p.check("the hold does not turn the airframe",
        max(abs(v) for v in after_hold) <= TOLERANCE_DEG,
        f"worst |yaw| {max(abs(v) for v in after_hold):.2f} deg after moveToPositionAsync")
p.check("a settled reset is repeatable",
        sp_settle <= TOLERANCE_DEG,
        f"spread {sp_settle:.2f} deg across {N} identical resets")

# The headline: does anything HOLD the heading once the reset returns? An aircraft left free
# to rotate will present a different attitude to the camera every episode, whatever the
# teleport delivered.
drift = [abs(b - a) for a, b in zip(after_hold, after_settle)]
drift = [d if d <= 180 else 360 - d for d in drift]
p.metric("settle_drift_worst_deg", round(max(drift), 2))
p.metric("settle_drift_median_deg", round(statistics.median(drift), 2))
p.check(f"heading is held through the {SETTLE_S}s settle",
        max(drift) <= TOLERANCE_DEG,
        f"worst drift {max(drift):.2f} deg, median {statistics.median(drift):.2f} deg")

# Which stage lost it — stated rather than left to be inferred from three numbers.
if sp_pose > TOLERANCE_DEG:
    p.note("the TELEPORT is where attitude is lost", "simSetVehiclePose did not take")
elif sp_hold > TOLERANCE_DEG:
    p.note("the HOLD is where attitude is lost",
           "the pose was correct until moveToPositionAsync ran")
elif sp_settle > TOLERANCE_DEG:
    p.note("attitude drifts during the SETTLE",
           f"clean after the hold, {sp_settle:.2f} deg apart {SETTLE_S}s later")
else:
    p.note("all three stages within tolerance",
           "if traces still disagree, the drift is after reset returns")

p.finish()
