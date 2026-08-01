"""The drone: arming, setpoints, telemetry — with CARLA-Air's quirks handled once.

Two behaviours measured by the conformance suite are wrapped here so no caller has to
remember them:

* **After `reset()` the vehicle does not hold station.** It runs away — a constant
  +7.06 m/s climb that never stops in one configuration (a session reached -1566 m NED
  before anyone noticed), a slow sink in another. The direction varies between sessions;
  that it does not hold does not. `reset()` below therefore always issues a position
  setpoint before returning, and refuses to return until the vehicle is tracking it.
* **`moveToPositionAsync().join()` returns at the target, then the vehicle relaxes
  ~4 m away** and holds there. Waypoint accuracy is metres, not centimetres. Nothing here
  hides that — but `goto()` reports the *measured* pose, never the commanded one, so an
  episode log cannot silently record a position the aircraft was not at.
"""
from __future__ import annotations

import math
import time

import airsim

from . import frames


class Vehicle:
    def __init__(self, client: airsim.MultirotorClient, name: str = "SimpleFlight"):
        self._c = client
        self._name = name

    # ---------- lifecycle ----------

    def reset(self, hold_ned=(0.0, 0.0, -40.0), speed=8.0, settle_s=2.0):
        """Reset to the start pose and leave the aircraft *holding a setpoint*.

        Never return a vehicle that has only been armed: see the module docstring.
        """
        self._c.reset()
        time.sleep(3.0)
        self._c.enableApiControl(True, self._name)
        self._c.armDisarm(True, self._name)
        self._c.moveToPositionAsync(*hold_ned, speed, vehicle_name=self._name).join()
        time.sleep(settle_s)
        return self.state()

    def land(self):
        try:
            self._c.landAsync(vehicle_name=self._name).join()
        finally:
            self._c.armDisarm(False, self._name)
            self._c.enableApiControl(False, self._name)

    # ---------- telemetry ----------

    def state(self):
        s = self._c.getMultirotorState(vehicle_name=self._name)
        k = s.kinematics_estimated
        q = k.orientation
        return {
            "t": time.time(),
            "position": [k.position.x_val, k.position.y_val, k.position.z_val],
            "velocity": [k.linear_velocity.x_val, k.linear_velocity.y_val, k.linear_velocity.z_val],
            "angular_velocity": [k.angular_velocity.x_val, k.angular_velocity.y_val,
                                 k.angular_velocity.z_val],
            "orientation": [q.w_val, q.x_val, q.y_val, q.z_val],
            "yaw": frames.quat_to_yaw(q.w_val, q.x_val, q.y_val, q.z_val),
            "landed": int(s.landed_state),
            "armed": True,
        }

    def collision(self):
        c = self._c.simGetCollisionInfo(vehicle_name=self._name)
        return {
            "has_collided": bool(c.has_collided),
            "object_name": c.object_name,
            "position": [c.position.x_val, c.position.y_val, c.position.z_val],
            "object_id": int(c.object_id),
            "penetration_m": float(c.penetration_depth),
            "time_stamp": int(c.time_stamp),
        }

    # ---------- control ----------

    def velocity(self, vx: float, vy: float, vz: float, duration: float = 0.5, yaw_deg=None):
        """One velocity setpoint, NED, m/s.

        `duration` must exceed the caller's resend period or the vehicle stalls between
        commands. The control node streams at 10 Hz with duration 0.5 s.
        """
        kw = {"vehicle_name": self._name}
        if yaw_deg is not None:
            kw["yaw_mode"] = airsim.YawMode(False, yaw_deg)
        self._c.moveByVelocityAsync(float(vx), float(vy), float(vz), float(duration), **kw)

    def goto(self, x: float, y: float, z: float, speed: float = 6.0, settle_s: float = 0.0):
        """Blocking position move. Returns the *measured* pose, not the commanded one."""
        self._c.moveToPositionAsync(float(x), float(y), float(z), float(speed),
                                    vehicle_name=self._name).join()
        if settle_s:
            time.sleep(settle_s)
        return self.state()

    def yaw(self, deg: float, timeout_s: float = 10.0):
        self._c.rotateToYawAsync(float(deg), timeout_sec=timeout_s, vehicle_name=self._name).join()
        return self.state()

    def hold(self):
        """Stop. Distinct from 'send nothing', which is what makes it run away."""
        s = self.state()
        p = s["position"]
        self._c.moveToPositionAsync(p[0], p[1], p[2], 1.0, vehicle_name=self._name)
        return s

    # ---------- helpers ----------

    def distance_to(self, ned) -> float:
        p = self.state()["position"]
        return math.dist(p, list(ned))
