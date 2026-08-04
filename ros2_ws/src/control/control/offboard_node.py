#!/usr/bin/env python3
"""The fast loop: hold the last grounded waypoint and stream setpoints at 10 Hz.

See-Point-Fly's shape is a slow generator over a fast controller — the VLM answers well
under 1 Hz while something underneath keeps the aircraft flying. This node is that
something, and it is the only place that publishes `/fmu/in/trajectory_setpoint`.

Three behaviours are load-bearing:

* **It never stops sending.** A gap with no setpoint is exactly the condition under which
  the simulator's vehicle runs away — a constant 7 m/s climb in one configuration, a sink
  in another. So the loop publishes every tick regardless of whether the VLM has spoken,
  falling back to a zero-velocity hold.
* **A bounded step, not the full ray.** The grounded point lies on a *surface* — that is
  what depth means. Commanding the whole displacement is commanding a controlled flight
  into a building; the conformance suite did that once and hit
  `BP_Block13NY_Top_C_1024`. Each step is capped, and the node re-observes.
* **`duration` exceeds the resend period.** `moveByVelocityAsync` with a duration shorter
  than the interval between commands leaves the vehicle unactuated between them.

`OffboardControlMode` is republished every tick because that is what PX4 requires to stay
in offboard, and matching the real requirement here is the point of a PX4-shaped graph.
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from interfaces.msg import GroundedWaypoint
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)
NAN = float("nan")


class OffboardController(Node):
    def __init__(self):
        super().__init__("offboard_control")

        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("setpoint_duration_s", 0.5)
        self.declare_parameter("max_speed_mps", 5.0)
        # Slew rate on the commanded velocity, m/s per second. 0 disables it.
        #
        # Without this the velocity is recomputed from scratch every tick and jumps the
        # instant a new waypoint arrives - which for a VLM is a fresh direction every ~4 s,
        # so the aircraft snaps onto each new heading at full speed. That reads as a series
        # of lurches in the chase footage and, at street level, is what puts a wall inside
        # the stopping distance.
        #
        # 2.5 m/s^2 reaches the 5 m/s cap in two seconds: brisk, but a corner taken over two
        # seconds instead of one tick.
        self.declare_parameter("max_accel_mps2", 2.5)
        # Slew rate on the HEADING, degrees per second. 0 disables it.
        #
        # The heading is derived from the instantaneous velocity, and the velocity is
        # recomputed every tick at 10 Hz - so any wobble in the velocity becomes a wobble in
        # where the aircraft is pointing, ten times a second. On the chase camera that reads
        # as constant jitter, and it is worse than cosmetic: the drone camera is bolted to
        # the airframe, so the frame the model is shown swings with it.
        #
        # 22.5 deg/s: a right angle takes four seconds, which is roughly one VLM decision
        # cycle. 45 was tried first and still read as jittery on the chase camera - the
        # limit has to be slow relative to how often the heading is REVISED, not merely
        # slow in absolute terms.
        self.declare_parameter("max_yaw_rate_dps", 22.5)
        self._last_v = None
        self._last_v_t = 0.0
        self._last_yaw = None
        self._last_yaw_t = 0.0
        self.declare_parameter("max_step_m", 20.0)
        self.declare_parameter("standoff_m", 8.0)
        self.declare_parameter("arrival_radius_m", 2.0)
        self.declare_parameter("min_altitude_m", 15.0)
        self.declare_parameter("max_altitude_m", 120.0)
        self.declare_parameter("enabled", True)

        self._odom: VehicleOdometry | None = None
        self._target: np.ndarray | None = None
        self._target_stamp = None
        self._arrived = False

        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", PX4_QOS)
        self.pub_mode = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", PX4_QOS)
        self.pub_arrived = self.create_publisher(Bool, "/control/arrived", 5)
        self.pub_target = self.create_publisher(Point, "/control/active_target", 5)

        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry",
                                 self._on_odom, PX4_QOS)
        self.create_subscription(GroundedWaypoint, "/control/waypoint",
                                 self._on_waypoint, 5)

        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"offboard controller streaming at {rate:.0f} Hz")

    # ------------------------------------------------------------------ inputs

    def _on_odom(self, msg: VehicleOdometry):
        self._odom = msg

    def _on_waypoint(self, wp: GroundedWaypoint):
        if not wp.valid:
            self.get_logger().info(f"ignoring invalid waypoint: {wp.reason}")
            return
        if self._odom is None:
            return

        here = np.array([float(c) for c in self._odom.position])
        goal = np.array([wp.position.x, wp.position.y, wp.position.z])
        ray = goal - here
        dist = float(np.linalg.norm(ray))
        if dist < 1e-3:
            return

        # Stop short of the surface, then cap the step. Both matter: standoff keeps us off
        # the building the pixel landed on, the cap keeps the aircraft re-observing.
        #
        # A bearing-only waypoint has no surface behind it — the range is a fixed guess, not
        # a measurement — so applying standoff there would shorten an already-arbitrary step
        # for no reason, and at close range could cancel it entirely and stall the loop.
        standoff = 0.0 if wp.bearing_only else float(self.get_parameter("standoff_m").value)
        reach = max(0.0, dist - standoff)
        step = min(reach, float(self.get_parameter("max_step_m").value))
        target = here + ray / dist * step

        lo = float(self.get_parameter("min_altitude_m").value)
        hi = float(self.get_parameter("max_altitude_m").value)
        target[2] = float(np.clip(target[2], -hi, -lo))   # NED: altitude is -z

        self._target = target
        self._target_stamp = self.get_clock().now()
        self._arrived = False
        self.pub_target.publish(Point(x=float(target[0]), y=float(target[1]), z=float(target[2])))
        self.get_logger().info(
            f"target {target.round(1).tolist()} — {step:.1f} m of a {dist:.1f} m ray "
            + ("(bearing only)" if wp.bearing_only else f"(depth {wp.depth:.1f} m)"))

    # -------------------------------------------------------------- the fast loop

    def _tick(self):
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        mode = OffboardControlMode()
        mode.timestamp = now_us
        mode.position = False
        mode.velocity = True
        self.pub_mode.publish(mode)

        if not bool(self.get_parameter("enabled").value):
            # Disabled means "hands off the aircraft" — drop the target too. An episode
            # reset flies the vehicle to its start with a blocking position command, and a
            # controller still streaming a stale waypoint fights it: measured, a reset to
            # (107.6, -159.4, -55.0) landed at (-14.2, -18.4, +5.1) because this loop kept
            # pulling toward the previous episode's target the whole way.
            if self._target is not None:
                self.get_logger().info("disabled — dropping the active target")
                self._target = None
            return
        if self._odom is None:
            return

        sp = TrajectorySetpoint()
        sp.timestamp = now_us
        sp.position = [NAN, NAN, NAN]
        sp.acceleration = [NAN, NAN, NAN]
        sp.jerk = [NAN, NAN, NAN]
        sp.yaw = NAN
        sp.yawspeed = NAN

        if self._target is None:
            # No waypoint yet — hold. Never send nothing: see the module docstring.
            self._last_v = None   # a hold restarts the ramp
            self._last_yaw = None
            sp.velocity = [0.0, 0.0, 0.0]
            self.pub_sp.publish(sp)
            return

        here = np.array([float(c) for c in self._odom.position])
        err = self._target - here
        dist = float(np.linalg.norm(err))

        if dist < float(self.get_parameter("arrival_radius_m").value):
            self._last_v = None   # a hold restarts the ramp
            self._last_yaw = None
            sp.velocity = [0.0, 0.0, 0.0]
            self.pub_sp.publish(sp)
            if not self._arrived:
                self._arrived = True
                self.pub_arrived.publish(Bool(data=True))
                self.get_logger().info(f"arrived within {dist:.2f} m")
            return

        speed = min(float(self.get_parameter("max_speed_mps").value), dist)
        v = err / dist * speed

        # Limit how fast the COMMAND may change, not how fast the aircraft may fly. The
        # target is what the VLM moved; the ramp is what stops the airframe being asked to
        # follow a step.
        accel = float(self.get_parameter("max_accel_mps2").value)
        now = time.monotonic()
        if accel > 0.0 and self._last_v is not None:
            dt = max(1e-3, min(0.5, now - self._last_v_t))
            dv = v - self._last_v
            mag = float(np.linalg.norm(dv))
            cap = accel * dt
            if mag > cap:
                v = self._last_v + dv * (cap / mag)
        self._last_v, self._last_v_t = v.copy(), now

        sp.velocity = [float(v[0]), float(v[1]), float(v[2])]

        # Face where we are going; a camera pointing off-axis annotates the wrong scene.
        if abs(v[0]) + abs(v[1]) > 0.5:
            want = float(math.atan2(v[1], v[0]))
            rate = math.radians(float(self.get_parameter("max_yaw_rate_dps").value))
            if rate > 0.0 and self._last_yaw is not None:
                dt = max(1e-3, min(0.5, now - self._last_yaw_t))
                # Shortest way round. Without the wrap a heading crossing +/-pi takes the
                # long way and the aircraft spins almost a full turn to face 1 degree away.
                delta = math.atan2(math.sin(want - self._last_yaw),
                                   math.cos(want - self._last_yaw))
                cap = rate * dt
                if abs(delta) > cap:
                    want = self._last_yaw + math.copysign(cap, delta)
                    want = math.atan2(math.sin(want), math.cos(want))
            self._last_yaw, self._last_yaw_t = want, now
            sp.yaw = want

        self.pub_sp.publish(sp)


def main(argv=None):
    rclpy.init(args=argv)
    node = OffboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
