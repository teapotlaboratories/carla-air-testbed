#!/usr/bin/env python3
"""Fly the testbed entirely over ROS 2: take off, waypoint, attitude, land, and read every sensor.

    # terminal 1
    ./scripts/bringup.sh --config configs/testbed.yaml --backend geometric

    # terminal 2
    source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
    export ROS_DOMAIN_ID=42
    python3 examples/ros2_full_control.py

**This file imports nothing from the testbed.** No `sim_bridge`, no `carla`, no `airsim` — only
`rclpy` and `px4_msgs`. That restriction is the point: if a demo of the command surface needs
the 3.10 side, the surface is not actually closed, and a client written against a real Pixhawk
would not work here.

For the same reason every message below is a **PX4 message on a PX4 topic**. There is no
friendlier `/testbed/takeoff`, because inventing one would break the property the whole shim
exists for: moving to hardware means deleting `carla_air_bridge` and starting
`uxrce_dds_client`, not rewriting the client.

**It takes the aircraft off the autonomy loop first.** `bringup.sh` starts the full
VLM -> grounding -> control chain, and that controller streams its own setpoints at 10 Hz onto
the same `/fmu/in/trajectory_setpoint` this example publishes to. Left running, it wins: a
takeoff commanded to NED 35 m reached 15.6 m and the aircraft flew where the VLM wanted, with
no error anywhere to say why. So this disables `offboard_control` on entry and restores it on
exit, and says so in the log.

Run it with `--dry-run` to print the plan and exit without touching the aircraft.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from px4_msgs.msg import (
    SensorCombined,
    SensorGps,
    TrajectorySetpoint,
    VehicleAirData,
    VehicleAttitudeSetpoint,
    VehicleCommand,
    VehicleMagnetometer,
    VehicleOdometry,
)
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import PointCloud2

#: PX4's own QoS. Best-effort and transient-local on both sides — a RELIABLE subscription
#: receives NOTHING from a BEST_EFFORT publisher, silently, which is the single easiest way to
#: spend an afternoon on a topic that looks connected and never delivers.
PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

#: Town10HD: the AirSim NED origin sits 27.45 m above the street, so NED altitude is NOT
#: height above ground. Ground level is NED z = +27.45.
GROUND_NED_Z = 27.45


def euler_to_quaternion(roll, pitch, yaw):
    """Roll/pitch/yaw (radians, FRD) to the quaternion PX4 actually wants.

    `VehicleAttitudeSetpoint` carries `q_d`, not three Euler angles. Converting here rather
    than adding an Euler topic keeps the wire format identical to a real flight controller's.
    """
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy]


class FullControlDemo(Node):
    def __init__(self):
        super().__init__("ros2_full_control_demo")

        # ---------------- commanding ----------------
        self.pub_command = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", PX4_QOS)
        self.pub_setpoint = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", PX4_QOS)
        self.pub_attitude = self.create_publisher(
            VehicleAttitudeSetpoint, "/fmu/in/vehicle_attitude_setpoint", PX4_QOS)

        # ---------------- every sensor stream ----------------
        self.latest = {}
        self.counts = {}
        subs = [
            (VehicleOdometry, "/fmu/out/vehicle_odometry", "odometry"),
            (SensorCombined, "/fmu/out/sensor_combined", "imu"),
            (VehicleAirData, "/fmu/out/vehicle_air_data", "baro"),
            (VehicleMagnetometer, "/fmu/out/vehicle_magnetometer", "mag"),
            (SensorGps, "/fmu/out/sensor_gps", "gps"),
        ]
        for msg_type, topic, key in subs:
            self.create_subscription(
                msg_type, topic, self._store(key), PX4_QOS)
        # The lidar is a sensor_msgs type on a normal QoS profile, not a PX4 one — it comes
        # from CARLA rather than from the flight-controller shim.
        self.create_subscription(PointCloud2, "/sensors/lidar/points", self._store("lidar"), 1)

    def set_autonomy(self, enabled: bool) -> bool:
        """Enable or disable the offboard controller, IF somebody started one.

        Two publishers on one setpoint topic is not a conflict either of them can detect —
        the bridge simply forwards whatever arrived last, and at 10 Hz against this example's
        occasional commands the autonomy loop wins every time.
        """
        # `offboard_control` lives in examples/navigation and is NOT started by bringup.sh
        # since 2026-08-04, so "not running" is the ordinary case and must not read as a
        # fault. A short probe, because waiting 15 s for a node nobody launched is 15 s
        # added to every run of this example. The warning that matters is the opposite one:
        # if it IS running, it streams setpoints at 10 Hz and will win every conflict.
        client = self.create_client(SetParameters, "/offboard_control/set_parameters")
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "offboard_control is not running - nothing to disable, which is the normal "
                "case after bringup.sh (it starts the simulator only)")
            return False
        request = SetParameters.Request()
        request.parameters = [Parameter(
            name="enabled",
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enabled))]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        ok = bool(future.result() and future.result().results[0].successful)
        self.get_logger().info(
            f"autonomy controller {'enabled' if enabled else 'DISABLED for this demo'}"
            + ("" if ok else " (failed)"))
        return ok

    def _store(self, key):
        def cb(msg):
            self.latest[key] = msg
            self.counts[key] = self.counts.get(key, 0) + 1
        return cb

    # ------------------------------------------------------------------ helpers

    def _stamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def spin_for(self, seconds):
        """Pump callbacks for a while. Sensors keep arriving; nothing is missed."""
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, key, timeout=20.0):
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if key in self.latest:
                return True
        return False

    def altitude(self):
        odom = self.latest.get("odometry")
        return None if odom is None else -float(odom.position[2])

    # ------------------------------------------------------------------ 1. takeoff / land

    def send_command(self, command, **params):
        msg = VehicleCommand()
        msg.timestamp = self._stamp()
        msg.command = command
        for i in range(1, 8):
            setattr(msg, f"param{i}", float(params.get(f"param{i}", 0.0)))
        # A real PX4 filters on these; the bridge ignores them, but sending the right values
        # means the same client works against hardware without edits.
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_command.publish(msg)

    def takeoff(self, altitude_m=30.0):
        """VEHICLE_CMD_NAV_TAKEOFF. param7 is altitude, exactly as MAVLink defines it."""
        self.get_logger().info(f"takeoff -> {altitude_m:.0f} m (NED)")
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param7=altitude_m)

    def land(self):
        self.get_logger().info("land")
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # ------------------------------------------------------------------ 2. waypoint

    def waypoint(self, north, east, down, yaw=None):
        """A position setpoint. Velocity NaN is PX4's way of saying "ignore velocity".

        Leaving the unused fields at NaN rather than 0.0 matters: zeros are a *command* to
        hold still, and a controller that reads them as one will refuse to move.
        """
        msg = TrajectorySetpoint()
        msg.timestamp = self._stamp()
        msg.position = [float(north), float(east), float(down)]
        msg.velocity = [math.nan] * 3
        msg.acceleration = [math.nan] * 3
        msg.jerk = [math.nan] * 3
        msg.yaw = float(yaw) if yaw is not None else math.nan
        msg.yawspeed = math.nan
        self.pub_setpoint.publish(msg)
        self.get_logger().info(
            f"waypoint -> N {north:.1f}  E {east:.1f}  NED z {down:.1f} "
            f"({GROUND_NED_Z - down:.1f} m AGL)")

    def velocity(self, north, east, down, yaw=None):
        """A velocity setpoint, on the SAME topic as the waypoint.

        **Which one you get is decided by which fields are finite**, and velocity wins: the
        bridge checks velocity first, so a message with both set is a velocity command and the
        position is ignored. That is PX4's own convention, not ours.

        Streamed, not sent once — the sidecar gives each velocity command a lifetime
        (`setpoint_duration_s`, 0.5 s by default) and the aircraft stops when it lapses. That
        is deliberate: a link that dies mid-command leaves an aircraft that coasts to a halt
        rather than one that keeps going.
        """
        msg = TrajectorySetpoint()
        msg.timestamp = self._stamp()
        msg.position = [math.nan] * 3
        msg.velocity = [float(north), float(east), float(down)]
        msg.acceleration = [math.nan] * 3
        msg.jerk = [math.nan] * 3
        msg.yaw = float(yaw) if yaw is not None else math.nan
        msg.yawspeed = math.nan
        self.pub_setpoint.publish(msg)

    # ------------------------------------------------------------------ 3. attitude

    def attitude(self, roll_deg, pitch_deg, yaw_deg):
        """Roll/pitch/yaw as PX4 carries it: a quaternion in `q_d`.

        Altitude is not in this message — PX4 expresses the vertical axis as `thrust_body`,
        and the bridge holds the current altitude rather than guessing a mapping from
        normalised thrust onto AirSim's Z controller.
        """
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = self._stamp()
        msg.q_d = euler_to_quaternion(math.radians(roll_deg), math.radians(pitch_deg),
                                      math.radians(yaw_deg))
        msg.yaw_sp_move_rate = 0.0
        msg.thrust_body = [0.0, 0.0, -0.5]
        self.pub_attitude.publish(msg)
        self.get_logger().info(
            f"attitude -> roll {roll_deg:+.0f} deg  pitch {pitch_deg:+.0f} deg  yaw {yaw_deg:+.0f} deg")

    # ------------------------------------------------------------------ 4. sensors

    def print_sensors(self):
        print("\n" + "=" * 74)
        print("  SENSORS  (everything below arrived over ROS 2, nothing was polled)")
        print("=" * 74)

        odom = self.latest.get("odometry")
        if odom:
            p, v = odom.position, odom.velocity
            print(f"  odometry     NED ({p[0]:+8.1f}, {p[1]:+8.1f}, {p[2]:+8.1f}) m")
            print(f"               vel ({v[0]:+8.2f}, {v[1]:+8.2f}, {v[2]:+8.2f}) m/s")
            print(f"               altitude {-p[2]:.1f} m NED = {GROUND_NED_Z - p[2]:.1f} m AGL")

        imu = self.latest.get("imu")
        if imu:
            a, g = imu.accelerometer_m_s2, imu.gyro_rad
            print(f"  imu          accel ({a[0]:+7.3f}, {a[1]:+7.3f}, {a[2]:+7.3f}) m/s2")
            print(f"               gyro  ({g[0]:+7.3f}, {g[1]:+7.3f}, {g[2]:+7.3f}) rad/s")

        baro = self.latest.get("baro")
        if baro:
            print(f"  barometer    {baro.baro_alt_meter:8.2f} m   {baro.baro_pressure_pa:9.1f} Pa"
                  f"   {baro.ambient_temperature:5.1f} C   rho {baro.rho:.4f}")

        mag = self.latest.get("mag")
        if mag:
            m = mag.magnetometer_ga
            print(f"  magnetometer ({m[0]:+7.4f}, {m[1]:+7.4f}, {m[2]:+7.4f}) Gauss")

        gps = self.latest.get("gps")
        if gps:
            print(f"  gps          {gps.latitude_deg:.6f}, {gps.longitude_deg:.6f}"
                  f"   alt {gps.altitude_msl_m:.1f} m")
            print(f"               fix {gps.fix_type}   eph {gps.eph:.2f}  epv {gps.epv:.2f}"
                  f"   sats {gps.satellites_used}")

        lidar = self.latest.get("lidar")
        if lidar:
            print(f"  lidar        {lidar.width} points   {lidar.point_step} B/point"
                  f"   frame '{lidar.header.frame_id}'")
            print(f"               fields: {', '.join(f.name for f in lidar.fields)}")

        missing = {"odometry", "imu", "baro", "mag", "gps", "lidar"} - set(self.latest)
        if missing:
            print(f"\n  NOT RECEIVED: {', '.join(sorted(missing))}")
            print("  (lidar needs it enabled in configs/sim/carla_sensors.yaml)")
        print(f"\n  messages received: "
              + "  ".join(f"{k}={v}" for k, v in sorted(self.counts.items())))
        print("=" * 74 + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--altitude", type=float, default=30.0,
                    help="takeoff altitude, metres NED (default 30 = 57 m AGL)")
    ap.add_argument("--dry-run", action="store_true", help="print the plan and exit")
    args = ap.parse_args()

    plan = [
        "1. takeoff        VehicleCommand VEHICLE_CMD_NAV_TAKEOFF -> /fmu/in/vehicle_command",
        "2. waypoint       TrajectorySetpoint (position, velocity NaN) -> /fmu/in/trajectory_setpoint",
        "3. velocity       TrajectorySetpoint (velocity, position NaN) -> same topic; velocity wins",
        "4. attitude       VehicleAttitudeSetpoint (q_d) -> /fmu/in/vehicle_attitude_setpoint",
        "5. land           VehicleCommand VEHICLE_CMD_NAV_LAND -> /fmu/in/vehicle_command",
        "   sensors        odometry, imu, baro, mag, gps, lidar -- all subscribed throughout",
    ]
    if args.dry_run:
        print("\n".join(plan))
        return 0

    rclpy.init()
    node = FullControlDemo()
    try:
        node.get_logger().info("waiting for the graph...")
        if not node.wait_for("odometry", timeout=25.0):
            node.get_logger().error(
                "no odometry after 25 s. Is bringup.sh running, and is ROS_DOMAIN_ID=42?")
            return 1
        # Odometry runs at 20 Hz and everything else at 5, so arriving first proves only that
        # the graph is up. Give the slower streams a moment before the first snapshot, or it
        # prints a page of NOT RECEIVED that looks like a fault and is just impatience.
        node.spin_for(2.0)

        # Take the aircraft off the autonomy loop, or every command below is overridden.
        node.set_autonomy(False)

        node.print_sensors()

        # 1 ----------------------------------------------------------- takeoff
        node.takeoff(args.altitude)
        node.spin_for(20.0)
        node.get_logger().info(f"altitude now {node.altitude():.1f} m NED")

        # 2 ----------------------------------------------------------- waypoint
        odom = node.latest["odometry"]
        node.waypoint(odom.position[0] + 40.0, odom.position[1], -args.altitude)
        node.spin_for(18.0)

        # 3 ----------------------------------------------------------- velocity
        before = node.latest["odometry"].position[0]
        node.get_logger().info("streaming a 5 m/s northward velocity for 4 s")
        for _ in range(40):
            node.velocity(5.0, 0.0, 0.0)
            node.spin_for(0.1)
        node.spin_for(1.5)
        moved = node.latest["odometry"].position[0] - before
        node.get_logger().info(f"velocity command moved it {moved:+.1f} m north")

        # 4 ----------------------------------------------------------- attitude
        # Streamed, not sent once: an attitude command is a setpoint, and AirSim replaces the
        # active command each call. One message would be obeyed for a fraction of a second.
        node.get_logger().info("holding a 10 deg pitch-forward attitude for 5 s")
        for _ in range(50):
            node.attitude(roll_deg=0.0, pitch_deg=10.0, yaw_deg=45.0)
            node.spin_for(0.1)

        node.print_sensors()

        # 5 ----------------------------------------------------------- land
        node.land()
        node.spin_for(25.0)
        node.get_logger().info(f"final altitude {node.altitude():.1f} m NED")
        node.print_sensors()
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        # Hand the aircraft back, so the graph is left as it was found.
        try:
            node.set_autonomy(True)
        except Exception:  # noqa: BLE001 - shutting down anyway
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
