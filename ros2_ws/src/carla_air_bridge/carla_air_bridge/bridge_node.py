#!/usr/bin/env python3
"""Turn the CARLA-Air simulator into a PX4-shaped ROS 2 graph.

This node is the **only** place that knows the simulator exists. Everything downstream —
the VLM client, the grounding node, the controller, the episode runner — talks to
`/fmu/out/*` and `/fmu/in/*` exactly as it would against a real Pixhawk 6C. Moving to
hardware means deleting this node and starting `uxrce_dds_client`, not rewriting the stack.

**This is deliberately a shim, and the shim is not free.** CARLA-Air has no PX4 in it at
all — no MAVLink, no uORB, no lockstep; the flight controller is AirSim's SimpleFlight. So
the PX4 semantics below are *emulated*, and three of them are worth knowing before trusting
a number that came out of here:

* `vehicle_odometry.timestamp` is host time, not a PX4 boot clock synced over
  `timesync_status`. Latency figures across this bridge measure the bridge, not a flight
  controller.
* `vehicle_status.arming_state` follows AirSim's API-control flag. There is no failsafe
  state machine behind it, so nothing here exercises PX4's arming logic.
* `TrajectorySetpoint` velocity is forwarded to `moveByVelocityAsync`. Position setpoints
  are honoured via `moveToPosition`; acceleration and jerk fields are ignored, because
  SimpleFlight has nowhere to put them.

**Three connections to the sidecar, not one.** Image capture takes ~500 ms; telemetry takes
~5 ms. Sharing one connection puts every odometry read behind the current capture and the
measured `/fmu/out/vehicle_odometry` rate collapses from 20 Hz to 1.5 Hz — which starves the
10 Hz controller of fresh position and makes it steer on stale data. Splitting control off
as well was worth another step: sharing one AirSim client between 20 Hz of telemetry and
10 Hz of setpoints capped odometry at 12.3 Hz. So `self.sim` is telemetry and world,
`self.media` is image capture, `self.ctrl` is setpoint forwarding, and the sidecar backs
each with its own AirSim client.

The odometry rate is what the sim can actually serve, not PX4's 100 Hz.
"""
from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleAttitudeSetpoint,
    VehicleCommand,
    SensorCombined,
    SensorGps,
    TrajectorySetpoint,
    VehicleAirData,
    VehicleLocalPosition,
    VehicleMagnetometer,
    VehicleOdometry,
    VehicleStatus,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Bool, String
from interfaces.msg import Collision
from interfaces.srv import (ChaseRecording, DestroyActors, ResetVehicle, SetCameraPose,
                            SetWeather, SpawnTraffic)

from .client import SimBridgeClient, SimBridgeError

# PX4's uXRCE-DDS bridge publishes best-effort with a small keep-last depth, and real
# subscribers are written against that. Matching it here means a node that works in this
# testbed does not silently fail on hardware because of a QoS mismatch.
PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


def _advance(deadline: float, now: float, period: float) -> float:
    """Next deadline, anchored to the schedule rather than to `now`.

    Re-anchoring on `now` looks equivalent and is not: the timer fires a little late, the
    next deadline is pushed out by that lateness, and the error compounds. Measured, a
    20 Hz timer guarded that way settled at 11.8 Hz. Anchoring on the previous deadline
    holds the cadence; falling more than one period behind (a long blocking call) resyncs
    to now instead of replaying the backlog as a burst.
    """
    return deadline + period if (now - deadline) < period else now + period


class CarlaAirBridge(Node):
    def __init__(self):
        super().__init__("carla_air_bridge")

        self.declare_parameter("socket_path", "/tmp/carla_air_testbed.sock")
        self.declare_parameter("odometry_rate_hz", 20.0)
        # AirSim creates IMU, barometer, magnetometer and GPS automatically for a multirotor,
        # so publishing them needs no simulator config — but it is NOT free. Measured against
        # /camera/rgb/image_raw, which is this project's bottleneck:
        #
        #     off     baseline        5 Hz    -1.7%
        #     10 Hz   -14%           20 Hz    -28%
        #
        # The cost is RPC round trips on a single-threaded Python sidecar, not the sensors
        # themselves. 5 Hz is the default because it is nearly free and is already faster than
        # a real GPS; raise it only if something actually consumes IMU at rate. 0 disables.
        self.declare_parameter("sensor_rate_hz", 5.0)
        # CARLA semantic LiDAR, spawned from configs/sim/carla_sensors.yaml. Unlike the AirSim
        # sensors this does NOT ride the RPC image path — the sensor pushes asynchronously
        # inside UE4 and we only fetch the newest sweep. 0 disables the publisher.
        self.declare_parameter("lidar_rate_hz", 10.0)
        self.declare_parameter("image_rate_hz", 4.0)
        self.declare_parameter("publish_depth", True)
        self.declare_parameter("publish_segmentation", False)
        self.declare_parameter("frame_id", "drone")
        self.declare_parameter("setpoint_duration_s", 0.5)

        self._frame_id = self.get_parameter("frame_id").value
        self._cv = CvBridge()
        self._seq = 0
        self._last_camera = None
        self._offboard_active = False
        # rclpy timers accumulate missed firings and replay them back-to-back once a long
        # callback returns. The sidecar blocks for SECONDS on reset/goto/spawn_traffic, so
        # without a guard the odometry timer bursts afterwards — measured at 110 Hz against
        # a 20 Hz setting, 1107 unique timestamps in 10 s. That burst is not free telemetry:
        # every firing is a blocking RPC that competes with the image path for the sidecar.
        # Collapse a backlog to a single firing.
        # Newest NED altitude, kept for attitude commands: PX4 carries thrust rather
        # than a height, so an attitude setpoint holds wherever the aircraft already is.
        # Initialised here so a setpoint arriving before the first odometry hits the
        # None guard instead of raising AttributeError.
        self._last_z = None
        self._next_odom = 0.0
        # Set while a reset is in flight. AirSim's reset() tears the vehicle down and
        # rebuilds it, and every RPC arriving during that window competes with it: measured,
        # six back-to-back resets with this node's own polling running went 26.8 s -> 60.1 s
        # -> hung, with nothing else in the graph at all. The timers check this and skip.
        self._resetting = False
        self._next_image = 0.0


        socket_path = self.get_parameter("socket_path").value
        self.sim = SimBridgeClient(socket_path)      # telemetry + world
        self.media = SimBridgeClient(socket_path)    # image capture only
        self.ctrl = SimBridgeClient(socket_path)     # setpoint forwarding only
        # A fourth, for the world-control services. `reset` blocks for **16.2 s measured**
        # while the aircraft arms, flies to the pose and settles; on `self.sim` that would
        # stall odometry and the world tick for the whole of it. With this split, odometry
        # held 19.9 Hz against a 20 Hz target THROUGH that reset. Same reasoning as above.
        self.world = SimBridgeClient(socket_path)    # world-control services only
        try:
            self.sim.connect()
            self.media.connect()
            self.ctrl.connect()
            self.world.connect()
        except OSError as exc:
            self.get_logger().error(
                f"cannot reach sim_bridge at {socket_path}: {exc}. "
                "Start it with: ./.venv/bin/python sim_bridge/server.py")
            raise
        info = self.sim.describe()
        self.get_logger().info(
            f"bridged to CARLA-Air — map={info['map']} hfov={info['camera']['hfov_deg']:.1f}deg "
            f"server={info['carla_server_version']}")

        # ---- PX4-shaped outputs ----
        self.pub_odom = self.create_publisher(VehicleOdometry, "/fmu/out/vehicle_odometry", PX4_QOS)
        self.pub_local = self.create_publisher(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", PX4_QOS)
        self.pub_status = self.create_publisher(VehicleStatus, "/fmu/out/vehicle_status", PX4_QOS)
        self.pub_imu = self.create_publisher(SensorCombined, "/fmu/out/sensor_combined", PX4_QOS)
        self.pub_air = self.create_publisher(VehicleAirData, "/fmu/out/vehicle_air_data", PX4_QOS)
        self.pub_mag = self.create_publisher(
            VehicleMagnetometer, "/fmu/out/vehicle_magnetometer", PX4_QOS)
        self.pub_gps = self.create_publisher(SensorGps, "/fmu/out/sensor_gps", PX4_QOS)
        self.pub_lidar = self.create_publisher(PointCloud2, "/sensors/lidar/points", 1)

        # ---- sensors (ordinary ROS types; nothing PX4 about a camera) ----
        self.pub_rgb = self.create_publisher(Image, "/camera/rgb/image_raw", 5)
        self.pub_info = self.create_publisher(CameraInfo, "/camera/rgb/camera_info", 5)
        self.pub_depth = self.create_publisher(Image, "/camera/depth/image_raw", 5)
        self.pub_seg = self.create_publisher(Image, "/camera/segmentation/image_raw", 5)
        # The camera's world pose, straight from simGetCameraInfo. Publishing it makes the
        # bridge the single source of truth: before this, `grounding` and the oracle each
        # reconstructed it from odometry plus their own camera_pitch_deg parameter, so a
        # pitch change in one place silently put every waypoint somewhere else.
        self.pub_campose = self.create_publisher(PoseStamped, "/camera/pose", 5)

        # ---- testbed-side signals the sim can answer but a Pixhawk cannot ----
        # Collision, not Bool. The flag alone sends people looking through video for a
        # building the log already knew the name of.
        self.pub_collision = self.create_publisher(Collision, "/sim/collision", 5)
        self.pub_traffic = self.create_publisher(String, "/sim/traffic_stats", 5)

        # ---- PX4-shaped inputs ----
        setpoint_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self._on_setpoint, PX4_QOS,
            callback_group=setpoint_group)
        self.create_subscription(
            OffboardControlMode, "/fmu/in/offboard_control_mode", self._on_offboard, PX4_QOS,
            callback_group=setpoint_group)
        # The command surface a ROS-only client needs. These are PX4's own messages on PX4's
        # own topics, not friendlier inventions: a node written against them ports to a real
        # Pixhawk by deleting this bridge, which is the entire point of the shim.
        self.create_subscription(
            VehicleCommand, "/fmu/in/vehicle_command", self._on_command, PX4_QOS,
            callback_group=setpoint_group)
        self.create_subscription(
            VehicleAttitudeSetpoint, "/fmu/in/vehicle_attitude_setpoint",
            self._on_attitude, PX4_QOS, callback_group=setpoint_group)

        # ---- world control (see todo.md R-01) ----
        # Services, not topics, because each of these has a meaningful failure the caller
        # must see: an unknown weather preset, a map that refused half the spawn points, a
        # reset that could not reach its pose. A topic would drop those on the floor and a
        # scenario would score a number that meant nothing.
        #
        # NOT PX4 messages, deliberately. Nothing on a real Pixhawk teleports an airframe or
        # spawns pedestrians, so borrowing `VehicleCommand` here would imply a portability
        # these calls do not have. The rule stays clean: `/fmu/*` is what survives the move
        # to hardware, `/sim/*` is what does not.
        world_group = MutuallyExclusiveCallbackGroup()
        self.create_service(ResetVehicle, "/sim/reset_vehicle", self._srv_reset,
                            callback_group=world_group)
        self.create_service(SpawnTraffic, "/sim/spawn_traffic", self._srv_spawn_traffic,
                            callback_group=world_group)
        self.create_service(SetWeather, "/sim/set_weather", self._srv_set_weather,
                            callback_group=world_group)
        self.create_service(DestroyActors, "/sim/destroy_actors", self._srv_destroy_actors,
                            callback_group=world_group)
        self.create_service(SetCameraPose, "/sim/set_camera_pose", self._srv_set_camera_pose,
                            callback_group=world_group)
        self.create_service(ChaseRecording, "/sim/chase_recording", self._srv_chase_recording,
                            callback_group=world_group)

        # One callback group per channel. Without this the executor serialises the timers
        # in a single thread and the two connections buy nothing.
        odo_hz = float(self.get_parameter("odometry_rate_hz").value)
        img_hz = float(self.get_parameter("image_rate_hz").value)
        self.create_timer(1.0 / odo_hz, self._tick_odometry,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0 / img_hz, self._tick_images,
                          callback_group=MutuallyExclusiveCallbackGroup())
        sensor_hz = float(self.get_parameter("sensor_rate_hz").value)
        if sensor_hz > 0:
            self._next_sensor = time.monotonic()
            self.create_timer(1.0 / sensor_hz, self._tick_sensors,
                              callback_group=MutuallyExclusiveCallbackGroup())
        lidar_hz = float(self.get_parameter("lidar_rate_hz").value)
        if lidar_hz > 0:
            self._next_lidar = time.monotonic()
            self.create_timer(1.0 / lidar_hz, self._tick_lidar,
                              callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0, self._tick_world,
                          callback_group=MutuallyExclusiveCallbackGroup())

    # ------------------------------------------------------------------ outputs

    def _stamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _on_command(self, msg: VehicleCommand):
        """PX4 VehicleCommand -> sidecar. Takeoff, land and arm/disarm.

        Only the commands this simulator can honour are acted on; anything else is logged
        rather than silently dropped, because a client sending an unsupported command and
        seeing nothing happen has no way to tell that from a broken link.
        """
        try:
            if msg.command == VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF:
                # param7 is altitude in MAVLink, metres above the origin. NED z is DOWN, so a
                # positive requested altitude becomes a negative setpoint.
                alt = float(msg.param7) if msg.param7 else 30.0
                self.sim.takeoff(altitude_ned=-abs(alt))
                self.get_logger().info(f"takeoff to NED {-abs(alt):.1f}")
            elif msg.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
                self.sim.land()
                self.get_logger().info("landing")
            elif msg.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
                # SimpleFlight has no separate arm step: reset() and takeoff() both arm, and
                # land() disarms. Accepted and logged so a PX4-shaped client's startup
                # sequence runs unchanged rather than erroring on an unknown command.
                self.get_logger().info(
                    f"arm/disarm {msg.param1:.0f} acknowledged (implicit in this simulator)")
            else:
                self.get_logger().warn(
                    f"unsupported VehicleCommand {msg.command}", throttle_duration_sec=5.0)
        except (SimBridgeError, OSError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f"vehicle command {msg.command} failed: {exc}")

    def _on_attitude(self, msg: VehicleAttitudeSetpoint):
        """PX4 VehicleAttitudeSetpoint -> AirSim roll/pitch/yaw hold.

        PX4 carries the desired attitude as a quaternion `q_d`, not Euler angles, so it is
        converted here rather than asking callers to send something PX4 would not.

        Altitude comes from the last odometry rather than from the message: PX4 expresses the
        vertical axis as `thrust_body`, and mapping normalised thrust onto AirSim's Z
        controller would be a guess. Holding the current altitude is honest and is what an
        attitude command usually means on a multirotor.
        """
        if self._last_z is None:
            self.get_logger().warn("attitude setpoint before any odometry — ignoring",
                                   throttle_duration_sec=5.0)
            return
        w, x, y, z = (float(v) for v in msg.q_d)
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        try:
            self.sim.attitude(roll, pitch, yaw, self._last_z,
                              float(self.get_parameter("setpoint_duration_s").value))
        except (SimBridgeError, OSError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f"attitude failed: {exc}")

    def _tick_lidar(self):
        if self._resetting:
            return       # see _resetting: AirSim is rebuilding the vehicle
        """Republish the newest semantic LiDAR sweep as PointCloud2.

        The sidecar hands over CARLA's raw buffer rather than a decoded list: 24 bytes per
        detection, and at 120000 points/second a Python list of tuples would be roughly 40x
        the bytes across the socket for identical information.

        CARLA's points are in ITS frame relative to the sensor - x forward, y RIGHT, z UP -
        while everything else in this graph is NED (z DOWN). The two extra fields are kept as
        they are: `object_idx` is what makes "the same building as those other 4000 points"
        answerable, and dropping it would leave this no better than a depth image.
        """
        hz = float(self.get_parameter("lidar_rate_hz").value)
        if hz <= 0.0:
            return
        now = time.monotonic()
        if now < self._next_lidar:
            return
        self._next_lidar = _advance(self._next_lidar, now, 1.0 / hz)
        try:
            sweep = self.sim.lidar()
        except (SimBridgeError, OSError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(f"lidar() failed: {exc}", throttle_duration_sec=5.0)
            return
        if not sweep or not sweep.get("count"):
            return

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.height = 1
        msg.width = int(sweep["count"])
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = int(sweep["stride"])
        msg.row_step = msg.point_step * msg.width
        F = PointField
        msg.fields = [
            F(name="x", offset=0, datatype=F.FLOAT32, count=1),
            F(name="y", offset=4, datatype=F.FLOAT32, count=1),
            F(name="z", offset=8, datatype=F.FLOAT32, count=1),
            F(name="cos_inc_angle", offset=12, datatype=F.FLOAT32, count=1),
            F(name="object_idx", offset=16, datatype=F.UINT32, count=1),
            F(name="object_tag", offset=20, datatype=F.UINT32, count=1),
        ]
        msg.data = bytes(sweep["raw"])
        self.pub_lidar.publish(msg)

    def _tick_sensors(self):
        if self._resetting:
            return       # see _resetting: AirSim is rebuilding the vehicle
        """IMU, barometer, magnetometer and GPS, from one sidecar call.

        These are AirSim's simulated instruments, not a second view of ground truth — the
        noise models are active, which is what makes them worth publishing at all. Anything
        wanting truth should read `/fmu/out/vehicle_odometry`.
        """
        # Read the rate first: the parameter is settable at runtime, and 0 means "stop
        # publishing" rather than "divide by zero" — which is what the obvious version does
        # the moment someone turns these off on a live graph to test exactly that.
        hz = float(self.get_parameter("sensor_rate_hz").value)
        if hz <= 0.0:
            return
        now = time.monotonic()
        if now < self._next_sensor:
            return
        self._next_sensor = _advance(self._next_sensor, now, 1.0 / hz)
        try:
            s = self.sim.sensors()
        except (SimBridgeError, OSError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(f"sensors() failed: {exc}", throttle_duration_sec=5.0)
            return

        stamp = self._stamp_us()

        imu = SensorCombined()
        imu.timestamp = stamp
        # AirSim's body frame is already FRD, which is the frame SensorCombined documents,
        # so the vectors go straight across with no axis permutation.
        imu.gyro_rad = np.array(s["imu"]["gyro"], dtype=np.float32)
        imu.accelerometer_m_s2 = np.array(s["imu"]["accel"], dtype=np.float32)
        self.pub_imu.publish(imu)

        air = VehicleAirData()
        air.timestamp = air.timestamp_sample = stamp
        air.baro_alt_meter = float(s["baro"]["altitude"])
        air.baro_pressure_pa = float(s["baro"]["pressure"])
        # AirSim reports temperature in KELVIN; this field is Celsius.
        air.ambient_temperature = float(s["env"]["temperature_k"]) - 273.15
        air.rho = float(s["env"]["density"])
        self.pub_air.publish(air)

        mag = VehicleMagnetometer()
        mag.timestamp = mag.timestamp_sample = stamp
        # Gauss on both sides — no conversion.
        mag.magnetometer_ga = np.array(s["mag"]["field"], dtype=np.float32)
        self.pub_mag.publish(mag)

        g = s["gps"]
        gps = SensorGps()
        gps.timestamp = gps.timestamp_sample = stamp
        gps.latitude_deg = float(g["lat"])
        gps.longitude_deg = float(g["lon"])
        gps.altitude_msl_m = float(g["alt"])
        gps.altitude_ellipsoid_m = float(g["alt"])
        gps.fix_type = int(g["fix"])
        gps.eph = float(g["eph"])
        gps.epv = float(g["epv"])
        gps.vel_n_m_s, gps.vel_e_m_s, gps.vel_d_m_s = (float(v) for v in g["vel"])
        gps.vel_m_s = float(math.hypot(g["vel"][0], g["vel"][1]))
        gps.vel_ned_valid = bool(g["valid"])
        gps.satellites_used = 12 if g["fix"] >= 3 else 0
        self.pub_gps.publish(gps)

    def _tick_odometry(self):
        if self._resetting:
            return       # see _resetting: AirSim is rebuilding the vehicle
        now = time.monotonic()
        if now < self._next_odom:
            return
        self._next_odom = _advance(
            self._next_odom, now, 1.0 / float(self.get_parameter("odometry_rate_hz").value))
        try:
            s = self.sim.state()
        except (SimBridgeError, OSError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(f"state() failed: {exc}", throttle_duration_sec=5.0)
            return

        now = self._stamp_us()
        # Indexing is INSIDE the guarded block on purpose. A sidecar that answers with an
        # error-shaped or partial dict used to raise KeyError here, outside the except, and
        # rclpy does not catch callback exceptions — the executor propagated it and the whole
        # bridge node exited(1) mid-run. One bad reply must degrade a topic, not end the node.
        p, v, q = s["position"], s["velocity"], s["orientation"]
        self._last_z = float(p[2])

        odom = VehicleOdometry()
        odom.timestamp = now
        odom.timestamp_sample = now
        odom.pose_frame = VehicleOdometry.POSE_FRAME_NED
        odom.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        odom.position = np.array(p, dtype=np.float32)
        odom.q = np.array(q, dtype=np.float32)
        odom.velocity = np.array(v, dtype=np.float32)
        odom.angular_velocity = np.array(s["angular_velocity"], dtype=np.float32)
        self.pub_odom.publish(odom)

        loc = VehicleLocalPosition()
        loc.timestamp = now
        loc.timestamp_sample = now
        loc.xy_valid = loc.z_valid = loc.v_xy_valid = loc.v_z_valid = True
        loc.x, loc.y, loc.z = (float(c) for c in p)
        loc.vx, loc.vy, loc.vz = (float(c) for c in v)
        loc.heading = float(s["yaw"])
        self.pub_local.publish(loc)

        st = VehicleStatus()
        st.timestamp = now
        st.arming_state = (VehicleStatus.ARMING_STATE_ARMED if s["armed"]
                           else VehicleStatus.ARMING_STATE_DISARMED)
        st.nav_state = (VehicleStatus.NAVIGATION_STATE_OFFBOARD if self._offboard_active
                        else VehicleStatus.NAVIGATION_STATE_AUTO_LOITER)
        self.pub_status.publish(st)

    def _tick_images(self):
        if self._resetting:
            return       # see _resetting: AirSim is rebuilding the vehicle
        now = time.monotonic()
        if now < self._next_image:
            return
        self._next_image = _advance(
            self._next_image, now, 1.0 / float(self.get_parameter("image_rate_hz").value))
        want_depth = bool(self.get_parameter("publish_depth").value)
        want_seg = bool(self.get_parameter("publish_segmentation").value)
        try:
            cap = self.media.capture(rgb=True, depth=want_depth, segmentation=want_seg)
        except (SimBridgeError, OSError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(f"capture() failed: {exc}", throttle_duration_sec=5.0)
            return

        stamp = self.get_clock().now().to_msg()
        imgs = cap["images"]
        self._last_camera = cap["camera"]

        cam = cap["camera"]
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            float(c) for c in cam["position"])
        qw, qx, qy, qz = (float(c) for c in cam["orientation"])
        pose.pose.orientation.w, pose.pose.orientation.x = qw, qx
        pose.pose.orientation.y, pose.pose.orientation.z = qy, qz
        self.pub_campose.publish(pose)

        rgb = imgs.get("rgb")
        if rgb is not None:
            msg = self._cv.cv2_to_imgmsg(rgb, encoding="bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            self.pub_rgb.publish(msg)
            self.pub_info.publish(self._camera_info(rgb.shape, stamp))

        depth = imgs.get("depth")
        if depth is not None:
            # 32FC1 metres, PX4-irrelevant but the ROS convention. AirSim marks sky with
            # float16 max (65504.0); republish it as +inf so consumers can use isfinite().
            d = depth.astype(np.float32).copy()
            d[d >= 65504.0 * 0.99] = np.inf
            msg = self._cv.cv2_to_imgmsg(d, encoding="32FC1")
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            self.pub_depth.publish(msg)

        seg = imgs.get("segmentation")
        if seg is not None:
            msg = self._cv.cv2_to_imgmsg(seg, encoding="bgr8")
            msg.header.stamp = stamp
            msg.header.frame_id = self._frame_id
            self.pub_seg.publish(msg)

    def _camera_info(self, shape, stamp) -> CameraInfo:
        h, w = shape[0], shape[1]
        hfov = math.radians(self._last_camera["hfov_deg"])
        fx = (w / 2.0) / math.tan(hfov / 2.0)
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self._frame_id
        info.width, info.height = int(w), int(h)
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, w / 2.0, 0.0, fx, h / 2.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, w / 2.0, 0.0, 0.0, fx, h / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    # --------------------------------------------------------- world control services

    def _srv_reset(self, req, resp):
        """Teleport and hold. Blocking - 16.2 s measured for a ~60 m move, so this needs
        its own connection and its own executor thread or it stalls every timer."""
        self._resetting = True
        try:
            speed = float(req.speed) if req.speed > 0.0 else 8.0
            state = self.world.reset(hold_ned=list(req.hold_ned), speed=speed)
            resp.success = True
            resp.position_ned = [float(v) for v in state["position"]]
            self.get_logger().info(
                f"reset -> commanded {list(req.hold_ned)} settled {resp.position_ned}")
        except (SimBridgeError, OSError, ValueError) as exc:
            resp.success = False
            resp.message = str(exc)
            self.get_logger().error(f"reset failed: {exc}")
        finally:
            # In `finally`: a reset that fails must not leave the graph permanently mute.
            self._resetting = False
        return resp

    def _srv_spawn_traffic(self, req, resp):
        try:
            # An empty `near_ned` means map-wide, which is a different request from
            # "concentrate at the origin" - so the length is what decides, not the values.
            # Length is what decides, and it can only decide because near_ned is an
            # UNBOUNDED array - see the note in SpawnTraffic.srv.
            if len(req.near_ned) not in (0, 3):
                raise ValueError(
                    f"near_ned needs 0 values (map-wide) or 3 (a point), got {len(req.near_ned)}")
            near = list(req.near_ned) if len(req.near_ned) == 3 else None
            radius = float(req.radius_m) if req.radius_m > 0.0 else 70.0
            got = self.world.spawn_traffic(vehicles=int(req.vehicles),
                                           walkers=int(req.walkers),
                                           near_ned=near, radius_m=radius)
            resp.spawned = int(got.get("vehicles", 0))
            resp.walkers_spawned = int(got.get("walkers", 0))
            # Asked-for and got are routinely different: the map refuses spawn points that
            # are occupied. Saying so here beats a scenario silently running near-empty.
            if resp.spawned < int(req.vehicles):
                resp.message = f"map accepted {resp.spawned} of {int(req.vehicles)} vehicles"
            # Two ways this "succeeds" and still gives the caller something they did not
            # ask for, both of which used to be invisible:
            #
            #   1. zero spawns - not partial success, a failure;
            #   2. the sidecar falling back to MAP-WIDE because the radius held too few
            #      spawn points. Measured, that is the difference between 20 cars within
            #      60 m of the aircraft and 5, and the camera only sees the neighbourhood.
            if near is not None and not got.get("clustered", True):
                resp.message = (
                    f"NOT clustered: only {got.get('near_candidates', 0)} spawn points within "
                    f"{radius:.0f} m of {[round(v, 1) for v in near]}, need "
                    f"{int(req.vehicles)} - fell back to map-wide")
                self.get_logger().warn(resp.message)
            resp.success = not (int(req.vehicles) > 0 and resp.spawned == 0)
            if not resp.success:
                resp.message = (f"no vehicles spawned near {near or 'map-wide'} "
                                f"within {radius:.0f} m - is that point on the map?")
        except (SimBridgeError, OSError, ValueError) as exc:
            resp.success = False
            resp.message = str(exc)
            self.get_logger().error(f"spawn_traffic failed: {exc}")
        return resp

    def _srv_set_weather(self, req, resp):
        try:
            got = self.world.set_weather(preset=req.preset)
            resp.success = True
            resp.applied = str(got["weather"])
        except (SimBridgeError, OSError, ValueError) as exc:
            # An unknown preset must FAIL, not fall back to clear skies: a scenario called
            # rain_descent running in sunshine scores fine and means nothing.
            resp.success = False
            resp.message = str(exc)
            self.get_logger().error(f"set_weather failed: {exc}")
        return resp

    def _srv_destroy_actors(self, req, resp):
        try:
            got = self.world.destroy_actors()
            resp.success = True
            resp.destroyed = int(got.get("destroyed", 0))
        except (SimBridgeError, OSError, KeyError, TypeError, ValueError) as exc:
            resp.success = False
            resp.message = str(exc)
            self.get_logger().error(f"destroy_actors failed: {exc}")
        return resp

    def _srv_set_camera_pose(self, req, resp):
        try:
            if len(req.xyz) != 3:
                raise ValueError(f"xyz needs 3 values, got {len(req.xyz)}")
            self.world.set_camera_pose(xyz=list(req.xyz), pitch=float(req.pitch),
                                       roll=float(req.roll), yaw=float(req.yaw))
            resp.success = True
            self.get_logger().info(
                f"camera pose -> xyz={[round(v, 2) for v in req.xyz]} pitch={req.pitch}")
        except (SimBridgeError, OSError, ValueError) as exc:
            resp.success = False
            resp.message = str(exc)
            self.get_logger().error(f"set_camera_pose failed: {exc}")
        return resp

    def _srv_chase_recording(self, req, resp):
        """Start or stop the exterior recording. Never allowed to fail a flight."""
        try:
            if req.start:
                if not req.path:
                    raise ValueError("path is required when start is true")
                # Pass zeros THROUGH. Substituting literals here re-hid the config: the
                # sidecar cannot tell "the caller wants 1280" from "the caller said nothing".
                self.world.chase_start(path=req.path, width=int(req.width),
                                       height=int(req.height), fps=float(req.fps))
                self.get_logger().info(f"chase recording -> {req.path}")
            else:
                got = self.world.chase_stop()
                resp.frames = int(got.get("frames", 0))
                resp.dropped = int(got.get("dropped", 0))
                resp.seconds = float(got.get("seconds", 0.0))
            resp.success = True
        except (SimBridgeError, OSError, ValueError) as exc:
            # Reported, not raised. A spectator camera that cannot start must not take the
            # episode with it - worst case the run has no video.
            resp.success = False
            resp.message = str(exc)
            self.get_logger().warn(f"chase recording: {exc}")
        return resp

    def _tick_world(self):
        if self._resetting:
            return       # see _resetting: AirSim is rebuilding the vehicle
        try:
            col = self.sim.collision()
            self.pub_collision.publish(Collision(
                has_collided=bool(col["has_collided"]),
                object_name=str(col.get("object_name") or ""),
                object_id=int(col.get("object_id", 0)),
                position_ned=[float(v) for v in col.get("position", (0.0, 0.0, 0.0))],
                penetration_m=float(col.get("penetration_m", 0.0))))
            # Traffic stalls intermittently (4/15 vs 11/15 across two runs); upstream ships
            # a watchdog for it and so do we. Without this an episode's "urban traffic" can
            # quietly become a car park.
            self.sim.watchdog()
            stats = self.sim.traffic_stats()
            self.pub_traffic.publish(String(
                data=f"spawned={stats['spawned']} moving={stats['moving']} "
                     f"walkers={stats['walkers']} "
                     f"walkers_moving={stats.get('walkers_moving', -1)} "
                     f"controllers={stats.get('controllers', -1)}"))
        except (SimBridgeError, OSError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(f"world tick failed: {exc}", throttle_duration_sec=10.0)

    # ------------------------------------------------------------------- inputs

    def _on_offboard(self, msg: OffboardControlMode):
        self._offboard_active = bool(msg.velocity or msg.position)

    def _on_setpoint(self, msg: TrajectorySetpoint):
        """Forward a PX4 setpoint to SimpleFlight.

        Velocity wins when both are finite: the SPF inner loop streams velocity, and a
        position field left at NaN is PX4's way of saying "ignore me".
        """
        duration = float(self.get_parameter("setpoint_duration_s").value)
        yaw_deg = math.degrees(msg.yaw) if math.isfinite(msg.yaw) else None
        try:
            if all(math.isfinite(c) for c in msg.velocity):
                self.ctrl.velocity(msg.velocity[0], msg.velocity[1], msg.velocity[2],
                                   duration=duration, yaw_deg=yaw_deg)
            elif all(math.isfinite(c) for c in msg.position):
                self.ctrl.call("velocity", vx=0.0, vy=0.0, vz=0.0, duration=0.1, yaw_deg=None)
                self.ctrl.goto(msg.position[0], msg.position[1], msg.position[2])
            else:
                self.get_logger().warn("setpoint had neither finite velocity nor position",
                                       throttle_duration_sec=5.0)
        except (SimBridgeError, OSError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f"setpoint rejected: {exc}", throttle_duration_sec=2.0)

    def destroy_node(self):
        try:
            self.sim.close()
            self.media.close()
            self.ctrl.close()
        finally:
            super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = CarlaAirBridge()
    # 7 mutually-exclusive callback groups (odometry, images, sensors, lidar and world-tick
    # timers, plus setpoints and world-control services) sharing 5 threads. Not one each:
    # the timers are short and never all due together, and this was measured rather than
    # assumed - odometry held 19.9 Hz against a 20 Hz target THROUGH a 16.2 s blocking
    # reset. What matters is that `reset` cannot hold a timer's thread, which the separate
    # world_group guarantees. Add a group and re-measure before assuming it still holds.
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
