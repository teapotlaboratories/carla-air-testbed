#!/usr/bin/env python3
"""Onboard video and telemetry for the web console, taken from ROS 2 topics.

**R-03 step 1.** The console used to get both by calling the sidecar over the Unix socket:
`view_jpeg` for the picture, `state` and `collision` for the numbers. That works, and it costs
more than it looks:

* **It opens a SECOND AirSim capture.** The image path is this project's bottleneck and the
  ROS graph is already on it. **Measured 2026-08-07**, three drift-controlled pairs each way,
  watching `/camera/rgb/image_raw` while a browser streams:

      console running, not streaming   6.39 - 6.66 Hz   baseline
      streaming on the socket          5.060 Hz         -24.0%
      streaming on ROS                 5.968 Hz          -6.6%

  So subscribing does **not** make the console free — re-encoding to JPEG still costs the
  machine that is also rendering — but it costs far less, which is the difference between a
  console you must close before a scored run and one you can leave open.

  **Read that baseline carefully.** It is the console *running and idle*, not absent. An idle
  ROS console still converts every frame in `_on_image`, so its idle cost is INSIDE the
  baseline, while an idle socket console makes no calls at all and costs ~0. The comparison is
  therefore biased toward ROS: -6.6% is a floor on this console's total cost, not the total. A
  true no-console baseline has not been measured.
* **It needs the socket, and the socket is unreachable from outside the containers.** The
  stack serves it on a Docker volume whose host mountpoint is `Permission denied` under the
  rootless daemon. A node on the DDS graph needs no socket at all, which is why this step is
  also the cheapest route to a containerised console.

Everything below the `RosSource` class is **pure**: plain numbers in, plain dicts out, no
`rclpy` and no `px4_msgs`. That is deliberate — it is the part with the frame conventions in
it, and it is testable in `tests/` with no simulator, no graph and no display.

## Traps this file exists to not fall into

* **PX4 QoS is not the default.** `/fmu/out/*` is published BEST_EFFORT + TRANSIENT_LOCAL. A
  subscriber that takes the default RELIABLE profile matches nothing and **receives nothing,
  silently, with the publisher right there and healthy**.
* **NED z is not altitude.** Positive z is *below* the origin, and the origin sits 27.45 m
  above the street on Town10HD. The page already knows this; this file must not "help" by
  flipping a sign.
* **No silent fallback.** If ROS was asked for and the frames do not arrive, this reports that
  it has no frames. It does **not** quietly reopen the socket — a fallback nobody is told
  about is how an episode recorder spent two days writing mp4v at the wrong length (T-02).
"""
from __future__ import annotations

import math
import threading
import time

import ros_control

#: The origin sits this far above the street on Town10HD. Here only so the console and the
#: page cannot disagree about it; the conversion functions below never apply it.
GROUND_NED_Z = 27.45

#: How old the last frame or odometry sample may be before the source calls itself stale.
#: The camera runs at ~4-8 Hz and odometry at ~16 Hz, so a second is many missed samples and
#: not a hiccup.
STALE_AFTER_S = 2.0


# --------------------------------------------------------------------------- pure conversions

def quat_to_yaw(w: float, x: float, y: float, z: float) -> float:
    """Yaw in radians from a PX4 quaternion.

    Identical to `sim_bridge/carla_air/frames.py:quat_to_yaw`, and duplicated rather than
    imported on purpose: this module runs on the 3.12 side and importing the 3.10 tree to get
    four lines of arithmetic would put a package boundary where a formula belongs.
    """
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def odometry_to_state(position, q, velocity, armed=True, t=None):
    """A `VehicleOdometry` reduced to exactly what `/api/state` already returns.

    The shape is not negotiable — `webui/index.html` reads `position`, `velocity`, `yaw` and
    `armed` off it, and step 1 must not require the page to be rewritten to keep working.

    `position` and `velocity` are NED metres; `q` is (w, x, y, z). Sequences of any kind are
    accepted because a ROS message hands over `numpy.float32` arrays and the JSON encoder will
    not take those.
    """
    return {
        "t": time.time() if t is None else t,
        "position": [float(v) for v in position[:3]],
        "velocity": [float(v) for v in velocity[:3]],
        "yaw": quat_to_yaw(*(float(v) for v in q[:4])),
        "armed": bool(armed),
        "source": "ros",
    }


def collision_to_dict(has_collided, object_name="", position_ned=(0.0, 0.0, 0.0)):
    """A `Collision` message reduced to what the page reads off `/api/state`.

    `has_collided` is latched until a reset, matching AirSim's own semantics — so a console
    opened after an impact still learns the run is spoiled.
    """
    return {
        "has_collided": bool(has_collided),
        "object_name": str(object_name or ""),
        "position": [float(v) for v in position_ned[:3]],
        "source": "ros",
    }


def freshness(last_seen, now=None, stale_after=STALE_AFTER_S):
    """`(age_seconds, is_fresh)` for a monotonic stamp, or `(None, False)` if never seen.

    Split out because "have I ever received one" and "was the last one recent" are different
    questions with different remedies — nothing published versus something stopped — and the
    status endpoint has to tell them apart.
    """
    if last_seen is None:
        return None, False
    age = (time.monotonic() if now is None else now) - last_seen
    return age, age <= stale_after


# --------------------------------------------------------------------------- the live source

class RosSource:
    """Subscribes to the camera, odometry, status and collision topics on a background thread.

    Frames are cached **raw** and encoded to JPEG on demand rather than in the subscription
    callback, so with no browser attached the *encode* does not happen.

    That is not the same as costing nothing when idle: `_on_image` still converts every frame,
    and the subscription itself makes the publisher serialise to one more reader. Whatever that
    comes to has never been measured separately — it is folded into the baseline of the numbers
    above rather than isolated.
    """

    #: Set once `import rclpy` has been attempted, so the reason for a refusal can be reported
    #: rather than guessed at.
    import_error: str | None = None

    def __init__(self, image_topic="/camera/rgb/image_raw",
                 odom_topic="/fmu/out/vehicle_odometry",
                 status_topic="/fmu/out/vehicle_status",
                 collision_topic="/sim/collision"):
        self.topics = dict(image=image_topic, odom=odom_topic,
                           status=status_topic, collision=collision_topic)
        self._lock = threading.Lock()
        self._frame = None                 # (numpy BGR array, monotonic stamp)
        self._jpeg = None                  # (frame stamp, quality, encoded bytes)
        self._state = None                 # (dict, monotonic stamp)
        self._collision = collision_to_dict(False)
        self._armed = True
        self._node = None
        self._thread = None
        self._pub_setpoint = None
        self._pub_command = None
        self._services = {}
        self._stop = threading.Event()
        self._cv = None
        self._cv2 = None

    # -- lifecycle ------------------------------------------------------------------------

    @classmethod
    def importable(cls):
        """Whether this process can be a ROS node at all, with the reason cached if not."""
        try:
            import rclpy  # noqa: F401
            from cv_bridge import CvBridge  # noqa: F401
            return True
        except Exception as exc:  # noqa: BLE001 — any import failure means the same thing here
            cls.import_error = f"{type(exc).__name__}: {exc}"
            return False

    def start(self):
        """Bring the node up and spin it on a daemon thread. Raises if ROS is not importable."""
        import cv2
        import rclpy
        from cv_bridge import CvBridge
        from interfaces.msg import Collision
        from px4_msgs.msg import VehicleOdometry, VehicleStatus
        from rclpy.node import Node
        from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                               QoSReliabilityPolicy)
        from sensor_msgs.msg import Image

        # NOT the default profile. /fmu/out/* is BEST_EFFORT + TRANSIENT_LOCAL, and a RELIABLE
        # subscriber matches nothing and receives nothing with no error anywhere.
        px4_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=5)

        self._cv2, self._cv = cv2, CvBridge()
        if not rclpy.ok():
            rclpy.init()
        self._node = Node("carla_air_webui")

        # depth 1 on the image: the console wants the LATEST frame, never a backlog. A deeper
        # queue would show the browser what the aircraft saw a second ago.
        self._node.create_subscription(Image, self.topics["image"], self._on_image, 1)
        self._node.create_subscription(VehicleOdometry, self.topics["odom"], self._on_odom, px4_qos)
        self._node.create_subscription(VehicleStatus, self.topics["status"], self._on_status, px4_qos)
        self._node.create_subscription(Collision, self.topics["collision"], self._on_collision, 5)

        # R-03 step 2: the command surface. PX4 messages on PX4 topics for flight, the four
        # existing /sim/* services for the world. Nothing new was added to the ROS surface.
        from px4_msgs.msg import TrajectorySetpoint, VehicleCommand
        from interfaces.srv import DestroyActors, ResetVehicle, SetWeather, SpawnTraffic

        self._pub_setpoint = self._node.create_publisher(
            TrajectorySetpoint, ros_control.SETPOINT_TOPIC, px4_qos)
        self._pub_command = self._node.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", px4_qos)
        self._services = {
            "/sim/reset_vehicle": self._node.create_client(ResetVehicle, "/sim/reset_vehicle"),
            "/sim/spawn_traffic": self._node.create_client(SpawnTraffic, "/sim/spawn_traffic"),
            "/sim/set_weather": self._node.create_client(SetWeather, "/sim/set_weather"),
            "/sim/destroy_actors": self._node.create_client(DestroyActors, "/sim/destroy_actors"),
        }

        def spin():
            while not self._stop.is_set() and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.2)

        self._thread = threading.Thread(target=spin, name="webui-ros", daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        # Leaving the context initialised is harmless at process exit and a nuisance anywhere
        # else — a second node in the same process would inherit a half-torn-down context.
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001 — never let teardown raise
            pass

    # -- callbacks ------------------------------------------------------------------------

    def _on_image(self, msg):
        frame = self._cv.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with self._lock:
            self._frame = (frame, time.monotonic())

    def _on_odom(self, msg):
        state = odometry_to_state(msg.position, msg.q, msg.velocity, armed=self._armed)
        with self._lock:
            self._state = (state, time.monotonic())

    def _on_status(self, msg):
        from px4_msgs.msg import VehicleStatus
        self._armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def _on_collision(self, msg):
        with self._lock:
            self._collision = collision_to_dict(msg.has_collided, msg.object_name,
                                                msg.position_ned)

    # -- what the server asks for ----------------------------------------------------------

    def latest_jpeg(self, quality=75):
        """The most recent frame as JPEG bytes, or None if none has arrived yet.

        **Encodes each frame at most once.** The stream serves 12 fps and the camera publishes
        at ~6.4 Hz, so without this roughly half of every encode is the same frame again — pure
        CPU spent on the machine that is also rendering. Keyed on the frame's arrival stamp
        rather than its content, which is cheap and cannot collide: a new frame always gets a
        new `time.monotonic()`.
        """
        with self._lock:
            entry = self._frame
            cached = self._jpeg
        if entry is None:
            return None
        frame, stamp = entry
        if cached is not None and cached[0] == stamp and cached[1] == quality:
            return cached[2]
        ok, buf = self._cv2.imencode(".jpg", frame,
                                     [int(self._cv2.IMWRITE_JPEG_QUALITY), int(quality)])
        if not ok:
            return None
        data = buf.tobytes()
        with self._lock:
            self._jpeg = (stamp, quality, data)
        return data

    def state(self):
        with self._lock:
            return None if self._state is None else dict(self._state[0])

    def collision(self):
        with self._lock:
            return dict(self._collision)

    def health(self):
        """Per-topic age and freshness, so the page can say *why* it has no picture."""
        with self._lock:
            frame_at = None if self._frame is None else self._frame[1]
            state_at = None if self._state is None else self._state[1]
        video_age, video_ok = freshness(frame_at)
        state_age, state_ok = freshness(state_at)
        return {
            "source": "ros",
            "topics": dict(self.topics),
            "video": {"age_s": None if video_age is None else round(video_age, 2),
                      "fresh": video_ok, "ever": frame_at is not None},
            "state": {"age_s": None if state_age is None else round(state_age, 2),
                      "fresh": state_ok, "ever": state_at is not None},
        }

    # -- the command surface (R-03 step 2) --------------------------------------------------

    def contested(self):
        """Nodes other than this console publishing setpoints. Zero means we are alone."""
        if self._node is None:
            return 0
        return ros_control.others_publishing(
            self._node.count_publishers(ros_control.SETPOINT_TOPIC))

    def command(self, method, args=None):
        """Issue a console command over ROS 2. Raises `KeyError` if it has no ROS equivalent.

        **Refuses to fly while another node is publishing setpoints.** Two publishers on one
        setpoint topic produce no error at all, just an aircraft that goes somewhere neither
        asked for — measured in `examples/ros2_full_control.py`, where a takeoff to NED 35 m
        reached 15.6 m. Seizing control instead would be a large side effect for a button
        press; the console's job after step 1 is watching a run, so it declines and says who
        it is declining to.
        """
        kind, payload = ros_control.plan(method, args)

        if self._pub_setpoint is None or self._pub_command is None:
            raise RuntimeError(
                "the ROS command surface is not up — start() did not finish, so this console "
                "can subscribe but not command. Check its startup log.")

        if method in ros_control.GUARDED_METHODS:
            others = self.contested()
            if others:
                raise ros_control.Contested(
                    f"{others} other node(s) are publishing {ros_control.SETPOINT_TOPIC} — "
                    f"commanding from here would fight them, with no error to say why. Stop the "
                    f"controller (examples/navigation) first, or watch rather than fly.")

        if kind == "setpoint":
            from px4_msgs.msg import TrajectorySetpoint
            msg = TrajectorySetpoint()
            msg.timestamp = int(time.time() * 1e6)
            for field, value in payload.items():
                setattr(msg, field, value)
            self._pub_setpoint.publish(msg)
            return {"sent": "trajectory_setpoint"}

        if kind == "command":
            from px4_msgs.msg import VehicleCommand
            name, params = payload
            msg = VehicleCommand()
            msg.timestamp = int(time.time() * 1e6)
            msg.command = getattr(VehicleCommand, name)
            for i in range(1, 8):
                setattr(msg, f"param{i}", float(params.get(f"param{i}", 0.0)))
            # A real PX4 filters on these; the bridge ignores them. Sending the right values
            # means the same client works against hardware without edits.
            msg.target_system = msg.target_component = 1
            msg.source_system = msg.source_component = 1
            msg.from_external = True
            self._pub_command.publish(msg)
            return {"sent": name}

        name, fields = payload
        client = self._services[name]
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"{name} is not available — is the bridge node up?")
        request = client.srv_type.Request()
        for field, value in fields.items():
            if hasattr(request, field):
                setattr(request, field, value)
        future = client.call_async(request)
        # The executor spins on our own thread, so block on the future rather than spinning
        # here — spin_until_future_complete from a second thread deadlocks against it.
        for _ in range(600):
            if future.done():
                break
            time.sleep(0.05)
        if not future.done():
            raise RuntimeError(f"{name} did not answer within 30 s")
        reply = future.result()
        return {k: getattr(reply, k) for k in reply.get_fields_and_field_types()}
