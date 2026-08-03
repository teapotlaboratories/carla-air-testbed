"""Python 3.12 client for the 3.10 sim_bridge. Mirror image of `sim_bridge/protocol.py`.

The protocol module is shared verbatim between the two interpreters — it is imported here
from the repository rather than duplicated, because a wire format that drifts between the
two halves of a bridge is the classic way to lose a day.
"""
from __future__ import annotations

import importlib.util
import os
import socket
import threading

_PROTOCOL_PATH = os.environ.get(
    "TESTBED_PROTOCOL",
    os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))), "sim_bridge", "protocol.py"),
)


def _load_protocol():
    spec = importlib.util.spec_from_file_location("testbed_protocol", _PROTOCOL_PATH)
    if spec is None or spec.loader is None:
        raise ImportError(
            f"cannot load the shared protocol from {_PROTOCOL_PATH}. "
            "Set TESTBED_PROTOCOL to sim_bridge/protocol.py."
        )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


protocol = _load_protocol()
decode_image = protocol.decode_image
DEFAULT_SOCKET = protocol.DEFAULT_SOCKET


class SimBridgeError(RuntimeError):
    """The sim side raised. Carries its traceback so the ROS log shows the real cause."""

    def __init__(self, method, error, tb=""):
        super().__init__(f"{method}: {error}")
        self.remote_traceback = tb


class SimBridgeClient:
    """Blocking, thread-safe RPC client. One instance == one connection.

    Calls on a single instance are serialised by `self._lock`, which is required: the wire
    protocol is a stream, and two threads interleaving frames on the same socket corrupts
    both replies. To get parallelism, open a second instance — the sidecar handles each
    connection on its own thread with its own AirSim client. The bridge node does exactly
    that: one client for telemetry and control, one for image capture.
    """

    def __init__(self, path: str = DEFAULT_SOCKET, timeout: float = 60.0):
        self.path = path
        self.timeout = timeout
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._next_id = 0

    def connect(self):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        s.connect(self.path)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        self._sock = s
        return self

    def close(self):
        if self._sock:
            self._sock.close()
            self._sock = None

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def call(self, method: str, **args):
        if method not in protocol.METHODS:
            raise SimBridgeError(method, "not a known method — see protocol.METHODS")
        if self._sock is None:
            raise SimBridgeError(method, "not connected")
        with self._lock:
            self._next_id += 1
            rid = self._next_id
            protocol.send(self._sock, {"id": rid, "method": method, "args": args})
            reply = protocol.recv(self._sock)
        if not reply.get("ok"):
            raise SimBridgeError(method, reply.get("error", "unknown"), reply.get("traceback", ""))
        return reply.get("result")

    # ---- convenience wrappers, so nodes read as intent rather than as RPC ----

    def describe(self):
        return self.call("describe")

    def reset(self, hold_ned=(0.0, 0.0, -40.0), speed=8.0):
        return self.call("reset", hold_ned=list(hold_ned), speed=speed)

    def state(self):
        return self.call("state")

    def sensors(self):
        """IMU + barometer + magnetometer + GPS + environment, one round trip."""
        return self.call("sensors")

    def attitude(self, roll, pitch, yaw, z, duration=0.2):
        return self.call("attitude", roll=roll, pitch=pitch, yaw=yaw, z=z, duration=duration)

    def takeoff(self, altitude_ned=-30.0, speed=6.0):
        return self.call("takeoff", altitude_ned=altitude_ned, speed=speed)

    def lidar(self):
        """Newest semantic LiDAR sweep, or None if none is configured."""
        return self.call("lidar")

    def carla_sensors(self):
        return self.call("carla_sensors")

    def collision(self):
        return self.call("collision")

    def capture(self, rgb=True, depth=True, segmentation=False):
        r = self.call("capture", rgb=rgb, depth=depth, segmentation=segmentation)
        # No vehicle state here on purpose: capture() runs on the media connection, and
        # reading state through it would put telemetry back behind the 500 ms capture that
        # the two-connection split exists to avoid. Take state from the telemetry client.
        return {
            "images": {k: decode_image(v) for k, v in r["images"].items()},
            "camera": r["camera"],
        }

    def ground(self, u, v, rgb_shape=None):
        return self.call("ground", u=int(u), v=int(v),
                         rgb_shape=list(rgb_shape) if rgb_shape else None)

    def velocity(self, vx, vy, vz, duration=0.5, yaw_deg=None):
        return self.call("velocity", vx=float(vx), vy=float(vy), vz=float(vz),
                         duration=float(duration), yaw_deg=yaw_deg)

    def goto(self, x, y, z, speed=6.0, settle_s=0.0):
        return self.call("goto", x=float(x), y=float(y), z=float(z),
                         speed=float(speed), settle_s=float(settle_s))

    def hold(self):
        return self.call("hold")

    def land(self):
        return self.call("land")

    def set_camera_pose(self, xyz=(0.5, 0.0, 0.1), pitch=0.0, roll=0.0, yaw=0.0):
        return self.call("set_camera_pose", xyz=list(xyz), pitch=pitch, roll=roll, yaw=yaw)

    def spawn_traffic(self, vehicles=15, walkers=10, near_ned=None, radius_m=70.0):
        """`near_ned` concentrates the traffic; without it, spawn points are shuffled
        map-wide and a small fleet spreads across the whole of Town10HD."""
        args = {"vehicles": vehicles, "walkers": walkers, "radius_m": radius_m}
        if near_ned is not None:
            args["near_ned"] = list(near_ned)
        return self.call("spawn_traffic", **args)

    def traffic_stats(self):
        return self.call("traffic_stats")

    def watchdog(self):
        return self.call("watchdog")

    def set_weather(self, preset="ClearNoon"):
        return self.call("set_weather", preset=preset)

    def destroy_actors(self):
        return self.call("destroy_actors")

    def carla_to_ned(self, x, y, z=0.0):
        return self.call("carla_to_ned", x=float(x), y=float(y), z=float(z))["ned"]
