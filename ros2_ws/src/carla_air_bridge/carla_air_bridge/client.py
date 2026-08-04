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
import time

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


class _Slot:
    """One in-flight call: somewhere for the reader thread to put the answer."""

    __slots__ = ("event", "reply", "last_activity", "method")

    def __init__(self, method):
        self.event = threading.Event()
        self.reply = None
        self.method = method
        self.last_activity = time.monotonic()


class SimBridgeClient:
    """Thread-safe RPC client. One instance == one connection.

    **Replies are matched to calls by id, by a reader thread.** That is not a refinement; it
    is what makes a slow call survivable. The obvious design - send, then block reading the
    next frame - assumes the next frame off the socket is your reply. It is not, the moment
    any caller gives up early:

        caller sends id 41 (reset), waits, gives up at the timeout, raises
        the sidecar finishes anyway and writes the id 41 reply into the socket
        caller sends id 42 (state) and reads ... the buffered id 41 reply

    From then on every call gets the previous call's answer, forever, with no resync. That is
    exactly how a 40-episode sweep died: `state()` unpacked `reset()`'s reply and raised
    `KeyError: 'position'`, which rclpy does not catch, so the whole bridge node exited(1).
    See docs/rpc-path.html.

    With a reader thread the abandoned reply is popped and DROPPED, and the stream stays in
    step. A timeout then costs one failed call rather than the graph.

    Calls are still serialised on the write side by `self._lock` - two threads interleaving
    frames would corrupt both. To get parallelism, open a second instance; the sidecar
    handles each connection on its own thread with its own AirSim client.
    """

    #: How long to wait with NO news at all before giving up on a call. Not a limit on how
    #: long a call may take: any progress frame resets it (see `_reader`), so a `reset` that
    #: keeps reporting can run for minutes while a wedged one fails promptly. That
    #: distinction is the entire point - a fixed ceiling cannot tell slow from dead.
    DEFAULT_PATIENCE_S = 60.0

    def __init__(self, path: str = DEFAULT_SOCKET, timeout: float = DEFAULT_PATIENCE_S):
        self.path = path
        self.timeout = timeout
        self._sock: socket.socket | None = None
        self._lock = threading.Lock()          # serialises WRITES only
        self._next_id = 0
        self._pending: dict[int, _Slot] = {}
        self._pending_lock = threading.Lock()
        self._reader: threading.Thread | None = None
        self._closing = False
        self._dead: str | None = None          # why the connection ended, if it has
        self._generation = 0                   # bumped per connect(); see _read_loop

    def connect(self):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        # No socket timeout. The reader blocks here indefinitely on purpose - waiting is now
        # the caller's decision, per call, and a blocking read is how a late reply gets
        # consumed instead of buffered.
        s.connect(self.path)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        self._sock = s
        self._closing = False
        self._dead = None
        # A generation number, so a reader from a PREVIOUS connection cannot report on this
        # one. Without it, reconnecting is a race the fresh connection loses: close() sets
        # _closing, the old reader is still blocked in recv(), connect() clears _closing, and
        # only THEN does the old reader wake with its socket error - sees _closing is False,
        # concludes the connection died, and calls _fail_all() on a connection that is
        # perfectly healthy and belongs to somebody else. Nothing in the graph reconnects
        # today (each client connects once in bridge_node), which is the only reason this has
        # never fired.
        self._generation += 1
        gen = self._generation
        self._reader = threading.Thread(target=self._read_loop, args=(s, gen), daemon=True,
                                        name=f"simbridge-reader-{os.path.basename(self.path)}")
        self._reader.start()
        return self

    def close(self):
        self._closing = True
        if self._sock:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
            self._sock = None
        self._fail_all("connection closed")

    @property
    def connected(self) -> bool:
        return self._sock is not None and self._dead is None

    # ------------------------------------------------------------------ reader

    def _read_loop(self, sock, gen):
        try:
            while not self._closing and gen == self._generation:
                frame = protocol.recv(sock)
                rid = frame.get("id")
                if "progress" in frame and "ok" not in frame:
                    # Liveness, not an answer. Refresh the slot's clock and keep reading.
                    with self._pending_lock:
                        slot = self._pending.get(rid)
                    if slot is not None:
                        slot.last_activity = time.monotonic()
                    continue
                with self._pending_lock:
                    slot = self._pending.pop(rid, None)
                if slot is None:
                    # THE WHOLE POINT: a reply whose caller already gave up. Consumed and
                    # dropped here, so it can never be mistaken for the next call's answer.
                    self.late_replies += 1
                    continue
                slot.reply = frame
                slot.event.set()
        except Exception as exc:                                        # noqa: BLE001
            if not self._closing:
                self._fail_all(f"reader stopped: {exc}", gen)
        else:
            self._fail_all("connection closed by the sidecar", gen)

    def _fail_all(self, why, gen=None):
        # `gen` is the connection this verdict is about. A reader unwinding from a socket
        # that has already been replaced must not condemn its successor.
        if gen is not None and gen != self._generation:
            return
        self._dead = why
        with self._pending_lock:
            slots, self._pending = list(self._pending.values()), {}
        for slot in slots:
            slot.reply = {"ok": False, "error": why}
            slot.event.set()

    # -------------------------------------------------------------------- call

    late_replies = 0

    def call(self, method: str, **args):
        if method not in protocol.METHODS:
            raise SimBridgeError(method, "not a known method — see protocol.METHODS")
        if self._sock is None:
            raise SimBridgeError(method, "not connected")
        if self._dead:
            raise SimBridgeError(method, self._dead)

        slot = _Slot(method)
        with self._lock:
            self._next_id += 1
            rid = self._next_id
            with self._pending_lock:
                self._pending[rid] = slot
            try:
                protocol.send(self._sock, {"id": rid, "method": method, "args": args})
            except OSError:
                with self._pending_lock:
                    self._pending.pop(rid, None)
                raise

        # Wait in slices so a progress frame can extend the deadline. Waiting on the whole
        # patience at once would ignore progress entirely.
        while True:
            if slot.event.wait(1.0):
                break
            if time.monotonic() - slot.last_activity >= self.timeout:
                with self._pending_lock:
                    self._pending.pop(rid, None)   # tombstone gone; the reader will drop it
                raise SimBridgeError(
                    method, f"no reply and no progress for {self.timeout:.0f}s (id {rid})")

        reply = slot.reply or {}
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

    def chase_start(self, path, width=1280, height=720, fps=30.0):
        """The exterior follow camera. A spectator view, scored on nothing."""
        return self.call("chase_start", path=path, width=width, height=height, fps=fps)

    def chase_stop(self):
        return self.call("chase_stop")

    def carla_to_ned(self, x, y, z=0.0):
        return self.call("carla_to_ned", x=float(x), y=float(y), z=float(z))["ned"]
