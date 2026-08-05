#!/usr/bin/env python3
"""The 3.10 half of the testbed: owns the CARLA and AirSim clients, serves the ROS 2 side.

    ./.venv/bin/python sim_bridge/server.py [--socket PATH] [--config configs/testbed.yaml]

**Three AirSim clients, not one.** An image capture takes ~500 ms and telemetry takes ~5 ms;
sharing one RPC client puts every odometry read behind the current capture and drops the
`/fmu/out/vehicle_odometry` rate from 20 Hz to 1.5 Hz — measured, and enough to starve the
10 Hz offboard controller of fresh position. So telemetry and control get one client, media
gets another, and control gets a third — each behind its own lock, with connections handled
one thread apiece. Control is separate from telemetry because they were serialising against
each other: 20 Hz of state() plus 10 Hz of velocity() through a single AirSim client
capped odometry at 12.3 Hz. The ROS 2 bridge opens three connections for the same reason.

AirSim itself tolerates multiple clients; what is not safe is two threads inside one
msgpack-rpc client, which the locks prevent.
"""
from __future__ import annotations

import argparse
import os
import socket
import sys
import threading
import time
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import airsim  # noqa: E402
import carla  # noqa: E402

import protocol  # noqa: E402
from carla_air.camera import Camera  # noqa: E402
from carla_air.carla_sensors import (CarlaSensorRig,  # noqa: E402
                                     semantic_lidar_payload)
from carla_air.chase import ChaseCamera  # noqa: E402
from carla_air.frames import (OFFSETS, DEFAULT_OFFSET, carla_to_ned,  # noqa: E402
                              quat_to_yaw)
from carla_air.vehicle import Vehicle  # noqa: E402
from carla_air.world import World  # noqa: E402


class SimBridge:
    #: telemetry reads — must never queue behind capture OR behind a control command
    FAST = frozenset({"ping", "state", "collision", "carla_to_ned", "sensors"})
    #: control writes — their own client so a 10 Hz setpoint stream cannot starve telemetry
    #: `reset` belongs here and not in the default slow class, which is where it sat until
    #: 2026-08-03. It COMMANDS the vehicle, so CONTROL is right semantically - but the bug
    #: was mechanical: `reset` drove `self.vehicle` (the TELEMETRY client) under slow_lock
    #: while FAST `state`/`collision` drove the same msgpack-rpc connection at 20 Hz under
    #: fast_lock. Two locks, one socket. It surfaced as
    #:     reset: IOLoop is already running
    #: and, when the wedged connection stalled the dispatcher long enough for CARLA's own
    #: RPC to time out, as an uncaught carla::client::TimeoutException that terminated the
    #: whole sidecar mid-sweep. `reset` now uses `self.control`, which has its own client
    #: and its own lock, so telemetry keeps running at 20 Hz through a 16 s reset.
    CONTROL = frozenset({"velocity", "hold", "goto", "yaw", "land", "takeoff", "attitude",
                         "reset"})
    #: frames — the media AirSim client, or CARLA for the chase view. Neither touches the
    #: telemetry or control sockets, so serialising them against `reset`/`land`/`goto` bought
    #: nothing and cost everything: `land` blocks for the whole descent while holding
    #: `slow_lock`, and BOTH video streams froze behind it for tens of seconds. Reported as
    #: "pressing land breaks the web stream"; it was the lock, not the stream.
    #:
    #: `set_camera_pose` belongs here too, and its absence was the FOURTH instance of this
    #: same root cause. It drives `self.camera`, which is built on `self.airsim_media` - but
    #: it sat outside every class and so took `slow_lock`, letting it run concurrently with
    #: `capture` on the one msgpackrpc socket. msgpackrpc answers that with
    #: "IOLoop is already running", the call fails, and the CAMERA PITCH IS SILENTLY NOT
    #: APPLIED - on a measurement surface that every scored episode depends on.
    #: `describe` and `ground` are here for the same mechanical reason as `set_camera_pose`:
    #: both call self.camera, which is built on the media client. They sat in the slow class
    #: until 2026-08-03, free to race a capture on the one connection. `ground` is the
    #: dangerous one - it takes a real depth capture.
    MEDIA = frozenset({"capture", "view_jpeg", "chase_jpeg", "lidar", "set_camera_pose",
                       "describe", "ground"})

    #: CARLA's RPC timeout, and it is NOT a nicety. When it expires, CARLA raises
    #: `carla::client::TimeoutException` from a C++ thread where nothing catches it, so it
    #: calls terminate() and takes the WHOLE SIDECAR down - mid-episode, with a broken pipe
    #: as the only clue on the ROS side. That has now happened twice: once at 120k lidar
    #: points/second, and again during a 40-episode sweep, where destroy -> spawn 30
    #: vehicles + 20 walkers -> reset -> 30 fps chase recording, back to back, pushed a
    #: single call past 30 s. It cannot be caught in Python, so the only defence is not
    #: reaching it.
    #:
    #: 120 s costs nothing when the simulator is healthy: the timeout is a ceiling, not a
    #: delay. It only matters when the simulator is genuinely wedged, and then a slower
    #: failure is far better than a dead sidecar.
    CARLA_TIMEOUT_S = 120.0

    def __init__(self, carla_port=2000, airsim_port=41451, seed=None, timeout=None):
        timeout = self.CARLA_TIMEOUT_S if timeout is None else timeout
        self.carla_client = carla.Client("127.0.0.1", carla_port)
        self.carla_client.set_timeout(timeout)

        def _airsim():
            c = airsim.MultirotorClient(ip="127.0.0.1", port=airsim_port, timeout_value=timeout)
            c.confirmConnection()
            return c

        self.airsim_client = _airsim()        # telemetry reads
        self.airsim_control = _airsim()       # setpoints and blocking moves
        self.airsim_media = _airsim()         # image capture only
        self.fast_lock = threading.Lock()
        self.control_lock = threading.Lock()
        self.media_lock = threading.Lock()
        self.slow_lock = threading.Lock()

        self.world = World(self.carla_client, seed=seed)
        self.vehicle = Vehicle(self.airsim_client)
        self.control = Vehicle(self.airsim_control)
        self.camera = Camera(self.airsim_media)
        self.offset = OFFSETS.get(self.world.map_name, DEFAULT_OFFSET)
        self._last_rgb_shape = None

        # The chase camera is a CARLA sensor, so it costs nothing on the AirSim image path
        # the model depends on. Created lazily: a run that never records never spawns it.
        #
        # It gets its OWN AirSim client, and that is not tidiness. The locks here guard
        # DISPATCH CLASSES GUARD DISPATCH, NOT SOCKETS. Two methods in different classes
        # hold different locks and can still write the SAME msgpack-rpc connection at once.
        # This has now bitten five times, each with an error naming nothing about the cause:
        #   `Existing exports of data: object cannot be re-sized` from inside tornado,
        #   `IOLoop is already running`,
        #   and an uncaught carla::client::TimeoutException that killed the whole sidecar.
        # The rule that actually holds: EVERY method touching a given AirSim client must be
        # in the class that owns that client's lock. self.vehicle -> FAST, self.control ->
        # CONTROL, self.camera -> MEDIA. Check that before adding a method, not after.
        self._chase = None
        self._chase_client = None
        self._chase_vehicle = None
        self._chase_stop = threading.Event()
        self._chase_thread = None
        self._make_airsim = _airsim
        self._landing = None
        self._flying = None

        # CARLA sensors that follow the aircraft, declared in configs/sim/carla_sensors.yaml
        # and spawned once here. They render in-process and push asynchronously, so unlike the
        # AirSim sensors they do not compete with image capture for the RPC path.
        self._rig = CarlaSensorRig(self.world._w)
        self._rig_status = self._rig.spawn()
        if self._rig_status["spawned"]:
            self._ensure_follow_thread()

    # ---------- methods (names must match protocol.METHODS) ----------

    def ping(self):
        return {"ok": True}

    def describe(self):
        info = self.camera.info()
        return {
            "map": self.world.map_name,
            "carla_client_version": self.carla_client.get_client_version(),
            "carla_server_version": self.carla_client.get_server_version(),
            "camera": info,
            "frame_offset": list(self.offset),
            "spawn_points": len(self.world.spawn_points()),
        }

    def reset(self, hold_ned=(0.0, 0.0, -40.0), speed=8.0):
        # self.control, NOT self.vehicle - see the note on CONTROL above.
        # `on_stage` reports each step: this is the slowest call in the protocol and the one
        # a caller most needs to distinguish from a hang.
        return self.control.reset(tuple(hold_ned), speed, on_stage=emit_progress)

    def state(self):
        return self.vehicle.state()

    def sensors(self):
        return self.vehicle.sensors()

    def collision(self):
        return self.vehicle.collision()

    def capture(self, rgb=True, depth=True, segmentation=False):
        imgs = self.camera.capture(rgb=rgb, depth=depth, segmentation=segmentation)
        if imgs.get("rgb") is not None:
            self._last_rgb_shape = imgs["rgb"].shape
        return {
            "images": {k: protocol.encode_image(v) for k, v in imgs.items()},
            "camera": self.camera.info(),
        }

    def ground(self, u, v, rgb_shape=None):
        """Pixel -> world NED. Captures its own depth so the frame is current."""
        shape = tuple(rgb_shape) if rgb_shape else self._last_rgb_shape
        if shape is None:
            raise RuntimeError("no RGB frame seen yet — call capture() first or pass rgb_shape")
        imgs = self.camera.capture(rgb=False, depth=True)
        if imgs["depth"] is None:
            raise RuntimeError("depth capture returned an empty buffer")
        return self.camera.ground(int(u), int(v), shape, imgs["depth"])

    def velocity(self, vx, vy, vz, duration=0.5, yaw_deg=None):
        self.control.velocity(vx, vy, vz, duration, yaw_deg)
        return {"sent": True}

    def goto(self, x, y, z, speed=6.0, settle_s=0.0):
        return self.control.goto(x, y, z, speed, settle_s)

    def attitude(self, roll, pitch, yaw, z, duration=0.2):
        return self.control.attitude(roll, pitch, yaw, z, duration)

    def yaw(self, deg, timeout_s=10.0):
        return self.control.yaw(deg, timeout_s)

    def hold(self):
        return self.control.hold()

    def takeoff(self, altitude_ned=-30.0, speed=6.0):
        """Arm and climb, returning immediately — the mirror of `land`.

        Blocking for the whole climb would freeze the console exactly as `land` used to, so
        the same treatment: control client, background thread, "started" not "finished".
        """
        if self._flying is not None and self._flying.is_alive():
            return {"takeoff": True, "already": True}

        def _run():
            try:
                self.control.takeoff(altitude_ned=altitude_ned, speed=speed)
            except Exception:  # noqa: BLE001 — a failed takeoff must not kill the sidecar
                pass

        self._flying = threading.Thread(target=_run, daemon=True)
        self._flying.start()
        return {"takeoff": True, "altitude_ned": altitude_ned}

    def land(self):
        """Start a landing and return immediately.

        `landAsync().join()` blocks for the whole descent — tens of seconds from 55 m. Holding
        an RPC open that long makes the console look hung, and the disarm afterwards cannot
        simply be dropped: cutting power mid-descent turns a landing into a fall. So the
        blocking part runs on a thread and the reply is "started", not "finished".

        On the CONTROL client, not telemetry. `self.vehicle` is the telemetry client, which
        `state` also drives under a DIFFERENT lock — two locks on one msgpack-rpc socket is
        exactly the race that produced "Existing exports of data: object cannot be re-sized"
        earlier today.
        """
        if self._landing is not None and self._landing.is_alive():
            return {"landing": True, "already": True}

        def _run():
            try:
                self.control.land()
            except Exception:  # noqa: BLE001 — a failed landing must not kill the sidecar
                pass

        self._landing = threading.Thread(target=_run, daemon=True)
        self._landing.start()
        return {"landing": True}
        return {"landed": True}

    def set_camera_pose(self, xyz=(0.5, 0.0, 0.1), pitch=0.0, roll=0.0, yaw=0.0):
        self.camera.set_pose(tuple(xyz), pitch, roll, yaw)
        return self.camera.info()

    def view_jpeg(self, quality=75, width=0):
        """The drone's own camera as JPEG bytes — for the web console's live view.

        Encoding on this side rather than shipping a raw array keeps a 640x480 frame at
        roughly 40 kB instead of 900 kB over the socket. Runs on the MEDIA client, so it
        queues behind other captures rather than stalling telemetry.

        **Takes no lock.** The dispatcher already holds `slow_lock` for any method outside
        FAST and CONTROL, and `threading.Lock` is not reentrant — acquiring it here
        deadlocked the sidecar permanently on the first call, taking every other slow method
        down with it because the lock was never released.

        This is a real AirSim capture and therefore competes with the ROS graph for the
        image path.
        """
        import cv2  # noqa: PLC0415

        frame = self.camera.capture(rgb=True, depth=False, segmentation=False)["rgb"]
        if width and width < frame.shape[1]:
            h = int(frame.shape[0] * width / frame.shape[1])
            frame = cv2.resize(frame, (int(width), h), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
        if not ok:
            raise RuntimeError("failed to encode the onboard frame")
        return {"jpeg": buf.tobytes(), "shape": list(frame.shape[:2])}

    def chase_view(self, width=1280, height=720, fps=30.0, distance=14.0, above=6.0):
        """Bring the chase camera up for LIVE viewing, without recording to a file.

        Separate entry point from `chase_start` so the console can show a picture without
        creating an mp4 nobody asked for. Recording can still be layered on afterwards.

        **30 Hz is free; 60 is not.** Every chase frame is a full extra render pass. Measured
        against the simulator's own tick rate, which is what actually matters:

            chase off        sim 60.0 fps
            30 Hz @ 1080p    sim 59.8 fps    <- no measurable cost
            60 Hz @ 1080p    sim 49.5 fps    <- steals ~10 fps from the simulation
            30 Hz @  720p    sim 59.8 fps
            60 Hz @  720p    sim 56.0 fps

        Real-time factor stays 1.000 throughout, which is the trap: CARLA stretches
        `delta_seconds` to keep simulated time tracking wall time, so a slowed simulator looks
        healthy on the RTF and is quietly integrating physics on a coarser timestep
        (16.7 ms -> 20.3 ms at 60 Hz/1080p). Judge this by tick rate, not by RTF.
        """
        self._ensure_chase(width, height, fps, distance, above)
        return {"streaming": True, "size": [width, height], "fps": fps}

    def chase_jpeg(self, quality=75):
        if self._chase is None:
            return {"jpeg": None}
        return {"jpeg": self._chase.latest_jpeg(quality)}

    def _ensure_follow_thread(self):
        """One thread drives the chase camera AND every CARLA sensor from a single pose read.

        A thread each would mean a pose read each, on a client that exists precisely so those
        reads stay cheap.
        """
        if self._chase_vehicle is None:
            self._chase_client = self._make_airsim()
            self._chase_vehicle = Vehicle(self._chase_client)
        if self._chase_thread is None or not self._chase_thread.is_alive():
            self._chase_stop.clear()
            self._chase_thread = threading.Thread(target=self._chase_follow, daemon=True)
            self._chase_thread.start()

    def carla_sensors(self):
        """What is spawned, from what config, and whether anything failed."""
        return self._rig.describe()

    def lidar(self):
        """Newest semantic LiDAR sweep as raw bytes plus how to decode it.

        Returns None when no lidar is configured or nothing has arrived yet — the caller
        publishes nothing rather than an empty cloud, which would look like a clear sky.
        """
        sensor = self._rig.sensors.get("lidar")
        if sensor is None:
            return None
        return semantic_lidar_payload(sensor.drain())

    def _ensure_chase(self, width, height, fps, distance, above):
        # Rebuild when the requested spec differs. Returning early on "a camera exists"
        # silently served whatever the FIRST caller asked for: open the live view at 720p/30,
        # then start a 1080p/10 recording, and you got a 720p/30 file with no error anywhere.
        # A CARLA sensor's resolution and tick are fixed at spawn, so the only honest way to
        # honour a new spec is a new sensor.
        want = (int(width), int(height), float(fps))
        if self._chase is not None:
            have = (self._chase.width, self._chase.height, self._chase.fps)
            if have != want:
                self._chase.destroy()
                self._chase = None
        if self._chase is None:
            self._chase = ChaseCamera(self.world._w, width=width, height=height, fps=fps,
                                      distance=distance, above=above)
        # Own connection -> own socket -> no interleaving with telemetry or reset.
        self._ensure_follow_thread()

    def _chase_defaults(self):
        """`sidecar.chase_camera` from configs/testbed.yaml.

        This block existed and was READ BY NOTHING until 2026-08-03: the resolution was
        hardcoded in three places (here, the bridge, run_episode), so setting 1920x1080 in
        the config produced a 1280x720 recording and no error anywhere. Exactly the "a config
        change has no effect" failure the quickstart warns about, shipped in the config file
        that is supposed to be the single source.
        """
        import os
        import yaml
        path = os.environ.get("TESTBED_CONFIG") or os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "configs", "testbed.yaml")
        out = {"width": 1280, "height": 720, "fps": 30.0, "distance": 14.0, "above": 6.0,
               "crf": 26}
        try:
            with open(path) as fh:
                block = ((yaml.safe_load(fh) or {}).get("sidecar") or {}).get("chase_camera") or {}
            out.update({k: v for k, v in block.items() if k in out})
        except Exception:                     # noqa: BLE001 - a bad edit must not stop a run
            pass
        return out

    def chase_start(self, path, width=0, height=0, fps=0.0,
                    distance=None, above=None):
        """Begin recording an HD exterior view that follows the aircraft.

        Separate from `capture()` on purpose: that one feeds the model and is a measurement
        surface, this one is a spectator and is scored on nothing.
        """
        # Two different sentinels, because the fields differ in what counts as a real value.
        #
        # width/height/fps: 0 means "the config decides". A zero resolution or frame rate is
        # meaningless, so the sentinel can never collide with something a caller wanted, and
        # ChaseRecording.srv documents 0 that way for its int32/float64 fields.
        #
        # distance/above: None, NOT 0. `above=0` is a perfectly sensible request - a chase
        # camera level with the aircraft rather than looking down at it - and treating it as
        # "unset" silently substituted the config's 6.0, giving the caller a shot they did
        # not ask for with nothing logged. A sentinel must be a value the caller cannot
        # legitimately mean.
        d = self._chase_defaults()
        width = int(width) or int(d["width"])
        height = int(height) or int(d["height"])
        fps = float(fps) or float(d["fps"])
        distance = float(d["distance"]) if distance is None else float(distance)
        above = float(d["above"]) if above is None else float(above)
        self._ensure_chase(width, height, fps, distance, above)
        self._chase.start(path)
        return {"recording": path, "size": [width, height], "fps": fps}

    def chase_stop(self):
        if self._chase is None:
            return {"frames": 0, "dropped": 0}
        self._chase_stop.set()
        if self._chase_thread is not None:
            # 5 s was not enough at 1080p: the encoder still has a queue to flush when the
            # follower stops, the join timed out, `stop()` never ran, and the file was left
            # without a moov atom - i.e. unplayable, which is the one outcome a recording
            # must not have. Measured: a 78 s 1080p clip needs ~15 s to drain.
            self._chase_thread.join(timeout=45.0)
            self._chase_thread = None
        return self._chase.stop()

    def _chase_follow(self, hz=20.0):
        """Drive the camera from the aircraft's pose, on a connection nothing else uses.

        No lock is taken deliberately: this thread is the only user of `_chase_client`, so
        there is nothing to serialise against. Sharing the telemetry client instead — even
        under `fast_lock` — races `reset`, which drives the same socket under `slow_lock`.

        A failure here must never stop a flight; the camera simply stops following.
        """
        period = 1.0 / hz
        while not self._chase_stop.wait(period):
            try:
                st = self._chase_vehicle.state()
                pos, yaw = st["position"], quat_to_yaw(*st["orientation"])
                if self._chase is not None:
                    self._chase.follow(pos, yaw)
                self._rig.follow(pos, yaw)
            except Exception:  # noqa: BLE001
                continue

    def spawn_traffic(self, vehicles=15, walkers=10, near_ned=None, radius_m=150.0):
        return self.world.spawn_traffic(vehicles, walkers,
                                        near_ned=near_ned, radius_m=radius_m)

    def traffic_stats(self):
        return self.world.traffic_stats()

    def watchdog(self):
        return {"restarted": self.world.tick_watchdog()}

    def set_weather(self, preset="ClearNoon"):
        return {"weather": self.world.set_weather(preset)}

    def destroy_actors(self):
        return {"destroyed": self.world.destroy_all()}

    def carla_to_ned(self, x, y, z=0.0):
        return {"ned": list(carla_to_ned(x, y, z, self.offset))}

    def shutdown(self):
        return {"bye": True}


#: Where a running method finds its own connection, so it can report progress without
#: every method signature growing a `send` argument. Thread-local because each connection is
#: served by its own thread: a module global would let one connection's progress frames land
#: on another's socket, which is the same class of bug this whole change exists to remove.
_CURRENT = threading.local()


def emit_progress(stage: str) -> None:
    """Tell the caller this call is still alive, and what it is doing.

    Long operations - `reset` above all - used to be indistinguishable from wedged ones: the
    caller had a fixed ceiling and no information, so the ceiling had to be generous enough
    for the worst case, which made a genuine hang take just as long to notice. A progress
    frame resets the caller's patience clock, so slow and dead stop looking alike.

    Best-effort by construction. A caller that has already given up is gone, and failing to
    tell it about a stage it no longer cares about must never fail the operation itself.
    """
    conn = getattr(_CURRENT, "conn", None)
    rid = getattr(_CURRENT, "rid", None)
    if conn is None or rid is None:
        return
    try:
        protocol.send(conn, {"id": rid, "progress": stage})
    except OSError:
        pass


def _handle(bridge: SimBridge, conn: socket.socket):
    """One connection, one thread. Locks are per underlying RPC client, not global."""
    try:
        while True:
            req = protocol.recv(conn)
            rid, method = req.get("id"), req.get("method")
            args = req.get("args") or {}
            if method not in protocol.METHODS:
                protocol.send(conn, {"id": rid, "ok": False,
                                     "error": f"unknown method {method!r}"})
                continue
            if method in bridge.FAST:
                lock = bridge.fast_lock
            elif method in bridge.CONTROL:
                lock = bridge.control_lock
            elif method in bridge.MEDIA:
                lock = bridge.media_lock
            else:
                lock = bridge.slow_lock
            _CURRENT.conn, _CURRENT.rid = conn, rid
            try:
                # Time the WAIT separately from the WORK. D-04 is a `destroy` that stops
                # answering while the sidecar is alive, and those two numbers tell apart the
                # two explanations: destroy itself being slow, versus destroy queued behind a
                # 1 Hz world tick that holds the same lock and grows with actor count.
                _t0 = time.monotonic()
                with lock:
                    _waited = time.monotonic() - _t0
                    _t1 = time.monotonic()
                    result = getattr(bridge, method)(**args)
                _worked = time.monotonic() - _t1
                if _waited > 1.0 or _worked > 5.0:
                    print(f"slow rpc: {method} waited {_waited:.1f}s for the lock, "
                          f"ran {_worked:.1f}s", flush=True)
                protocol.send(conn, {"id": rid, "ok": True, "result": result})
            except Exception as exc:  # noqa: BLE001 — must not kill the server
                protocol.send(conn, {"id": rid, "ok": False, "error": str(exc),
                                     "traceback": traceback.format_exc()})
            finally:
                _CURRENT.conn = _CURRENT.rid = None
            if method == "shutdown":
                return
    except (ConnectionError, protocol.ProtocolError, OSError) as exc:
        print(f"client gone: {exc}", flush=True)
    finally:
        conn.close()


def serve(bridge: SimBridge, path: str):
    if os.path.exists(path):
        os.unlink(path)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(path)
    os.chmod(path, 0o600)
    srv.listen(8)
    print(f"sim_bridge listening on {path}", flush=True)

    try:
        while True:
            conn, _ = srv.accept()
            conn.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
            print("client connected", flush=True)
            threading.Thread(target=_handle, args=(bridge, conn), daemon=True).start()
    finally:
        srv.close()
        if os.path.exists(path):
            os.unlink(path)


def _install_stack_dumper():
    """`kill -USR1 <sidecar pid>` prints every thread's stack to the sidecar log.

    Added after D-04: the sidecar stopped answering `destroy` twice in one session while
    `status.sh` still showed it running with its socket present. A wedge is invisible to a
    process count, and without this the only way to ask what it was doing was to guess.

    faulthandler writes to fd 2 from a signal handler, so it works even when every Python
    thread is blocked in a C call, which is exactly the case worth catching.
    """
    import faulthandler
    import signal
    faulthandler.enable()
    if hasattr(faulthandler, "register"):
        faulthandler.register(signal.SIGUSR1, all_threads=True, chain=False)
        print("stack dumper: kill -USR1 %d to dump all thread stacks" % os.getpid(),
              flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--socket", default=protocol.DEFAULT_SOCKET)
    ap.add_argument("--carla-port", type=int, default=2000)
    ap.add_argument("--airsim-port", type=int, default=41451)
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args()

    _install_stack_dumper()
    bridge = SimBridge(args.carla_port, args.airsim_port, args.seed)
    d = bridge.describe()
    print(f"connected: map={d['map']} camera_hfov={d['camera']['hfov_deg']:.1f} "
          f"offset={d['frame_offset']}", flush=True)
    serve(bridge, args.socket)


if __name__ == "__main__":
    main()
