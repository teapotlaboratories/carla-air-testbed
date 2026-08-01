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
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import airsim  # noqa: E402
import carla  # noqa: E402

import protocol  # noqa: E402
from carla_air.camera import Camera  # noqa: E402
from carla_air.frames import OFFSETS, DEFAULT_OFFSET, carla_to_ned  # noqa: E402
from carla_air.vehicle import Vehicle  # noqa: E402
from carla_air.world import World  # noqa: E402


class SimBridge:
    #: telemetry reads — must never queue behind capture OR behind a control command
    FAST = frozenset({"ping", "state", "collision", "carla_to_ned"})
    #: control writes — their own client so a 10 Hz setpoint stream cannot starve telemetry
    CONTROL = frozenset({"velocity", "hold", "goto", "yaw"})

    def __init__(self, carla_port=2000, airsim_port=41451, seed=None, timeout=30.0):
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
        self.slow_lock = threading.Lock()

        self.world = World(self.carla_client, seed=seed)
        self.vehicle = Vehicle(self.airsim_client)
        self.control = Vehicle(self.airsim_control)
        self.camera = Camera(self.airsim_media)
        self.offset = OFFSETS.get(self.world.map_name, DEFAULT_OFFSET)
        self._last_rgb_shape = None

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
        return self.vehicle.reset(tuple(hold_ned), speed)

    def state(self):
        return self.vehicle.state()

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

    def yaw(self, deg, timeout_s=10.0):
        return self.control.yaw(deg, timeout_s)

    def hold(self):
        return self.control.hold()

    def land(self):
        self.vehicle.land()
        return {"landed": True}

    def set_camera_pose(self, xyz=(0.5, 0.0, 0.1), pitch=0.0, roll=0.0, yaw=0.0):
        self.camera.set_pose(tuple(xyz), pitch, roll, yaw)
        return self.camera.info()

    def spawn_traffic(self, vehicles=15, walkers=10):
        return self.world.spawn_traffic(vehicles, walkers)

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
            else:
                lock = bridge.slow_lock
            try:
                with lock:
                    result = getattr(bridge, method)(**args)
                protocol.send(conn, {"id": rid, "ok": True, "result": result})
            except Exception as exc:  # noqa: BLE001 — must not kill the server
                protocol.send(conn, {"id": rid, "ok": False, "error": str(exc),
                                     "traceback": traceback.format_exc()})
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


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--socket", default=protocol.DEFAULT_SOCKET)
    ap.add_argument("--carla-port", type=int, default=2000)
    ap.add_argument("--airsim-port", type=int, default=41451)
    ap.add_argument("--seed", type=int, default=None)
    args = ap.parse_args()

    bridge = SimBridge(args.carla_port, args.airsim_port, args.seed)
    d = bridge.describe()
    print(f"connected: map={d['map']} camera_hfov={d['camera']['hfov_deg']:.1f} "
          f"offset={d['frame_offset']}", flush=True)
    serve(bridge, args.socket)


if __name__ == "__main__":
    main()
