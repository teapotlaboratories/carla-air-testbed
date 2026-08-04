#!/usr/bin/env python3
"""Set up a scenario from plain ROS 2 — no sidecar socket, no carla, no airsim.

    ./scripts/bringup.sh --config configs/testbed.yaml --backend geometric      # terminal 1
    python3 examples/ros2_world_control.py        # terminal 2, ROS 2 python (3.12)

The companion to `ros2_full_control.py`, which flies the aircraft. This one moves the *world*:
teleport, traffic, weather, teardown — the four things every scenario does before anything
takes off, and the four that until now needed the Unix socket.

Imports are the point. `rclpy`, `interfaces` and `std_msgs` — nothing from this project's
3.10 side. If that list ever grows a `sim_bridge`, the ROS surface is not closed.

Why services and not topics: each of these has a failure the caller must see. An unknown
weather preset, a map that refused half the spawn points, a reset that could not reach its
pose. A topic drops those on the floor and the scenario scores a number that means nothing.
"""
from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from interfaces.msg import Collision
from interfaces.srv import DestroyActors, ResetVehicle, SetWeather, SpawnTraffic

#: `reset` arms the aircraft, flies it to the pose and settles — **measured 16.2 s** for a
#: ~60 m move, not the ~5 s it looks like. A default 5 s timeout expires mid-flight and
#: reports a failure that did not happen.
RESET_TIMEOUT_S = 60.0
DEFAULT_TIMEOUT_S = 30.0


class WorldControl(Node):
    def __init__(self):
        super().__init__("ros2_world_control_example")
        # NOT `self.clients` - rclpy's Node already owns that name as a read-only
        # property, and assigning to it raises AttributeError at construction.
        self.srv_clients = {
            "reset": self.create_client(ResetVehicle, "/sim/reset_vehicle"),
            "traffic": self.create_client(SpawnTraffic, "/sim/spawn_traffic"),
            "weather": self.create_client(SetWeather, "/sim/set_weather"),
            "destroy": self.create_client(DestroyActors, "/sim/destroy_actors"),
        }
        # Read-side state. Already on ROS before this example existed; shown here so the
        # example covers the whole world surface rather than only the new half.
        self.collided = None
        self.hit_what = ""
        self.traffic_line = None
        self.create_subscription(Collision, "/sim/collision", self._on_collision, 5)
        self.create_subscription(String, "/sim/traffic_stats", self._on_traffic, 5)

    def _on_collision(self, msg):
        self.collided = msg.has_collided
        self.hit_what = msg.object_name

    def _on_traffic(self, msg):
        self.traffic_line = msg.data

    def wait_for_services(self, timeout_s=30.0):
        missing = [n for n, c in self.srv_clients.items()
                   if not c.wait_for_service(timeout_sec=timeout_s)]
        if missing:
            raise SystemExit(
                f"no such service: {', '.join(missing)}\n"
                "Is the bridge up?  ./scripts/bringup.sh --config configs/testbed.yaml\n"
                f"And is ROS_DOMAIN_ID=42?")

    def call(self, name, request, timeout_s=DEFAULT_TIMEOUT_S):
        """Blocking call that returns the response, or None on timeout."""
        future = self.srv_clients[name].call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done():
            self.get_logger().error(f"{name}: timed out after {timeout_s}s")
            return None
        return future.result()

    def pump(self, seconds):
        """Spin for a while so the subscriptions actually receive. Both publish at 1 Hz."""
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    node = WorldControl()
    failures = []

    def check(label, ok, detail=""):
        print(f"  {'PASS' if ok else 'FAIL'}  {label}{' — ' + detail if detail else ''}",
              flush=True)
        if not ok:
            failures.append(label)

    try:
        node.wait_for_services()

        # ---- 1. teardown first, so a leftover graph does not skew the counts ----
        print("\n[1/6] destroy any leftover actors")
        r = node.call("destroy", DestroyActors.Request())
        check("destroy_actors", r is not None and r.success,
              f"destroyed {r.destroyed}" if r else "no response")

        # ---- 2. weather, including the failure path ----
        print("\n[2/6] set_weather ClearNoon")
        r = node.call("weather", SetWeather.Request(preset="ClearNoon"))
        check("set_weather", r is not None and r.success and r.applied == "ClearNoon",
              f"applied={r.applied!r}" if r else "no response")

        print("\n[3/6] set_weather with a bogus preset — MUST fail, not fall back")
        r = node.call("weather", SetWeather.Request(preset="NoSuchWeather"))
        check("bogus preset rejected", r is not None and not r.success,
              (r.message[:70] if r else "no response"))

        # ---- 3. reset ----
        print("\n[4/6] reset_vehicle to [60, -100, -60] (blocking, ~16 s)")
        req = ResetVehicle.Request()
        req.hold_ned = [60.0, -100.0, -60.0]
        req.speed = 10.0
        r = node.call("reset", req, timeout_s=RESET_TIMEOUT_S)
        if r is None or not r.success:
            check("reset_vehicle", False, r.message if r else "no response")
        else:
            pos = list(r.position_ned)
            err = sum((a - b) ** 2 for a, b in zip(pos, req.hold_ned)) ** 0.5
            # The aircraft relaxes a few metres off a setpoint; that is the station-keeping
            # floor, not a failed reset, which is why scenarios use a 20 m success radius.
            check("reset_vehicle", err < 20.0,
                  f"settled {[round(v, 1) for v in pos]}, {err:.1f} m from commanded")

        # ---- 4. traffic ----
        print("\n[5/6] spawn_traffic 12 vehicles + 8 walkers near the reset pose")
        req = SpawnTraffic.Request()
        req.vehicles, req.walkers = 12, 8
        req.near_ned = [60.0, -100.0, -60.0]
        req.radius_m = 150.0
        r = node.call("traffic", req, timeout_s=60.0)
        check("spawn_traffic", r is not None and r.success and r.spawned > 0,
              f"{r.spawned} vehicles, {r.walkers_spawned} walkers"
              f"{' — ' + r.message if r and r.message else ''}" if r else "no response")

        # ---- 5. the read side ----
        print("\n[6/6] read /sim/collision and /sim/traffic_stats (1 Hz, so ~3 s)")
        node.pump(3.5)
        check("collision topic", node.collided is not None, f"has_collided={node.collided}")
        check("traffic_stats topic", node.traffic_line is not None, node.traffic_line or "")

    finally:
        # Leave the map as we found it. A scenario that inherits the previous run's traffic
        # is how two episodes with the same seed diverge.
        print("\ncleanup: destroy_actors")
        try:
            r = node.call("destroy", DestroyActors.Request(), timeout_s=30.0)
            print(f"  destroyed {r.destroyed}" if r and r.success else "  cleanup failed")
        except Exception as exc:                                  # noqa: BLE001
            print(f"  cleanup failed: {exc}")
        node.destroy_node()
        rclpy.try_shutdown()

    print("\n" + ("FAILED: " + ", ".join(failures) if failures else "all checks passed"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
