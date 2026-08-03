"""CARLA sensors that follow the aircraft, declared in configuration rather than in code.

CARLA has no actor for an AirSim vehicle, so nothing can be *attached* to the drone. What
works instead is what the chase camera already proved: spawn the sensor free-floating and
rewrite its transform each tick from the aircraft's NED pose. That makes "a sensor on the
drone" a matter of bookkeeping, and once it is bookkeeping it may as well be a config file.

**Why these live on the CARLA side at all**, when AirSim can also do LiDAR: AirSim's sensor
API is RPC request/response on the same path as image capture, which is this project's
bottleneck — publishing four cheap AirSim sensors at 20 Hz already cost 28% of the RGB rate.
A CARLA sensor renders inside the same UE4 process and **pushes** its output asynchronously,
so it never queues behind a `simGetImages` call.

A scanning sensor emits an ARC per simulation tick, not a sweep, so keeping only the newest
measurement would hand a consumer a thin slice while looking like a full view. Arcs are
accumulated between fetches and drained together — bounded, because an unbounded queue fed by
a simulation thread is how a consumer that stops reading takes the simulator down with it.
"""
from __future__ import annotations

import os
import threading

import carla

from .frames import ned_to_carla

DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "configs", "testbed.yaml")


class FollowedSensor:
    """One CARLA sensor, spawned free-floating and dragged along behind the aircraft."""

    def __init__(self, world, name, blueprint_id, offset, attributes):
        self.name = name
        self.blueprint_id = blueprint_id
        self.offset = offset                      # NED body-frame metres
        self.latest = None                        # newest measurement, for non-scanning sensors
        self.pending = []                         # arcs accumulated since the last take()
        self.count = 0
        self._lock = threading.Lock()

        blueprint = world.get_blueprint_library().find(blueprint_id)
        for key, value in (attributes or {}).items():
            if not blueprint.has_attribute(key):
                # Loud rather than silent: CARLA ignores unknown attributes, so a typo in the
                # config would otherwise present as a sensor that quietly behaves differently
                # from what the file says.
                raise ValueError(
                    f"{blueprint_id} has no attribute {key!r} — check configs/testbed.yaml")
            blueprint.set_attribute(key, str(value))

        self.actor = world.spawn_actor(blueprint, carla.Transform())
        self.actor.listen(self._on_data)

    #: How many measurements to hold between fetches. A rotating LiDAR emits an ARC per
    #: simulation tick, not a sweep, so keeping only the newest throws away five of every six
    #: at a 10 Hz fetch against a 60 Hz sim — the consumer then sees a thin slice and has no
    #: way to know the rest was discarded. Bounded so a consumer that stops fetching costs
    #: memory that stops growing rather than memory that does not.
    MAX_PENDING = 24

    def _on_data(self, measurement):
        """CARLA's sensor thread. Store and return — never process here."""
        with self._lock:
            self.latest = measurement
            self.pending.append(measurement)
            if len(self.pending) > self.MAX_PENDING:
                del self.pending[0]
            self.count += 1

    def take(self):
        """The newest measurement only."""
        with self._lock:
            return self.latest

    def drain(self):
        """Everything accumulated since the last call, and clear.

        For a scanning sensor this is what "the current view" actually means: the union of the
        arcs swept since you last looked.
        """
        with self._lock:
            out, self.pending = self.pending, []
            return out

    def follow(self, ned_xyz, yaw_rad):
        import math

        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        ox, oy, oz = self.offset
        # Rotate the body-frame offset into NED by the aircraft's yaw, then convert. Skipping
        # the rotation would leave a forward-mounted sensor pointing north regardless of
        # which way the aircraft faces.
        n = ned_xyz[0] + ox * c - oy * s
        e = ned_xyz[1] + ox * s + oy * c
        d = ned_xyz[2] + oz
        x, y, z = ned_to_carla(n, e, d)
        self.actor.set_transform(carla.Transform(
            carla.Location(x=x, y=y, z=z),
            carla.Rotation(pitch=0.0, yaw=math.degrees(yaw_rad), roll=0.0)))

    def describe(self):
        return {"name": self.name, "blueprint": self.blueprint_id,
                "measurements": self.count, "has_data": self.latest is not None}

    def destroy(self):
        try:
            self.actor.stop()
        finally:
            self.actor.destroy()


class CarlaSensorRig:
    """Every configured sensor, spawned once and followed together."""

    def __init__(self, world, config_path=None):
        self.world = world
        self.path = config_path or os.environ.get("TESTBED_CARLA_SENSORS", DEFAULT_CONFIG)
        self.sensors: dict[str, FollowedSensor] = {}
        self.errors: list[str] = []

    def spawn(self):
        """Spawn everything the config enables. Returns what came up, and what did not.

        A sensor that fails to spawn is recorded and skipped rather than raised: one bad
        attribute in a config file should cost that sensor, not the whole flight.
        """
        if not os.path.exists(self.path):
            return {"spawned": [], "errors": [f"no config at {self.path}"]}

        import yaml

        with open(self.path) as fh:
            config = yaml.safe_load(fh) or {}

        # One `sensors:` list covers both simulators; `source` says who provides each. The
        # airsim ones are polled by the bridge over RPC and are none of this class's business.
        for entry in config.get("sensors", []):
            if not entry.get("enabled") or entry.get("source") != "carla":
                continue
            name = entry.get("name") or entry.get("blueprint")
            try:
                off = entry.get("offset") or {}
                self.sensors[name] = FollowedSensor(
                    self.world, name, entry["blueprint"],
                    (float(off.get("x", 0.0)), float(off.get("y", 0.0)), float(off.get("z", 0.0))),
                    entry.get("attributes"))
            except Exception as exc:  # noqa: BLE001
                self.errors.append(f"{name}: {exc}")
        return {"spawned": sorted(self.sensors), "errors": self.errors}

    def follow(self, ned_xyz, yaw_rad):
        for sensor in self.sensors.values():
            try:
                sensor.follow(ned_xyz, yaw_rad)
            except Exception:  # noqa: BLE001
                # Deliberately broad. `carla::client::TimeoutException` surfaces here when the
                # simulator is too busy to answer, and it is NOT a RuntimeError — an aggressive
                # LiDAR made CARLA's RPC time out at 30 s and the escaping exception terminated
                # the entire sidecar, taking a flight with it. A sensor that cannot be moved
                # this tick is worth losing; the process is not.
                pass

    def describe(self):
        return {"config": self.path,
                "sensors": [s.describe() for s in self.sensors.values()],
                "errors": self.errors}

    def destroy(self):
        for sensor in self.sensors.values():
            try:
                sensor.destroy()
            except RuntimeError:
                pass
        self.sensors.clear()


def semantic_lidar_payload(measurements):
    """One or more semantic LiDAR arcs as plain bytes, plus the metadata to decode them.

    Takes a LIST because a rotating LiDAR emits an arc per simulation tick, and the union of
    the arcs since the last fetch is the honest answer to "what can the aircraft see". Passing
    a single measurement would publish a slice while looking like a sweep.

    Handed across the interpreter seam as the raw buffer rather than a list of tuples: at
    120000 points/second a Python list would be roughly 40x the bytes and would be built and
    torn down 20 times a second on the sidecar's thread.

    Each detection is 24 bytes — three float32 for the point, one float32 for the cosine of
    the incidence angle, then two uint32 for the object index and its semantic tag. Points are
    in CARLA's own frame relative to the sensor: x forward, y RIGHT, z UP.
    """
    if not measurements:
        return None
    if not isinstance(measurements, (list, tuple)):
        measurements = [measurements]
    raw = b"".join(bytes(m.raw_data) for m in measurements)
    if not raw:
        return None
    newest = measurements[-1]
    return {
        "raw": raw,
        "stride": 24,
        "count": len(raw) // 24,
        "arcs": len(measurements),
        "fields": ["x", "y", "z", "cos_inc_angle", "object_idx", "object_tag"],
        "frame": int(getattr(newest, "frame", 0)),
        "t": float(getattr(newest, "timestamp", 0.0)),
    }
