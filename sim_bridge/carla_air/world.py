"""The CARLA side: weather, traffic, pedestrians — the shared world under the aircraft.

Two things here are not optional, both learned the hard way:

* **Spawn traffic with batched `SpawnActor().then(SetAutopilot(...))`.** Spawning first and
  calling `set_autopilot()` afterwards leaves the vehicles parked.
* **Run the watchdog.** Traffic-manager vehicles stall intermittently — two runs of the same
  probe gave 4/15 and 11/15 vehicles moving. Upstream's own `auto_traffic.py` ships a
  health check for exactly this; `tick_watchdog()` is that check. Without it, an episode's
  "dense urban traffic" may be a car park.

Pedestrians need a `controller.ai.walker` **and** a `go_to_location`; a bare walker is a
statue.
"""
from __future__ import annotations

import math
import os
import random

import carla

from .frames import carla_to_ned


class World:
    def __init__(self, client: carla.Client, seed: int | None = None):
        self._c = client
        self._w = client.get_world()
        self._tm = client.get_trafficmanager()
        self._rng = random.Random(seed)
        self.vehicle_ids: list[int] = []
        self.walker_ids: list[int] = []
        self.controller_ids: list[int] = []
        if seed is not None:
            self._tm.set_random_device_seed(seed)

    @property
    def map_name(self) -> str:
        return self._w.get_map().name.split("/")[-1]

    def spawn_points(self):
        return self._w.get_map().get_spawn_points()

    # ---------- traffic ----------

    def spawn_traffic(self, vehicles: int = None, walkers: int = None,
                      near_ned=None, radius_m: float = None):
        """Populate the map, optionally concentrating the traffic around a point.

        Without `near_ned` this shuffles all 155 map-wide spawn points and takes the first
        N — which spreads a small fleet across the whole of Town10HD. Measured: only about
        45 of those points lie within 60 m of the `cross_the_plaza` start, so 15 vehicles
        put roughly **four** cars anywhere near the aircraft, and 10 walkers about **two**.
        The city was populated; the drone's own neighbourhood was not, and the camera only
        ever sees the neighbourhood.

        `near_ned` restricts spawning to a radius around a point — pass the scenario's start
        and the aircraft comes up over moving traffic instead of an empty grid. Selection is
        still shuffled within the radius, so seeds stay meaningful.

        The 70 m default is measured, not guessed. Counting actors within 60 m of the
        `cross_the_plaza` start, with 30 vehicles and 20 walkers requested:

            map-wide          5 cars,  1 walker
            radius 150 m      6 cars,  4 walkers
            radius  90 m     17 cars,  8 walkers
            radius  70 m     20 cars, 17 walkers      <- 28 of 30 moving

        Wider radii spend the fleet on streets the camera never sees.
        """
        cfg = self.traffic_cfg()
        vehicles = int(cfg["vehicles"] if vehicles is None else vehicles)
        walkers = int(cfg["walkers"] if walkers is None else walkers)
        radius_m = float(cfg["radius_m"] if radius_m is None else radius_m)

        bp = self._w.get_blueprint_library()
        self._tm.set_global_distance_to_leading_vehicle(2.5)

        cars = [b for b in bp.filter("vehicle.*") if int(b.get_attribute("number_of_wheels")) == 4]
        points = self.spawn_points()
        clustered, near_candidates = near_ned is None, len(points)
        if near_ned is not None:
            cx, cy = float(near_ned[0]), float(near_ned[1])
            near = [p for p in points
                    if math.hypot(*(v - c for v, c in zip(
                        carla_to_ned(p.location.x, p.location.y, p.location.z)[:2], (cx, cy))))
                    <= radius_m]
            # Fall back rather than spawn nothing: a scenario sited away from roads should
            # still get traffic, just not clustered. REPORTED, not silent — the difference
            # is 20 cars within 60 m of the aircraft versus 5, and a caller that asked for
            # a busy neighbourhood and quietly got a sparse city scores a number that does
            # not mean what it looks like.
            clustered = len(near) >= vehicles
            near_candidates = len(near)
            points = near if clustered else points
        self._rng.shuffle(points)
        batch = []
        for i, sp in enumerate(points[:vehicles]):
            b = cars[i % len(cars)]
            if b.has_attribute("color"):
                b.set_attribute("color", self._rng.choice(
                    b.get_attribute("color").recommended_values))
            b.set_attribute("role_name", "autopilot")
            batch.append(carla.command.SpawnActor(b, sp).then(
                carla.command.SetAutopilot(carla.command.FutureActor, True, self._tm.get_port())))
        self.vehicle_ids = [r.actor_id for r in self._c.apply_batch_sync(batch, False) if not r.error]

        wbps = bp.filter("walker.pedestrian.*")
        wbatch = []
        walker_spots = []
        for i in range(walkers):
            # The navmesh sampler is map-wide with no location argument, so "near the
            # aircraft" has to be rejection-sampled. Capped: an unreachable radius must cost
            # a few wasted draws, not an unbounded loop inside episode setup.
            loc = None
            for _ in range(60 if near_ned is not None else 1):
                candidate = self._w.get_random_location_from_navigation()
                if candidate is None:
                    continue
                if near_ned is None:
                    loc = candidate
                    break
                n, e, _ = carla_to_ned(candidate.x, candidate.y, candidate.z)
                if math.hypot(n - float(near_ned[0]), e - float(near_ned[1])) <= radius_m:
                    loc = candidate
                    break
            if loc is None:
                continue
            walker_spots.append(loc)
            wbatch.append(carla.command.SpawnActor(wbps[i % len(wbps)], carla.Transform(loc)))
        self.walker_ids = [r.actor_id for r in self._c.apply_batch_sync(wbatch, True) if not r.error]

        # NOT controller.ai.walker.
        #
        # It is accepted by this build and does nothing. Measured directly against CARLA,
        # bypassing the sidecar entirely, 8 walkers each with a started controller, a
        # destination and a max speed:
        #
        #     WalkerControl applied directly :  6/6 moved, max 7.94 m in 6 s
        #     controller.ai.walker           :  0/8 moved, max 0.00 m in 6 s
        #
        # So the walkers and their physics are fine; the AI half is inert. The likely cause
        # is the client/server drift this fork warns about on every connect (client aa9c92b
        # vs server adaf011-dirty) - the RPC is accepted and silently does nothing.
        #
        # So we steer them. Destinations still come FROM the navmesh, so pedestrians head
        # towards walkable places; the steering between them is a straight line rather than
        # a path, which is crude up close and indistinguishable from the real thing under a
        # drone camera at 20-40 m. See todo.md T-03.
        self.controller_ids = []
        self._walker_targets = {}
        for i, wid in enumerate(self.walker_ids):
            origin = walker_spots[i] if i < len(walker_spots) else None
            self._walker_targets[wid] = {
                "target": self._walker_destination(origin, radius_m if near_ned else None),
                # A FIXED FRACTION of the configured range, not a fixed speed. Widening
                # walker_speed_min/max in the config then changes walkers already walking,
                # while each keeps its own place in the spread so they never move as one.
                "pace": self._rng.random(),
            }
        self._steer_walkers()

        return {"vehicles": len(self.vehicle_ids), "walkers": len(self.walker_ids),
                "clustered": bool(clustered), "near_candidates": int(near_candidates)}

    def _walker_destination(self, near=None, radius_m=None):
        """A navmesh point to head for, near `near` when a radius is given."""
        fallback = None
        for _ in range(30):
            c = self._w.get_random_location_from_navigation()
            if c is None:
                continue
            fallback = fallback or c
            if near is None or radius_m is None or c.distance(near) <= radius_m:
                return c
        return fallback

    def _steer_walkers(self, arrive_m=None):
        """Point each walker at its destination. Cheap enough to run at 1 Hz.

        A new destination is chosen on arrival, so they keep moving rather than stopping in
        place - which is what made the original bug so easy to mistake for "no pedestrians".
        """
        if not self._walker_targets:
            return 0
        cfg = self.traffic_cfg()
        arrive_m = float(cfg["walker_arrive_m"] if arrive_m is None else arrive_m)
        roam_m = float(cfg["walker_roam_m"])
        lo, hi = float(cfg["walker_speed_min"]), float(cfg["walker_speed_max"])
        steered = 0
        for a in self._w.get_actors(list(self._walker_targets)):
            plan = self._walker_targets.get(a.id)
            if plan is None:
                continue
            try:
                here = a.get_location()
                target = plan["target"]
                if target is None or here.distance(target) <= arrive_m:
                    target = plan["target"] = self._walker_destination(here, roam_m)
                    if target is None:
                        continue
                dx, dy = target.x - here.x, target.y - here.y
                n = math.hypot(dx, dy)
                if n < 1e-3:
                    continue
                ctl = carla.WalkerControl()
                ctl.speed = lo + plan["pace"] * (hi - lo)
                ctl.direction = carla.Vector3D(dx / n, dy / n, 0.0)
                a.apply_control(ctl)
                steered += 1
            except RuntimeError:
                pass                          # reaped between calls
        return steered

    def tick_watchdog(self):
        """Re-arm autopilot on stalled vehicles. Call at ~1 Hz for the life of an episode."""
        if not self.vehicle_ids:
            return 0
        # Pedestrians are steered from here too: this is the only thing in the system that
        # already ticks at 1 Hz for the life of an episode.
        self._steer_walkers()
        restarted = 0
        for a in self._w.get_actors(self.vehicle_ids):
            try:
                v = a.get_velocity()
                if (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5 < 0.01:
                    a.set_autopilot(True, self._tm.get_port())
                    restarted += 1
            except RuntimeError:
                pass  # actor reaped by CARLA
        return restarted

    _last_walker_pos: dict = {}
    _walker_targets: dict = {}

    #: Defaults if the config has no `sidecar.traffic` block, so an old config still runs.
    TRAFFIC_DEFAULTS = {
        "vehicles": 15, "walkers": 10, "radius_m": 70.0,
        "walker_speed_min": 1.0, "walker_speed_max": 1.7,
        "walker_arrive_m": 3.0, "walker_roam_m": 80.0,
    }

    def traffic_cfg(self):
        """`sidecar.traffic`, re-read whenever the file changes.

        Re-reading rather than caching at startup is the point: this is called on every
        spawn AND on every 1 Hz steering tick, so editing configs/testbed.yaml changes
        pedestrians that are already walking, within a second, with no restart. An mtime
        check keeps that to one `stat` per tick.

        A broken edit must not stop the world - a half-saved YAML file is a normal thing to
        catch mid-write - so a parse failure keeps the last good values.
        """
        path = os.environ.get("TESTBED_CONFIG") or os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
            "configs", "testbed.yaml")
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return self._traffic_cache or dict(self.TRAFFIC_DEFAULTS)
        if self._traffic_cache is None or mtime != self._traffic_mtime:
            try:
                import yaml
                with open(path) as fh:
                    block = ((yaml.safe_load(fh) or {}).get("sidecar") or {}).get("traffic") or {}
                merged = dict(self.TRAFFIC_DEFAULTS)
                merged.update({k: v for k, v in block.items() if k in merged})
                self._traffic_cache, self._traffic_mtime = merged, mtime
            except Exception:                 # noqa: BLE001 - a bad edit must not stop traffic
                if self._traffic_cache is None:
                    self._traffic_cache = dict(self.TRAFFIC_DEFAULTS)
        return self._traffic_cache

    _traffic_cache = None
    _traffic_mtime = None

    def _walkers_displaced(self, threshold_m=0.05):
        """How many walkers have actually changed position since the last call."""
        moved = 0
        seen = {}
        for a in self._w.get_actors(self.walker_ids):
            try:
                loc = a.get_location()
            except RuntimeError:
                continue                      # reaped between calls
            seen[a.id] = (loc.x, loc.y, loc.z)
            was = self._last_walker_pos.get(a.id)
            if was is not None:
                d = sum((n - o) ** 2 for n, o in zip(seen[a.id], was)) ** 0.5
                if d > threshold_m:
                    moved += 1
        self._last_walker_pos = seen
        return moved

    def traffic_stats(self):
        """Counts, and specifically what is MOVING.

        `spawned` was never the interesting number. Pedestrians that exist but stand still
        are indistinguishable from pedestrians that were never created - in footage and in
        a scenario - and that is exactly what happened: `walkers_moving` was added on
        2026-08-03 after a recording showed 35 spawned pedestrians and not one of them
        walking.
        """
        def moving_count(ids, threshold):
            n = 0
            for a in self._w.get_actors(ids):
                try:
                    v = a.get_velocity()
                    if (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5 > threshold:
                        n += 1
                except RuntimeError:
                    pass                      # actor reaped by CARLA between calls
            return n

        return {"spawned": len(self.vehicle_ids),
                "moving": moving_count(self.vehicle_ids, 0.5),
                "walkers": len(self.walker_ids),
                # By DISPLACEMENT, not velocity. Walkers driven by the AI controller are
                # moved kinematically, and CARLA can report get_velocity() as zero for them
                # even while they walk - so a velocity threshold cannot tell "standing
                # still" from "walking". Comparing positions between calls can.
                "walkers_moving": self._walkers_displaced(),
                # If this is not equal to `walkers`, the AI controllers did not attach and
                # no amount of go_to_location will make anybody walk.
                "controllers": len(self.controller_ids)}

    # ---------- environment ----------

    WEATHER = {
        "ClearNoon": carla.WeatherParameters.ClearNoon,
        "CloudyNoon": carla.WeatherParameters.CloudyNoon,
        "WetNoon": carla.WeatherParameters.WetNoon,
        "HardRainNoon": carla.WeatherParameters.HardRainNoon,
        "ClearSunset": carla.WeatherParameters.ClearSunset,
        "HardRainSunset": carla.WeatherParameters.HardRainSunset,
    }

    def set_weather(self, preset: str):
        """One call, and the AirSim drone camera sees it — mean pixel delta 45.8 measured
        between ClearNoon and HardRainSunset. This is the air-ground coupling that makes
        CARLA-Air worth using."""
        if preset not in self.WEATHER:
            raise ValueError(f"unknown weather {preset!r}; have {sorted(self.WEATHER)}")
        self._w.set_weather(self.WEATHER[preset])
        return preset

    def destroy_all(self):
        """Destroy by id in one batch.

        Never call `.destroy()` on a handle CARLA has already reaped: it throws a C++
        `std::runtime_error` that escapes as `terminate` and core-dumps the interpreter.
        """
        for cid in self.controller_ids:
            try:
                self._w.get_actor(cid).stop()
            except RuntimeError:
                pass
        ids = self.controller_ids + self.walker_ids + self.vehicle_ids
        if ids:
            # apply_batch_SYNC, not apply_batch. Fire-and-forget leaves destruction in
            # flight, and a spawn_traffic immediately afterwards attaches walker
            # controllers to walkers that are half-reaped. CARLA then throws
            #     set_actor_simulate_physics: Actor could not be found in the registry
            # as an uncaught C++ std::runtime_error, which calls terminate() and takes the
            # whole sidecar down mid-episode. Observed doing exactly that.
            #
            # The batch is small (tens of actors) and this runs during scenario setup, so
            # waiting costs nothing that matters.
            self._c.apply_batch_sync([carla.command.DestroyActor(i) for i in ids], True)
        self.vehicle_ids, self.walker_ids, self.controller_ids = [], [], []
        self._walker_targets = {}      # else destroyed walkers keep being steered
        self._last_walker_pos = {}
        return len(ids)
