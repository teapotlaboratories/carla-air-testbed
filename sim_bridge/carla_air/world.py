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

    def spawn_traffic(self, vehicles: int = 15, walkers: int = 10,
                      near_ned=None, radius_m: float = 70.0):
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

        ctrl_bp = bp.find("controller.ai.walker")
        self.controller_ids = [
            r.actor_id for r in self._c.apply_batch_sync(
                [carla.command.SpawnActor(ctrl_bp, carla.Transform(), wid)
                 for wid in self.walker_ids], True) if not r.error]
        for i, cid in enumerate(self.controller_ids):
            c = self._w.get_actor(cid)
            c.start()
            # Send them somewhere nearby. A map-wide destination makes pedestrians walk
            # straight out of frame, which looks identical to having spawned none.
            target = None
            if near_ned is not None and i < len(walker_spots):
                origin = walker_spots[i]
                for _ in range(30):
                    candidate = self._w.get_random_location_from_navigation()
                    if candidate is None:
                        continue
                    if candidate.distance(origin) <= radius_m:
                        target = candidate
                        break
            c.go_to_location(target or self._w.get_random_location_from_navigation())

        return {"vehicles": len(self.vehicle_ids), "walkers": len(self.walker_ids),
                "clustered": bool(clustered), "near_candidates": int(near_candidates)}

    def tick_watchdog(self):
        """Re-arm autopilot on stalled vehicles. Call at ~1 Hz for the life of an episode."""
        if not self.vehicle_ids:
            return 0
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

    def traffic_stats(self):
        moving = 0
        for a in self._w.get_actors(self.vehicle_ids):
            try:
                v = a.get_velocity()
                if (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5 > 0.5:
                    moving += 1
            except RuntimeError:
                pass
        return {"spawned": len(self.vehicle_ids), "moving": moving,
                "walkers": len(self.walker_ids)}

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
        return len(ids)
