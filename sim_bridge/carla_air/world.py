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

import random

import carla


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

    def spawn_traffic(self, vehicles: int = 15, walkers: int = 10):
        bp = self._w.get_blueprint_library()
        self._tm.set_global_distance_to_leading_vehicle(2.5)

        cars = [b for b in bp.filter("vehicle.*") if int(b.get_attribute("number_of_wheels")) == 4]
        points = self.spawn_points()
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
        for i in range(walkers):
            loc = self._w.get_random_location_from_navigation()
            if loc is None:
                continue
            wbatch.append(carla.command.SpawnActor(wbps[i % len(wbps)], carla.Transform(loc)))
        self.walker_ids = [r.actor_id for r in self._c.apply_batch_sync(wbatch, True) if not r.error]

        ctrl_bp = bp.find("controller.ai.walker")
        self.controller_ids = [
            r.actor_id for r in self._c.apply_batch_sync(
                [carla.command.SpawnActor(ctrl_bp, carla.Transform(), wid)
                 for wid in self.walker_ids], True) if not r.error]
        for cid in self.controller_ids:
            c = self._w.get_actor(cid)
            c.start()
            c.go_to_location(self._w.get_random_location_from_navigation())

        return {"vehicles": len(self.vehicle_ids), "walkers": len(self.walker_ids)}

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
            self._c.apply_batch([carla.command.DestroyActor(i) for i in ids])
        self.vehicle_ids, self.walker_ids, self.controller_ids = [], [], []
        return len(ids)
