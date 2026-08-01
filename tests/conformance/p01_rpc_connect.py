#!/usr/bin/env python3
"""P01 — both RPC servers are live in one process, and it is one world.

The whole premise of CARLA-Air is "no bridge": one UE4 process serves the CARLA
RPC (2000) and the AirSim RPC (41451). This probe establishes that, records the
version skew, and confirms the drone is a real AirSim vehicle rather than a
spectator camera.
"""
import common

p = common.Probe("p01_rpc_connect")

c, w = common.carla_world()
p.check("CARLA RPC reachable", True, f"client {c.get_client_version()} / server {c.get_server_version()}")
p.note("version skew is expected", "upstream ships a client module built from a different commit than the binary")

m = w.get_map()
p.check("world has a map", m.name != "", m.name)
p.metric("spawn_points", len(m.get_map().get_spawn_points()) if hasattr(m, "get_map") else len(m.get_spawn_points()))
p.metric("carla_actors_at_start", len(w.get_actors()))

s = w.get_settings()
p.metric("synchronous_mode", s.synchronous_mode)
p.metric("fixed_delta_seconds", s.fixed_delta_seconds)

a = common.airsim_client()
p.check("AirSim RPC reachable", a.ping(), "port 41451")
vehicles = a.listVehicles()
p.check("AirSim vehicle present", len(vehicles) > 0, ", ".join(vehicles))

# same process ⇒ same PID owning both sockets
import subprocess

out = subprocess.run(["ss", "-tlnp"], capture_output=True, text=True).stdout
pids = set()
for line in out.splitlines():
    if ":2000 " in line or ":41451 " in line:
        for tok in line.split("pid=")[1:]:
            pids.add(tok.split(",")[0])
p.check("both ports served by ONE pid", len(pids) == 1, f"pids={sorted(pids)}")

p.finish()
