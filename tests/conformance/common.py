"""Shared helpers for the CARLA-Air probes.

Every probe is a standalone script that prints a PASS/FAIL line per check and
exits non-zero if anything failed, so `run_probes.sh` can be trusted as a gate
rather than read as prose.
"""
import json
import os
import sys
import time

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "out")
os.makedirs(OUT, exist_ok=True)

CARLA_PORT = int(os.environ.get("CARLA_PORT", 2000))
AIRSIM_PORT = int(os.environ.get("AIRSIM_PORT", 41451))


class Probe:
    def __init__(self, name):
        self.name = name
        self.checks = []
        self.metrics = {}
        print("=" * 68)
        print(f"  {name}")
        print("=" * 68)

    def check(self, label, ok, detail=""):
        self.checks.append((label, bool(ok), detail))
        print(f"  [{'PASS' if ok else 'FAIL'}] {label}" + (f" — {detail}" if detail else ""))
        return ok

    def note(self, label, detail=""):
        print(f"  [INFO] {label}" + (f" — {detail}" if detail else ""))

    def metric(self, key, value):
        self.metrics[key] = value
        print(f"  [MET ] {key} = {value}")

    def finish(self):
        failed = [c for c in self.checks if not c[1]]
        print("-" * 68)
        print(f"  {len(self.checks) - len(failed)}/{len(self.checks)} passed")
        with open(os.path.join(OUT, f"{self.name}.json"), "w") as f:
            json.dump(
                {
                    "probe": self.name,
                    "checks": [{"label": l, "ok": o, "detail": d} for l, o, d in self.checks],
                    "metrics": self.metrics,
                },
                f,
                indent=2,
            )
        sys.exit(1 if failed else 0)


def carla_world(timeout=20.0):
    import carla

    c = carla.Client("127.0.0.1", CARLA_PORT)
    c.set_timeout(timeout)
    return c, c.get_world()


def airsim_client(timeout=20.0):
    import airsim

    a = airsim.MultirotorClient(ip="127.0.0.1", port=AIRSIM_PORT, timeout_value=timeout)
    a.confirmConnection()
    return a


# Town10HD CARLA→AirSim frame offset, from upstream's COORDINATE_SYSTEMS.md
# (measured by upstream on v0.1.6; p06 re-measures it rather than trusting it).
# The AirSim NED origin sits offshore on this map — (0,0) is open water, which is
# why every probe that wants buildings has to transform a CARLA spawn point first.
OFFSET = (172.20, -183.86, 27.45)


def carla_to_airsim(loc):
    """carla.Location (metres, z-up) → AirSim NED tuple (metres, z-down)."""
    return (loc.x + OFFSET[0], loc.y + OFFSET[1], -loc.z + OFFSET[2])


def goto_city(a, w, altitude=-40.0, speed=12.0, spawn_index=0):
    """Fly the drone over a CARLA spawn point — i.e. over actual streets."""
    sp = w.get_map().get_spawn_points()[spawn_index]
    x, y, _ = carla_to_airsim(sp.location)
    a.moveToPositionAsync(float(x), float(y), float(altitude), speed).join()
    time.sleep(2)
    return sp, (x, y, altitude)


def takeoff(a, z=-30.0, speed=6.0):
    """Arm, take off, and climb to `z` metres NED. Returns the reached position."""
    a.enableApiControl(True)
    a.armDisarm(True)
    a.takeoffAsync().join()
    p = a.getMultirotorState().kinematics_estimated.position
    a.moveToPositionAsync(p.x_val, p.y_val, z, speed).join()
    time.sleep(1.0)
    return a.getMultirotorState().kinematics_estimated.position


def land(a):
    try:
        a.landAsync().join()
    finally:
        a.armDisarm(False)
        a.enableApiControl(False)
