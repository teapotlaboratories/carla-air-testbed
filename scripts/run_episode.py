#!/usr/bin/env python3
"""Set the world up for an episode, then hand scoring to the ROS 2 episode runner.

Runs on the **Python 3.10** side, because resetting the aircraft, spawning traffic and
setting weather all go through the sim_bridge socket. The split is deliberate: this script
owns the world, the ROS 2 graph owns the flight, and `evaluation/episode_runner` owns the
score. No component does two of those.

    ./.venv/bin/python scripts/run_episode.py --scenario cross_the_plaza --seeds 1 2 3

The ROS 2 graph must already be running (`./scripts/bringup.sh`). This script drives the
`/episode/set` service through `ros2 service call`, which is the one place the two Python
versions have to talk without the socket — a subprocess, not an import.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                "sim_bridge"))

import protocol  # noqa: E402
import socket as _socket  # noqa: E402

import yaml  # noqa: E402

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCENARIOS = os.path.join(PROJ, "ros2_ws", "src", "evaluation", "scenarios", "default.yaml")


class Sim:
    """Minimal synchronous client — the ROS-side one lives in carla_air_bridge/client.py."""

    def __init__(self, path=protocol.DEFAULT_SOCKET, timeout=120.0):
        self.s = _socket.socket(_socket.AF_UNIX, _socket.SOCK_STREAM)
        self.s.settimeout(timeout)
        self.s.connect(path)
        self.n = 0

    def call(self, method, **args):
        self.n += 1
        protocol.send(self.s, {"id": self.n, "method": method, "args": args})
        r = protocol.recv(self.s)
        if not r.get("ok"):
            raise RuntimeError(f"{method}: {r.get('error')}\n{r.get('traceback', '')}")
        return r.get("result")


def load_scenario(name):
    with open(SCENARIOS) as f:
        doc = yaml.safe_load(f)
    for s in doc["scenarios"]:
        if s["name"] == name:
            return s
    raise SystemExit(f"unknown scenario {name!r}; have "
                     f"{[s['name'] for s in doc['scenarios']]}")


def ros_param(node, name, value):
    """Set a ROS parameter from the 3.10 side, via the CLI (the one unavoidable subprocess)."""
    return subprocess.run(
        ["bash", "-lc",
         f"export ROS_DOMAIN_ID=${{TESTBED_ROS_DOMAIN_ID:-42}} && "
         "source /opt/ros/jazzy/setup.bash && "
         f"source {PROJ}/ros2_ws/install/setup.bash && "
         f"ros2 param set {node} {name} {value}"],
        capture_output=True, text=True, timeout=30)


def ros_service_call(scenario, seed, instruction=""):
    payload = (f"{{scenario: '{scenario}', seed: {seed}, "
               f"instruction: '{instruction}', start: true}}")
    return subprocess.run(
        ["bash", "-lc",
         # Same DDS domain as scripts/bringup.sh, or the service is simply not found —
         # and on domain 0 you would reach drone-sim's graph instead. See bringup.sh.
         f"export ROS_DOMAIN_ID=${{TESTBED_ROS_DOMAIN_ID:-42}} && "
         "source /opt/ros/jazzy/setup.bash && "
         f"source {PROJ}/ros2_ws/install/setup.bash && "
         f"ros2 service call /episode/set interfaces/srv/SetEpisode \"{payload}\""],
        capture_output=True, text=True, timeout=60)


def run_one(sim, scen, seed, camera_pitch):
    print(f"\n=== {scen['name']} seed={seed} ===", flush=True)

    # World first, aircraft second: traffic spawning moves actors around and the reset
    # must be the last thing that touches the vehicle before the clock starts.
    sim.call("destroy_actors")
    sim.call("set_weather", preset=scen.get("weather", "ClearNoon"))
    traffic = sim.call("spawn_traffic",
                       vehicles=int(scen.get("traffic_vehicles", 15)),
                       walkers=int(scen.get("traffic_walkers", 10)))
    print(f"  traffic: {traffic}", flush=True)

    start = list(scen["start_ned"])
    # The reset must own the aircraft exclusively. The offboard controller streams setpoints
    # at 10 Hz and will happily drag the vehicle toward the previous episode's waypoint
    # while reset() is trying to fly it to the start.
    ros_param("/offboard_control", "enabled", "false")
    time.sleep(0.5)

    # reset() leaves the aircraft holding a setpoint — never merely armed. Skipping that is
    # how a session ends up at -1566 m NED.
    sim.call("reset", hold_ned=start, speed=10.0)
    sim.call("set_camera_pose", xyz=[0.5, 0.0, 0.1], pitch=camera_pitch, roll=0.0, yaw=0.0)
    state = sim.call("state")
    err = sum((a - b) ** 2 for a, b in zip(state["position"], start)) ** 0.5
    print(f"  start pose: {[round(c, 1) for c in state['position']]} "
          f"(commanded {start}, error {err:.1f} m)", flush=True)
    if err > 15.0:
        print("  WARNING: reset did not reach the start — episode will not be comparable",
              flush=True)

    ros_param("/offboard_control", "enabled", "true")

    r = ros_service_call(scen["name"], seed, scen["instruction"])
    if r.returncode != 0 or "accepted=True" not in r.stdout.replace(" ", ""):
        print(f"  episode/set failed:\n{r.stdout}\n{r.stderr}", flush=True)
        return None

    # Wait for THIS episode's file, by id. Matching on a scenario+seed prefix returns the
    # previous run of the same seed the instant it is checked — a sweep would silently
    # report stale results and never notice, because they look entirely plausible.
    m = re.search(r"episode_id\s*=\s*'([^']+)'", r.stdout)
    if not m:
        print(f"  could not read episode_id from the service reply:\n{r.stdout}", flush=True)
        return None
    episode_id = m.group(1)
    print(f"  episode running [{episode_id}] — {scen['instruction']!r}", flush=True)

    deadline = time.time() + float(scen.get("timeout_s", 240.0)) + 30.0
    while time.time() < deadline:
        time.sleep(5.0)
        s = sim.call("state")
        c = sim.call("collision")
        print(f"    alt {abs(s['position'][2]):5.1f} m  "
              f"pos {[round(v, 1) for v in s['position']]}"
              f"{'  COLLIDED: ' + c['object_name'] if c['has_collided'] else ''}", flush=True)
        result = result_for(episode_id)
        if result:
            return result
    print("  runner did not report a result before the deadline", flush=True)
    return None


def result_for(episode_id):
    """The result file for exactly this episode, or None if it has not landed yet."""
    path = os.path.join(PROJ, "out", "episodes", f"{episode_id}.json")
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--seeds", type=int, nargs="+", default=[1])
    ap.add_argument("--socket", default=protocol.DEFAULT_SOCKET)
    ap.add_argument("--camera-pitch", type=float, default=-0.5,
                    help="radians; must match grounding's camera_pitch_deg")
    args = ap.parse_args()

    scen = load_scenario(args.scenario)
    sim = Sim(args.socket)
    print(f"connected to sim_bridge: {sim.call('describe')['map']}")

    results = []
    try:
        for seed in args.seeds:
            r = run_one(sim, scen, seed, args.camera_pitch)
            if r:
                results.append(r)
                print(f"  -> {'SUCCESS' if r['success'] else 'FAILURE (' + r['failure_mode'] + ')'}"
                      f"  {r['final_distance_m']:.1f} m from goal, {r['steps']} steps",
                      flush=True)
    finally:
        sim.call("destroy_actors")

    if results:
        ok = sum(1 for r in results if r["success"])
        print(f"\n=== {args.scenario}: {ok}/{len(results)} succeeded "
              f"({100.0 * ok / len(results):.0f}%) ===")
        # A success rate over N seeds, never a single pass.
        out = os.path.join(PROJ, "out", f"sweep-{args.scenario}.json")
        with open(out, "w") as f:
            json.dump({"scenario": args.scenario, "seeds": args.seeds,
                       "success_rate": ok / len(results), "episodes": results}, f, indent=2)
        print(f"wrote {out}")


if __name__ == "__main__":
    main()
