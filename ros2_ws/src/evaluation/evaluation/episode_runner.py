#!/usr/bin/env python3
"""Run seeded navigation episodes and score them. The reason this is a testbed.

An episode is: reset the aircraft to a seeded start, spawn seeded traffic and weather, give
the VLM an instruction, let the loop fly, and stop on success, collision, timeout or step
budget. The output is one `EpisodeResult` per run and a JSON table over a sweep — a success
rate over N seeds, never a single pass.

**What the numbers can and cannot mean.** The simulator's own repeatability was measured at
0.04 m of trajectory divergence between identical runs, so the harness is not the noise
floor. But waypoint accuracy is ~4 m (the vehicle relaxes after reaching a setpoint), so a
success radius tighter than that measures the controller's relaxation rather than the
model's navigation. The default is 20 m, matching AerialVLN, and the scenario file can
override it — going below ~8 m is measuring the wrong thing.

SPL follows Anderson et al.: success weighted by the ratio of the shortest path to the path
actually flown. With no obstacle-aware planner in the loop, the straight-line distance is
used as the shortest path, which makes SPL optimistic in cluttered scenes. Stated here
rather than buried, because an SPL that is not comparable to the literature's should not be
quietly reported as if it were.
"""
from __future__ import annotations

import json
import math
import os
import time
import uuid
from dataclasses import asdict, dataclass, field

import numpy as np
import rclpy
import yaml
from interfaces.msg import Annotation2D, Collision, EpisodeResult, EpisodeStatus, GroundedWaypoint
from interfaces.srv import SetEpisode
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String

PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


@dataclass
class Episode:
    """One scenario instance. Everything needed to replay it lives in this object."""

    episode_id: str
    scenario: str
    instruction: str
    seed: int
    start_ned: list
    goal_ned: list | None
    success_radius_m: float = 20.0
    max_steps: int = 30
    timeout_s: float = 240.0
    weather: str = "ClearNoon"
    traffic_vehicles: int = 15
    traffic_walkers: int = 10

    started_at: float = 0.0
    steps: int = 0
    path_length_m: float = 0.0
    collisions: int = 0
    vlm_latencies: list = field(default_factory=list)
    backend: str = ""
    state: str = "pending"
    failure_mode: str = "none"


class EpisodeRunner(Node):
    def __init__(self):
        super().__init__("episode_runner")

        self.declare_parameter("scenarios_file", "")
        self.declare_parameter("results_dir", "out/episodes")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("status_rate_hz", 2.0)

        self._episode: Episode | None = None
        self._odom: VehicleOdometry | None = None
        self._last_pos: np.ndarray | None = None
        self._scenarios = self._load_scenarios()

        self.pub_status = self.create_publisher(EpisodeStatus, "/episode/status", 5)
        self.pub_result = self.create_publisher(EpisodeResult, "/episode/result", 5)
        self.pub_instruction = self.create_publisher(String, "/vlm/instruction", 5)
        # Transient-local: the oracle backend needs the goal even if it subscribes after the
        # episode has already started.
        goal_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                              history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.pub_goal = self.create_publisher(PointStamped, "/episode/goal", goal_qos)

        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry",
                                 self._on_odom, PX4_QOS)
        self.create_subscription(Annotation2D, "/vlm/annotation", self._on_annotation, 5)
        self.create_subscription(GroundedWaypoint, "/control/waypoint", self._on_wp, 5)
        self.create_subscription(Collision, "/sim/collision", self._on_collision, 5)

        self.srv = self.create_service(SetEpisode, "/episode/set", self._on_set)
        self.create_timer(1.0 / float(self.get_parameter("status_rate_hz").value), self._tick)

        os.makedirs(self.get_parameter("results_dir").value, exist_ok=True)
        self.get_logger().info(
            f"episode runner ready — {len(self._scenarios)} scenarios loaded. "
            "Start one with: ros2 service call /episode/set interfaces/srv/SetEpisode "
            "\"{scenario: 'cross_the_plaza', seed: 1, start: true}\"")

    # ---------------------------------------------------------------- scenarios

    def _load_scenarios(self) -> dict:
        path = self.get_parameter("scenarios_file").value
        if not path or not os.path.exists(path):
            return {}
        with open(path) as f:
            doc = yaml.safe_load(f) or {}
        return {s["name"]: s for s in doc.get("scenarios", [])}

    def _make_episode(self, name: str, seed: int, instruction: str = "") -> Episode:
        s = self._scenarios.get(name)
        if s is None:
            raise KeyError(f"unknown scenario {name!r}; have {sorted(self._scenarios)}")
        return Episode(
            episode_id=f"{name}-s{seed}-{uuid.uuid4().hex[:6]}",
            scenario=name,
            instruction=instruction or s["instruction"],
            seed=seed,
            start_ned=list(s["start_ned"]),
            goal_ned=list(s["goal_ned"]) if s.get("goal_ned") else None,
            success_radius_m=float(s.get("success_radius_m", 20.0)),
            max_steps=int(s.get("max_steps", 30)),
            timeout_s=float(s.get("timeout_s", 240.0)),
            weather=s.get("weather", "ClearNoon"),
            traffic_vehicles=int(s.get("traffic_vehicles", 15)),
            traffic_walkers=int(s.get("traffic_walkers", 10)),
        )

    # ------------------------------------------------------------------- inputs

    def _on_odom(self, msg: VehicleOdometry):
        self._odom = msg
        if self._episode is None or self._episode.state != "running":
            self._last_pos = None
            return
        p = np.array([float(c) for c in msg.position])
        if self._last_pos is not None:
            self._episode.path_length_m += float(np.linalg.norm(p - self._last_pos))
        self._last_pos = p

    def _on_annotation(self, msg: Annotation2D):
        if self._episode and self._episode.state == "running":
            self._episode.steps += 1
            self._episode.vlm_latencies.append(float(msg.latency_s))
            self._episode.backend = msg.backend
            if msg.terminal:
                self._finish("none" if self._within_goal() else "model_declared_done")

    def _on_wp(self, msg: GroundedWaypoint):
        if self._episode and self._episode.state == "running" and not msg.valid:
            self.get_logger().info(f"invalid waypoint during episode: {msg.reason}")

    def _on_collision(self, msg: Collision):
        if msg.has_collided and self._episode and self._episode.state == "running":
            self._episode.collisions += 1
            # Name what was hit. `has_collided` latches until the next reset, so without
            # this the log says a run was spoiled and not by what.
            self.get_logger().warn(
                f"collision with {msg.object_name or 'an unnamed actor'} "
                f"at {[round(v, 1) for v in msg.position_ned]}")
            self._finish("collision")

    # -------------------------------------------------------------------- logic

    def _distance_to_goal(self) -> float:
        if self._episode is None or self._episode.goal_ned is None or self._odom is None:
            return float("nan")
        p = np.array([float(c) for c in self._odom.position])
        return float(np.linalg.norm(p - np.array(self._episode.goal_ned)))

    def _within_goal(self) -> bool:
        d = self._distance_to_goal()
        return math.isfinite(d) and d <= self._episode.success_radius_m

    def _tick(self):
        ep = self._episode
        if ep is None:
            return
        if ep.state == "running":
            elapsed = time.time() - ep.started_at
            if self._within_goal():
                self._finish("none")
            elif elapsed > ep.timeout_s:
                self._finish("timeout")
            elif ep.steps >= ep.max_steps:
                self._finish("max_steps")

        st = EpisodeStatus()
        st.header.stamp = self.get_clock().now().to_msg()
        st.episode_id = ep.episode_id
        st.instruction = ep.instruction
        st.seed = ep.seed
        st.step = ep.steps
        st.elapsed_s = float(time.time() - ep.started_at) if ep.started_at else 0.0
        st.distance_to_goal = self._distance_to_goal()
        st.path_length = ep.path_length_m
        st.collided = ep.collisions > 0
        st.state = ep.state
        self.pub_status.publish(st)

    def _on_set(self, req: SetEpisode.Request, resp: SetEpisode.Response):
        if not req.start:
            if self._episode and self._episode.state == "running":
                self._finish("aborted")
            resp.accepted, resp.message = True, "aborted"
            return resp
        try:
            ep = self._make_episode(req.scenario, int(req.seed), req.instruction)
        except KeyError as exc:
            resp.accepted, resp.message = False, str(exc)
            return resp

        # The sim-side reset (pose, traffic, weather) is driven by scripts/run_episode.py,
        # which owns the sim_bridge connection. This node scores; it does not fly.
        ep.state = "running"
        ep.started_at = time.time()
        self._episode = ep
        self._last_pos = None
        self.pub_instruction.publish(String(data=ep.instruction))
        if ep.goal_ned:
            g = PointStamped()
            g.header.stamp = self.get_clock().now().to_msg()
            g.header.frame_id = "map"
            g.point.x, g.point.y, g.point.z = (float(c) for c in ep.goal_ned)
            self.pub_goal.publish(g)
        self.get_logger().info(f"episode {ep.episode_id}: {ep.instruction!r}")
        resp.accepted, resp.episode_id, resp.message = True, ep.episode_id, "running"
        return resp

    def _finish(self, failure_mode: str):
        ep = self._episode
        if ep is None or ep.state != "running":
            return
        success = failure_mode == "none" and self._within_goal()
        ep.state = "success" if success else "failure"
        ep.failure_mode = "none" if success else failure_mode

        final = self._distance_to_goal()
        straight = (float(np.linalg.norm(np.array(ep.goal_ned) - np.array(ep.start_ned)))
                    if ep.goal_ned else float("nan"))
        spl = 0.0
        if success and math.isfinite(straight) and ep.path_length_m > 0:
            spl = straight / max(straight, ep.path_length_m)

        r = EpisodeResult()
        r.header.stamp = self.get_clock().now().to_msg()
        r.episode_id = ep.episode_id
        r.instruction = ep.instruction
        r.scenario = ep.scenario
        r.seed = ep.seed
        r.success = success
        r.failure_mode = ep.failure_mode
        r.final_distance_m = final
        r.path_length_m = ep.path_length_m
        r.spl = float(spl)
        r.steps = ep.steps
        r.duration_s = float(time.time() - ep.started_at)
        r.collisions = ep.collisions
        r.vlm_latency_mean_s = float(np.mean(ep.vlm_latencies)) if ep.vlm_latencies else 0.0
        r.vlm_latency_max_s = float(np.max(ep.vlm_latencies)) if ep.vlm_latencies else 0.0
        r.backend = ep.backend
        self.pub_result.publish(r)

        out = os.path.join(self.get_parameter("results_dir").value, f"{ep.episode_id}.json")
        with open(out, "w") as f:
            json.dump(asdict(ep) | {"success": success, "spl": spl,
                                    "final_distance_m": final}, f, indent=2)
        self.get_logger().info(
            f"{ep.episode_id}: {'SUCCESS' if success else 'FAILURE (' + ep.failure_mode + ')'} "
            f"— {final:.1f} m from goal, {ep.steps} steps, {ep.path_length_m:.0f} m flown")


def main(argv=None):
    rclpy.init(args=argv)
    node = EpisodeRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
