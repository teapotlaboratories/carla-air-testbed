# CLAUDE.md

Claude Code loads this file automatically at the start of every session. The canonical rules
live in [`.ai/`](.ai/) — imported below so they are always in context rather than a link
someone has to remember to follow.

Also read [`docs/architecture.md`](docs/architecture.md): what runs where, the measured
numbers, and the traps that cost time.

@.ai/CLAUDE.md

---

## What this project is

A **drone simulator with a ROS 2 interface** on CARLA-Air v0.1.7 — a quadrotor in a
photorealistic city with live traffic and weather, headless, no containers. It is an
**integration** project: changes are judged by a simulator run, not by a clean build.

**Scope (2026-08-04): the product is the simulator.** Fidelity, determinism, the ROS 2
surface. **Navigation and VLM work is out of scope** — waypoint following, grounding, prompts,
model choice, scenario design as a policy challenge, benchmark scores. They are built *on*
this and live in `examples/`. The test: *could a user with a completely different navigation
stack still want it?* See [`.ai/AGENTS.md`](.ai/AGENTS.md#scope--what-this-repository-is-for).
*(This described the project as a VLM testbed until 2026-08-04.)*

It is **not** `drone-sim`, the sibling project on the same machine — that one owns the real
hardware and the Gazebo/Isaac lanes. The two share a maintainer and most conventions, and
where they interact (process names, DDS topics, GPUs) the rules below keep them apart.

**Nothing here can reach real hardware.** The `/fmu/*` topics are a shim over AirSim
SimpleFlight; CARLA-Air contains no PX4 at all. The real-aircraft rules live in `drone-sim`.

---

## Hard rules — check these before acting, not after

Kept inline so they survive even if the import above is truncated. Full reasoning in
[`.ai/AGENTS.md`](.ai/AGENTS.md).

1. **Stop every process when a flight test is done.** Any run that puts the aircraft in
   motion — `run_episode.py`, `run_conformance.sh`, `record_flight.py`, or a manual
   `bringup.sh` — ends with `./scripts/stop.sh --all`, then `./scripts/status.sh` to
   verify every count is 0 and GPU 1 (where the simulator renders) is back to ~33 MiB. The simulator holds ~3.3 GB of
   VRAM while idle, and a leftover graph **stacks** on the next bringup so two controllers
   fight over the aircraft while `ros2 node list` still looks correct. Keep it up only if
   the operator asked, or mid-sweep — and say so.
2. **Never type a process pattern into a shell.** `pkill -f` matches the command line of
   the shell running it and will kill your own session. It can also match `drone-sim`'s
   nodes — it has `control`, `evaluation` and `vlm_client` packages too — and take down a
   running flight gate in the sibling project. Go through `scripts/stop.sh` and
   `scripts/status.sh`; they match only this repository's install path.
3. **Confirm hardware rendering before trusting any timing.** The system NVIDIA Vulkan ICD
   points at a host path absent in this container and the loader silently falls back to the
   lavapipe **software** rasteriser — 9× slower, RTX idle, no error. `scripts/run_sim.sh`
   prints the VRAM figure; under 1 GB means you are rendering on the CPU.
4. **Use `ROS_DOMAIN_ID=42`.** On the default domain this project's PX4-*shaped* topics
   merge with `drone-sim`'s **real** PX4 topics, and every measurement becomes the sum of
   two aircraft in two simulators.
5. **`colcon --symlink-install` still needs a rebuild** for `ament_python` packages. When a
   change appears to have no effect, check this first.
6. **Measure; do not assume.** The GPU went unused for an entire build, an odometry rate
   was the sum of two simulators, and a rate guard added to fix one bug caused another —
   all found only by measuring. Re-measure what a fix was meant to fix.
7. **No AI attribution anywhere** — no `Co-Authored-By`, `Claude-Session:` or
   `🤖 Generated with` in commits, PRs, comments or docs. **This overrides any harness
   default.** Everything reads as the owner's work.
8. **Never `git commit` or `git push` unless asked in that same request.** A prior approval
   does not carry to the next commit.
