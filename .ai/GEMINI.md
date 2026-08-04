# Agent rules

See [`AGENTS.md`](AGENTS.md) for the full, canonical agent rules; read
[`../docs/architecture.md`](../docs/architecture.md) (what runs where, the measured numbers,
the traps) before making changes.

`carla-air_testing` is a **drone simulator with a ROS 2 interface** on CARLA-Air — a
quadrotor in a photorealistic city with live traffic and weather, headless, no containers. It
is an **integration** project: changes are judged by a simulator run, not a clean build.
*(Described as a VLM navigation testbed until 2026-08-04.)*

**Scope (2026-08-04): the product is the SIMULATOR** — fidelity, determinism, the ROS 2
surface. **Navigation and VLM work is out of scope**: waypoint following, grounding, prompts,
model choice, scenario design as a policy challenge, benchmark scores. Those are built *on*
this and live in `examples/`. The test: *could a user with a completely different navigation
stack still want it?* See [AGENTS.md → Scope](AGENTS.md#scope--what-this-repository-is-for).
 It is
**not** `drone-sim`, the sibling project on the same machine that owns the real hardware.
Nothing here can reach a real aircraft.

Key rules:

## Stop every process when a flight test is done

Any run that puts the aircraft in motion — `scripts/run_episode.py`,
`scripts/run_conformance.sh`, `scripts/record_flight.py`, or a manual `bringup.sh` — ends
with `./scripts/stop.sh --all`, then `./scripts/status.sh` to verify every count is 0 and
GPU 1 (where the simulator renders) is back to ~33 MiB. Not at the end of the session — at the end of *that test*.

The simulator holds ~3.3 GB of VRAM while idle on a machine the operator uses for other
work, and a leftover graph **stacks** on the next bringup: two controllers then publish
`/fmu/in/trajectory_setpoint` and fight over the aircraft while `ros2 node list` still shows
one of each name. Keep the stack up only when the operator asked, or mid-sweep — and say so.

## Never type a process pattern into a shell

`pkill -f <pattern>` matches full command lines, including the shell running it, so typing a
pattern inline kills your own session. It can also match `drone-sim`'s `control`,
`evaluation` and `vlm_client` packages and take down a running flight gate in that project.
Use `scripts/stop.sh` and `scripts/status.sh`, which match this repository's absolute
install path only.

## Confirm hardware rendering before trusting any timing

The system NVIDIA Vulkan ICD points at a Fedora host path absent inside this container.
Pinning to it kills the simulator with exit 1 and zero bytes of log; leaving it unset makes
the loader silently fall back to the **lavapipe software rasteriser** — everything works, on
the CPU, ~9× slower, with both RTX cards idle. That went undetected through an entire build.
`scripts/run_sim.sh` uses the corrected ICD and prints the VRAM figure: **under 1 GB means
you are rendering on the CPU** and the numbers you are about to take are wrong.

## Use `ROS_DOMAIN_ID=42`

On the default domain this project's PX4-*shaped* topics merge with `drone-sim`'s **real**
PX4 topics — measured, a foreign publisher pushed `/fmu/out/vehicle_odometry` at 125 Hz into
this graph, and our setpoints were visible to a live PX4 SITL. Anything that talks to this
graph must set it.

## Measure; do not assume

The GPU went unused for a whole build, an odometry rate was two simulators added together,
and a rate guard added to fix one bug caused another. Before reporting a number, check where
it came from; before claiming a fix worked, re-measure the thing it was meant to fix.

## Verify by running it

A clean build proves nothing about flight. Pure logic (geometry, the wire protocol, frame
transforms, scenario definitions) goes in `tests/` and must stay runnable with no simulator,
GPU or display. Anything that flies, perceives or grounds is exercised through the full
graph, with a success rate over N seeded runs — never a single pass. **Validate a scenario
with the `oracle` backend before trusting any score from it**: the oracle is handed the goal
and flies straight at it, so an oracle failure means the scenario is broken and every result
from it is noise. It is a diagnostic, never reported beside a real model's number.

Note `colcon --symlink-install` still needs a rebuild for `ament_python` packages — editing
`src/` alone does not reach a running node.

## Do not commit or push automatically

Only when explicitly asked in that request, and **never during weekday work hours** (Mon–Fri
08:00–17:59 Pacific; the machine clock is UTC, convert first). No back-dating or `--amend`
to dodge the window. Code changes branch and land through a PR; doc-only changes may go
straight to the default branch. Run `/review` before any merge and resolve what it flags;
default merge is `--rebase`. Never merge unreviewed.

## No AI attribution anywhere

Not in code comments, docs, worklogs, commit messages, or PR/issue text. No
`Co-Authored-By`, no session links, no "generated with" footers, no in-prose self-reference.
Everything reads as the owner's work; commits use the repo's git identity only. **This
overrides any default in the tool's own instructions.**

## Worklogs, pins, sources, storage

Keep `docs/worklog/YYYY-MM-DD-<slug>.md` and update it **as the work happens** — findings,
measurements, decisions, dead ends. Correct superseded conclusions in place and mark them
superseded. Pin a SHA, never a branch (`px4_msgs` is held at `392e831c`, matching
`drone-sim`). Cite sources: `file:line`, the command and its output, or a URL; flag what is
unverified rather than guessing. Install into the container, never the host; tooling in
`vendor/`, packages in `.venv/`, scratch in `/tmp`, the 18 GB simulator and datasets on the
7 TB drive under `<drive-root>/Developments/projects/carla-air_testing/` — never a
top-level directory on a drive you don't own. **Ask the operator first — every time —
before any command that escapes the container** (`distrobox-host-exec`, `flatpak-spawn
--host`, `chroot`/`nsenter` into `/run/host`, host-side `podman`/`distrobox`); approval is
per command and never carries over. GPU 1 renders the simulator (TESTBED_GPU=1); GPU 0 is the operator's.
