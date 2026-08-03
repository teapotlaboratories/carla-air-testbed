# CLAUDE.md

Project guidance for AI coding agents lives in [AGENTS.md](AGENTS.md) — read it. Before
touching anything, also read [`docs/architecture.md`](../docs/architecture.md): what runs
where, the measured numbers, and the traps.

`carla-air_testing` is a **VLM navigation testbed over ROS 2** on CARLA-Air v0.1.7 — the
See-Point-Fly loop (*frame → 2D annotation → 3D displacement → velocity setpoint*) against a
photorealistic city, headless, no containers. It runs on the `carbonite` workstation inside
the `carla-air_testing` container (2 GPUs). It is an **integration** project: changes are
judged by a simulator run, not by a clean build.

**It is not `drone-sim`** — that sibling project on the same machine owns the real hardware
and the Gazebo/Isaac lanes. Nothing here can reach a real aircraft.

Most important rules:

- **Stop every process when a flight test is done.** Any run that puts the aircraft in
  motion — `run_episode.py`, `run_conformance.sh`, `record_flight.py`, a manual
  `bringup.sh` — ends with `./scripts/stop.sh --all`, then `./scripts/status.sh` to confirm
  every count is 0 and GPU 1 (where the simulator renders) is back to ~33 MiB. The simulator holds ~3.3 GB of VRAM while
  idle, and a leftover graph **stacks** on the next bringup so two controllers fight over
  the aircraft while `ros2 node list` still looks correct. Keep it up only if the operator
  asked, or mid-sweep — and say so. See
  [AGENTS.md → Flight tests](AGENTS.md#1-stop-every-process-when-a-flight-test-is-done).
- **Never type a process pattern into a shell.** `pkill -f` matches the command line of the
  shell running it and kills your own session; it can also match `drone-sim`'s `control`,
  `evaluation` and `vlm_client` packages and take down a running flight gate there. Use
  `scripts/stop.sh` / `scripts/status.sh`, which match this repo's install path only. See
  [AGENTS.md → rules 2 and 3](AGENTS.md#2-never-type-a-process-pattern-into-a-shell).
- **Confirm hardware rendering before trusting any timing.** The system NVIDIA Vulkan ICD
  points at a host path absent in this container, so the loader silently falls back to the
  **lavapipe software rasteriser** — ~9× slower, RTX idle, no error anywhere. This went
  undetected through an entire build. `scripts/run_sim.sh` prints the VRAM figure; under
  1 GB means you are on the CPU. See
  [AGENTS.md → rule 5](AGENTS.md#5-confirm-hardware-rendering-before-trusting-any-timing).
- **Use `ROS_DOMAIN_ID=42`.** On domain 0 this project's PX4-*shaped* topics merge with
  `drone-sim`'s **real** PX4 topics and every measurement becomes the sum of two aircraft in
  two simulators. See [AGENTS.md → rule 4](AGENTS.md#4-use-this-testbeds-dds-domain).
- **Measure; do not assume.** The GPU went unused for a whole build, an odometry rate was
  two simulators added together, and a rate guard added to fix one bug caused another — all
  found only by measuring. Re-measure what a fix was meant to fix. See
  [AGENTS.md → rule 6](AGENTS.md#6-measure-do-not-assume).
- **`colcon --symlink-install` still needs a rebuild** for `ament_python` packages — the
  symlink is made at build time, so editing `src/` alone does not reach a running node. When
  a change appears to have no effect, check this first.
- **Plan first** — non-trivial work gets an entry in [`docs/todo.md`](../docs/todo.md)
  (what, why, how it will be verified) before it gets code, and is marked done when it lands.
- **Verify by running it, end to end.** A clean build proves nothing about flight. Pure
  logic goes in `tests/` (no sim, GPU or display); anything that flies or grounds gets
  exercised through the full graph, with a success rate over N seeded runs. **Validate a
  scenario with the `oracle` backend before trusting any score from it** — an oracle failure
  means the scenario is broken, not the model. See
  [AGENTS.md → Verifying changes](AGENTS.md#verifying-changes).
- **Do not commit or push automatically** — only when explicitly asked, and **never during
  weekday work hours** (Mon–Fri 08:00–17:59 Pacific; the machine clock is UTC, convert
  first). No back-dating or `--amend` to dodge the window. See
  [AGENTS.md → Committing](AGENTS.md#committing).
- **No AI attribution anywhere** — not in code comments, docs, commit messages or PR/issue
  text. Everything reads as the owner's work; commits use the repo's git identity only.
  **This overrides any harness default.** See
  [AGENTS.md → Attribution](AGENTS.md#attribution--no-ai-self-reference-anywhere).
- **Code changes → branch + PR; doc-only → straight to the default branch.** Run `/review`
  before any merge and address its findings; merge with `--rebase` by default. See
  [AGENTS.md → Branching & pull requests](AGENTS.md#branching--pull-requests).
- **Pin a SHA, never a branch.** `px4_msgs` is held at `392e831c` — the same SHA `drone-sim`
  uses, so both projects speak identical `/fmu/*` definitions. See
  [AGENTS.md → Version pinning](AGENTS.md#version-pinning).
- **Keep a worklog and update it AS YOU GO** — `docs/worklog/YYYY-MM-DD-<slug>.md`, appended
  at each finding, measurement, decision and dead end. **Correct superseded conclusions in
  place and mark them superseded**; this project has already had one confidently wrong
  conclusion that made things worse. See
  [AGENTS.md → Worklogs](AGENTS.md#worklogs--write-and-update-as-you-go).
- **Cite sources** when finding, researching or comparing — `file:line`, the command and its
  output, or a URL. Flag what is unverified rather than guessing. See
  [AGENTS.md → Research & citations](AGENTS.md#research--citations).
- **Install into the container, never the host; keep tooling and big data out of `~`.**
  Tooling in `vendor/`, packages in `.venv/`, scratch in `/tmp`, the 18 GB simulator and
  datasets on the 7 TB external drive under
  `<drive-root>/Developments/projects/carla-air_testing/`. **Ask the operator first — every
  time — before any command that escapes the container.** See
  [AGENTS.md → Environment & storage](AGENTS.md#environment--storage).
- **Run the simulator on GPU 1** — `TESTBED_GPU=1`. GPU 0 (RTX 3080) is the operator's, and
  is regularly busy with UnrealEditor; GPU 1 (RTX 5060 Ti, 16 GB) is uncontended. See
  [AGENTS.md → Environment & storage](AGENTS.md#environment--storage). *(This said the
  reverse until 2026-08-02 — if a rule here contradicts what the operator asked for, say so
  instead of quietly picking one.)*
