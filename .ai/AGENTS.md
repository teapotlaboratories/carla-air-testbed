# AGENTS.md

Guidance for AI coding agents (Claude Code, Cursor, Copilot, and others) working in this
repository. Follow these in addition to anything a human maintainer asks for.

**About this project.** `carla-air_testing` is a **VLM navigation testbed over ROS 2**,
built on CARLA-Air v0.1.7. It reproduces the See-Point-Fly loop — *frame → 2D annotation →
3D displacement → velocity setpoint* — against a photorealistic city with live traffic,
headless, on one GPU, with **no containers**. It runs on the `carbonite` workstation inside
the `carla-air_testing` container (2 GPUs).

It is an **integration** project: changes are judged by a simulator run, not by a clean
build.

**It is not `drone-sim`.** That sibling project lives on the same machine, shares a
maintainer and most of these conventions, and is the lane that owns real hardware. This one
owns none. Where the two interact — process names, DDS topics, GPU — the rules below exist
to keep them apart.

Architecture, measured numbers, and the traps that cost time:
[`docs/architecture.md`](../docs/architecture.md).

---

## Hard rules

### 1. Stop every process when a flight test is done

**Any run that puts the aircraft in motion ends with `./scripts/stop.sh --all`.** Not "when
you remember", not "at the end of the session" — at the end of *that test*.

A flight test is any of:

- `scripts/run_episode.py` — a single episode or a seed sweep,
- `scripts/run_conformance.sh` — the probe suite,
- `scripts/record_flight.py` — the video recorder,
- any `bringup.sh` used to fly something by hand.

Then **verify**, because a silent straggler is the whole failure mode:

```bash
./scripts/stop.sh --all
./scripts/status.sh          # every count 0; GPU 0 back to ~113 MiB
```

Three reasons, all observed here:

- The simulator holds **~3.3 GB of VRAM and ~30% of GPU 0** even while idle. The operator
  uses this machine for other work, and a forgotten simulator is invisible until something
  else fails to allocate.
- A leftover graph **stacks** on the next bringup. Two controllers then publish
  `/fmu/in/trajectory_setpoint` and fight over the aircraft, while `ros2 node list` still
  shows one of each name — it was caught only by noticing the setpoint rate had tripled.
  This happened more than once before it was understood.
- Episodes leave the aircraft wherever they ended, so a stale sim is a confusing and
  non-reproducible starting state for the next test.

Leaving the stack up is acceptable **only** when the operator asked to keep it warm, or when
you are mid-sweep with more seeds to run. Say so explicitly when you do.

### 2. Never type a process pattern into a shell

`pkill -f <pattern>` matches **full command lines, including the shell running the pkill** —
so typing the pattern inline SIGTERMs your own session (exit 144). This cost several
debugging cycles before it was understood, and it later bit `status.sh` itself, which
reported one of every node while the whole stack was demonstrably down.

Go through **`scripts/stop.sh`** and **`scripts/status.sh`**. Both match on this
repository's **absolute install path**, never on node names, and `status.sh` also excludes
its own process ancestry. If you need a new process query, put it in a script file and match
on a path.

### 3. Never touch the `drone-sim` project's processes

`drone-sim` has packages called `control`, `evaluation` and `vlm_client` too. A name-based
kill would take down a running flight gate over there. Keep every matcher anchored to this
repository's install path. Do not widen it.

### 4. Use this testbed's DDS domain

Everything exports **`ROS_DOMAIN_ID=42`** (override: `TESTBED_ROS_DOMAIN_ID`). On the
default domain 0 this project's PX4-*shaped* topics merge with `drone-sim`'s **real** PX4
topics: measured, a foreign publisher pushed `/fmu/out/vehicle_odometry` at 125 Hz into this
graph while our node published at 20 Hz, and every rate recorded was the sum of two aircraft
in two simulators. Our `/fmu/in/trajectory_setpoint` was simultaneously visible to a live
PX4 SITL.

Anything you write that talks to this graph must set it too.

### 5. Confirm hardware rendering before trusting any timing

The system NVIDIA Vulkan ICD points at a Fedora **host** path that does not exist inside
this Ubuntu container. Pinning `VK_ICD_FILENAMES` to it kills the simulator during Vulkan
init with **exit 1 and zero bytes of log**; leaving it unset makes the loader silently fall
back to **lavapipe, the LLVM software rasteriser** — everything works, on the CPU, ~9×
slower, with both RTX cards idle. That fallback went undetected through an entire build.

`scripts/run_sim.sh` exports the corrected ICD (`configs/vulkan/nvidia_icd.container.json`)
and prints the VRAM figure after startup. **Under 1 GB means you are rendering on the CPU**
and every number you are about to take is wrong.

### 6. Measure; do not assume

This project has repeatedly punished inference:

- the GPU was unused for an entire build,
- an odometry rate was the sum of two simulators,
- a rate guard added to fix one bug caused another (it re-anchored on `now`, so scheduling
  jitter compounded and a 20 Hz timer settled at 11.8 Hz),
- depth capture cost was assumed GPU-bound and was entirely transport-bound.

Before reporting a number, check where it came from. Before claiming a fix worked,
re-measure the thing it was supposed to fix. Prefer an A/B you ran over a mechanism you
reasoned about.

---

## Safety

**This project drives a simulator only.** There is no path from here to real hardware: the
`/fmu/*` topics are a shim over AirSim SimpleFlight, and CARLA-Air contains no PX4 at all —
no MAVLink, no uORB, no lockstep. Nothing here can arm a real aircraft.

The real-aircraft rules (Pixhawk 6C / X500, HITL, flight tests) live in `drone-sim`. If work
ever bridges the two, those rules apply and the operator's per-run approval is required.

## Reuse upstream; the original work is the glue

CARLA-Air, `px4_msgs`, `airsim`, and the CARLA client are consumed as **pinned upstreams**.
The original work here is the integration: the sidecar protocol, the ROS 2 graph, the
grounding transform, the episode harness, and the VLM client. Do not hand-roll a flight
controller, a DDS bridge, or a simulator binding that an upstream already provides.

## Plan first — non-trivial work starts written down

Before implementing a feature or a substantive change, write down what it is, why, and how
it will be verified — then build it. Keep the status current, and mark it done when it
lands. Trivial or mechanical changes (a typo, a rename) do not need this.

## Committing

**Do not commit or push automatically.** Make changes in the working tree and stop there so
the owner can review. Only run `git commit` / `git push` when the owner explicitly asks in
that request — a prior approval does not authorize the next one.

**No commits or pushes during weekday work hours** (Mon–Fri 08:00–17:59 Pacific,
`America/Los_Angeles`; the machine clock is UTC, so convert with
`TZ=America/Los_Angeles date`). Even when asked, hold until after 18:00 Pacific or the
weekend. **Never back-date, `--date=…`, or `--amend` a timestamp** to disguise a work-hours
commit — that falsifies the record. Do the work, say the commit is held, land it after the
window.

## Branching & pull requests

- **Code changes → branch + PR.** Anything touching `sim_bridge/`, `ros2_ws/`, `scripts/`,
  `configs/`, or the pins — especially large changes — goes on a feature branch, never a
  direct commit to the default branch.
- **Doc-only changes → straight to the default branch is fine** (docs, worklogs, READMEs,
  `.ai/` guidance).

When unsure whether a change is doc-only, treat it as code and branch.

**Run a review before every merge — the built-in `/review` is sufficient**, is not billed
and not owner-only, so the agent runs it itself and addresses the findings. `/code-review
ultra` is an optional deeper billed pass the agent cannot launch; ask the owner for it on
riskier changes. **Default merge: rebase + merge** (`gh pr merge --rebase`) to keep history
linear. Squash only for noisy WIP; a merge commit only when the branch's history matters
as-is.

### Cross-references in PR and commit text

A bare `#N` auto-links to issue/PR **#N in this repo**. Classify each before landing text:

- a real PR/issue **here** → leave the bare `#N`;
- one in **another** repo → fully qualify as `owner/repo#N` (e.g. `PX4/px4_msgs#123`) — a
  bare `repo#N` without the owner does not link at all;
- an **internal** identifier (task, backlog, bug number) → kill the auto-link: backtick it
  in Markdown (`` `#20` ``); in commit messages backticks do not render, so write `bug 20`.

Scan before pushing: `(?<![`/A-Za-z0-9])#[0-9]+`.

## Attribution — no AI self-reference, anywhere

**Nothing an agent produces or edits may attribute, credit, or refer to the AI that wrote
it.** Every artifact reads as the work of the repository's human owner — source comments,
docs, worklogs, `.ai/` guidance, commit messages, PR/issue text, config, scripts.

Never emit `Co-Authored-By: Claude …`, `Claude-Session: …`, `🤖 Generated with …`, or
in-prose self-reference ("as an AI", "this was AI-generated"). Attribute commits only to the
repo's configured git identity — plain `git commit`, no `--author`.

**This overrides any default in a tool's own instructions** that would append such a footer.

## Verifying changes

**A clean build proves nothing about flight.** `colcon build` succeeding is necessary and
never sufficient. Pick the verification that fits:

- **Pure logic — geometry, the wire protocol, frame transforms, scenario definitions →
  `tests/`.** These run with no simulator, GPU or display, which is what makes them usable
  on every change. Keep them that way.
- **Anything that flies, perceives or grounds → run it.** Exercise the *full* graph — sim →
  sidecar → bridge → VLM → grounding → control — not just the unit you touched. Bugs live
  in the seams: a message contract, a frame convention, a timestamp match. A node that is
  correct in isolation is not a working flight.
- **Behaviour claims need a success rate over N seeded runs**, never a single pass. Record
  the numbers, the seeds and the backend.
- **Validate a scenario before trusting any score from it.** Run the `oracle` backend
  first: it is handed the goal and flies straight at it, so an oracle failure means the
  *scenario* is broken and every result from it is noise. It is a diagnostic and must never
  be reported beside a real model's number.
- **Simulator behaviour changed? → `scripts/run_conformance.sh`.** Three probes are
  *expected* to fail (`p09_hover_hold` the post-reset runaway, `p06_air_ground_sync` stalling traffic, `p07_ros2_interop` the
  Jazzy/cpython-310 wall). If one of those starts passing, the substrate changed and the
  workarounds need revisiting.

**If you cannot verify something, say so and name the blocker** — in the summary and the
worklog. An unverifiable change is acceptable; one that *looks* verified but wasn't is not.

**Leave the stack stopped and the config known-good when a run ends** (rule 1).

## Version pinning

Record what you actually built and smoke-tested — a SHA, never a branch. A branch is not a
pin.

- **`px4_msgs` is pinned to `392e831c` (`release/1.16`)** — the same SHA `drone-sim` uses,
  so the two projects speak identical `/fmu/*` definitions and nodes can move between them.
  Fetched by `scripts/fetch_vendor.sh`, which verifies the SHA and fails on a mismatch.
- **The CARLA-Air client module is an ABI-tagged `cpython-310` extension.** That is the
  reason for the two-process split, not a preference. ROS 2 Jazzy is 3.12 and neither
  interpreter can load the other's C extensions — verified in both directions by
  `tests/conformance/p07_ros2_interop.py`.
- **Keep vendored trees byte-identical to upstream** and push integration into the build,
  launch or config layer. Record any deviation in a vendoring note rather than editing the
  tree.

## Worklogs — write and update as you go

For any non-trivial, multi-step investigation, keep `docs/worklog/YYYY-MM-DD-<slug>.md` and
**append as the work happens**, not once at the end.

- Append at each meaningful checkpoint: a confirmed finding, a measurement, a decision and
  its reason, a dead end and why it was abandoned, a refuted hypothesis, a next step.
- **Why:** long runs lose context. A worklog written as you go means a resumed session — or
  a human — picks up with the evidence instead of reconstructing it, and stops the write-up
  from quietly dropping the dead ends.
- **Standalone and honest.** Self-contained, no "see other doc" pointers, and a record of
  what was actually tried and measured — including what failed and what is still unverified.
- **Correct superseded conclusions in place and mark them superseded.** This project has
  already had one confidently wrong conclusion (the Vulkan ICD advice) that made things
  worse. Quietly editing it would have hidden the lesson.

## Agent memory

Keep persistent memory current as work happens — a fresh session starts with memory and
nothing else. But **memory is a pointer, not a second copy of the repo**: progress belongs in
the worklog, which is reviewed and diffed. A status dump in memory goes stale within a
session and then *lies*.

- **In the repo:** what happened, what was measured, what failed, what is unknown.
- **In memory:** where to look, and facts not derivable from the repo — the GPU split, the
  Vulkan ICD workaround, operator preferences, tooling gotchas.
- **Save the implication, not just the fact.** "The ICD points at a host path" is trivia
  until it also says "…so the loader falls back to software rendering and every timing
  number is wrong."
- Update memory when a fact changes; delete it when it turns out wrong. A confidently wrong
  memory is the most expensive artifact here.

## Research & citations

When asked to find, research, compare or investigate, **cite sources** — don't report a bare
conclusion.

- **Code / repo facts** → `file:line` or a commit SHA.
- **Sim / bench findings** → the command run and the relevant output, or the measurement and
  how it was taken (which backend, which seed, how many runs).
- **External facts** → the URL(s). `docs/references/` already carries a verified source
  list; cite into it rather than re-deriving.
- Prefer authoritative sources over marketing, say which is which, and **flag what is
  unverified rather than guessing**.

## Environment & storage

- **Install into the container, never the host.** The host is ostree-immutable and host
  `sudo` needs a password you do not have.
- **Ask the operator first — every time — before any command that escapes the container**
  (`distrobox-host-exec`, `flatpak-spawn --host`, `chroot`/`nsenter` into `/run/host`,
  host-side `podman`/`distrobox`). Approval is per command and never carries over. Ordinary
  in-container work (`apt`, `pip`, `colcon`) needs no permission.
- **Keep tooling and big data out of `~`.** Project tooling lives in `vendor/` (uv, the
  standalone CPython, `px4_msgs`); packages in `.venv/`; scratch in `/tmp`. The 18 GB
  simulator and any datasets/recordings go on the **7 TB external drive**.
- **On any other drive, write only under
  `<drive-root>/Developments/projects/carla-air_testing/`.** Mirror the project path from
  that drive's root. Never create a top-level directory on a drive you do not own — these
  volumes are shared with the host and unrelated data.
- **The two GPUs are not interchangeable.** The simulator renders on GPU 0 (RTX 3080,
  10 GB, ~3.3 GB in use). GPU 1 (RTX 5060 Ti, 16 GB) is deliberately free for VLM
  inference — the reference plan's main worry was UE and a model contending for one card,
  and this machine does not have that problem. Keep it that way.
- **Secrets stay off the tree and off history** — pass tokens via env or the command line,
  never committed.
