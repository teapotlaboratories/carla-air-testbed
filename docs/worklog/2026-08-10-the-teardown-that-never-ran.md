# 2026-08-10 — the grace period nobody needed, and the teardown block that never ran

Started with one narrow question left open on PR #9 (T-06): `docker stop -t 10` was only ever
exercised against an `alpine sleep` container, which dies on TERM instantly, so whether
CarlaUE4 eats the full 10 s grace period was **unmeasured**. One stack bring-up would settle it.

It settled it in 1.2 seconds, and then the second trial found something the first could not.

## The question that was asked: no, CarlaUE4 does not eat the grace period

Stack up on GPU 1, hardware rendering confirmed (3688 MiB), then the shipped command timed
directly:

    docker stop -t 10 carla-air-sim    →  1.175 s
    ExitCode                           →  143   (128 + 15 = SIGTERM)
    GPU 1 VRAM                         →  4006 MiB → 32 MiB

**143, not 137.** That is the whole answer: 137 would mean the grace period expired and Docker
sent SIGKILL. 143 means the process took SIGTERM and left on its own, and it did so in about a
tenth of the budget. `-t 10` is not costing anyone ten seconds.

Two structural facts checked rather than assumed, because either one would have made the
number meaningless:

- **PID 1 in the container is the Unreal binary itself**, not a shell. `ENTRYPOINT
  ["/bin/bash", "-lc"]` looks like it interposes one, but bash `exec`s a lone simple command,
  so `/proc/1/cmdline` is `CarlaUE4-Linux-Shipping`. Had a shell been sitting at PID 1 it would
  have taken the TERM and not forwarded it, and the 10 s would have been structural rather than
  Unreal being slow.
- **SIGTERM is caught, not ignored.** `/proc/1/status` gives `SigCgt: 00000041400144ff` — bit 14
  set, which is signal 15. It is absent from `SigIgn`.

So the host path's caution ("Unreal does not always go down on the first TERM") does **not**
transfer to the container. Worth stating plainly: the grace period is cheap insurance here, not
a cost.

## The question that was not asked: `stop.sh --all` never reaches the block this PR changed

Trial 2 ran the *shipped* path — `./scripts/stop.sh --all` against a real stack — because one
measurement of a teardown is thin and the PR's own real-container check had used `alpine`.

    stop.sh --all                 8.23 s
    output                        "stopped: graph and sidecar, simulator stopped (3 stragglers)"
    docker ps -a                  carla-air-sim   Exited (143)     ← still there
    GPU 1                         32 MiB

The container was **stopped but never removed**, and *nothing said so* — not the new
`STILL RUNNING` warning, not the new `stopped but NOT REMOVED` warning, not the success line.
All three are in the block this PR adds. None of them ran.

### Why: the host escalation kills the containerised simulator first

`scripts/stop.sh:147-155` escalates TERM/TERM/KILL against `pkill -x "CarlaUE4-Linux-"`, and
that runs **before** the container block at `scripts/stop.sh:179-207`. Three facts make it
reach inside the container:

| | |
|---|---|
| the container's CarlaUE4 runs as **`deck`** on the host | `docker top carla-air-sim` — UID column, not root |
| its `comm` is exactly `CarlaUE4-Linux-` | `/proc/1/comm`, truncated to 15 chars — precisely what `-x` matches |
| host `pgrep`/`ps` sees container processes | `targets()` listed the sidecar *inside* `carla-air-bridge` as a straggler |

So `pkill` wins the race. By the time the container block is reached, the container has already
exited — and its guard is
`if docker ps --format '{{.Names}}' | grep -qx "$c"`, which asks **is it running**. It is not.
The `if` is false, the whole block is skipped, and the container is left in `Exited (143)`.

**The graceful stop, the removal, and both verifications are dead code in the container lane.**
They are reachable only when `pkill -x` finds nothing — which is exactly the case an `alpine
sleep` container reproduces, and is why the PR's real-container check passed while missing this.

This is the third time this week a defect has been found in code written to prevent that same
class of defect — after the `${ALL:-0}` seeding that made a teardown escalate, and the guard
that classified `reset` as world control. Here a PR about *verifying* the teardown shipped a
verification that never executes.

### And the state it leaves is the one R-08 was filed about

`Exited (143)`, not removed, holds no VRAM — but **blocks the next start by that name**. That is
the exact failure R-08 records from 2026-08-07: a stale container silently served old code after
`webui.sh --in-stack` failed with `exit 125`, and it cost a wrong diagnosis. The teardown path
now manufactures that state on every container-lane run.

## The fix: stop the container before signalling anything

Two changes to `scripts/stop.sh`, both small, and the first is the one that matters:

1. **The container block moves ahead of the `pkill` escalation.** Then `docker stop -t 10` is
   what actually stops the simulator, the removal and both checks run, and `sim_alive` is
   already false when the escalation is reached — so its loop breaks on the first iteration
   instead of burning `sleep 2`.
2. **Its guard becomes `docker ps -a`, not `docker ps`** — act whenever the container *exists*,
   not only while it is *running*. A container something else already stopped still has to be
   removed. This is belt-and-braces given (1), and it is the half that is robust to anything
   else stopping the container first.

Verified against a real stack, not an `alpine` stand-in:

    stopped container carla-air-sim          ← the line that never appeared before
    docker ps -a                              carla-air-sim absent
    GPU 1                                     3744 MiB → 32 MiB
    docker events   kill → stop → die exitCode=143 → destroy

That event sequence is the claim in the PR title, finally observable: TERM first (`kill`),
Unreal exits of its own accord (`die 143`, not 137), removal after (`destroy`). Previously the
`destroy` never happened at all.

Two structural tests pin it, in the style T-05 established for `--all` (which no test may pass,
because it would `pkill` a real CarlaUE4): one asserts `docker stop -t 10` appears before the
`pkill` escalation, one asserts the guard keeps its `-a`. **Both fail against the pre-fix script
and pass after** — checked by reverting `stop.sh` alone and re-running them.

Suite: **251 passed, 1 skipped.** The PR body's "248" was one commit stale; the branch tip was
at 249 before these two.

## A third thing, filed rather than fixed here

`stop.sh --all` reported **3 stragglers that it can never kill**: the sidecar and the two ROS
processes, running inside `carla-air-bridge` and `carla-air-ros`. They are uid **0** on the host
while `stop.sh` runs as uid 1000, so TERM and KILL both bounce, and the script exits `rc=1`
naming processes no amount of retrying will remove.

They are not stragglers in the sense the message means. They are container processes, and
`stack_up.sh --down` removes them cleanly. But `stop.sh` only knows about `carla-air-sim`
(`scripts/stop.sh:181`), so in the container lane the documented rule-1 teardown leaves **two
containers running**, reports failure about them, and says `simulator stopped`. Filed as its own
item; it is a scope question about which script owns the container lane, not a T-06 fix.
