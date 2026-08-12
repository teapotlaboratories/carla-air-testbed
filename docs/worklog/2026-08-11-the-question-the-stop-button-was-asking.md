# 2026-08-11 — the stop button was asking the wrong question, and the obvious fix was worse

T-08, filed while reviewing PR #9 and fixed today. Small in diff, and worth a log for one
reason: the first fix that came to mind would have reintroduced this project's oldest incident.

## The defect

`stop_simulator()` chose its lane with `lifecycle.deployment(self.stack_running())`, and
`stack_running()` asks `docker ps` — **running containers only**. So a `carla-air-sim` that
existed but was *stopped* looked identical to no container at all. The button took the host
lane, ran `run_sim.sh --kill` against a host process that was not there, and returned success.

Demonstrated against real Docker with a container in exactly that state:

    before   {"stopped": "simulator", "deployment": "host", "detail": ["stopped"]}
             leftover container: still there

Nothing in the reply mentioned the container, because nothing had looked.

PR #9's whole thesis was that *running* and *exists* are different questions. It answered that
correctly **inside** the container branch while the check that selects the branch still
conflated them — which is why this was filed rather than folded into that PR.

## Why `stack_running()` is not the thing to fix

The one-line fix is to make it ask `docker ps -a`. That is wrong, and it is wrong in a
direction that would not have shown up in these tests: `stack_running()` also drives the
**Start** button, where it means "is the stack up". Widening it to "does a container object
exist" would make Start believe a dead stack was alive and refuse to start anything.

So the stop path got its own, finer probe — `sim_container_state()`, three answers instead of
a boolean — and `stack_running()` was left alone.

## The obvious fix was worse than the bug

Given three states, the natural mapping is *running or stopped → container lane, absent → host
lane*. There is a container, so use the container path. It reads well and it is wrong.

**A host-native simulator can be running at the same time as a leftover container object.**
Under that mapping the button would remove the leftover, report `deployment: container`, never
run `run_sim.sh --kill`, and leave 3.3 GB of VRAM held while telling the operator the simulator
had been stopped. That is the 2026-08-03 incident — success reported while the GPU was still
full — arriving by a new route.

The distinction that resolves it: **a merely-present container is litter, not a deployment.**
Only a *running* container means the simulator lives in the container lane. So the leftover
sweep rides on the host path, separately from deciding where the simulator actually is:

| container | lane | what happens |
|---|---|---|
| running | container | `docker stop -t 10` → `rm -f` → verify (unchanged) |
| **stopped** | **host** | `run_sim.sh --kill` **and** the leftover is removed and named |
| absent | host | `run_sim.sh --kill`, nothing claimed about containers |

`test_a_stopped_container_does_not_hide_a_running_host_simulator` exists specifically to fail
when someone takes the shortcut, and it does: mutating the gate to the natural-looking version
turns it red along with two others.

    after    {"deployment": "host", "detail": ["stopped",
              "also removed a leftover stopped container carla-air-sim"]}
             leftover container: gone

## The fake was answering a constant

Five existing tests had to change, and one of them exposed a hole worth recording. They
injected `stack_running`, which is no longer what decides — fine, mechanical. But `_FakeRun`
returned the **same** `docker ps` output before and after a `docker rm`, so every "is it
actually gone" assertion was checking a value that could not move. The tests passed for the
right reason only by accident of how they were written.

`_FakeRun` now models what a removal *achieves* — `remove`, `stop_only`, `nothing` — which is
the thing under test. That is the same lesson as the stale-bytecode diagnostic on 2026-08-07
and the `ss`-cannot-see-a-UDP-client one before it: **the tool was not measuring what I
believed it was measuring, and nothing errored.**

## Verified

- Real stack, container running: `state = running`, `deployment: container`, container gone,
  GPU 1 back to 32 MiB. Unchanged, as intended.
- Real Docker, container stopped-but-present: before/after as above.
- Probe with no `docker` binary at all falls back to the host lane rather than raising.
- **271 passed, 1 skipped.**

## Still open

T-07 — `stop.sh --all` leaves `carla-air-bridge` and `carla-air-ros` running and reports three
"stragglers" it cannot kill. Untouched here, and still a scope question about which script owns
the container lane rather than a bug in a line of code.

---

# T-07 — the container lane gets an owner

Decided by the operator the same day: `stop.sh --all` **delegates** to `stack_up.sh --down`
rather than growing its own sweep.

## Why delegation rather than the other two

The three options were: hand over; widen `stop.sh`'s container matching to the `carla-air-`
prefix as `stack_up.sh` already does; or document that the container lane's teardown is a
different command.

Widening duplicates `down()`'s sweep in two scripts, and this project has already deleted one
parallel structure that drifted from what it described. Documenting leaves rule 1 with two
commands, and the wrong one silently half-works — the worst shape, because it looks like it
did something.

Handing over keeps **one** command true in both lanes.

## The ordering does two jobs

The handover runs *before* the kill escalation, not after. With the containers already gone,
`targets()` finds nothing container-internal to misreport and the escalation wastes no
`sleep 2` failing to signal processes it could never signal. The fix for the ownership and the
fix for the false alarm are the same edit — which is why the test that pins the ordering is
worth more than it looks.

Measured against a live four-container stack:

| | before *(2026-08-10)* | after |
|---|---|---|
| `stop.sh --all` | 8.23 s, **exit 1** | **1 s, exit 0** |
| reported | `3 stragglers`, `simulator stopped` | `0 stragglers, 0 in containers` |
| containers left | **2 running** | **0** |

The 8× speedup is incidental but tells the story: most of that time was three signal rounds
against processes that bounced every one.

## The false alarm needed wording, not hiding

Plain `stop.sh` — no `--all` — still cannot do anything about the containers, and should not:
"the graph and the sidecar, leaving the simulator up" has no containerised equivalent, because
here they *are* containers and `stack_up.sh --down` would take the simulator too. So that case
is told rather than guessed at.

But it was still printing `4 process(es) survived TERM and KILL`, and **survived** is a claim
about resistance. Those processes are uid 0 on the host while the script is uid 1000; every
signal bounced with EPERM. Nothing resisted anything. Someone reading that goes looking for a
wedged process that was never wedged.

`kill -0` splits the two populations, and it is the right instrument because it asks the actual
question — *could this script have acted on it?* — rather than inferring containerhood from a
uid or a PID namespace, which are both proxies that can be wrong. Both populations still set a
non-zero exit: the machine is not clean either way. Only the sentence changed, because only the
sentence was false.

That is the fourth message this week that was untrue while the mechanism under it worked
correctly — after the borrowed "blocks the next start", the health probe that would have cried
wolf on its own documented example, and `custom arguments--port 8099`. The pattern is worth
naming: **the code gets reviewed and the strings do not.**

## Verified

- Live four-container stack: `--all` hands over, 0 containers left, GPU 1 at 33 MiB, exit 0.
- Same stack, plain `stop.sh`: stack untouched (4 containers still up), accurate wording.
- Host lane with nothing running: unchanged, exit 0.
- **274 passed, 1 skipped.** Three new structural tests, all mutation-checked.
