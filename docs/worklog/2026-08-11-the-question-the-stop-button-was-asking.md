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

---

# R-03 step 4 — the chase camera, and a lifetime that follows a browser tab

The last piece of R-03, and the one that had been deferred twice as "an `image_transport`
pipeline nobody else consumes". It is now a topic that nobody consumes *most of the time*, and
that is the entire design.

## The property worth protecting

The chase camera is a CARLA sensor **created on demand** — `chase_jpeg` returns `None` when
nobody has asked for one — and it is a full extra render pass. Measured, and already in the
code before this work: 30 Hz @1080p leaves the simulator at 59.8 of 60.0 fps, 60 Hz @1080p
takes it to 49.5. Real-time factor stays 1.000 through all of that, so it is judged on tick
rate and never on RTF.

An always-on publisher would have spent that permanently to serve a topic with, most of the
time, no consumer. So the lifetime runs the whole way down the chain:

    browser tab → console subscription → bridge subscription count → CARLA sensor

Verified end to end: console up with the pane never opened, sensor **down**; pane opened,
sensor **up** and ~9 MB of frames in 10 s; tab closed, sensor **down** again after the
hold-off.

## Two owners, one sensor, fixed at spawn

A CARLA sensor's resolution and tick cannot change after it is created, so `_ensure_chase`
respawns whenever a caller wants a different spec. That was survivable while it took a human
doing two conflicting things. A subscription-driven topic makes it something `ros2 topic echo`
can do by accident, to a sensor an mp4 is being written from.

**A recording outranks a viewer**, decided by the operator. Exercised as the real thing rather
than a happy path: a 1920×1080 recording started first, then a subscriber configured for
1280×720. Published frames measured 1920×1080, the sensor was kept when the subscriber left
mid-recording, and the file decoded as 440 frames, 0 dropped, 24.1 s.

**And I had the consequence wrong in the plan.** I wrote that destroying the sensor under a
recording would leave an mp4 without a moov atom — unplayable. It does not:
`ChaseCamera.destroy()` calls `stop()`, which drains the queue and closes the writer. The file
is finalised. What you get is a **playable recording that is silently short**, which is worse
in the way this project keeps paying for. Corrected in the plan in place before writing any
code against it.

## The leak I nearly shipped

`/api/chase/view` is the call the page makes after Start, so the pane has something to show. On
the socket it brings the camera up. Ported naively it would have kept calling `chase_view` —
which takes a **claim** in the sidecar that the console has no `chase_release` to pair with,
because step 4 gave that job to the subscription.

The claim count would have stuck at one forever, so unsubscribing the pane would never release
the sensor and the render pass would have run for the rest of the session — the exact cost the
design exists to remove, reintroduced by the button that used to be necessary. Found by asking
what the page still called rather than by anything failing.

## What the measurements say, honestly

The chase topic costs `/camera/rgb/image_raw` nothing measurable. Alternated twice rather than
run once, because one pair sits inside this baseline's own drift:

    unsubscribed   3.754 Hz   3.846 Hz
    subscribed     3.721 Hz   3.856 Hz

The subscribed figures land *inside* the gap between the two baselines. So the claim is **no
measurable cost with a detection floor of about 2.4%** — not "free". Same discipline as the
−6.6% console measurement, and for the same reason: the first version of that one reported a
10% drop that was entirely drift.

## A test that could not fail, again

Mutation-checking the sidecar tests caught that my own fixture left `_chase_thread` as `None`,
so `chase_stop`'s park-the-follower branch never executed and the test for it passed against
the mutated code. That is the third fake this week that answered a constant — after `_FakeRun`
returning the same `docker ps` output before and after a removal, and the stale-bytecode
diagnostic in August.

The console-side tests **stub `sensor_msgs` rather than skipping it**. An `importorskip` would
have been easier and would have meant those seven tests never ran in the documented offline
suite, which is the 3.10 venv with no ROS messages on it. A test that does not run is not a
test; it is a comment with a decorator.

## R-03 is closed

Five `*.call(` sites remain in the console and all five are the socket-mode fallback, reached
only when `ROS is None`. A literal zero is only reachable by deleting the fallback, which would
make the console unusable without a graph — the same qualification step 2 recorded, still
honest.
