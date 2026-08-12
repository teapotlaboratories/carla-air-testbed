# 2026-08-10 — the console becomes the stack's fourth container, opt-in

R-08, start to finish. The interesting parts are not the Dockerfile; they are that one of the
two defects the item was filed for **did not exist**, and that the thing most worth verifying
is the one that fails silently.

## One of the two filed defects was never real

R-08 opened with two rough edges, both attributed to nobody owning the console's lifecycle:

1. `stack_up.sh --down` leaves `carla-air-webui` running — it has to be removed by hand.
2. A stale container silently served old code, after `webui.sh --in-stack` died with
   `exit 125` (name already in use) while the previous container kept answering on the port.

**The first is false.** `down()` has swept every `^carla-air-` container since the original
containerisation commit `d5ed0ff`, and `carla-air-webui` matches that prefix like everything
else. Tested rather than argued: with the console up, `--down` printed `stopped
carla-air-webui` and left nothing at all in `docker ps -a`.

So the entry asserted a defect from reading the code — or from not reading it — rather than
from running it. It cost nothing this time because the fix direction was the same either way,
but it is the same failure rule 6 exists for, and it is worth recording that a *plan* can carry
an unverified claim just as easily as a conclusion can.

The second was entirely real, and is the one the design turns on.

## What shipped

- `docker/webui.Dockerfile` — `FROM carla-air/ros:1`, installs **nothing**. Since R-03 step 1
  the console is an `rclpy` node, so `rclpy`, `cv_bridge` and `msgpack` are already there. A
  test pins that no `pip install` or `apt-get install` appears: needing one would mean the
  console had grown a dependency the graph does not have, which is worth noticing rather than
  quietly satisfying.
- `docker/webui-entrypoint.sh` — sources ROS and the mounted workspace, then **refuses** if
  `interfaces`/`px4_msgs` are missing.
- `stack_up.sh --console` — a managed fourth container joining the simulator's network and IPC
  namespaces, with `docker rm -f` **before** `docker run`.
- `status.sh` — a `console container` row.
- `webui.sh --in-stack` — kept for the host-adjacent case, now clears a stale container first
  and says that `--console` is the managed equivalent.

## Why the entrypoint refuses instead of starting

This is the part worth writing down. Without the workspace on the path the console **does not
crash**. It falls back to the sidecar socket, which opens a *second* AirSim capture — the exact
thing R-03 step 1 removed — and the camera cost goes from 6.6% back to 24%. Nothing errors;
the panes still fill; the number just gets worse.

A quiet regression is worse than a refusal, so the entrypoint checks and exits. That check has
its own test, run against a directory with no console in it.

The same reasoning drove the `--ipc container:` assertion. The stack shares one IPC namespace
and Fast-DDS prefers shared memory, so a console outside it discovers the graph and then
receives **nothing** — no error, empty panes, topics publishing at 16 Hz the whole time. That
is measured behaviour from `stack_run.sh`'s own header, not a guess, and it is pinned.

## Verified against a real stack

| criterion | result |
|---|---|
| default bringup keeps the invariant | 3 containers, `ros2 node list` = `/carla_air_bridge` **alone**, nothing on :8080 |
| `--console` brings up four | 4 containers, `GET /` → 200, node list = bridge **+ `/carla_air_webui`** |
| starting twice replaces a stale one | `41bb69a400e4` → `e076e91a0675`, one container by that name, still 200 |
| `--down` leaves none | `stopped carla-air-webui`, and no `carla-air-*` in `docker ps -a` |

**The replacement test was set up as the incident, not as a happy path.** Before the second
run the console container was deliberately left *stopped but present* — `Exited (137)`, still
holding the name — because that is the shape that produced the wrong diagnosis on 2026-08-07.
Replacing a *running* container is the easy case and would have proved less.

**And the console is really on ROS in there**: `/api/status` reported `source: ros`, video
0.16 s old, state 0.03 s old. That is the check that would have caught a silent fallback, and
the reason it is the first thing looked at rather than "does the page load".

## The invariant, which is the whole reason for the flag

`CLAUDE.md` says that after a bringup `ros2 node list` is `/carla_air_bridge` alone — "you
bring the agent". The console has been an `rclpy` node since R-03 step 1, so on-by-default and
that invariant cannot both hold. Opt-in was the operator's call and the help text now says
*why*, because a flag whose reason is not written down becomes a default the next time someone
finds it inconvenient.

Checked in both directions rather than assumed: one node without `--console`, two with it.

## Tests

11, no Docker and no GPU. Behavioural for the paths that exit during argument parsing —
`--help`, a refused `--consle`, `--console` with no `--config` — and **structural** for
everything else, because `--config PATH --console` brings up a real simulator and no test may
pass it. That is the same rule `--all` has in `test_stop_args.py`, and for the same reason.

Three were mutation-checked, since a structural test that cannot fail is decoration: flipping
`CONSOLE=0` to `1`, deleting the pre-emptive `docker rm -f`, and dropping `--ipc container:`
each turned exactly one test red.

**262 passed, 1 skipped.**

## The self-review found three, and one of them broke the documented example

Reviewing my own diff before proposing it, as every merge this week has earned:

**The health probe contradicted the help text I had just written.** It curled
`http://127.0.0.1:8080/` unconditionally, while `TESTBED_CONSOLE_ARGS` is documented — with
`--bind netbird` as *the example* — as the way to change where the console listens. Using the
documented example would have made a healthy console spend 20 s failing a probe and then print
`WARNING: … did not answer on :8080`. A check that cries wolf on its own documented usage is
worse than no check, because the next person learns to ignore it.

Now the probe runs only when the address is actually known (default arguments, and `curl`
present); otherwise it says what it did *not* check. The container-exited check runs either
way, because that is the failure that matters. Verified both ways:

    default        → "console up on http://127.0.0.1:8080"          200
    --port 8099    → "console container up — not probed, TESTBED_CONSOLE_ARGS=… decides the
                      address"                                       200 on :8099

**The image check ran at step 4, after a 55 s bringup.** Nothing builds `carla-air/webui:1`
automatically, so "not built yet" is the *ordinary* first-run case, and being told about it
only after the stack is up — with one step failed and three containers running — is a bad way
to find out. Moved above step 1, where the config check already lives for the same reason.
Verified: a bogus image name now exits immediately with `Nothing was started.`, and
`docker ps -a` confirms it.

**`curl` was assumed present.** Same spurious-warning outcome on a machine without it; now
part of the probe guard.

And a fourth, caught by running it rather than reading it: my own "not probed" message printed
`custom arguments--port 8099`, because I wrote `${VAR:+a}${VAR:-b}` as though the two branches
were exclusive. They are not — when `VAR` is set, `:+` gives `a` and `:-` gives the *value*.
Replaced with a plain `if`. Small, but it is the third time this week a message has said
something untrue while the mechanism underneath worked.

Two more tests, both mutation-checked: moving the image check back into step 4 and making the
probe unconditional each turn one test red.

## One thing found on the way

`docker/webui-entrypoint.sh` was not executable in the repository — the Dockerfile `chmod +x`es
it inside the image, so the container was fine and only the test that runs it directly failed.
Fixed at the source; the Dockerfile's `chmod` stays as belt-and-braces for a checkout that
loses the bit.
