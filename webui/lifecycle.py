#!/usr/bin/env python3
"""Which deployment the console's start/stop buttons should drive.

**R-03 step 3.** Starting and stopping the simulator can never be ROS calls — there is no graph
to call into before the simulator exists, and the stop button's whole job is to destroy the
graph. That is R-03's permanent carve-out, and it is not what this step changes.

What it changes is *which* processes those buttons reach. They always drove the **host** scripts,
which was right when the console only ran against a host-native bringup. Since 2026-08-06 the
stack can be containerised, and then:

* **Start was actively wrong.** `run_sim.sh` starts a host-native `CarlaUE4`, so pressing Start
  while the containerised stack is up gives you a *second* simulator competing for GPU 1 —
  not the one you are looking at.
* **Stop simulator silently did nothing**, because it only kills a host process.
* **Stop everything happened to work**, because `stop.sh --all` learned to `docker rm -f` the
  container.

The decision is pure and lives here so it can be tested without Docker, a simulator or a graph.

## Why "am I in the stack" is passed in, not detected

The obvious check is `/run/.containerenv` or `/.dockerenv`. **On this machine that is always
true**: the whole project runs inside a podman container called `drone-sim`, so a marker-file
check cannot tell "I am the console inside the stack" from "I am the console in the ordinary
development environment", and would refuse the stop button in the normal case.

So `scripts/webui.sh --in-stack` sets `TESTBED_IN_STACK=1` explicitly when it delegates to
`stack_run.sh`. Explicit beats inferred, and here inferred is simply wrong.
"""
from __future__ import annotations

#: The two deployments the buttons can drive.
#:
#: There was a `scripts_for()` table here mapping each deployment to its commands. It was
#: deleted 2026-08-08: `server.py` never called it, so three tests asserted on a parallel
#: structure that drove nothing — and it had already drifted, claiming Start runs
#: `stack_up.sh` when the real Start reports the stack is up and runs nothing at all. One
#: test of the shipped path beats three of a shadow of it.
HOST = "host"
CONTAINER = "container"


def deployment(stack_running):
    """`container` when the containerised stack is up, otherwise `host`.

    Deliberately decided per press rather than once at startup: the console outlives the stack,
    and someone who stops a containerised stack and brings up a host-native one must not have
    their next button press aimed at the deployment that is gone.
    """
    return CONTAINER if stack_running else HOST


def refusal(action, in_stack):
    """Why this action must be refused, or None to allow it.

    **The one action worth refusing is stopping the stack the console is running inside.** With
    `--in-stack` the console's container joins the simulator container's network and IPC
    namespaces, so `docker rm -f` on the simulator takes the console down with it — mid-request.
    The browser sees a dropped connection, not an error, and nothing anywhere says what
    happened. That is the worst possible failure shape: destructive, and silent about it.

    Refusing is the same posture the contention guard takes — decline and say why, rather than
    either doing it or greying the button out with no explanation.
    """
    if in_stack and action in ("stop_all", "stop_sim"):
        return ("this console is running INSIDE the stack, and stopping it would kill this "
                "server mid-request — you would see a dropped connection rather than a result. "
                "Run ./scripts/stop.sh --all from a shell on the host instead.")
    return None


class Refused(RuntimeError):
    """A lifecycle action declined because performing it would destroy the caller.

    Distinct from a failure: nothing went wrong, and the operator has a clear alternative.
    Surfaced as HTTP 409 so the page can say so rather than showing a red error.
    """
