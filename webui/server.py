#!/usr/bin/env python
"""A web console for the testbed: start the simulator, watch both cameras, fly it by hand.

    ./.venv/bin/python webui/server.py            # then open http://localhost:8080

Everything else in this project drives a *scripted* episode. There was no way to simply look
at the simulator, or to fly somewhere and see what the camera sees — which is what scenario
design actually needs, and what turned three flight-test failures into log archaeology.

**Onboard video and telemetry come from ROS 2 topics** when this runs somewhere `rclpy` is
importable — R-03 step 1. Control and the chase pane still go over the Unix socket to
`sim_bridge`; steps 2 and 4 move those. Which source is in use is printed at startup and
reported by `/api/status`, because the one thing this must never do is degrade quietly.

Four decisions worth knowing:

* **Stdlib `ThreadingHTTPServer`.** `tornado` is importable, but only because `msgpackrpc`
  depends on it, and msgpackrpc runs its own IOLoop for the AirSim connection. Sharing that
  library invites the same class of bug that cost a flight test earlier — two users, one
  event loop.
* **MJPEG (`multipart/x-mixed-replace`).** It renders in a plain `<img>` with no client-side
  decoding and no extra dependency. Both sources run near 10 Hz, so the latency a real video
  codec would buy back is not there to win.
* **One socket per concern.** Streams and control each get their own connection to the
  sidecar. Sharing one would interleave a control write with a 40 kB frame read, which is
  the failure this project has already paid for once.
* **On ROS it contends far less — not zero.** The onboard view used to be a *second* AirSim
  capture on the image path this project is bottlenecked by. Measured 2026-08-07, three
  drift-controlled pairs each way, against a baseline of **the console running but not
  streaming**: the socket path costs **24.0%** of `/camera/rgb/image_raw`, the ROS path
  **6.6%**. Two caveats that belong with those numbers: re-encoding to JPEG is not free, and an
  idle ROS console still converts every frame, so its idle cost is inside the baseline and
  6.6% is a floor rather than a total. The page states the number rather than the word
  "safe".
"""
from __future__ import annotations

import argparse
import hmac
import json
import os
import secrets
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PROJ, "sim_bridge"))
sys.path.insert(0, HERE)

import lifecycle  # noqa: E402
import protocol  # noqa: E402
import ros_control  # noqa: E402
import ros_source  # noqa: E402

SOCKET = os.environ.get("TESTBED_SOCKET", protocol.DEFAULT_SOCKET)

#: The live ROS subscriber, or None when video and telemetry are on the socket. Set once in
#: `main()` and read from the request threads; never reassigned after the server starts.
ROS = None

#: Set when the server is reachable from anything but this machine. Every /api and /stream
#: request must then carry `?k=<token>`.
TOKEN = None


def netbird_address():
    """The wt0 (NetBird) address, or None. Read from `ip`, not guessed."""
    try:
        out = subprocess.run(["ip", "-o", "-4", "addr", "show", "wt0"],
                             capture_output=True, text=True, timeout=5).stdout
    except Exception:  # noqa: BLE001
        return None
    for part in out.split():
        if "/" in part and part.count(".") == 3:
            return part.split("/")[0]
    return None


class Sim:
    """One connection to the sidecar, with a lock. Create one per concern, not one shared."""

    def __init__(self, path=SOCKET, timeout=30.0):
        self.path, self.timeout = path, timeout
        self._sock = None
        self._lock = threading.Lock()
        self._id = 0

    def _connect(self):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        s.connect(self.path)
        self._sock = s

    def call(self, method, **args):
        with self._lock:
            if self._sock is None:
                self._connect()
            self._id += 1
            try:
                protocol.send(self._sock, {"id": self._id, "method": method, "args": args})
                reply = self._recv_reply()
            except (OSError, EOFError):
                # A dropped socket is normal here: the sidecar is started and stopped from
                # this very page. Reconnect once rather than surfacing it as a UI error.
                self._sock = None
                self._connect()
                protocol.send(self._sock, {"id": self._id, "method": method, "args": args})
                reply = self._recv_reply()
        if not reply.get("ok"):
            raise RuntimeError(reply.get("error", "unknown sidecar error"))
        return reply.get("result")

    def _recv_reply(self):
        """Read frames until an actual reply arrives, skipping progress.

        The sidecar interleaves `{"id": N, "progress": "..."}` frames into long calls so a
        caller can tell slow from wedged. This console is synchronous and does not use them,
        but it must still consume them - a progress frame left in the stream would be
        returned as the next call's result.
        """
        while True:
            frame = protocol.recv(self._sock)
            if "ok" in frame:
                return frame

    def close(self):
        with self._lock:
            if self._sock is not None:
                self._sock.close()
                self._sock = None


class Processes:
    """Start and stop the simulator and the sidecar, using the project's own scripts.

    Deliberately shells out to `scripts/run_sim.sh` and `scripts/stop.sh` rather than
    reimplementing either: those carry the Vulkan ICD repair, the GPU pin, the VRAM check
    that catches software rendering, and the path-scoped process matching that keeps
    `drone-sim` alive. A second implementation would drift from them silently.
    """

    def __init__(self):
        self.log = []
        self._lock = threading.Lock()

    def _run(self, argv, env=None, background=False):
        merged = dict(os.environ)
        merged.setdefault("TESTBED_GPU", "1")   # GPU 0 is the operator's; see .ai/AGENTS.md
        if env:
            merged.update(env)
        if background:
            return subprocess.Popen(argv, cwd=PROJ, env=merged,
                                    stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        return subprocess.run(argv, cwd=PROJ, env=merged, capture_output=True, text=True,
                              timeout=600)

    #: Set by `scripts/webui.sh --in-stack`. NOT inferred from /run/.containerenv, which is
    #: present for the whole project on this machine and would refuse the stop button always.
    IN_STACK = os.environ.get("TESTBED_IN_STACK") == "1"

    def stack_running(self):
        """Is the containerised stack up? Asked per press, not cached — the console outlives
        the stack, and a button aimed at a deployment that is gone is worse than no button.

        **Being inside the stack is proof the stack exists**, and that shortcut is not an
        optimisation — it is required. The ROS image ships no `docker` binary, so a console
        started with `--in-stack` gets `OSError` from the probe below, concludes there is no
        stack, and takes the HOST path: pressing Start then asks for `CARLAAIR_RELEASE` and
        offers to launch a second simulator. Found by finally running `--in-stack`; every unit
        test passed because they inject this answer rather than compute it.
        """
        if self.IN_STACK:
            return True
        try:
            names = subprocess.run(["docker", "ps", "--format", "{{.Names}}"],
                                   capture_output=True, text=True, timeout=10).stdout.split()
        except (OSError, subprocess.SubprocessError):
            return False
        return os.environ.get("TESTBED_SIM_CONTAINER", "carla-air-sim") in names

    def _guard(self, action):
        why = lifecycle.refusal(action, self.IN_STACK)
        if why:
            raise lifecycle.Refused(why)

    def start(self):
        with self._lock:
            target = lifecycle.deployment(self.stack_running())
            if target == lifecycle.CONTAINER:
                # The stack is already up — starting the HOST simulator here would give a
                # second CarlaUE4 competing for GPU 1 with the one being watched.
                return {"simulator": True, "sidecar": True, "deployment": target,
                        "detail": ["the containerised stack is already running"]}

            if not os.environ.get("CARLAAIR_RELEASE"):
                raise RuntimeError(
                    "CARLAAIR_RELEASE is not set in the environment that launched this "
                    "server — the simulator cannot be found. Export it and restart.")
            r = self._run([os.path.join(PROJ, "scripts", "run_sim.sh")])
            out = (r.stdout or "") + (r.stderr or "")
            if r.returncode != 0:
                raise RuntimeError(f"run_sim.sh failed:\n{out[-800:]}")

            if not os.path.exists(SOCKET):
                self._run([os.path.join(PROJ, ".venv", "bin", "python"),
                           os.path.join(PROJ, "sim_bridge", "server.py"),
                           "--socket", SOCKET], background=True)
                for _ in range(60):
                    if os.path.exists(SOCKET):
                        break
                    time.sleep(1.0)
                if not os.path.exists(SOCKET):
                    raise RuntimeError("the sidecar never created its socket")
            return {"simulator": True, "sidecar": True, "deployment": target,
                    "detail": out.strip().splitlines()[-3:]}

    def stop(self):
        """Everything: simulator, sidecar, and any ROS graph under this repo's install path."""
        self._guard("stop_all")
        with self._lock:
            # stop.sh --all covers BOTH deployments: it kills the host processes and, since
            # 2026-08-06, docker rm -f's the simulator container too. So this one button is
            # already deployment-agnostic and needs no branch.
            r = self._run([os.path.join(PROJ, "scripts", "stop.sh"), "--all"])
            return {"stopped": True, "detail": (r.stdout or "").strip().splitlines()[-3:]}

    def stop_simulator(self):
        """Just the simulator. `run_sim.sh --kill` matches the process NAME, never a pattern
        typed into a shell — `pkill -f` would also match this server's own command line, and
        `drone-sim`'s nodes. That distinction cost a sibling project's perception stack once
        already."""
        self._guard("stop_sim")
        with self._lock:
            if lifecycle.deployment(self.stack_running()) == lifecycle.CONTAINER:
                # run_sim.sh --kill matches a host process name and would silently do nothing.
                name = os.environ.get("TESTBED_SIM_CONTAINER", "carla-air-sim")
                # SIGTERM with a grace period first. Unreal does not always go down on the
                # first signal, and `docker rm -f` is an immediate SIGKILL — harsher than the
                # host path this replaced, which escalates. Failure here is not fatal: the
                # removal below is the hammer, and the check after it is what decides.
                self._run(["docker", "stop", "-t", "10", name])
                r = self._run(["docker", "rm", "-f", name])
                # Report what was ACHIEVED, not what was attempted. stop.sh learned this the
                # expensive way on 2026-08-03: it announced success twice while the simulator
                # held 3.5 GB of VRAM, because nothing checked. Reintroducing that one file
                # over would be worse, not better.
                # Report what was ACHIEVED. A non-zero rm is a hint, not the verdict: ask
                # whether the container is actually gone, because that is the question.
                # "Running" and "exists" are different questions. A stopped-but-present
                # container holds no GPU memory - the simulator IS down - so raising for it
                # would report a failure that did not happen. It still blocks the next start
                # by name, which is worth saying, but in the reply rather than as an error.
                detail = [f"removed container {name}"]
                running = self._run(["docker", "ps", "--format", "{{.Names}}"])
                if name in (running.stdout or "").split():
                    raise RuntimeError(
                        f"{name} is STILL RUNNING after docker stop and rm -f:\n"
                        f"{((r.stderr or '') + (r.stdout or '')).strip()[-400:]}")
                present = self._run(["docker", "ps", "-a", "--format", "{{.Names}}"])
                if name in (present.stdout or "").split():
                    detail = [f"{name} stopped but NOT removed — holds no GPU memory, but it "
                              f"will block the next start by that name"]
                return {"stopped": "simulator", "deployment": lifecycle.CONTAINER,
                        "detail": detail}
            r = self._run([os.path.join(PROJ, "scripts", "run_sim.sh"), "--kill"])
            return {"stopped": "simulator", "deployment": lifecycle.HOST,
                    "detail": (r.stdout or "").strip().splitlines()[-2:]}

    def status(self):
        r = self._run([os.path.join(PROJ, "scripts", "status.sh")])
        text = r.stdout or ""
        counts, gpu = {}, []
        for line in text.splitlines():
            parts = line.split()
            if len(parts) == 2 and parts[1].isdigit():
                counts[parts[0]] = int(parts[1])
            elif "MiB" in line and "," in line:
                gpu.append(line.strip())
        return {"counts": counts, "gpu": gpu,
                "socket": os.path.exists(SOCKET),
                "graph_up": any(counts.get(n, 0) > 0 for n in
                                ("carla_air_bridge", "vlm_client", "grounding", "control"))}


PROCS = Processes()
CONTROL = Sim()      # control + telemetry
ONBOARD = Sim()      # drone-camera stream
CHASE = Sim()        # chase-camera stream


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):        # quiet; the page is the interface
        pass

    def _authorised(self):
        """Constant-time token check. No token configured means loopback-only, so allow."""
        if TOKEN is None:
            return True
        supplied = parse_qs(urlparse(self.path).query).get("k", [""])[0]
        return hmac.compare_digest(supplied, TOKEN)

    def _deny(self):
        body = b'{"error":"missing or bad token; append ?k=<token> to the URL"}'
        self.send_response(401)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    # ---------------------------------------------------------------- helpers

    def _json(self, payload, code=200):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        n = int(self.headers.get("Content-Length") or 0)
        return json.loads(self.rfile.read(n) or b"{}")

    def _mjpeg(self, produce, fps):
        """Stream frames as multipart/x-mixed-replace until the client goes away."""
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        # An endless body has no Content-Length, and HTTP/1.1 keep-alive would leave the
        # client waiting for a length that never comes. Close-delimited is the honest framing.
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        period = 1.0 / max(1.0, float(fps))
        try:
            while True:
                started = time.time()
                try:
                    jpeg = produce()
                except Exception:  # noqa: BLE001 — sidecar down, or mid-restart
                    jpeg = None
                if jpeg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: " + str(len(jpeg)).encode()
                                     + b"\r\n\r\n" + jpeg + b"\r\n")
                    self.wfile.flush()
                time.sleep(max(0.0, period - (time.time() - started)))
        except (BrokenPipeError, ConnectionResetError):
            pass                                  # the tab was closed; not an error

    # ---------------------------------------------------------------- routes

    def do_GET(self):
        path = self.path.split("?")[0]
        if not self._authorised():
            return self._deny()
        try:
            if path in ("/", "/index.html"):
                with open(os.path.join(HERE, "index.html"), "rb") as fh:
                    body = fh.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path == "/api/status":
                status = PROCS.status()
                if ROS is not None:
                    status["source"] = ROS.health()
                    status["source"]["contested"] = ROS.contested()
                else:
                    # Distinguish "asked for the socket" from "could not have ROS". Both are
                    # degraded relative to step 2, and only one of them is a problem.
                    why = ros_source.RosSource.import_error
                    status["source"] = {
                        "source": "socket",
                        "reason": why or "asked for with --source socket",
                        "rclpy_available": why is None,
                    }
                self._json(status)
            elif path == "/api/state":
                # On ROS the numbers arrive by subscription, so this is a cache read rather
                # than two round trips to the sidecar. Falling back per-request would hide a
                # dead graph behind numbers that look fine, so a live source that has not
                # received anything yet says exactly that instead.
                if ROS is not None:
                    state = ROS.state()
                    if state is None:
                        raise RuntimeError(
                            f"no odometry on {ROS.topics['odom']} yet — is the graph up?")
                    self._json({"state": state, "collision": ROS.collision()})
                else:
                    self._json({"state": CONTROL.call("state"),
                                "collision": CONTROL.call("collision")})
            # 12 fps onboard, 30 fps chase. The onboard view is an AirSim capture and tops
            # out near 38 fps on that RPC path, which the ROS graph also uses — so it stays
            # modest. The chase view is a CARLA sensor and 30 Hz costs the simulator nothing
            # measurable (59.8 vs 60.0 fps tick rate).
            elif path == "/stream/onboard":
                # On ROS this is the frame the agent already received, re-encoded — no second
                # capture, so it costs the simulator nothing and may run during a scored
                # episode. On the socket it is a capture of its own, and 12 fps is a deliberate
                # ceiling: that RPC path tops out near 38 fps and the graph shares it.
                if ROS is not None:
                    self._mjpeg(lambda: ROS.latest_jpeg(quality=75), fps=12)
                else:
                    self._mjpeg(lambda: ONBOARD.call("view_jpeg", quality=75)["jpeg"], fps=12)
            elif path == "/stream/chase":
                self._mjpeg(lambda: CHASE.call("chase_jpeg", quality=75)["jpeg"], fps=30)
            else:
                self._json({"error": "not found"}, 404)
        except Exception as exc:  # noqa: BLE001 — a UI must not 500 silently
            try:
                self._json({"error": str(exc)}, 500)
            except Exception:  # noqa: BLE001 — headers already sent on a stream
                pass

    def do_POST(self):
        path = self.path.split("?")[0]
        if not self._authorised():
            return self._deny()
        try:
            body = self._read_json()
            if path == "/api/sim/start":
                self._json(PROCS.start())
            elif path == "/api/sim/stop":
                CONTROL.close(); ONBOARD.close(); CHASE.close()
                self._json(PROCS.stop())
            elif path == "/api/sim/stop_sim_only":
                # Leaves the sidecar alone. Useful when you want the simulator down but the
                # console still able to report status without a reconnect storm.
                CONTROL.close(); ONBOARD.close(); CHASE.close()
                self._json(PROCS.stop_simulator())
            elif path == "/api/chase/view":
                self._json(CONTROL.call("chase_view",
                                        width=int(body.get("width", 1280)),
                                        height=int(body.get("height", 720))))
            elif path == "/api/command":
                method = body.get("method")
                if method not in protocol.METHODS:
                    raise RuntimeError(f"unknown method {method!r}")
                args = body.get("args", {})
                # R-03 step 2: anything with a ROS equivalent goes over ROS. The rest — the
                # chase pane, the sidecar diagnostics — stays on the socket until step 4.
                # `via` is reported so the page can never be wrong about which path ran.
                if ROS is not None and method in ros_control.ROS_METHODS:
                    try:
                        self._json({"result": ROS.command(method, args), "via": "ros"})
                    except ros_control.Contested as exc:
                        # 409, not 500: nothing failed. The console declined to fight another
                        # publisher, which is a state the caller can resolve.
                        self._json({"error": str(exc), "contested": True}, 409)
                else:
                    self._json({"result": CONTROL.call(method, **args), "via": "socket"})
            else:
                self._json({"error": "not found"}, 404)
        except lifecycle.Refused as exc:
            # 409, not 500: nothing failed. The console declined to destroy itself, and the
            # message names what to do instead.
            self._json({"error": str(exc), "refused": True}, 409)
        except Exception as exc:  # noqa: BLE001
            self._json({"error": str(exc)}, 500)


def _open_source(choice):
    """Pick where video and telemetry come from, and say which — loudly, on stderr or stdout.

    `--source ros` **fails** rather than falling back. Someone who asked for ROS precisely to
    avoid a second AirSim capture, and silently got one, would be measuring the thing they
    were trying to eliminate. That is the T-02 lesson: a silent fallback on a measurement path
    is a bug in the fallback.
    """
    if choice == "socket":
        print("video + telemetry: sidecar socket (asked for)", flush=True)
        print("  this opens a SECOND camera capture — not during a scored episode", flush=True)
        return None

    if not ros_source.RosSource.importable():
        why = ros_source.RosSource.import_error
        if choice == "ros":
            sys.exit(f"--source ros, but this interpreter cannot be a ROS node: {why}\n"
                     f"Run it under the ROS environment (scripts/webui.sh does that), or pass "
                     f"--source socket to accept the second capture deliberately.")
        print(f"video + telemetry: sidecar socket — rclpy is not importable here ({why})",
              flush=True)
        print("  this opens a SECOND camera capture — not during a scored episode", flush=True)
        return None

    src = ros_source.RosSource().start()
    print(f"video + telemetry: ROS 2 topics — {src.topics['image']}, {src.topics['odom']}",
          flush=True)
    print("  no second capture: costs ~6.6% of the camera rate while streaming, vs ~24% on\n"
          "  the socket (measured 2026-08-07) — low enough to leave open during a scored run",
          flush=True)
    return src


def main():
    global TOKEN, ROS

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--bind", default=os.environ.get("TESTBED_WEB_BIND", "127.0.0.1"),
                    help="address to listen on, or 'netbird' for the wt0 address "
                         "(default: 127.0.0.1, this machine only)")
    ap.add_argument("--port", type=int, default=int(os.environ.get("TESTBED_WEB_PORT", 8080)))
    ap.add_argument("--token", default=os.environ.get("TESTBED_WEB_TOKEN"),
                    help="require ?k=<token>; generated automatically when not on loopback")
    ap.add_argument("--no-token", action="store_true",
                    help="serve a non-loopback address with NO authentication (don't)")
    ap.add_argument("--source", choices=("auto", "ros", "socket"),
                    default=os.environ.get("TESTBED_WEB_SOURCE", "auto"),
                    help="where onboard video and telemetry come from: ROS 2 topics when "
                         "rclpy is importable, else the sidecar socket (default: auto)")
    args = ap.parse_args()

    bind = args.bind
    if bind == "netbird":
        bind = netbird_address()
        if not bind:
            sys.exit("no wt0 (NetBird) address found — is the daemon connected?")

    loopback = bind in ("127.0.0.1", "localhost", "::1")
    # This endpoint starts processes and flies an aircraft. Off loopback it gets a token by
    # default rather than on request: an unauthenticated control surface on a mesh network is
    # reachable by every peer on it.
    if not loopback and not args.no_token:
        TOKEN = args.token or secrets.token_urlsafe(18)
    elif args.token:
        TOKEN = args.token

    # Before the socket is bound, so a --source ros that cannot be honoured exits without
    # first advertising a URL that would then serve no pictures.
    ROS = _open_source(args.source)

    server = ThreadingHTTPServer((bind, args.port), Handler)
    server.daemon_threads = True

    shown = bind if not loopback else "localhost"
    url = f"http://{shown}:{args.port}/" + (f"?k={TOKEN}" if TOKEN else "")
    # flush=True on every line: stdout is block-buffered when this is redirected to a log,
    # and the generated token would then sit unseen in the buffer — which makes the whole
    # server unreachable for the person who just started it.
    print(f"web console:  {url}", flush=True)
    print(f"  sidecar socket: {SOCKET}", flush=True)
    if TOKEN:
        print("  token required — the URL above already carries it", flush=True)
        # Also written to a file, so a backgrounded server's URL survives a lost log.
        try:
            url_file = os.path.join(PROJ, "out", "webui-url.txt")
            os.makedirs(os.path.dirname(url_file), exist_ok=True)
            with open(url_file, "w") as fh:
                fh.write(url + "\n")
            os.chmod(url_file, 0o600)          # it contains the token
            print(f"  url also written to {url_file}", flush=True)
        except OSError:
            pass
    elif not loopback:
        print("  !! NO TOKEN and not loopback: anyone who can reach this port can fly it",
              flush=True)
    print("  ctrl-c stops the server; the simulator keeps running (stop it from the page)",
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if ROS is not None:
            ROS.stop()


if __name__ == "__main__":
    main()
