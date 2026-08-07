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
  capture on the image path this project is bottlenecked by. Measured 2026-08-07 with a
  browser streaming, three drift-controlled pairs each way: the socket path costs **24.0%** of
  `/camera/rgb/image_raw`, the ROS path **6.6%**. Re-encoding to JPEG is not free, so the
  honest claim is 3.6x cheaper rather than free — and the page says the number rather than
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

import protocol  # noqa: E402
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

    def start(self):
        with self._lock:
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
            return {"simulator": True, "sidecar": True, "detail": out.strip().splitlines()[-3:]}

    def stop(self):
        """Everything: simulator, sidecar, and any ROS graph under this repo's install path."""
        with self._lock:
            r = self._run([os.path.join(PROJ, "scripts", "stop.sh"), "--all"])
            return {"stopped": True, "detail": (r.stdout or "").strip().splitlines()[-3:]}

    def stop_simulator(self):
        """Just the simulator. `run_sim.sh --kill` matches the process NAME, never a pattern
        typed into a shell — `pkill -f` would also match this server's own command line, and
        `drone-sim`'s nodes. That distinction cost a sibling project's perception stack once
        already."""
        with self._lock:
            r = self._run([os.path.join(PROJ, "scripts", "run_sim.sh"), "--kill"])
            return {"stopped": "simulator", "detail": (r.stdout or "").strip().splitlines()[-2:]}

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
                status["source"] = (ROS.health() if ROS is not None
                                    else {"source": "socket", "reason": ros_source.RosSource.import_error})
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
                self._json({"result": CONTROL.call(method, **body.get("args", {}))})
            else:
                self._json({"error": "not found"}, 404)
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
