"""Wire protocol between the 3.10 sim side and the 3.12 ROS 2 side.

Why this exists at all: the CARLA-Air client module is an ABI-tagged
`libcarla.cpython-310` extension and ROS 2 Jazzy is Python 3.12. Neither interpreter can
load the other's C extensions — measured in both directions — so the two halves of this
testbed are two processes, and this module is the seam.

Framing is deliberately dull: a 4-byte big-endian length followed by a msgpack object.
msgpack is already present (it is an `airsim` dependency) and both interpreters have it.
Image payloads ride as raw `bytes` with their shape in the header rather than as nested
lists, which is the difference between ~1 ms and ~200 ms per frame.

This module is imported by **both** interpreters, so it must stay dependency-free beyond
msgpack and the standard library.
"""
from __future__ import annotations

import socket
import struct

import msgpack

DEFAULT_SOCKET = "/tmp/carla_air_testbed.sock"
_HEADER = struct.Struct(">I")
MAX_FRAME = 64 * 1024 * 1024


class ProtocolError(RuntimeError):
    pass


def send(sock: socket.socket, obj) -> None:
    payload = msgpack.packb(obj, use_bin_type=True)
    sock.sendall(_HEADER.pack(len(payload)) + payload)


def _recv_exactly(sock: socket.socket, n: int) -> bytes:
    chunks, remaining = [], n
    while remaining:
        chunk = sock.recv(min(remaining, 1 << 20))
        if not chunk:
            raise ConnectionError("peer closed mid-frame")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def recv(sock: socket.socket):
    (length,) = _HEADER.unpack(_recv_exactly(sock, _HEADER.size))
    if length > MAX_FRAME:
        raise ProtocolError(f"frame of {length} bytes exceeds the {MAX_FRAME} limit")
    return msgpack.unpackb(_recv_exactly(sock, length), raw=False)


# ---------------------------------------------------------------------------
# Request/response shapes.
#
#   ->  {"id": int, "method": str, "args": {...}}
#   <-  {"id": int, "ok": bool, "result": ...}            on success
#   <-  {"id": int, "ok": false, "error": str, "traceback": str}
#
# Methods are listed here so both sides fail loudly on a typo instead of silently at
# runtime. Keep in sync with sim_bridge/server.py.
# ---------------------------------------------------------------------------
METHODS = frozenset({
    "ping",
    "describe",         # map, versions, camera intrinsics, offsets
    "reset",            # reset + leave holding a setpoint
    "state",            # vehicle kinematics
    "collision",
    "capture",          # rgb / depth / segmentation
    "ground",           # pixel -> world NED
    "velocity",         # one velocity setpoint
    "goto",             # blocking position move
    "yaw",
    "hold",
    "land",
    "set_camera_pose",
    "spawn_traffic",
    "traffic_stats",
    "watchdog",
    "set_weather",
    "destroy_actors",
    "carla_to_ned",
    "shutdown",
})


def encode_image(arr):
    """numpy array -> {shape, dtype, data} with the payload as raw bytes."""
    if arr is None:
        return None
    return {"shape": list(arr.shape), "dtype": str(arr.dtype), "data": arr.tobytes()}


def decode_image(blob):
    """Inverse of `encode_image`. Imports numpy lazily so the seam stays importable."""
    if blob is None:
        return None
    import numpy as np

    return np.frombuffer(blob["data"], dtype=np.dtype(blob["dtype"])).reshape(blob["shape"])
