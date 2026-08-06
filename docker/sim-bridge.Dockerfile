# The 3.10 sidecar: owns the CARLA and AirSim clients, serves the ROS 2 side over a socket.
#
#   docker build -f docker/sim-bridge.Dockerfile -t carla-air/sim-bridge:1 .
#
# **Ubuntu 22.04 because it ships CPython 3.10 natively**, which is the whole reason this
# image is easier than the host setup: no uv, no standalone interpreter build, no vendor/
# python tree. The 3.10 requirement is not a preference — `libcarla` is an ABI-tagged
# cpython-310 extension and ROS 2 Jazzy is 3.12, which is why this project is two processes
# at all.
#
# The repository is MOUNTED, not copied. Same call as the simulator's 18 GB release: the
# environment is the slow, stable part and belongs in the image; the code changes every few
# minutes and belongs on a bind mount. A fully hermetic image that bakes the source is a
# reasonable follow-up, and is not what makes this stack reproducible today.
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3.10 python3.10-venv python3-pip \
        curl ca-certificates \
    # opencv-python needs these at import time even headless; without them `import cv2`
    # fails with a bare libGL error that names nothing about OpenCV.
        libgl1 libglib2.0-0 \
 && rm -rf /var/lib/apt/lists/*

# Order matters, and the comments in scripts/setup_env.sh explain why at length. airsim 1.8.1
# declares neither numpy nor setuptools as build dependencies but imports numpy at setup.py
# time, so it must go last with --no-build-isolation.
#
# numpy<2 is not cosmetic: airsim 1.8.1 predates the NumPy 2 ABI break, and
# msgpack-rpc-python pins tornado 4.5.3, which is likewise 3.10-era.
#
# PyYAML, av and pytest are NOT optional: apply_config.py reads the config before anything
# starts, h264.py is the only path to H.264 (opencv ships no libx264), and pytest is the
# command the docs tell a new user to run.
RUN python3.10 -m pip install --no-cache-dir --upgrade pip \
 && python3.10 -m pip install --no-cache-dir \
      "numpy<2" opencv-python pygame Pillow setuptools wheel msgpack-rpc-python \
      PyYAML av pytest \
 && python3.10 -m pip install --no-cache-dir --no-build-isolation airsim

# The CARLA client module. NOT the PyPI `carla` wheel — CARLA-Air is a fork with a different
# server ABI — and NOT the copy inside the release, whose RPATH is hardcoded to the upstream
# author's own machine and whose vendored libs expect a system that no longer exists.
#
# The repository's copy is properly auditwheel-vendored and imports with no system
# dependencies at all. Pinned by commit and checksummed, because a raw URL follows a moving
# ref otherwise. Same SHA and MD5 as scripts/setup_env.sh — keep them in step.
ARG CARLAAIR_REPO_SHA=d70247b52043d6eadb849ea41cac861ad8567dba
ARG CARLA_MODULE_MD5=a3fa4579f646f6ef1dcac5fa9e03c0b8
RUN set -eu; \
    site="$(python3.10 -c 'import site; print(site.getsitepackages()[0])')"; \
    curl -fsSL -o /tmp/carla.tar.gz \
      "https://raw.githubusercontent.com/louiszengCN/CarlaAir/${CARLAAIR_REPO_SHA}/env_setup/carla_python_module.tar.gz"; \
    echo "${CARLA_MODULE_MD5}  /tmp/carla.tar.gz" | md5sum -c -; \
    tar xzf /tmp/carla.tar.gz -C "$site"; \
    rm /tmp/carla.tar.gz

# Prove the environment at BUILD time rather than discovering it mid-flight. A server is not
# required for this — the client module imports and reports its version without one.
RUN python3.10 - <<'PY'
import airsim, carla, cv2, numpy, yaml, av
print("python ", __import__("sys").version.split()[0])
print("numpy  ", numpy.__version__)
print("carla  ", carla.Client("localhost", 9999).get_client_version())
print("OK — client environment ready")
PY

WORKDIR /workspace
ENTRYPOINT ["/bin/bash", "-lc"]
