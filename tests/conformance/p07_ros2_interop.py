#!/usr/bin/env python3
"""P07 — can the CARLA-Air clients live in the same process as ROS 2 Jazzy?

Upstream's ROS 2 example works by putting the conda env's site-packages on
PYTHONPATH ahead of ROS 2's system Python, so `import carla` and `import rclpy`
coexist. That trick has one precondition nobody states: ROS 2 Humble and the
CARLA-Air client are both CPython 3.10, so the same interpreter can load both.

drone-sim standardised on Jazzy (Python 3.12). This probe checks whether the
same trick survives that, by importing the shipped extension under the ROS 2
interpreter. It is expected to fail — the point is to record *how* it fails, and
therefore what a bridge would have to look like.

Run with the ROS 2 interpreter, not the venv:
    /usr/bin/python3 probes/p07_ros2_interop.py
"""
import glob
import os
import subprocess
import sys

import common  # noqa: E402  (only for Probe/OUT; imports nothing heavy)

p = common.Probe("p07_ros2_interop")

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VENV_SITE = os.path.join(PROJ, ".venv", "lib", "python3.10", "site-packages")

so = glob.glob(os.path.join(VENV_SITE, "carla", "libcarla.cpython-*.so"))
p.check("shipped CARLA extension found", bool(so), so[0] if so else VENV_SITE)
abi = os.path.basename(so[0]).split(".")[1] if so else "?"
p.metric("carla_extension_abi_tag", abi)

ros_distro = os.environ.get("ROS_DISTRO", "")
ros_py = subprocess.run(
    ["bash", "-lc", "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1 && python3 -c 'import sys;print(sys.version.split()[0])'"],
    capture_output=True, text=True,
).stdout.strip()
p.metric("ros2_distro_on_box", ros_distro or "jazzy (not sourced in this shell)")
p.metric("ros2_python", ros_py or "unknown")

major_minor = "cp3" + ros_py.split(".")[1] if ros_py else ""
compatible = abi.replace("cpython-", "cp") == major_minor
p.check(
    "ROS 2 interpreter can load the CARLA extension (same ABI)",
    compatible,
    f"extension is {abi}, ROS 2 Python is {ros_py}",
)

# Prove it rather than infer it.
res = subprocess.run(
    ["bash", "-lc",
     f"source /opt/ros/jazzy/setup.bash >/dev/null 2>&1 && "
     f"PYTHONPATH={VENV_SITE}:$PYTHONPATH python3 -c 'import rclpy, carla; print(\"both imported\")'"],
    capture_output=True, text=True,
)
p.check("import rclpy + carla in one interpreter", res.returncode == 0,
        (res.stdout or res.stderr).strip().splitlines()[-1] if (res.stdout or res.stderr) else "")

# The reverse direction: does rclpy load under the 3.10 venv?
venv_py = os.path.join(PROJ, ".venv", "bin", "python")
res2 = subprocess.run(
    ["bash", "-lc",
     f"source /opt/ros/jazzy/setup.bash >/dev/null 2>&1 && {venv_py} -c 'import rclpy; print(\"rclpy ok\")'"],
    capture_output=True, text=True,
)
p.check("rclpy (Jazzy) imports under the 3.10 client venv", res2.returncode == 0,
        (res2.stdout or res2.stderr).strip().splitlines()[-1] if (res2.stdout or res2.stderr) else "")

p.note("if both directions fail", "the clients and the ROS 2 graph must be separate processes "
                                  "with an IPC hop between them — not the in-process bridge upstream ships")
p.finish()
