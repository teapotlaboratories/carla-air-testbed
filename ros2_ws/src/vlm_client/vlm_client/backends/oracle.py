"""The oracle: a backend that knows the answer. A diagnostic, never a competitor.

Every other backend is scored on how well it navigates. This one is scored on whether the
*scenario* is navigable at all — which is a question you must be able to answer before any
model's number means anything.

Without it, a failed episode is ambiguous in a way that cannot be resolved by looking
harder: the geometric baseline finished 88 m from the goal on `cross_the_plaza`, and nothing
in that result distinguishes "the baseline is weak" from "the goal is inside a building".
With an oracle, the ambiguity collapses:

* oracle succeeds -> the scenario is sound, and every other backend's score is a real
  measurement of that backend,
* oracle fails    -> the scenario is broken, and every number ever collected from it is
  noise. Fix the scenario, discard the results.

**It deliberately breaks the backend contract**, and that is why it is in its own file
rather than alongside the mocks. `VlmBackend.annotate` takes an image and an instruction on
purpose — the model must not see poses or metres, or the comparison between models stops
being fair. The oracle is handed the goal and the camera pose through a side channel. It is
therefore *not* an upper bound on VLM performance and must never be reported in the same
table as one; it is an upper bound on what the *harness plus controller* can achieve, which
is a different and much more useful thing to know.
"""
from __future__ import annotations

import math
import os
import sys

from .base import Annotation, VlmBackend

# frames.py lives on the 3.10 sim side but is pure maths with no carla/airsim imports, so
# the 3.12 ROS side can load it directly rather than duplicating the projection.
_FRAMES = os.environ.get(
    "TESTBED_FRAMES",
    os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))))
        , "sim_bridge", "carla_air"),
)
if _FRAMES not in sys.path:
    sys.path.insert(0, _FRAMES)

from frames import Intrinsics, project  # noqa: E402


class OracleBackend(VlmBackend):
    """Annotate the pixel the goal projects to. Flies straight at it."""

    name = "oracle"

    def __init__(self):
        self._goal = None          # world NED
        self._cam_pos = None
        self._cam_quat = None
        self._hfov_deg = 90.0
        self.off_screen = 0        # how often the goal left the frame — a scenario smell

    def set_context(self, goal_ned, cam_pos, cam_quat, hfov_deg):
        """Side channel. See the module docstring for why this is not part of the contract."""
        self._goal = goal_ned
        self._cam_pos = cam_pos
        self._cam_quat = cam_quat
        self._hfov_deg = hfov_deg

    def describe(self):
        return {"backend": self.name, "goal": self._goal, "off_screen": self.off_screen}

    def annotate(self, image, instruction, history=None):
        h, w = image.shape[:2]
        if self._goal is None or self._cam_pos is None:
            return None

        intr = Intrinsics(width=w, height=h, hfov_deg=self._hfov_deg)
        uv = project(self._goal, intr, self._cam_pos, self._cam_quat)

        if uv is None:
            # Goal is behind us. Annotating a frame edge would make the controller yaw
            # toward it, which is the right recovery and is what a competent model would do.
            self.off_screen += 1
            return Annotation(u=w - 1, v=h // 2, confidence=0.2,
                              rationale="goal is behind the camera; turning")

        u, v, depth = uv
        on_screen = 0 <= u < w and 0 <= v < h
        if not on_screen:
            self.off_screen += 1
        # Clamp into the frame: the grounding node needs a valid pixel, and the direction is
        # still correct even when the target is past the edge.
        u = min(w - 1, max(0, int(round(u))))
        v = min(h - 1, max(0, int(round(v))))

        dist = math.dist(self._goal, self._cam_pos)
        return Annotation(
            u=u, v=v,
            confidence=1.0 if on_screen else 0.5,
            rationale=(f"goal at {dist:.0f} m projects to ({u},{v}), depth {depth:.0f} m"
                       + ("" if on_screen else " [off-screen, clamped]")),
            terminal=dist < 15.0,
            metadata={"distance_m": dist, "on_screen": on_screen},
        )
