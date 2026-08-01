"""Deterministic backends with no model behind them.

These are what make the testbed testable. Every one of them is reproducible from a seed, so
the whole loop — capture, ground, fly, score — can run in CI and in regression sweeps
without a GPU, an API key or a network. When a real backend's success rate moves, these say
whether the simulator or the harness moved underneath it.

`ScriptedBackend` is the useful one for regression: it replays a fixed pixel sequence, so a
change in episode outcome is unambiguously a change in the stack, not in the model.
`GeometricBackend` is the honest baseline — it is what "reasonable navigation without any
language understanding" scores, and any VLM that cannot beat it is not earning its latency.
"""
from __future__ import annotations

import random

from .base import Annotation, VlmBackend


class MockBackend(VlmBackend):
    """Seeded random pixels inside a safe central band. The floor, not a baseline."""

    name = "mock"

    def __init__(self, seed: int = 0, margin: float = 0.2, terminal_after: int | None = None):
        self._rng = random.Random(seed)
        self._margin = margin
        self._terminal_after = terminal_after
        self._calls = 0

    def annotate(self, image, instruction, history=None):
        self._calls += 1
        h, w = image.shape[:2]
        lo_u, hi_u = int(w * self._margin), int(w * (1 - self._margin))
        lo_v, hi_v = int(h * self._margin), int(h * (1 - self._margin))
        return Annotation(
            u=self._rng.randint(lo_u, hi_u),
            v=self._rng.randint(lo_v, hi_v),
            confidence=0.5,
            rationale=f"mock pixel {self._calls} for {instruction!r}",
            terminal=self._terminal_after is not None and self._calls >= self._terminal_after,
        )


class ScriptedBackend(VlmBackend):
    """Replay a fixed list of pixels. The regression backend.

    Pixels are given as fractions of the frame so the script survives a resolution change:
    `[(0.5, 0.7), (0.4, 0.65), ...]`.
    """

    name = "scripted"

    def __init__(self, script, loop: bool = False):
        self._script = [tuple(p) for p in script]
        self._loop = loop
        self._i = 0

    def annotate(self, image, instruction, history=None):
        if self._i >= len(self._script):
            if not self._loop:
                return Annotation(u=image.shape[1] // 2, v=image.shape[0] // 2,
                                  confidence=1.0, rationale="script exhausted", terminal=True)
            self._i = 0
        fu, fv = self._script[self._i]
        self._i += 1
        h, w = image.shape[:2]
        return Annotation(u=int(fu * w), v=int(fv * h), confidence=1.0,
                          rationale=f"scripted step {self._i}/{len(self._script)}",
                          terminal=self._i >= len(self._script) and not self._loop)


class GeometricBackend(VlmBackend):
    """Aim at the most open direction in the lower half of the frame.

    No language understanding whatsoever — it reads the instruction only to decide whether
    to bias left or right on the words "left" and "right". That is exactly why it belongs
    here: it is the score a VLM has to beat to be worth its latency.

    Needs depth, which the node supplies out of band via `set_depth()` rather than through
    `annotate()`, so the interface stays image-and-instruction for every backend.
    """

    name = "geometric"

    def __init__(self):
        self._depth = None

    def set_depth(self, depth):
        self._depth = depth

    def annotate(self, image, instruction, history=None):
        import numpy as np

        h, w = image.shape[:2]
        if self._depth is None:
            return Annotation(u=w // 2, v=int(h * 0.62), confidence=0.1,
                              rationale="no depth; centre of the lower half")

        d = np.asarray(self._depth, dtype=np.float32)
        dh, dw = d.shape
        band = d[int(dh * 0.45):int(dh * 0.85), :]
        band = np.where(np.isfinite(band), band, 0.0)          # sky is not "open"
        cols = band.mean(axis=0)

        text = (instruction or "").lower()
        if "left" in text:
            cols[dw // 2:] *= 0.5
        elif "right" in text:
            cols[: dw // 2] *= 0.5

        best = int(cols.argmax())
        return Annotation(
            u=int(best * w / dw),
            v=int(h * 0.62),
            confidence=float(min(1.0, cols[best] / max(1.0, float(cols.max())))),
            rationale=f"most open column {best}/{dw}, mean depth {cols[best]:.1f} m",
        )
