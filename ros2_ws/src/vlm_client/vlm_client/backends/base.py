"""The one interface every VLM backend implements.

Keeping this narrow is the point of the testbed. A backend is handed an image and an
instruction and returns a pixel — it never sees a pose, a map, a velocity or metres.
Everything spatial happens downstream in `grounding`, which is what makes the models
swappable and the comparison between them fair.

That framing is See-Point-Fly's, not ours: *"consider action prediction for AVLN as a 2D
spatial grounding task… decompose vague language instructions into iterative annotation of
2D waypoints on the input image"*. If a backend needs more than this to work, it is not
answering the same question as the others and the comparison is meaningless.
"""
from __future__ import annotations

import abc
from dataclasses import dataclass, field


@dataclass
class Annotation:
    """A backend's answer. Pixel coordinates are in the frame it was given."""

    u: int
    v: int
    confidence: float = 0.0
    rationale: str = ""
    terminal: bool = False
    metadata: dict = field(default_factory=dict)


class VlmBackend(abc.ABC):
    """Stateless per call, by contract.

    A backend may cache a session internally, but the node makes no promise about call
    order and will reset episodes underneath it. Anything an episode needs remembered goes
    in the episode runner, not here.
    """

    name: str = "base"

    @abc.abstractmethod
    def annotate(self, image, instruction: str, history: list[Annotation] | None = None) -> Annotation | None:
        """Return where to go next, as a pixel in `image`.

        `image` is an HxWx3 uint8 BGR array. Return None to mean "no answer this step" —
        the controller holds station rather than guessing, which is the safe reading.
        """

    def describe(self) -> dict:
        return {"backend": self.name}

    def close(self) -> None:
        pass
