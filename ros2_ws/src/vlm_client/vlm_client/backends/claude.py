"""The first real model in the loop: Claude, over the Anthropic API.

This is the backend the whole testbed was built to make measurable. It sees exactly what
`mock`, `scripted` and `geometric` see — one BGR frame and one instruction — and returns one
pixel. No pose, no metres, no map. That restriction is the reason a number from this backend
can be compared with a number from any other; see `backends/base.py`.

Three choices here are load-bearing, and each one is a latency or reliability decision rather
than a taste one:

* **Structured outputs, not prose parsing.** `output_config.format` constrains the reply to a
  JSON schema, so the pixel arrives as an integer instead of something to regex out of a
  sentence. A model that wants to explain itself can, in the `reasoning` field, without ever
  threatening the parse.
* **`effort: low` by default.** This is a control loop, not a chat. A scenario allows 40 steps
  in 300 s — 7.5 s per decision — and a high-effort call with adaptive thinking can exceed
  that on its own, turning every episode into a timeout that says nothing about navigation.
  Raise it with the `claude_effort` parameter when measuring quality rather than throughput.
* **Adaptive thinking left on.** Disabling it is legal at `effort: low`, and it is tempting
  for the latency, but on this model a thinking-disabled request can leak internal tags into
  the visible response. With a schema in play that is a parse failure rather than a cosmetic
  one, so the default keeps thinking on and buys latency with `effort` instead.

**Credentials come from the environment and are never a ROS parameter.** ROS parameters are
readable by anything on the graph, land in `ros2 param dump`, and would end up in a launch
log — an episode artifact this repo commits. Any credential source the SDK itself resolves
will do; see `_credential_source`.

**A Claude.ai subscription is not one of them.** Pro/Max covers claude.ai and Claude Code;
the Anthropic API is separately billed with its own credits. In particular
`~/.claude/.credentials.json` is Claude Code's own OAuth token — different audience, different
scopes — and is not a credential this SDK can use. Checked on 2026-08-02, when exactly that
file was the only Anthropic credential on the machine.
"""
from __future__ import annotations

import base64
import json
import os
import time

from .base import Annotation, VlmBackend

# The schema the reply is constrained to. Structured outputs reject numeric bounds
# (`minimum`/`maximum`), so u and v are validated and clamped on this side instead — see
# `_clamp`. Every field is required: an optional field is one the model can quietly omit.
_SCHEMA = {
    "type": "object",
    "properties": {
        "u": {"type": "integer", "description": "Horizontal pixel, 0 is the left edge."},
        "v": {"type": "integer", "description": "Vertical pixel, 0 is the top edge."},
        "confidence": {
            "type": "number",
            "description": "0.0 to 1.0. How sure you are this pixel is the right next move.",
        },
        "reasoning": {
            "type": "string",
            "description": "One sentence on what you see and why that pixel.",
        },
        "arrived": {
            "type": "boolean",
            "description": "True only if the goal described by the instruction is reached.",
        },
    },
    "required": ["u", "v", "confidence", "reasoning", "arrived"],
    "additionalProperties": False,
}

_SYSTEM = """\
You are the perception step of a drone navigation loop, flying over a photorealistic city.

Each turn you get one forward-facing camera frame and one instruction. You return a single
pixel in that frame. The pixel is a DIRECTION, not a destination: the aircraft converts it \
into a short move toward whatever that pixel is looking at, flies a few metres, and shows \
you a new frame. You will be asked many times on the way to one goal.

How to choose the pixel:

- Pick the point you want to move TOWARD next, not the goal's final resting place.
- If the goal is visible, aim at it.
- If it is not visible, aim at the most promising open route toward it — a street running \
the right way, a gap between buildings, open sky past a rooftop.
- Never aim at a surface you would hit. A wall filling the frame means turn, not forward. \
Aim beside the obstacle, at the free space you would pass through.
- Pixels near the horizontal centre mean straight ahead; left and right steer.
- Prefer a decisive move over a timid one. Aiming at the frame centre every turn is how an \
aircraft hovers until it runs out of steps.

ALTITUDE — the part that is easy to get wrong:

The camera points downward, so most of the frame is ground. Your pixel does not only steer \
the aircraft; it also sets its height, because the aircraft flies to the actual 3D point that \
pixel is looking at. **Aim at a patch of ground and you command a descent to that patch.** \
Doing that repeatedly walks the aircraft into the ground, and the flight is over.

So, unless the instruction explicitly asks you to descend or land:

- Aim high in the frame — at or near the horizon, or at the upper part of a distant \
structure. That keeps the aircraft level while still steering it.
- When an instruction names something on the ground ("the plaza", "the intersection", "the \
avenue"), it is telling you WHERE to go, not what height to fly at. Aim at the sky or \
horizon ABOVE that thing, not at the thing itself.
- Only aim at the lower part of the frame when you actually want to lose height.

Set `arrived` true only when the instruction's goal is genuinely reached, not when it is \
merely in sight. Ending early scores as a failure at whatever distance you stopped.\
"""


def _credential_source():
    """Which credential the SDK will resolve, or None if it would find nothing.

    The SDK's own order is `ANTHROPIC_API_KEY`, then `ANTHROPIC_AUTH_TOKEN`, then an OAuth
    profile written by `ant auth login`. Checking for all three rather than the key alone
    matters: an operator who authenticated with the CLI has working credentials and no env
    var, and demanding the key would lock them out for no reason.

    This is a pre-flight check, not authentication. It answers "is there anything to try?" so
    a missing credential fails at construction with a useful message, instead of on the first
    frame with the aircraft already in the air.
    """
    if os.environ.get("ANTHROPIC_API_KEY"):
        return "ANTHROPIC_API_KEY"
    if os.environ.get("ANTHROPIC_AUTH_TOKEN"):
        return "ANTHROPIC_AUTH_TOKEN"
    config = os.environ.get("ANTHROPIC_CONFIG_DIR") or os.path.join(
        os.path.expanduser("~"), ".config", "anthropic")
    if os.path.isdir(os.path.join(config, "credentials")):
        return f"oauth profile in {config}"
    return None


_NO_CREDENTIALS = """\
no Anthropic API credentials found. This backend needs API access, which is billed \
separately from a Claude.ai Pro/Max subscription — the subscription covers claude.ai and \
Claude Code, not the API. Note ~/.claude/.credentials.json is Claude Code's own token and \
cannot be used here.

Either:
  export ANTHROPIC_API_KEY=...      # from console.anthropic.com
  ant auth login                    # if your account has an API organisation

Both are read from the environment on purpose: a ROS parameter would be readable from the \
graph and written into the launch log.

If you would rather not pay per call, `local vLLM on GPU 1` is the other half of the V-01 \
fork in docs/todo.md and needs no API credentials at all.\
"""


class ClaudeBackend(VlmBackend):
    """Claude, over the Messages API. One frame in, one pixel out."""

    name = "claude"

    def __init__(self, model="claude-opus-5", effort="low", max_tokens=16000,
                 jpeg_quality=90, timeout_s=60.0, max_retries=2, fallbacks=True,
                 thinking="adaptive"):
        try:
            import anthropic
        except ImportError as exc:  # pragma: no cover - environment, not logic
            raise RuntimeError(
                "the anthropic SDK is not importable from this interpreter. It is a ROS-side "
                "(python 3.12) dependency and lives in vendor/py312 — run "
                "scripts/fetch_vendor.sh, and make sure bringup.sh put it on PYTHONPATH."
            ) from exc

        self.credential_source = _credential_source()
        if self.credential_source is None:
            raise RuntimeError(_NO_CREDENTIALS)

        self._anthropic = anthropic
        # A per-request timeout well under the episode's per-step budget. Without it the SDK
        # default is ten minutes, and one wedged call would eat an entire episode.
        self._client = anthropic.Anthropic(timeout=timeout_s, max_retries=max_retries)
        self.model = model
        self.effort = effort
        self.max_tokens = int(max_tokens)
        self.jpeg_quality = int(jpeg_quality)
        self.fallbacks = bool(fallbacks)
        self.thinking = thinking

        self.calls = 0
        self.refusals = 0
        self.errors = 0
        self.input_tokens = 0
        self.output_tokens = 0
        self.cache_read_tokens = 0
        self.latencies: list[float] = []

    # ------------------------------------------------------------------ helpers

    def _encode(self, image) -> str:
        """BGR frame -> base64 JPEG.

        JPEG rather than PNG on purpose: image *tokens* are priced by dimensions, so the
        format only changes upload time — and at 1440x1080 a quality-90 JPEG is a fraction of
        the bytes with no difference the model can see.
        """
        import cv2

        ok, buf = cv2.imencode(".jpg", image,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            raise RuntimeError("cv2.imencode failed on the frame")
        return base64.standard_b64encode(buf.tobytes()).decode("ascii")

    @staticmethod
    def _clamp(value, hi):
        return max(0, min(int(hi) - 1, int(value)))

    def _user_content(self, image, instruction, history):
        lines = [f"Instruction: {instruction}"]
        if history:
            lines.append("")
            lines.append("Your last few moves, oldest first — do not simply repeat them:")
            for a in history:
                lines.append(f"  ({a.u}, {a.v}) — {a.rationale}")
        lines.append("")
        lines.append("Where should the aircraft move next? Answer with one pixel.")

        return [
            {"type": "image",
             "source": {"type": "base64", "media_type": "image/jpeg",
                        "data": self._encode(image)}},
            {"type": "text", "text": "\n".join(lines)},
        ]

    def _request(self, image, instruction, history):
        kwargs = dict(
            model=self.model,
            max_tokens=self.max_tokens,
            # The system prompt is byte-identical every call, so it is worth a cache
            # breakpoint. It sits near the minimum cacheable length — if it falls under,
            # nothing caches and nothing breaks.
            system=[{"type": "text", "text": _SYSTEM,
                     "cache_control": {"type": "ephemeral"}}],
            output_config={"effort": self.effort,
                           "format": {"type": "json_schema", "schema": _SCHEMA}},
            messages=[{"role": "user",
                       "content": self._user_content(image, instruction, history)}],
        )
        if self.thinking:
            kwargs["thinking"] = {"type": self.thinking}
        if self.fallbacks:
            # A safety classifier can decline a request outright. Letting the API re-serve it
            # on the recommended model turns a dead episode into a completed one; "default"
            # routes by refusal category so there is no model list to maintain here.
            kwargs["betas"] = ["server-side-fallback-2026-07-01"]
            kwargs["fallbacks"] = "default"
            return self._client.beta.messages.create(**kwargs)
        return self._client.messages.create(**kwargs)

    # ------------------------------------------------------------------ contract

    def describe(self):
        p50 = p95 = None
        if self.latencies:
            ordered = sorted(self.latencies)
            p50 = round(ordered[len(ordered) // 2], 2)
            p95 = round(ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))], 2)
        return {
            "backend": self.name,
            "model": self.model,
            "effort": self.effort,
            "thinking": self.thinking or "disabled",
            "calls": self.calls,
            "refusals": self.refusals,
            "errors": self.errors,
            "latency_p50_s": p50,
            "latency_p95_s": p95,
            "input_tokens": self.input_tokens,
            "output_tokens": self.output_tokens,
            "cache_read_tokens": self.cache_read_tokens,
        }

    def annotate(self, image, instruction, history=None):
        h, w = image.shape[:2]
        t0 = time.perf_counter()
        try:
            response = self._request(image, instruction, history or [])
        except self._anthropic.APIError as exc:
            # Returning None means "no answer this frame"; the controller holds station
            # rather than guessing. A transient API failure must not move the aircraft.
            self.errors += 1
            raise RuntimeError(f"Anthropic API error: {exc}") from exc
        finally:
            self.latencies.append(time.perf_counter() - t0)

        self.calls += 1
        usage = getattr(response, "usage", None)
        if usage is not None:
            self.input_tokens += getattr(usage, "input_tokens", 0) or 0
            self.output_tokens += getattr(usage, "output_tokens", 0) or 0
            self.cache_read_tokens += getattr(usage, "cache_read_input_tokens", 0) or 0

        # Check the stop reason before touching content. On a refusal `content` is empty or
        # partial, and indexing it is how this would turn into a crash instead of a decline.
        if response.stop_reason == "refusal":
            self.refusals += 1
            return None

        text = next((b.text for b in response.content if b.type == "text"), None)
        if not text:
            return None
        data = json.loads(text)

        u = self._clamp(data["u"], w)
        v = self._clamp(data["v"], h)
        return Annotation(
            u=u, v=v,
            confidence=float(max(0.0, min(1.0, data.get("confidence", 0.5)))),
            rationale=str(data.get("reasoning", ""))[:400],
            terminal=bool(data.get("arrived", False)),
            metadata={"model": getattr(response, "model", self.model),
                      "clamped": (u, v) != (int(data["u"]), int(data["v"]))},
        )

    def close(self):
        self._client.close()
