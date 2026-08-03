#!/usr/bin/env python3
"""The Claude backend's request shape and reply handling — with no network and no API key.

Every test here stubs the SDK. That is the point: the parts worth testing are the ones that
decide what gets sent and what happens to what comes back, and those are exactly the parts a
live call would hide behind a bill and a latency budget.

The backend lives in a ROS package but imports no ROS: `vlm_client/__init__.py` is empty and
`backends/claude.py` pulls in the SDK lazily inside `__init__`, so the 3.10 offline suite can
load it. That is not an accident — see the module docstring in `backends/claude.py`.

    ./.venv/bin/python -m pytest tests/test_claude_backend.py -q
"""
from __future__ import annotations

import json
import os
import sys
import types

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "ros2_ws", "src", "vlm_client"))


# ---------- the stub ----------

class _APIError(Exception):
    """Stands in for anthropic.APIError, which the backend catches by name."""


class _Block:
    def __init__(self, text):
        self.type = "text"
        self.text = text


class _Usage:
    def __init__(self, i=100, o=20, cache=0):
        self.input_tokens = i
        self.output_tokens = o
        self.cache_read_input_tokens = cache


class _Response:
    def __init__(self, payload=None, stop_reason="end_turn", usage=None, model="stub-model"):
        self.content = [] if payload is None else [_Block(json.dumps(payload))]
        self.stop_reason = stop_reason
        self.usage = usage if usage is not None else _Usage()
        self.model = model


class _Messages:
    def __init__(self, owner):
        self._owner = owner

    def create(self, **kwargs):
        self._owner.calls.append(kwargs)
        if self._owner.raises is not None:
            raise self._owner.raises
        return self._owner.responses.pop(0)


class _FakeClient:
    def __init__(self, **init_kwargs):
        self.init_kwargs = init_kwargs
        self.calls = []
        self.responses = []
        self.raises = None
        self.closed = False
        self.messages = _Messages(self)
        self.beta = types.SimpleNamespace(messages=_Messages(self))

    def close(self):
        self.closed = True


@pytest.fixture
def sdk(monkeypatch):
    """Install a fake `anthropic` module and hand back the client the backend will build."""
    built = []

    def _factory(**kwargs):
        client = _FakeClient(**kwargs)
        built.append(client)
        return client

    fake = types.ModuleType("anthropic")
    fake.Anthropic = _factory
    fake.APIError = _APIError
    fake.__version__ = "0.0.0-stub"
    monkeypatch.setitem(sys.modules, "anthropic", fake)
    monkeypatch.setenv("ANTHROPIC_API_KEY", "sk-ant-test-not-a-real-key")
    return built


@pytest.fixture
def backend(sdk):
    from vlm_client.backends.claude import ClaudeBackend
    b = ClaudeBackend()
    return b, sdk[0]


def frame(w=640, h=480):
    """A frame with actual structure — a flat image can hide an encoder bug."""
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[: h // 2, :, 2] = 200          # bright upper half
    img[h // 2 :, :, 0] = 120
    return img


def reply(**over):
    payload = {"u": 320, "v": 240, "confidence": 0.8,
               "reasoning": "open street ahead", "arrived": False}
    payload.update(over)
    return _Response(payload)


# ---------- construction ----------

def test_missing_credentials_fail_at_construction(sdk, monkeypatch, tmp_path):
    """Must fail on the ground, not on the first frame with the aircraft already up."""
    from vlm_client.backends.claude import ClaudeBackend
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    monkeypatch.delenv("ANTHROPIC_AUTH_TOKEN", raising=False)
    monkeypatch.setenv("ANTHROPIC_CONFIG_DIR", str(tmp_path))  # no credentials/ inside
    with pytest.raises(RuntimeError, match="no Anthropic API credentials"):
        ClaudeBackend()


def test_missing_credentials_message_names_the_subscription_trap(sdk, monkeypatch, tmp_path):
    """A Claude.ai subscription is the thing people reach for first, and it does not work."""
    from vlm_client.backends.claude import ClaudeBackend
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    monkeypatch.setenv("ANTHROPIC_CONFIG_DIR", str(tmp_path))
    with pytest.raises(RuntimeError) as exc:
        ClaudeBackend()
    text = str(exc.value)
    assert "subscription" in text
    assert "ant auth login" in text          # the OAuth alternative
    assert "vLLM" in text                    # the no-API-billing alternative


def test_an_oauth_profile_counts_as_credentials(sdk, monkeypatch, tmp_path):
    """`ant auth login` writes a profile and sets no env var — demanding the key would
    lock out an operator whose credentials are perfectly good."""
    from vlm_client.backends.claude import ClaudeBackend
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    monkeypatch.delenv("ANTHROPIC_AUTH_TOKEN", raising=False)
    (tmp_path / "credentials").mkdir()
    monkeypatch.setenv("ANTHROPIC_CONFIG_DIR", str(tmp_path))
    b = ClaudeBackend()
    assert "oauth profile" in b.credential_source


def test_auth_token_counts_as_credentials(sdk, monkeypatch, tmp_path):
    from vlm_client.backends.claude import ClaudeBackend
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    monkeypatch.setenv("ANTHROPIC_CONFIG_DIR", str(tmp_path))
    monkeypatch.setenv("ANTHROPIC_AUTH_TOKEN", "oat-...")
    assert ClaudeBackend().credential_source == "ANTHROPIC_AUTH_TOKEN"


def test_claude_code_credentials_are_not_accepted(sdk, monkeypatch, tmp_path):
    """~/.claude/.credentials.json is Claude Code's own OAuth token — different audience and
    scopes. It must not be mistaken for an API credential just because it exists."""
    from vlm_client.backends.claude import ClaudeBackend
    monkeypatch.delenv("ANTHROPIC_API_KEY", raising=False)
    monkeypatch.delenv("ANTHROPIC_AUTH_TOKEN", raising=False)
    home = tmp_path / "home"
    (home / ".claude").mkdir(parents=True)
    (home / ".claude" / ".credentials.json").write_text('{"claudeAiOauth": {}}')
    monkeypatch.setenv("HOME", str(home))
    monkeypatch.delenv("ANTHROPIC_CONFIG_DIR", raising=False)
    with pytest.raises(RuntimeError, match="no Anthropic API credentials"):
        ClaudeBackend()


def test_missing_sdk_names_the_interpreter_split(monkeypatch):
    """The failure a fresh install actually hits: the SDK installed for the wrong python."""
    from vlm_client.backends.claude import ClaudeBackend
    monkeypatch.setenv("ANTHROPIC_API_KEY", "sk-ant-test")
    monkeypatch.setitem(sys.modules, "anthropic", None)  # forces ImportError
    with pytest.raises(RuntimeError, match="vendor/py312"):
        ClaudeBackend()


def test_client_gets_a_timeout_well_under_the_step_budget(backend):
    """The SDK default is 10 minutes; one wedged call would consume a whole episode."""
    b, client = backend
    assert client.init_kwargs["timeout"] <= 120.0
    assert client.init_kwargs["max_retries"] >= 1


# ---------- request shape ----------

def test_request_carries_image_and_instruction(backend):
    b, client = backend
    client.responses.append(reply())
    b.annotate(frame(), "fly to the far tower")

    sent = client.calls[0]
    blocks = sent["messages"][0]["content"]
    image = next(x for x in blocks if x["type"] == "image")
    text = next(x for x in blocks if x["type"] == "text")
    assert image["source"]["media_type"] == "image/jpeg"
    assert len(image["source"]["data"]) > 100
    assert "fly to the far tower" in text["text"]


def test_reply_is_schema_constrained(backend):
    """Without this the pixel has to be regexed out of prose, which is where parsers die."""
    b, client = backend
    client.responses.append(reply())
    b.annotate(frame(), "go")

    fmt = client.calls[0]["output_config"]["format"]
    assert fmt["type"] == "json_schema"
    props = fmt["schema"]["properties"]
    assert {"u", "v", "confidence", "reasoning", "arrived"} <= set(props)
    assert props["u"]["type"] == "integer" and props["v"]["type"] == "integer"
    # Every field required: an optional field is one the model can quietly drop.
    assert set(fmt["schema"]["required"]) == set(props)
    assert fmt["schema"]["additionalProperties"] is False


def test_effort_defaults_low_for_the_control_loop(backend):
    b, client = backend
    client.responses.append(reply())
    b.annotate(frame(), "go")
    assert client.calls[0]["output_config"]["effort"] == "low"


def test_thinking_stays_on_by_default(backend):
    """Disabling it is legal at low effort and leaks internal tags into a schema-bound reply."""
    b, client = backend
    client.responses.append(reply())
    b.annotate(frame(), "go")
    assert client.calls[0]["thinking"] == {"type": "adaptive"}


def test_system_prompt_is_cached_and_stable(backend):
    b, client = backend
    client.responses += [reply(), reply()]
    b.annotate(frame(), "go")
    b.annotate(frame(), "go")

    first, second = client.calls[0]["system"], client.calls[1]["system"]
    assert first[0]["cache_control"] == {"type": "ephemeral"}
    # Byte-identical across calls, or the cache never reads.
    assert first[0]["text"] == second[0]["text"]


def test_history_is_offered_without_the_old_images(backend):
    """Re-sending frames would multiply image cost by the history length for no gain."""
    from vlm_client.backends.base import Annotation
    b, client = backend
    client.responses.append(reply())
    past = [Annotation(u=10, v=20, rationale="turned left"),
            Annotation(u=30, v=40, rationale="held course")]
    b.annotate(frame(), "go", past)

    blocks = client.calls[0]["messages"][0]["content"]
    assert sum(1 for x in blocks if x["type"] == "image") == 1
    text = next(x for x in blocks if x["type"] == "text")["text"]
    assert "turned left" in text and "(30, 40)" in text


def test_fallbacks_use_the_beta_endpoint_and_matching_header(backend):
    b, client = backend
    client.responses.append(reply())
    b.annotate(frame(), "go")
    sent = client.calls[0]
    assert sent["fallbacks"] == "default"
    assert sent["betas"] == ["server-side-fallback-2026-07-01"]


def test_fallbacks_can_be_turned_off(sdk):
    from vlm_client.backends.claude import ClaudeBackend
    b = ClaudeBackend(fallbacks=False)
    client = sdk[0]
    client.responses.append(reply())
    b.annotate(frame(), "go")
    assert "fallbacks" not in client.calls[0]
    assert "betas" not in client.calls[0]


# ---------- reply handling ----------

def test_happy_path(backend):
    b, client = backend
    client.responses.append(reply(u=100, v=200, confidence=0.9, arrived=False))
    ann = b.annotate(frame(), "go")
    assert (ann.u, ann.v) == (100, 200)
    assert ann.confidence == pytest.approx(0.9)
    assert ann.terminal is False
    assert "open street" in ann.rationale


def test_refusal_declines_instead_of_crashing(backend):
    """`content` is empty on a refusal — indexing it is how this becomes a crash."""
    b, client = backend
    client.responses.append(_Response(payload=None, stop_reason="refusal"))
    assert b.annotate(frame(), "go") is None
    assert b.refusals == 1


def test_out_of_frame_pixels_are_clamped(backend):
    """Schema validation cannot enforce ranges, so the bound has to live on this side."""
    b, client = backend
    client.responses.append(reply(u=9999, v=-40))
    ann = b.annotate(frame(w=640, h=480), "go")
    assert (ann.u, ann.v) == (639, 0)
    assert ann.metadata["clamped"] is True


def test_confidence_is_clamped_to_zero_one(backend):
    b, client = backend
    client.responses.append(reply(confidence=4.2))
    assert b.annotate(frame(), "go").confidence == pytest.approx(1.0)


def test_arrived_becomes_terminal(backend):
    b, client = backend
    client.responses.append(reply(arrived=True))
    assert b.annotate(frame(), "go").terminal is True


def test_api_error_raises_rather_than_inventing_a_pixel(backend):
    """A transient API failure must not move the aircraft."""
    b, client = backend
    client.raises = _APIError("503 overloaded")
    with pytest.raises(RuntimeError, match="Anthropic API error"):
        b.annotate(frame(), "go")
    assert b.errors == 1


def test_usage_and_latency_are_tallied(backend):
    b, client = backend
    client.responses += [_Response({"u": 1, "v": 1, "confidence": 0.5,
                                    "reasoning": "x", "arrived": False},
                                   usage=_Usage(i=900, o=40, cache=700))] * 1
    b.annotate(frame(), "go")
    d = b.describe()
    assert d["calls"] == 1
    assert d["input_tokens"] == 900 and d["output_tokens"] == 40
    assert d["cache_read_tokens"] == 700
    assert d["latency_p50_s"] is not None
    assert d["backend"] == "claude" and d["model"] == "claude-opus-5"


def test_errors_still_record_latency(backend):
    """Otherwise a slow failure looks free in the p95."""
    b, client = backend
    client.raises = _APIError("timeout")
    with pytest.raises(RuntimeError):
        b.annotate(frame(), "go")
    assert len(b.latencies) == 1


def test_close_releases_the_client(backend):
    b, client = backend
    b.close()
    assert client.closed is True


# ---------- the contract ----------

def test_backend_satisfies_the_interface(backend):
    from vlm_client.backends.base import VlmBackend
    b, _ = backend
    assert isinstance(b, VlmBackend)
    assert b.name == "claude"


def test_the_prompt_never_mentions_poses_or_metres():
    """The comparison is only fair while every backend sees image + instruction and no more."""
    from vlm_client.backends.claude import _SYSTEM
    lowered = _SYSTEM.lower()
    for leak in ("ned", "latitude", "waypoint coordinate", "metres from", "meters from"):
        assert leak not in lowered
