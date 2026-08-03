# 2026-08-02 — V-01: the first real model in the loop

Backlog item [V-01](../todo.md), unblocked by the operator choosing **Claude API** over a
local vLLM on the idle 5060 Ti. This is the backend the testbed was built to make measurable:
everything before it was either a heuristic or a diagnostic.

No simulator was started for this. The work is a backend plus its plumbing, and the parts
worth testing are testable against a stubbed SDK — see §5.

---

## 1. What it is

`ros2_ws/src/vlm_client/vlm_client/backends/claude.py`, registered as `claude` in `BACKENDS`.
It honours the same contract as the baselines — one BGR frame and one instruction in, one
pixel out, no pose and no metres — because that restriction is the only reason its score can
sit in a table beside `geometric`'s.

```bash
export ANTHROPIC_API_KEY=sk-ant-...
./scripts/bringup.sh --backend claude
```

## 2. The interpreter seam, again

The SDK is a **python 3.12** dependency. The `.venv` in this repo is **3.10** and owns the
carla/airsim clients; the ROS graph — and therefore every backend — runs under Jazzy's 3.12.
Installing `anthropic` into the venv would look right and fail at runtime with a
`ModuleNotFoundError` that reads like a missing package rather than a wrong interpreter.

So it goes to `vendor/py312`, installed by `scripts/fetch_vendor.sh` and appended to
`PYTHONPATH` by `scripts/bringup.sh`. Appended rather than prepended, so `vendor/` can never
shadow a ROS-supplied module. `vendor/` is already git-ignored.

This is the same seam as the `libcarla` split — the third distinct way it has bitten this
project — so it is now a row in the architecture trap table rather than only a comment.

## 3. Four decisions that are latency or reliability, not taste

**Structured outputs, not prose parsing.** `output_config.format` pins the reply to a JSON
schema, so the pixel arrives as an integer. A model that wants to explain itself does so in a
`reasoning` field where it cannot threaten the parse. The schema **cannot express numeric
bounds** — `minimum`/`maximum` are unsupported — so `u`/`v` are clamped on our side and the
annotation records whether clamping happened.

**`effort: low` by default.** This is a control loop, not a chat. `avoid_the_block` allows 40
steps in 300 s, which is 7.5 s per decision; a high-effort call with adaptive thinking can
exceed that on its own. Every episode would then time out, and the number would measure the
budget rather than the navigation. `claude_effort` raises it when the question is quality
instead of throughput.

**Adaptive thinking stays on.** Disabling it is legal at low effort and tempting for the
latency, but on this model a thinking-disabled reply can leak internal tags into the visible
response. With a schema in play that is a parse failure, not a cosmetic one. Latency gets
bought with `effort` instead.

**Fallbacks on.** A safety classifier can decline a request outright; letting the API re-serve
it on the recommended model turns a dead episode into a completed one. `claude_fallbacks`
turns it off.

## 4. Credentials are not a ROS parameter

They come from the environment and nowhere else. ROS parameters are readable by anything on
the graph, appear in `ros2 param dump`, and would be written into the launch log — and this
repo commits episode artifacts. A missing credential fails at **construction** rather than on
the first frame with the aircraft already airborne.

**Corrected after the first draft.** The initial version hard-required `ANTHROPIC_API_KEY`.
That was too strict: `ant auth login` writes an OAuth profile that the SDK resolves *and sets
no environment variable*, so an operator with perfectly good credentials would have been
locked out by a check that only looked for the key. `_credential_source()` now accepts
`ANTHROPIC_API_KEY`, `ANTHROPIC_AUTH_TOKEN`, or a profile directory, in the SDK's own
resolution order.

### A Claude.ai subscription is not an API credential

Asked whether a subscription would do, and checked rather than assumed. On this machine:

```
~/.config/anthropic          absent      -- no `ant auth login` profile
ANTHROPIC_API_KEY            unset
ANTHROPIC_AUTH_TOKEN         unset
~/.claude/.credentials.json  present     -- Claude Code's own subscription token
```

The last one is the trap. It is an Anthropic OAuth credential sitting on disk, and it is not
usable here: Pro/Max covers claude.ai and Claude Code, the API is billed separately with its
own credits, and Claude Code's token carries a different audience and scopes. Repurposing it
would be using a credential outside the scope it was issued for, so it is explicitly rejected
rather than quietly attempted — there is a test asserting exactly that.

The failure message now names the distinction, both credential routes, and the fact that
`local vLLM` is the option that needs no API billing at all. Someone hitting this at 2am
should not have to work out why the token they can see does not count.

The same reasoning put a 60 s client timeout in: the SDK default is ten minutes, and one
wedged call would silently consume an entire episode.

## 5. Verification without a key or a network

`tests/test_claude_backend.py` — 22 tests, no network, no key, stubbed SDK. The backend lives
in a ROS package but imports no ROS (`vlm_client/__init__.py` is empty, and the SDK import is
lazy inside `__init__`), so the 3.10 offline suite loads it fine.

What they pin down:

- **Request shape** — image block present and JPEG, instruction present, schema attached with
  every field required and `additionalProperties: false`, `effort: low`, thinking adaptive,
  system block carries a cache breakpoint and is byte-identical across calls, history is
  offered as text without re-sending old frames, fallbacks ride the beta endpoint with the
  matching header.
- **Reply handling** — happy path, a refusal declining instead of indexing empty `content`,
  out-of-frame pixels clamped, confidence clamped, `arrived` becoming `terminal`, an API
  error raising rather than inventing a pixel, usage and latency tallied (including on the
  error path, or a slow failure would look free in the p95).
- **The contract** — that the system prompt never mentions poses, metres or coordinates. The
  comparison with the baselines is only fair while the model sees an image and a sentence.

Suite total: **95 offline tests**, up from 73.

Also verified by hand: the backend constructs under the real 3.12 SDK (0.120.2) with
`vendor/py312` on the path, and `BACKENDS['claude']()` builds through the node's own registry
with ROS sourced.

## 6. The flight test, and what it found

Key supplied by the operator, validated with a 16-token call, then a single-call test of the
real request shape against the live API: structured outputs, the fallbacks beta, `effort: low`
and the image block were all accepted, and on a synthetic street scene the model answered
`(320, 210)` — the centre of the road corridor — reasoning *"the dark gap between the two
buildings is the open street corridor"*. 4.68 s, 465 in / 71 out, about $0.004.

So the integration works. Then it flew, and did not:

| run | result | note |
|---|---|---|
| 1 | FAILURE (max_steps), **141.9 m** from goal | 80 m scenario — it ended further away than it started |
| 2 | FAILURE (max_steps), **195.8 m** from goal | after an altitude-aware prompt fix; *worse* |

Both descended from 55 m to the controller's 15 m floor within six steps and then circled.

### It is not a model failure — it is the camera pitch

The camera is pitched **28.6° down**. Working out where level flight actually lives in the
frame:

```
fx = fy = 320.6,  vertical FOV 73.6 deg,  480 rows
horizon (level flight) at row v=65 of 480  ->  the top 13.6% of the image

  aim at v=  0   ->  +8.2 deg   climb    +2.9 m per 20 m step
  aim at v= 65   ->   0.0 deg   LEVEL     0.0 m
  aim at v=240   -> -28.6 deg   descend  -9.6 m      <- the frame centre
  aim at v=360   -> -49.1 deg   descend -15.1 m
```

**Aiming at the middle of the image commands a 9.6 m descent per step.** Six near-centre
annotations put a 55 m aircraft on the 15 m floor — which is precisely the observed
trace (−43, −37, −30, −24, −21, −18, floor) in both runs.

This is structural, and it applies to **any** backend that does not already know the goal's
altitude. There is no prompt that fixes it: the only pixels that hold altitude are the top
13.6% of the frame, and a model asked to point at where it wants to go will almost never
choose them.

### Why this was invisible until now

**The oracle is immune to it by construction.** It projects the episode goal — which sits at
flight altitude — into the image, so its pixel lands at v≈65 automatically and it flies level
without ever reasoning about height. Every scenario in this repo was validated with that
oracle.

So the oracle certified four scenarios against a defect it structurally cannot exhibit. That
is the same failure shape as [E-02](2026-08-02-avoid-the-block.md), where a straight-line
policy certified scenarios that a straight line trivially solved — a validator that cannot
see the flaw it is being used to rule out.

### A second, independent problem

Run 2's first waypoint was `[78.7, -139.6, -43.0]` from a start of `[113.9, -163.6, -55.3]`,
with the goal at `[187.6, -159.4, -55.0]` — **due north**. The model steered south-east.

It had no way to do better. `cross_the_plaza` says *"fly across the open plaza and stop above
the far side"*, and from one forward camera at an arbitrary reset heading there is nothing in
the frame that identifies which plaza or which direction. The oracle is handed the goal; the
`geometric` baseline ignores goals entirely and also scores 0/5. **These scenarios have never
been checked for solvability from vision and language alone**, because nothing that had to
solve them that way had ever run.

### What this does not mean

It does **not** mean Claude cannot navigate. Nothing here measured that. Two episodes were
lost to a geometry problem in the harness before navigation was ever exercised, and reporting
`0/2` as a model score would be exactly the kind of number this project's own rules say not to
trust. The fix is in the testbed, and the choice between the options is the operator's:

| option | cost |
|---|---|
| **Reduce the camera pitch** (say −10°, horizon at v≈183) | Every backend sees a different image; the E-01 baselines would need re-running. |
| **Decouple altitude in grounding** — pixel steers, altitude held unless the instruction implies a change | Changes the See-Point-Fly semantics; arguably the honest reading of "2D annotation → 3D displacement" for an aircraft. Also invalidates baselines. |
| **Tell the model where the horizon is** (row 65) | Cheap, and it is camera calibration rather than pose, so the contract survives. But it leaves only 13.6% of the frame usable for level flight — coarse steering, and it does nothing for the direction problem. |

None of them addresses the direction ambiguity, which is a scenario-design question rather
than a geometry one.

## 7. What is not verified

**No flight test — blocked on API credentials** (see §4). Everything above is request-shape
and reply-handling correctness; none of it says whether the model can fly.

Two ways forward, and the choice is the operator's:

| | |
|---|---|
| **Get API access** | A key from the Console, or `ant auth login` if the account has an API organisation. The backend is finished and waiting. |
| **V-01b — local vLLM on GPU 1** | No credentials, no per-call cost. The 5060 Ti is idle by design and 16 GB fits a quantised 7B-class VLM. A sibling backend, not a rewrite — same contract, and the schema-constrained reply and client-side clamping both carry over. |

When the key is available, the run is the same 5 seeds x 4 scenarios as E-01:

```bash
export ANTHROPIC_API_KEY=sk-ant-...
export CARLAAIR_RELEASE=... TESTBED_GPU=1
BACKENDS="claude" ./scripts/run_sweep.sh
```

The bar to clear is `geometric` on the three open scenarios. `avoid_the_block` is the
interesting one: it is the only scenario where the model has room to beat the **oracle** as
well, since a straight-line policy provably cannot solve it (see the E-02 worklog) while
`survey_buildings.py --route` shows a way around at 1.18x the direct distance.

Budget note, since it is a paid backend: at roughly 40 calls per episode, a 20-episode sweep
is ~800 calls. The backend accumulates call count, token spend and p50/p95 decision latency
and logs them on shutdown, so the cost of a sweep is visible in the run log rather than only
on the invoice.
