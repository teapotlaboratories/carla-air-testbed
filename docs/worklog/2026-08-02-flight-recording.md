# 2026-08-02 — Recording every flight test

Backlog item [E-05](../todo.md). Two recorders: the onboard view the model is scored on, and
an HD exterior camera that follows the aircraft.

> **This worklog was written after the work, not during it**, which breaks the
> "update it AS YOU GO" rule in [`.ai/AGENTS.md`](../../.ai/AGENTS.md#worklogs--write-and-update-as-you-go).
> Noted rather than hidden — a worklog reconstructed at the end is exactly the one that
> quietly drops the dead ends, and two of the more useful findings below (§4, §5) were
> discovered and nearly forgotten mid-task.

---

## 1. Why

An episode left a JSON and nothing to look at. `141.9 m from goal` is a number, not an
explanation, and the first real VLM flight showed what that costs: the aircraft descended 40 m
onto the controller's altitude floor and circled there for twenty steps, and working out why
meant reading the offboard node's target log and reconstructing the camera geometry by hand.

The very first recorded episode made the *next* failure obvious in about five seconds: the
aircraft was over open sea and the model's own caption read *"Continuing forward over the
water toward the horizon past the rooftops maintains altitude while completing the crossing to
the far side."* It thought the ocean was the plaza. No log dig would have surfaced that as
fast as one frame did.

## 2. Two recorders, two different jobs

| | `evaluation/recorder` (onboard) | `carla_air/chase.py` (exterior) |
|---|---|---|
| Source | subscribes to `/camera/rgb/image_raw` | its own CARLA `sensor.camera.rgb` |
| Resolution | camera-native, 640x480 | 1920x1080 |
| Shows | what the model was scored on, plus its crosshair and rationale | the aircraft in the world |
| Output | `out/videos/<episode_id>.mp4` | `out/chase/<episode_id>.mp4` |

**The onboard one subscribes rather than captures.** A recorder with its own AirSim client
would be a fourth client on the transport that is already the depth bottleneck, and — worse —
it would record a *different* view from the one being scored. Subscribing costs the simulator
nothing and is evidence by construction.

**The chase camera is a CARLA sensor, and that is the whole reason it is affordable.** AirSim's
image path is RPC request/response; a 1080p grab on it would contend directly with the frames
the model needs. A CARLA sensor renders inside the same UE4 process and pushes frames out
asynchronously on its own tick. Measured before building anything: free-floating at 1920x1080,
**9.7 Hz**, and `set_transform` while streaming is free. In the real run: **1102 frames, 0
dropped.**

The sensor is spawned *unattached* — CARLA has no actor for an AirSim vehicle — so following
means writing a transform each tick from the aircraft's NED pose.

## 3. Things that would have gone wrong quietly

- **Frames never block CARLA's thread.** The sensor callback does one reshape and a
  non-blocking queue put; a writer thread owns the encoder. On backlog, frames are **dropped,
  not buffered** — an unbounded queue would eventually take the simulator down with it.
- **`raw_data` must be copied.** It is a view onto a buffer CARLA reuses for the next frame.
- **Yaw interpolates the short way round**, or the camera whips through 360 degrees every time
  the aircraft crosses north.
- **`chase_stop` runs in a `finally`.** A timeout or an exception still closes the file;
  otherwise the video of the run that went wrong is the one left unplayable.
- **The onboard recorder closes on `destroy_node` too**, because a sweep SIGKILLs the graph and
  an unclosed mp4 has no moov atom — the final episode of every sweep would not play.

## 4. The concurrency bug, and the latent one under it

The first flight test with the chase camera died at reset:

```
RuntimeError: reset: Existing exports of data: object cannot be re-sized
  ... tornado/iostream.py:395 -> self._write_buffer += data
```

That message names nothing about the cause. It is a `bytearray` being resized while a
`memoryview` export is live — two threads writing one msgpack-rpc socket at once.

**My error:** the follow thread read the pose via the shared telemetry client, on the
assumption that `fast_lock` protected it.

**The latent error it exposed:** the locks in `SimBridge` guard **dispatch classes, not
clients**. `state` is FAST (`fast_lock`); `reset` is neither FAST nor CONTROL (`slow_lock`) —
and *both* drive `self.airsim_client`. Two locks, one socket. That hazard has been present
since the three-client split; nothing hit it because every caller was serialised by
`run_episode.py`. A follow thread polling at 20 Hz is the first thing in this project that
genuinely runs concurrently with everything else.

Fix: the follower gets its own AirSim connection, created lazily on first `chase_start` —
which is what the existing telemetry/control/media split does, for this exact reason. No lock
is taken, because nothing else touches that socket.

Two regression tests in `tests/test_offline.py`. The first one initially **failed against my
own docstring**, which names both locks in order to explain why neither is used; it now matches
`with self.fast_lock` rather than the bare word. An assertion that matches explanatory prose is
one that will eventually lie.

## 5. 1080p is not 1920x1080 here

Asked for HD, the obvious answer is 1920x1080. On this camera it is wrong twice over:

```
                    fx      VFOV      horizon row      level-flight band
640x480  (4:3)     320.0   73.7deg    65 / 480         top 13.7%
1440x1080 (4:3)    720.0   73.7deg   147 / 1080        top 13.7%
1920x1080 (16:9)   960.0   58.7deg    17 / 1080        top  1.5%
```

16:9 narrows the vertical field of view, which would push the only rows that command level
flight from 13.7% of the frame down to 1.5% — actively worsening the descent bug diagnosed the
same day. And `FOV_Degrees` is *horizontal*, so a 16:9 RGB buffer against the 4:3 depth buffer
covers a different vertical field: `frames.scale_to()` would then map RGB pixels onto the wrong
depth pixels, silently, on every waypoint.

This only ever applied to the **drone's** camera, which is a measurement surface. The chase
camera has no such constraint and is a true 1920x1080.

### A detour that should not have happened

I briefly raised the **drone** camera to 1440x1080, having misread the request as being about
the onboard view. It has been reverted to 640x480. Changing it changes the experiment: image
tokens, aspect ratio, and where level flight sits in the frame.

The detour did produce one real result, which is why it is recorded rather than deleted: at
640x480 the model descends to the floor; at 1440x1080 it **holds altitude** (45-54 m), with the
same prompt. The level-flight band goes from 65 rows to 147, so "aim near the horizon" becomes
an instruction it can actually execute. That is evidence about the camera-pitch problem — it
just should not have been obtained by permanently altering the measurement.

Cost of the higher capture, measured while it was in place: **7.8 -> 7.15 Hz** (-8%) and model
latency p50 **3.89 -> 4.46 s**. Far cheaper than the "wildly superlinear" warning in
`camera.py` suggests, because that figure was for depth *and* segmentation at RGB resolution.
The docstring has been corrected.

## 6. No H.264 on this box

```
mp4v: OK        avc1/H264/X264: unavailable       system ffmpeg: absent
```

OpenCV's bundled FFmpeg has no libx264, so recording falls back to `mp4v` (MPEG-4 Part 2), at
roughly 3-4x the size for the same quality. About **80 MB per episode** across both views, so a
20-episode sweep is ~1.6 GB — fine on the 7 TB drive, but the reason the files look large.
Installing `ffmpeg` in the container would fix it; that is a container change and has not been
made unprompted.

For sharing, a downscaled copy is cheap to produce (1920x1080 -> ~1262x710 brought a 59.5 MB
file to 26.2 MB) and the full-resolution original stays on disk.

## 7. Also corrected today

`.ai/AGENTS.md` and `.ai/CLAUDE.md` said the simulator renders on **GPU 0** with GPU 1 held
free for VLM inference. That has been wrong since `TESTBED_GPU` landed and the operator asked
for the 5060 Ti — the project memory recorded the real decision (GPU 1, because GPU 0 is
regularly busy with UnrealEditor) while the rule files kept the old one. Every flight this
session ran on GPU 1, contradicting the written rule, and **that contradiction should have been
flagged the first time rather than silently resolved in memory's favour.** Corrected across
`CLAUDE.md`, `.ai/CLAUDE.md`, `.ai/AGENTS.md`, `.ai/GEMINI.md`, `.ai/copilot-instructions.md`
and `.ai/.cursorrules`, including the "GPU 0 is back to ~113 MiB" idle check, which is now
GPU 1 at ~33 MiB.
