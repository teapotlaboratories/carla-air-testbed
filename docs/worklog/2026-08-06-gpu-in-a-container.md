# 2026-08-06 — hardware Vulkan in a nested container, solved

P-01 was deferred on 2026-08-01 as "GPU passthrough does not survive nesting here". It does.
The blocker was a **missing GLVND package set**, and the recipe below gets
`PHYSICAL_DEVICE_TYPE_DISCRETE_GPU` on GPU 1 from a plain `ubuntu` base.

## The recipe

```bash
docker run --gpus '"device=nvidia.com/gpu=1"' <image>
```

The image needs, and this is the whole of it:

```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
      libvulkan1 libxext6 libx11-6 libglvnd0 libgl1 libegl1 \
 && mkdir -p /usr/lib64 \
 && ln -sf /lib/x86_64-linux-gnu/libGLX_nvidia.so.0 /usr/lib64/libGLX_nvidia.so.0
```

Verified on **both** `ubuntu:22.04` (the sidecar's cpython-3.10 base) and `ubuntu:24.04`
(ROS Jazzy), on GPU 1:

    deviceType = PHYSICAL_DEVICE_TYPE_DISCRETE_GPU
    deviceName = NVIDIA GeForce RTX 5060 Ti
    driverName = NVIDIA

## Why it took four wrong answers

The operator supplied a measured account of the working `drone-sim` renderer, which is what
made this tractable — there was a proven-good configuration on the same machine to diff
against. Everything below was tested one variable at a time.

| tried | result |
|---|---|
| `--gpus '"device=..."'` vs `--device nvidia.com/gpu=all` | identical. **The flag is not the difference**, contradicting my first guess |
| `NVIDIA_DRIVER_CAPABILITIES=graphics,...` | no change — the CDI spec is static, unlike the legacy runtime it does not re-decide injection from an env var |
| Ubuntu 22.04 vs 24.04 (loader 1.3.204 vs 1.3.275) | **identical** — both fail without GLVND, both work with it. The P-01 record's "22.04 → 24.04, still fails" was true and irrelevant |
| GPU 0 (3080) vs GPU 1 (5060 Ti) | identical. Not a card problem |
| bind-mounting `libnvidia-api.so.1` | no change |
| the `/usr/lib64` symlink alone | necessary but **not sufficient** |
| **+ `libglvnd0 libgl1 libegl1`** | **works** |

## The two real causes, in order

1. **The injected ICD points at a Fedora path.** CDI writes
   `/etc/vulkan/icd.d/nvidia_icd.x86_64.json` with
   `"library_path": "/usr/lib64/libGLX_nvidia.so.0"`, because the host is Bazzite
   (Fedora-family). An Ubuntu container keeps its libraries under multiarch, so the path does
   not resolve. This is the *same* bug `scripts/run_sim.sh` already works around inside the
   distrobox, and the operator's document identifies it as the load-bearing one for their
   renderer.

2. **`libGLX_nvidia.so.0` is a GLVND *vendor* library.** It is meant to be loaded through the
   GLVND dispatch layer, and without `libGLX.so.0` / `libEGL.so.1` present it loads but does
   not expose its entry points — which surfaces as
   `Could not get 'vkCreateInstance' via 'vk_icdGetInstanceProcAddr'` and a silent fall back
   to llvmpipe.

The operator's document does not mention (2) because their Unreal base image already carries
GLVND; it was invisible from inside a configuration that worked. Diffing the library lists
between a working container and a failing one is what exposed it — 58 libs against 48, and
the ten extra were all `libGLX*` / `libEGL*`.

## What this unblocks

`P-01` in full, not just the two non-Vulkan images. `sim-bridge` and the ROS graph already
built and ran; the simulator image was the only one blocked, and it is not blocked any more.

## Method note, since this is the fifth time it has mattered

Four hypotheses died before the fifth held, and every one of them died to *changing one
variable and measuring*, never to reasoning. The one that finally worked came from diffing a
known-good container against a known-bad one — the same move that found the annotation
oscillation and the reset altitude error earlier this week. **When there is a working example
on the same machine, diff against it before theorising.**

Also worth recording: my own first probe reported "no ICD in the container" because it looked
only in `/usr/share/vulkan/icd.d` and the file is in `/etc/vulkan/icd.d`. That wrong reading
survived two commits before the operator's document contradicted it.
