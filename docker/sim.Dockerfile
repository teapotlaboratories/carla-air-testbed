# The CARLA-Air simulator, with hardware Vulkan.
#
#   docker build -f docker/sim.Dockerfile -t carla-air/sim:v0.1.7 .
#   ./scripts/run_sim_docker.sh --config configs/testbed.yaml
#
# The 18 GB release is MOUNTED, never baked: it is a licensed binary drop that changes
# independently of this repository, and an image carrying it would be unbuildable by anyone
# who has not already downloaded it.
#
# **Everything about the GPU is in the RUN below and the two flags in run_sim_docker.sh.**
# P-01 sat blocked for five days on the belief that Vulkan could not survive nesting here; it
# can, and the recipe is small. See docs/worklog/2026-08-06-gpu-in-a-container.md for how the
# four wrong answers were eliminated.
FROM ubuntu:22.04

# 22.04 rather than 24.04 for the same reason the sidecar uses it — the release is built
# against an older glibc — and NOT because of Vulkan: both bases were measured working.
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    # --- the Vulkan loader itself. The DRIVER is injected by CDI at run time; do not install
    #     an NVIDIA userspace here, it would shadow the injected one and break on a host
    #     driver update.
        libvulkan1 \
    # --- GLVND. THIS is what P-01 was missing, and it is invisible from inside a working
    #     setup because every Unreal base image already ships it. `libGLX_nvidia.so.0` is a
    #     GLVND *vendor* library: without the dispatch layer present it loads but exposes no
    #     entry points, which surfaces as
    #         Could not get 'vkCreateInstance' via 'vk_icdGetInstanceProcAddr'
    #     and then a SILENT fall back to llvmpipe — the exact failure this project has a hard
    #     rule about.
        libglvnd0 libgl1 libegl1 \
    # --- X11 client libs the NVIDIA GLX vendor library links against, even headless.
        libxext6 libx11-6 \
    # --- what the UE4 shipping binary itself needs beyond libc/libstdc++.
        libsdl2-2.0-0 xdg-user-dirs \
        ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# The injected ICD is written for a FEDORA-FAMILY host and says
#   "library_path": "/usr/lib64/libGLX_nvidia.so.0"
# because that is where NVIDIA libraries live there. This container is Ubuntu, where they are
# under multiarch. The symlink makes the injected manifest resolve.
#
# It DANGLES AT BUILD TIME and resolves at run time — the target is itself created by the CDI
# hook when the container starts. That is intended. Do not "fix" it by pointing at the
# versioned libGLX_nvidia.so.610.43.03: that pins the driver version and breaks on a host
# update. Same reasoning as scripts/run_sim.sh's ICD repair inside the distrobox.
RUN mkdir -p /usr/lib64 \
 && ln -sf /lib/x86_64-linux-gnu/libGLX_nvidia.so.0 /usr/lib64/libGLX_nvidia.so.0

# Unreal refuses to run as root ("Refusing to run with the root privileges."), and that abort
# masquerades as a GPU failure. A non-root user with a real home is not optional.
RUN useradd -m -u 1000 -s /bin/bash sim
USER sim
WORKDIR /home/sim
ENV HOME=/home/sim

# AirSim reads this ONCE at startup and there is no way to change it on a running simulator,
# so the launcher writes it in before starting rather than baking a stale copy here.
RUN mkdir -p /home/sim/Documents/AirSim

ENTRYPOINT ["/bin/bash", "-lc"]
