# The ROS 2 side: the bridge node, and whatever else is launched against the interface.
#
#   docker build -f docker/ros.Dockerfile -t carla-air/ros:1 .
#
# ROS 2 Jazzy is python 3.12, which is the other half of the two-interpreter seam — the
# sidecar image is 3.10 because `libcarla` is an ABI-tagged cpython-310 extension and neither
# interpreter can load the other's C extensions. That split is the reason this project is two
# processes, and it is why there are two images rather than one.
#
# Like the other two, the environment is baked and the WORKSPACE IS MOUNTED. The image and
# the host both run Jazzy on Ubuntu 24.04, so a `colcon build` done on either side loads on
# the other; building in the image would mean a rebuild for every message-definition change.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    # What the graph imports beyond the ros-base set.
        ros-jazzy-cv-bridge \
        ros-jazzy-rosbag2-storage-mcap \
        python3-colcon-common-extensions \
        python3-numpy python3-yaml \
        python3-pip \
    # PyAV is the only path to H.264 for the recorders; opencv ships no libx264 (GPL vs
    # Apache). On the host this lives in vendor/py312 and is put on PYTHONPATH; in the image
    # it is simply installed. Both are fine — what is not fine is it being ABSENT, which
    # silently degrades every recording to mp4v at the wrong length (see T-02).
        python3-av \
    # msgpack is the wire format of the two-interpreter seam: sim_bridge/protocol.py is
    # imported by the ROS-side client and imports it at module scope, so without this the
    # bridge node dies at import with a bare ModuleNotFoundError that names nothing about
    # the seam. On the host it comes from vendor/py312.
        python3-msgpack \
        git \
 && rm -rf /var/lib/apt/lists/*

# ROS_DOMAIN_ID 42 is not a preference. On domain 0 this project's PX4-SHAPED topics merge
# with the sibling project's REAL PX4 topics and every measurement becomes the sum of two
# aircraft in two simulators. Set here so a container started by hand cannot forget it.
ENV ROS_DOMAIN_ID=42

WORKDIR /workspace
ENTRYPOINT ["/bin/bash", "-lc"]
