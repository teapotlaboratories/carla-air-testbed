# The web console, as the stack's optional fourth container. R-08.
#
#   docker build -f docker/webui.Dockerfile -t carla-air/webui:1 .
#   ./scripts/stack_up.sh --config configs/testbed.yaml --console
#
# **Thin on purpose, and it installs nothing.** Since R-03 step 1 the console *is* an `rclpy`
# node — onboard video from `/camera/rgb/image_raw`, telemetry from `/fmu/out/vehicle_odometry`
# and `/sim/collision` — so everything it needs is already in the ROS image: `rclpy`,
# `cv_bridge` for `imgmsg_to_cv2`, and `msgpack` for the sidecar seam it still uses for the
# chase pane and the socket fallback. A `pip install` here would be a sign the console had
# grown a dependency the graph does not have, which is worth noticing rather than papering
# over.
#
# So why an image at all, rather than borrowing `carla-air/ros:1` as `webui.sh --in-stack`
# does? Because borrowing is what left the console unmanaged: no image meant no entrypoint,
# which meant the command line lived in whichever script happened to start it, and nobody
# owned its lifecycle. That is R-08's whole subject.
FROM carla-air/ros:1

# The console must be TOLD it is inside the stack; it cannot detect it. `/run/.containerenv`
# is present for this entire project on this machine, so a marker-file check cannot tell
# "the console inside the stack" from "the console in the ordinary development environment"
# and would refuse the stop button in the normal case. Baked here rather than passed by the
# caller so a container started by hand cannot get it wrong — being in this image IS being in
# the stack.
ENV TESTBED_IN_STACK=1

# The socket lives on the shared volume, not at the per-container /tmp default, which would be
# empty here. Overridable, because stack_up.sh is the one that decides the mount point.
ENV TESTBED_SOCKET=/run/carla-air/sim.sock

COPY docker/webui-entrypoint.sh /usr/local/bin/webui-entrypoint
RUN chmod +x /usr/local/bin/webui-entrypoint

ENTRYPOINT ["/usr/local/bin/webui-entrypoint"]
