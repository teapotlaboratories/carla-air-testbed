# An Annotation2D after geometry: the same intent as a point in the world.
#
# `valid` is false when the pixel unprojected to sky — a legitimate model answer that must
# never be flown to. Consumers check `valid` before touching `position`.

std_msgs/Header header

Annotation2D source             # what produced this, kept for traceability
geometry_msgs/Point position    # NED metres, same frame as /fmu/out/vehicle_odometry
float32 depth                   # perspective depth at the annotated pixel, metres
float32 range                   # distance from the camera to `position`, metres
bool valid                      # false = nothing actionable could be derived at all
bool bearing_only               # true = direction is real, range is a guess (see below)
string reason                   # how this was derived, or why it was not

# bearing_only exists because a pixel with no depth is NOT useless. Sky, or anything past
# the depth cutoff, still tells you which way the model is pointing — and a system that
# discards that can never turn toward a distant goal, because every annotation toward the
# horizon gets thrown away. See-Point-Fly takes a bounded step along the bearing and
# re-observes; so do we. When bearing_only is true, `position` is a point at a fixed range
# along the ray rather than a surface, `depth` is 0, and the controller must not apply its
# standoff (there is no surface to stand off from).
