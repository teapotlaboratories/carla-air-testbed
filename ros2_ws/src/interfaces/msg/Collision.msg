# What the aircraft has hit, if anything.
#
# Replaces a bare `Bool` on /sim/collision. The flag alone is not enough to act on: the
# episode runner prints what was hit, and "COLLIDED" with no object name has sent people
# looking through video for a building the log already knew the name of.
#
# Latched semantics, like AirSim's own: `has_collided` stays true after an impact until the
# vehicle is reset, so a subscriber that joins late still learns the run is spoiled.
bool has_collided
string object_name              # empty when has_collided is false
int32 object_id
float64[3] position_ned         # where the impact happened
float64 penetration_m
