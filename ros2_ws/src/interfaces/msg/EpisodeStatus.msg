# Live state of one navigation episode, published while it runs.

std_msgs/Header header

string episode_id
string instruction
uint32 seed
uint32 step                     # VLM iterations so far
float32 elapsed_s
float32 distance_to_goal        # metres, NED; NaN when the scenario declares no goal
float32 path_length             # metres flown so far
bool collided
string state                    # pending | running | success | failure | aborted
