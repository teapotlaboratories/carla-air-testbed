# Terminal record of one episode. One of these per row in the results table.

std_msgs/Header header

string episode_id
string instruction
string scenario
uint32 seed
bool success                    # reached within the scenario's success radius
string failure_mode             # collision | timeout | max_steps | invalid_waypoint | none

float32 final_distance_m        # to goal
float32 path_length_m
float32 spl                     # success weighted by path length
uint32 steps
float32 duration_s
uint32 collisions

float32 vlm_latency_mean_s
float32 vlm_latency_max_s
string backend
