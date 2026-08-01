# A VLM's answer: where to go, expressed as a pixel in the image it was shown.
#
# This is the whole See-Point-Fly interface. The model never sees metres, a map or a pose —
# it annotates the frame, and everything downstream is geometry. Keeping the message this
# small is deliberate: it is what lets the VLM be swapped without touching the flight path.

std_msgs/Header header          # stamp/frame of the IMAGE this annotates, not of "now"

uint32 u                        # pixel column, in the RGB frame's resolution
uint32 v                        # pixel row
float32 confidence              # 0..1, backend-reported; 0 when a backend cannot say

string instruction              # the natural-language instruction being followed
string rationale                # free text from the model, for the log — never parsed

bool terminal                   # model believes the instruction is complete
string backend                  # which VlmBackend produced this (mock, claude, vllm, ...)
float32 latency_s               # wall-clock the backend took, for the episode metrics
