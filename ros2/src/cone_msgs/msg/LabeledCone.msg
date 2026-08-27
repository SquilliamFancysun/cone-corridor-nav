# One classified cone in the body frame (x forward, y left, meters).

# Colors are the classes. Left/right are relative to the direction of travel,
# so a corridor driven in reverse still has blue on its own left.
uint8 CLASS_BLUE=0       # left corridor boundary
uint8 CLASS_YELLOW=1     # right corridor boundary
uint8 CLASS_RED=2        # junction gate — always placed in pairs
uint8 CLASS_ORANGE=3     # dead end — the wall across a stub
uint8 CLASS_MAGENTA=4    # goal

# Red and orange are the pair to watch: they are the two most similar colors on
# the track and they mean opposite things. A dead end read as a gate hands off
# to junction_exec at a wall; a gate read as a dead end misses the junction.

uint8 cone_class
float32 confidence             # detector confidence [0, 1]
geometry_msgs/Point position   # fused position estimate

# Individual range estimates for the three-way cross-check (NaN if unavailable)
float32 range_stereo   # OAK-D spatial detection
float32 range_bbox     # Z = f * h_real / h_pixels
float32 range_lidar    # LD06 cluster centroid
