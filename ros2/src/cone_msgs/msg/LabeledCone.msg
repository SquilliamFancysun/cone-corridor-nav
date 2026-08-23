# One classified cone in the body frame (x forward, y left, meters).

uint8 CLASS_BOUNDARY=0   # orange — corridor boundary
uint8 CLASS_GATE=1       # blue — junction gate
uint8 CLASS_GOAL=2       # green — goal

uint8 cone_class
float32 confidence             # detector confidence [0, 1]
geometry_msgs/Point position   # fused position estimate

# Individual range estimates for the three-way cross-check (NaN if unavailable)
float32 range_stereo   # OAK-D spatial detection
float32 range_bbox     # Z = f * h_real / h_pixels
float32 range_lidar    # LD06 cluster centroid
