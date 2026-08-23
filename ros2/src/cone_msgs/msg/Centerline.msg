# Output of the corridor layer: the line to drive down, in the body frame.
std_msgs/Header header
geometry_msgs/Point[] points      # ordered near -> far
float32 corridor_half_width       # meters
bool single_boundary_fallback     # true if offset from one visible side only
