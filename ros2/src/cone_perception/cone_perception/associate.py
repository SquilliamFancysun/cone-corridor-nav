"""Project LiDAR clusters into the image, match to YOLO boxes,
publish cone_msgs/LabeledConeArray in the body frame.

The algorithm is already written and tested, in `cone_perception.fusion`:
`associate(candidates, detections, intrinsics, detection_age_s)` returns one
LabeledCone-shaped record per cluster, labelled where a box matched and
UNLABELED where none did. This file is only the rclpy wrapper, and it does not
exist yet.

Note that it does NOT project into the image, despite this docstring's original
plan. Matching is done on horizontal bearing in the camera frame, comparing each
detection's measured bearing against the bearing predicted from the cluster
(whose range is known, so the camera/lidar lever arm is handled exactly). A 2D
lidar contributes one plane, so bearing is the only dimension the two sensors
genuinely share, and going through the image plane would add a dependency on the
full 6-DOF extrinsic for no extra information.

To build this node:
  - message_filters ApproximateTime over the cluster and detection topics, or a
    latest-detections buffer as fusion_view.py uses
  - carry the detection's CAPTURE time through, not the time fusion ran; the
    staleness gate is meaningless otherwise
  - copy field for field into cone_msgs/LabeledCone; range_stereo stays NaN
"""


def main():
    raise SystemExit(
        f"{__name__}: not implemented yet -- the association algorithm lives in "
        "cone_perception.fusion and runs today via "
        "model/capture/fusion_view.py")
