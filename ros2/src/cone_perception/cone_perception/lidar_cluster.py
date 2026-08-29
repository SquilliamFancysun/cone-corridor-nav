"""LD06 scan -> cone-sized clusters (centroid + point count).

The algorithm is already written and tested, in `cone_perception.clustering`:
`cone_candidates(scan, calibration)` takes an `ld06.Scan` and returns candidates
in base_link, with the chassis arc masked and the size gate applied. This file
is only the rclpy wrapper around it, and that wrapper does not exist yet.

Until it does, `model/capture/fusion_view.py` runs the same function on the car
against the serial port directly. Anything learned there transfers unchanged --
that is the point of keeping the algorithm free of rclpy.

To build this node:
  - subscribe to whatever publishes the LD06 (no ROS driver exists yet either;
    `model/capture/ld06.py` is a complete decoder that a small node could wrap)
  - load calibration.json once at startup, exactly as fusion_view.py does, and
    refuse to run without it -- the bearing sign is not guessable
  - call clustering.cone_candidates and publish
"""


def main():
    raise SystemExit(
        f"{__name__}: not implemented yet -- the clustering algorithm lives in "
        "cone_perception.clustering and runs today via "
        "model/capture/fusion_view.py")
