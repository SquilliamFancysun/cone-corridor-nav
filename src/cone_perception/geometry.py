"""Camera geometry: a detection's bearing, and a cluster's bearing as the camera sees it.

Both halves of the association reduce to one horizontal angle, which is the
whole reason this layer is small. The LD06 is a single plane, so bearing is the
only dimension the two sensors genuinely share; range comes from the lidar,
which is far better at it than a 7.5 cm stereo baseline could be.

Sign convention, matching depth_view.py's Projector and everything else in this
repo: left is positive, counterclockwise from straight ahead (REP-103).

Pure Python -- no depthai, no cv2, no rclpy. The detector backends in
model/capture/ produce Detections; nothing here knows how.
"""

import math
from collections import namedtuple

from cone_perception import extrinsics

# One detected box. Coordinates are NORMALISED to [0, 1] so that ultralytics
# (which reports pixels) and DepthAI (which reports normalised) can both feed
# the same association without anyone downstream tracking a resolution.
#
# `clipped` is set by the backend when the box touches a frame edge. Such a box
# has a centre that has moved inward and a height that is cut short, so both its
# bearing and its range_bbox lie -- see range_from_bbox.
Detection = namedtuple(
    "Detection", "cls confidence u v w h clipped")

# fx, fy, cx, cy in pixels for a frame of width x height.
Intrinsics = namedtuple("Intrinsics", "fx fy cx cy width height")

# "Argument not supplied", which is NOT the same as a cone height of None.
# None is a real value here -- it is what extrinsics.CONE_HEIGHT_M holds before
# anyone has measured the cones -- so overloading it as the default would make
# an explicit `cone_height_m=None` silently fall back to the module constant and
# return a confident number for an unmeasured quantity.
_UNSET = object()


def intrinsics_from_hfov(width, height, hfov_deg=None):
    """Approximate intrinsics from a field of view, for when the device is absent.

    Used by the replay backend, where there is no OAK-D to ask. A real device
    should always be asked instead -- depth_view.intrinsics() does it in one
    call -- because this assumes a centred principal point and square pixels,
    and both are only approximately true.
    """
    hfov = math.radians(extrinsics.CAMERA_HFOV_DEG if hfov_deg is None else hfov_deg)
    fx = (width / 2.0) / math.tan(hfov / 2.0)
    return Intrinsics(fx=fx, fy=fx, cx=width / 2.0, cy=height / 2.0,
                      width=width, height=height)


def detection_bearing(det, intr):
    """Horizontal bearing of a box centre, in the CAMERA frame. Left positive.

    Only the horizontal centre is used. A cone's vertical position in frame says
    something about range, but far less reliably than the lidar already does.
    """
    u_px = det.u * intr.width
    return math.atan2(-(u_px - intr.cx), intr.fx)


def bearing_from_camera(x, y):
    """Bearing of a base_link point AS THE CAMERA SEES IT. Left positive.

    This is what makes the 2 in lever arm exact rather than ignored. The camera
    sits 0.05 m behind the lidar, which at 1.5 m is 1.9 deg of parallax -- small,
    but the association gate is only a few degrees wide, so spending one
    subtraction to remove it is free accuracy. It is only possible at all
    because the cluster's range is known; the detection's is not.
    """
    cam_x, cam_y, _ = extrinsics.CAMERA_IN_BASE
    bearing = math.atan2(y - cam_y, x - cam_x)
    return wrap_pi(bearing - math.radians(extrinsics.CAMERA_YAW_DEG))


def range_from_bbox(det, intr, cone_height_m=_UNSET):
    """Z = fy * cone_height / box_height_px, in metres. NaN when unusable.

    The weakest of the three range channels and not used for positioning --
    range_lidar is. It exists to DISAGREE: when a cluster has been matched to
    the wall behind a cone rather than the cone, the two ranges diverge, and
    nothing else in the pipeline notices.

    NaN when the box is clipped (a cone cut off by the frame edge has a
    meaningless height) or when the cone height was never measured.
    """
    height_m = (extrinsics.CONE_HEIGHT_M if cone_height_m is _UNSET
                else cone_height_m)
    if height_m is None or det.clipped:
        return float("nan")
    h_px = det.h * intr.height
    if h_px <= 0:
        return float("nan")
    return intr.fy * height_m / h_px


def wrap_pi(rad):
    """Fold radians into (-pi, pi]."""
    return (rad + math.pi) % (2 * math.pi) - math.pi


def in_camera_fov(bearing_rad, margin_deg=2.0):
    """True if a camera-frame bearing falls inside the usable image.

    Shrunk by a margin: a cone at the very edge is usually clipped, and a
    clipped box's centre has moved inward, so its bearing points somewhere the
    cone is not.
    """
    return abs(bearing_rad) <= extrinsics.camera_half_fov_rad(margin_deg)
