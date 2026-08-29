"""Camera<->LiDAR extrinsic calibration values and helpers.
Measured constants live here and ONLY here.

Frame: `base_link` is AT THE LIDAR, x forward, y left, z up (REP-103).

That is not an arbitrary choice -- it is what every recorded session already
assumed. lidar_view.py's --mount-x/-y/-z default to 0, which makes base_link be
the sensor, and every scans.mcap on disk was written that way. Moving base_link
to the rear axle later is one constant here plus a re-record of nothing: the
scans do not change, only the transform published above them.
"""

import math

# Cone height, base to tip, in meters.
#
# Feeds the range_bbox channel of LabeledCone.msg: Z = f * CONE_HEIGHT_M / h_px.
# The whole track must use ONE cone size or this estimate is silently wrong for
# every cone that differs. Measure the cones you actually laid out and record
# the same number in data/layouts/track_v1.md.
#
# Measured 2026-08-28: 7 inches.
CONE_HEIGHT_M = 0.1778

# Where the camera sits in base_link, i.e. relative to the lidar, in meters.
#
# Measured 2026-08-28 with a tape: the lidar is 2 in (0.0508 m) FORWARD of the
# camera, so the camera is behind it and x is negative; the camera is 4.5 in
# (0.1143 m) ABOVE the lidar, so z is positive. No lateral offset was measured
# and none is apparent, so y is 0.
#
# Only x and y matter for association -- bearing is a horizontal angle -- but z
# is recorded because it is what determines where the lidar plane cuts the cone,
# and because the /tf the harness publishes needs all three.
CAMERA_IN_BASE = (-0.0508, 0.0, 0.1143)

# Camera yaw relative to base_link, degrees, left positive.
#
# ASSUMED ZERO, not measured: both sensors were mounted pointing forward by eye.
# A non-zero value shows up as every cone in the frame being labelled with a
# consistent left or right bias, which is exactly what step 4 of the plan's
# verification looks for. If that happens, solve it the way calibrate.py solves
# the lidar's sign -- from cones at known bearings -- and record it here.
CAMERA_YAW_DEG = 0.0

# OAK-D Lite RGB horizontal field of view, degrees. From the Luxonis spec.
#
# The LD06 sweeps ~218 deg of usable forward arc and the camera sees 69 deg of
# it, so most lidar clusters have no detection available even when the detector
# is working perfectly. fusion.py treats that as normal and emits UNLABELED
# rather than reaching for a box that was never in frame.
CAMERA_HFOV_DEG = 69.0


def camera_half_fov_rad(margin_deg=0.0):
    """Half the horizontal FOV, optionally shrunk by a margin.

    A cone at the very edge of the frame is usually clipped, and a clipped box
    has a centre that has moved inward -- so its bearing lies about which way
    the cone actually is. Shrinking the acceptance window is cheaper than
    detecting the clip.
    """
    return math.radians(max(0.0, CAMERA_HFOV_DEG / 2.0 - margin_deg))


def check_measured():
    """Complain about anything still unmeasured. Returns a list of strings.

    Called by the harness at startup so an unmeasured constant is announced
    once, loudly, rather than quietly producing plausible-looking numbers.
    """
    problems = []
    if CONE_HEIGHT_M is None:
        problems.append(
            "CONE_HEIGHT_M is unset: range_bbox cannot be computed, so the "
            "cross-check that catches a cluster matched to the wall behind a "
            "cone is unavailable.")
    if CAMERA_YAW_DEG == 0.0:
        problems.append(
            "CAMERA_YAW_DEG is 0 by assumption, not by measurement. A "
            "consistent left/right bias on every label means it is wrong.")
    return problems
