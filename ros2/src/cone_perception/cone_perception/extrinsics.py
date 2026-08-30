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

# Height of the lidar's scan plane above the ground, in meters.
#
# Derived 2026-08-30, not measured directly: the camera lens was measured at
# 9.5 in off the ground, and CAMERA_IN_BASE puts the camera 4.5 in above the
# lidar, so the lidar sits at 5.0 in = 0.127 m. (Derived, so it inherits the
# assumption that both were measured to comparable reference points.)
#
# Worth recording because it decides WHERE ON THE CONE the lidar plane cuts.
# At 0.127 m against a 0.1778 m cone that is 71% of the way up, well into the
# taper, which predicts a much narrower cross-section than the 6.5 cm
# sim/cone_field.py assumes -- and so fewer returns and a shorter usable range.
#
# The prediction is wrong, and the car's own calibration is what says so.
# calibration.json recorded 5.2 and 5.8 points per scan on a cone at 965 mm,
# which back out to an effective width of 7.0-7.8 cm: MORE than the assumed
# 6.5 cm, not less. Beam divergence and the clustering gap rule apparently more
# than make up for the taper. Trust the measurement over the geometry here.
LIDAR_HEIGHT_M = 0.127

# Camera yaw relative to base_link, degrees, left positive.
#
# ASSUMED ZERO, not measured: both sensors were mounted pointing forward by eye.
# A non-zero value shows up as every cone in the frame being labelled with a
# consistent left or right bias, which is exactly what step 4 of the plan's
# verification looks for. If that happens, solve it the way calibrate.py solves
# the lidar's sign -- from cones at known bearings -- and record it here.
CAMERA_YAW_DEG = 0.0

# Where the REAR AXLE sits in base_link, in meters.
#
# Pure pursuit pivots the car about its rear axle; base_link is at the lidar,
# which docs/hardware-baseline.md places at the front edge of the chassis. The
# gap between those two points is most of the wheelbase, and getting it wrong
# makes the car cut corners by an amount no lookahead tweak can absorb -- it is
# a geometry error, not a gain error. See the frame section of
# cone_nav/control/pure_pursuit.py.
#
# Measured 2026-08-30 with a tape, marks dropped to the floor and measured
# there: measuring straight through the air between the lidar and the axle
# returns the hypotenuse, which over this run and the height difference is
# several percent long.
#
#   lidar -> rear axle   14.25 in = 0.36195 m, so x is -0.36195 (axle is BEHIND)
#   front -> rear axle   13.00 in = 0.33020 m (WHEELBASE_M below)
#   implied lidar -> front axle = 1.25 in, i.e. the lidar overhangs the front
#   axle by just over an inch, which is what "front edge of the chassis" means
#   on this car.
#
# y is 0 because no lateral offset was measured and none is apparent -- the same
# basis, and the same caveat, as CAMERA_IN_BASE above.
#
# z is 0 and is the one value here that is NOT measured. It does not enter the
# steering calculation at all: pure pursuit is planar, and z only feeds the /tf
# the harness draws. Recorded as 0 rather than None so the control layer's
# measured-check passes on the two values that do matter; fill it in if the
# Foxglove transform ever needs to be right.
REAR_AXLE_IN_BASE = (-0.36195, 0.0, 0.0)

# Front axle to rear axle, in meters. Measured 2026-08-30: 13.00 in.
#
# The bicycle model's only vehicle parameter: delta = atan(WHEELBASE_M * curvature).
# It scales the steering command directly, so an error here is a proportional
# steering error at every angle.
WHEELBASE_M = 0.3302

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


def check_vehicle_measured():
    """Complain about anything the CONTROL layer needs and does not have.

    Kept apart from check_measured() rather than folded into it, because these
    two lists have different audiences and different severities. The perception
    tools -- fusion_view, detect_view, lidar_view -- print check_measured() at
    startup and do not steer, so vehicle geometry is not their business and
    warning them about it is exactly the crying-wolf that makes startup warnings
    stop being read.

    A tool that actuates should treat a non-empty list here as fatal, not as a
    warning. Every value in it is one that produces confident, wrong steering.
    """
    problems = []
    if REAR_AXLE_IN_BASE is None:
        problems.append(
            "REAR_AXLE_IN_BASE is unmeasured: pure pursuit would pivot the car "
            "about the lidar at its nose, and cut every corner to the inside.")
    if WHEELBASE_M is None:
        problems.append(
            "WHEELBASE_M is unmeasured: the bicycle model has no vehicle to "
            "model, so every steering angle is scaled by an unknown factor.")
    return problems
