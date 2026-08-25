"""Camera<->LiDAR extrinsic calibration values and helpers.
Measured constants live here and ONLY here."""

# Cone height, base to tip, in meters.
#
# Feeds the range_bbox channel of LabeledCone.msg: Z = f * CONE_HEIGHT_M / h_px.
# The whole track must use ONE cone size or this estimate is silently wrong for
# every cone that differs. Measure the cones you actually laid out and record
# the same number in data/layouts/track_v1.md.
CONE_HEIGHT_M = None  # MEASURE ME before trusting range_bbox
