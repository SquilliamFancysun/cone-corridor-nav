"""Label the cones the camera structurally cannot see. Pure function, no rclpy.

## The blind spot this exists for

The two sensors do not overlap where you would assume. The LD06 stops resolving
a cone past about 3 m (`cone_perception/clustering.py` has the arithmetic: under
2 returns, and one return is noise). The camera sweeps 69 deg, so a boundary
cone 0.75 m off the corridor axis only enters frame past

    x = 0.75 / tan(32.5 deg) = 1.18 m

Everything the camera can label AND the lidar can range therefore lives in a
window roughly 1.18 m to 3.0 m ahead. Measured against `sim/drive_sim.py` on a
straight corridor -- mean centerline points per tick, and whether the car
actually completes the run:

    cone spacing   camera only   + this module   drives?
        1.50 m         2.4            4.6           no
        1.25 m         2.4            5.0           yes
        1.00 m         3.2            6.8           yes
        0.75 m         6.2           10.2           yes
        0.50 m         8.6           14.2           yes

Two things fall out of that table, and the second is the one worth acting on.

This module roughly quadruples the usable midpoints at every spacing -- the
cones in the near blind spot are the ones the lidar sees BEST, four or five
returns each and centimetre-accurate, and only their colour was missing.

But it does not rescue a sparse track. At 1.5 m the car still does not drive,
because this module cannot see further than the lidar; it fills the near field,
and a corridor whose next row is beyond the clustering limit has nothing there
to fill.

The cutoff sits between 1.25 m and 1.5 m, and it moved once already: an earlier
version of this table put it between 1.0 m and 1.25 m, on an ESTIMATED cone
cross-section of 6.5 cm. The measured value for this car's cones is 7.4 cm (see
`sim/cone_field.py`), which buys about half a metre of lidar range and shifts
the boundary one row. **Lay the corridor at 1.0 m or tighter** and the question
does not arise; 1.25 m is passing here by one row and is not where to sit.

The cones in that near blind spot are the ones the lidar sees BEST: four or five
returns each, centimetre-accurate. Only their colour is missing, and on a
corridor the geometry supplies it.

## Why this is not the naive rule the fork tests exist to defeat

`test_the_chain_does_not_cross_the_island` in `test_centerline.py` punishes
"every blue cone is my left boundary", and this looks like a cousin of it. The
difference is which cones it touches:

  - A cone the camera COULD see and did not label is left UNLABELED. That is a
    detector miss or a phantom cluster, and it is exactly the evidence
    `boundary_split.py` keeps its `unlabeled` bucket to surface. Painting a
    colour over it would hide the failure.
  - Only cones outside the camera's field are filled, and those are near, which
    on a corridor means they are in the corridor the car is already in.

At a fork the ambiguity is 2-3 m out -- in frame, where the camera does label
things and this module keeps its hands off. So this composes with junction
handling rather than fighting it. It is still wrong if a fork is ever close
enough to fall inside 1.18 m, which on a 1.5 m corridor cannot happen: the
branches have not separated yet at that range (see the island-nose section of
`data/layouts/track_v1.md`).

The honest summary: this is a corridor-following aid, correct while the car is
inside a corridor, and it must be reported separately from detected labels so
that nobody reads a geometric guess as a measurement.
"""

import math

from cone_perception import geometry
from cone_perception.cone_classes import CLASS_BLUE, CLASS_YELLOW, UNLABELED
from cone_perception.fusion import LabeledCone

from cone_nav.corridor.boundary_split import MIN_X_M

# Confidence stamped on a geometrically assigned cone. Zero because no detector
# was involved, and `LabeledCone.msg` documents the field as "detector
# confidence [0, 1]" -- a real detection never reports 0, so this doubles as the
# marker that tells the two apart downstream and in the logs.
GEOMETRIC_CONFIDENCE = 0.0

# Cones further out than this are not filled even when they are out of frame.
# Past the camera's near limit, being out of frame stops meaning "too close to
# see" and starts meaning "off to the side" -- a cone in the next corridor over,
# or the far wall of a junction, and neither belongs to the wall the car is
# following.
MAX_FILL_RANGE_M = 2.0

# A cone within this of the corridor axis is not assigned a side at all. The
# sign of a near-zero offset is noise, and a boundary cone that lands in the
# middle of the corridor would put a midpoint where there is no corridor.
MIN_OFFSET_M = 0.25

# Cones behind the car are NOT filled, and this is load-bearing rather than
# tidiness. `centerline._longest_forward_chain` builds its chain by walking
# midpoints in order of increasing distance FROM THE CAR, which is only the
# forward direction while every midpoint is ahead -- the assumption its
# docstring records, and one the camera used to enforce for free, since a cone
# behind the car was never in frame to be labelled.
#
# This module can label cones the camera never could, so it inherits the duty of
# keeping that assumption true. Filling a cone at x = -1.5 m manufactures a
# midpoint behind the car; a run of them is a longer "increasing distance" chain
# than the real corridor ahead, and the car follows it backwards, stops, and
# reports a centerline with four points and zero reach. Observed in
# `sim/drive_sim.py` before this gate existed.
#
# MIN_X_M is boundary_split's, not a second opinion: a cone level with the front
# axle is still a wall of the corridor the car is in.


def fill_unlabeled(cones, reference_heading_rad=0.0,
                   max_range_m=MAX_FILL_RANGE_M, min_offset_m=MIN_OFFSET_M,
                   min_x_m=MIN_X_M, fill_in_fov=False):
    """UNLABELED cones the camera could not have seen -> BLUE / YELLOW.

    `fill_in_fov` drops the field-of-view test, so EVERY unlabelled cluster in
    range gets a side. That is a different and much stronger claim -- it is only
    correct when the camera is genuinely not contributing, because in normal
    operation an in-frame unlabelled cluster is a detector miss and painting
    over it destroys the only evidence of one. Reserve it for an explicit
    "driving without the detector" mode, announce it loudly at startup, and do
    not let it default on.

    Note what it does NOT buy: it cannot see further than the lidar, so it does
    not rescue a corridor whose cones are spaced beyond the ~3 m clustering
    limit. It fills the near field better, nothing more.

    Returns `(cones, filled_count)`. Never mutates its input: the caller holds
    the fused list, and a status panel that reports "4 labelled by camera" after
    this ran would be wrong.

    `reference_heading_rad` is the direction the corridor runs, left positive.
    Zero -- straight ahead -- is right on a straight and slightly wrong mid-bend,
    where the corridor axis has rotated relative to the car. The caller can pass
    the heading of the previous frame's centerline to do better; the error only
    matters for a cone almost exactly on the axis, which `min_offset_m` already
    refuses to assign.
    """
    out = []
    filled = 0
    cos_h = math.cos(reference_heading_rad)
    sin_h = math.sin(reference_heading_rad)

    for cone in cones:
        if cone.cone_class != UNLABELED:
            out.append(cone)
            continue

        # The camera's own predicate, not a copy of it -- fusion.associate uses
        # this same pair to decide `out_of_fov`, and the two must not drift.
        bearing = geometry.bearing_from_camera(cone.x, cone.y)
        if not fill_in_fov and geometry.in_camera_fov(bearing):
            # In frame and still unlabelled: a detector miss or a phantom
            # cluster. Leave it, so it stays visible as one.
            out.append(cone)
            continue

        reach = max_range_m if not fill_in_fov else float("inf")
        if math.hypot(cone.x, cone.y) > reach or cone.x < min_x_m:
            out.append(cone)
            continue

        # Perpendicular offset from the corridor axis. Left positive.
        offset = -cone.x * sin_h + cone.y * cos_h
        if abs(offset) < min_offset_m:
            out.append(cone)
            continue

        out.append(LabeledCone(
            cone_class=CLASS_BLUE if offset > 0 else CLASS_YELLOW,
            confidence=GEOMETRIC_CONFIDENCE,
            x=cone.x, y=cone.y,
            range_stereo=cone.range_stereo,
            range_bbox=cone.range_bbox,
            range_lidar=cone.range_lidar,
            points=cone.points))
        filled += 1

    return out, filled


def is_geometric(cone):
    """True if this cone's colour came from geometry rather than the detector."""
    return (cone.confidence == GEOMETRIC_CONFIDENCE
            and cone.cone_class != UNLABELED)


def heading_of(line, default=0.0):
    """Direction the centerline runs, for feeding back as the reference axis.

    Taken over the whole line rather than its first segment: the near end is the
    noisiest part, being built from the cones with the fewest returns.
    """
    if not line.points or len(line.points) < 2:
        return default
    (x0, y0), (x1, y1) = line.points[0], line.points[-1]
    if math.hypot(x1 - x0, y1 - y0) < 1e-6:
        return default
    return math.atan2(y1 - y0, x1 - x0)
