"""Synthetic cone fields: a track, a car pose, and what the two sensors would see.

This exists because the one behaviour that matters most cannot be tested any
other way. A fork is where a naive corridor extractor fuses two corridors into
nonsense, and building a two-junction track to find that out is a field trip.
Here it is a function call.

Deliberately not a simulator. There is no vehicle model, no time, and no
dynamics -- just "place the car here, and here is the Scan and the Detection
list it would get". That is exactly the input the pure layers take, and nothing
more is needed to test them.

Frame: layout coordinates are the surveyed frame of data/layouts/track_v1.md
(origin at the start line, x forward along Corridor A, y left). `observe()`
converts to the car's base_link.
"""

import math
import os

from ld06 import Scan

# Class names, matching the Roboflow classes and LabeledCone.msg.
BLUE, MAGENTA, ORANGE, RED, YELLOW = "blue", "magenta", "orange", "red", "yellow"

# From data/layouts/track_v1.md.
CORRIDOR_WIDTH_M = 1.5
STRAIGHT_SPACING_M = 1.5
FORK_SPACING_M = 0.75
BRANCH_DIVERGENCE_DEG = 25.0

# The cross-section a cone presents to the lidar plane. Not the 7 in height --
# that is what the camera measures. See the range arithmetic in
# cone_perception/clustering.py for why this number bounds how far the lidar
# can see a cone at all.
#
# 0.065 was an estimate. This is the measured value for THIS car's cones at THIS
# scan-plane height: calibration.json recorded 5.2 and 5.8 points per scan on a
# cone at 965 mm, and at the LD06's 0.8 deg step those back out to 7.0 cm and
# 7.8 cm of effective width. The mean is kept rather than the better of the two,
# and it is two observations at one range -- so it is a better number than the
# estimate, not a precise one.
#
# It makes the sim slightly LESS pessimistic about range (>=2 returns out to
# ~2.8 m rather than ~2.3 m). The cone-spacing conclusion in
# cone_nav/corridor/side_assign.py survives it; check that table again if this
# number ever moves much further.
CONE_WIDTH_M = 0.074


class Cone(object):
    __slots__ = ("color", "x", "y", "segment")

    def __init__(self, color, x, y, segment):
        self.color = color
        self.x = x
        self.y = y
        self.segment = segment

    def __repr__(self):
        return f"Cone({self.color}, {self.x:.2f}, {self.y:.2f}, {self.segment})"


def _along(start, heading_deg, distance):
    rad = math.radians(heading_deg)
    return (start[0] + distance * math.cos(rad),
            start[1] + distance * math.sin(rad))


def _offset(point, heading_deg, lateral):
    """Move sideways from a centerline point. Positive lateral is to the left."""
    rad = math.radians(heading_deg)
    return (point[0] - lateral * math.sin(rad),
            point[1] + lateral * math.cos(rad))


def corridor_segment(start, heading_deg, length, segment, spacing=STRAIGHT_SPACING_M,
                     half_width=CORRIDOR_WIDTH_M / 2.0, skip_first=False):
    """One length of corridor: a blue wall on the left, a yellow wall on the right.

    Left and right are in the direction of travel, which is what makes a fork
    resolve itself -- the island between two branches has a yellow face toward
    the left branch and a blue face toward the right one, with no extra class.
    """
    count = max(2, int(round(length / spacing)) + 1)
    cones = []
    for i in range(count):
        if skip_first and i == 0:
            continue
        centre = _along(start, heading_deg, length * i / (count - 1))
        cones.append(Cone(BLUE, *_offset(centre, heading_deg, half_width), segment))
        cones.append(Cone(YELLOW, *_offset(centre, heading_deg, -half_width), segment))
    return cones


def island_nose_distance(half_width=CORRIDOR_WIDTH_M / 2.0,
                         divergence_deg=BRANCH_DIVERGENCE_DEG):
    """How far past a junction the two branches' inner walls finally cross.

        nose = half_width / sin(divergence)

    Below this distance the branches have not separated: their inner walls are
    still on the wrong sides of each other, and placing cones along them from
    the junction outward builds two walls that intersect. A fork is not a point
    where the corridor splits -- it is a region, `nose` metres long, where the
    corridor is wide and the boundary is genuinely ambiguous. That is what the
    red gate pair is for.

    At the track_v1.md numbers (0.75 m, 25 deg) this is 1.78 m, which is LONGER
    than the 1.5 m dead-end stub that document specifies. See track_v1() for
    what that means.
    """
    return half_width / math.sin(math.radians(divergence_deg))


def branch(junction, heading_deg, length, segment, island_on_right,
           spacing=FORK_SPACING_M, half_width=CORRIDOR_WIDTH_M / 2.0,
           nose_clearance_m=0.35, skip_first_outer=True):
    """One branch off a junction, with its island-facing wall starting at the nose.

    The outer wall runs the whole length -- it is a continuation of the wall the
    car has been following. The inner wall cannot: it only exists once the two
    branches have separated, so it begins at the island nose and not before.

    `nose_clearance_m` starts the inner wall just past the crossing rather than
    exactly on it, so the two branches' first inner cones sit side by side with
    a real gap instead of on top of each other. Coincident cones are not a thing
    a triangulation can do anything sensible with.
    """
    inner_start = (half_width / math.tan(math.radians(BRANCH_DIVERGENCE_DEG))
                   + nose_clearance_m)
    count = max(2, int(round(length / spacing)) + 1)
    cones = []
    for i in range(count):
        travelled = length * i / (count - 1)
        centre = _along(junction, heading_deg, travelled)
        left = Cone(BLUE, *_offset(centre, heading_deg, half_width), segment)
        right = Cone(YELLOW, *_offset(centre, heading_deg, -half_width), segment)
        inner, outer = (right, left) if island_on_right else (left, right)
        if not (skip_first_outer and i == 0):
            cones.append(outer)
        if travelled >= inner_start - 1e-6:
            cones.append(inner)
    # The nose cone itself, which the sampling grid will not land on.
    nose_centre = _along(junction, heading_deg, inner_start)
    lateral = -half_width if island_on_right else half_width
    cones.append(Cone(YELLOW if island_on_right else BLUE,
                      *_offset(nose_centre, heading_deg, lateral), segment))
    return cones


def gate_pair(centre, heading_deg, segment, half_width=CORRIDOR_WIDTH_M / 2.0):
    """A red pair straddling the corridor. The midpoint between them is the mouth."""
    return [Cone(RED, *_offset(centre, heading_deg, half_width), segment),
            Cone(RED, *_offset(centre, heading_deg, -half_width), segment)]


def dead_end_wall(junction, heading_deg, length, segment,
                  half_width=CORRIDOR_WIDTH_M / 2.0):
    """The wall across the end of a stub, with one orange cone in the middle.

    The two corner positions are already the last cones of the side walls, so
    the wall needs exactly one more -- otherwise the 1.5 m gap reads as corridor
    and the car drives into it.
    """
    end = _along(junction, heading_deg, length)
    return [Cone(ORANGE, end[0], end[1], segment)]


def track_v1(dead_end_length_m=None):
    """The two-fork corridor of data/layouts/track_v1.md, as designed.

    Route is LEFT at J1, RIGHT at J2, so the net path is a gentle S and the
    heading goes 0 -> +25 -> 0. The branches not on the route are the dead ends.

    ONE DEPARTURE FROM THE DOCUMENT, and it is a real finding rather than a
    modelling convenience. track_v1.md specifies a 1.5 m dead-end stub at a
    +-25 deg divergence, but island_nose_distance() puts the point where the two
    branches separate at 1.78 m. A 1.5 m stub therefore never becomes a corridor
    of its own -- its walls are still tangled with the through-branch's when it
    ends. The default here is the shortest stub that has a metre of genuine
    walled corridor past the nose; pass `dead_end_length_m=1.5` to reproduce the
    document exactly and watch the boundary go ambiguous.
    """
    stub = (island_nose_distance() + 1.0 if dead_end_length_m is None
            else dead_end_length_m)
    cones = []
    origin = (0.0, 0.0)

    # Corridor A, then the J1 gate 1.0 m before the fork.
    cones += corridor_segment(origin, 0.0, 3.0, "corridor_a")
    cones += gate_pair(_along(origin, 0.0, 2.0), 0.0, "gate_j1")

    j1 = _along(origin, 0.0, 3.0)
    # Route goes left; the right branch is the dead end. The island between them
    # is on the right of the left branch and on the left of the right branch.
    cones += branch(j1, BRANCH_DIVERGENCE_DEG, 3.5, "corridor_b",
                    island_on_right=True)
    cones += branch(j1, -BRANCH_DIVERGENCE_DEG, stub, "dead_end_a",
                    island_on_right=False)
    cones += dead_end_wall(j1, -BRANCH_DIVERGENCE_DEG, stub, "dead_end_a")

    j2 = _along(j1, BRANCH_DIVERGENCE_DEG, 3.5)
    cones += gate_pair(_along(j1, BRANCH_DIVERGENCE_DEG, 2.5),
                       BRANCH_DIVERGENCE_DEG, "gate_j2")
    # Route goes right, back to heading 0; the left branch is the dead end.
    cones += branch(j2, 0.0, 3.0, "corridor_c", island_on_right=False)
    cones += branch(j2, 2 * BRANCH_DIVERGENCE_DEG, stub, "dead_end_b",
                    island_on_right=True)
    cones += dead_end_wall(j2, 2 * BRANCH_DIVERGENCE_DEG, stub, "dead_end_b")

    cones.append(Cone(MAGENTA, *_along(j2, 0.0, 3.0), "goal"))
    return cones


def straight_corridor(length=6.0, spacing=STRAIGHT_SPACING_M):
    """The simplest possible field: one straight corridor, no junctions."""
    return corridor_segment((0.0, 0.0), 0.0, length, "corridor_a", spacing=spacing)


def load_layout(csv_path=None):
    """Surveyed cones from track_v1.csv, or the designed track if it has no rows.

    The CSV holds MEASURED positions and is the D5 deliverable, so it wins as
    soon as it has content. Until the track is surveyed it is a header and
    comments, and falling back to the design is what lets this be useful now.
    """
    path = csv_path or os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "data", "layouts", "track_v1.csv")
    cones = []
    try:
        with open(path, encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if not line or line.startswith(("#", "id,")):
                    continue
                parts = line.split(",")
                if len(parts) < 5:
                    continue
                cones.append(Cone(parts[1].strip(), float(parts[2]),
                                  float(parts[3]), parts[4].strip()))
    except OSError:
        return track_v1()
    return cones or track_v1()


# --- what the car sees --------------------------------------------------

class Pose(object):
    """Where the car is in the layout frame."""

    __slots__ = ("x", "y", "heading_deg")

    def __init__(self, x=0.0, y=0.0, heading_deg=0.0):
        self.x = x
        self.y = y
        self.heading_deg = heading_deg

    def to_car(self, point):
        """Layout point -> base_link (x forward, y left)."""
        dx, dy = point[0] - self.x, point[1] - self.y
        rad = math.radians(-self.heading_deg)
        return (dx * math.cos(rad) - dy * math.sin(rad),
                dx * math.sin(rad) + dy * math.cos(rad))


def cones_in_car_frame(layout, pose, max_range_m=None):
    """Layout cones expressed in base_link, nearest first."""
    out = []
    for cone in layout:
        x, y = pose.to_car((cone.x, cone.y))
        if max_range_m is not None and math.hypot(x, y) > max_range_m:
            continue
        out.append(Cone(cone.color, x, y, cone.segment))
    out.sort(key=lambda c: math.hypot(c.x, c.y))
    return out


def synth_scan(cones, points_per_rev=450, background_mm=0, t=0.0,
               cone_width_m=CONE_WIDTH_M, range_noise_mm=0.0, seed=None):
    """Cones in base_link -> the Scan the LD06 would return.

    Generated with mirror=False and angle_offset=0, so the calibration that
    inverts it is `{"mirror": False, "angle_offset_deg": 0.0}` -- car bearing is
    the negated sensor bearing, per ld06.Scan.to_xy.

    Angular extent is computed per cone from its range, which is what makes this
    honest about the sensor's real limitation: past ~3 m a cone covers less than
    one angular step and simply does not appear, exactly as on the track.
    """
    import random
    rng = random.Random(seed)

    step = 360.0 / points_per_rev
    angles, ranges = [], []
    for i in range(points_per_rev):
        sensor_bearing = i * step
        car_bearing = -sensor_bearing  # mirror=False, offset=0
        rad = math.radians(car_bearing)
        best = None
        for cone in cones:
            cone_range = math.hypot(cone.x, cone.y)
            if cone_range < 1e-3:
                continue
            cone_bearing = math.atan2(cone.y, cone.x)
            half_extent = math.atan2(cone_width_m / 2.0, cone_range)
            delta = abs((rad - cone_bearing + math.pi) % (2 * math.pi) - math.pi)
            if delta <= half_extent and (best is None or cone_range < best):
                best = cone_range
        if best is None:
            if background_mm <= 0:
                continue
            distance = background_mm
        else:
            distance = best * 1000.0
        if range_noise_mm:
            distance += rng.gauss(0.0, range_noise_mm)
        angles.append(sensor_bearing)
        ranges.append(max(0.0, distance))

    return Scan(t=t, angles_deg=angles, ranges_mm=ranges,
                intensities=[120] * len(angles), speed_hz=10.0)


IDENTITY_CALIBRATION = {"mirror": False, "angle_offset_deg": 0.0,
                        "chassis_arcs_sensor": []}


def synth_detections(cones, intr, class_ids, hfov_deg=None, dropout=(),
                     confidence=0.9, cone_height_m=0.1778):
    """Cones in base_link -> the Detections the camera would report.

    Boxes are placed from the true bearing, so a test that passes here is
    testing the association and not the detector. `dropout` is the set of colors
    to omit, which is how "the camera missed this cone" is expressed.
    """
    from cone_perception import extrinsics

    half_fov = math.radians((hfov_deg or extrinsics.CAMERA_HFOV_DEG) / 2.0)
    cam_x, cam_y, _ = extrinsics.CAMERA_IN_BASE

    out = []
    for cone in cones:
        if cone.color in dropout or cone.color not in class_ids:
            continue
        dx, dy = cone.x - cam_x, cone.y - cam_y
        bearing = math.atan2(dy, dx)
        if abs(bearing) > half_fov:
            continue
        distance = math.hypot(dx, dy)
        if distance < 1e-3:
            continue
        u_px = intr.cx - math.tan(bearing) * intr.fx
        h_px = intr.fy * cone_height_m / distance
        u = u_px / intr.width
        if not 0.0 <= u <= 1.0:
            continue
        out.append(_detection(class_ids[cone.color], confidence, u,
                              0.5, h_px, intr))
    return out


def _detection(cls, confidence, u, v, h_px, intr):
    from cone_perception.geometry import Detection
    h = h_px / intr.height
    # A cone is roughly half as wide as it is tall in frame.
    w = h * 0.5
    clipped = (u - w / 2 <= 0.0 or u + w / 2 >= 1.0
               or v - h / 2 <= 0.0 or v + h / 2 >= 1.0)
    return Detection(cls=cls, confidence=confidence, u=u, v=v, w=w, h=h,
                     clipped=clipped)


def arc_pose(start, heading_deg, radius, sweep_deg, left=True):
    """Where a constant-radius arc ends, as (point, heading_deg).

    Exposed so arcs can be chained into an S without the caller re-deriving the
    rotation. Composing them by eye -- translating the next arc to the previous
    one's last cone -- produces a layout whose two halves do not join, which
    looks almost right in a plot and is not a corridor at all.
    """
    sign = 1.0 if left else -1.0
    centre = _offset(start, heading_deg, sign * radius)
    rot = math.radians(sign * sweep_deg)
    dx, dy = start[0] - centre[0], start[1] - centre[1]
    point = (centre[0] + dx * math.cos(rot) - dy * math.sin(rot),
             centre[1] + dx * math.sin(rot) + dy * math.cos(rot))
    return point, heading_deg + sign * sweep_deg


def curved_corridor(radius=6.0, sweep_deg=60.0, segment="curve",
                    spacing=STRAIGHT_SPACING_M,
                    half_width=CORRIDOR_WIDTH_M / 2.0, left=True,
                    lead_in_m=2.0, start=(0.0, 0.0), start_heading_deg=0.0,
                    skip_first=False):
    """A corridor that bends, for exercising the control layer.

    `straight_corridor` cannot fail a steering-sign error -- a car that steers
    backwards tracks a straight line just as well as one that steers correctly,
    right up until the corridor turns. This is the smallest field that tells the
    two apart, which is why it exists in the layout generator rather than in a
    test fixture.

    A straight `lead_in_m` comes first so the car starts on a settled line
    rather than mid-corner, matching how a real run begins.

    Not part of track_v1 and not surveyed. The real track's junctions are built
    from `branch()`; this is a sim-only shape.
    """
    cones = []
    if lead_in_m > 0:
        cones.extend(corridor_segment(start, start_heading_deg, lead_in_m,
                                      segment, spacing=spacing,
                                      half_width=half_width,
                                      skip_first=skip_first))
        skip_first = False
    arc_start = _along(start, start_heading_deg, lead_in_m)

    sign = 1.0 if left else -1.0
    centre = _offset(arc_start, start_heading_deg, sign * radius)
    # Arc length between cone rows, converted to the angle it subtends.
    step_deg = math.degrees(spacing / radius)
    count = max(2, int(round(sweep_deg / step_deg)) + 1)

    for i in range(0 if (lead_in_m <= 0 and not skip_first) else 1, count):
        theta = math.radians(sweep_deg * i / (count - 1))
        rot = sign * theta
        dx, dy = arc_start[0] - centre[0], arc_start[1] - centre[1]
        point = (centre[0] + dx * math.cos(rot) - dy * math.sin(rot),
                 centre[1] + dx * math.sin(rot) + dy * math.cos(rot))
        heading = start_heading_deg + math.degrees(rot)
        cones.append(Cone(BLUE, *_offset(point, heading, half_width), segment))
        cones.append(Cone(YELLOW, *_offset(point, heading, -half_width), segment))
    return cones


def s_bend_corridor(radius=4.0, sweep_deg=45.0, spacing=STRAIGHT_SPACING_M,
                    half_width=CORRIDOR_WIDTH_M / 2.0, lead_in_m=2.0):
    """Left then right, joined at a shared tangent.

    The shape that catches a controller which only works in one direction, and
    the one that most resembles the net path of track_v1 -- whose two junctions
    take the heading 0 -> +25 -> 0 for the same reason.
    """
    first = curved_corridor(radius=radius, sweep_deg=sweep_deg,
                            segment="s-bend-a", spacing=spacing,
                            half_width=half_width, left=True,
                            lead_in_m=lead_in_m)
    # The arc begins AFTER the lead-in, not at the origin. Computing the joint
    # from (0, 0) silently offsets the whole second half by lead_in_m.
    joint, heading = arc_pose(_along((0.0, 0.0), 0.0, lead_in_m), 0.0,
                              radius, sweep_deg, left=True)
    # The first arc already placed a row at the joint, so the second starts one
    # step along rather than doubling it up.
    second = curved_corridor(radius=radius, sweep_deg=sweep_deg,
                             segment="s-bend-b", spacing=spacing,
                             half_width=half_width, left=False,
                             lead_in_m=0.0, start=joint,
                             start_heading_deg=heading, skip_first=True)
    return first + second
