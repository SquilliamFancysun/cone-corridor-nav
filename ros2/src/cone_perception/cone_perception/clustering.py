"""LD06 scan -> cone-sized clusters, in the car frame.

The clusterer itself came from calibrate.py, which needed it first and grew a
tested version. It lives here now because the nav stack needs the same code and
a second copy of "what counts as one object" is exactly the kind of duplication
that drifts silently: calibrate.py re-exports these names, and its tests still
exercise them through that path.

Pure Python -- no pyserial, no rclpy, no numpy. lidar_view.py and fusion_view.py
own the port and hand Scans here.

Angle convention matches ld06.Scan.to_xy and calibrate.Solution.car_bearing:

    car_bearing = sign * sensor_bearing + angle_offset,  sign = +1 if mirror
                                                                else -1

with car bearings measured counterclockwise from straight ahead, left positive.

## What the lidar can actually see

A traffic cone is small and the LD06's angular resolution is not. At ~450
points per revolution the spacing is 0.8 deg, so a cone presenting a 0.065 m
cross-section at the height the lidar plane cuts it subtends:

    2.0 m -> 1.9 deg -> ~2.3 returns
    3.0 m -> 1.2 deg -> ~1.6 returns
    4.0 m -> 0.9 deg -> ~1.2 returns

Past about 3 m a cone is one return or none, and one return is indistinguishable
from noise. That is a property of the sensor, not of this code, and it bounds
how far ahead the corridor layer can see -- the camera sees much further, but
only the lidar supplies range. `min_points` is 2 rather than calibrate.py's 3
for this reason, and the harness reports observed points-per-cluster so the
real number can replace this arithmetic.
"""

import math

# A cone at 1 m fills perhaps 6 deg of bearing; consecutive returns on it land
# under 1 deg apart. 3 deg tolerates a dropped packet mid-cone without welding
# the cone to whatever is behind it.
GAP_DEG = 3.0
GAP_MM = 150.0
MIN_CLUSTER_POINTS = 3

# Nothing wider than this is a cone -- it is a wall, a person, or two cones the
# gap rule joined. Deliberately generous: the camera is what confirms a cluster
# is a cone, so this gate only has to remove the obviously-not, and a tight one
# built on numbers nobody measured would reject real cones instead.
MAX_CONE_WIDTH_M = 0.30

# Applied only to clusters of 3+ points. A 2-point cluster spans one angular
# step, which at 1 m is 0.014 m -- under any sane floor -- so applying this to
# them would reject exactly the far-away cones the sensor is straining to see.
MIN_CONE_WIDTH_M = 0.02

# Cone returns thin out to nothing past ~3 m (see the module docstring). 5 m
# leaves room for the arithmetic above to be pessimistic without letting the
# whole far wall in as candidates.
MAX_CONE_RANGE_M = 5.0

# Chassis arcs are measured to the degree; cones just outside one still catch
# the odd body return. Widening the mask costs a sliver of real world in a
# direction that is mostly car anyway.
CHASSIS_MARGIN_DEG = 2.0


def wrap180(deg):
    """Fold into (-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


def wrap360(deg):
    """Fold into [0, 360). Not just `%`: a tiny negative float rounds up to
    exactly 360.0 under Python's modulo, which then reads as out of range."""
    value = deg % 360.0
    return 0.0 if value >= 360.0 else value


def circular_mean(degs):
    """Mean bearing, immune to the 359/1 wrap that breaks the arithmetic one."""
    x = sum(math.cos(math.radians(d)) for d in degs)
    y = sum(math.sin(math.radians(d)) for d in degs)
    if abs(x) < 1e-12 and abs(y) < 1e-12:
        # Antipodal inputs cancel: there is no meaningful mean, and returning 0
        # would look like a real answer.
        return None
    return wrap360(math.degrees(math.atan2(y, x)))


def circular_spread(degs, centre=None):
    """Largest deviation from the mean, in degrees. The stability number."""
    if not degs:
        return 0.0
    centre = circular_mean(degs) if centre is None else centre
    if centre is None:
        return 180.0
    return max(abs(wrap180(d - centre)) for d in degs)


def _median(values):
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return float(ordered[mid])
    return (ordered[mid - 1] + ordered[mid]) / 2.0


class Cluster(object):
    """A run of returns that are adjacent in bearing and agree in range."""

    __slots__ = ("bearing_deg", "range_mm", "points", "width_deg", "near_mm", "far_mm")

    def __init__(self, bearing_deg, range_mm, points, width_deg, near_mm, far_mm):
        self.bearing_deg = bearing_deg
        self.range_mm = range_mm
        self.points = points
        self.width_deg = width_deg
        self.near_mm = near_mm
        self.far_mm = far_mm

    def __repr__(self):
        return (f"Cluster({self.bearing_deg:.1f} deg, {self.range_mm:.0f} mm, "
                f"{self.points} pts, {self.width_deg:.1f} deg wide)")


def _make_cluster(group):
    angles = [a for a, _ in group]
    ranges = [r for _, r in group]
    return Cluster(
        bearing_deg=circular_mean(angles),
        # Median, not mean: one return that skimmed the cone's edge and landed
        # on the ground behind it should not drag the range.
        range_mm=_median(ranges),
        points=len(group),
        width_deg=wrap360(angles[-1] - angles[0]),
        near_mm=min(ranges),
        far_mm=max(ranges),
    )


def cluster_scan(angles_deg, ranges_mm, gap_deg=GAP_DEG, gap_mm=GAP_MM,
                 min_points=MIN_CLUSTER_POINTS, min_mm=1):
    """Split one revolution into objects, in sensor bearings.

    Everything is clustered, including the chassis returns -- filtering by range
    first would cut a cluster in half whenever an object straddles the window
    edge, and the chassis arc is worth seeing rather than hiding.
    """
    pts = sorted((wrap360(a), float(r)) for a, r in zip(angles_deg, ranges_mm)
                 if r >= min_mm)
    if not pts:
        return []

    groups = [[pts[0]]]
    for angle, dist in pts[1:]:
        prev_a, prev_r = groups[-1][-1]
        if angle - prev_a <= gap_deg and abs(dist - prev_r) <= gap_mm:
            groups[-1].append((angle, dist))
        else:
            groups.append([(angle, dist)])

    # An object sitting on the sensor's zero arrives as two groups, one at each
    # end of the sorted list. Join them before anything counts points.
    if len(groups) > 1:
        first_a, first_r = groups[0][0]
        last_a, last_r = groups[-1][-1]
        if first_a + 360.0 - last_a <= gap_deg and abs(first_r - last_r) <= gap_mm:
            groups[0] = groups.pop() + groups[0]

    return [_make_cluster(g) for g in groups if len(g) >= min_points]


# --- the car frame ------------------------------------------------------

def car_bearing(sensor_bearing_deg, mirror, angle_offset_deg):
    """Sensor bearing -> car bearing in (-180, 180], left positive."""
    sign = 1.0 if mirror else -1.0
    return wrap180(sign * sensor_bearing_deg + angle_offset_deg)


def _arc_span(arc):
    """Width of a chassis arc in degrees, handling the full-circle record."""
    start, end = arc["start_deg"], arc["end_deg"]
    span = (end - start) % 360.0
    # chassis_arcs() writes Arc(0.0, 360.0) when the sensor sees car in every
    # direction. That is a 360 deg span, but the modulo above calls it 0.
    if span == 0.0 and end != start:
        return 360.0
    return span


def _in_chassis(sensor_bearing_deg, arcs, margin_deg):
    for arc in arcs:
        span = _arc_span(arc) + 2 * margin_deg
        if span >= 360.0:
            return True
        offset = (sensor_bearing_deg - arc["start_deg"] + margin_deg) % 360.0
        if offset <= span:
            return True
    return False


class ConeCandidate(object):
    """One cluster that could be a cone, in base_link (x forward, y left, m).

    "Could be" is the whole point: nothing here knows what colour it is, or
    whether it is a cone at all. fusion.py decides that by asking the camera,
    and a candidate that gets no answer is still published, as UNLABELED.
    """

    __slots__ = ("x", "y", "range_m", "bearing_rad", "width_m", "points",
                 "sensor_bearing_deg")

    def __init__(self, x, y, range_m, bearing_rad, width_m, points,
                 sensor_bearing_deg):
        self.x = x
        self.y = y
        self.range_m = range_m
        self.bearing_rad = bearing_rad
        self.width_m = width_m
        self.points = points
        self.sensor_bearing_deg = sensor_bearing_deg

    @property
    def xy(self):
        return (self.x, self.y)

    def __repr__(self):
        return (f"ConeCandidate(({self.x:.2f}, {self.y:.2f}) m, "
                f"{math.degrees(self.bearing_rad):+.1f} deg, "
                f"{self.width_m * 100:.0f} cm, {self.points} pts)")


def cone_candidates(scan, calibration, max_range_m=MAX_CONE_RANGE_M,
                    min_points=2, max_width_m=MAX_CONE_WIDTH_M,
                    min_width_m=MIN_CONE_WIDTH_M,
                    chassis_margin_deg=CHASSIS_MARGIN_DEG):
    """LD06 Scan -> ConeCandidate list in base_link.

    `calibration` is the dict calibrate.load() returns: `mirror`,
    `angle_offset_deg`, and the measured `chassis_arcs_sensor`. The arcs are
    used rather than a hardcoded rear sector because the masking limits that are
    correct for one mount are wrong for the next one, and the arc was already
    measured -- see the chassis note in docs/hardware-baseline.md.

    Raises ValueError if the calibration has no bearing convention. A scan
    clustered against the wrong sign puts every cone on the wrong side of the
    car, which looks entirely plausible right up to the first junction.
    """
    if "mirror" not in calibration or "angle_offset_deg" not in calibration:
        raise ValueError(
            "calibration has no mirror/angle_offset_deg -- run "
            "`lidar_view.py --calibrate`. Clustering against an unverified "
            "sign mirrors the whole corridor.")

    mirror = bool(calibration["mirror"])
    offset = float(calibration["angle_offset_deg"])
    arcs = calibration.get("chassis_arcs_sensor") or []

    # Masked before clustering, not after: a chassis return adjacent in bearing
    # and range to a real one would otherwise be welded into its cluster and
    # drag the centroid toward the car.
    angles, ranges = [], []
    for angle, dist in zip(scan.angles_deg, scan.ranges_mm):
        if dist < 1:
            continue
        if _in_chassis(wrap360(angle), arcs, chassis_margin_deg):
            continue
        angles.append(angle)
        ranges.append(dist)

    out = []
    for cluster in cluster_scan(angles, ranges, min_points=min_points):
        range_m = cluster.range_mm / 1000.0
        if range_m > max_range_m:
            continue
        width_m = math.radians(cluster.width_deg) * range_m
        if width_m > max_width_m:
            continue
        # Only for clusters big enough for the floor to mean something; see
        # MIN_CONE_WIDTH_M.
        if cluster.points >= 3 and width_m < min_width_m:
            continue

        bearing_deg = car_bearing(cluster.bearing_deg, mirror, offset)
        bearing_rad = math.radians(bearing_deg)
        out.append(ConeCandidate(
            x=range_m * math.cos(bearing_rad),
            y=range_m * math.sin(bearing_rad),
            range_m=range_m,
            bearing_rad=bearing_rad,
            width_m=width_m,
            points=cluster.points,
            sensor_bearing_deg=cluster.bearing_deg,
        ))
    return out
