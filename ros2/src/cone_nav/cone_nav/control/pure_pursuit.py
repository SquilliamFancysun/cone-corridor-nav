"""Pure pursuit with lookahead scaled to speed. Pure function, no rclpy.

Takes the centerline `corridor/centerline.py` produces -- a polyline in
base_link, near to far -- and returns the front-wheel angle that drives along it.

## The frame problem, which is the whole reason this module has an `origin`

Pure pursuit's geometry is derived for a bicycle steered at the front and
pivoting about the REAR AXLE. Every textbook statement of it -- including the
curvature 2y/L^2 below -- measures the lookahead point from that axle.

`base_link` is not there. Per `cone_perception/extrinsics.py` it is at the
LIDAR, which `docs/hardware-baseline.md` places at the front edge of the
chassis: most of the wheelbase away. Feeding this function base_link points with
an implicit origin at (0, 0) claims the car pivots about its own nose, and the
error that follows is large -- on the fixtures in `test_pure_pursuit.py`, a
0.25 m offset moves the command by around 40%.

Which DIRECTION it moves depends on the path, so this is not a bias that a
lookahead tweak could absorb even in principle: moving the pivot back also moves
where the lookahead circle meets the line, and on a curve those two effects push
opposite ways. That is what makes it a geometry error rather than a gain error,
and why it has to be measured rather than tuned.

So `origin` is a required part of the calculation, passed by the caller as
`extrinsics.REAR_AXLE_IN_BASE`, and it defaults to (0, 0) only so the unit tests
can state a case in the axle frame directly without arithmetic in the fixture.

## Sign convention

Left positive, counterclockwise from straight ahead, matching REP-103 and every
other frame in this repo (`cone_perception/geometry.py`, `clustering.py`,
`cone_field.py`). A positive `delta_rad` steers left.
"""

import math

# The physical steering limit, used only to normalise `delta_rad` into the
# [-1, 1] the actuator wants.
#
# 0.35 rad (20 deg) is a starting guess for a 1/10 scale car, NOT a measurement,
# and it is the primary tuning knob: it is a pure gain on the steering command,
# so lowering it makes the car steer harder for the same geometry. Measure it
# properly by commanding full lock on a stand and reading the wheel angle, then
# record the number here.
MAX_STEER_RAD = 0.35

# A target nearer than this is not steerable -- the geometry divides by the
# distance to it, so a point on top of the axle produces an enormous curvature
# from what is probably just noise in the nearest midpoint.
MIN_TARGET_M = 0.15


class PursuitResult(object):
    """The steering command, plus enough to see why it came out that way.

    `target` is in base_link (not the axle frame) so the harness can draw it in
    the same picture as the cones and the centerline without transforming it
    back.
    """

    __slots__ = ("delta_rad", "target", "distance_m", "short_line")

    def __init__(self, delta_rad, target, distance_m, short_line):
        self.delta_rad = delta_rad
        self.target = target
        self.distance_m = distance_m
        self.short_line = short_line

    @property
    def normalised(self):
        """delta_rad as the [-1, 1] the actuator takes. Left positive."""
        return clamp(self.delta_rad / MAX_STEER_RAD, -1.0, 1.0)

    def __repr__(self):
        tail = " (short)" if self.short_line else ""
        return (f"PursuitResult({math.degrees(self.delta_rad):.1f} deg, "
                f"target {self.target[0]:.2f},{self.target[1]:.2f} at "
                f"{self.distance_m:.2f} m{tail})")


def clamp(value, low, high):
    return low if value < low else (high if value > high else value)


def _segment_circle_t(p1, p2, origin, radius):
    """Largest t in [0, 1] where the segment p1->p2 crosses the circle, or None.

    Largest rather than smallest because the walk goes near to far and wants the
    point where the path LEAVES the circle. The smaller root is where it entered
    -- behind the car on the first segment, and already passed on every later
    one -- so taking it would plant the target closer than the lookahead asked
    for, and on a curve would plant it on the wrong side of the arc.
    """
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    fx, fy = p1[0] - origin[0], p1[1] - origin[1]

    a = dx * dx + dy * dy
    if a < 1e-12:
        return None
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius

    disc = b * b - 4.0 * a * c
    if disc < 0.0:
        return None
    disc = math.sqrt(disc)

    for t in ((-b + disc) / (2.0 * a), (-b - disc) / (2.0 * a)):
        if 0.0 <= t <= 1.0:
            return t
    return None


def lookahead_point(points, lookahead_m, origin=(0.0, 0.0)):
    """First intersection of the circle of radius L about origin with the
    polyline, walking near to far.

    Returns `(point, short_line)`. When the line never reaches the lookahead
    distance the far end is returned with `short_line=True` rather than None:
    a centerline that stops two metres out is still worth steering toward, and
    it is `speed_ctrl` that decides a short line means slow down. Returns
    `(None, ...)` only when there is nothing usable at all.
    """
    if not points or len(points) < 2:
        return None, False

    for i in range(len(points) - 1):
        p1, p2 = points[i], points[i + 1]
        t = _segment_circle_t(p1, p2, origin, lookahead_m)
        if t is not None:
            return ((p1[0] + t * (p2[0] - p1[0]),
                     p1[1] + t * (p2[1] - p1[1])), False)

    # No crossing. Either the whole line is inside the circle (short) or the
    # whole line is outside it (the chain starts further out than the lookahead,
    # which happens when the nearest cones are lost). The far end serves the
    # first case; the near end serves the second, because aiming at the far end
    # of a line that begins off to one side would cut the corner into it.
    far = points[-1]
    if math.hypot(far[0] - origin[0], far[1] - origin[1]) <= lookahead_m:
        return far, True
    return points[0], False


def steering_angle(points, lookahead_m, wheelbase_m, origin=(0.0, 0.0)):
    """Centerline -> front wheel angle. None when there is nothing to follow.

    None is a real answer and the caller must handle it by commanding zero
    throttle -- see `speed_ctrl.duty`. It means the perception layer has not
    given us a line we can steer along, and coasting straight ahead on the last
    good command is exactly the wrong thing to do at the moment the car has
    stopped understanding where the corridor is.
    """
    target, short_line = lookahead_point(points, lookahead_m, origin)
    if target is None:
        return None

    # The rear axle carries the same heading as base_link, so this is a pure
    # translation -- no rotation, which is why `origin` is a point and not a
    # pose.
    x = target[0] - origin[0]
    y = target[1] - origin[1]
    distance = math.hypot(x, y)

    if distance < MIN_TARGET_M:
        return None
    # Behind the axle. Pure pursuit has no answer here that is not a reversing
    # manoeuvre, and quietly steering toward it would swing the car around.
    if x <= 0.0:
        return None

    # Curvature of the arc from the axle through the target: 2y / Ld^2. Then the
    # bicycle model's steer for that curvature.
    curvature = 2.0 * y / (distance * distance)
    return PursuitResult(
        delta_rad=math.atan(wheelbase_m * curvature),
        target=target,
        distance_m=distance,
        short_line=short_line,
    )
