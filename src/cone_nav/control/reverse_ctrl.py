"""Steering for a car that is travelling backwards. Pure function, no rclpy.

`pure_pursuit` cannot do this job. It chases a lookahead point on the path
ahead, and a reversing car's path is behind it -- inside the 142 deg the
chassis blocks (`docs/hardware-baseline.md`), where there is no lookahead point
to be had at any price. So reverse gets a different law: regulate the two
errors the car CAN still see, and let the geometry do the rest.

## The convention, first, because it is what actually bites

Two frames are in play and they are opposites, so the law is written against
one of them explicitly:

    h   the CORRIDOR AXIS's direction, in the CAR's frame, left positive.
        This is what `side_assign.heading_of` and `Junction.axis_rad` return.
    y   the CAR's offset from the corridor centreline, left positive.

The car's own heading relative to the axis is `-h`, and mixing the two is not
a hypothetical: the first version of this module derived the law in terms of
the car's heading and then fed it `h`, which inverts one term and only one.
The result reverses beautifully for about a second and then spins the car
through 140 degrees -- measured, on `junction-left-blocked`, before the sign
was fixed. The heading trace of that run looks like a controller working right
up until it does not.

## The sign result, which is not the obvious one

"Steering is reversed when you back up" is the folk rule, and it is half wrong,
because getting it wrong by symmetry produces a controller that corrects
heading and fights position.

Take the rear-axle bicycle model with velocity `v`, the car's heading
`t = -h` measured from the axis, and steer `d`:

    y' = v sin t          t' = (v / L) tan d

Forward, `v > 0`. A car left of centre (`y > 0`) needs `y' < 0`, so `t < 0`, so
`t' < 0` from `t = 0`, so `d < 0` -- steer right. A car whose nose is left of
the axis (`t > 0`) needs `t' < 0`, so again `d < 0`.

Reverse, `v < 0`. A car left of centre still needs `y' < 0`, but now
`y' = -|v| sin t`, so it needs `t > 0` -- nose swung LEFT -- and `t' > 0` out of
`t' = -(|v|/L) tan d` means `d < 0`. Steer right, exactly as before. Meanwhile a
nose left of the axis (`t > 0`) needs `t' < 0`, which now means `d > 0`.

So the **cross-track term keeps its sign and the heading term flips**. Written
in `h`, which is what the caller actually has:

    forward   d = +k_h * h  -  k_y * y
    reverse   d = -k_h * h  -  k_y * y

Negating the whole law -- the folk rule applied literally -- gives
`d = -k_h*h + k_y*y`, which drives the car AWAY from the centreline. On a 1.5 m
corridor that reaches a cone in well under a metre, and it does it while the
heading trace looks healthy, which is the worst way to be wrong.

## Why this is the shakier half of the manoeuvre

Reversing is not the mirror of driving forward. Forward, cross-track error is
corrected through a heading change that the same steering input also damps;
backward, the two terms pull against each other, and the loop is only stable
while `k_t` dominates `k_y`. A gain pair that tracks beautifully at 0.2 m/s can
oscillate into a wall at 0.5. `MAX_REVERSE_MPS` is not a comfort limit.

Both gains are starting points to be settled in `sim/drive_sim.py`, the same
status `pure_pursuit.MAX_STEER_RAD` carries and for the same reason: nothing
here has been measured on a car yet.
"""

import math

from cone_nav.control.pure_pursuit import MAX_STEER_RAD, clamp

# Heading gain. Dominant by design -- see the module docstring on stability.
#
# Settled in `sim/drive_sim.py` on 2026-09-02, which is what the bottom of this
# docstring asks for, against both mirror layouts of `junction-*-blocked` with
# the rear 142 deg masked. Swept 1.2-3.0 against K_CROSS 0.3-1.0; this pair is
# the only one that backs out of BOTH and does it with no excursion over
# 0.17 m, at 0.036/0.053 m mean cross-track.
#
# Read the sweep before trusting the pair. Neighbouring cells fail -- (2.4,
# 0.6) loses the right-hand layout with a 1.77 m peak -- so this is a working
# point on a marginal system, not a broad optimum, and most of what separates
# the cells is which ticks of a speckled detection band the car happened to
# catch. Still nothing measured on a car: `docs/junction-bringup.md` stage 8d
# is where that happens, and expect to re-tune there.
K_HEADING = 2.4

# Cross-track gain, in steer-radians per metre of offset. Deliberately well
# under K_HEADING: a reversing car that chases position harder than it holds
# angle winds itself across the corridor. The sweep above bears that out --
# every cell with a 1.8 m-plus cross-track excursion in it is one where this
# gain was raised toward the heading one.
K_CROSS = 0.3

# The speed above which these gains have never been checked. A reverse loop
# stiffens with speed and this one is not gain-scheduled, so the ceiling is part
# of the controller rather than a preference.
MAX_REVERSE_MPS = 0.3


class ReverseCommand(object):
    """A steer, and the errors it was computed from -- both worth logging.

    A reverse that drifts is diagnosed from which error was growing, and the
    steer alone cannot say: the same command serves a big heading error and a
    big offset pulling opposite ways.
    """

    __slots__ = ("delta_rad", "heading_err_rad", "cross_track_m", "reference")

    def __init__(self, delta_rad, heading_err_rad, cross_track_m, reference):
        self.delta_rad = delta_rad
        self.heading_err_rad = heading_err_rad
        self.cross_track_m = cross_track_m
        self.reference = reference

    @property
    def normalised(self):
        return clamp(self.delta_rad / MAX_STEER_RAD, -1.0, 1.0)

    def __repr__(self):
        return (f"ReverseCommand({math.degrees(self.delta_rad):+.1f} deg, "
                f"heading {math.degrees(self.heading_err_rad):+.1f} deg, "
                f"offset {self.cross_track_m:+.2f} m, {self.reference})")


def steer(heading_err_rad, cross_track_m, reference="", k_heading=K_HEADING,
          k_cross=K_CROSS, max_steer_rad=MAX_STEER_RAD):
    """The two errors -> a steering angle for a car in reverse.

    `heading_err_rad` is `h`: the corridor AXIS's direction in the car's frame,
    left positive -- NOT the car's heading, which is its negative. See the
    module docstring; that confusion is the one failure this module has
    actually had. `cross_track_m` is `y`, the car's own offset from the
    centreline, left positive. Both are what the car SEES ahead of it, which is
    the whole reason this law can run while the path itself is invisible.
    """
    delta = -k_heading * heading_err_rad - k_cross * cross_track_m
    return ReverseCommand(clamp(delta, -max_steer_rad, max_steer_rad),
                          heading_err_rad, cross_track_m, reference)


def corridor_error(line, origin=(0.0, 0.0)):
    """A centerline ahead of the car -> (heading_err, cross_track), or None.

    Used while the car is backing out of a corridor it can still see down --
    the branch, before it crosses the junction line. `None` when the line is
    too short to have a direction, which the caller must treat as no reference
    rather than as zero error: zero error commands a straight reverse, and a
    straight reverse is exactly what a car with a wrong heading must not do.
    """
    if line is None or len(line.points) < 2:
        return None
    (x0, y0), (x1, y1) = line.points[0], line.points[-1]
    dx, dy = x1 - x0, y1 - y0
    if math.hypot(dx, dy) < 1e-6:
        return None
    heading = math.atan2(dy, dx)

    # The line's lateral offset at the car, taken perpendicular to the line
    # itself rather than as a y-difference, so a corridor at an angle is not
    # credited with an offset it does not have. The car sits at `origin`; the
    # line passes through (x0, y0) running along `heading`.
    ox, oy = origin[0], origin[1]
    line_offset = -(ox - x0) * math.sin(heading) + (oy - y0) * math.cos(heading)
    # `line_offset` is where the CAR sits relative to the line, which is
    # already the sign convention `steer` wants.
    return heading, line_offset
