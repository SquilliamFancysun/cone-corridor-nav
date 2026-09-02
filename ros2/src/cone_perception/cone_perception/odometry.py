"""Where the car is, relative to where it started. Pure function, no rclpy.

`ego_motion.rigid_step` measures one step. This sums them. That is the whole
module, and the sum is the part that needs the warning label.

## What this is, and the line it must not cross

`ego_motion`'s docstring is explicit that a Step is a measurement and the SUM of
Steps is a random walk, and that it "is not a pose estimate and must never be
treated as one". This module builds exactly the thing that docstring warns
about, so it is worth being precise about what changed: nothing did. The sum is
still a random walk. What is new is a caller for which that is the right tool.

The rule that keeps this honest is **nothing steers by a pose**. Every control
input in this repo is computed fresh in base_link from the current scan, and
that stays true. A pose here is read by two things only:

  - `cone_nav/topology/graph_builder.py`, for an edge length that no decision
    depends on in a tree-shaped maze;
  - `analysis/map_from_log.py`, off the car, after the run.

Drift that would be disqualifying in a controller is merely a plotting error in
both. If a third caller ever appears, the question to ask first is whether it
would still be correct with tens of centimetres of error after twenty metres,
because that is roughly what this delivers.

## The deadband, which is a real choice and not a copied constant

`ego_motion.DEADBAND_M` (8 mm) exists because `topo_state` clamps negative
travel: fed raw jitter, a distance FLOOR random-walks upward while the car
stands still. That argument does not transfer here. A pose integrator sums
SIGNED steps, so jitter largely cancels instead of accumulating, while a
deadband would systematically under-count genuinely slow motion -- exactly what
a hand-pushed dry run is made of.

So the default here is no deadband, which is the opposite of `dry_run_travel`'s
choice for a defensible reason rather than an oversight. It is a parameter
because the argument above is a prediction, and `analysis/map_from_log.py`
against a surveyed layout is what settles it with a number.
"""

import math


class Pose(object):
    """The car's position and heading in the frame it started in.

    x forward, y left, yaw counter-clockwise -- REP-103, the same convention as
    base_link, with the origin wherever the car was on the first step.
    """

    __slots__ = ("x", "y", "yaw_rad", "steps", "measured", "path_m")

    def __init__(self, x=0.0, y=0.0, yaw_rad=0.0):
        self.x = x
        self.y = y
        self.yaw_rad = yaw_rad
        # Ticks integrated, and how many of those carried a real measurement.
        # Their difference is the run's blind fraction and belongs in any
        # report that quotes a distance from this.
        self.steps = 0
        self.measured = 0
        self.path_m = 0.0

    # --- integrating ---------------------------------------------------

    def integrate(self, step, deadband_m=0.0):
        """Fold one `ego_motion.Step` in. `None` is no motion, not an error.

        A Step is the car's motion expressed in the EARLIER scan's frame, so it
        rotates by the heading held BEFORE the step -- applying this tick's yaw
        first would turn the car about the wrong point and bend every path into
        a spiral.
        """
        self.steps += 1
        if step is None:
            return self

        forward, lateral = step.forward_m, step.lateral_m
        if deadband_m > 0.0 and math.hypot(forward, lateral) <= deadband_m:
            forward = lateral = 0.0

        cos_y, sin_y = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        self.x += forward * cos_y - lateral * sin_y
        self.y += forward * sin_y + lateral * cos_y
        self.yaw_rad = _wrap(self.yaw_rad + step.yaw_rad)
        self.path_m += math.hypot(forward, lateral)
        self.measured += 1
        return self

    # --- reading -------------------------------------------------------

    @property
    def xy(self):
        return (self.x, self.y)

    @property
    def yaw_deg(self):
        return math.degrees(self.yaw_rad)

    def snapshot(self):
        """An immutable (x, y, yaw_rad) to keep past this tick.

        The Pose keeps mutating, so anything recording where a junction was --
        `graph_builder`, chiefly -- must take one of these rather than a
        reference to the live object.
        """
        return (self.x, self.y, self.yaw_rad)

    def to_world(self, x, y):
        """A point in the car's CURRENT base_link -> the start frame.

        This is what turns a tick's cone clusters into map landmarks.
        """
        cos_y, sin_y = math.cos(self.yaw_rad), math.sin(self.yaw_rad)
        return (self.x + x * cos_y - y * sin_y,
                self.y + x * sin_y + y * cos_y)

    def __repr__(self):
        return (f"Pose({self.x:+.2f}, {self.y:+.2f} m, "
                f"{self.yaw_deg:+.1f} deg, {self.measured}/{self.steps} steps)")


def distance_between(a, b):
    """Straight-line metres between two snapshots.

    The right measure for a `graph_builder` edge, and deliberately not
    `Pose.path_m`. Path length sums magnitudes, so per-step noise accumulates
    with a positive bias and a car standing still slowly gains distance; the
    separation of two poses does not, because the noise cancels in the sum
    before the magnitude is taken.
    """
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _wrap(angle_rad):
    """To (-pi, pi]. Kept bounded so a long run's heading does not drift into
    a float where small yaw increments stop resolving."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))
