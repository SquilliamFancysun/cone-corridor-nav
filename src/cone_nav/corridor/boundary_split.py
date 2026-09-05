"""Blue / yellow cones -> left / right boundary groups. Pure function, no rclpy.

Thin by design. The interesting decision -- which cones belong to THIS corridor
when a fork puts two of them in view -- is not made here; it falls out of the
triangulation in centerline.py. All this does is bucket by class and discard
what is behind or out of range, which is the part every consumer would
otherwise repeat.
"""

import math

from cone_perception.cone_classes import (
    CLASS_BLUE,
    CLASS_MAGENTA,
    CLASS_ORANGE,
    CLASS_RED,
    CLASS_YELLOW,
    UNLABELED,
)

# Cones this far behind the car are history. Not 0: a cone level with the front
# axle is still one wall of the corridor the car is in, and cutting at 0 makes
# the nearest midpoint jump forward as the car creeps past it.
MIN_X_M = -0.5

# Beyond this the lidar reports at most one return per cone (see the arithmetic
# in cone_perception/clustering.py), so a "cone" out here is usually noise.
MAX_RANGE_M = 5.0


class Boundaries(object):
    """Cones sorted into the roles the corridor layer acts on."""

    __slots__ = ("left", "right", "gates", "dead_ends", "goal", "unlabeled")

    def __init__(self, left, right, gates, dead_ends, goal, unlabeled):
        self.left = left
        self.right = right
        self.gates = gates
        self.dead_ends = dead_ends
        self.goal = goal
        self.unlabeled = unlabeled

    @property
    def boundary_cones(self):
        """Blue and yellow together -- what the corridor midpoints come from."""
        return self.left + self.right

    def counts(self):
        return {
            "left": len(self.left), "right": len(self.right),
            "gates": len(self.gates), "dead_ends": len(self.dead_ends),
            "goal": len(self.goal), "unlabeled": len(self.unlabeled),
        }

    def __repr__(self):
        return f"Boundaries({self.counts()})"


def split(cones, min_x_m=MIN_X_M, max_range_m=MAX_RANGE_M):
    """LabeledCones -> Boundaries, dropping what is behind or too far.

    UNLABELED cones are kept in their own bucket rather than discarded. They do
    not shape the centerline -- an unlabelled cluster could be either wall, and
    guessing is worse than ignoring -- but they are the evidence that the
    detector missed something, and the harness draws them.
    """
    buckets = {CLASS_BLUE: [], CLASS_YELLOW: [], CLASS_RED: [],
               CLASS_ORANGE: [], CLASS_MAGENTA: [], UNLABELED: []}

    for cone in cones:
        if cone.x < min_x_m:
            continue
        if math.hypot(cone.x, cone.y) > max_range_m:
            continue
        if cone.cone_class in buckets:
            buckets[cone.cone_class].append(cone)

    def by_range(group):
        return sorted(group, key=lambda c: math.hypot(c.x, c.y))

    return Boundaries(
        left=by_range(buckets[CLASS_BLUE]),
        right=by_range(buckets[CLASS_YELLOW]),
        gates=by_range(buckets[CLASS_RED]),
        dead_ends=by_range(buckets[CLASS_ORANGE]),
        goal=by_range(buckets[CLASS_MAGENTA]),
        unlabeled=by_range(buckets[UNLABELED]),
    )
