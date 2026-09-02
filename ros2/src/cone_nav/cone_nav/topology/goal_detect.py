"""Magenta in range -> the goal, and why not when there is none. Pure, no rclpy.

## The shape this reads

`data/layouts/track_v1.md` lays the goal as "One cone, centered, end of Corridor
C". One cone -- not a pair, not a line -- which makes the geometry here trivial
next to `gate_detect`'s and moves the entire burden somewhere else: onto deciding
that a magenta sighting is TRUSTWORTHY.

## Why the strictness is here and not left to the detector

Magenta was the model's worst class for two generations. v1 and v2 both scored
0.000 recall on the test set and both called 9 of 13 magenta instances RED
(`model/training/v1/report_test.md`). v3 fixes the recall. What no detector can
promise is that the confusion never runs the other way -- and a red gate cone
read as magenta is a car that stops dead in the middle of the course, which is a
worse failure than never seeing the goal at all.

So a goal is accepted only where the layout says a goal can be:

  - **In arm range.** Past `GOAL_ARM_RANGE_M` the LD06 returns one point or none
    per cone (the arithmetic is in `cone_perception/clustering.py`), so the range
    the stop triggers on would be a hope rather than a measurement.
  - **Near the corridor axis.** The goal is centered in a 1.5 m corridor. A
    magenta out where a boundary cone lives is a mislabelled boundary cone.
  - **Alone.** Two magentas in arm range is not a goal plus a decoy, it is a
    scene this module cannot read. Guessing which one to drive at is the single
    error with no recovery, so it declines instead.

One guard deliberately does NOT live here, because it is not geometry:
`drive_junction.py` arms the stop only once the route is spent, so a magenta
glimpsed at the first junction is never offered to this module at all.

## Which axis the offset is measured against, which is not a detail

The off-axis test needs a corridor direction, and at the goal there is no longer
a corridor to take one from -- that is what being at the goal MEANS. Measured on
the car 2026-09-01 (`goal-dry.jsonl`): the centerline emptied 1.1 m out, so
`side_assign.heading_of` fell back to its `default` and `axis_rad` froze at a
value taken off a two-point single-boundary fallback. It was wrong by 50-64
degrees, and 26 ticks of a clean approach were refused OFF_AXIS for a trophy the
camera was labelling at the time -- which it could only do with the trophy inside
its 34.5 degree half-frame, so the offset and the frame could not both be right.

The cost was not the refusals themselves. The offset shrinks with range, so the
goal was finally accepted at 0.49 m and the stop would likely still have fired;
what was lost was the margin. The run-in is meant to begin at `RUN_IN_M` = 1.0 m
and began at 0.45 m, because confirmation could not complete until the error had
shrunk out of the way.

So `trusted_axis` below decides what to hand this module, and a stale heading is
not it.

## Why `survey` exists and `detect` is written in terms of it

Copied from `gate_detect`, and for its reason. "No goal this tick" has four
causes with four unrelated fixes, and a bare None cannot tell them apart: a
trophy at 3.4 m and no trophy anywhere look identical in a boolean and send you
to opposite ends of the track. `survey` returns the reason beside the answer and
`detect` is a one-line wrapper over it, so the diagnosis in the trial log is
produced by the same code path as the decision and cannot drift from it.
"""

import math

from cone_nav.corridor.boundary_split import split
from cone_nav.topology.topo_state import corridor_reacquired

# Past this the lidar cannot range a cone honestly -- see the module docstring.
# The same value and the same reason as `gate_detect.GATE_ARM_RANGE_M`; they are
# separate constants because they answer to one sensor limit for two different
# decisions, and a layout change that moves one need not move the other.
GOAL_ARM_RANGE_M = 3.0

# How far off the corridor axis a magenta may sit and still be the goal.
#
# The corridor is 1.5 m wide, so its boundary cones stand at +-0.75 m. Half a
# metre rejects a mislabelled boundary cone with 0.25 m to spare while passing
# any trophy laid to the build rule -- which asks for the goal within 0.25 m of
# the axis, itself set by `side_assign.MIN_OFFSET_M` rather than by taste.
MAX_OFFSET_M = 0.50

# Why `detect` declined, in the words the trial log carries. Short enough to sit
# in a column, specific enough to name the fix: DISTANCE sends you to where the
# car is standing, OFF_AXIS to the tape measure, MULTIPLE to the detector or to
# whatever else magenta is standing in the scene, NO_MAGENTA to the model.
NO_MAGENTA = "no magenta"
DISTANCE = "magenta in view, beyond arm range"
OFF_AXIS = "magenta off the corridor axis"
MULTIPLE = "more than one magenta in range"


class GoalSurvey(object):
    """Every magenta in the scene, what `detect` made of them, and why.

    `magenta` is all of them out to `boundary_split`'s range cut; `in_arm` is the
    subset `detect` is allowed to accept. The two differing is the most useful
    thing this record says, exactly as it is in `RedSurvey`: it separates a goal
    the car cannot yet range from a goal that is not there, and those look
    identical in a count of accepted goals.
    """

    __slots__ = ("magenta", "in_arm", "goal", "offset_m", "reason")

    def __init__(self, magenta, in_arm, goal, offset_m, reason):
        self.magenta = magenta
        self.in_arm = in_arm
        self.goal = goal
        self.offset_m = offset_m
        self.reason = reason

    @property
    def ranges_m(self):
        """Range to every magenta, nearest first. `split` already sorted them."""
        return [math.hypot(c.x, c.y) for c in self.magenta]

    @property
    def range_m(self):
        """Range to the accepted goal, or None when there is not one."""
        if self.goal is None:
            return None
        return math.hypot(self.goal.x, self.goal.y)

    @property
    def bearing_deg(self):
        """Bearing to the nearest magenta in the CAR's frame, left positive.

        Logged beside `offset_m` because the two disagreeing is the signature of
        a bad axis, and on 2026-09-01 that had to be inferred from range and
        offset after the fact instead of read off a column.
        """
        cone = self.goal or (self.in_arm[0] if self.in_arm else
                             (self.magenta[0] if self.magenta else None))
        if cone is None:
            return None
        return math.degrees(math.atan2(cone.y, cone.x))

    def __repr__(self):
        where = ("%.2f m" % self.range_m) if self.goal is not None else "-"
        return (f"GoalSurvey({len(self.in_arm)}/{len(self.magenta)} in range, "
                f"{where}, {self.reason or 'detected'})")


def _offset(cone, axis_rad):
    """Perpendicular offset from the corridor axis, left positive.

    The same expression `side_assign.fill_unlabeled` uses to decide which wall a
    cone is on and `gate_detect` uses to order a triple, spelled out here for the
    same reason it is spelled out there: three call sites that must agree, and a
    shared private helper across modules would be a worse coupling than three
    copies of one line with this comment on each.
    """
    return -cone.x * math.sin(axis_rad) + cone.y * math.cos(axis_rad)


def trusted_axis(corridor_line, axis_rad):
    """The axis to measure a goal's offset against: the corridor's, or the car's.

    `axis_rad` is a one-tick feedback off the previous centerline, and
    `heading_of` holds its last value when that line dies rather than admitting
    it has none. Near the goal the line dies every time, so the fed-back heading
    is stale exactly where this module is asked to trust it -- see the module
    docstring for what that cost on the car.

    The predicate is `topo_state.corridor_reacquired`, reused rather than
    restated: it already means "this line is evidence of a real two-sided
    corridor", already refuses a single-boundary fallback, and is already tested.

    Falling back to the car's own frame is sound rather than merely safe. The
    test is a LATERAL offset, and a boundary cone sits about 0.75 m off the
    centre whatever the range; a car that has just driven down the corridor is
    aligned with it, so 0.50 m still separates a goal from a wall. It is the one
    frame that needs no estimate to be right.
    """
    return axis_rad if corridor_reacquired(corridor_line) else 0.0


def survey(cones, axis_rad=0.0, arm_range_m=GOAL_ARM_RANGE_M,
           max_offset_m=MAX_OFFSET_M):
    """LabeledCones -> GoalSurvey: the goal if there is one, and why if not.

    Never raises and never returns None. A tick with no magenta in it is a normal
    tick on a corridor, and nearly every tick of any run is that.
    """
    magenta = split(cones).goal
    in_arm = [c for c in magenta if math.hypot(c.x, c.y) <= arm_range_m]

    goal, offset, reason = None, None, ""
    if len(in_arm) > 1:
        # Declining is the point. See the module docstring: there is no safe way
        # to pick one, and picking wrong drives the car at the wrong object.
        reason = MULTIPLE
    elif len(in_arm) == 1:
        candidate = in_arm[0]
        offset = _offset(candidate, axis_rad)
        if abs(offset) <= max_offset_m:
            goal = candidate
        else:
            reason = OFF_AXIS
    elif magenta:
        reason = DISTANCE
    else:
        reason = NO_MAGENTA

    return GoalSurvey(magenta=magenta, in_arm=in_arm, goal=goal,
                      offset_m=offset, reason=reason)


def detect(cones, axis_rad=0.0, arm_range_m=GOAL_ARM_RANGE_M,
           max_offset_m=MAX_OFFSET_M):
    """LabeledCones -> the goal cone, or None if this is not one.

    The decision half of `survey`, kept as its own name because that is what a
    caller that only has to STOP wants. Written as a wrapper rather than as a
    second implementation so the reason logged for a rejection is always the
    reason for this tick's rejection.
    """
    return survey(cones, axis_rad=axis_rad, arm_range_m=arm_range_m,
                  max_offset_m=max_offset_m).goal
