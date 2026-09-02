"""Red triple in range -> the two gates it defines. Pure function, no rclpy.

## The shape this reads

Junction v2 (`data/layouts/junction_v2.md`) marks a fork with THREE red cones
in a line across the mouth. The left and centre cones bracket the left branch's
gate; the centre and right cones bracket the right branch's. The centre cone is
shared, and it is also the island nose -- the physical divider that
`junction_exec.keep_branch` cuts on.

    RED L  --------+           left gate  = midpoint(L, C)
                   | 1.5 m
    RED C  --------+           right gate = midpoint(C, R)
                   | 1.5 m
    RED R  --------+           span L..R  = 3.0 m

## Why "exactly three" and not "at least two"

Red-orange is the detector's tracked confusion (6% orange->red, 3% red->orange
in the v1 report), and the two directions fail differently: a dead end read as a
gate hands the car to a junction manoeuvre at a wall. Requiring exactly three
reds, all inside `GATE_ARM_RANGE_M`, with BOTH gaps inside the corridor-pair
window, is a much narrower target than a lone misread orange can hit -- the
dead-end wall cone sits 2.5 m or more past the junction, alone.

The cost of being strict is that a one-tick detector dropout un-detects the
junction. That is deliberate: `topo_state` latches the junction on commit, so a
dropout inside the mouth costs nothing, and a dropout during approach only
delays arming by a tick. Recovering a gate from two cones would mean guessing
which of the three is missing, and guessing wrong points the car at the wrong
branch.

## Why the axis is fitted here rather than passed in

`Junction.axis_rad` is the forward normal of the line through the three reds,
fitted per tick from the cones themselves. It is NOT the caller's `axis_rad`
feedback, because `junction_exec.keep_branch` cuts a half-plane that passes
within 0.30 m of the routed branch's inner wall at the first exit row: a 10 deg
error in the axis moves that cut by 0.35 m at 2 m and drops the wall the car is
about to follow. The reds span 3 m, so the fit is well conditioned and needs no
history.

The caller's heading is still used to ORDER the three cones left-to-right,
where a coarse axis is plenty and the fit is not yet available.

## Why `survey` exists and `detect` is written in terms of it

"No junction this tick" has four causes -- no reds at all, reds in view but not
all three inside the arm range, a fourth red, and three reds whose gaps miss the
pair window -- and they have four unrelated fixes. A boolean cannot tell them
apart, and neither can a count: the arm range is a SLANT range, so a car parked
2.75 m from the red line has its outer reds at 3.06 m and reports one red in
range while three sit in plain view.

`survey` returns the reason alongside the answer, and `detect` is a one-line
wrapper over it, so the diagnosis in the trial log is produced by the same code
path as the decision and cannot drift away from it.
"""

import math

from cone_nav.corridor.boundary_split import split
from cone_nav.corridor.centerline import MAX_PAIR_EDGE_M, MIN_PAIR_EDGE_M

# Past this the LD06 returns fewer than two points per cone (the arithmetic is
# in cone_perception/clustering.py), so a "red" out here is one return and a
# hope. Arming a junction manoeuvre on that is worse than arming late.
GATE_ARM_RANGE_M = 3.0

# How far a red may sit off the line fitted through the three, perpendicular
# distance. A real gate is one tape line; its residual is centimetres. The
# guard exists because the gap window alone once passed a trio strung 2.2 m
# ALONG the corridor -- three unrelated reds whose mutual spacings happened to
# land inside [0.6, 2.5] -- and committed a junction at a gate that was not
# there (observed 2026-09-01, with label memory multiplying the near reds).
MAX_LINE_OFFSET_M = 0.20

# Why `detect` declined, in the words the trial log carries. Short enough to sit
# in a column, specific enough to name the fix: DISTANCE sends you to where the
# car is standing, CROWDED to the clear band around each red, EXTRA to the
# detector, GAPS to the tape.
NO_REDS = "no reds"
DISTANCE = "reds in view, not all three in range"
CROWDED = "fewer than three reds"
EXTRA = "more than three reds in range"
GAPS = "gaps outside the pair window"
SCATTER = "reds not in a line"


class RedSurvey(object):
    """Every red in the scene, what `detect` made of them, and why.

    `reds` is all of them out to `boundary_split`'s range cut; `in_arm` is the
    subset `detect` is allowed to build a gate from. The two differing is the
    single most useful thing this record says, because it separates a track the
    car cannot see from a car standing in the wrong place, and those look
    identical in a count of recovered triples.
    """

    __slots__ = ("reds", "in_arm", "gaps_m", "junction", "reason")

    def __init__(self, reds, in_arm, gaps_m, junction, reason):
        self.reds = reds
        self.in_arm = in_arm
        self.gaps_m = gaps_m
        self.junction = junction
        self.reason = reason

    @property
    def ranges_m(self):
        """Range to every red, nearest first. `split` already sorted them."""
        return [math.hypot(c.x, c.y) for c in self.reds]

    def __repr__(self):
        gaps = ("%.2f/%.2f" % self.gaps_m) if self.gaps_m else "-"
        return (f"RedSurvey({len(self.in_arm)}/{len(self.reds)} in range, "
                f"gaps {gaps}, {self.reason or 'detected'})")


class Junction(object):
    """The three red cones, the two gates, and the axis they define."""

    __slots__ = ("left", "centre", "right", "left_gate", "right_gate",
                 "axis_rad")

    def __init__(self, left, centre, right, left_gate, right_gate, axis_rad):
        self.left = left
        self.centre = centre
        self.right = right
        self.left_gate = left_gate
        self.right_gate = right_gate
        self.axis_rad = axis_rad

    def gate_for(self, turn):
        """The gate midpoint the route names. `turn` is 'left' or 'right'."""
        if turn == "left":
            return self.left_gate
        if turn == "right":
            return self.right_gate
        raise ValueError(f"turn must be 'left' or 'right', got {turn!r}")

    def range_for(self, turn):
        """Distance from base_link to that gate's midpoint."""
        gate = self.gate_for(turn)
        return math.hypot(gate[0], gate[1])

    @property
    def gaps_m(self):
        """The two gate widths, left first. Logged so a mis-laid track shows up."""
        return (_distance(self.left, self.centre),
                _distance(self.centre, self.right))

    def __repr__(self):
        left, right = self.gaps_m
        return (f"Junction(gaps {left:.2f}/{right:.2f} m, "
                f"axis {math.degrees(self.axis_rad):.1f} deg)")


def _distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def _midpoint(a, b):
    return ((a.x + b.x) / 2.0, (a.y + b.y) / 2.0)


def _offset(cone, axis_rad):
    """Perpendicular offset from the corridor axis, left positive.

    The same expression `side_assign.fill_unlabeled` uses to decide which wall a
    cone is on, reused rather than re-derived so the two cannot drift apart.
    """
    return -cone.x * math.sin(axis_rad) + cone.y * math.cos(axis_rad)


def fit_axis(cones, default_rad=0.0):
    """Forward normal of the best-fit line through the cones.

    Total least squares via the 2x2 scatter matrix: the principal eigenvector is
    the line direction, and for a symmetric [[a, b], [b, c]] its angle is
    `0.5 * atan2(2b, a - c)`. Ordinary least squares is wrong here -- the gate
    line is very nearly vertical in base_link (constant x, spread in y), which is
    exactly where a y-on-x fit blows up.

    The normal has a 180 deg ambiguity; it is resolved forward, because
    `boundary_split` has already dropped anything behind x = -0.5 m and the car
    only ever meets a gate ahead of itself.
    """
    if len(cones) < 2:
        return default_rad

    mean_x = sum(c.x for c in cones) / len(cones)
    mean_y = sum(c.y for c in cones) / len(cones)
    sxx = sum((c.x - mean_x) ** 2 for c in cones)
    syy = sum((c.y - mean_y) ** 2 for c in cones)
    sxy = sum((c.x - mean_x) * (c.y - mean_y) for c in cones)
    if sxx + syy < 1e-12:
        return default_rad

    line_rad = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
    normal_rad = line_rad + math.pi / 2.0
    if math.cos(normal_rad) < 0.0:
        normal_rad += math.pi
    return math.atan2(math.sin(normal_rad), math.cos(normal_rad))


def survey(cones, axis_rad=0.0, min_gap_m=MIN_PAIR_EDGE_M,
           max_gap_m=MAX_PAIR_EDGE_M, arm_range_m=GATE_ARM_RANGE_M):
    """LabeledCones -> RedSurvey: the junction if there is one, and why if not.

    `min_gap_m` and `max_gap_m` default to `centerline`'s own pair-edge window
    rather than to fresh constants, so a gate this module accepts is a gate
    `midpoint_graph` would also pair. The 3.0 m left-to-right span of a v2
    junction is deliberately WIDER than `MAX_PAIR_EDGE_M`, which is what stops
    the triangulation putting a spurious gate midpoint on the centre cone --
    see `data/layouts/junction_v2.md`.

    Never raises and never returns None. A tick with no reds in it is a normal
    tick on a corridor, and most ticks of any run are that.
    """
    reds = split(cones).gates
    in_arm = [c for c in reds if math.hypot(c.x, c.y) <= arm_range_m]

    # Gaps are measured off whichever three reds are available, in arm range
    # for preference but in view otherwise. That second case is the whole point
    # of measuring them here rather than only on success: a car standing a metre
    # too far back sees three perfectly good reds and recovers no triple, and
    # the gaps are what prove the track is laid right and the RANGE is the
    # problem. Measured on more or fewer than three there is no gate to measure.
    triple = in_arm if len(in_arm) == 3 else (reds if len(reds) == 3 else None)
    gaps, junction, collinear = None, None, True
    if triple is not None:
        fitted = fit_axis(triple, default_rad=axis_rad)
        # Order left to right across the mouth. Sorting on the fitted axis
        # rather than the caller's keeps the ordering correct when the car meets
        # the gate mid-turn, which is the normal case at the second junction of
        # a route.
        left, centre, right = sorted(triple, key=lambda c: _offset(c, fitted),
                                     reverse=True)
        gaps = (_distance(left, centre), _distance(centre, right))
        # Along-axis offset of each red from the centre one: the reds are
        # laid on one tape line ACROSS the axis, so a real gate's offsets are
        # centimetres, and three cones that will not sit on a line are not a
        # gate no matter what their spacings measure.
        along = [(c.x - centre.x) * math.cos(fitted)
                 + (c.y - centre.y) * math.sin(fitted)
                 for c in (left, centre, right)]
        collinear = max(abs(a) for a in along) <= MAX_LINE_OFFSET_M
        if (len(in_arm) == 3 and collinear
                and all(min_gap_m <= g <= max_gap_m for g in gaps)):
            junction = Junction(
                left=left, centre=centre, right=right,
                left_gate=_midpoint(left, centre),
                right_gate=_midpoint(centre, right),
                axis_rad=fitted,
            )

    if junction is not None:
        reason = ""
    elif not reds:
        reason = NO_REDS
    elif len(in_arm) > 3:
        reason = EXTRA
    elif len(reds) >= 3:
        # Three or more exist; if they are not all in range the car is too far
        # back; in range but not collinear is not a gate at all; else a gap
        # failed.
        if len(in_arm) < 3:
            reason = DISTANCE
        elif triple is not None and not collinear:
            reason = SCATTER
        else:
            reason = GAPS
    else:
        reason = CROWDED

    return RedSurvey(reds=reds, in_arm=in_arm, gaps_m=gaps,
                     junction=junction, reason=reason)


def detect(cones, axis_rad=0.0, min_gap_m=MIN_PAIR_EDGE_M,
           max_gap_m=MAX_PAIR_EDGE_M, arm_range_m=GATE_ARM_RANGE_M):
    """LabeledCones -> Junction, or None if this is not a junction.

    The decision half of `survey`, kept as its own name because that is what
    every caller that only has to STEER wants. Written as a wrapper rather than
    as a second implementation so the reason logged for a rejection is always
    the reason for this tick's rejection.
    """
    return survey(cones, axis_rad=axis_rad, min_gap_m=min_gap_m,
                  max_gap_m=max_gap_m, arm_range_m=arm_range_m).junction
