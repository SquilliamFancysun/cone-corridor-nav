"""Turn execution through a junction; reacquire the corridor on the far side.
Pure, no rclpy.

This is the whole of "junction navigation". There is no second control stack:
`centerline`, `pure_pursuit` and `speed_ctrl` run exactly as they do on a plain
corridor, and all that changes is the cone list handed to them.

`centerline._longest_forward_chain` says of itself:

    At a fork this returns the longer branch. When junction_exec lands it
    replaces this function with "the branch the route names" -- same graph,
    same midpoints, different choice.

It is done by filtering the INPUT rather than by rewriting that function. The
graph, the midpoints and the chain rule are all untouched; the other branch's
cones simply are not there to be chained. That keeps the longest-path DP and its
tests alone, and it means anything the corridor layer learns later is inherited
here for free.

## Why keep_branch is load-bearing rather than a refinement

At the first exit row the two branches' inner cones are 0.60 m apart and they
diverge slowly. That width sits inside `[MIN_PAIR_EDGE_M, MAX_PAIR_EDGE_M]`, so
a blue from one branch and a yellow from the other pair into a corridor midpoint
sitting on the centre red cone -- a midpoint on the island, pointing the car at
it. No divergence angle escapes that window at a usable distance (the pair only
exceeds 2.5 m some 3.5 m past the junction), so the geometry cannot fix it and
the filter must. `test_junction_exec.py` pins exactly that case.

## Why the cut is gated on being PAST the junction

The half-plane through the centre cone divides the two branches, and it divides
nothing else. Applied to the whole scene it also throws away one wall of the
corridor the car is still driving down -- for a left turn, every yellow cone
behind the junction -- which collapses the approach into a single-boundary
fallback several metres before the car needs to commit to anything. So cones
behind the junction line are kept unconditionally, and only what lies beyond it
is filtered. The car follows the corridor normally right up to the mouth.
"""

import math

from cone_nav.corridor.centerline import CenterlineResult
from cone_perception.cone_classes import CLASS_RED

# Slack on the branch cut, toward the divider. The routed branch's inner wall
# sits about 0.30 m off the cut at the first exit row, so this leaves 0.40 m of
# tolerance before an axis error would drop the wall the car is about to follow,
# while still leaving 0.20 m before it would start keeping the other branch's.
BRANCH_MARGIN_M = 0.10

# A chain point this close to the gate midpoint is the same feature seen twice.
# Keeping both threads a near-duplicate into the line and puts a kink in the
# mouth; `centerline` keeps gate midpoints out of the chain for the same reason.
ANCHOR_MERGE_M = 0.30


def select(junction, turn):
    """(gate midpoint, divider point) for the turn the route names.

    The divider is always the centre red cone -- it is shared by both gates and
    it is the physical island nose, so it is the one landmark that means the
    same thing whichever way the car is going. It comes back as a bare (x, y)
    rather than as the cone, because `topo_state` has to dead-reckon it through
    the stretch where the reds are no longer visible and a LabeledCone carries
    detector fields that would be lies once moved.
    """
    return junction.gate_for(turn), (junction.centre.x, junction.centre.y)


def _frame(cone, divider_xy, axis_rad):
    """Cone position relative to the junction: (along the axis, left of it).

    `offset` is `side_assign`'s expression, re-centred on the divider. Reused
    rather than re-derived so the two cannot drift apart.
    """
    dx = cone.x - divider_xy[0]
    dy = cone.y - divider_xy[1]
    sin_a, cos_a = math.sin(axis_rad), math.cos(axis_rad)
    return (dx * cos_a + dy * sin_a, -dx * sin_a + dy * cos_a)


def keep_branch(cones, divider_xy, axis_rad, turn, margin_m=BRANCH_MARGIN_M):
    """Drop the cones belonging to the branch the route did not name.

    Never mutates the input, matching `fill_unlabeled`'s convention. Returns
    `(kept, dropped_count)` so the caller can log how much of the scene was
    discarded -- a tick that drops nothing at a junction means the filter is not
    biting, and that is worth seeing in the trial log rather than inferring.

    Red cones are dropped outright rather than side-tested. They never form
    corridor midpoints (`midpoint_graph` only pairs blue with yellow), the gate
    midpoints they do form are not wanted past the mouth, and `gate_detect`
    reads them from the UNFILTERED list -- so nothing downstream misses them.
    Side-testing them instead makes a cone sitting exactly on the junction line
    depend on the sign of a floating-point zero, and the outer reds sit exactly
    there by construction.
    """
    if turn not in ("left", "right"):
        raise ValueError(f"turn must be 'left' or 'right', got {turn!r}")
    want_left = turn == "left"

    kept = []
    dropped = 0
    for cone in cones:
        if cone.cone_class == CLASS_RED:
            dropped += 1
            continue
        along, offset = _frame(cone, divider_xy, axis_rad)
        if along < 0.0:
            # Behind the mouth: the corridor the car is still in. The divider
            # says nothing about it. Junction v2 leaves 0.75 m clear either side
            # of the red line, so no boundary cone sits near this cut and the
            # bare sign test is safe here in a way it is not for the reds.
            kept.append(cone)
            continue
        on_route = offset > -margin_m if want_left else offset < margin_m
        if on_route:
            kept.append(cone)
        else:
            dropped += 1
    return kept, dropped


def junction_line(line, gate_xy, merge_m=ANCHOR_MERGE_M):
    """Thread an anchor point into the driven line, in distance order.

    Written for the gate midpoint and named for it, but the operation is general
    and `cone_nav/guidance/goal_stop.py` reuses it unchanged to hang the goal
    cone on the end of the line. The two cases share the property that makes an
    anchor necessary at all: the point matters to where the car should go, and
    the corridor pairs cannot express it -- `midpoint_graph` pairs only blue with
    yellow, so neither a red gate nor a magenta goal ever becomes a chain point.

    The anchor exists because the corridor pairs either side of the mouth do not
    themselves describe the mouth: the last incoming midpoint sits behind the
    red line and the first exit midpoint sits 0.75 m beyond it, and a straight
    hop between the two cuts the corner across the centre cone. The gate
    midpoint is the one point known to be in the middle of the opening.

    It is INSERTED by distance from the car rather than prepended, because
    `lookahead_point` walks the polyline near to far and a point out of order
    would have the car aim backwards through the mouth. Chain points within
    `merge_m` of the gate are dropped as the same feature seen twice.

    Returns a new `CenterlineResult`; the input is untouched.
    """
    def reach(point):
        return math.hypot(point[0], point[1])

    gate_reach = reach(gate_xy)
    points = [p for p in line.points
              if math.hypot(p[0] - gate_xy[0], p[1] - gate_xy[1]) > merge_m]

    index = len(points)
    for i, point in enumerate(points):
        if reach(point) > gate_reach:
            index = i
            break
    points = points[:index] + [tuple(gate_xy)] + points[index:]

    return CenterlineResult(
        points=points,
        corridor_half_width=line.corridor_half_width,
        single_boundary_fallback=line.single_boundary_fallback,
        midpoints=line.midpoints,
        adjacency=line.adjacency,
        gates=line.gates,
    )
