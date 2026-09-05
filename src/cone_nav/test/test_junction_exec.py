"""The branch filter, and the one failure it exists to prevent.

The scene these tests build is the real junction v2 geometry from
`data/layouts/junction_v2.md`: three red cones 1.5 m apart across the mouth,
each branch starting at its own gate midpoint and diverging 20 deg, both walls
of both branches from the first row 0.75 m out.
"""

import math

import pytest

from cone_nav.corridor.centerline import CORRIDOR, centerline, midpoint_graph
from cone_nav.guidance.junction_exec import (
    ANCHOR_MERGE_M,
    junction_line,
    keep_branch,
    select,
)
from cone_nav.topology.gate_detect import detect
from cone_perception.cone_classes import CLASS_BLUE, CLASS_RED, CLASS_YELLOW
from cone_perception.fusion import LabeledCone

HALF_WIDTH_M = 0.75
GATE_GAP_M = 1.5
DIVERGENCE_DEG = 20.0
SPACING_M = 0.75


def cone(x, y, cls, confidence=0.9):
    return LabeledCone(cone_class=cls, confidence=confidence, x=x, y=y,
                       range_lidar=math.hypot(x, y), points=4)


def _branch(sign, rows, x0):
    """One branch, from its own gate midpoint at (0, sign*0.75), in car frame.

    Outer wall is blue on the left branch and yellow on the right, so each
    branch keeps blue-left / yellow-right in its own direction of travel.
    """
    rad = math.radians(sign * DIVERGENCE_DEG)
    ux, uy = math.cos(rad), math.sin(rad)
    nx, ny = -math.sin(rad), math.cos(rad)
    out = []
    for i in range(1, rows + 1):
        t = SPACING_M * i
        cx, cy = x0 + ux * t, sign * HALF_WIDTH_M + uy * t
        left = (cx + nx * HALF_WIDTH_M, cy + ny * HALF_WIDTH_M)
        right = (cx - nx * HALF_WIDTH_M, cy - ny * HALF_WIDTH_M)
        out.append(cone(left[0], left[1], CLASS_BLUE))
        out.append(cone(right[0], right[1], CLASS_YELLOW))
    return out


def scene(junction_x=2.0, rows=3, incoming=2):
    """The full junction as the car sees it, junction line at `junction_x`."""
    cones = [cone(junction_x, GATE_GAP_M, CLASS_RED),
             cone(junction_x, 0.0, CLASS_RED),
             cone(junction_x, -GATE_GAP_M, CLASS_RED)]
    for i in range(1, incoming + 1):
        x = junction_x - SPACING_M * i
        cones.append(cone(x, HALF_WIDTH_M, CLASS_BLUE))
        cones.append(cone(x, -HALF_WIDTH_M, CLASS_YELLOW))
    cones += _branch(+1, rows, junction_x)
    cones += _branch(-1, rows, junction_x)
    return cones


def filtered(turn, junction_x=2.0, **kwargs):
    cones = scene(junction_x=junction_x, **kwargs)
    junction = detect(cones)
    assert junction is not None, "the fixture must be a detectable junction"
    gate_xy, divider_xy = select(junction, turn)
    kept, dropped = keep_branch(cones, divider_xy, junction.axis_rad, turn)
    return cones, kept, dropped, gate_xy


def island_midpoints(cones, divider_x, tolerance=0.35):
    """Corridor midpoints sitting on the island axis, which is y = 0 here."""
    midpoints, _adjacency = midpoint_graph(cones)
    return [m for m in midpoints
            if m.kind == CORRIDOR and abs(m.y) < tolerance and m.x > divider_x]


# --- the failure the filter exists for ----------------------------------

def test_the_unfiltered_scene_puts_a_midpoint_on_the_island():
    """The two branches' inner cones are 0.60 m apart at the first row, which
    is inside [MIN_PAIR_EDGE_M, MAX_PAIR_EDGE_M]. Left branch's yellow pairs
    with right branch's blue and the midpoint lands on the island, pointing the
    car at the divider. This is the thing keep_branch removes."""
    assert island_midpoints(scene(), divider_x=2.0)


def test_the_filter_removes_the_cross_branch_pair():
    _cones, kept, _dropped, _gate = filtered("left")
    assert not island_midpoints(kept, divider_x=2.0)


@pytest.mark.parametrize("turn", ["left", "right"])
def test_no_divergence_angle_would_have_saved_us(turn):
    """Stated as a test because it is the justification for the filter being
    load-bearing: the cross-branch width only leaves the pairing window several
    metres past the junction, long after the car has committed."""
    _cones, kept, _dropped, _gate = filtered(turn)
    assert not island_midpoints(kept, divider_x=2.0)


# --- what the filter keeps ----------------------------------------------

def test_the_incoming_corridor_survives_the_filter():
    """The half-plane divides the two branches and nothing else. Applied to the
    whole scene it would throw away one wall of the corridor the car is still
    driving down, several metres before it needs to commit."""
    _cones, kept, _dropped, _gate = filtered("left")
    behind = [c for c in kept if c.x < 2.0 - 1e-9]
    assert sorted(c.cone_class for c in behind) == sorted(
        [CLASS_BLUE, CLASS_YELLOW] * 2)


def test_the_approach_still_has_a_two_sided_corridor():
    """The consequence of the test above, stated the way the car experiences
    it: no single-boundary fallback while merely approaching a junction."""
    _cones, kept, _dropped, _gate = filtered("left", junction_x=2.5)
    assert not centerline(kept).single_boundary_fallback


def test_a_left_turn_keeps_the_left_branch():
    _cones, kept, _dropped, _gate = filtered("left")
    ahead = [c for c in kept if c.x > 2.0]
    assert ahead and all(c.y > -0.5 for c in ahead)


def test_a_right_turn_keeps_the_right_branch():
    _cones, kept, _dropped, _gate = filtered("right")
    ahead = [c for c in kept if c.x > 2.0]
    assert ahead and all(c.y < 0.5 for c in ahead)


def test_the_routed_branch_still_forms_a_real_corridor():
    """Filtering must not be so aggressive that the branch stops pairing."""
    _cones, kept, _dropped, _gate = filtered("left")
    line = centerline(kept)
    assert len(line.points) >= 2
    assert not line.single_boundary_fallback


def test_a_gate_arms_later_than_its_arm_range_suggests():
    """GATE_ARM_RANGE_M is measured to each CONE, and the outer reds sit 1.5 m
    off the axis. So a 3.0 m arm range first sees a whole triple when the
    junction LINE is sqrt(3.0^2 - 1.5^2) = 2.60 m out, not 3.0 m. That leaves
    1.1 m of approach before the 1.5 m commit -- about nine ticks at the
    measured 9.9 Hz. Worth knowing rather than discovering on the track."""
    assert detect(scene(junction_x=2.55)) is not None
    assert detect(scene(junction_x=2.65)) is None


def test_the_reds_are_always_dropped():
    _cones, kept, _dropped, _gate = filtered("left")
    assert not [c for c in kept if c.cone_class == CLASS_RED]


def test_a_red_exactly_on_the_junction_line_is_not_kept_by_a_signed_zero():
    """The outer reds sit exactly on the cut by construction, so side-testing
    them makes the result depend on the sign of a floating-point zero. Observed:
    a fitted axis of -0.0 rad put the left red at along = -1.5e-17 and kept it."""
    cones = scene()
    kept, _dropped = keep_branch(cones, (2.0, 0.0), -0.0, "left")
    assert not [c for c in kept if c.cone_class == CLASS_RED]


def test_the_dropped_count_reports_the_bite():
    """A tick that drops nothing at a junction means the filter is not biting,
    which is worth seeing in the trial log rather than inferring."""
    _cones, _kept, dropped, _gate = filtered("left")
    assert dropped > 0


def test_the_input_is_never_mutated():
    cones = scene()
    before = [(c.x, c.y, c.cone_class) for c in cones]
    keep_branch(cones, (detect(cones).centre.x, 0.0), 0.0, "left")
    assert [(c.x, c.y, c.cone_class) for c in cones] == before


def test_an_unknown_turn_is_refused():
    cones = scene()
    with pytest.raises(ValueError):
        keep_branch(cones, (detect(cones).centre.x, 0.0), 0.0, "straight")


def test_select_returns_the_gate_and_the_divider_point():
    """The divider comes back as a bare (x, y) rather than the cone, because
    topo_state has to carry it forward through the blind period and a moved
    LabeledCone would carry detector fields that are no longer true."""
    junction = detect(scene())
    gate_xy, divider_xy = select(junction, "left")
    assert gate_xy == junction.left_gate
    assert divider_xy == (junction.centre.x, junction.centre.y)


# --- the gate anchor ----------------------------------------------------

class FakeLine(object):
    def __init__(self, points):
        self.points = points
        self.corridor_half_width = 0.75
        self.single_boundary_fallback = False
        self.midpoints = []
        self.adjacency = {}
        self.gates = []


def test_the_anchor_goes_in_by_distance_not_at_the_front():
    """lookahead_point walks the polyline near to far, so a point out of order
    has the car aim backwards through the mouth."""
    line = junction_line(FakeLine([(1.0, 0.0), (3.0, 0.5)]), (2.0, 0.75))
    assert line.points == [(1.0, 0.0), (2.0, 0.75), (3.0, 0.5)]


def test_the_line_stays_ordered_by_distance():
    line = junction_line(FakeLine([(1.0, 0.0), (2.0, 0.1), (3.0, 0.5)]),
                         (2.4, 0.75))
    reaches = [math.hypot(x, y) for x, y in line.points]
    assert reaches == sorted(reaches)


def test_an_anchor_past_the_whole_chain_goes_last():
    line = junction_line(FakeLine([(1.0, 0.0), (1.5, 0.0)]), (2.5, 0.75))
    assert line.points[-1] == (2.5, 0.75)


def test_a_near_duplicate_chain_point_is_merged_away():
    """The corridor pair either side of the mouth can produce a midpoint almost
    on the gate. Threading both in puts a kink in the opening; centerline keeps
    gate midpoints out of the chain for the same reason."""
    line = junction_line(FakeLine([(1.0, 0.0), (2.0, 0.75 + ANCHOR_MERGE_M / 2)]),
                         (2.0, 0.75))
    assert line.points == [(1.0, 0.0), (2.0, 0.75)]


def test_an_empty_chain_gives_a_line_too_short_to_drive():
    """Deliberate. One point means pure_pursuit returns None and speed_ctrl
    stops -- which is right when the car cannot see where the branch goes."""
    assert junction_line(FakeLine([]), (2.0, 0.75)).points == [(2.0, 0.75)]


def test_the_anchor_does_not_mutate_the_line_it_was_given():
    original = FakeLine([(1.0, 0.0), (3.0, 0.5)])
    junction_line(original, (2.0, 0.75))
    assert original.points == [(1.0, 0.0), (3.0, 0.5)]


def test_the_rest_of_the_result_is_carried_through():
    line = junction_line(FakeLine([(1.0, 0.0)]), (2.0, 0.75))
    assert line.corridor_half_width == 0.75
    assert line.single_boundary_fallback is False
