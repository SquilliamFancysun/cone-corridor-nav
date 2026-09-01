"""What counts as a junction, and -- mostly -- what does not.

Red-orange is the detector's tracked confusion, and a false gate hands the car
to a turn manoeuvre at a wall. So the bulk of these tests push malformed cone
sets at `detect` and require None back.
"""

import math

import pytest

from cone_nav.corridor.centerline import (
    GATE,
    MAX_PAIR_EDGE_M,
    MIN_PAIR_EDGE_M,
    midpoint_graph,
)
from cone_nav.topology.gate_detect import (
    CROWDED,
    DISTANCE,
    EXTRA,
    GAPS,
    GATE_ARM_RANGE_M,
    NO_REDS,
    detect,
    fit_axis,
    survey,
)
from cone_perception.cone_classes import (
    CLASS_BLUE,
    CLASS_ORANGE,
    CLASS_RED,
    CLASS_YELLOW,
)
from cone_perception.fusion import LabeledCone


def cone(x, y, cls=CLASS_RED, confidence=0.9, points=4):
    return LabeledCone(cone_class=cls, confidence=confidence, x=x, y=y,
                       range_lidar=math.hypot(x, y), points=points)


def triple(x=2.0, gap=1.5):
    """The junction v2 gate, square on to the car at range `x`."""
    return [cone(x, gap), cone(x, 0.0), cone(x, -gap)]


def rotated_triple(x=2.0, gap=1.5, axis_deg=20.0):
    """The same gate met at `axis_deg` off the car's heading."""
    rad = math.radians(axis_deg)
    dx, dy = -math.sin(rad), math.cos(rad)
    centre = (x, 0.0)
    return [cone(centre[0] + gap * dx, centre[1] + gap * dy),
            cone(*centre),
            cone(centre[0] - gap * dx, centre[1] - gap * dy)]


# --- the happy path -----------------------------------------------------

def test_three_reds_give_two_gates():
    junction = detect(triple())
    assert junction is not None
    assert junction.left_gate == pytest.approx((2.0, 0.75))
    assert junction.right_gate == pytest.approx((2.0, -0.75))


def test_the_centre_cone_is_shared_by_both_gates():
    """It is the island nose. keep_branch cuts on it, so it must be the same
    cone in both gates rather than two cones that happen to coincide."""
    junction = detect(triple())
    assert junction.centre is not junction.left
    assert junction.centre is not junction.right
    assert junction.gaps_m == pytest.approx((1.5, 1.5))


def test_left_is_to_the_left():
    junction = detect(triple())
    assert junction.left.y > junction.centre.y > junction.right.y


def test_the_route_picks_the_gate():
    junction = detect(triple())
    assert junction.gate_for("left") == junction.left_gate
    assert junction.gate_for("right") == junction.right_gate
    assert junction.range_for("left") == pytest.approx(math.hypot(2.0, 0.75))


def test_an_unknown_turn_is_refused():
    with pytest.raises(ValueError):
        detect(triple()).gate_for("straight")


# --- the fitted axis ----------------------------------------------------

def test_the_axis_of_a_square_on_gate_is_straight_ahead():
    assert detect(triple()).axis_rad == pytest.approx(0.0, abs=1e-9)


def test_the_axis_is_fitted_from_the_reds_not_taken_from_the_caller():
    """The caller is told the corridor runs straight ahead; the cones say it
    runs 20 deg left. keep_branch cuts within 0.30 m of the routed branch's
    inner wall, so believing the caller here drops that wall."""
    junction = detect(rotated_triple(axis_deg=20.0), axis_rad=0.0)
    assert math.degrees(junction.axis_rad) == pytest.approx(20.0, abs=0.01)


def test_ordering_survives_meeting_the_gate_mid_turn():
    """At 20 deg off, the three cones' raw y values no longer order them."""
    junction = detect(rotated_triple(axis_deg=20.0))
    assert junction.gaps_m == pytest.approx((1.5, 1.5))
    assert junction.left_gate[1] > junction.right_gate[1]


def test_a_near_vertical_gate_line_does_not_blow_up():
    """The gate line is constant-x and spread in y, which is exactly where an
    ordinary y-on-x least squares fit diverges. Hence total least squares."""
    assert fit_axis(triple()) == pytest.approx(0.0, abs=1e-9)


def test_fit_axis_falls_back_when_there_is_nothing_to_fit():
    assert fit_axis([cone(1.0, 0.0)], default_rad=0.5) == 0.5
    assert fit_axis([], default_rad=0.5) == 0.5


# --- what is not a junction ---------------------------------------------

def test_two_reds_are_not_a_junction():
    """A pair is track_v1's old gate shape. Recovering a v2 gate from two cones
    means guessing which of the three is missing, and guessing wrong points the
    car down the other branch."""
    assert detect([cone(2.0, 1.5), cone(2.0, 0.0)]) is None


def test_four_reds_in_range_are_not_a_junction():
    """Something is wrong -- a misread orange, or a second junction bleeding
    into view -- and neither is a thing to commit a turn on."""
    assert detect(triple() + [cone(1.0, 1.0)]) is None


def test_a_fourth_red_out_of_range_does_not_spoil_a_good_gate():
    """The arm-range filter runs before the count, so the next junction's cones
    appearing at 4 m must not un-detect the one in front of the car."""
    assert detect(triple() + [cone(4.5, 0.0)]) is not None


def test_no_reds_is_not_a_junction():
    assert detect([cone(2.0, 0.75, cls=CLASS_BLUE)]) is None


def test_a_lone_misread_orange_cannot_fake_a_gate():
    """The tracked failure mode: 6% of oranges came back red in the v1 report.
    A dead-end wall cone is one cone, alone, well past the junction."""
    assert detect([cone(4.0, 0.0, cls=CLASS_ORANGE)]) is None
    assert detect([cone(4.0, 0.0)]) is None


def test_a_gate_beyond_the_arm_range_is_not_armed():
    """Past 3 m the LD06 returns under two points per cone, so the positions
    these gaps are measured from are one return and a hope."""
    assert detect(triple(x=GATE_ARM_RANGE_M + 0.5)) is None


def test_gaps_wider_than_a_corridor_pair_are_refused():
    assert detect(triple(gap=MAX_PAIR_EDGE_M + 0.1)) is None


def test_gaps_narrower_than_a_corridor_pair_are_refused():
    """Below MIN_PAIR_EDGE_M the gap is not a mouth the car fits through."""
    assert detect(triple(gap=MIN_PAIR_EDGE_M - 0.05)) is None


def test_a_lopsided_triple_is_refused():
    """Both gaps must pass, not the average. One good gap and one 0.2 m gap is
    a mis-laid junction, and committing to it drives at a cone."""
    assert detect([cone(2.0, 1.5), cone(2.0, 0.0), cone(2.0, -0.2)]) is None


def test_reds_behind_the_car_are_not_a_junction():
    """boundary_split already drops x < -0.5; this pins that we inherit it."""
    assert detect([cone(-1.0, 1.5), cone(-1.0, 0.0), cone(-1.0, -1.5)]) is None


# --- the geometry choice, checked against centerline itself -------------

def test_the_left_to_right_span_is_wider_than_any_corridor_pair():
    """The reason junction v2 uses 1.5 m gaps rather than 0.75 m.

    midpoint_graph pairs red with red, so all three of L-C, C-R and L-R are
    candidate gates. At a 3.0 m span the L-R edge exceeds MAX_PAIR_EDGE_M and
    is dropped by the width filter that already exists -- no new code. At
    0.75 m gaps the span would be 1.5 m, inside the window, and a phantom gate
    midpoint would land on the centre cone, inviting the car to drive at it.
    """
    assert 2 * 1.5 > MAX_PAIR_EDGE_M
    assert MIN_PAIR_EDGE_M <= 1.5 <= MAX_PAIR_EDGE_M
    assert MIN_PAIR_EDGE_M <= 2 * 0.75 <= MAX_PAIR_EDGE_M


def test_a_junction_in_a_corridor_yields_exactly_the_two_gates():
    """The claim above, run through the real midpoint_graph in a scene that has
    corridor cones in it -- which is the only scene the car ever sees."""
    scene = triple() + [
        cone(0.5, 0.75, cls=CLASS_BLUE), cone(0.5, -0.75, cls=CLASS_YELLOW),
        cone(1.25, 0.75, cls=CLASS_BLUE), cone(1.25, -0.75, cls=CLASS_YELLOW),
    ]
    gates = [m for m in midpoint_graph(scene)[0] if m.kind == GATE]
    assert len(gates) == 2
    assert sorted(round(g.y, 2) for g in gates) == [-0.75, 0.75]
    assert all(g.width_m == pytest.approx(1.5) for g in gates)


def test_gate_detect_does_not_read_its_reds_from_the_centerline():
    """Why `detect` goes to boundary_split rather than CenterlineResult.gates.

    Three exactly-collinear points have no Delaunay triangulation, so a
    perfectly laid gate produces ZERO triangles and `midpoint_graph` returns
    nothing at all. In a real scene the corridor cones break the degeneracy, but
    a junction glimpsed on its own -- the corridor behind, nothing yet ahead --
    is precisely the moment the machine needs to arm. Reading the red bucket
    directly is immune to it.
    """
    assert midpoint_graph(triple()) == ([], {})
    assert detect(triple()) is not None


# --- the survey, which is what the trial log carries ---------------------

def test_the_survey_and_the_detector_never_disagree():
    """`detect` is a wrapper over `survey`, and this is the property that makes
    that worth doing: the reason recorded in the log is the reason for THIS
    tick's decision, not a second opinion computed alongside it."""
    for cones in (triple(2.0, 1.35), triple(2.0, 0.2), triple(4.0, 1.35),
                  triple(2.75, 1.35), triple(2.0, 1.35) + [cone(1.0, 0.3)],
                  [], [cone(1.0, 0.0)]):
        found, decided = survey(cones).junction, detect(cones)
        assert (found is None) == (decided is None)
        if found is not None:
            assert found.gaps_m == pytest.approx(decided.gaps_m)
            assert found.left_gate == pytest.approx(decided.left_gate)
            assert found.right_gate == pytest.approx(decided.right_gate)
        # A reason is recorded exactly when there is no junction, so a log can
        # never show a rejection with no cause or a gate with a complaint.
        assert bool(survey(cones).reason) == (found is None)


def test_a_car_too_far_back_is_told_so_rather_than_blamed_on_the_track():
    """The arm range is a SLANT range. An outer red 1.35 m off the axis is
    3.06 m away when the red line is only 2.75 m ahead, so all three cones can
    be in plain view with one of them countable -- which reads as a mis-laid
    track unless the survey says otherwise."""
    found = survey(triple(2.75, 1.35))
    assert found.junction is None
    assert found.reason == DISTANCE
    assert len(found.reds) == 3
    assert len(found.in_arm) < 3
    # ...and the gaps are still measured, which is what proves the tape work is
    # fine and only the standing position is wrong.
    assert found.gaps_m == pytest.approx((1.35, 1.35), abs=0.01)


def test_the_gaps_are_measured_even_when_they_are_what_failed():
    found = survey(triple(2.0, 0.25))
    assert found.junction is None
    assert found.reason == GAPS
    assert found.gaps_m == pytest.approx((0.25, 0.25), abs=0.01)


def test_the_four_reasons_are_distinct():
    assert survey([]).reason == NO_REDS
    assert survey([cone(1.0, 0.0), cone(1.0, 1.0)]).reason == CROWDED
    assert survey(triple(2.0, 1.35) + [cone(1.0, 0.2)]).reason == EXTRA
    assert survey(triple(2.75, 1.35)).reason == DISTANCE
    assert survey(triple(2.0, 0.25)).reason == GAPS
    assert survey(triple(2.0, 1.35)).reason == ""


def test_the_survey_reports_every_red_by_range_nearest_first():
    found = survey(triple(2.0, 1.35))
    assert len(found.ranges_m) == 3
    assert found.ranges_m == sorted(found.ranges_m)
    assert found.ranges_m[0] == pytest.approx(2.0, abs=0.01)


def test_three_reds_strung_along_the_corridor_are_not_a_gate():
    """Observed live: reds at 0.74, 1.08 and 2.95 m whose mutual spacings
    happened to land inside the gap window, committing a junction at a gate
    that was not there. A real gate is one tape line ACROSS the axis; a trio
    with metres of along-axis scatter fails it whatever its spacings say."""
    from cone_nav.topology.gate_detect import SCATTER

    strung = [cone(0.7, 0.3), cone(1.1, -0.3), cone(2.9, 0.1)]
    assert detect(strung) is None
    assert survey(strung).reason == SCATTER


def test_tape_slop_does_not_trip_the_collinearity_guard():
    """+-10 cm of along-axis slop is sloppy tape, not a phantom gate."""
    slop = [cone(2.05, 0.76), cone(1.98, 0.0), cone(2.08, -0.76)]
    assert detect(slop) is not None
