"""Closed-loop junction runs, and the layout constraints they depend on.

`sim/` is not in pytest.ini's testpaths, so these run with `pytest sim`.

The load-bearing test here is `test_the_route_is_what_picks_the_branch`: every
other run could pass with the junction code deleted, because
`_longest_forward_chain` picks a branch on its own and will sometimes pick the
right one. Only driving the SAME track with the WRONG route shows that the
route is steering anything.
"""

import math

import pytest

from cone_nav.control.speed_ctrl import MIN_REACH_M
from cone_nav.topology import gate_detect
from cone_perception import extrinsics

from sim import cone_field
from sim.drive_sim import simulate

WHEELBASE = extrinsics.WHEELBASE_M
AXLE = extrinsics.REAR_AXLE_IN_BASE
OTHER = {"left": "right", "right": "left"}


def run(turn, route=None, **layout):
    layout_cones = cone_field.track_junction(turn, **layout)
    return layout_cones, simulate(layout_cones, WHEELBASE, AXLE,
                                  lookahead_m=1.0, route=[route or turn])


# --- the manoeuvre ------------------------------------------------------

@pytest.mark.parametrize("turn", ["left", "right"])
def test_the_car_takes_the_junction_and_reaches_the_goal(turn):
    _layout, result = run(turn)
    assert result.completed, result.outcome
    assert result.struck_cone is None


@pytest.mark.parametrize("turn", ["left", "right"])
def test_it_does_not_clip_the_divider(turn):
    """The centre red cone is the island nose. Cutting the corner across it is
    the manoeuvre's characteristic failure."""
    _layout, result = run(turn)
    assert result.struck_cone is None, (
        f"struck a {result.struck_cone.color} cone" if result.struck_cone else "")


@pytest.mark.parametrize("turn", ["left", "right"])
def test_the_route_is_what_picks_the_branch(turn):
    """Same track, wrong route: the car must end up somewhere else. Without
    this, a run that merely follows the longer branch looks like success."""
    _layout, right_way = run(turn)
    _layout, wrong_way = run(turn, route=OTHER[turn])
    assert right_way.completed
    assert not wrong_way.completed


@pytest.mark.parametrize("turn", ["left", "right"])
def test_the_line_never_runs_short_through_the_mouth(turn):
    """The constraint the whole geometry answers to: speed_ctrl stops the car
    when reach drops under MIN_REACH_M, and a car that stops in a junction
    mouth cannot restart -- the scan does not change while it stands still."""
    _layout, result = run(turn)
    mouth = [t for t in result.ticks if t.topo == "traverse"]
    assert mouth, "the machine never entered the manoeuvre"
    assert min(t.reach_m for t in mouth) > MIN_REACH_M


@pytest.mark.parametrize("turn", ["left", "right"])
def test_the_branch_filter_actually_bites(turn):
    _layout, result = run(turn)
    assert max(t.dropped for t in result.ticks) > 0


@pytest.mark.parametrize("turn", ["left", "right"])
def test_the_turn_is_consumed_once_and_the_machine_lets_go(turn):
    """A manoeuvre that never ends leaves the branch filter cutting on a
    divider that is metres behind the car."""
    _layout, result = run(turn)
    states = [t.topo for t in result.ticks]
    assert states[-1] == "follow"
    assert "traverse" in states


# --- the layout constraints the runs depend on --------------------------

def test_the_junction_is_detectable_on_the_approach():
    """Measured, because it is thin: the outer reds enter lidar range at
    sqrt(3.0^2 - gap^2) and leave the camera frame at gap / tan(32.5 deg)."""
    layout = cone_field.track_junction("left")
    from cone_nav.corridor.boundary_split import split
    from cone_perception.geometry import intrinsics_from_hfov
    from sim.drive_sim import PREVIEW_H, PREVIEW_W, observe, pipeline

    intr = intrinsics_from_hfov(PREVIEW_W, PREVIEW_H)
    seen = 0
    for i in range(30):
        scan, dets = observe(layout, cone_field.Pose(i * 0.12, 0.0, 0.0), intr)
        _r, cones, _l, _f, _c, _d, _g = pipeline(scan, dets, intr)
        if gate_detect.detect(cones) is not None:
            seen += 1
    assert seen >= 2, f"only {seen} ticks of the approach see a whole triple"


@pytest.mark.parametrize("spacing", [0.75, 1.0])
def test_boundary_cones_stay_clear_of_the_reds(spacing):
    """The constraint that makes the triple recoverable at all, and it is the
    OPPOSITE of track_v1.md's "densify at the fork" advice.

    Two cones inside `clustering.GAP_DEG` of each other in bearing merge into
    one cluster. At 0.5 m spacing the first exit row lands 0.29 m from an outer
    red and the junction is never detected on any tick of the approach -- the
    car sails past and follows whichever branch is longer. At 0.75 m the
    clearance is 0.53 m and a whole triple is recovered on four consecutive
    ticks.
    """
    layout = cone_field.track_junction("left", spacing=spacing)
    reds = [c for c in layout if c.color == "red"]
    others = [c for c in layout if c.color != "red"]
    clearance = min(math.hypot(r.x - o.x, r.y - o.y)
                    for r in reds for o in others)
    assert clearance > 0.4, f"boundary cones sit {clearance:.2f} m from a red"


def test_dense_spacing_hides_the_junction():
    """The failure the test above guards against, pinned so the number in its
    docstring is not just an assertion."""
    layout = cone_field.track_junction("left", spacing=0.5)
    reds = [c for c in layout if c.color == "red"]
    others = [c for c in layout if c.color != "red"]
    clearance = min(math.hypot(r.x - o.x, r.y - o.y)
                    for r in reds for o in others)
    assert clearance < 0.4


def test_the_gate_span_is_wider_than_a_corridor_pair():
    """So the triangulation drops the outer-red edge instead of putting a
    phantom gate midpoint on the centre cone."""
    from cone_nav.corridor.centerline import MAX_PAIR_EDGE_M
    assert 2 * cone_field.JUNCTION_GATE_GAP_M > MAX_PAIR_EDGE_M


def test_the_dead_end_is_marked_orange_and_the_goal_magenta():
    layout = cone_field.track_junction("left")
    assert sum(1 for c in layout if c.color == "orange") == 1
    assert sum(1 for c in layout if c.color == "magenta") == 1
    assert sum(1 for c in layout if c.color == "red") == 3


# --- the build sheet must not drift from the builder --------------------

def _doc_table():
    """The `id color x y segment` rows out of data/layouts/junction_v2.md."""
    import os
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "data", "layouts", "junction_v2.md")
    rows = []
    for line in open(os.path.normpath(path), encoding="utf-8"):
        parts = line.split()
        if len(parts) == 5 and parts[0].isdigit():
            rows.append((parts[1], float(parts[2]), float(parts[3]), parts[4]))
    return rows


def test_the_build_sheet_matches_the_track_it_documents():
    """junction_v2.md is what someone lays the track from. A drawing that has
    drifted from track_junction() sends them out to build something the sim has
    never driven, so the numbers are generated and this pins that they still
    are."""
    cones = cone_field.track_junction("left")
    centre = [c for c in cones if c.color == "red"][1]
    expected = [(c.color, round(c.x - centre.x, 3), round(c.y - centre.y, 3),
                 c.segment) for c in cones]
    assert _doc_table() == expected


def test_the_build_sheet_states_the_gate_gap_the_code_uses():
    import os
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "data", "layouts", "junction_v2.md")
    text = open(os.path.normpath(path), encoding="utf-8").read()
    assert f"{cone_field.JUNCTION_GATE_GAP_M:.2f} m" in text
    assert f"{cone_field.JUNCTION_DIVERGENCE_DEG:.0f}" in text
