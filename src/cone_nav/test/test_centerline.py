"""The corridor layer: midpoints, the same-colour gate pair, and forks.

The fork tests are the reason this module uses a triangulation at all. A
boundary walk pairs "nearest blue ahead" with "nearest yellow ahead", and at a Y
those two cones belong to different corridors -- so the walk emits a midpoint
inside the island and steers the car into it. That specific failure is what
test_the_chain_does_not_cross_the_island checks for.
"""

import math

import pytest

from cone_nav.corridor import boundary_split
from cone_nav.corridor.centerline import (
    CORRIDOR,
    GATE,
    centerline,
    midpoint_graph,
)
from cone_perception.cone_classes import (
    CLASS_BLUE,
    CLASS_MAGENTA,
    CLASS_ORANGE,
    CLASS_RED,
    CLASS_YELLOW,
    UNLABELED,
)
from cone_perception.fusion import LabeledCone
from sim import cone_field

CLASS_OF = {"blue": CLASS_BLUE, "yellow": CLASS_YELLOW, "red": CLASS_RED,
            "orange": CLASS_ORANGE, "magenta": CLASS_MAGENTA}


def labeled(cones):
    """sim Cones -> LabeledCones, as though fusion had labelled them perfectly."""
    return [LabeledCone(cone_class=CLASS_OF[c.color], confidence=0.9,
                        x=c.x, y=c.y, range_lidar=math.hypot(c.x, c.y))
            for c in cones]


def straight(length=6.0):
    return labeled(cone_field.straight_corridor(length))


def track_at(x, y=0.0, heading=0.0, max_range=None):
    pose = cone_field.Pose(x, y, heading)
    return labeled(cone_field.cones_in_car_frame(
        cone_field.track_v1(), pose, max_range_m=max_range))


# --- the straight case --------------------------------------------------

def test_a_straight_corridor_yields_a_line_down_the_middle():
    result = centerline(straight())
    assert len(result.points) >= 4
    assert not result.single_boundary_fallback
    for x, y in result.points:
        assert abs(y) < 0.05, "the centerline is not centred"


def test_the_centerline_is_ordered_near_to_far():
    result = centerline(straight())
    xs = [p[0] for p in result.points]
    assert xs == sorted(xs)


def test_the_measured_half_width_matches_the_track():
    result = centerline(straight())
    assert result.corridor_half_width == pytest.approx(0.75, abs=0.05)


def test_midpoints_come_from_opposite_colours():
    cones = straight()
    midpoints, _ = midpoint_graph(cones)
    assert midpoints
    for m in midpoints:
        if m.kind != CORRIDOR:
            continue
        assert {cones[m.a].cone_class, cones[m.b].cone_class} == {
            CLASS_BLUE, CLASS_YELLOW}


def test_an_absurdly_long_pairing_is_rejected():
    """Delaunay will join cones across an island; that edge is nonsense."""
    cones = [
        LabeledCone(CLASS_BLUE, 0.9, 1.0, 4.0),
        LabeledCone(CLASS_YELLOW, 0.9, 1.0, -4.0),
        LabeledCone(CLASS_BLUE, 0.9, 2.0, 4.0),
        LabeledCone(CLASS_YELLOW, 0.9, 2.0, -4.0),
    ]
    midpoints, _ = midpoint_graph(cones)
    assert all(m.width_m <= 2.5 for m in midpoints)


# --- the same-colour pair ----------------------------------------------

def test_a_red_pair_produces_a_midpoint_between_two_cones_of_one_colour():
    """The junction mouth. A boundary walk cannot express this at all."""
    cones = straight(3.0) + [
        LabeledCone(CLASS_RED, 0.9, 2.0, 0.75),
        LabeledCone(CLASS_RED, 0.9, 2.0, -0.75),
    ]
    result = centerline(cones)
    assert result.gates, "no gate midpoint was produced"
    gate = result.gates[0]
    assert gate.kind == GATE
    assert cones[gate.a].cone_class == CLASS_RED
    assert cones[gate.b].cone_class == CLASS_RED
    assert gate.x == pytest.approx(2.0, abs=0.01)
    assert gate.y == pytest.approx(0.0, abs=0.01)


def test_a_lone_red_cone_is_not_a_gate():
    """gate_detect keys on pairs, so a single red is undetectable, not weak."""
    cones = straight(3.0) + [LabeledCone(CLASS_RED, 0.9, 2.0, 0.75)]
    assert centerline(cones).gates == []


def test_orange_and_magenta_do_not_pair_with_anything():
    """Only blue<->yellow and red<->red are corridor geometry."""
    cones = straight(3.0) + [
        LabeledCone(CLASS_ORANGE, 0.9, 2.5, 0.0),
        LabeledCone(CLASS_MAGENTA, 0.9, 2.8, 0.0),
    ]
    midpoints, _ = midpoint_graph(cones)
    for m in midpoints:
        classes = {cones[m.a].cone_class, cones[m.b].cone_class}
        assert not (classes & {CLASS_ORANGE, CLASS_MAGENTA})


def test_unlabeled_clusters_never_shape_the_centerline():
    """An unlabelled cluster could be either wall; guessing is worse than ignoring."""
    plain = centerline(straight())
    noisy = centerline(straight() + [
        LabeledCone(UNLABELED, 0.0, 2.0, 0.1),
        LabeledCone(UNLABELED, 0.0, 3.0, -0.2),
    ])
    assert plain.points == noisy.points


# --- forks --------------------------------------------------------------

def test_a_fork_is_visible_as_a_branch_in_the_graph():
    cones = track_at(2.0, max_range=6.0)
    midpoints, adjacency = midpoint_graph(cones)
    assert any(len(neighbours) >= 3 for neighbours in adjacency.values()), (
        "no midpoint branches, so this field does not contain a fork")


def test_the_chain_does_not_cross_the_island():
    """The failure a boundary walk makes: a midpoint pairing two corridors.

    Every point on the emitted line must be inside the corridor, which for the
    track means within half a corridor width of SOME real pair -- not floating
    in the island between the two branches.
    """
    cones = track_at(2.0, max_range=6.0)
    result = centerline(cones)
    assert len(result.points) >= 3
    for x, y in result.points:
        nearest = min(math.hypot(x - c.x, y - c.y) for c in cones)
        assert nearest < 1.1, f"({x:.2f}, {y:.2f}) is not near any cone"


def test_the_chain_moves_steadily_away_from_the_car():
    """The DAG constraint: no doubling back down the corridor just travelled."""
    result = centerline(track_at(2.0, max_range=6.0))
    distances = [math.hypot(x, y) for x, y in result.points]
    assert distances == sorted(distances)


def test_the_longer_branch_wins():
    """At J1 the route branch is 3.5 m and the dead-end stub is 1.5 m."""
    result = centerline(track_at(2.0, max_range=7.0))
    end = result.points[-1]
    # The route turns left (+25 deg), the dead end right. In the car frame at
    # x=2 on corridor A, left is +y.
    assert end[1] > 0.0, "the chain took the short dead-end branch"


# --- degenerate cases ---------------------------------------------------

def test_a_single_visible_wall_falls_back_to_an_offset():
    cones = [c for c in straight() if c.cone_class == CLASS_BLUE]
    result = centerline(cones)
    assert result.single_boundary_fallback
    assert result.points
    # Blue is the LEFT wall, so the corridor lies to its right.
    for (x, y), cone in zip(result.points, cones):
        assert y < cone.y


def test_the_yellow_only_fallback_offsets_the_other_way():
    cones = [c for c in straight() if c.cone_class == CLASS_YELLOW]
    result = centerline(cones)
    assert result.single_boundary_fallback
    for (x, y), cone in zip(result.points, cones):
        assert y > cone.y


def test_the_fallback_sits_half_a_corridor_from_the_wall():
    cones = [c for c in straight() if c.cone_class == CLASS_BLUE]
    result = centerline(cones)
    for (x, y), cone in zip(result.points, cones):
        assert math.hypot(x - cone.x, y - cone.y) == pytest.approx(0.75, abs=0.01)


def test_no_cones_is_not_an_error():
    result = centerline([])
    assert result.points == []
    assert result.midpoints == []


def test_one_wall_of_a_single_cone_cannot_be_fitted():
    result = centerline([LabeledCone(CLASS_BLUE, 0.9, 1.0, 0.75)])
    assert result.points == []
    assert not result.single_boundary_fallback


def test_the_full_track_from_the_start_line_produces_a_usable_line():
    result = centerline(track_at(0.0, max_range=5.0))
    assert len(result.points) >= 3
    assert not result.single_boundary_fallback


# --- boundary_split -----------------------------------------------------

def test_split_buckets_by_class():
    cones = straight(3.0) + [
        LabeledCone(CLASS_RED, 0.9, 2.0, 0.75),
        LabeledCone(CLASS_ORANGE, 0.9, 2.5, 0.0),
        LabeledCone(CLASS_MAGENTA, 0.9, 2.8, 0.0),
        LabeledCone(UNLABELED, 0.0, 1.0, 0.2),
    ]
    got = boundary_split.split(cones)
    assert got.left and got.right
    assert len(got.gates) == 1
    assert len(got.dead_ends) == 1
    assert len(got.goal) == 1
    assert len(got.unlabeled) == 1
    assert all(c.cone_class == CLASS_BLUE for c in got.left)
    assert all(c.cone_class == CLASS_YELLOW for c in got.right)


def test_split_drops_what_is_behind_the_car():
    cones = [LabeledCone(CLASS_BLUE, 0.9, -2.0, 0.75),
             LabeledCone(CLASS_BLUE, 0.9, 1.0, 0.75)]
    assert len(boundary_split.split(cones).left) == 1


def test_split_keeps_a_cone_level_with_the_car():
    """A cone at the front axle is still a wall of the corridor being driven."""
    cones = [LabeledCone(CLASS_BLUE, 0.9, -0.2, 0.75)]
    assert len(boundary_split.split(cones).left) == 1


def test_split_drops_what_is_out_of_lidar_range():
    cones = [LabeledCone(CLASS_BLUE, 0.9, 20.0, 0.75)]
    assert boundary_split.split(cones).left == []


def test_split_orders_each_group_by_range():
    cones = [LabeledCone(CLASS_BLUE, 0.9, 3.0, 0.75),
             LabeledCone(CLASS_BLUE, 0.9, 1.0, 0.75)]
    assert [c.x for c in boundary_split.split(cones).left] == [1.0, 3.0]


# --- invariants that only a fork exercises ------------------------------

def test_the_half_width_survives_a_fork_in_view():
    """Diagonal pairings and the junction's wide pairs must not move it."""
    for x in (0.0, 2.0, 3.0):
        result = centerline(track_at(x, max_range=6.0))
        assert result.corridor_half_width == pytest.approx(0.75, abs=0.05), (
            f"half-width wrong with the car at x={x}")


def test_the_island_nose_pair_is_not_a_corridor_midpoint():
    """The two branches' inner walls start side by side, ~0.3 m apart.

    Pairing them puts a midpoint on the island tip and invites the car to drive
    straight at it.
    """
    cones = track_at(2.0, max_range=6.0)
    midpoints, _ = midpoint_graph(cones)
    assert all(m.width_m >= 0.6 for m in midpoints)


def test_the_graph_stays_connected_through_a_fork():
    """Shared-cone adjacency, not shared-triangle. The reason for the switch.

    With shared-triangle the branches formed components disconnected from the
    corridor behind them, so the car's own branch became unreachable and the
    chain fell through to the single-boundary fallback.
    """
    cones = track_at(2.0, max_range=6.0)
    midpoints, adjacency = midpoint_graph(cones)
    # Forward midpoints only. The ones behind the car are their own component
    # by construction -- the car has driven past them -- and counting them here
    # would measure the wrong thing.
    corridor = [i for i, m in enumerate(midpoints)
                if m.kind == CORRIDOR and m.x > 0.0]
    start = min(corridor, key=lambda i: math.hypot(midpoints[i].x, midpoints[i].y))

    forward = set(corridor)
    seen, stack = {start}, [start]
    while stack:
        for nxt in adjacency[stack.pop()]:
            if nxt in forward and nxt not in seen:
                seen.add(nxt)
                stack.append(nxt)
    assert len(seen) > len(corridor) / 2, (
        "the component reachable from the car covers less than half the "
        "midpoints, so the fork has split the graph")


def test_a_dead_end_shorter_than_the_island_nose_is_flagged_by_geometry():
    """track_v1.md's 1.5 m stub cannot separate from the through-branch.

    Not an assertion about our code -- an assertion about the track, kept here
    because it is the arithmetic that justifies sim's departure from the
    document. See cone_field.island_nose_distance.
    """
    assert cone_field.island_nose_distance() > 1.5


def test_the_document_track_still_produces_a_line():
    """Even with the too-short stub, the corridor layer must not fall over."""
    layout = cone_field.track_v1(dead_end_length_m=1.5)
    cones = labeled(cone_field.cones_in_car_frame(
        layout, cone_field.Pose(2.0, 0.0, 0.0), max_range_m=6.0))
    result = centerline(cones)
    assert len(result.points) >= 3
