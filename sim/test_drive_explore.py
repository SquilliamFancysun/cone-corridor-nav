"""Closed-loop exploration: the car chooses a branch with no route, and names
a wall when it meets one.

`sim/` is not in pytest.ini's testpaths, so these run with `pytest sim`.

The load-bearing pair here is `test_it_reaches_the_goal_with_no_route_file`
against `test_a_blocked_branch_is_reported_as_a_dead_end`. Either alone proves
little: reaching the goal could be the route code with a policy bolted on, and
stopping at a wall is what `speed_ctrl`'s reach floor already does for free. It
is the two together -- the SAME policy on two layouts that differ only in which
branch is walled -- that show a decision is being made.

`test_the_search_takes_the_branch_it_has_not_tried` is what separates this from
a car that merely stops: the value it asserts is the one that has to survive
into the next run.
"""

import pytest

from cone_nav.guidance.explore import ExplorePolicy
from cone_perception import extrinsics

from sim.drive_sim import build_track, simulate

WHEELBASE = extrinsics.WHEELBASE_M
AXLE = extrinsics.REAR_AXLE_IN_BASE
OTHER = {"left": "right", "right": "left"}


def explore(track, first="left", **kw):
    policy = ExplorePolicy(first=first)
    result = simulate(build_track(track), WHEELBASE, AXLE,
                      cursor=policy, max_time_s=40.0, **kw)
    return policy, result


# --- the open case ------------------------------------------------------

@pytest.mark.parametrize("turn", ["left", "right"])
def test_it_reaches_the_goal_with_no_route_file(turn):
    """No route anywhere in this run. The policy picks the branch, the
    existing TopoState drives it, and the goal latch stops the car."""
    policy, result = explore(f"junction-{turn}", first=turn)
    assert result.stopped_at_goal, result.outcome
    assert policy.path == [turn]
    assert policy.dead_ends == 0


def test_a_clean_run_never_declares_a_dead_end():
    """The false positive that would matter most: a corridor the car is
    driving perfectly well, called a wall."""
    _, result = explore("junction-left", first="left")
    assert "dead end" not in result.outcome


def test_the_driving_is_no_worse_without_a_route():
    """Swapping the cursor must not touch the control loop. Same track, same
    gains, same cross-track -- the seam is above all of that."""
    _, explored = explore("junction-left", first="left")
    routed = simulate(build_track("junction-left"), WHEELBASE, AXLE,
                      route=["left"], max_time_s=40.0)
    assert explored.mean_xtrack_m == pytest.approx(routed.mean_xtrack_m,
                                                   abs=0.01)


# --- the walled case ----------------------------------------------------

@pytest.mark.parametrize("turn", ["left", "right"])
def test_a_blocked_branch_is_reported_as_a_dead_end(turn):
    """The mirror layout: the branch the policy picks is the walled stub. The
    car drove correctly and still arrived at a wall, which is the only honest
    way to test the detector."""
    _, result = explore(f"junction-{turn}-blocked", first=turn)
    assert result.outcome.startswith("dead end"), result.outcome
    assert not result.stopped_at_goal


@pytest.mark.parametrize("turn", ["left", "right"])
def test_the_search_takes_the_branch_it_has_not_tried(turn):
    """The value that has to survive into the next run. A policy that detected
    the wall and then offered the same branch again would loop forever."""
    policy, _ = explore(f"junction-{turn}-blocked", first=turn)
    assert policy.dead_ends == 1
    assert policy.current == OTHER[turn]
    assert policy.path == []


def test_the_dead_end_names_the_orange_wall_when_it_can_see_one():
    """The sim's classifier is perfect, so the corroborated path is the one
    taken here. On the car orange recall is 0.687 and the geometry-only path
    carries runs where it is missed -- see cone_nav/topology/dead_end.py."""
    _, result = explore("junction-left-blocked", first="left")
    assert "orange wall seen" in result.outcome


def test_the_wall_is_found_at_about_a_metre():
    """Where the reach floor bites. Worth pinning as a number: a layout change
    that moved it far from MIN_REACH_M would mean the latch and the stop had
    stopped coinciding, and the car would sit at a wall it did not name."""
    _, result = explore("junction-left-blocked", first="left")
    reach = float(result.outcome.split("ends")[1].split("m")[0])
    assert 0.7 < reach < 1.0
