"""The car as its own reverse: a dead end that the run survives.

`sim/` is not in pytest.ini's testpaths, so these run with `pytest sim`.

`test_drive_explore.py` proves the SEARCH is right -- the car picks a branch,
meets a wall, and names the branch it has not tried. Everything after that was
done by a person picking the car up (`docs/junction-bringup.md` stage 7b). These
close that gap, and the load-bearing test is
`test_it_finishes_a_blocked_maze_with_nobody_touching_it`: the same policy, the
same two mirror layouts, and the goal reached with no intervention at all.

Every test here runs `blind_rear=True`. Reversing is the one direction the car
cannot see, and validating it against the sim's default 360 deg lidar would be
optimistic in exactly the way that matters.
"""

import pytest

from cone_nav.guidance.explore import ExplorePolicy
from cone_perception import extrinsics

from sim.drive_sim import build_track, simulate

WHEELBASE = extrinsics.WHEELBASE_M
AXLE = extrinsics.REAR_AXLE_IN_BASE
OTHER = {"left": "right", "right": "left"}


def explore(track, first="left", reverse=True, **kw):
    policy = ExplorePolicy(first=first)
    result = simulate(build_track(track), WHEELBASE, AXLE,
                      cursor=policy, max_time_s=120.0, reverse=reverse,
                      blind_rear=True, **kw)
    return policy, result


# --- the whole feature --------------------------------------------------

@pytest.mark.parametrize("turn", ["left", "right"])
def test_it_finishes_a_blocked_maze_with_nobody_touching_it(turn):
    """The point of all of it. The policy takes the walled branch, the car
    meets the wall, backs ITSELF out to the junction, and takes the branch it
    has not tried -- with no operator and no carry."""
    policy, result = explore(f"junction-{turn}-blocked", first=turn)
    assert result.stopped_at_goal, result.outcome
    assert policy.dead_ends == 1
    assert policy.path == [OTHER[turn]]


@pytest.mark.parametrize("turn", ["left", "right"])
def test_without_a_reverse_the_same_run_ends_at_the_wall(turn):
    """The control. Same track, same policy, `reverse` off: the run stops where
    stage 7b has a person pick the car up. That fallback is still the behaviour
    on the track until the manoeuvre is proven there."""
    policy, result = explore(f"junction-{turn}-blocked", first=turn,
                             reverse=False)
    assert result.outcome.startswith("dead end"), result.outcome
    assert not result.stopped_at_goal
    assert policy.path == []


def test_the_backout_does_not_consume_the_branch_it_is_going_back_for():
    """`topo_state` is held down through the manoeuvre for this. Reversing
    takes the car back THROUGH the junction, which rises into view ahead of a
    car pointing the wrong way down it -- and an approach armed on that
    sighting would commit a traverse on the very branch the search is backing
    out to try."""
    policy, _ = explore("junction-left-blocked", first="left")
    assert policy.path == ["right"]
    assert policy.index == 1


# --- the false positives ------------------------------------------------

@pytest.mark.parametrize("turn", ["left", "right"])
def test_a_clean_run_never_reverses(turn):
    """The expensive failure: a car backing out of a corridor it was driving
    perfectly well. Nothing about arming the manoeuvre may make a dead end
    easier to declare."""
    policy, result = explore(f"junction-{turn}", first=turn)
    assert result.stopped_at_goal, result.outcome
    assert policy.dead_ends == 0
    assert "backout" not in result.outcome


def test_arming_the_reverse_does_not_change_how_the_car_drives_forward():
    """The manoeuvre is a branch off the control loop, not a change to it. A
    track with no wall in it must drive identically either way."""
    _, with_reverse = explore("junction-left", first="left", reverse=True)
    _, without = explore("junction-left", first="left", reverse=False)
    assert with_reverse.mean_xtrack_m == pytest.approx(without.mean_xtrack_m,
                                                       abs=1e-9)
    assert len(with_reverse.ticks) == len(without.ticks)


# --- what the reverse itself did ----------------------------------------

def test_it_backs_out_far_enough_to_see_the_junction_and_no_further():
    """The bound is a safety net, never the thing that ends a healthy
    manoeuvre. A run that finishes on the bound rather than on a sighting
    reports `backout failed`, so reaching the goal is already proof -- this
    pins the margin as well, so a track change that eats it is a failing test
    rather than a puzzling afternoon."""
    _, result = explore("junction-left-blocked", first="left")
    assert result.stopped_at_goal
    assert "backed out" not in result.outcome


def test_the_car_does_not_strike_a_cone_while_reversing():
    """It is reversing into an arc it cannot see, over ground it drove forward
    a moment ago. The sim's strike test runs through the manoeuvre like any
    other tick."""
    for turn in ("left", "right"):
        _, result = explore(f"junction-{turn}-blocked", first=turn)
        assert result.struck_cone is None, result.outcome
