"""Closed-loop goal runs: the car recognises the trophy and stops at it.

`sim/` is not in pytest.ini's testpaths, so these run with `pytest sim`.

The load-bearing test here is `test_the_stop_needs_the_detector`: without it
every other run in this file would pass with the goal code deleted. The corridor
ends at the trophy, so `speed_ctrl`'s reach floor already halts the car about
0.8 m away, and "stopped near the goal" is free. Driving the SAME track with
magenta dropped from the detector is what shows the stop is being DECIDED rather
than stumbled into -- the same trick `test_the_route_is_what_picks_the_branch`
plays on the junction.
"""

import math

import pytest

from cone_nav.guidance.goal_stop import STOP_RANGE_M
from cone_perception import extrinsics

from sim import cone_field
from sim.drive_sim import DT_S, simulate

WHEELBASE = extrinsics.WHEELBASE_M
AXLE = extrinsics.REAR_AXLE_IN_BASE
OTHER = {"left": "right", "right": "left"}

# One tick of travel at the cogging floor: the stop fires on the tick that sees
# the goal inside STOP_RANGE_M, and the car completes that tick's step before
# the run ends. Anything inside this of the commanded range is on target.
TICK_TRAVEL_M = 0.05 * 7.5 * DT_S


def run(turn, route=None, **kw):
    layout = cone_field.track_junction(turn)
    return layout, simulate(layout, WHEELBASE, AXLE, lookahead_m=1.0,
                            route=[route or turn], **kw)


def goal_of(layout):
    return next(c for c in layout if c.color == "magenta")


def nose_gap(layout, result):
    """Distance from base_link -- the lidar, and the front of the car -- to the
    trophy at the end of the run.

    Reconstructed from the axle pose because that is what `Vehicle` tracks, and
    because the stop range is quoted in base_link: measuring from the axle would
    read 0.36 m long and quietly pass a car that had driven over the trophy.
    """
    last = result.ticks[-1]
    heading = math.radians(last.heading_deg)
    nose = (last.x - AXLE[0] * math.cos(heading),
            last.y - AXLE[0] * math.sin(heading))
    goal = goal_of(layout)
    return math.hypot(goal.x - nose[0], goal.y - nose[1])


# --- the arrival --------------------------------------------------------

@pytest.mark.parametrize("turn", ["left", "right"])
def test_the_car_stops_at_the_goal(turn):
    _layout, result = run(turn)
    assert result.stopped_at_goal, result.outcome
    assert result.struck_cone is None


@pytest.mark.parametrize("turn", ["left", "right"])
def test_it_stops_at_the_commanded_distance(turn):
    layout, result = run(turn)
    assert nose_gap(layout, result) == pytest.approx(STOP_RANGE_M,
                                                     abs=TICK_TRAVEL_M)


@pytest.mark.parametrize("turn", ["left", "right"])
def test_it_does_not_stop_so_close_that_the_trophy_stops_being_a_cluster(turn):
    """`clustering.MIN_CONE_RANGE_M` discards a return inside 0.20 m as the
    chassis arc leaking. Stopping inside that would mean the trigger fired on a
    goal the car could no longer see."""
    from cone_perception.clustering import MIN_CONE_RANGE_M
    layout, result = run(turn)
    assert nose_gap(layout, result) > MIN_CONE_RANGE_M


@pytest.mark.parametrize("turn", ["left", "right"])
def test_the_stop_is_reached_with_the_goal_in_hand_not_on_dead_reckoning(turn):
    """The blind budget exists for a flicker, not as the normal path. A run that
    arrives on a carried estimate says so, and on a clean track it must not."""
    _layout, result = run(turn)
    assert result.ticks[-1].goal_state == "stopped"
    assert result.ticks[-1].goal_reason == ""


# --- the control: is the stop actually being decided? -------------------

@pytest.mark.parametrize("turn", ["left", "right"])
def test_the_stop_needs_the_detector(turn):
    """The test this file exists for. With magenta dropped the car still ends up
    near the trophy -- the corridor runs out there and the reach floor stops it
    -- but it must not CLAIM to have reached the goal."""
    _layout, result = run(turn, dropout=("magenta",))
    assert not result.stopped_at_goal
    assert result.ticks[-1].goal_state == "seeking"


@pytest.mark.parametrize("turn", ["left", "right"])
def test_a_goal_that_is_never_detected_leaves_the_car_further_out(turn):
    """And it stops further away than the goal stop would, which is the
    difference the feature buys: ~0.8 m of coasting-to-a-halt against 0.30 m of
    arriving."""
    layout, detected = run(turn)
    layout, blind = run(turn, dropout=("magenta",))
    assert nose_gap(layout, blind) > nose_gap(layout, detected) + 0.3


# --- the arming guard ---------------------------------------------------

@pytest.mark.parametrize("turn", ["left", "right"])
def test_the_goal_never_arms_while_a_turn_is_outstanding(turn):
    """The guard that makes a red-gate-read-as-magenta harmless. Driving the
    wrong route never consumes the junction, so the route never empties and the
    stop stays disarmed for the whole run."""
    _layout, result = run(turn, route=OTHER[turn])
    assert not result.stopped_at_goal
    assert all(t.goal_state == "seeking" for t in result.ticks)


@pytest.mark.parametrize("turn", ["left", "right"])
def test_it_does_not_stop_during_the_junction_manoeuvre(turn):
    """Nothing seen at the fork may stop the car: the goal lies past it, and a
    stop in the mouth is a run ended mid-course."""
    _layout, result = run(turn)
    engaged = [t for t in result.ticks if t.topo in ("approach", "traverse")]
    assert engaged, "the machine never engaged the junction"
    assert all(t.goal_state == "seeking" for t in engaged)


# --- the run-in ---------------------------------------------------------

@pytest.mark.parametrize("turn", ["left", "right"])
def test_the_car_is_still_moving_when_it_reaches_the_stop(turn):
    """The failure this whole run-in exists to prevent: the reach floor halting
    the car at 0.64 m from the trophy, where it cannot restart because the scan
    does not change while it stands still."""
    _layout, result = run(turn)
    moving = [t for t in result.ticks if t.goal_state == "run_in"
              and t.goal_range_m > STOP_RANGE_M]
    assert moving, "the run-in never happened"
    assert all(t.duty > 0.0 for t in moving), \
        [(round(t.goal_range_m, 2), t.reason) for t in moving if t.duty <= 0.0]


@pytest.mark.parametrize("turn", ["left", "right"])
def test_the_goal_range_closes_monotonically(turn):
    """A range that jumps is a latch that has re-bound onto something else."""
    _layout, result = run(turn)
    ranges = [t.goal_range_m for t in result.ticks
              if t.goal_state in ("run_in", "stopped")]
    for earlier, later in zip(ranges, ranges[1:]):
        assert later <= earlier + 0.02
