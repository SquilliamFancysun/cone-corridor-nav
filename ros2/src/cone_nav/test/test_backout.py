"""The manoeuvre that replaces the operator's hands at a dead end.

Two failures would matter on the track and neither is a wrong steering angle.
The first is arriving in the junction MOUTH -- the car reverses through the
junction and spends several ticks a metre from all three reds, and a manoeuvre
that stops there hands `topo_state` a car pointing down the branch it just left.
The second is not stopping at all, in the one direction the car cannot see.

So most of what follows is about where the manoeuvre ENDS, not how it steers.
"""

import math

import pytest

from cone_nav.control import speed_ctrl
from cone_nav.guidance.backout import (
    ABANDONED,
    ARRIVED,
    BACKING,
    IDLE,
    MOUTH_CLEAR_RANGE_M,
    BackoutManoeuvre,
)


class FakeLine(object):
    """A corridor running straight ahead, optionally offset and angled."""

    def __init__(self, points=None):
        self.points = [(1.0, 0.0), (2.0, 0.0)] if points is None else points
        self.single_boundary_fallback = False


class FakeJunction(object):
    """Only `range_for` is read, and only for the turn being resumed."""

    def __init__(self, range_m, turn="right"):
        self._range = range_m
        self._turn = turn

    def range_for(self, turn):
        if turn not in ("left", "right"):
            raise ValueError(turn)
        return self._range if turn == self._turn else self._range


def backing(commit_range_m=2.0, budget_m=2.0, turn="right", **kw):
    m = BackoutManoeuvre(**kw)
    m.begin(turn, budget_m=budget_m, commit_range_m=commit_range_m)
    return m


def run(m, ticks, line=None, junction=None, travel_m=0.04, armed=True):
    line = FakeLine() if line is None else line
    for _ in range(ticks):
        if m.state != BACKING:
            break
        m.update(line, junction, travel_m=travel_m, armed=armed)
    return m.state


# --- beginning ----------------------------------------------------------

def test_it_refuses_to_back_out_with_nowhere_to_go():
    """`ExplorePolicy.dead_end()` returns None when the search is spent. There
    is no junction behind the car to return to, so reversing is motion for its
    own sake in the direction with no lidar."""
    m = BackoutManoeuvre()
    assert m.begin(None, budget_m=2.0, commit_range_m=2.0) == IDLE
    assert not m.active
    assert "nothing left" in m.reason


def test_beginning_commands_a_reverse():
    m = backing()
    m.update(FakeLine(), None, travel_m=0.0)
    assert m.duty < 0.0
    assert m.duty == pytest.approx(-speed_ctrl.MAX_REVERSE_DUTY)


# --- where it stops -----------------------------------------------------

def test_it_stops_on_the_first_whole_triple_clear_of_the_mouth():
    m = backing(commit_range_m=2.0)
    assert run(m, 50, junction=FakeJunction(2.4)) == ARRIVED
    assert m.duty == 0.0
    assert "2.40 m" in m.reason


def test_a_sighting_short_of_the_commit_range_is_still_an_arrival():
    """The regression the sim found, and the reason the commit range is not the
    arrival test. Driving in, the car commits at the FAR edge of the band a
    triple can be recovered from; backing out it enters at the NEAR edge and
    the reds leave arm range again before it climbs that far. Measured on
    junction-left-blocked: a ten-tick window from 2.24 m against a commit range
    near 2.7, which the car reversed straight through and off the corridor."""
    m = backing(commit_range_m=2.7)
    assert run(m, 50, junction=FakeJunction(2.24)) == ARRIVED


def test_a_junction_seen_from_inside_the_mouth_is_not_an_arrival():
    """The failure this range floor exists for. Backing out, the car passes
    THROUGH the junction and sees all three reds from close range, pointing
    down the branch it just left. Stopping there is worse than not stopping."""
    m = backing(commit_range_m=2.0, budget_m=2.0)
    assert run(m, 500, junction=FakeJunction(0.9)) == ABANDONED
    assert "without seeing the junction" in m.reason


def test_one_tick_of_a_gate_is_not_a_sighting():
    """`gate_detect` recovers whole triples on a fraction of ticks; a single
    frame at range must not end the manoeuvre on its own."""
    m = backing(commit_range_m=2.0)
    m.update(FakeLine(), FakeJunction(2.4), travel_m=0.04)
    assert m.state == BACKING and m.confirm == 1


def test_a_blank_tick_does_not_wipe_the_count():
    """The window, and the whole reason for it. Sweeping the band a triple can
    be recovered from, it comes back on roughly one tick in three -- so a rule
    that reset on every blank asks for a run that does not occur, and the car
    reverses through a good re-approach pose and off the end of its bound.
    `dead_end.py` makes the same argument at length; this got it wrong the same
    way first."""
    m = backing(commit_range_m=2.0)
    m.update(FakeLine(), FakeJunction(2.4), travel_m=0.04)
    m.update(FakeLine(), None, travel_m=0.04)
    m.update(FakeLine(), None, travel_m=0.04)
    assert m.confirm == 1, "a blank tick wiped the evidence"
    assert m.state == BACKING
    m.update(FakeLine(), FakeJunction(2.4), travel_m=0.04)
    assert m.state == ARRIVED


def test_the_window_forgets_a_sighting_that_has_scrolled_out_of_it():
    """It is a window, not a running total: two triples a hundred ticks apart
    are two different places, not evidence about one."""
    m = backing(commit_range_m=2.0, budget_m=100.0)
    m.update(FakeLine(), FakeJunction(2.4), travel_m=0.04)
    assert m.confirm == 1
    for _ in range(m.arrive_window_ticks):
        m.update(FakeLine(), None, travel_m=0.04)
    assert m.confirm == 0
    assert m.state == BACKING


def test_it_arrives_on_a_tick_it_can_actually_see_the_gate():
    """So the range it stops on and reports is measured this tick rather than
    carried from wherever the car was a second ago."""
    m = backing(commit_range_m=2.0)
    m.update(FakeLine(), FakeJunction(2.4), travel_m=0.04)
    m.update(FakeLine(), None, travel_m=0.04)
    assert m.state == BACKING
    m.update(FakeLine(), FakeJunction(2.51), travel_m=0.04)
    assert m.state == ARRIVED
    assert m.gate_range_m == pytest.approx(2.51)


def test_the_mouth_floor_does_not_depend_on_having_committed():
    """A traverse that timed out rather than passing leaves no commit range.
    The mouth still has to be rejected, so the floor is a constant rather than
    anything derived from the drive in."""
    m = backing(commit_range_m=0.0)
    assert m.arrive_range_m == MOUTH_CLEAR_RANGE_M
    assert run(m, 500, junction=FakeJunction(1.0)) == ABANDONED


def test_the_commit_range_still_sizes_the_bound():
    """Where it is genuinely useful: how much further than the recorded edge
    the car has to travel."""
    near = backing(commit_range_m=1.0, budget_m=2.0)
    far = backing(commit_range_m=3.0, budget_m=2.0)
    assert far.bound_m > near.bound_m


# --- the bound ----------------------------------------------------------

def test_it_gives_up_rather_than_reversing_off_the_track():
    """The terminating condition is a perception event and the car is blind
    behind. Abandoning is today's behaviour, which works."""
    m = backing(commit_range_m=2.0, budget_m=1.0)
    assert run(m, 500, junction=None) == ABANDONED
    assert m.travelled_m >= m.bound_m
    assert m.duty == 0.0


def test_the_bound_covers_the_edge_the_commit_range_and_slack():
    """The recorded edge starts at the `passed` snapshot, not at the junction,
    so backing out an edge length alone lands the car short."""
    m = backing(commit_range_m=2.0, budget_m=3.0)
    assert m.bound_m > 3.0 + 2.0


def test_travel_counts_regardless_of_sign():
    """Reverse travel arrives as a negative `odo_forward_m`, and a bound that
    counted it signed would never be reached."""
    m = backing(budget_m=1.0, commit_range_m=0.5)
    run(m, 500, travel_m=-0.04)
    assert m.state == ABANDONED
    assert m.travelled_m > 0.0


def test_it_times_out():
    m = backing(budget_m=1e6, commit_range_m=2.0, max_ticks=10)
    assert run(m, 500, travel_m=0.0) == ABANDONED
    assert "timed out" in m.reason


# --- steering -----------------------------------------------------------

def test_it_will_not_command_a_straight_reverse_with_no_reference():
    """`reverse_ctrl.corridor_error` returns None on a line too short to have a
    direction, and its docstring is explicit that zero must not be substituted:
    zero error commands a straight reverse, which is exactly what a car with an
    unknown heading must not do. So it holds briefly and then stops."""
    m = backing(budget_m=100.0, commit_range_m=2.0, max_blind_ticks=3)
    assert run(m, 50, line=FakeLine(points=[])) == ABANDONED
    assert "no corridor to steer on" in m.reason


def test_a_blind_tick_holds_the_last_steer_rather_than_straightening():
    m = backing(max_blind_ticks=3)
    m.update(FakeLine(points=[(1.0, 0.4), (2.0, 0.4)]), None, travel_m=0.04)
    held = m.steer_normalised
    assert held != 0.0
    m.update(FakeLine(points=[]), None, travel_m=0.04)
    assert m.steer_normalised == held


def test_the_blind_count_resets_on_a_good_tick():
    m = backing(max_blind_ticks=3)
    for _ in range(3):
        m.update(FakeLine(points=[]), None, travel_m=0.04)
    assert m.blind_ticks == 3 and m.state == BACKING
    m.update(FakeLine(), None, travel_m=0.04)
    assert m.blind_ticks == 0


def test_a_car_left_of_the_centreline_steers_right_in_reverse():
    """The sign result `reverse_ctrl` warns about, exercised through the
    manoeuvre so a caller that inverted an argument on the way in is caught
    here rather than on the track. Left of centre is a corridor whose line
    lies to the car's RIGHT."""
    m = backing()
    m.update(FakeLine(points=[(1.0, -0.5), (2.0, -0.5)]), None, travel_m=0.04)
    assert m.cross_track_m > 0.0
    assert m.steer_normalised < 0.0


def test_a_heading_error_alone_flips_relative_to_forward():
    """The half of the folk rule that is right. With no offset, an axis
    pointing left of the car takes right steer in reverse."""
    m = backing()
    ahead = math.tan(math.radians(15.0))
    m.update(FakeLine(points=[(0.0, 0.0), (1.0, ahead)]), None, travel_m=0.04)
    assert m.heading_err_rad > 0.0
    assert m.cross_track_m == pytest.approx(0.0, abs=1e-9)
    assert m.steer_normalised < 0.0


# --- the deadman --------------------------------------------------------

def test_releasing_the_deadman_abandons_rather_than_pausing():
    """X keeps one meaning: stop, something is wrong. A manoeuvre that resumed
    on the next press would reverse the car while the operator was reaching
    for it."""
    m = backing()
    assert run(m, 5, armed=False) == ABANDONED
    assert "released" in m.reason
    assert m.duty == 0.0


def test_an_abandoned_manoeuvre_stays_abandoned():
    m = backing()
    run(m, 5, armed=False)
    m.update(FakeLine(), FakeJunction(2.4), travel_m=0.04, armed=True)
    assert m.state == ABANDONED


def test_release_returns_it_to_idle_for_the_next_dead_end():
    m = backing()
    run(m, 50, junction=FakeJunction(2.4))
    m.release()
    assert m.state == IDLE and m.travelled_m == 0.0
    assert m.begin("left", budget_m=1.0, commit_range_m=2.0) == BACKING
