"""The route entry is the thing being protected.

Consuming one when the car did not actually pass a gate means every later
junction gets the wrong turn, and on the track that looks exactly like a
detector fault. So most of these tests are about a route entry NOT being
consumed -- and about the travel floor, which is what makes that possible when
the reds vanish two metres short of the mouth.
"""

import math

import pytest

from cone_nav.guidance.route_exec import RouteCursor
from cone_nav.topology.topo_state import (
    APPROACH,
    CLEAR_PAST_GATE_M,
    COMMIT_WINDOW_TICKS,
    FOLLOW,
    MAX_TRAVERSE_TICKS,
    MAX_ANCHOR_BLIND_TICKS,
    MIN_REACQUIRE_POINTS,
    PASSED_CONFIRM_TICKS,
    TRAVERSE,
    TopoState,
    corridor_reacquired,
)

GATE_RANGE_M = 2.5


class FakeCone(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y


class FakeJunction(object):
    """Only what topo_state touches: a centre cone, an axis, a gate range, and
    the gate midpoint for each turn -- which it carries forward as the anchor."""

    def __init__(self, gate_range=GATE_RANGE_M, axis_rad=0.0, gap=0.7):
        self.gate_range = gate_range
        self.centre = FakeCone(gate_range, 0.0)
        self.axis_rad = axis_rad
        self.gap = gap

    def range_for(self, _turn):
        return self.gate_range

    def gate_for(self, turn):
        # Midway between the centre cone and the outer red on that side, which
        # is what gate_detect.Junction returns.
        side = +1.0 if turn == "left" else -1.0
        return (self.gate_range, side * self.gap / 2.0)


class FakeLine(object):
    def __init__(self, points=3, fallback=False):
        self.points = [(float(i), 0.0) for i in range(points)]
        self.single_boundary_fallback = fallback


GOOD = FakeLine(points=MIN_REACQUIRE_POINTS)
SHORT = FakeLine(points=MIN_REACQUIRE_POINTS - 1)
WOBBLY = FakeLine(points=MIN_REACQUIRE_POINTS, fallback=True)

# Enough travel per tick that a handful of ticks clears the floor.
STRIDE_M = 1.0
CLEAR_TICKS = int(math.ceil((GATE_RANGE_M + CLEAR_PAST_GATE_M) / STRIDE_M))


def machine(turns=("left", "right")):
    return TopoState(RouteCursor(list(turns)))


def commit(topo, junction=None):
    """Drive the machine into TRAVERSE the way the pipeline would."""
    junction = junction or FakeJunction()
    topo.update(junction, GOOD)
    topo.update(junction, GOOD)
    assert topo.state == TRAVERSE
    return junction


def pass_gate(topo, line=GOOD):
    """Drive until the manoeuvre ends, and no further -- `note` is a per-tick
    event, so an extra tick would clear the very thing under test."""
    for _ in range(CLEAR_TICKS + PASSED_CONFIRM_TICKS + 2):
        if topo.state == FOLLOW:
            return
        topo.update(None, line, travel_m=STRIDE_M)


# --- arming and committing ----------------------------------------------

def test_a_plain_corridor_stays_in_follow():
    topo = machine()
    assert topo.update(None, GOOD) == FOLLOW
    assert not topo.engaged


def test_a_junction_in_view_arms_the_machine():
    topo = machine()
    assert topo.update(FakeJunction(), GOOD) == APPROACH
    assert topo.engaged


def test_a_single_sighting_is_enough_to_commit():
    """Deliberate, and measured rather than chosen. On some layouts the whole
    approach yields ONE recovered triple; a machine that waits for a second
    drives past the fork. What rejects a bad commit is gate_detect's own
    three-cones-two-gaps test, not a count here."""
    topo = machine()
    junction = FakeJunction()
    topo.update(junction, GOOD)
    topo.update(None, GOOD)
    assert topo.state == TRAVERSE
    assert topo.latched is junction


def test_the_commit_window_is_wide_enough_to_hold_a_sighting():
    """At COMMIT_CONFIRM_TICKS = 1 the window never has to bridge a gap, so
    this only pins that the constants are consistent: raising the count is the
    documented knob, and it is useless if the window cannot hold that many."""
    assert COMMIT_WINDOW_TICKS >= 1


def test_losing_an_uncommitted_junction_costs_no_route_entry():
    topo = machine()
    topo.update(FakeJunction(), GOOD)
    for _ in range(COMMIT_WINDOW_TICKS + 1):
        topo.update(None, GOOD)
    assert topo.cursor.current == "left"


def test_a_junction_past_the_end_of_the_route_is_ignored():
    """A red triple glimpsed after the last turn must not restart the machine
    and must not walk the cursor into a turn that does not exist."""
    topo = machine(turns=["left"])
    commit(topo)
    pass_gate(topo)
    assert topo.cursor.exhausted
    for _ in range(COMMIT_WINDOW_TICKS + 2):
        assert topo.update(FakeJunction(), GOOD) == FOLLOW
    assert topo.latched is None


# --- the blind period ---------------------------------------------------

def test_the_latch_survives_the_reds_vanishing():
    """The outer reds leave the camera frame while the car is still ~2 m short
    of the mouth. The manoeuvre must not end there."""
    topo = machine()
    junction = commit(topo)
    assert topo.update(None, SHORT) == TRAVERSE
    assert topo.junction is junction
    assert topo.engaged


def test_a_healthy_corridor_alone_does_not_end_the_manoeuvre():
    """The bug this floor exists for. The corridor the car is still IN looks
    perfectly healthy all the way down the approach, so 'corridor reacquired'
    on its own ended the turn 2.2 m before the fork -- measured in the sim, on
    tick 9 of 65, after which the car followed the longer branch into the dead
    end."""
    topo = machine()
    commit(topo)
    for _ in range(PASSED_CONFIRM_TICKS * 3):
        topo.update(None, GOOD, travel_m=0.0)
    assert topo.state == TRAVERSE
    assert topo.cursor.current == "left"


def test_the_anchor_is_carried_when_the_triple_is_not_seen():
    """This is the rule that changed, and why. Refusing a latched anchor was
    right while nothing measured the car's motion -- a frozen point is wrong by
    however far it has driven. odometry.py made that false, and the refusal was
    costing the whole manoeuvre: measured 2026-09-02 (`explore-4.jsonl`), a
    186-tick traverse recovered the triple on 5 ticks, so for 181 of them there
    was no gate to steer at and the car drove straight into the divider."""
    topo = machine()
    commit(topo)
    assert topo.anchor_ok
    topo.update(None, SHORT, travel_m=0.1)
    assert topo.engaged
    assert topo.anchor_ok, "a blind tick must still have a gate to steer at"


def test_the_carried_anchor_moves_with_the_car():
    """Carried, not frozen. A metre driven puts the gate a metre nearer."""
    topo = machine()
    commit(topo)
    before = topo.anchor_xy[0]
    topo.update(None, SHORT, travel_m=1.0)
    assert topo.anchor_xy[0] == pytest.approx(before - 1.0)


def test_the_anchor_retires_once_it_is_behind_the_car():
    """Geometry ends the carry, not a timer. Past the gate the car is in the
    mouth and the exit corridor is the better guide -- which is what the old
    'no carried anchor' rule was really protecting."""
    topo = machine()
    commit(topo)
    topo.update(None, SHORT, travel_m=GATE_RANGE_M + 0.2)
    assert topo.engaged
    assert not topo.anchor_ok


def test_a_live_sighting_refreshes_the_carried_anchor():
    topo = machine()
    junction = commit(topo)
    topo.update(None, SHORT, travel_m=1.0)
    carried = topo.anchor_xy[0]
    topo.update(junction, GOOD)
    assert topo.anchor_xy[0] == pytest.approx(junction.gate_for("left")[0])
    assert topo.anchor_xy[0] != pytest.approx(carried)


def test_the_anchor_is_the_gate_the_route_asked_for():
    """Left and right gate midpoints sit either side of the divider, so
    carrying the wrong one steers the car into the branch it did not choose."""
    left = machine(("left",))
    commit(left)
    assert left.anchor_xy[1] > 0

    right = machine(("right",))
    commit(right)
    assert right.anchor_xy[1] < 0


def test_a_stalled_odometry_retires_the_anchor_on_ticks_instead():
    """The net under the geometric test: with no measured motion the anchor is
    never moved, so the x test can never retire it."""
    topo = machine()
    commit(topo)
    for _ in range(MAX_ANCHOR_BLIND_TICKS + 1):
        topo.update(None, SHORT, travel_m=0.0)
    assert topo.anchor_xy[0] > 0, "still ahead -- only the tick net can end it"
    assert not topo.anchor_ok


def test_a_finished_manoeuvre_drops_the_anchor():
    topo = machine()
    commit(topo)
    pass_gate(topo)
    assert topo.state == FOLLOW
    assert topo.anchor_xy is None
    assert not topo.anchor_ok


def test_blind_ticks_count_the_staleness():
    topo = machine()
    commit(topo)
    topo.update(None, SHORT)
    topo.update(None, SHORT)
    assert topo.blind_ticks == 2
    topo.update(FakeJunction(), SHORT)
    assert topo.blind_ticks == 0


# --- carrying the divider forward ---------------------------------------

def test_driving_forward_brings_the_divider_nearer():
    topo = machine()
    commit(topo, FakeJunction(gate_range=2.5))
    topo.update(None, SHORT, travel_m=1.0)
    assert topo.divider_xy[0] == pytest.approx(1.5)
    assert topo.divider_xy[1] == pytest.approx(0.0)


def test_turning_swings_the_divider_and_the_axis():
    topo = machine()
    commit(topo, FakeJunction(gate_range=2.0))
    topo.update(None, SHORT, travel_m=0.0, yaw_delta_rad=0.2)
    assert topo.divider_xy[0] == pytest.approx(2.0 * math.cos(0.2))
    assert topo.divider_xy[1] == pytest.approx(-2.0 * math.sin(0.2))
    assert topo.axis_rad == pytest.approx(-0.2)


def test_a_live_sighting_beats_a_carried_forward_estimate():
    topo = machine()
    commit(topo)
    topo.update(None, SHORT, travel_m=1.0)
    topo.update(FakeJunction(gate_range=0.8), SHORT, travel_m=1.0)
    assert topo.divider_xy[0] == pytest.approx(0.8)


def test_the_divider_is_dropped_when_the_manoeuvre_ends():
    topo = machine()
    commit(topo)
    pass_gate(topo)
    assert topo.divider_xy is None


# --- leaving the manoeuvre ----------------------------------------------

def test_clearing_the_gate_ends_the_manoeuvre_and_consumes_the_turn():
    topo = machine()
    commit(topo)
    pass_gate(topo)
    assert topo.state == FOLLOW
    assert topo.cursor.current == "right"
    assert topo.latched is None


def test_a_stalled_car_never_clears_the_gate():
    """The floor is a distance, so a car sitting still cannot pass a junction
    by waiting -- which a tick counter would have allowed."""
    topo = machine()
    commit(topo)
    for _ in range(MAX_TRAVERSE_TICKS - 1):
        topo.update(None, GOOD, travel_m=0.0)
    assert topo.cursor.current == "left"


def test_a_one_tick_dropout_does_not_consume_a_route_entry():
    topo = machine()
    commit(topo)
    for _ in range(CLEAR_TICKS + PASSED_CONFIRM_TICKS - 2):
        topo.update(None, GOOD, travel_m=STRIDE_M)
    assert topo.state == TRAVERSE, "the fixture must stop short of passing"
    topo.update(FakeJunction(), GOOD, travel_m=STRIDE_M)
    assert topo.state == TRAVERSE
    assert topo.cursor.current == "left"


def test_a_single_boundary_fallback_is_not_a_reacquired_corridor():
    """A fallback line is what the car produces while it is confused about one
    wall, which is exactly the state the mouth puts it in."""
    topo = machine()
    commit(topo)
    pass_gate(topo, line=WOBBLY)
    assert topo.state == TRAVERSE
    assert topo.cursor.current == "left"


def test_a_short_line_is_not_a_reacquired_corridor():
    topo = machine()
    commit(topo)
    pass_gate(topo, line=SHORT)
    assert topo.state == TRAVERSE


def test_the_turn_is_consumed_exactly_once():
    topo = machine()
    commit(topo)
    for _ in range(PASSED_CONFIRM_TICKS * 6):
        topo.update(None, GOOD, travel_m=STRIDE_M)
    assert topo.cursor.index == 1


def test_a_stalled_manoeuvre_gives_up_without_spending_the_turn():
    """If the corridor never comes back, the turn was not demonstrably taken.
    Keep the route entry and stop steering on a latch that is seconds old."""
    topo = machine()
    commit(topo)
    for _ in range(MAX_TRAVERSE_TICKS):
        topo.update(None, SHORT, travel_m=STRIDE_M)
    assert topo.state == FOLLOW
    assert topo.cursor.current == "left"
    assert "timed out" in topo.note


def test_the_note_reports_a_clean_pass():
    topo = machine()
    commit(topo)
    pass_gate(topo)
    assert topo.note == "passed"


def test_the_note_is_a_per_tick_event_not_a_status():
    """It is written into one trial-log row and cleared, so a reader counting
    'passed' rows counts gates rather than ticks."""
    topo = machine()
    commit(topo)
    pass_gate(topo)
    assert topo.note == "passed"
    topo.update(None, GOOD)
    assert topo.note == ""


# --- the reacquisition predicate ----------------------------------------

def test_corridor_reacquired_rejects_nothing_at_all():
    assert not corridor_reacquired(None)


def test_corridor_reacquired_accepts_a_real_corridor():
    assert corridor_reacquired(GOOD)


def test_a_second_junction_is_taken_with_the_second_turn():
    """The end-to-end point of the cursor: two junctions, two different turns."""
    topo = machine()
    commit(topo)
    pass_gate(topo)
    commit(topo)
    assert topo.latched_turn == "right"


def test_the_traverse_bound_is_sizable_for_a_pushed_car():
    """20 s crosses any mouth under power; a hand-pushed dry run with measured
    odometry cleared its distance floor at tick 188 of 200 (2026-09-01) and
    timed out twelve ticks later while walking a healthy course. The bound is
    a parameter so the caller that knows the pace can size it -- and it still
    fires, because it is a net, not a metronome."""
    from cone_nav.guidance.route_exec import RouteCursor
    from cone_nav.topology.topo_state import MAX_TRAVERSE_TICKS, TopoState

    topo = TopoState(RouteCursor(["left"]), max_traverse_ticks=5)
    assert topo.max_traverse_ticks == 5
    assert TopoState(RouteCursor(["left"])).max_traverse_ticks \
        == MAX_TRAVERSE_TICKS
