"""The expensive failure here is a false positive, so most of these are refusals.

A dead end declared in a clear corridor sends the car reversing out of somewhere
it was driving perfectly well, and on the track that looks like the detector
failing rather than the decision failing. The signal is deliberately slow and
deliberately geometric; these pin both.
"""

import pytest

from cone_nav.topology.dead_end import (
    CLEAR,
    DEAD_END,
    LONE_CONFIRM_TICKS,
    DeadEndLatch,
)


class FakeLine(object):
    def __init__(self, reach, fallback=False):
        self.points = [(x * 0.5, 0.0) for x in range(1, 4)] if reach else []
        if reach:
            self.points[-1] = (reach, 0.0)
        self.single_boundary_fallback = fallback


class FakeCone(object):
    def __init__(self, x=1.0, y=0.0):
        self.x = x
        self.y = y


def cones(n=8):
    return [FakeCone(x=0.5 * i) for i in range(n)]


def run(latch, ticks, line, cone_list=None, oranges=(), armed=True):
    cone_list = cones() if cone_list is None else cone_list
    for _ in range(ticks):
        latch.update(line, cone_list, oranges=oranges, armed=armed)
    return latch.state


# --- the refusals -------------------------------------------------------

def test_a_healthy_corridor_is_never_a_dead_end():
    latch = DeadEndLatch()
    assert run(latch, 50, FakeLine(2.5)) == CLEAR
    assert "reaches" in latch.reason


def test_a_blind_car_is_not_a_wall():
    """The dropout that matters: line collapsed, nothing in view. A wall is
    made of cones."""
    latch = DeadEndLatch()
    assert run(latch, 50, FakeLine(0.3), cone_list=cones(2)) == CLEAR
    assert "2 cones" in latch.reason


def test_a_single_boundary_fallback_is_not_a_wall():
    """That line is what the car produces while confused about one wall."""
    latch = DeadEndLatch()
    assert run(latch, 50, FakeLine(0.3, fallback=True)) == CLEAR
    assert "fallback" in latch.reason


def test_no_line_at_all_is_refused():
    latch = DeadEndLatch()
    assert run(latch, 50, None) == CLEAR


def test_it_does_not_fire_while_held_down():
    """Through a junction mouth and over the goal run-in the corridor is
    allowed to end, and the caller says so."""
    latch = DeadEndLatch()
    assert run(latch, 50, FakeLine(0.3), oranges=[FakeCone()],
               armed=False) == CLEAR
    assert latch.reason == "not armed"


def test_a_one_tick_dropout_does_not_accumulate():
    """Confirmation must be consecutive, or a flickering corridor eventually
    adds up to a wall."""
    latch = DeadEndLatch()
    for _ in range(20):
        latch.update(FakeLine(0.3), cones(), oranges=[FakeCone()])
        latch.update(FakeLine(2.5), cones(), oranges=[FakeCone()])
    assert latch.state == CLEAR


# --- firing -------------------------------------------------------------

def test_a_short_line_with_an_orange_wall_confirms():
    latch = DeadEndLatch()
    assert run(latch, 6, FakeLine(0.4), oranges=[FakeCone(1.2, 0.0)]) == DEAD_END
    assert "orange wall seen" in latch.reason
    assert "0.40 m" in latch.reason


def test_it_confirms_without_any_orange_at_all():
    """Orange recall is 0.687 -- a third of walls are missed. The geometric
    signal has to stand alone, and only pays for it in time."""
    latch = DeadEndLatch()
    assert run(latch, 6, FakeLine(0.4)) == CLEAR
    assert run(latch, LONE_CONFIRM_TICKS, FakeLine(0.4)) == DEAD_END
    assert "geometry alone" in latch.reason


def test_an_orange_shortens_the_wait():
    fast, slow = DeadEndLatch(), DeadEndLatch()
    run(fast, 6, FakeLine(0.4), oranges=[FakeCone(1.0, 0.0)])
    run(slow, 6, FakeLine(0.4))
    assert fast.latched and not slow.latched


def test_an_orange_across_the_field_is_not_this_corridors_wall():
    latch = DeadEndLatch()
    run(latch, 6, FakeLine(0.4), oranges=[FakeCone(1.0, 3.0)])
    assert not latch.latched


def test_an_orange_far_down_the_track_is_not_this_corridors_wall():
    latch = DeadEndLatch()
    run(latch, 6, FakeLine(0.4), oranges=[FakeCone(4.0, 0.0)])
    assert not latch.latched


def test_an_orange_behind_the_car_does_not_count():
    latch = DeadEndLatch()
    run(latch, 6, FakeLine(0.4), oranges=[FakeCone(-1.0, 0.0)])
    assert not latch.latched


# --- the latch ----------------------------------------------------------

def test_the_latch_is_sticky_once_set():
    """A stopped car's scan does not change, so nothing downstream may depend
    on the signal persisting -- but nothing may clear it either."""
    latch = DeadEndLatch()
    run(latch, 6, FakeLine(0.4), oranges=[FakeCone(1.0, 0.0)])
    assert run(latch, 20, FakeLine(2.5)) == DEAD_END


def test_the_deadman_clears_it():
    latch = DeadEndLatch()
    run(latch, 6, FakeLine(0.4), oranges=[FakeCone(1.0, 0.0)])
    latch.release()
    assert latch.state == CLEAR
    assert latch.confirm == 0


def test_reach_is_reported_for_the_log():
    latch = DeadEndLatch()
    latch.update(FakeLine(0.42), cones())
    assert latch.reach_m == pytest.approx(0.42)


# --- re-arming after a release ------------------------------------------

def test_it_does_not_name_the_same_wall_twice_without_moving():
    """Measured on the car 2026-09-02: X was re-pressed at the wall, the
    unchanged scan re-confirmed, and the run recorded three dead ends for two
    real walls -- spending a cursor.dead_end() the search had not earned."""
    latch = DeadEndLatch()
    run(latch, 20, FakeLine(0.4), oranges=[FakeCone(1.0, 0.0)])
    assert latch.latched
    latch.release()
    assert run(latch, 60, FakeLine(0.4), oranges=[FakeCone(1.0, 0.0)]) == CLEAR
    assert "re-arming in" in latch.reason


def test_a_lift_reads_as_no_travel_so_it_stays_suppressed():
    """rigid_step cannot see a carry, so an operator-assisted recovery gives
    zero travel -- and a car put back at the junction must not immediately
    re-name the wall it was carried away from."""
    latch = DeadEndLatch()
    run(latch, 20, FakeLine(0.4), oranges=[FakeCone(1.0, 0.0)])
    latch.release()
    for _ in range(60):
        latch.update(FakeLine(0.4), cones(), oranges=[FakeCone(1.0, 0.0)],
                     travel_m=0.0)
    assert not latch.latched


def test_driving_away_re_arms_it():
    latch = DeadEndLatch()
    run(latch, 20, FakeLine(0.4), oranges=[FakeCone(1.0, 0.0)])
    latch.release()
    for _ in range(20):        # 20 x 0.03 m = 0.6 m, past the 0.5 m floor
        latch.update(FakeLine(2.5), cones(), travel_m=0.03)
    assert run(latch, 20, FakeLine(0.4), oranges=[FakeCone(1.0, 0.0)]) == DEAD_END


def test_reversing_away_counts_as_travel_too():
    """The floor is about new evidence, not direction -- and Phase 2 backs
    the car out of the wall it just named."""
    latch = DeadEndLatch()
    run(latch, 20, FakeLine(0.4), oranges=[FakeCone(1.0, 0.0)])
    latch.release()
    for _ in range(20):
        latch.update(FakeLine(2.5), cones(), travel_m=-0.03)
    assert run(latch, 20, FakeLine(0.4), oranges=[FakeCone(1.0, 0.0)]) == DEAD_END


def test_the_first_wall_of_a_run_needs_no_travel_first():
    """A car that starts facing a wall must still name it."""
    latch = DeadEndLatch()
    assert run(latch, 20, FakeLine(0.4), oranges=[FakeCone(1.0, 0.0)]) == DEAD_END


def test_a_collapsed_line_is_not_a_wall():
    """Measured on the car 2026-09-02 (`explore-3.jsonl`): the latch fired
    twice reporting "corridor ends 0.00 m ahead", which is a contradiction --
    reach 0.00 with zero centerline points is the pairing failing, not a
    corridor that stopped short. Cone count alone did not catch it: clusters
    were in view and the pairing still produced nothing."""
    latch = DeadEndLatch()
    empty = FakeLine(0)
    assert len(empty.points) == 0
    assert run(latch, 60, empty, oranges=[FakeCone(1.0, 0.0)]) == CLEAR
    assert "collapsed" in latch.reason


def test_a_real_wall_still_has_a_line_behind_it():
    """The distinction the refusal turns on: a wall is a corridor that stops
    SHORT, so the last pair is still there and reach lands near 0.8 m."""
    latch = DeadEndLatch()
    assert run(latch, 6, FakeLine(0.8), oranges=[FakeCone(1.0, 0.0)]) == DEAD_END
    assert "0.80 m" in latch.reason


def test_an_orange_where_a_boundary_cone_stands_is_not_a_wall():
    """A wall cone stands across the corridor near its centreline; a boundary
    cone stands on the wall at the 0.75 m half-width. Measured 2026-09-02 in a
    setting sun, orange was 27% of boundary-ish sightings on a track with one
    orange cone per dead end -- the misreads were yellows, and they sit exactly
    there."""
    latch = DeadEndLatch()
    run(latch, 6, FakeLine(0.4), oranges=[FakeCone(1.0, -0.75)])
    assert not latch.latched, "a misread yellow must not shorten the wait"


def test_an_orange_across_the_end_still_counts():
    """The genuine case has to survive the tighter gate: a wall cone seen by a
    car that is offset or angled, but still near the axis."""
    latch = DeadEndLatch()
    assert run(latch, 6, FakeLine(0.4),
               oranges=[FakeCone(1.0, 0.35)]) == DEAD_END
    assert "orange wall seen" in latch.reason


def test_a_rejected_orange_only_costs_time_not_the_wall():
    """The refusal must not veto the dead end -- geometry is the signal and
    orange is corroboration, so a misread should slow it to the twelve-tick
    path rather than silence it."""
    latch = DeadEndLatch()
    assert run(latch, LONE_CONFIRM_TICKS, FakeLine(0.4),
               oranges=[FakeCone(1.0, -0.75)]) == DEAD_END
    assert "geometry alone" in latch.reason


def test_a_well_placed_orange_beats_the_single_boundary_refusal():
    """Measured 2026-09-02 (`explore-6.jsonl`): an orange was present on 110 of
    110 ticks at a wall, within 4 cm of the centreline, and 63 of those ticks
    were refused as a fallback. A car at a dead end is close and angled, so one
    boundary legitimately leaves the arc -- it has arrived, not got confused."""
    latch = DeadEndLatch()
    assert run(latch, 6, FakeLine(0.4, fallback=True),
               oranges=[FakeCone(1.0, 0.05)]) == DEAD_END


def test_a_fallback_with_no_orange_is_still_refused():
    """The guard still does its original job. Without positive evidence, a
    one-sided line is a confused car."""
    latch = DeadEndLatch()
    assert run(latch, 60, FakeLine(0.4, fallback=True)) == CLEAR
    assert "no orange to say otherwise" in latch.reason


def test_a_misplaced_orange_does_not_rescue_a_fallback():
    """The exception rides on the SAME position test as the corroboration, so
    a misread yellow at the corridor wall cannot buy it either."""
    latch = DeadEndLatch()
    assert run(latch, 60, FakeLine(0.4, fallback=True),
               oranges=[FakeCone(1.0, -0.75)]) == CLEAR


def test_an_orange_cannot_rescue_a_collapsed_line():
    """Zero points is the absence of a corridor, not one that stopped short.
    No amount of orange makes "ends 0.00 m ahead" mean something."""
    latch = DeadEndLatch()
    assert run(latch, 60, FakeLine(0), oranges=[FakeCone(1.0, 0.0)]) == CLEAR
    assert "collapsed" in latch.reason
