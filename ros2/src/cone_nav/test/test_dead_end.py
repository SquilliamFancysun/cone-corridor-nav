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
