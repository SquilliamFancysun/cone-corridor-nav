"""When the car stops at the goal, and when it must not.

Two failures are being defended against and they pull in opposite directions. A
car that stops early -- at a junction, on a misread red -- has ended the run in
the middle of the course. A car that fails to stop drives into the trophy. So the
arming guard and the latch get most of these tests, and the blind budget gets the
rest, because the third failure is stopping on a point the car is only
remembering.
"""

import math

import pytest

from cone_nav.guidance.goal_stop import (
    GOAL_GATE_M,
    RUN_IN,
    RUN_IN_M,
    SEEKING,
    STOP_RANGE_M,
    STOPPED,
    GoalLatch,
)
from cone_perception.cone_classes import CLASS_MAGENTA
from cone_perception.fusion import LabeledCone


def goal_at(x, y=0.0):
    return LabeledCone(cone_class=CLASS_MAGENTA, confidence=0.9, x=x, y=y,
                       range_lidar=math.hypot(x, y), points=4)


# One tick of travel at the duty floor: 0.375 m/s at ~10 Hz. Sightings are fed
# at this rate because that is the only rate the latch will accept -- a goal
# that moves further than GOAL_GATE_M in a tick is, by construction, a different
# object.
STEP_M = 0.04


def confirm(latch, x, ticks=None):
    """Feed enough sightings at a fixed range to get past the identity debounce.

    A standing car: no travel, so no carry, and the goal does not move.
    """
    for _ in range(ticks if ticks is not None else latch.confirm_ticks):
        latch.update(goal_at(x), armed=True)
    return latch


def approach(latch, start, stop, step=STEP_M, armed=True):
    """Close on the goal from `start` to `stop` at a legal rate.

    `update` carries the goal back by `travel_m` before taking the sighting, so
    feeding a sighting one step nearer each tick is what a real approach looks
    like from the car: the carried estimate and the measurement agree, and the
    hop is nil.
    """
    x = start
    while x > stop:
        delta = min(step, x - stop)
        x -= delta
        latch.update(goal_at(x), armed=armed, travel_m=delta)
    return latch


# --- arming -------------------------------------------------------------

def test_it_does_not_stop_while_the_route_still_has_turns():
    """The guard that makes a red-read-as-magenta harmless. The goal is at the
    END of the route by construction, so anything seen before then is a misread
    and acting on it ends the run mid-course."""
    latch = GoalLatch()
    for _ in range(30):
        latch.update(goal_at(0.2), armed=False)
    assert latch.state == SEEKING
    assert not latch.stopped


def test_an_unarmed_tick_does_not_accumulate_toward_a_later_stop():
    """Sightings taken while disarmed must not count once the route empties --
    otherwise the confirmation is already spent and the first armed tick stops
    the car wherever it happens to be."""
    latch = GoalLatch()
    for _ in range(10):
        latch.update(goal_at(2.0), armed=False)
    assert not latch.confirmed


def test_it_arms_once_the_route_is_spent():
    latch = confirm(GoalLatch(), 2.0)
    assert latch.confirmed
    assert latch.goal_xy == pytest.approx((2.0, 0.0))


# --- the identity debounce ----------------------------------------------

def test_one_sighting_is_not_enough():
    """Unlike the junction, where a whole approach can yield a single triple,
    the trophy is dead ahead and in frame for the entire run-in. Sightings are
    plentiful here, so they can be spent on confirmation."""
    latch = GoalLatch()
    latch.update(goal_at(0.25), armed=True)
    assert not latch.stopped
    assert latch.state == SEEKING


def test_the_sightings_have_to_be_consecutive():
    latch = GoalLatch(confirm_ticks=3)
    latch.update(goal_at(2.0), armed=True)
    latch.update(goal_at(2.0), armed=True)
    latch.update(None, armed=True)
    assert not latch.confirmed
    latch.update(goal_at(2.0), armed=True)
    assert not latch.confirmed


# --- the run-in ---------------------------------------------------------

def test_the_reach_floor_is_not_touched_until_the_final_stretch():
    """`run_in` stands a safety rule down, so it must be false for the whole
    approach and true only over the last metre."""
    latch = approach(GoalLatch(), 2.5, RUN_IN_M + 0.1)
    assert latch.confirmed
    assert not latch.run_in
    approach(latch, RUN_IN_M + 0.1, RUN_IN_M - 0.1)
    assert latch.run_in
    assert latch.state == RUN_IN


def test_the_anchor_is_offered_before_the_run_in_begins():
    """The goal has to be on the driven line well before the speed law changes
    -- it is what the car is steering at, not just what stops it."""
    latch = approach(GoalLatch(), 2.5, 2.0)
    assert latch.anchor_ok
    assert not latch.run_in


# --- the trigger --------------------------------------------------------

def test_it_stops_inside_the_stop_range():
    latch = approach(GoalLatch(), 1.5, STOP_RANGE_M + 0.05)
    assert not latch.stopped
    approach(latch, STOP_RANGE_M + 0.05, STOP_RANGE_M - 0.02)
    assert latch.stopped
    assert latch.state == STOPPED
    assert latch.note == "goal reached"


def test_the_trigger_is_not_debounced():
    """At the duty floor the car covers 3.8 cm a tick, so three ticks of
    confirmation on the trigger would be 11 cm of overshoot bought for nothing.
    The identity is already confirmed by the time this matters."""
    latch = approach(GoalLatch(), 1.5, STOP_RANGE_M + 0.02)
    assert not latch.stopped
    latch.update(goal_at(STOP_RANGE_M - 0.01), armed=True, travel_m=STEP_M)
    assert latch.stopped


def test_the_stop_is_on_slant_range_not_forward_distance():
    latch = approach(GoalLatch(), 1.5, 0.42)
    # 0.29 m ahead but 0.30 m across is 0.417 m away -- not an arrival.
    latch.update(goal_at(0.29, 0.30), armed=True, travel_m=0.0)
    assert not latch.stopped


# --- the latch ----------------------------------------------------------

def test_it_stays_stopped_when_the_goal_drops_out():
    """The trophy leaves the camera frame and eventually falls under
    `clustering.MIN_CONE_RANGE_M` entirely. None of that may restart the car."""
    latch = approach(GoalLatch(), 1.5, STOP_RANGE_M - 0.02)
    assert latch.stopped
    for _ in range(50):
        latch.update(None, armed=True)
    assert latch.stopped


def test_a_fresh_sighting_elsewhere_does_not_restart_it():
    latch = approach(GoalLatch(), 1.5, STOP_RANGE_M - 0.02)
    for _ in range(10):
        latch.update(goal_at(2.5), armed=True)
    assert latch.stopped


def test_disarming_does_not_restart_it():
    """`armed` gates entry, not the hold. Nothing about the route may put a
    stopped car back in motion."""
    latch = approach(GoalLatch(), 1.5, STOP_RANGE_M - 0.02)
    latch.update(None, armed=False)
    assert latch.stopped


def test_release_lets_the_car_drive_again():
    """The deadman rising edge, so a trophy can be reset and the run repeated
    without restarting the tool."""
    latch = approach(GoalLatch(), 1.5, STOP_RANGE_M - 0.02)
    assert latch.stopped
    latch.release()
    assert latch.state == SEEKING
    assert not latch.stopped
    assert latch.goal_xy is None
    assert not latch.confirmed


# --- the blind budget ---------------------------------------------------

def test_the_goal_is_carried_through_a_dropout():
    """Without this the anchor vanishes in the last metre, the line collapses,
    and the car stops on 'no steerable target' where it cannot restart."""
    latch = confirm(GoalLatch(), 0.9)
    assert latch.state == RUN_IN
    latch.update(None, armed=True, travel_m=0.2)
    assert latch.anchor_ok
    assert latch.range_m == pytest.approx(0.7)


def test_the_carry_follows_a_turn_as_well_as_a_travel():
    latch = confirm(GoalLatch(), 2.0)
    latch.update(None, armed=True, travel_m=0.0,
                 yaw_delta_rad=math.radians(10.0))
    # The car turned left, so a point dead ahead moved to its right.
    assert latch.goal_xy[1] < 0.0
    assert latch.range_m == pytest.approx(2.0)


def test_a_live_sighting_beats_the_carried_estimate():
    """The carry is dead reckoning; a measurement that agrees with it wins."""
    latch = confirm(GoalLatch(), 2.0)
    # Carried to 1.80, measured at 1.78 -- a 2 cm disagreement, well inside the
    # gate, and the measurement is what is kept.
    latch.update(goal_at(1.78), armed=True, travel_m=0.2)
    assert latch.goal_xy == pytest.approx((1.78, 0.0))
    assert latch.blind_ticks == 0


def test_the_carry_gives_up_rather_than_stopping_on_a_memory():
    """Bounded, because this is a point the car steers at. Past the budget the
    latch drops and the car stops honestly instead of arriving at a place it is
    only remembering."""
    latch = confirm(GoalLatch(max_blind_ticks=5), 0.9)
    for _ in range(6):
        latch.update(None, armed=True, travel_m=0.01)
    assert latch.state == SEEKING
    assert not latch.stopped
    assert latch.note == "goal lost during run-in"
    assert not latch.anchor_ok


def test_a_stop_made_on_a_carried_goal_says_so():
    """Stopping is the fail-safe direction, so it is allowed inside the budget --
    but a run that finished on dead reckoning must not read as a clean arrival."""
    latch = confirm(GoalLatch(), 0.8)
    latch.update(None, armed=True, travel_m=0.55)
    assert latch.stopped
    assert latch.note == "goal reached (carried)"


# --- the re-binding gate ------------------------------------------------

def test_a_confirmed_goal_will_not_jump_to_a_distant_sighting():
    """`fusion` matches on bearing alone, and skips its range cross-check when
    the box is clipped -- which the trophy's is, up close. Two objects on one
    bearing then become interchangeable, and nothing below this gate notices."""
    latch = approach(GoalLatch(), 1.5, 0.60)
    before = latch.range_m
    latch.update(goal_at(1.77), armed=True, travel_m=0.0)
    assert latch.range_m == pytest.approx(before)
    assert latch.hops == 1
    assert latch.blind_ticks == 1


def test_the_gate_admits_a_sighting_that_could_be_the_same_object():
    latch = approach(GoalLatch(), 1.5, 0.60)
    latch.update(goal_at(0.60 - GOAL_GATE_M + 0.01), armed=True, travel_m=0.0)
    assert latch.hops == 0
    assert latch.blind_ticks == 0


def test_an_unconfirmed_hop_restarts_the_count_rather_than_being_refused():
    """Nothing is being abandoned yet, so the new object is as good a candidate
    as the old one -- but what earns confirmation must be one that held still."""
    latch = GoalLatch(confirm_ticks=3)
    latch.update(goal_at(1.50), armed=True)
    latch.update(goal_at(1.50), armed=True)
    latch.update(goal_at(0.58), armed=True)
    assert not latch.confirmed
    assert latch.goal_xy == pytest.approx((0.58, 0.0))
    assert latch.hops == 1


def test_the_label_alternating_between_two_objects_does_not_move_the_latch():
    """The track regression, from `goal-dry2.jsonl` 2026-09-01.

    Through the whole run-in the magenta label alternated between the trophy at
    0.58 m and something 1.17 m behind it, fifteen times, with exactly one
    magenta in view on every tick. The anchor is what the car steers at and what
    feeds `reach`, so unchecked that is a 1.17 m target jump every few ticks --
    and a latch bound to the far object when the car reaches the trophy does not
    stop at all. It stopped that day because the coin landed the right way.
    """
    latch = approach(GoalLatch(), 1.5, 0.58)
    assert latch.confirmed
    for _ in range(8):
        latch.update(goal_at(1.75), armed=True, travel_m=0.0)
        latch.update(goal_at(0.58), armed=True, travel_m=0.0)
    assert latch.range_m == pytest.approx(0.58, abs=0.01)
    assert latch.hops == 8
    assert not latch.stopped


def test_a_run_that_never_pins_the_goal_down_gives_up_rather_than_arriving():
    """Eight alternations is survivable because the real trophy keeps coming
    back. A label that never returns to it must exhaust the blind budget."""
    latch = approach(GoalLatch(max_blind_ticks=5), 1.5, 0.58)
    for _ in range(6):
        latch.update(goal_at(1.75), armed=True, travel_m=0.0)
    assert latch.state == SEEKING
    assert latch.note == "goal lost during run-in"


def test_a_goal_never_seen_is_not_being_carried():
    """`blind_ticks` is read as evidence that an arrival was made on dead
    reckoning, so it must count only ticks where something is actually being
    carried. A powered run that had not yet seen the trophy reported
    'carried 336' -- true of nothing at all."""
    latch = GoalLatch()
    for _ in range(50):
        latch.update(None, armed=True, travel_m=0.04)
    assert latch.blind_ticks == 0
    assert latch.goal_xy is None
