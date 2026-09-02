"""The speed law, and specifically its refusals.

The interesting assertions here are the zeros. A car that drives too fast is a
tuning problem; a car that keeps driving after perception has stopped producing
a corridor is the failure that ends a session, so every path to duty 0.0 gets a
test and a named reason.
"""

import pytest

from cone_nav.control.pure_pursuit import steering_angle
from cone_nav.control.speed_ctrl import (
    DEFAULT_MAX_DUTY,
    FULL_REACH_M,
    MAX_DUTY_STEP,
    MIN_MOVE_DUTY,
    MIN_REACH_M,
    duty,
    ramp,
    reach_of,
    reverse_duty,
)

WHEELBASE = 0.25


class FakeLine(object):
    """Stands in for centerline.CenterlineResult.

    Only the three fields the speed law reads. Building a real CenterlineResult
    would mean building a cone field to produce it, which would make these tests
    depend on the triangulation -- and then a corridor-layer regression would
    show up as a speed-control failure.
    """

    def __init__(self, points, single_boundary_fallback=False):
        self.points = points
        self.single_boundary_fallback = single_boundary_fallback


def straight(length=6.0, step=0.5):
    return [(x * step, 0.0) for x in range(1, int(length / step) + 1)]


def pursuit_for(points, lookahead=1.5):
    return steering_angle(points, lookahead, WHEELBASE)


# --- reach --------------------------------------------------------------

def test_reach_is_along_track_not_arc_length():
    """A line that wanders sideways has not bought any more forward visibility."""
    wiggly = [(1.0, 0.0), (1.5, 2.0), (2.0, -2.0)]
    assert reach_of(FakeLine(wiggly)) == pytest.approx(2.0)


def test_reach_of_an_empty_line_is_zero():
    assert reach_of(FakeLine([])) == 0.0


def test_reach_is_measured_from_the_origin():
    line = FakeLine(straight(length=3.0))
    assert reach_of(line, origin=(-0.25, 0.0)) == pytest.approx(3.25)


# --- the refusals -------------------------------------------------------

def test_no_steerable_target_stops_the_car():
    result = duty(None, FakeLine(straight()))
    assert result.duty == 0.0
    assert not result.moving
    assert "no steerable target" in result.reason


def test_a_one_point_line_stops_the_car():
    points = straight()
    result = duty(pursuit_for(points), FakeLine([(2.0, 0.0)]))
    assert result.duty == 0.0
    assert "too short" in result.reason


def test_a_corridor_seen_only_a_little_way_ahead_stops_the_car():
    # Fine step, so the line has plenty of points and it is genuinely the REACH
    # that stops the car rather than pure pursuit running out of polyline.
    points = straight(length=0.8, step=0.2)
    result = duty(pursuit_for(points), FakeLine(points))
    assert result.duty == 0.0
    assert "ahead" in result.reason


def test_the_reach_floor_is_a_boundary_not_a_slope():
    """Just under MIN_REACH_M is a stop; just over is motion."""
    under = straight(length=MIN_REACH_M - 0.1, step=0.1)
    over = straight(length=MIN_REACH_M + 0.4, step=0.1)
    assert duty(pursuit_for(under), FakeLine(under)).duty == 0.0
    assert duty(pursuit_for(over), FakeLine(over)).duty > 0.0


# --- standing the reach floor down, for the goal run-in -----------------

def test_the_reach_floor_can_be_lowered_by_the_caller():
    """The goal run-in's one requirement. Without it the reach rule stops the
    car 0.64 m from the trophy -- before any stop range can trigger, with
    'corridor visible only ...' in the log, and unrecoverably, since the scan
    does not change while the car stands still."""
    points = straight(length=0.6, step=0.15)
    assert duty(pursuit_for(points), FakeLine(points)).duty == 0.0
    relaxed = duty(pursuit_for(points), FakeLine(points), min_reach_m=0.0)
    assert relaxed.duty > 0.0
    assert relaxed.reason == ""


def test_a_relaxed_floor_still_crawls_rather_than_accelerating():
    """Reach is what scales duty between the floor and FULL_REACH_M, so a line
    this short lands on MIN_MOVE_DUTY. The last metre is driven at the cogging
    floor, which is the only speed that makes sense there."""
    points = straight(length=0.6, step=0.15)
    result = duty(pursuit_for(points), FakeLine(points), min_reach_m=0.0)
    assert result.duty == pytest.approx(MIN_MOVE_DUTY)


def test_a_one_point_line_does_not_move_the_car_by_default():
    """`pure_pursuit` will steer at a single target; deciding to MOVE on one is
    this module's call, and by default it refuses. Two points is the right floor
    for a CORRIDOR, whose lone midpoint says nothing trustworthy."""
    line = FakeLine([(0.6, 0.0)])
    assert duty(pursuit_for(line.points), line).duty == 0.0


def test_the_goal_run_in_may_drive_at_a_single_point():
    """...and the wrong floor for a measured object at the end of a course,
    where the corridor has genuinely run out and the trophy is all that is
    left."""
    line = FakeLine([(0.6, 0.0)])
    result = duty(pursuit_for(line.points), line, min_reach_m=0.0, min_points=1)
    assert result.duty == pytest.approx(MIN_MOVE_DUTY)
    assert result.reason == ""


def test_relaxing_the_floor_changes_nothing_about_a_healthy_corridor():
    """It moves one refusal and nothing else -- a corridor that was drivable is
    driven at exactly the same duty."""
    points = straight(length=4.0, step=0.25)
    plain = duty(pursuit_for(points), FakeLine(points))
    relaxed = duty(pursuit_for(points), FakeLine(points), min_reach_m=0.0)
    assert relaxed.duty == pytest.approx(plain.duty)


# --- the dead band ------------------------------------------------------

def test_duty_never_lands_between_zero_and_the_motor_floor():
    """The property the whole MIN_MOVE_DUTY constant exists for: a commanded
    duty in the dead band is a stalled, buzzing motor and a log full of
    plausible small numbers. Swept across every condition that derates."""
    for length in (1.1, 1.5, 2.0, 3.0, 6.0):
        for fallback in (False, True):
            points = straight(length=length, step=0.25)
            line = FakeLine(points, single_boundary_fallback=fallback)
            value = duty(pursuit_for(points), line).duty
            assert value == 0.0 or value >= MIN_MOVE_DUTY, (
                f"length={length} fallback={fallback} gave {value}")


def test_a_hard_turn_still_moves():
    """Derating must not silently stall the car mid-corner."""
    points = [(0.6, 0.0), (1.2, 0.5), (1.8, 1.4), (2.4, 2.6)]
    result = duty(pursuit_for(points), FakeLine(points))
    assert result.duty >= MIN_MOVE_DUTY


# --- the derates --------------------------------------------------------

def test_a_straight_clear_corridor_runs_at_the_cap():
    points = straight()
    assert duty(pursuit_for(points), FakeLine(points)).duty == pytest.approx(
        DEFAULT_MAX_DUTY)


def test_turning_is_slower_than_straight():
    line = straight()
    curve = [(0.5, 0.0), (1.0, 0.2), (1.5, 0.6), (2.0, 1.2), (2.5, 2.0)]
    fast = duty(pursuit_for(line), FakeLine(line)).duty
    slow = duty(pursuit_for(curve), FakeLine(curve)).duty
    assert slow < fast


def test_the_single_boundary_fallback_is_penalised():
    points = straight()
    measured = duty(pursuit_for(points), FakeLine(points)).duty
    inferred = duty(pursuit_for(points),
                    FakeLine(points, single_boundary_fallback=True)).duty
    assert inferred < measured


def test_speed_eases_off_as_the_corridor_ahead_thins_out():
    near = straight(length=1.4, step=0.2)
    far = straight(length=FULL_REACH_M + 1.0, step=0.2)
    assert duty(pursuit_for(near), FakeLine(near)).duty < \
        duty(pursuit_for(far), FakeLine(far)).duty


def test_the_cap_is_respected():
    points = straight()
    result = duty(pursuit_for(points), FakeLine(points), max_duty=0.06)
    assert result.duty <= 0.06


# --- the ramp -----------------------------------------------------------

def test_the_ramp_limits_the_rise():
    assert ramp(0.0, 0.10, max_step=0.02) == pytest.approx(0.02)
    assert ramp(0.02, 0.10, max_step=0.02) == pytest.approx(0.04)


def test_the_ramp_does_not_slow_a_stop():
    """Every path to a commanded zero is either a safety condition or a
    perception dropout. Neither is improved by easing into it."""
    assert ramp(0.10, 0.0) == 0.0


def test_the_ramp_passes_a_small_rise_through_untouched():
    assert ramp(0.05, 0.055, max_step=0.02) == pytest.approx(0.055)


def test_the_ramp_allows_an_immediate_reduction():
    assert ramp(0.10, 0.06, max_step=0.02) == pytest.approx(0.06)


def test_a_full_ramp_reaches_the_cap_in_a_sane_number_of_ticks():
    """At 10 Hz this is the standstill-to-full time, and it wants to be under
    about a second."""
    value, ticks = 0.0, 0
    while value < DEFAULT_MAX_DUTY and ticks < 100:
        value = ramp(value, DEFAULT_MAX_DUTY)
        ticks += 1
    assert value == pytest.approx(DEFAULT_MAX_DUTY)
    assert ticks <= 10


# --- reverse ------------------------------------------------------------

def test_reverse_duty_is_negative_and_at_the_floor():
    """Reverse is the direction with no lidar behind it, so it runs at the
    cogging floor and no faster."""
    assert reverse_duty() == -MIN_MOVE_DUTY


def test_reverse_duty_magnitude_is_capped_however_it_is_passed():
    assert reverse_duty(0.08) == -0.08
    assert reverse_duty(-0.08) == -0.08


def test_the_ramp_eases_into_reverse_the_same_way_it_eases_forward():
    assert ramp(0.0, -0.05) == pytest.approx(-MAX_DUTY_STEP)


def test_the_ramp_will_not_cross_zero_in_one_direction_or_the_other():
    """A car asked to reverse while still rolling forward reaches zero at once
    and starts again. Sliding across would command a duty that means nothing
    in either direction."""
    assert ramp(0.08, -0.05) == 0.0
    assert ramp(-0.05, 0.08) == 0.0


def test_a_commanded_stop_is_still_immediate_from_reverse():
    assert ramp(-0.05, 0.0) == 0.0


def test_the_old_forward_contract_is_unchanged():
    """Every caller that never commands a negative must see exactly what it
    saw before."""
    assert ramp(0.0, 0.10) == pytest.approx(MAX_DUTY_STEP)
    assert ramp(0.06, 0.06) == pytest.approx(0.06)
    assert ramp(0.06, 0.0) == 0.0
