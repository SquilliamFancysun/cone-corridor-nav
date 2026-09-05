"""Pure pursuit, with the sign convention pinned down hard.

Most of these tests exist for one reason: a steering sign error is invisible
until the car is moving, and then it is a wall. So `left is positive` is
asserted from several directions -- the raw angle, the normalised command, and
a curve rather than a single point -- rather than trusted once.

Cases are stated in the AXLE frame (origin defaulting to (0, 0)) except where
the test is specifically about the base_link-to-axle offset, which is what
test_origin_offset_changes_the_command covers.
"""

import math

import pytest

from cone_nav.control.pure_pursuit import (
    MAX_STEER_RAD,
    MIN_TARGET_M,
    lookahead_point,
    smooth,
    steering_angle,
)

WHEELBASE = 0.25


def straight(length=6.0, step=0.5):
    """A centerline straight down +x."""
    return [(x * step, 0.0) for x in range(1, int(length / step) + 1)]


def arc(radius, sweep_deg=60.0, step_deg=5.0, left=True):
    """A centerline curving away from the car, tangent to +x at the origin."""
    sign = 1.0 if left else -1.0
    points = []
    n = int(sweep_deg / step_deg)
    for i in range(1, n + 1):
        theta = math.radians(i * step_deg)
        points.append((radius * math.sin(theta),
                       sign * radius * (1.0 - math.cos(theta))))
    return points


# --- the lookahead point ------------------------------------------------

def test_lookahead_lands_on_the_circle():
    point, short = lookahead_point(straight(), 1.5)
    assert short is False
    assert math.hypot(*point) == pytest.approx(1.5, abs=1e-9)


def test_lookahead_follows_a_curve_off_axis():
    point, short = lookahead_point(arc(3.0), 1.5)
    assert short is False
    assert math.hypot(*point) == pytest.approx(1.5, abs=1e-9)
    # A left curve puts the target left of centre. If this ever reads negative,
    # the whole stack has a mirrored frame.
    assert point[1] > 0.0


def test_lookahead_takes_the_far_root_not_the_near_one():
    """The first segment crosses the circle once going out; a naive smallest-root
    implementation picks the entry point on a line that starts inside."""
    points = [(0.5, 0.0), (3.0, 0.0)]
    point, _ = lookahead_point(points, 1.5)
    assert point[0] == pytest.approx(1.5)


def test_short_line_returns_the_far_end_and_says_so():
    point, short = lookahead_point(straight(length=1.0), 2.5)
    assert short is True
    assert point == (1.0, 0.0)


def test_a_line_starting_beyond_the_lookahead_aims_at_its_near_end():
    """Not the far end: a chain that begins off to one side would have the car
    cut the corner into whatever is between it and there."""
    points = [(3.0, 1.0), (4.0, 1.2)]
    point, short = lookahead_point(points, 1.5)
    assert short is False
    assert point == (3.0, 1.0)


def test_an_empty_line_is_no_point():
    assert lookahead_point([], 1.5) == (None, False)


def test_a_single_point_is_a_target_rather_than_a_polyline():
    """There is no segment to intersect, but there is somewhere to go.

    This is the end of a course and nothing else: the corridor's last midpoint
    has passed behind the car and the goal anchor is all that is left on the
    line. Refusing it froze the car 0.3004 m from the trophy with `no steerable
    target` -- one tick short of the stop, and in a state it could not drive out
    of, because the scan does not change while the car stands still.

    Reported short, because a single point never reaches the lookahead distance.
    """
    assert lookahead_point([(1.0, 0.0)], 1.5) == ((1.0, 0.0), True)


# --- the steering command -----------------------------------------------

def test_straight_ahead_is_zero_steer():
    result = steering_angle(straight(), 1.5, WHEELBASE)
    assert result.delta_rad == pytest.approx(0.0, abs=1e-12)
    assert result.normalised == pytest.approx(0.0, abs=1e-12)


def test_left_curve_steers_left():
    result = steering_angle(arc(3.0, left=True), 1.5, WHEELBASE)
    assert result.delta_rad > 0.0
    assert result.normalised > 0.0


def test_right_curve_steers_right():
    result = steering_angle(arc(3.0, left=False), 1.5, WHEELBASE)
    assert result.delta_rad < 0.0
    assert result.normalised < 0.0


def test_the_two_directions_are_symmetric():
    left = steering_angle(arc(3.0, left=True), 1.5, WHEELBASE)
    right = steering_angle(arc(3.0, left=False), 1.5, WHEELBASE)
    assert left.delta_rad == pytest.approx(-right.delta_rad, abs=1e-12)


def test_tighter_curve_steers_harder():
    tight = steering_angle(arc(1.5), 1.5, WHEELBASE)
    wide = steering_angle(arc(6.0), 1.5, WHEELBASE)
    assert tight.delta_rad > wide.delta_rad > 0.0


def test_curvature_matches_the_closed_form():
    """delta = atan(wheelbase * 2y / Ld^2), checked against the arithmetic rather
    than against itself."""
    lookahead = 2.0
    points = arc(4.0)
    target, _ = lookahead_point(points, lookahead)
    expected = math.atan(WHEELBASE * 2.0 * target[1] / (lookahead ** 2))
    result = steering_angle(points, lookahead, WHEELBASE)
    assert result.delta_rad == pytest.approx(expected, abs=1e-9)


def test_a_longer_lookahead_steers_more_gently():
    """The defining behaviour of the tuning knob: if this ever inverts, tuning
    at the track will chase its own tail."""
    near = steering_angle(arc(3.0), 1.0, WHEELBASE)
    far = steering_angle(arc(3.0), 2.5, WHEELBASE)
    assert 0.0 < far.delta_rad < near.delta_rad


def test_wheelbase_scales_the_command():
    small = steering_angle(arc(3.0), 1.5, 0.20)
    large = steering_angle(arc(3.0), 1.5, 0.40)
    assert large.delta_rad > small.delta_rad > 0.0


# --- the offset that the whole module exists to get right ----------------

def test_origin_offset_materially_changes_the_command():
    """The error the module docstring is about. If this ever reads
    `approx(equal)`, the offset is being dropped somewhere.

    Deliberately asserts magnitude and not direction. Moving the pivot back
    changes both the lateral offset to the target AND where the lookahead circle
    meets the line, and on a curve those push opposite ways -- so which way the
    command moves is a property of the path, not a constant. What is stable, and
    what matters, is that 0.25 m of offset is worth tens of percent.
    """
    points = arc(3.0)
    at_lidar = steering_angle(points, 1.5, WHEELBASE, origin=(0.0, 0.0))
    at_axle = steering_angle(points, 1.5, WHEELBASE, origin=(-0.25, 0.0))
    assert at_axle.delta_rad != pytest.approx(at_lidar.delta_rad, rel=0.2)


def test_lookahead_is_measured_from_the_origin():
    point, _ = lookahead_point(straight(), 1.5, origin=(-0.25, 0.0))
    assert math.hypot(point[0] + 0.25, point[1]) == pytest.approx(1.5, abs=1e-9)


# --- the refusals -------------------------------------------------------

def test_no_line_is_no_command():
    assert steering_angle([], 1.5, WHEELBASE) is None


def test_a_single_point_still_yields_a_command():
    """Steering at one measured point is well defined, so this module answers.

    Whether to MOVE on it is not this module's call and is not granted here:
    `speed_ctrl.duty` keeps its own `min_points` refusal, and only the goal
    run-in waives it. See `test_speed_ctrl.py`.
    """
    result = steering_angle([(1.0, 0.0)], 1.5, WHEELBASE)
    assert result is not None
    assert result.short_line
    assert result.delta_rad == pytest.approx(0.0, abs=1e-12)


def test_a_line_entirely_behind_the_car_is_no_command():
    """Not a reversing manoeuvre, and not a hard swing toward it."""
    points = [(-2.0, 0.0), (-3.0, 0.0)]
    assert steering_angle(points, 1.5, WHEELBASE) is None


def test_a_target_on_top_of_the_axle_is_no_command():
    points = [(0.02, 0.0), (0.05, 0.0)]
    result = steering_angle(points, 1.5, WHEELBASE)
    assert result is None


def test_min_target_is_the_boundary():
    """Just past the floor is steerable; just inside it is not."""
    over = steering_angle([(MIN_TARGET_M + 0.01, 0.0),
                           (MIN_TARGET_M + 0.02, 0.0)], 1.5, WHEELBASE)
    under = steering_angle([(MIN_TARGET_M - 0.01, 0.0),
                            (MIN_TARGET_M - 0.005, 0.0)], 1.5, WHEELBASE)
    assert over is not None
    assert under is None


# --- normalisation ------------------------------------------------------

def test_normalised_saturates_at_full_lock():
    """A hairpin must not command more than the servo can do."""
    result = steering_angle(arc(0.4), 0.5, 0.5)
    assert result.delta_rad > MAX_STEER_RAD
    assert result.normalised == pytest.approx(1.0)


def test_normalised_is_the_angle_over_the_limit():
    result = steering_angle(arc(3.0), 1.5, WHEELBASE)
    assert result.normalised == pytest.approx(result.delta_rad / MAX_STEER_RAD)


# --- the median filter --------------------------------------------------

def test_a_single_tick_spike_is_rejected_outright():
    """The measured failure: the command sits quiet and then slams as the
    centerline chain flickers between two neighbouring solutions."""
    history, out = [], None
    for value in (1.5, 1.5, 1.5, 13.5, 1.5):
        history, out = smooth(history, value)
    assert out == pytest.approx(1.5)


def test_a_two_tick_spike_is_rejected_by_five_but_not_by_three():
    """Why the window is 5. Seven of the trial's excursions lasted two ticks,
    and two of those outvote a window of three."""
    def run(window, values):
        history, out = [], None
        for v in values:
            history, out = smooth(history, v, window=window)
        return out

    values = (1.5, 1.5, 1.5, 13.5, 13.0)
    assert run(5, values) == pytest.approx(1.5)
    assert run(3, values) > 5.0


def test_a_real_corner_passes_through_undistorted():
    """A median must not smear a sustained change the way a mean would. Once
    the new value holds for a majority of the window, it IS the output."""
    history, out = [], None
    for value in (0.0, 0.0, 8.0, 8.0, 8.0):
        history, out = smooth(history, value)
    assert out == pytest.approx(8.0)


def test_the_filter_lags_by_half_a_window_on_a_step():
    """The cost, stated so it cannot be forgotten: two ticks at window 5. That
    is free at walking pace and expensive above about 2 m/s -- see the table on
    SMOOTH_WINDOW."""
    history = []
    for _ in range(5):
        history, _ = smooth(history, 0.0)
    outs = []
    for _ in range(3):
        history, out = smooth(history, 10.0)
        outs.append(out)
    assert outs[0] == pytest.approx(0.0)
    assert outs[1] == pytest.approx(0.0)
    assert outs[2] == pytest.approx(10.0)


def test_no_target_clears_the_history_rather_than_feeding_a_zero():
    """A dropout must not drag the filter toward centre and then have it climb
    back out, steering on the memory of a corridor the car could not see."""
    history = []
    for _ in range(5):
        history, _ = smooth(history, 8.0)
    history, out = smooth(history, None)
    assert history == []
    assert out == 0.0
    # Re-acquisition starts from the new corridor, not the old one.
    history, out = smooth(history, -3.0)
    assert out == pytest.approx(-3.0)


def test_it_fills_from_the_first_sample():
    """No warm-up period where the output is meaningless."""
    history, out = smooth([], 4.0)
    assert out == pytest.approx(4.0)


def test_the_window_never_grows_past_its_bound():
    history = []
    for i in range(50):
        history, _ = smooth(history, float(i))
    assert len(history) == 5


def test_window_of_one_is_a_passthrough():
    """The escape hatch for a fast run, where the lag costs more than the
    spikes do."""
    history, out = [], None
    for value in (1.0, 99.0, 2.0):
        history, out = smooth(history, value, window=1)
    assert out == pytest.approx(2.0)
