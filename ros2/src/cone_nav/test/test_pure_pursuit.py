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


@pytest.mark.parametrize("points", [[], [(1.0, 0.0)]])
def test_too_few_points_is_no_point(points):
    assert lookahead_point(points, 1.5) == (None, False)


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

@pytest.mark.parametrize("points", [[], [(1.0, 0.0)]])
def test_no_line_is_no_command(points):
    assert steering_angle(points, 1.5, WHEELBASE) is None


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
