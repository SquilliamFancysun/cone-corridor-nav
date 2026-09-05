"""The signs, and almost nothing else.

Everything else in this module is two multiplications. What can actually be
wrong is the sign of either term, and the failure is quiet: a controller with
the cross-track term negated holds heading beautifully while walking the car
into the wall, so a heading trace from a failing run looks like a passing one.
"""

import math

import pytest

from cone_nav.control.pure_pursuit import MAX_STEER_RAD
from cone_nav.control.reverse_ctrl import (
    K_CROSS,
    K_HEADING,
    corridor_error,
    steer,
)


class FakeLine(object):
    def __init__(self, points):
        self.points = points
        self.single_boundary_fallback = False


# --- the sign law -------------------------------------------------------

def test_a_car_left_of_centre_steers_right():
    """Same sign as forward driving -- the half of the folk rule that is
    wrong. Backing up, the nose must swing LEFT to walk the car right, and
    swinging the nose left in reverse takes right steer."""
    assert steer(0.0, +0.3).delta_rad < 0


def test_a_car_right_of_centre_steers_left():
    assert steer(0.0, -0.3).delta_rad > 0


def test_a_corridor_running_left_means_the_nose_is_right_of_it_so_steer_right():
    """The argument is the CORRIDOR's direction in the car's frame, not the
    car's heading -- they are opposites, and feeding one where the other is
    meant inverts this term alone. That mistake spun the car through 140 deg
    on junction-left-blocked before it was caught."""
    assert steer(math.radians(15), 0.0).delta_rad < 0


def test_a_corridor_running_right_steers_left():
    assert steer(math.radians(-15), 0.0).delta_rad > 0


def test_it_turns_the_car_towards_a_corridor_it_is_misaligned_with():
    """The property the sign has to deliver, stated without reference to any
    frame: reversing with steer `d` changes heading by `-(v/L)tan d` for
    `v < 0`, so a corridor off to the left must produce a heading INCREASE."""
    delta = steer(math.radians(15), 0.0).delta_rad
    heading_rate = -math.tan(delta)          # sign of t' for v < 0
    assert heading_rate > 0


def test_the_two_terms_oppose_when_the_errors_do():
    """The case that separates this law from a negated forward one. A corridor
    off to the left while the car sits left of it: heading says right, offset
    says right too -- so take the case where they genuinely fight."""
    command = steer(math.radians(-15), +0.3)
    # corridor to the right wants left steer; sitting left wants right steer
    assert K_HEADING * math.radians(15) > K_CROSS * 0.3
    assert command.delta_rad > 0


def test_no_error_is_no_steer():
    assert steer(0.0, 0.0).delta_rad == 0.0


def test_the_steer_is_clamped_to_the_mechanism():
    assert steer(math.radians(90), 0.0).delta_rad == pytest.approx(-MAX_STEER_RAD)
    assert steer(math.radians(-90), 0.0).delta_rad == pytest.approx(MAX_STEER_RAD)


def test_the_normalised_command_matches_the_servo_convention():
    assert steer(math.radians(90), 0.0).normalised == pytest.approx(-1.0)


def test_the_errors_are_carried_for_the_log():
    """A drifting reverse is diagnosed from which error grew, and the steer
    alone cannot say."""
    command = steer(0.1, -0.2, reference="junction")
    assert command.heading_err_rad == pytest.approx(0.1)
    assert command.cross_track_m == pytest.approx(-0.2)
    assert command.reference == "junction"


# --- reading the errors off a corridor ---------------------------------

def test_a_straight_corridor_dead_ahead_is_no_error():
    heading, cross = corridor_error(FakeLine([(1.0, 0.0), (3.0, 0.0)]))
    assert heading == pytest.approx(0.0)
    assert cross == pytest.approx(0.0)


def test_a_corridor_off_to_the_left_puts_the_car_to_its_right():
    heading, cross = corridor_error(FakeLine([(1.0, 0.2), (3.0, 0.2)]))
    assert heading == pytest.approx(0.0)
    assert cross == pytest.approx(-0.2)


def test_a_corridor_at_an_angle_gives_both_errors_at_once():
    """A line through (1, 0) running away at 45 deg crosses x = 0 at y = -1,
    so a car at the origin sits 1.0 m above it -- 0.707 m perpendicular. Both
    numbers are checkable by hand, which is the point of this case."""
    heading, cross = corridor_error(FakeLine([(1.0, 0.0), (2.0, 1.0)]))
    assert heading == pytest.approx(math.radians(45))
    assert cross == pytest.approx(math.cos(math.radians(45)))


def test_the_offset_is_perpendicular_not_a_y_difference():
    """A corridor at 45 deg passing exactly through the car has zero offset.
    Measuring it as a y-difference against the line's first point would
    invent one and steer against a corridor the car is already on."""
    _, cross = corridor_error(FakeLine([(1.0, 1.0), (2.0, 2.0)]))
    assert cross == pytest.approx(0.0)


def test_a_line_too_short_to_have_a_direction_is_no_reference():
    """Not zero error. Zero error commands a straight reverse, and a straight
    reverse is what a car with a wrong heading must not do."""
    assert corridor_error(FakeLine([(1.0, 0.0)])) is None
    assert corridor_error(None) is None
