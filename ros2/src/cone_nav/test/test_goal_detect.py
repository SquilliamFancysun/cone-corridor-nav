"""What counts as the goal, and -- mostly -- what does not.

Magenta was called RED on 69% of instances by two model generations, and the
confusion running the other way is the expensive one: a red gate read as magenta
stops the car dead in the middle of the course. So the bulk of these tests push
plausible-but-wrong magenta at `detect` and require None back, with a reason that
names the fix.
"""

import math

import pytest

from cone_nav.topology.goal_detect import (
    DISTANCE,
    GOAL_ARM_RANGE_M,
    MAX_OFFSET_M,
    MULTIPLE,
    NO_MAGENTA,
    OFF_AXIS,
    detect,
    survey,
)
from cone_perception.cone_classes import (
    CLASS_BLUE,
    CLASS_MAGENTA,
    CLASS_RED,
    CLASS_YELLOW,
)
from cone_perception.fusion import LabeledCone


def cone(x, y, cls=CLASS_MAGENTA, confidence=0.9, points=4):
    return LabeledCone(cone_class=cls, confidence=confidence, x=x, y=y,
                       range_lidar=math.hypot(x, y), points=points)


def corridor(x=2.0, half_width=0.75):
    """A blue and a yellow wall either side, as any real scene would have."""
    return [cone(x, half_width, CLASS_BLUE), cone(x, -half_width, CLASS_YELLOW)]


# --- the happy path -----------------------------------------------------

def test_a_centered_magenta_in_range_is_the_goal():
    goal = detect([cone(2.0, 0.0)] + corridor())
    assert goal is not None
    assert (goal.x, goal.y) == pytest.approx((2.0, 0.0))


def test_the_survey_reports_the_range_and_no_reason():
    got = survey([cone(1.5, 0.0)])
    assert got.goal is not None
    assert got.range_m == pytest.approx(1.5)
    assert got.reason == ""


def test_the_walls_around_it_are_not_goals():
    """Only magenta is ever a candidate. A corridor on its own has no goal in
    it, however centered its cones look."""
    assert detect(corridor()) is None
    assert survey(corridor()).reason == NO_MAGENTA


# --- the refusals -------------------------------------------------------

def test_no_magenta_at_all():
    got = survey([cone(2.0, 0.0, CLASS_RED)])
    assert got.goal is None
    assert got.reason == NO_MAGENTA
    assert got.magenta == []


def test_a_magenta_beyond_arm_range_is_seen_but_not_accepted():
    """The distinction the reason exists for: the trophy IS there and the car is
    standing too far back. `magenta` says so where a bare None could not."""
    far = GOAL_ARM_RANGE_M + 0.5
    got = survey([cone(far, 0.0)])
    assert got.goal is None
    assert got.reason == DISTANCE
    assert len(got.magenta) == 1
    assert got.in_arm == []
    assert got.ranges_m[0] == pytest.approx(far)


def test_arm_range_is_slant_range_not_forward_distance():
    """A goal 2.9 m ahead and 1.0 m across is 3.07 m away. Measuring x alone
    would arm the stop on a cone the lidar cannot range."""
    got = survey([cone(2.9, 1.0)], max_offset_m=2.0)
    assert got.goal is None
    assert got.reason == DISTANCE


def test_a_magenta_off_the_corridor_axis_is_a_mislabelled_boundary_cone():
    got = survey([cone(2.0, MAX_OFFSET_M + 0.2)])
    assert got.goal is None
    assert got.reason == OFF_AXIS


def test_the_rejected_offset_is_reported_so_the_log_names_the_fix():
    got = survey([cone(2.0, 0.9)])
    assert got.reason == OFF_AXIS
    assert got.offset_m == pytest.approx(0.9)


def test_the_offset_is_measured_against_the_corridor_not_the_car():
    """Met mid-bend, a centered goal is off the CAR's axis and on the
    corridor's. Judging it against the car would refuse every goal on a curve."""
    axis = math.radians(20.0)
    at = (2.0 * math.cos(axis), 2.0 * math.sin(axis))
    assert detect([cone(*at)], axis_rad=axis) is not None
    # ...and the same cone against a straight-ahead axis is 0.68 m off it.
    assert detect([cone(*at)]) is None


def test_two_magentas_in_range_are_refused_rather_than_guessed_between():
    """There is no safe way to pick one, and picking wrong drives the car at the
    wrong object. Declining costs a stop; guessing costs the run."""
    got = survey([cone(2.0, 0.0), cone(2.5, 0.3)])
    assert got.goal is None
    assert got.reason == MULTIPLE
    assert len(got.in_arm) == 2


def test_a_second_magenta_out_of_range_does_not_spoil_the_near_one():
    """MULTIPLE is about what the car could act on, not about what is in view.
    A stray magenta 4 m away must not veto a good goal at 2 m."""
    got = survey([cone(2.0, 0.0), cone(GOAL_ARM_RANGE_M + 1.0, 0.0)])
    assert got.goal is not None
    assert len(got.magenta) == 2
    assert len(got.in_arm) == 1


def test_a_magenta_behind_the_car_is_not_the_goal():
    """`boundary_split` drops it before this module ever sees it -- the goal is
    something the car is driving toward, and one behind it has been passed."""
    got = survey([cone(-1.5, 0.0)])
    assert got.goal is None
    assert got.reason == NO_MAGENTA


# --- the wrapper --------------------------------------------------------

def test_detect_is_the_surveys_decision():
    """Written as a wrapper so the reason in the log is always the reason for
    this tick's rejection, rather than a second implementation that can drift."""
    for cones in ([], [cone(2.0, 0.0)], [cone(2.0, 1.5)],
                  [cone(2.0, 0.0), cone(1.0, 0.0)]):
        assert detect(cones) is survey(cones).goal
