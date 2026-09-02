"""Integrating is three lines and one of them is easy to get wrong.

A Step is expressed in the frame the car held BEFORE it moved, so the rotation
has to use the old heading. Using the new one bends every straight line into a
spiral -- slowly enough that a short test passes and a whole run does not. Most
of what follows exists to catch that.
"""

import math

import pytest

from cone_perception.ego_motion import Step
from cone_perception.odometry import Pose, distance_between


def straight(forward):
    return Step(forward, 0.0, 0.0, 4)


def turning(forward, yaw_deg):
    return Step(forward, 0.0, math.radians(yaw_deg), 4)


# --- the basics ---------------------------------------------------------

def test_a_new_pose_is_at_the_origin():
    pose = Pose()
    assert pose.xy == (0.0, 0.0)
    assert pose.yaw_rad == 0.0
    assert pose.steps == 0


def test_driving_forward_moves_along_x():
    pose = Pose().integrate(straight(1.0))
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(0.0)


def test_reversing_moves_back_along_x():
    """rigid_step is signed and direction-agnostic; nothing here may clamp it.
    topo_state clamps negative travel, and this must not inherit that."""
    pose = Pose().integrate(straight(-0.5))
    assert pose.x == pytest.approx(-0.5)


def test_a_none_step_is_no_motion_but_still_a_tick():
    """The same safe convention an empty centerline gets."""
    pose = Pose().integrate(straight(1.0)).integrate(None)
    assert pose.x == pytest.approx(1.0)
    assert pose.steps == 2
    assert pose.measured == 1


# --- the rotation, which is the part worth testing ----------------------

def test_it_turns_then_translates_in_the_new_heading():
    pose = Pose()
    pose.integrate(turning(0.0, 90.0))
    pose.integrate(straight(1.0))
    assert pose.yaw_deg == pytest.approx(90.0)
    assert pose.x == pytest.approx(0.0, abs=1e-9)
    assert pose.y == pytest.approx(1.0)


def test_a_step_rotates_by_the_heading_held_before_it():
    """The spiral bug. A step that both advances and turns must land where the
    OLD heading pointed; using the new heading puts it off to the side."""
    pose = Pose().integrate(turning(1.0, 90.0))
    assert pose.x == pytest.approx(1.0)
    assert pose.y == pytest.approx(0.0, abs=1e-9)
    assert pose.yaw_deg == pytest.approx(90.0)


def test_four_right_angles_return_to_the_start():
    """The integration test for the whole thing: a closed square must close."""
    pose = Pose()
    for _ in range(4):
        pose.integrate(straight(1.0))
        pose.integrate(turning(0.0, 90.0))
    assert pose.x == pytest.approx(0.0, abs=1e-9)
    assert pose.y == pytest.approx(0.0, abs=1e-9)
    assert pose.yaw_deg == pytest.approx(0.0, abs=1e-9)


def test_lateral_motion_is_to_the_left():
    pose = Pose().integrate(Step(0.0, 0.5, 0.0, 3))
    assert pose.y == pytest.approx(0.5)


def test_heading_stays_wrapped_over_many_turns():
    pose = Pose()
    for _ in range(10):
        pose.integrate(turning(0.0, 100.0))
    assert -math.pi < pose.yaw_rad <= math.pi


# --- the deadband, which defaults off here ------------------------------

def test_there_is_no_deadband_by_default():
    """Opposite of dry_run_travel, and deliberately. A signed sum cancels
    jitter; a deadband would under-count a slow hand push."""
    pose = Pose().integrate(straight(0.004))
    assert pose.x == pytest.approx(0.004)


def test_a_deadband_can_be_asked_for():
    pose = Pose().integrate(straight(0.004), deadband_m=0.008)
    assert pose.x == 0.0
    # Still a measured tick: a step was supplied and a yaw may have come with
    # it. Only the translation was suppressed.
    assert pose.measured == 1


def test_jitter_cancels_over_a_signed_sum():
    """The argument the default rests on, made concrete."""
    pose = Pose()
    for i in range(100):
        pose.integrate(straight(0.003 if i % 2 else -0.003))
    assert pose.x == pytest.approx(0.0, abs=1e-9)


# --- what the map and the graph read ------------------------------------

def test_a_snapshot_does_not_follow_the_pose():
    pose = Pose().integrate(straight(1.0))
    mark = pose.snapshot()
    pose.integrate(straight(1.0))
    assert mark[0] == pytest.approx(1.0)


def test_edge_length_is_the_separation_of_two_snapshots():
    pose = Pose()
    a = pose.snapshot()
    pose.integrate(straight(2.5))
    assert distance_between(a, pose.snapshot()) == pytest.approx(2.5)


def test_separation_ignores_jitter_that_path_length_accumulates():
    """Why graph_builder uses distance_between and not path_m: a standing car
    gains path length and gains no separation."""
    pose = Pose()
    a = pose.snapshot()
    for i in range(100):
        pose.integrate(straight(0.003 if i % 2 else -0.003))
    assert distance_between(a, pose.snapshot()) == pytest.approx(0.0, abs=1e-9)
    assert pose.path_m == pytest.approx(0.3)


def test_a_cone_ahead_maps_in_front_of_the_car():
    pose = Pose().integrate(straight(1.0))
    assert pose.to_world(2.0, 0.0) == pytest.approx((3.0, 0.0))


def test_a_cone_maps_through_the_cars_heading():
    pose = Pose().integrate(turning(0.0, 90.0))
    x, y = pose.to_world(1.0, 0.0)
    assert (x, y) == pytest.approx((0.0, 1.0), abs=1e-9)


def test_a_landmark_stays_put_while_the_car_drives_past_it():
    """The property the whole map rests on: the same cone, seen from two
    places, lands on one point in the world frame."""
    pose = Pose()
    first = pose.to_world(3.0, 0.5)
    pose.integrate(straight(1.0))
    second = pose.to_world(2.0, 0.5)
    assert second == pytest.approx(first)
