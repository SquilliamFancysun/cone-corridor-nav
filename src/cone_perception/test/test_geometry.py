"""Camera geometry: signs, the lever arm, and when range_bbox refuses to answer.

Signs are the whole risk here. Every quantity is an angle, they all look
plausible mirrored, and a sign error puts every label on the wrong side of the
car -- which is invisible on a straight corridor and catastrophic at a fork.
"""

import math

import pytest

from cone_perception import extrinsics, geometry

INTR = geometry.intrinsics_from_hfov(640, 400)


def box(u, h=0.1, cls=0, clipped=False, confidence=0.9):
    return geometry.Detection(cls=cls, confidence=confidence, u=u, v=0.5,
                              w=h * 0.5, h=h, clipped=clipped)


def test_a_box_left_of_centre_has_a_positive_bearing():
    """Left positive, per REP-103 and depth_view.py's Projector."""
    assert geometry.detection_bearing(box(0.25), INTR) > 0


def test_a_box_right_of_centre_has_a_negative_bearing():
    assert geometry.detection_bearing(box(0.75), INTR) < 0


def test_a_centred_box_is_dead_ahead():
    assert geometry.detection_bearing(box(0.5), INTR) == pytest.approx(0.0, abs=1e-9)


def test_the_frame_edges_land_at_the_half_field_of_view():
    edge = geometry.detection_bearing(box(0.0), INTR)
    assert math.degrees(edge) == pytest.approx(extrinsics.CAMERA_HFOV_DEG / 2.0,
                                               abs=0.5)


def test_a_point_to_the_left_is_seen_to_the_left():
    assert geometry.bearing_from_camera(2.0, 1.0) > 0
    assert geometry.bearing_from_camera(2.0, -1.0) < 0


def test_the_lever_arm_shifts_the_bearing_of_a_near_cone():
    """The camera sits 5 cm behind the lidar, so it sees a near cone differently.

    Small, but the association gate is only 4 deg wide, and this is free.
    """
    x, y = 1.5, 0.75
    from_camera = geometry.bearing_from_camera(x, y)
    from_lidar = math.atan2(y, x)
    assert from_camera != pytest.approx(from_lidar, abs=1e-6)
    # 0.05 m of parallax at 1.5 m is under 2 deg, and toward the camera's rear
    # the cone appears LESS off-axis, not more.
    assert abs(math.degrees(from_camera - from_lidar)) < 2.0
    assert from_camera < from_lidar


def test_the_lever_arm_stops_mattering_at_range():
    near = abs(geometry.bearing_from_camera(1.0, 0.5) - math.atan2(0.5, 1.0))
    far = abs(geometry.bearing_from_camera(5.0, 2.5) - math.atan2(2.5, 5.0))
    assert far < near


def test_bearing_round_trips_through_a_detection():
    """A cone at a known place, imaged, must come back at the same bearing.

    This is the loop the association closes, so if it does not hold nothing
    downstream can work.
    """
    x, y = 3.0, 0.75
    expected = geometry.bearing_from_camera(x, y)
    u_px = INTR.cx - math.tan(expected) * INTR.fx
    got = geometry.detection_bearing(box(u_px / INTR.width), INTR)
    assert got == pytest.approx(expected, abs=1e-6)


def test_range_from_bbox_falls_off_with_box_height():
    near = geometry.range_from_bbox(box(0.5, h=0.4), INTR)
    far = geometry.range_from_bbox(box(0.5, h=0.1), INTR)
    assert far == pytest.approx(4 * near, rel=1e-6)


def test_range_from_bbox_matches_the_pinhole_arithmetic():
    h = 0.2
    got = geometry.range_from_bbox(box(0.5, h=h), INTR)
    assert got == pytest.approx(
        INTR.fy * extrinsics.CONE_HEIGHT_M / (h * INTR.height), rel=1e-9)


def test_a_clipped_box_refuses_to_estimate_range():
    """A cone cut off by the frame edge has a meaningless height."""
    assert math.isnan(geometry.range_from_bbox(box(0.5, h=0.2, clipped=True), INTR))


def test_range_is_unavailable_when_the_cone_height_was_never_measured():
    assert math.isnan(geometry.range_from_bbox(box(0.5), INTR, cone_height_m=None))


def test_a_zero_height_box_does_not_divide_by_zero():
    assert math.isnan(geometry.range_from_bbox(box(0.5, h=0.0), INTR))


def test_the_field_of_view_test_excludes_what_is_off_frame():
    assert geometry.in_camera_fov(0.0)
    assert not geometry.in_camera_fov(math.radians(60.0))


def test_the_field_of_view_margin_shrinks_the_window():
    edge = extrinsics.camera_half_fov_rad(0.0) - 1e-6
    assert geometry.in_camera_fov(edge, margin_deg=0.0)
    assert not geometry.in_camera_fov(edge, margin_deg=5.0)


def test_wrap_pi_folds_the_long_way_round():
    assert geometry.wrap_pi(math.pi * 1.5) == pytest.approx(-math.pi / 2)
    assert geometry.wrap_pi(-math.pi * 1.5) == pytest.approx(math.pi / 2)
