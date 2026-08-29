"""Clustering, and the conversion into the car frame.

The clusterer itself is exercised by model/capture/test_calibrate.py, which
predates this module and still runs against it through calibrate.py's
re-export. What is tested here is what was added on top: the chassis mask, the
size gate, and the sensor->car bearing conversion, which is the step that puts
cones on the correct side of the car.
"""

import math

import pytest

from cone_perception import clustering
from ld06 import Scan

IDENTITY = {"mirror": False, "angle_offset_deg": 0.0, "chassis_arcs_sensor": []}


def scan_with(objects, step_deg=0.8, background_mm=0):
    """objects = [(sensor_bearing_deg, range_mm, width_deg)]."""
    angles, ranges = [], []
    for i in range(int(round(360.0 / step_deg))):
        angle = i * step_deg
        dist = background_mm
        for bearing, obj_mm, width in objects:
            if abs(clustering.wrap180(angle - bearing)) <= width / 2.0:
                dist = obj_mm
                break
        if dist <= 0:
            continue
        angles.append(angle)
        ranges.append(dist)
    return Scan(t=0.0, angles_deg=angles, ranges_mm=ranges,
                intensities=[100] * len(angles), speed_hz=10.0)


def test_a_cone_becomes_one_candidate_in_the_car_frame():
    # sensor 30 deg with mirror=False, offset=0 -> car bearing -30 deg, so the
    # cone is ahead and to the RIGHT.
    scan = scan_with([(30.0, 1500, 3.0)])
    got = clustering.cone_candidates(scan, IDENTITY)
    assert len(got) == 1
    cone = got[0]
    assert math.degrees(cone.bearing_rad) == pytest.approx(-30.0, abs=1.0)
    assert cone.range_m == pytest.approx(1.5, abs=0.01)
    assert cone.x == pytest.approx(1.5 * math.cos(math.radians(-30)), abs=0.02)
    assert cone.y == pytest.approx(1.5 * math.sin(math.radians(-30)), abs=0.02)
    assert cone.y < 0, "a cone at car bearing -30 must be on the right"


def test_the_mount_sign_mirrors_the_whole_field():
    """The failure calibrate.py exists to prevent, reproduced one level up."""
    scan = scan_with([(30.0, 1500, 3.0)])
    right = clustering.cone_candidates(scan, IDENTITY)[0]
    mirrored = clustering.cone_candidates(
        scan, {"mirror": True, "angle_offset_deg": 0.0})[0]
    assert right.y == pytest.approx(-mirrored.y, abs=1e-6)


def test_angle_offset_rotates_the_field():
    scan = scan_with([(30.0, 1500, 3.0)])
    got = clustering.cone_candidates(
        scan, {"mirror": False, "angle_offset_deg": 90.0})[0]
    assert math.degrees(got.bearing_rad) == pytest.approx(60.0, abs=1.0)


def test_a_calibration_without_a_convention_is_refused():
    scan = scan_with([(30.0, 1500, 3.0)])
    with pytest.raises(ValueError, match="unverified"):
        clustering.cone_candidates(scan, {"chassis_arcs_sensor": []})


def test_chassis_returns_are_masked_out():
    calibration = dict(IDENTITY)
    calibration["chassis_arcs_sensor"] = [
        {"start_deg": 20.0, "end_deg": 162.0, "near_mm": 2, "far_mm": 133,
         "presence": 0.975}]
    # The body fills exactly the arc that was measured off it, and a real cone
    # sits outside.
    scan = scan_with([(91.0, 250, 142.0), (200.0, 1500, 3.0)])
    got = clustering.cone_candidates(scan, calibration)
    assert len(got) == 1
    assert got[0].range_m == pytest.approx(1.5, abs=0.01)


def test_the_chassis_mask_does_not_weld_itself_to_a_real_cone():
    """Masked before clustering, so body returns cannot drag a centroid."""
    calibration = dict(IDENTITY)
    calibration["chassis_arcs_sensor"] = [
        {"start_deg": 20.0, "end_deg": 40.0, "near_mm": 2, "far_mm": 300,
         "presence": 1.0}]
    # 14 deg is what a 6.5 cm cone actually subtends at 0.26 m; a narrower
    # object at this range is not cone-shaped and the width gate says so.
    scan = scan_with([(30.0, 250, 20.0), (48.0, 260, 14.0)])
    got = clustering.cone_candidates(scan, calibration)
    assert len(got) == 1
    assert math.degrees(got[0].bearing_rad) == pytest.approx(-48.0, abs=2.0)


def test_a_wall_is_too_wide_to_be_a_cone():
    scan = scan_with([(30.0, 1500, 40.0)])
    assert clustering.cone_candidates(scan, IDENTITY) == []


def test_something_beyond_the_range_limit_is_dropped():
    scan = scan_with([(30.0, 9000, 1.0)])
    assert clustering.cone_candidates(scan, IDENTITY) == []


def test_a_single_stray_return_is_not_a_cone():
    scan = scan_with([(30.0, 1500, 0.1)])
    assert clustering.cone_candidates(scan, IDENTITY) == []


def test_a_two_point_cluster_survives_the_width_floor():
    """The far-cone case: two returns spanning one step is all the sensor gives.

    A minimum-width gate applied to these would reject exactly the distant cones
    the corridor layer is straining to reach.
    """
    # At 1 m one angular step spans 1.4 cm, well under the floor.
    scan = scan_with([(30.0, 1000, 1.2)])
    got = clustering.cone_candidates(scan, IDENTITY)
    assert len(got) == 1
    assert got[0].points == 2
    assert got[0].width_m < clustering.MIN_CONE_WIDTH_M


def test_width_is_reported_in_metres_at_the_measured_range():
    scan = scan_with([(0.0, 2000, 3.2)])
    got = clustering.cone_candidates(scan, IDENTITY)[0]
    # 3.2 deg of object at 0.8 deg per step is five returns spanning 3.2 deg.
    assert got.width_m == pytest.approx(math.radians(3.2) * 2.0, abs=0.02)
