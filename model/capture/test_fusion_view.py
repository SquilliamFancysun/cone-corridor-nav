"""End to end: a synthetic scan and camera frame in, a centerline out.

The unit tests each check one layer against inputs built by hand. This one runs
the layers wired together the way fusion_view.py wires them, against a cone
field that produced both the lidar returns and the boxes -- so a sign error, a
frame mix-up or a units slip anywhere in the chain shows up here even though
every layer passes its own tests.

pipeline_once() is the function under test on purpose: it is the whole
algorithm and it does no I/O, so it is also exactly what a ROS node will call.
"""

import math

import pytest

import fusion_view
from cone_perception import geometry
from cone_perception.cone_classes import CLASS_BLUE, CLASS_RED, CLASS_YELLOW, UNLABELED
from sim import cone_field

CLASS_IDS = {"blue": CLASS_BLUE, "yellow": CLASS_YELLOW, "red": CLASS_RED}
INTR = geometry.intrinsics_from_hfov(416, 234)


def args(**overrides):
    parsed = fusion_view.parse_args([])
    for key, value in overrides.items():
        setattr(parsed, key, value)
    return parsed


def observe(layout, pose, dropout=(), scan_kwargs=None):
    """Both sensors looking at the same cones, from the same place."""
    cones = cone_field.cones_in_car_frame(layout, pose)
    scan = cone_field.synth_scan(cones, **(scan_kwargs or {}))
    dets = cone_field.synth_detections(cones, INTR, CLASS_IDS, dropout=dropout)
    return cones, scan, dets


class FakeDetectionSet:
    """A DetectionSet with a controllable age, so staleness is testable."""

    def __init__(self, detections, age=0.0):
        self.detections = detections
        self.inference_s = 0.05
        self._age = age

    def age(self, now=None):
        return self._age


def run(layout, pose, dropout=(), age=0.0, **arg_overrides):
    _cones, scan, dets = observe(layout, pose, dropout=dropout)
    return fusion_view.pipeline_once(
        scan, FakeDetectionSet(dets, age), cone_field.IDENTITY_CALIBRATION,
        INTR, args(**arg_overrides), now=0.0)


def test_a_straight_corridor_comes_out_as_a_centerline():
    result, bounds, line = run(cone_field.straight_corridor(5.0),
                               cone_field.Pose(0.5, 0.0, 0.0))
    assert result.candidates >= 4, "the lidar found almost nothing"
    assert result.matched >= 2, "nothing was labelled"
    assert len(line.points) >= 2
    for x, y in line.points:
        assert abs(y) < 0.25, f"centerline point ({x:.2f}, {y:.2f}) is off-centre"


def test_the_walls_land_on_the_correct_sides():
    """The end-to-end sign check. Everything looks fine mirrored until a fork."""
    result, bounds, _line = run(cone_field.straight_corridor(5.0),
                                cone_field.Pose(0.5, 0.0, 0.0))
    assert bounds.left and bounds.right
    for cone in bounds.left:
        assert cone.y > 0, "a blue cone landed on the right"
    for cone in bounds.right:
        assert cone.y < 0, "a yellow cone landed on the left"


def test_lidar_and_bbox_ranges_agree_on_every_matched_cone():
    """Two independent range estimates of the same cone, from different sensors."""
    result, _b, _l = run(cone_field.straight_corridor(5.0),
                         cone_field.Pose(0.5, 0.0, 0.0))
    checked = 0
    for cone in result.cones:
        if not cone.labeled or math.isnan(cone.range_bbox):
            continue
        checked += 1
        assert cone.range_disagreement() < 0.3, (
            f"{cone!r} lidar={cone.range_lidar:.2f} bbox={cone.range_bbox:.2f}")
    assert checked >= 2, "no cone had both range channels"


def test_a_cone_the_camera_missed_is_still_reported():
    result, bounds, _line = run(cone_field.straight_corridor(5.0),
                                cone_field.Pose(0.5, 0.0, 0.0),
                                dropout=("yellow",))
    assert bounds.unlabeled, "the dropped wall vanished instead of going grey"
    assert all(c.cone_class != CLASS_YELLOW for c in result.cones)


def test_dropping_one_wall_forces_the_single_boundary_fallback():
    _result, _bounds, line = run(cone_field.straight_corridor(5.0),
                                 cone_field.Pose(0.5, 0.0, 0.0),
                                 dropout=("yellow",))
    assert line.single_boundary_fallback
    assert line.points


def test_stale_detections_leave_every_cone_unlabeled():
    result, _bounds, _line = run(cone_field.straight_corridor(5.0),
                                 cone_field.Pose(0.5, 0.0, 0.0), age=1.0)
    assert result.stale
    assert result.matched == 0
    assert all(c.cone_class == UNLABELED for c in result.cones)


def test_a_red_gate_is_found_between_two_red_cones():
    layout = (cone_field.straight_corridor(4.0)
              + cone_field.gate_pair((2.0, 0.0), 0.0, "gate_j1"))
    _result, _bounds, line = run(layout, cone_field.Pose(0.5, 0.0, 0.0))
    assert line.gates, "the red pair produced no junction midpoint"
    assert line.gates[0].y == pytest.approx(0.0, abs=0.3)


def test_the_full_track_produces_a_line_from_the_start_line():
    _result, _bounds, line = run(cone_field.track_v1(),
                                 cone_field.Pose(0.0, 0.0, 0.0))
    assert len(line.points) >= 2


MIRRORED = {"mirror": True, "angle_offset_deg": 0.0, "chassis_arcs_sensor": []}


def test_a_mirrored_mount_is_invisible_on_a_symmetric_corridor():
    """Why calibrate.py exists, demonstrated rather than asserted in prose.

    A straight corridor driven down the middle is symmetric about the x axis, so
    mirroring the bearing sign maps each blue cone exactly onto a yellow one.
    The cluster positions are unchanged, every box still finds a partner, and
    the picture in Foxglove is correct in every respect except that each cone
    now carries its mirror image's label. Nothing downstream can tell.

    This is the failure that "looks fine on a straight and fails at a junction",
    and it is the reason the bearing sign has to be MEASURED rather than eyeballed.
    """
    _cones, scan, dets = observe(cone_field.straight_corridor(5.0),
                                 cone_field.Pose(0.5, 0.0, 0.0))
    right = fusion_view.pipeline_once(scan, FakeDetectionSet(dets),
                                      cone_field.IDENTITY_CALIBRATION, INTR,
                                      args(), now=0.0)[0]
    wrong = fusion_view.pipeline_once(scan, FakeDetectionSet(dets), MIRRORED,
                                      INTR, args(), now=0.0)[0]
    assert wrong.matched == right.matched, (
        "the symmetric case has become detectable; this test no longer says "
        "what it claims and the docstring needs rewriting")


def test_a_mirrored_mount_puts_the_car_on_the_wrong_side_of_the_corridor():
    """Break the symmetry and the error becomes observable -- but not as a swap.

    Mirroring maps the corridor onto itself, so blue is still on the left and
    yellow still on the right; the colours never look wrong. What moves is the
    CAR: driven along the left of the corridor, a mirrored mount reports it
    along the right. That is the error a junction turns into a wrong turn, and
    it is invisible in every quantity except lateral position.
    """
    pose = cone_field.Pose(0.5, 0.35, 0.0)
    _cones, scan, dets = observe(cone_field.straight_corridor(5.0), pose)

    def centroid_y(calibration):
        result, _bounds, _line = fusion_view.pipeline_once(
            scan, FakeDetectionSet(dets), calibration, INTR, args(), now=0.0)
        return sum(c.y for c in result.cones) / len(result.cones)

    right = centroid_y(cone_field.IDENTITY_CALIBRATION)
    wrong = centroid_y(MIRRORED)

    # The car sits left of the centreline, so the cones it can see average out
    # to its right -- and a mirrored mount reports them the same distance to its
    # left. The exact magnitude depends on which cones the lidar reached, so the
    # claim worth making is the reflection, not the number.
    assert right < -0.1, "the car should see the corridor off to its right"
    assert wrong == pytest.approx(-right, abs=1e-6)


def test_the_status_block_is_serialisable():
    """It goes into a Foxglove channel with a fixed schema; wrong types fail late."""
    result, _bounds, line = run(cone_field.straight_corridor(5.0),
                                cone_field.Pose(0.5, 0.0, 0.0))

    class FakeReader:
        count = 10

        class decoder:
            drop_rate = 0.0

    status = fusion_view.status_of(result, line, FakeReader(),
                                   FakeDetectionSet([]), elapsed=1.0)
    import json
    json.loads(json.dumps(status))
    for key in fusion_view.STATUS_SCHEMA["properties"]:
        assert key in status, f"{key} is in the schema but never populated"
