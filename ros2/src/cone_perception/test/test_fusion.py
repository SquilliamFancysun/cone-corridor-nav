"""Association: the right label on the right cluster, and no label when unsure.

The two failure modes worth guarding are opposite. Labelling a cluster with a
neighbouring cone's box puts a wall on the wrong side of the corridor; refusing
to label anything makes the car blind. Both are tested, along with the counters
the harness uses to tell which one is happening.
"""

import math

import pytest

from cone_perception import fusion, geometry
from cone_perception.clustering import ConeCandidate
from cone_perception.cone_classes import CLASS_BLUE, CLASS_YELLOW, UNLABELED

INTR = geometry.intrinsics_from_hfov(640, 400)


def candidate(x, y, points=4):
    r = math.hypot(x, y)
    return ConeCandidate(x=x, y=y, range_m=r, bearing_rad=math.atan2(y, x),
                         width_m=0.06, points=points, sensor_bearing_deg=0.0)


def detection_at(x, y, cls, confidence=0.9, height_m=0.1778):
    """A box placed where a cone at (x, y) in base_link would actually appear."""
    bearing = geometry.bearing_from_camera(x, y)
    cam_x, cam_y, _ = fusion.extrinsics.CAMERA_IN_BASE
    distance = math.hypot(x - cam_x, y - cam_y)
    u_px = INTR.cx - math.tan(bearing) * INTR.fx
    h = (INTR.fy * height_m / distance) / INTR.height
    return geometry.Detection(cls=cls, confidence=confidence,
                              u=u_px / INTR.width, v=0.5, w=h * 0.5, h=h,
                              clipped=False)


def test_one_cone_gets_its_own_label():
    cand = [candidate(2.0, 0.75)]
    dets = [detection_at(2.0, 0.75, CLASS_BLUE)]
    result = fusion.associate(cand, dets, INTR)
    assert result.matched == 1
    assert result.cones[0].cone_class == CLASS_BLUE
    assert result.cones[0].confidence == pytest.approx(0.9)


def test_position_always_comes_from_the_lidar():
    """The camera contributes a label and nothing else."""
    cand = [candidate(2.0, 0.75)]
    result = fusion.associate(cand, [detection_at(2.0, 0.75, CLASS_BLUE)], INTR)
    assert result.cones[0].x == pytest.approx(2.0)
    assert result.cones[0].y == pytest.approx(0.75)
    assert result.cones[0].range_lidar == pytest.approx(math.hypot(2.0, 0.75))


def test_two_walls_do_not_swap_labels():
    """The failure that matters: blue on the right and yellow on the left."""
    cand = [candidate(2.0, 0.75), candidate(2.0, -0.75)]
    dets = [detection_at(2.0, -0.75, CLASS_YELLOW),
            detection_at(2.0, 0.75, CLASS_BLUE)]
    result = fusion.associate(cand, dets, INTR)
    assert result.matched == 2
    by_side = {c.y > 0: c.cone_class for c in result.cones}
    assert by_side[True] == CLASS_BLUE
    assert by_side[False] == CLASS_YELLOW


def test_a_cluster_with_no_box_is_published_unlabeled():
    """A cone the detector missed must stay visible, not vanish."""
    result = fusion.associate([candidate(2.0, 0.75)], [], INTR)
    assert len(result.cones) == 1
    assert result.cones[0].cone_class == UNLABELED
    assert not result.cones[0].labeled
    assert result.cones[0].range_lidar == pytest.approx(math.hypot(2.0, 0.75))


def test_a_cluster_outside_the_camera_is_not_a_detector_failure():
    """The lidar sweeps ~218 deg and the camera 69, so this is the usual case."""
    behind = candidate(0.5, 3.0)
    result = fusion.associate([behind], [], INTR)
    assert result.out_of_fov == 1
    assert result.unmatched_in_fov == 0
    assert result.cones[0].cone_class == UNLABELED


def test_a_box_further_off_than_the_gate_is_refused():
    cand = [candidate(3.0, 0.0)]
    dets = [detection_at(3.0, 1.2, CLASS_BLUE)]  # ~22 deg away
    result = fusion.associate(cand, dets, INTR)
    assert result.matched == 0
    assert result.unmatched_in_fov == 1
    assert result.unmatched_detections == 1
    assert result.cones[0].cone_class == UNLABELED


def test_one_box_cannot_label_two_clusters():
    cand = [candidate(2.0, 0.0), candidate(2.2, 0.02)]
    dets = [detection_at(2.0, 0.0, CLASS_BLUE)]
    result = fusion.associate(cand, dets, INTR)
    assert result.matched == 1
    assert sum(c.labeled for c in result.cones) == 1


def test_the_closer_bearing_wins_a_contested_box():
    cand = [candidate(2.0, 0.6), candidate(2.0, 0.0)]
    dets = [detection_at(2.0, 0.0, CLASS_YELLOW)]
    result = fusion.associate(cand, dets, INTR)
    labeled = [c for c in result.cones if c.labeled]
    assert len(labeled) == 1
    assert labeled[0].y == pytest.approx(0.0)


def test_stale_detections_are_not_used_at_all():
    """A label from 400 ms ago is more likely on the wrong cone than the right one."""
    cand = [candidate(2.0, 0.75)]
    dets = [detection_at(2.0, 0.75, CLASS_BLUE)]
    result = fusion.associate(cand, dets, INTR, detection_age_s=0.4)
    assert result.stale
    assert result.matched == 0
    assert result.cones[0].cone_class == UNLABELED


def test_fresh_detections_are_used():
    cand = [candidate(2.0, 0.75)]
    dets = [detection_at(2.0, 0.75, CLASS_BLUE)]
    result = fusion.associate(cand, dets, INTR, detection_age_s=0.1)
    assert not result.stale
    assert result.matched == 1


def test_range_bbox_agrees_with_the_lidar_on_a_correct_match():
    """Both channels describe the same cone, so they must land close."""
    cand = [candidate(2.0, 0.0)]
    dets = [detection_at(2.0, 0.0, CLASS_BLUE)]
    cone = fusion.associate(cand, dets, INTR).cones[0]
    assert cone.range_disagreement() < 0.1


def test_the_wall_behind_a_cone_is_refused_its_label():
    """A box at 2 m against a cluster at 4 m: right bearing, wrong world.
    range_bbox was recorded "to DISAGREE" from the start, and once the
    flattened pitch let the world behind the cones into the scan, unarmed
    disagreement meant phantom boundary cones at the wall. The pairing is now
    refused and counted."""
    cand = [candidate(4.0, 0.0)]
    dets = [detection_at(2.0, 0.0, CLASS_BLUE)]
    result = fusion.associate(cand, dets, INTR)
    assert not result.cones[0].labeled
    assert result.range_rejected == 1


def test_an_honest_range_wobble_is_not_refused():
    """The gate is for walls, not for the box estimate's own +-10-15%."""
    cand = [candidate(2.3, 0.0)]
    dets = [detection_at(2.0, 0.0, CLASS_BLUE)]
    result = fusion.associate(cand, dets, INTR)
    assert result.cones[0].labeled
    assert result.range_rejected == 0


def test_stereo_range_is_never_claimed():
    """No stereo is run, and LabeledCone.msg permits NaN."""
    cand = [candidate(2.0, 0.0)]
    cone = fusion.associate(cand, [detection_at(2.0, 0.0, CLASS_BLUE)],
                            INTR).cones[0]
    assert math.isnan(cone.range_stereo)


def test_the_counters_add_up():
    cand = [candidate(2.0, 0.75), candidate(2.0, -0.75), candidate(0.5, 3.0)]
    dets = [detection_at(2.0, 0.75, CLASS_BLUE)]
    result = fusion.associate(cand, dets, INTR)
    assert result.candidates == 3
    assert result.matched + result.unmatched_in_fov + result.out_of_fov == 3
    assert len(result.cones) == 3
    assert result.as_dict()["detections"] == 1
