"""The transform, and the refusals.

Everything else here is bookkeeping. What can be quietly wrong is the frame:
a map built with the rotation dropped, or applied backwards, still produces a
plausible-looking scatter of cones -- it just is not the track. So the
load-bearing test drives a synthetic car past a known layout and asserts the
map comes back as that layout.

The refusals matter nearly as much. This script's whole job is to say whether
odometry can be trusted, so a version that reports a small residual over a run
it should have declined is worse than one that crashes.
"""

import math

import pytest

import map_from_log
from cone_perception.cone_classes import CLASS_BLUE, CLASS_YELLOW, UNLABELED

# Two straight walls 1.5 m apart, cones every 0.75 m -- a corridor.
WALLS = ([(0.75 * i, +0.75, CLASS_BLUE) for i in range(1, 9)]
         + [(0.75 * i, -0.75, CLASS_YELLOW) for i in range(1, 9)])


def drive(walls=WALLS, ticks=30, step=0.15, sees_m=3.0, jump_at=None):
    """A car driving +x along the corridor, logging what it can see."""
    rows = []
    for i in range(ticks):
        x = i * step
        visible = []
        for wx, wy, cls in walls:
            dx = wx - x
            if 0.0 < dx <= sees_m:
                visible.append(f"{dx:.3f},{wy:.3f},{cls}")
        rows.append({
            "t": i * 0.1, "pose_x": x, "pose_y": 0.0, "pose_yaw_deg": 0.0,
            "pose_jumps": 1 if (jump_at is not None and i >= jump_at) else 0,
            "odo_forward_m": step, "odo_lateral_m": 0.0, "odo_yaw_deg": 0.0,
            "odo_pairs": 4, "cones_xy": ";".join(visible),
        })
    return rows


# --- parsing ------------------------------------------------------------

def test_a_cone_field_round_trips():
    assert map_from_log.parse_cones("1.000,0.750,0") == [(1.0, 0.75, 0)]


def test_an_empty_field_is_no_cones_rather_than_an_error():
    assert map_from_log.parse_cones("") == []
    assert map_from_log.parse_cones(None) == []


def test_a_malformed_entry_is_skipped_not_fatal():
    """A run killed mid-write is the run worth reading."""
    assert map_from_log.parse_cones("1.0,2.0,0;garbage;3.0,4.0,4") == [
        (1.0, 2.0, 0), (3.0, 4.0, 4)]


# --- the transform ------------------------------------------------------

def test_it_reconstructs_the_corridor_it_drove():
    """The load-bearing one. Drop the rotation, or apply it backwards, and
    this scatters instead of landing on the walls."""
    landmarks, path, used = map_from_log.build(drive())
    assert used == 30
    assert len(landmarks) == len(WALLS)
    for wx, wy, _c in WALLS:
        assert min(math.hypot(m.x - wx, m.y - wy) for m in landmarks) < 0.05


def test_a_cone_seen_from_twenty_places_is_one_landmark():
    landmarks, _path, _used = map_from_log.build(drive())
    assert max(m.sightings for m in landmarks) > 5


def test_the_rotation_is_applied_at_all():
    """A car facing +y must map a cone ahead of it to +y, not +x."""
    rows = [{"pose_x": 0.0, "pose_y": 0.0, "pose_yaw_deg": 90.0,
             "cones_xy": f"2.000,0.000,{CLASS_BLUE}", "pose_jumps": 0}] * 3
    landmarks, _path, _used = map_from_log.build(rows, min_sightings=1)
    assert landmarks[0].x == pytest.approx(0.0, abs=1e-6)
    assert landmarks[0].y == pytest.approx(2.0)


def test_a_landmark_seen_once_is_noise_and_is_dropped():
    rows = drive(ticks=1)
    landmarks, _path, _used = map_from_log.build(rows)
    assert landmarks == []


def test_the_colour_is_the_one_the_camera_actually_gave():
    """Geometry never disagrees with the camera, it just never speaks -- so a
    cone labelled twice out of thirty ticks is that colour, not unlabelled."""
    mark = map_from_log.Landmark(0.0, 0.0, UNLABELED)
    for _ in range(20):
        mark.absorb(0.0, 0.0, UNLABELED)
    mark.absorb(0.0, 0.0, CLASS_BLUE)
    assert mark.cone_class == CLASS_BLUE


# --- the refusals -------------------------------------------------------

def test_mapping_stops_at_the_first_lift():
    """Everything after one is in a different frame, and mapping through it
    would draw a second offset copy of the course that reads as drift."""
    _landmarks, _path, used = map_from_log.build(drive(jump_at=10))
    assert used == 10


def test_a_log_with_no_cone_positions_is_declined(capsys):
    """Every trial log on disk predates the field. They cannot be mapped
    retrospectively and must not be mapped approximately."""
    assert map_from_log.report([{"t": 0.0, "pose_x": 0.0}], "old.jsonl") == 1
    assert "never recorded" in capsys.readouterr().out


def test_an_empty_log_is_declined():
    assert map_from_log.report([], "empty.jsonl") == 1


# --- scoring against the tape -------------------------------------------

def test_a_perfect_map_scores_near_zero():
    landmarks, _path, _used = map_from_log.build(drive())
    matched, unmatched = map_from_log.residuals(landmarks, WALLS)
    assert unmatched == 0
    assert max(matched) < 0.05


def test_drift_is_counted_rather_than_gated_away():
    """The failure this guards: dropping far landmarks would report a tiny
    residual over the handful that happened to stay put, which is exactly
    backwards -- the drift IS the finding."""
    landmarks, _path, _used = map_from_log.build(drive())
    # Shifted sideways, not along: an along-track shift leaves the far
    # landmarks inside the gate and would pass for the wrong reason.
    shifted = [(x, y + 5.0, c) for x, y, c in WALLS]
    matched, unmatched = map_from_log.residuals(landmarks, shifted)
    assert unmatched == len(landmarks)
    assert matched == []


# --- re-integration under a deadband ------------------------------------

def test_the_deadband_can_be_re_run_from_the_logged_steps():
    """What makes the deadband question measurable instead of arguable."""
    rows = drive(step=0.004, ticks=20)
    loose = map_from_log.poses(rows, deadband_m=0.0)
    tight = map_from_log.poses(rows, deadband_m=0.008)
    assert loose[-1][0] == pytest.approx(20 * 0.004)
    assert tight[-1][0] == 0.0
