"""Tests for the bearing solver, on synthesized scans with a known answer.

The point of calibrate.py is to replace a judgement call with arithmetic, so
the arithmetic is what has to be checked: a solver that confidently returns the
wrong sign is worse than the eyeball procedure it replaced.
"""

import json

import pytest

import calibrate
from ld06 import Scan


def sensor_bearing_for(car_bearing_deg, mirror, offset_deg):
    """Invert car = sign*sensor + offset. sign is +-1, so it is its own inverse."""
    sign = 1.0 if mirror else -1.0
    return calibrate.wrap360(sign * (car_bearing_deg - offset_deg))


def make_scan(objects, background_mm=4000, step_deg=0.8, t=0.0):
    """A revolution containing `objects` = [(sensor_bearing, range_mm, width)].

    Everything not covered by an object returns the background, which stands in
    for a distant wall — near enough to reality that clustering has to actually
    separate things rather than find the only returns present.
    """
    angles, ranges = [], []
    steps = int(round(360.0 / step_deg))
    for i in range(steps):
        angle = i * step_deg
        dist = background_mm
        for bearing, obj_mm, width in objects:
            if abs(calibrate.wrap180(angle - bearing)) <= width / 2.0:
                dist = obj_mm
                break
        angles.append(angle)
        ranges.append(dist)
    return Scan(t=t, angles_deg=angles, ranges_mm=ranges,
                intensities=[100] * len(angles), speed_hz=10.0)


def scans_with_cone(car_bearing, mirror, offset, count=10, range_mm=1000,
                    chassis=None):
    bearing = sensor_bearing_for(car_bearing, mirror, offset)
    objects = [(bearing, range_mm, 6.0)]
    if chassis is not None:
        objects.append(chassis)
    return [make_scan(objects, t=i * 0.1) for i in range(count)]


# --- clustering ---------------------------------------------------------

def test_cluster_separates_cone_from_background():
    scan = make_scan([(30.0, 1000, 6.0)])
    clusters = calibrate.cluster_scan(scan.angles_deg, scan.ranges_mm)
    cone = [c for c in clusters if c.range_mm < 2000]
    assert len(cone) == 1
    assert cone[0].bearing_deg == pytest.approx(30.0, abs=1.0)
    assert cone[0].range_mm == pytest.approx(1000, abs=1)
    assert cone[0].points >= 6


def test_cluster_across_zero_is_one_object():
    """A cone on the sensor's zero arrives as two runs in a sorted list."""
    scan = make_scan([(0.0, 1000, 8.0)])
    clusters = calibrate.cluster_scan(scan.angles_deg, scan.ranges_mm)
    cone = [c for c in clusters if c.range_mm < 2000]
    assert len(cone) == 1, "the cone was split at the wrap"
    assert abs(calibrate.wrap180(cone[0].bearing_deg)) < 1.0


def test_cluster_drops_zero_range_no_returns():
    scan = Scan(t=0.0, angles_deg=[10.0, 11.0, 12.0], ranges_mm=[0, 0, 0],
                intensities=[0, 0, 0], speed_hz=10.0)
    assert calibrate.cluster_scan(scan.angles_deg, scan.ranges_mm) == []


def test_find_candidates_reports_every_rival():
    scan = make_scan([(30.0, 1000, 6.0), (150.0, 1050, 10.0)])
    clusters = calibrate.cluster_scan(scan.angles_deg, scan.ranges_mm)
    hits = calibrate.find_candidates(clusters, 600, 1400)
    assert len(hits) == 2
    assert hits[0].points >= hits[1].points, "candidates are not ranked"


# --- per-pose measurement ----------------------------------------------

def test_measure_pose_recovers_the_bearing():
    scans = scans_with_cone(45.0, mirror=False, offset=0.0)
    obs = calibrate.measure_pose(scans, 45.0, 1000, 400)
    assert obs.scans == len(scans)
    assert obs.bearing_deg == pytest.approx(315.0, abs=1.0)
    assert obs.spread_deg < 0.5
    assert obs.ambiguous_scans == 0


def test_measure_pose_flags_a_rival_object():
    scans = [make_scan([(315.0, 1000, 6.0), (100.0, 1100, 6.0)], t=i * 0.1)
             for i in range(5)]
    obs = calibrate.measure_pose(scans, 45.0, 1000, 400)
    assert obs.ambiguous_scans == 5


def test_measure_pose_returns_none_when_nothing_is_in_range():
    scans = scans_with_cone(45.0, mirror=False, offset=0.0, range_mm=3000)
    assert calibrate.measure_pose(scans, 45.0, 1000, 400) is None


# --- the solver ---------------------------------------------------------

@pytest.mark.parametrize("mirror", [False, True])
@pytest.mark.parametrize("offset", [0.0, 12.5, -30.0, 175.0])
def test_solver_recovers_the_convention(mirror, offset):
    poses = [45.0, -45.0]
    observations = []
    for pose in poses:
        scans = scans_with_cone(pose, mirror, offset)
        observations.append(calibrate.measure_pose(scans, pose, 1000, 400))

    solution = calibrate.solve_convention(observations)
    assert solution.mirror is mirror
    assert calibrate.wrap180(solution.angle_offset_deg - offset) == pytest.approx(
        0.0, abs=1.0)
    assert solution.residual_deg < 1.0
    assert solution.decisive


def test_solver_maps_measurements_back_to_where_the_cones_were():
    poses = [45.0, -45.0, 90.0]
    observations = [
        calibrate.measure_pose(scans_with_cone(p, True, 20.0), p, 1000, 400)
        for p in poses]
    solution = calibrate.solve_convention(observations)
    for obs in observations:
        got = solution.car_bearing(obs.bearing_deg)
        assert calibrate.wrap180(got - obs.expected_deg) == pytest.approx(0.0, abs=1.0)


def test_one_pose_is_refused():
    """The whole reason this module exists: one cone cannot decide the sign."""
    obs = calibrate.measure_pose(scans_with_cone(45.0, False, 0.0), 45.0, 1000, 400)
    with pytest.raises(ValueError, match="two cone poses"):
        calibrate.solve_convention([obs])


def test_poses_too_close_together_are_refused():
    observations = [
        calibrate.measure_pose(scans_with_cone(p, False, 0.0), p, 1000, 400)
        for p in (40.0, 45.0)]
    with pytest.raises(ValueError, match="determined"):
        calibrate.solve_convention(observations)


def test_a_cone_dead_ahead_cannot_decide_the_sign():
    """A point on the x axis is its own reflection — README says so; prove it."""
    truth_mirror, truth_offset = True, 0.0
    observations = [
        calibrate.measure_pose(scans_with_cone(p, truth_mirror, truth_offset),
                               p, 1000, 400)
        for p in (0.0, 180.0)]
    solution = calibrate.solve_convention(observations)
    # Both signs fit an on-axis pair exactly, so the fit must not claim to have
    # settled it, whichever one it happened to pick.
    assert not solution.decisive


def test_flags_are_pasteable():
    observations = [
        calibrate.measure_pose(scans_with_cone(p, True, 12.0), p, 1000, 400)
        for p in (45.0, -45.0)]
    flags = calibrate.solve_convention(observations).flags()
    assert flags.startswith("--mirror")
    assert "--angle-offset 12" in flags


# --- chassis arc --------------------------------------------------------

def test_chassis_arc_is_found_where_the_car_is():
    scans = scans_with_cone(45.0, False, 0.0, count=20,
                            chassis=(184.0, 250, 16.0))
    arcs = calibrate.chassis_arcs(scans)
    assert len(arcs) == 1
    assert arcs[0].mid_deg == pytest.approx(184.0, abs=2.0)
    assert arcs[0].width_deg == pytest.approx(16.0, abs=3.0)
    assert arcs[0].near_mm == pytest.approx(250, abs=1)


def test_a_passer_by_is_not_the_chassis():
    """Near, but not on every revolution, so persistence must exclude it."""
    scans = []
    for i in range(20):
        objects = [(315.0, 1000, 6.0), (184.0, 250, 16.0)]
        if i < 4:
            objects.append((90.0, 300, 20.0))
        scans.append(make_scan(objects, t=i * 0.1))
    arcs = calibrate.chassis_arcs(scans)
    assert all(abs(calibrate.wrap180(a.mid_deg - 90.0)) > 20.0 for a in arcs)


def test_no_chassis_in_the_scan_plane_is_not_an_error():
    assert calibrate.chassis_arcs(scans_with_cone(45.0, False, 0.0)) == []
    assert calibrate.chassis_arcs([]) == []


# --- persistence --------------------------------------------------------

def test_save_and_load_round_trip(tmp_path):
    observations = [
        calibrate.measure_pose(scans_with_cone(p, True, 7.5), p, 1000, 400)
        for p in (45.0, -45.0)]
    solution = calibrate.solve_convention(observations)
    arcs = calibrate.chassis_arcs(
        scans_with_cone(45.0, True, 7.5, count=20, chassis=(184.0, 250, 16.0)))
    record = calibrate.build_record(solution, arcs=arcs,
                                    mount={"x": 0.1, "y": 0.0, "z": 0.12,
                                           "yaw_deg": 0.0},
                                    git_commit="abc1234", target_mm=1000.0)
    path = calibrate.save(record, str(tmp_path / "calibration.json"))

    back = calibrate.load(path)
    assert back["mirror"] is True
    assert back["angle_offset_deg"] == pytest.approx(7.5, abs=1.0)
    assert back["chassis_arcs_sensor"][0]["near_mm"] == 250
    assert back["mount"]["z"] == 0.12
    assert len(back["observations"]) == 2
    assert json.loads(open(path).read()) == back


def test_load_missing_file_is_not_an_error(tmp_path):
    assert calibrate.load(str(tmp_path / "nope.json")) is None


def test_load_rejects_a_file_that_is_not_a_calibration(tmp_path):
    path = tmp_path / "calibration.json"
    path.write_text('{"hello": "world"}')
    with pytest.raises(ValueError, match="re-run --calibrate"):
        calibrate.load(str(path))


# --- angle helpers ------------------------------------------------------

def test_circular_mean_survives_the_wrap():
    assert calibrate.circular_mean([359.0, 1.0]) == pytest.approx(0.0, abs=0.1)
    assert calibrate.circular_mean([10.0, 350.0]) == pytest.approx(0.0, abs=0.1)


def test_circular_mean_of_antipodal_bearings_is_undefined():
    assert calibrate.circular_mean([0.0, 180.0]) is None


def test_circular_spread_is_measured_across_the_wrap():
    assert calibrate.circular_spread([358.0, 2.0]) == pytest.approx(2.0, abs=0.1)
