"""What drive_junction.py refuses, and that it wires the manoeuvre up.

The loop itself needs a car. What can be tested at a desk is the argument
handling -- which is where the refusals live -- and that the pipeline actually
threads the state machine, which a synthetic scan can show.
"""

import math
import os

import pytest

import drive_junction
from cone_nav.guidance import goal_stop
from cone_nav.topology import topo_state

ROUTE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "..", "data", "routes", "route_v1.txt")
BASE = ["--dry-run", "--no-deadman", "--route", os.path.normpath(ROUTE)]


# --- arguments ----------------------------------------------------------

def test_the_route_is_loaded_and_validated():
    args = drive_junction.parse_args(BASE)
    assert args.route_turns == ["left", "right"]


def test_a_route_is_required():
    """Without one the car has no way to choose a branch, and choosing wrong at
    a junction is worse than not driving."""
    with pytest.raises(SystemExit):
        drive_junction.parse_args(["--dry-run", "--no-deadman"])


def test_a_bad_route_file_is_refused_before_the_camera_opens(tmp_path):
    bad = tmp_path / "bad.txt"
    bad.write_text("left\nstraight\n", encoding="utf-8")
    with pytest.raises(SystemExit):
        drive_junction.parse_args(["--dry-run", "--no-deadman",
                                   "--route", str(bad)])


def test_no_camera_is_fatal_rather_than_a_warning():
    """drive_corridor.py only warns, and its own help text says --no-camera is
    'WRONG at a fork'. The gate is red; geometry cannot infer red at all."""
    with pytest.raises(SystemExit):
        drive_junction.parse_args(BASE + ["--no-camera"])


def test_the_shared_arguments_are_the_corridor_ones():
    """They come off drive_corridor.build_parser rather than being restated, so
    the two scripts cannot drift on lookahead, duty, ports or the deadman."""
    args = drive_junction.parse_args(BASE)
    corridor = drive_corridor_args()
    for name in ("lookahead", "max_duty", "smooth_window", "bearing_gate",
                 "vesc_port", "joystick", "invert_steering", "max_range"):
        assert getattr(args, name) == getattr(corridor, name)


def drive_corridor_args():
    import drive_corridor
    return drive_corridor.parse_args(["--dry-run", "--no-deadman"])


def test_the_deadman_rule_is_inherited():
    with pytest.raises(SystemExit):
        drive_junction.parse_args(["--route", os.path.normpath(ROUTE),
                                   "--no-deadman"])


# --- the status record --------------------------------------------------

def test_the_status_record_extends_the_corridor_one():
    import drive_corridor
    corridor = set(drive_corridor.DRIVE_STATUS_SCHEMA["properties"])
    junction = set(drive_junction.JUNCTION_STATUS_SCHEMA["properties"])
    assert corridor < junction
    assert {"topo_state", "turn", "gate_range_m", "branch_cones_dropped",
            "travelled_m"} <= junction


def test_status_of_reports_the_manoeuvre():
    from cone_nav.guidance.route_exec import RouteCursor

    topo = topo_state.TopoState(RouteCursor(["left", "right"]))
    record = drive_junction.status_of({"duty": 0.1}, topo, None, 7)
    assert record["duty"] == 0.1
    assert record["topo_state"] == topo_state.FOLLOW
    assert record["turn"] == "left"
    assert record["branch_cones_dropped"] == 7
    assert record["route_remaining"] == 2
    assert set(record) - {"duty"} <= set(
        drive_junction.JUNCTION_STATUS_SCHEMA["properties"])


def test_reds_seen_is_reported_even_when_there_is_no_triple():
    """The ways stage 3 finds nothing -- no red detected at all, two of three
    recovered, three recovered from too far back -- are different problems with
    different fixes, and a log that only records whole triples cannot tell them
    apart."""
    from cone_nav.topology import gate_detect
    from cone_perception.cone_classes import CLASS_RED
    from cone_perception.fusion import LabeledCone

    def red(x, y):
        return LabeledCone(CLASS_RED, 0.9, x, y,
                           range_lidar=math.hypot(x, y), points=4)

    survey = gate_detect.survey([red(2.0, 1.35), red(2.0, 0.0)])
    record = drive_junction.status_of({"duty": 0.0}, None, None, 0, survey)
    assert record["reds_seen"] == 2
    assert record["gate_live"] is False
    assert record["gate_reason"] == gate_detect.CROWDED


def test_a_car_standing_too_far_back_is_distinguishable_in_the_log():
    """`reds_seen` counts SLANT range, so three cones in plain view can log as
    one. Without `reds_in_view` beside it that reads as a mis-laid track."""
    from cone_nav.topology import gate_detect
    from cone_perception.cone_classes import CLASS_RED
    from cone_perception.fusion import LabeledCone

    reds = [LabeledCone(CLASS_RED, 0.9, 2.75, y, range_lidar=1.0, points=4)
            for y in (1.35, 0.0, -1.35)]
    record = drive_junction.status_of({"duty": 0.0}, None, None, 0,
                                      gate_detect.survey(reds))
    assert record["reds_in_view"] == 3
    assert record["reds_seen"] < 3
    assert record["gate_reason"] == gate_detect.DISTANCE
    # The gaps are measured anyway, which is what says the tape work is fine.
    assert record["red_gaps_m"] == "1.35/1.35"
    assert record["reds_m"].startswith("2.75/")


def test_status_of_survives_having_no_state_machine():
    record = drive_junction.status_of({"duty": 0.0}, None, None, 0)
    assert record["topo_state"] == ""
    assert record["gate_range_m"] == 0.0
    assert record["reds_in_view"] == 0
    assert record["gate_reason"] == ""


# --- the travel estimate ------------------------------------------------

def test_a_dry_run_measures_its_travel_instead_of_assuming_it():
    """--dry-run pins the duty to zero, so travel cannot come from the motor.
    It used to come from an assumed push speed, which was exactly as honest
    as the operator's pace was close to the assumption -- 0.13 m/s against an
    assumed 0.5 declared a junction passed 1.94 m before the gate. Now it is
    scan-matched ego motion, at whatever pace the car actually moves."""
    from cone_perception import ego_motion

    step = ego_motion.Step(0.013, 0.001, 0.002, pairs=6)
    travel, yaw = drive_junction.dry_run_travel(step)
    assert travel == 0.013
    assert yaw == 0.002


def test_standing_still_accrues_no_distance():
    """Cluster jitter must not random-walk travelled_m upward while the car
    stands in the window admiring its own detection -- topo_state clamps
    negative travel, so unfiltered noise only ever adds."""
    from cone_perception import ego_motion

    jitter = ego_motion.Step(0.004, -0.002, 0.0, pairs=6)
    travel, _yaw = drive_junction.dry_run_travel(jitter)
    assert travel == 0.0


def test_no_measurement_reads_as_no_motion():
    travel, yaw = drive_junction.dry_run_travel(None)
    assert travel == 0.0 and yaw == 0.0


# --- the pipeline -------------------------------------------------------

class Args(object):
    max_range = 5.0
    bearing_gate = 4.0
    max_detection_age = 0.3
    no_fill = False
    no_camera = False
    lookahead = 1.0
    max_duty = 0.1


def test_the_pipeline_arms_the_machine_on_a_junction():
    """End to end on a synthetic scan: a red triple in front of the car has to
    reach topo_state and come back as APPROACH."""
    from cone_nav.guidance.route_exec import RouteCursor
    from cone_perception.geometry import intrinsics_from_hfov
    from sim import cone_field
    from sim.drive_sim import CLASS_IDS, PREVIEW_H, PREVIEW_W

    layout = cone_field.track_junction("left")
    # 2.4 m short of the junction line: inside the measured 2.60-2.10 m window
    # in which a whole triple is recoverable. See data/layouts/junction_v2.md.
    pose = cone_field.Pose(0.6, 0.0, 0.0)
    local = cone_field.cones_in_car_frame(layout, pose)
    scan = cone_field.synth_scan(local)
    intr = intrinsics_from_hfov(PREVIEW_W, PREVIEW_H)
    detections = cone_field.synth_detections(local, intr, CLASS_IDS)

    class Set(object):
        def __init__(self, d):
            self.detections = d

        def age(self, _now):
            return 0.0

    topo = topo_state.TopoState(RouteCursor(["left"]))
    out = drive_junction.drive_pipeline(
        scan, Set(detections), cone_field.IDENTITY_CALIBRATION, intr, Args(),
        0.0, topo=topo)
    junction, survey = out[7], out[9]
    assert junction is not None, "the triple was not detected"
    assert len(survey.in_arm) == 3
    assert survey.reason == ""
    assert topo.state == topo_state.APPROACH
    assert junction.gaps_m[0] == pytest.approx(
        cone_field.JUNCTION_GATE_GAP_M, abs=0.15)


def test_the_gate_band_masks_the_fill_where_the_reds_stand():
    """The trap seen on the track: standing in FOLLOW near the gate, the
    centre red labelled in frame, the outer reds past the frame edge. The old
    radius shrink protected them by starving the corridor; the band excludes
    exactly the gate line -- where only reds may stand -- and fills everything
    else at full reach."""
    from cone_perception.cone_classes import CLASS_RED, UNLABELED
    from cone_perception.fusion import LabeledCone
    from cone_nav.corridor import side_assign
    from cone_nav.topology import gate_detect

    def cone(cls, x, y):
        return LabeledCone(cls, 0.9 if cls != UNLABELED else 0.0, x, y,
                           range_lidar=math.hypot(x, y), points=4)

    # Car 0.9 m from a tight gate: centre red labelled, outers out of frame
    # and unlabeled -- plus a legitimate out-of-frame boundary cone at 0.5 m
    # that the blind-spot fill exists for.
    cones = [cone(CLASS_RED, 0.9, 0.0),
             cone(UNLABELED, 0.9, 0.76), cone(UNLABELED, 0.9, -0.76),
             cone(UNLABELED, 0.5, 0.75)]
    survey = gate_detect.survey(cones)
    mask = side_assign.gate_line_of(None, 0.0, survey.reds, 0.0)
    assert mask is not None

    filled, count = side_assign.fill_unlabeled(cones, gate_line=mask)
    # The boundary cone is painted; both out-of-frame reds are left alone.
    assert count == 1
    by_pos = {(c.x, c.y): c.cone_class for c in filled}
    assert by_pos[(0.9, 0.76)] == UNLABELED
    assert by_pos[(0.9, -0.76)] == UNLABELED
    assert by_pos[(0.5, 0.75)] != UNLABELED



def test_a_dry_run_announces_that_travel_is_measured(capsys):
    """The push-speed assumption and its warning are gone; the run should say
    plainly that pace no longer matters, because two days of stage-3 attempts
    were shaped by operators trying to walk at an assumed number."""
    args = drive_junction.parse_args(
        ["--route", os.path.normpath(ROUTE), "--dry-run", "--no-deadman"])
    drive_junction.announce(args)
    out = capsys.readouterr().out
    assert "MEASURED" in out
    assert "push-speed" not in [a for a in vars(args)]


# --- the goal -----------------------------------------------------------

def test_the_goal_stop_range_has_a_default_and_is_tunable():
    assert drive_junction.parse_args(BASE).goal_stop == goal_stop.STOP_RANGE_M
    args = drive_junction.parse_args(BASE + ["--goal-stop", "0.45"])
    assert args.goal_stop == 0.45


def test_a_goal_stop_inside_the_chassis_floor_is_refused():
    """Below `clustering.MIN_CONE_RANGE_M` the trophy's return is discarded as
    the chassis arc leaking, so the car would be driving at a goal it can no
    longer see and stopping -- if at all -- on dead reckoning."""
    with pytest.raises(SystemExit):
        drive_junction.parse_args(BASE + ["--goal-stop", "0.10"])


def test_the_goal_is_disarmed_until_the_route_is_spent_unless_told_otherwise():
    assert drive_junction.parse_args(BASE).goal_anywhere is False
    assert drive_junction.parse_args(BASE + ["--goal-anywhere"]).goal_anywhere


def test_the_status_record_carries_the_goal():
    junction = set(drive_junction.JUNCTION_STATUS_SCHEMA["properties"])
    assert {"goal_state", "goal_range_m", "goal_reason", "goal_offset_m",
            "magenta_in_view", "goal_armed", "goal_blind_ticks"} <= junction


def test_status_of_survives_having_no_goal_latch():
    """Same contract as the state machine: the record is always the full shape,
    so a column never silently disappears from the trial log."""
    record = drive_junction.status_of({"duty": 0.0}, None, None, 0)
    assert record["goal_state"] == ""
    assert record["goal_range_m"] == 0.0
    assert set(record) - {"duty"} <= set(
        drive_junction.JUNCTION_STATUS_SCHEMA["properties"])


def test_status_of_reports_why_a_magenta_was_not_accepted():
    """`goal_state` alone cannot separate 'no trophy' from 'trophy off to one
    side' from 'trophy too far back', and those have different fixes."""
    from cone_nav.topology import goal_detect
    from cone_perception.cone_classes import CLASS_MAGENTA
    from cone_perception.fusion import LabeledCone

    off = LabeledCone(CLASS_MAGENTA, 0.9, 2.0, 0.9, range_lidar=2.19, points=4)
    record = drive_junction.status_of({"duty": 0.0}, None, None, 0,
                                      goal_survey=goal_detect.survey([off]))
    assert record["goal_reason"] == goal_detect.OFF_AXIS
    assert record["goal_offset_m"] == pytest.approx(0.9)
    assert record["magenta_in_view"] == 1


def test_the_pipeline_drives_the_car_at_a_confirmed_goal():
    """End to end on a synthetic scan: a magenta ahead of the car has to reach
    the latch, land on the driven line, and produce throttle -- the last of
    those being the point, since the corridor here is too short to move on."""
    from cone_perception.geometry import intrinsics_from_hfov
    from sim import cone_field
    from sim.drive_sim import CLASS_IDS, PREVIEW_H, PREVIEW_W

    layout = cone_field.straight_corridor(length=3.0, spacing=0.5)
    goal_xy = (3.4, 0.0)
    layout = layout + [cone_field.Cone("magenta", goal_xy[0], goal_xy[1], "goal")]
    intr = intrinsics_from_hfov(PREVIEW_W, PREVIEW_H)

    class Set(object):
        def __init__(self, d):
            self.detections = d

        def age(self, _now):
            return 0.0

    latch = goal_stop.GoalLatch()
    # Walk the car up the corridor so the latch sees the goal on consecutive
    # ticks, as it would on a real approach. One step per tick at the duty
    # floor: the latch refuses a goal that moves further than GOAL_GATE_M
    # between ticks, because on the track that was a different object.
    step, start, ticks = 0.05, 2.40, 6
    for i in range(ticks):
        pose = cone_field.Pose(start + i * step, 0.0, 0.0)
        local = cone_field.cones_in_car_frame(layout, pose)
        out = drive_junction.drive_pipeline(
            cone_field.synth_scan(local),
            Set(cone_field.synth_detections(local, intr, CLASS_IDS)),
            cone_field.IDENTITY_CALIBRATION, intr, Args(), 0.0,
            travel_m=step, goal_latch=latch, goal_armed=True)

    final_x = start + (ticks - 1) * step
    line, duty, goal_survey = out[3], out[5], out[11]
    assert goal_survey.reason == "", goal_survey.reason
    assert latch.confirmed
    assert latch.anchor_ok
    assert latch.hops == 0
    # The goal is the far end of the driven line -- that is what the anchor is.
    assert line.points[-1] == pytest.approx((goal_xy[0] - final_x, 0.0), abs=0.05)
    assert duty.duty > 0.0, duty.reason


def test_a_magenta_does_not_stop_the_car_while_a_turn_is_outstanding():
    """The arming guard, at the pipeline level: same scene, `goal_armed` false,
    and the latch must stay out of it however clear the detection is."""
    from cone_perception.geometry import intrinsics_from_hfov
    from sim import cone_field
    from sim.drive_sim import CLASS_IDS, PREVIEW_H, PREVIEW_W

    layout = (cone_field.straight_corridor(length=3.0, spacing=0.5)
              + [cone_field.Cone("magenta", 1.2, 0.0, "goal")])
    intr = intrinsics_from_hfov(PREVIEW_W, PREVIEW_H)

    class Set(object):
        def __init__(self, d):
            self.detections = d

        def age(self, _now):
            return 0.0

    latch = goal_stop.GoalLatch()
    local = cone_field.cones_in_car_frame(layout, cone_field.Pose(0.0, 0.0, 0.0))
    for _ in range(8):
        drive_junction.drive_pipeline(
            cone_field.synth_scan(local),
            Set(cone_field.synth_detections(local, intr, CLASS_IDS)),
            cone_field.IDENTITY_CALIBRATION, intr, Args(), 0.0,
            goal_latch=latch, goal_armed=False)
    assert not latch.stopped
    assert not latch.confirmed
