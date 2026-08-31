"""What drive_junction.py refuses, and that it wires the manoeuvre up.

The loop itself needs a car. What can be tested at a desk is the argument
handling -- which is where the refusals live -- and that the pipeline actually
threads the state machine, which a synthetic scan can show.
"""

import math
import os

import pytest

import drive_junction
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
    """The two ways stage 3 finds nothing -- no red detected at all, versus
    two of three recovered -- are different problems with different fixes, and
    a log that only records whole triples cannot tell them apart."""
    record = drive_junction.status_of({"duty": 0.0}, None, None, 0, reds_seen=2)
    assert record["reds_seen"] == 2
    assert record["gate_live"] is False


def test_status_of_survives_having_no_state_machine():
    record = drive_junction.status_of({"duty": 0.0}, None, None, 0)
    assert record["topo_state"] == ""
    assert record["gate_range_m"] == 0.0


# --- the pipeline -------------------------------------------------------

class Args(object):
    max_range = 5.0
    bearing_gate = 4.0
    max_detection_age = 0.3
    no_fill = False
    no_camera = False
    lookahead = 1.0
    max_duty = 0.1
    fill_range_at_junction = 1.0


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
    junction, reds_seen = out[7], out[9]
    assert junction is not None, "the triple was not detected"
    assert reds_seen == 3
    assert topo.state == topo_state.APPROACH
    assert junction.gaps_m[0] == pytest.approx(
        cone_field.JUNCTION_GATE_GAP_M, abs=0.15)
