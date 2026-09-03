"""The flags that let the car back itself out, and the refusals around them.

`--reverse` is opt-in and stays that way. Off, every path is the one the
2026-09-02 demo drove, which is the point: the car gets the new code by rsync
and there is no clone on it to switch back with, so the flag is what makes a
redeploy safe to demo from.
"""

import pytest

from cone_nav.control import speed_ctrl

import drive_junction

EXPLORE = ["--explore"]


def args(extra):
    return drive_junction.parse_args(EXPLORE + extra)


# --- defaults -----------------------------------------------------------

def test_reverse_is_off_by_default():
    """The demo behaviour is the default behaviour. A deployed build with this
    flag unset drives exactly what stage 7b drove."""
    a = args([])
    assert a.reverse is False
    assert a.reverse_only is False


def test_the_reverse_duty_defaults_to_the_cogging_floor():
    assert args([]).max_reverse_duty == speed_ctrl.MAX_REVERSE_DUTY
    assert speed_ctrl.MAX_REVERSE_DUTY == speed_ctrl.MIN_MOVE_DUTY


# --- refusals -----------------------------------------------------------

def test_reverse_needs_something_to_back_out_to():
    """A route already says which way to turn at every junction. Backing out
    is how the SEARCH takes a branch it has not tried, and only --explore
    searches."""
    with pytest.raises(SystemExit):
        drive_junction.parse_args(["--route", "/dev/null", "--reverse"])


def test_the_two_reverse_modes_are_exclusive():
    with pytest.raises(SystemExit):
        args(["--reverse", "--reverse-only"])


def test_reverse_only_is_a_mode_of_its_own():
    with pytest.raises(SystemExit):
        args(["--reverse-only", "--dry-run"])
    with pytest.raises(SystemExit):
        args(["--reverse-only", "--steer-only"])
    assert args(["--reverse-only"]).mode == "reverse-only"


def test_the_reverse_duty_is_a_magnitude():
    """The sign belongs to `speed_ctrl.reverse_duty`, which is where the one
    place that decides direction should be. A negative here would be a second
    opinion about it."""
    with pytest.raises(SystemExit):
        args(["--reverse", "--max-reverse-duty", "-0.05"])
    with pytest.raises(SystemExit):
        args(["--reverse", "--max-reverse-duty", "0"])


def test_a_fast_reverse_is_allowed_but_says_so(capsys):
    """Not refused -- 8b may find the car needs more than the floor to move at
    all. But reverse_ctrl's loop stiffens with speed on gains nothing has
    measured, so it is not a quiet change."""
    a = args(["--reverse", "--max-reverse-duty", "0.15"])
    assert a.max_reverse_duty == 0.15
    assert "over the cogging floor" in capsys.readouterr().out


# --- what it tells the operator ----------------------------------------

def test_the_reverse_run_announces_that_nothing_is_behind_the_car(capsys):
    drive_junction.announce(args(["--reverse"]))
    out = capsys.readouterr().out
    assert "BACKS ITSELF OUT" in out
    assert "cannot see behind it" in out


def test_without_reverse_it_still_tells_you_to_carry_the_car(capsys):
    drive_junction.announce(args([]))
    assert "carry it back" in capsys.readouterr().out


def test_reverse_only_announces_loudly_and_says_nothing_else(capsys):
    drive_junction.announce(args(["--reverse-only"]))
    out = capsys.readouterr().out
    assert "REVERSE ONLY" in out
    assert "BACKWARDS" in out
    # It never reaches a junction, so the route and goal lines would be noise.
    assert "goal " not in out


# --- the status record --------------------------------------------------

def test_the_status_record_carries_the_manoeuvre():
    """Same contract as every other field: always the full shape, so a column
    never silently disappears from the trial log."""
    junction = set(drive_junction.JUNCTION_STATUS_SCHEMA["properties"])
    assert {"backout_state", "backout_reason", "backout_travelled_m",
            "backout_bound_m", "backout_gate_m", "backout_heading_err_deg",
            "backout_cross_track_m", "backout_blind_ticks"} <= junction


def test_status_of_survives_having_no_manoeuvre():
    record = drive_junction.status_of({"duty": 0.0}, None, None, 0)
    assert record["backout_state"] == ""
    assert record["backout_travelled_m"] == 0.0


# --- the pipeline hold --------------------------------------------------

def test_backing_out_holds_the_state_machine_off_a_live_junction():
    """The failure the hold exists for, on a real synthetic scan rather than a
    stub. Reversing takes the car back THROUGH its junction, so a whole triple
    appears ahead of a car pointing the wrong way down it and moving away. Left
    running, `_follow` arms an approach on that sighting and `_approach`
    commits a traverse a tick or two later -- consuming the very branch the
    search is backing out to try.

    Same scan, same pose, twice: it must arm normally and not while held.
    """
    from cone_nav.guidance.route_exec import RouteCursor
    from cone_nav.topology import topo_state
    from cone_perception.geometry import intrinsics_from_hfov
    from sim import cone_field
    from sim.drive_sim import CLASS_IDS, PREVIEW_H, PREVIEW_W

    from test_drive_junction import Args

    layout = cone_field.track_junction("left")
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

    def run(backing_out):
        topo = topo_state.TopoState(RouteCursor(["left"]))
        out = drive_junction.drive_pipeline(
            scan, Set(detections), cone_field.IDENTITY_CALIBRATION, intr,
            Args(), 0.0, topo=topo, backing_out=backing_out)
        return topo, out[7]

    driving, junction = run(False)
    assert junction is not None, "the triple was not detected at all"
    assert driving.state == topo_state.APPROACH

    held, junction_held = run(True)
    # The survey still runs -- the manoeuvre's own ending is read off it.
    assert junction_held is not None
    assert held.state == topo_state.FOLLOW
    assert held.live is None
