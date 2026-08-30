"""The closed loop, end to end, with no hardware.

These are the tests that would otherwise be track runs. Each one is a failure
that has actually cost a session on some project: a mirrored steering sign, a
car that stops mid-corridor, a controller that only works in one direction.

They are slow by this repo's standards -- a few seconds -- because each drives a
whole corridor through the real perception stack. That is the point: everything
below `synth_scan` is the code that runs on the car.
"""

import math

import pytest

from cone_perception import extrinsics
from sim import cone_field
from sim.drive_sim import (
    DEFAULT_LOOKAHEAD_M,
    Vehicle,
    cross_track_error,
    layout_centerline,
    simulate,
)

# The measured car, not a copy. A test that pins the placeholder geometry keeps
# passing after the real numbers land and stops describing the vehicle we have.
AXLE = extrinsics.REAR_AXLE_IN_BASE
WHEELBASE = extrinsics.WHEELBASE_M


def drive(layout, **kw):
    kw.setdefault("lookahead_m", DEFAULT_LOOKAHEAD_M)
    return simulate(layout, WHEELBASE, AXLE, **kw)


def straight():
    return cone_field.straight_corridor(length=8.0, spacing=1.0)


def curve(left=True):
    return cone_field.curved_corridor(radius=4.0, sweep_deg=70.0, spacing=1.0,
                                      left=left)


def s_bend():
    return cone_field.s_bend_corridor(radius=4.0, sweep_deg=45.0, spacing=1.0)


# --- the frame ----------------------------------------------------------

def test_base_link_leads_the_axle():
    """The sensors are ahead of the pivot. If this inverts, every lookahead is
    measured from the wrong end of the car."""
    v = Vehicle(WHEELBASE, AXLE)
    assert v.base_pose().x == pytest.approx(0.25)


def test_base_link_rotates_with_the_car():
    v = Vehicle(WHEELBASE, AXLE, heading_rad=math.pi / 2)
    pose = v.base_pose()
    assert pose.x == pytest.approx(0.0, abs=1e-9)
    assert pose.y == pytest.approx(0.25)


# --- the runs -----------------------------------------------------------

def test_it_drives_a_straight_corridor():
    result = drive(straight())
    assert result.completed, result.outcome
    assert result.struck_cone is None


def test_it_drives_a_left_bend():
    result = drive(curve(left=True))
    assert result.completed, result.outcome
    assert result.struck_cone is None


def test_it_drives_a_right_bend():
    """The test a mirrored steering sign cannot pass. A straight corridor is
    driven identically by a controller that steers backwards; a bend is not."""
    result = drive(curve(left=False))
    assert result.completed, result.outcome
    assert result.struck_cone is None


def test_it_drives_an_s_bend():
    result = drive(s_bend())
    assert result.completed, result.outcome
    assert result.struck_cone is None


def test_the_two_bend_directions_are_symmetric():
    """Asymmetry here means a sign has been applied in one place and not its
    mirror -- the kind of bug that drives one way beautifully."""
    left = drive(curve(left=True))
    right = drive(curve(left=False))
    assert left.mean_xtrack_m == pytest.approx(right.mean_xtrack_m, abs=0.02)


def test_it_stays_near_the_middle_of_the_corridor():
    """Half the corridor width is 0.75 m, so 15 cm of mean error is the line
    between 'following the corridor' and 'bouncing off its walls'."""
    for layout in (straight(), curve(), s_bend()):
        result = drive(layout)
        assert result.mean_xtrack_m < 0.15, (
            f"{result.outcome}: {result.mean_xtrack_m * 100:.1f} cm mean")


# --- the failure modes --------------------------------------------------

def test_no_cones_means_no_motion():
    """The most important zero. An empty field must not produce a car that
    coasts forward on the last command it liked."""
    result = drive([], max_time_s=5.0)
    assert result.distance_m == pytest.approx(0.0)
    assert not result.completed


def test_it_stops_when_the_corridor_runs_out():
    """A stub, not a corridor. The car should refuse rather than drive into the
    end of it."""
    result = drive(cone_field.straight_corridor(length=1.0, spacing=0.5),
                   max_time_s=10.0)
    assert not result.completed
    assert result.struck_cone is None


def test_it_still_drives_when_the_camera_sees_nothing():
    """Every colour dropped: the detector is dead, and only geometric side
    assignment is holding the corridor up. This is the contingency path from the
    plan, exercised rather than assumed."""
    result = drive(straight(), dropout=("blue", "yellow", "red", "orange",
                                        "magenta"), fill_in_fov=True)
    assert result.completed, result.outcome
    assert result.struck_cone is None


def test_cone_spacing_decides_whether_the_car_can_drive_at_all():
    """The track-build finding, pinned so it cannot quietly stop being true.

    The two sensors only overlap between about 1.18 m and 3.0 m ahead, and how
    many cone rows land in that window is set by SPACING. At the 1.5 m straights
    of data/layouts/track_v1.md there are too few midpoints to form a chain and
    the car does not move -- with or without the near-field fill, because the
    fill cannot see further than the lidar either.

    If the sparse cases ever start driving, the overlap has changed (a wider
    lens, a better lidar, a narrower corridor) and the spacing advice in
    side_assign's docstring needs revisiting.
    """
    def drives(spacing, **kw):
        layout = cone_field.straight_corridor(length=8.0, spacing=spacing)
        return drive(layout, max_time_s=30.0, **kw).completed

    assert not drives(1.5)
    assert drives(1.0)
    assert drives(0.75)
    # 1.25 m is deliberately not asserted either way. It sits one cone row from
    # the cutoff and has already flipped once, when the cone's lidar
    # cross-section went from estimated to measured. Pinning it would make this
    # test a tripwire on a number nobody should be building a track at.


def test_the_near_field_fill_is_what_makes_a_drivable_spacing_drivable():
    """At 1.0 m the camera alone still cannot do it; the fill is load-bearing."""
    layout = cone_field.straight_corridor(length=8.0, spacing=1.0)
    assert not drive(layout, fill_sides=False, max_time_s=30.0).completed
    assert drive(layout, fill_sides=True, max_time_s=30.0).completed


# --- the scoring itself -------------------------------------------------

def test_the_ideal_centerline_runs_up_the_middle():
    line = layout_centerline(straight())
    assert len(line) >= 4
    assert all(abs(y) < 1e-6 for _, y in line)


def test_cross_track_error_is_zero_on_the_line():
    line = layout_centerline(straight())
    assert cross_track_error((3.0, 0.0), line) == pytest.approx(0.0, abs=1e-9)


def test_cross_track_error_measures_the_offset():
    line = layout_centerline(straight())
    assert cross_track_error((3.0, 0.3), line) == pytest.approx(0.3, abs=1e-6)


# --- provenance ---------------------------------------------------------

def test_geometric_labels_are_distinguishable_from_detected_ones():
    """The report must never present an inferred colour as a measured one."""
    result = drive(straight())
    assert any(t.filled > 0 for t in result.ticks)
    assert any(t.labeled > 0 for t in result.ticks)
