"""Geometric side assignment, and above all what it refuses to touch.

The dangerous failure here is not a wrong colour -- it is this module quietly
painting over the UNLABELED bucket, which is the only signal the stack has that
the detector missed something. So most of these tests are about restraint.
"""

import math

import pytest

from cone_nav.corridor.side_assign import (
    GEOMETRIC_CONFIDENCE,
    MAX_FILL_RANGE_M,
    fill_unlabeled,
    heading_of,
    is_geometric,
)
from cone_perception.cone_classes import (
    CLASS_BLUE,
    CLASS_RED,
    CLASS_YELLOW,
    UNLABELED,
)
from cone_perception.fusion import LabeledCone


def cone(x, y, cls=UNLABELED, confidence=0.0, points=4):
    return LabeledCone(cone_class=cls, confidence=confidence, x=x, y=y,
                       range_lidar=math.hypot(x, y), points=points)


class FakeLine(object):
    def __init__(self, points):
        self.points = points


# --- what it fills ------------------------------------------------------

def test_a_near_cone_out_of_frame_gets_the_side_it_is_on():
    """0.75 m ahead, 0.75 m to the side: 45 deg, well outside the camera's
    32.5 deg acceptance, and the case the module exists for."""
    filled, count = fill_unlabeled([cone(0.75, 0.75), cone(0.75, -0.75)])
    assert count == 2
    assert filled[0].cone_class == CLASS_BLUE
    assert filled[1].cone_class == CLASS_YELLOW


def test_filled_cones_are_marked_as_geometric():
    filled, _ = fill_unlabeled([cone(0.75, 0.75)])
    assert filled[0].confidence == GEOMETRIC_CONFIDENCE
    assert is_geometric(filled[0])


def test_geometry_is_preserved_exactly():
    """Only the colour is inferred. The position came from the lidar and must
    survive untouched."""
    original = cone(0.8, -0.7, points=5)
    filled, _count = fill_unlabeled([original])
    assert (filled[0].x, filled[0].y) == (0.8, -0.7)
    assert filled[0].points == 5
    assert filled[0].range_lidar == original.range_lidar


def test_the_input_is_not_mutated():
    original = cone(0.75, 0.75)
    fill_unlabeled([original])
    assert original.cone_class == UNLABELED


# --- what it refuses ----------------------------------------------------

def test_a_cone_the_camera_could_see_is_left_alone():
    """In frame and unlabelled means the detector missed it. That is evidence,
    and painting over it would destroy the evidence."""
    filled, count = fill_unlabeled([cone(2.5, 0.4)])
    assert count == 0
    assert filled[0].cone_class == UNLABELED


def test_an_already_labelled_cone_is_never_overwritten():
    """The camera wins wherever it spoke -- including on a red gate cone that
    sits out of frame, which geometry would happily call a wall."""
    for cls in (CLASS_BLUE, CLASS_YELLOW, CLASS_RED):
        filled, count = fill_unlabeled([cone(0.75, 0.75, cls=cls,
                                             confidence=0.9)])
        assert count == 0
        assert filled[0].cone_class == cls
        assert filled[0].confidence == 0.9


def test_a_distant_out_of_frame_cone_is_left_alone():
    """Past the near window, out of frame stops meaning 'too close to see' and
    starts meaning 'in some other corridor'."""
    filled, count = fill_unlabeled([cone(0.5, MAX_FILL_RANGE_M + 1.0)])
    assert count == 0
    assert filled[0].cone_class == UNLABELED


def test_a_cone_near_the_axis_gets_no_side():
    """The sign of a near-zero offset is noise, and a boundary cone placed in
    the middle of the corridor invents a midpoint where there is no corridor."""
    filled, count = fill_unlabeled([cone(0.3, 0.05)])
    assert count == 0
    assert filled[0].cone_class == UNLABELED


def test_a_cone_behind_the_car_is_not_filled_into_the_corridor_ahead():
    """Directly behind is out of frame and within range, so only the offset rule
    stands between it and a colour. It must not get one."""
    filled, count = fill_unlabeled([cone(-1.0, 0.0)])
    assert count == 0


# --- the reference heading ----------------------------------------------

def test_the_reference_heading_rotates_the_split():
    """A cone to the left of straight-ahead is on the RIGHT of a corridor that
    runs further left still.

    The rotation has to be large because of what this module operates on: only
    cones outside a 32.5 deg frame are ever filled, so every candidate is a
    wide-angle one by construction, and swinging the axis past it takes a
    correspondingly wide swing. That is a property of the fill window, not a
    weakness of the offset rule.
    """
    ahead_left = cone(0.5, 0.5)
    straight, _ = fill_unlabeled([ahead_left])
    rotated, _ = fill_unlabeled([ahead_left], reference_heading_rad=math.radians(80))
    assert straight[0].cone_class == CLASS_BLUE
    assert rotated[0].cone_class == CLASS_YELLOW


def test_heading_of_a_straight_line_is_zero():
    assert heading_of(FakeLine([(1.0, 0.0), (3.0, 0.0)])) == pytest.approx(0.0)


def test_heading_of_a_left_curving_line_is_positive():
    assert heading_of(FakeLine([(1.0, 0.0), (2.0, 0.5), (3.0, 1.5)])) > 0.0


@pytest.mark.parametrize("points", [[], [(1.0, 1.0)], [(1.0, 1.0), (1.0, 1.0)]])
def test_heading_of_an_unusable_line_falls_back(points):
    assert heading_of(FakeLine(points), default=0.42) == 0.42


# --- the property that matters downstream -------------------------------

def test_a_corridor_of_unlabelled_near_cones_becomes_drivable():
    """The end-to-end claim in the module docstring, at the track's own 1.5 m
    straight spacing: without this the centerline has too few midpoints to
    steer along, and with it there are plenty."""
    from cone_nav.corridor.centerline import centerline

    cones = []
    for i in range(1, 4):
        cones.append(cone(i * 0.5, 0.75))
        cones.append(cone(i * 0.5, -0.75))

    before = centerline(cones, car_xy=(0.0, 0.0))
    filled, count = fill_unlabeled(cones)
    after = centerline(filled, car_xy=(0.0, 0.0))

    # Four, not six: the row at 1.5 m sits at 25.8 deg, inside the camera's
    # acceptance, so this module correctly declines to touch it. Only the two
    # nearer rows are in the blind spot.
    assert count == 4
    assert len(before.points) == 0
    assert len(after.points) >= 2


def test_cones_behind_the_car_are_never_filled():
    """The regression that stopped the car dead in sim/drive_sim.py.

    Filling behind manufactures midpoints behind, and the centerline's chain
    walk orders by distance from the car rather than by forward progress -- so a
    run of them reads as a longer chain than the real corridor and the car
    follows it backwards. It steers nowhere, reach is zero, and the centerline
    looks healthy the whole time.
    """
    behind = [cone(-1.5, 0.75), cone(-1.5, -0.75),
              cone(-0.8, 0.75), cone(-0.8, -0.75)]
    filled, count = fill_unlabeled(behind)
    assert count == 0
    assert all(c.cone_class == UNLABELED for c in filled)


def test_the_fill_never_produces_a_midpoint_behind_the_car():
    """Stated as the property rather than the boundary, so it holds however
    MIN_X_M is later retuned."""
    from cone_nav.corridor.centerline import centerline

    cones = []
    for x in (-2.0, -1.5, -1.0, -0.5, 0.5, 1.0):
        cones.append(cone(x, 0.75))
        cones.append(cone(x, -0.75))

    filled, _ = fill_unlabeled(cones)
    line = centerline(filled, car_xy=(0.0, 0.0))
    assert line.points, "expected a forward line"
    assert line.points[-1][0] > line.points[0][0], (
        f"chain runs backwards: {line.points}")
