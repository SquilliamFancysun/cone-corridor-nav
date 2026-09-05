"""Red labels that survive a camera dropout -- and only red, only briefly.

The invariants under test are the honesty rules from the module docstring:
memory preserves, never creates; fresh labels always win; everything expires;
and a remembered red is visibly remembered, not disguised as a measurement.
"""

import pytest

from cone_perception import label_memory
from cone_perception.cone_classes import (
    CLASS_BLUE,
    CLASS_RED,
    CLASS_YELLOW,
    UNLABELED,
)
from cone_perception.fusion import LabeledCone


def cone(cls, x, y, confidence=0.9):
    return LabeledCone(cls, confidence if cls != UNLABELED else 0.0, x, y,
                       range_lidar=(x * x + y * y) ** 0.5, points=4)


def test_a_red_survives_the_camera_losing_it():
    memory = label_memory.RedMemory()
    memory.apply([cone(CLASS_RED, 2.0, 0.75)], now=0.0)

    out, remembered = memory.apply([cone(UNLABELED, 1.9, 0.74)], now=0.1)
    assert remembered == 1
    assert out[0].cone_class == CLASS_RED


def test_a_remembered_red_is_visibly_remembered():
    """Provenance must survive: a remembered red carries MEMORY_CONFIDENCE,
    distinct from any real detection and from the fill's geometric 0.0."""
    memory = label_memory.RedMemory()
    memory.apply([cone(CLASS_RED, 2.0, 0.0)], now=0.0)
    out, _n = memory.apply([cone(UNLABELED, 1.95, 0.0)], now=0.1)
    assert out[0].confidence == label_memory.MEMORY_CONFIDENCE
    assert 0.0 < label_memory.MEMORY_CONFIDENCE < 0.25


def test_the_memory_rides_the_moving_cluster():
    """The car advances; the cluster's measured position walks toward it. The
    memory must follow the CLUSTER, not stay staked to old coordinates."""
    memory = label_memory.RedMemory()
    memory.apply([cone(CLASS_RED, 2.4, 0.75)], now=0.0)
    for i in range(1, 15):
        x = 2.4 - 0.12 * i
        out, n = memory.apply([cone(UNLABELED, x, 0.75)], now=i * 0.1)
        assert n == 1, f"lost the red at x={x:.2f}"
        assert out[0].cone_class == CLASS_RED


def test_it_expires():
    memory = label_memory.RedMemory(ttl_s=1.0)
    memory.apply([cone(CLASS_RED, 2.0, 0.0)], now=0.0)
    out, n = memory.apply([cone(UNLABELED, 2.0, 0.0)], now=1.5)
    assert n == 0
    assert out[0].cone_class == UNLABELED


def test_a_fresh_camera_label_rearms_the_clock():
    memory = label_memory.RedMemory(ttl_s=1.0)
    memory.apply([cone(CLASS_RED, 2.0, 0.0)], now=0.0)
    memory.apply([cone(CLASS_RED, 1.9, 0.0)], now=0.8)     # re-confirmed
    out, n = memory.apply([cone(UNLABELED, 1.8, 0.0)], now=1.5)
    assert n == 1 and out[0].cone_class == CLASS_RED


def test_only_red_is_remembered():
    """Blue and yellow have the geometric fill; a wrongly-persisted boundary
    colour is a cost with no compensating need."""
    memory = label_memory.RedMemory()
    memory.apply([cone(CLASS_BLUE, 1.0, 0.75), cone(CLASS_YELLOW, 1.0, -0.75)],
                 now=0.0)
    assert len(memory) == 0


def test_memory_never_overrides_a_live_label():
    """The detector, right or wrong, outranks the memory THIS tick -- memory
    is a stand-in for an absent measurement, not a second opinion."""
    memory = label_memory.RedMemory()
    memory.apply([cone(CLASS_RED, 2.0, 0.0)], now=0.0)
    out, n = memory.apply([cone(CLASS_YELLOW, 1.95, 0.0)], now=0.1)
    assert n == 0
    assert out[0].cone_class == CLASS_YELLOW


def test_one_cluster_cannot_become_two_reds():
    """A cone both freshly seen and remembered is one red -- refresh happens
    after restore, against the same entry."""
    memory = label_memory.RedMemory()
    memory.apply([cone(CLASS_RED, 2.0, 0.0)], now=0.0)
    memory.apply([cone(CLASS_RED, 1.95, 0.0)], now=0.1)
    assert len(memory) == 1


def test_memory_does_not_leak_onto_a_different_cluster():
    """A cluster standing where no red ever stood must stay unlabeled --
    memory is keyed to place, gated at the tracker's own match distance."""
    memory = label_memory.RedMemory()
    memory.apply([cone(CLASS_RED, 2.0, 0.75)], now=0.0)
    out, n = memory.apply([cone(UNLABELED, 2.0, -0.75)], now=0.1)
    assert n == 0
    assert out[0].cone_class == UNLABELED


def test_a_whole_gate_survives_a_dropout_together():
    memory = label_memory.RedMemory()
    gate = [cone(CLASS_RED, 2.0, 0.76), cone(CLASS_RED, 2.0, 0.0),
            cone(CLASS_RED, 2.0, -0.76)]
    memory.apply(gate, now=0.0)
    blind = [cone(UNLABELED, 1.9, 0.76), cone(UNLABELED, 1.9, 0.0),
             cone(UNLABELED, 1.9, -0.76)]
    out, n = memory.apply(blind, now=0.4)
    assert n == 3
    assert all(c.cone_class == CLASS_RED for c in out)


def test_memory_cannot_ride_onto_a_cone_the_camera_calls_blue():
    """The hop that multiplied reds on the track: an expiring red re-binding
    to whichever cluster drifted nearest. A cluster the camera has labelled
    another colour is not this red at any distance."""
    memory = label_memory.RedMemory()
    memory.apply([cone(CLASS_RED, 1.0, 0.10)], now=0.0)
    out, n = memory.apply([cone(CLASS_BLUE, 1.0, 0.15),
                           cone(UNLABELED, 1.0, 0.05)], now=0.1)
    assert n == 1
    assert out[0].cone_class == CLASS_BLUE       # untouched
    assert out[1].cone_class == CLASS_RED        # the rightful heir


def test_the_memory_gate_is_tighter_than_the_tracker():
    """A tracker swap washes out of a rigid fit; a memory hop IS the failure.
    The memory's per-tick reach must undercut ego_motion's."""
    from cone_perception import ego_motion
    assert label_memory.MEMORY_GATE_M < ego_motion.MATCH_GATE_M
    assert label_memory.MEMORY_GATE_M > 1.2 * 0.1   # fastest legal tick step


def test_forgetting_drops_everything_the_lift_invalidated():
    """Carrying the car back to a junction moves it by metres, and every entry
    is a base_link position re-bound on a 0.20 m gate sized for ONE tick of
    travel. Left in place, a remembered red lands on whichever cluster now sits
    where a different cone used to be -- a phantom red at the exact moment the
    car is trying to recognise the gate again."""
    memory = label_memory.RedMemory()
    cones = [LabeledCone(CLASS_RED, 0.9, 2.0, 0.0)]
    memory.apply(cones, 0.0)
    assert len(memory) == 1

    memory.forget()
    assert len(memory) == 0

    # a cluster where the remembered red used to be is no longer repainted
    out, remembered = memory.apply([LabeledCone(UNLABELED, 0.0, 2.0, 0.0)], 0.1)
    assert remembered == 0
    assert out[0].cone_class == UNLABELED
