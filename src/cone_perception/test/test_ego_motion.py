"""Ego-motion from cone sets: the measurement that replaced the assumed
push speed.

The convention under test everywhere: `Step` is the CAR's motion in its own
earlier frame. A car that drives forward sees every cone's x shrink; the step
it reports must be POSITIVE forward.
"""

import math
import random

import pytest

from cone_perception import ego_motion


class P(object):
    __slots__ = ("x", "y")

    def __init__(self, x, y):
        self.x = x
        self.y = y


def moved(cones, forward=0.0, lateral=0.0, yaw_rad=0.0, noise=0.0, seed=0):
    """Where the same physical cones appear one tick later, given the car's
    motion. The scene rotates and slides opposite to the car."""
    rng = random.Random(seed)
    out = []
    cos_t, sin_t = math.cos(-yaw_rad), math.sin(-yaw_rad)
    for c in cones:
        x, y = c.x - forward, c.y - lateral
        out.append(P(x * cos_t - y * sin_t + rng.gauss(0, noise),
                     x * sin_t + y * cos_t + rng.gauss(0, noise)))
    return out


CORRIDOR = [P(0.8, 0.75), P(0.8, -0.75), P(1.55, 0.75), P(1.55, -0.75),
            P(2.3, 0.75), P(2.3, -0.75)]


def test_pure_forward_motion_is_positive_forward():
    step = ego_motion.rigid_step(CORRIDOR, moved(CORRIDOR, forward=0.05))
    assert step.pairs == len(CORRIDOR)
    assert step.forward_m == pytest.approx(0.05, abs=1e-9)
    assert step.lateral_m == pytest.approx(0.0, abs=1e-9)
    assert step.yaw_rad == pytest.approx(0.0, abs=1e-9)


def test_rolling_backward_is_negative_not_clamped():
    """The clamp belongs to topo_state, not the measurement. A measurement
    that cannot go negative accumulates drift out of noise."""
    step = ego_motion.rigid_step(CORRIDOR, moved(CORRIDOR, forward=-0.04))
    assert step.forward_m == pytest.approx(-0.04, abs=1e-9)


def test_a_turn_is_recovered_alongside_the_travel():
    step = ego_motion.rigid_step(
        CORRIDOR, moved(CORRIDOR, forward=0.06, yaw_rad=math.radians(2.0)))
    assert step.forward_m == pytest.approx(0.06, abs=1e-3)
    assert math.degrees(step.yaw_rad) == pytest.approx(2.0, abs=1e-6)


def test_a_single_shared_cone_still_measures_translation():
    step = ego_motion.rigid_step([P(1.5, 0.2)],
                                 moved([P(1.5, 0.2)], forward=0.05))
    assert step.pairs == 1
    assert step.forward_m == pytest.approx(0.05, abs=1e-9)
    assert step.yaw_rad == 0.0


def test_no_common_cones_is_none_not_zero():
    """None means 'could not measure'; 0.0 would mean 'measured no motion'.
    The caller must be able to tell those apart."""
    assert ego_motion.rigid_step([P(1.0, 0.5)], [P(3.0, -1.9)]) is None
    assert ego_motion.rigid_step([], CORRIDOR) is None


def test_a_cone_leaving_the_scene_does_not_poison_the_step():
    """Cones enter and leave view every few ticks; the fit must ride on the
    ones that stayed."""
    later = moved(CORRIDOR, forward=0.05)[1:]      # nearest cone gone
    later.append(P(3.05, 0.75))                    # a new one appeared
    step = ego_motion.rigid_step(CORRIDOR, later)
    assert step.forward_m == pytest.approx(0.05, abs=0.02)


def test_centimetre_noise_yields_millimetre_steps_of_error():
    total_err = 0.0
    for seed in range(20):
        step = ego_motion.rigid_step(
            CORRIDOR, moved(CORRIDOR, forward=0.013, noise=0.008, seed=seed))
        total_err += abs(step.forward_m - 0.013)
    assert total_err / 20 < 0.006


def test_the_deadband_is_below_a_slow_push_and_above_the_jitter():
    """0.008 m per tick is 0.08 m/s -- slower than any deliberate push --
    while the fit noise on a healthy corridor is a few millimetres."""
    assert 0.003 < ego_motion.DEADBAND_M < 0.013


def test_the_match_gate_outruns_the_car():
    """The gate must never reject true motion: at 10 Hz it has to exceed the
    fastest per-tick displacement this car is allowed."""
    fastest_per_tick = 1.2 * 0.1        # 1.2 m/s at 10 Hz
    assert ego_motion.MATCH_GATE_M > 2 * fastest_per_tick
