"""How far the car moved between two scans, measured from the scene itself.
Pure function, no rclpy.

## Why this exists

`topo_state` needs travelled distance to decide when a junction is behind the
car, and nothing on this car measures it: the VESC encoder is unread, and a
dry run never opens the VESC at all. The first stand-in was a duty-cycle
estimate (zero in a dry run), the second an assumed push speed -- which made
the state machine exactly as honest as the operator's pace was close to the
assumption. Measured on the track 2026-09-01: a walk at 0.13 m/s against an
assumed 0.5 declared the junction passed while the car was still 1.94 m short
of the reds.

The scene already carries the answer. Every scan yields cone clusters placed
to a centimetre in base_link, and the cones do not move -- so when the car
does, the whole cone field shifts in the car's frame by exactly the car's
motion. Matching this scan's cones to the last scan's and fitting the rigid
transform between them measures both the travel and the yaw, at scan rate,
with no assumption about how the car is being propelled.

## What it is and is not

This is scan-to-scan visual odometry over a handful of landmarks, and it
inherits dead reckoning's nature: each step is a measurement, the SUM is a
random walk. At the observed cluster noise (millimetres on a median-range
centroid) the drift over a whole traverse is centimetres -- fine for a
distance FLOOR with 0.5 m of slack, which is its only consumer. It is not a
pose estimate and must never be treated as one.

A step needs two scans with at least one cone in common. Two or more matched
pairs recover rotation as well; a single pair recovers translation with yaw
assumed zero, which at one tick's timescale is a smaller lie than discarding
the measurement. No pairs -- an empty scene, or everything swapped between
ticks -- returns None, and the caller treats that tick as no motion, which is
the same convention `speed_ctrl` uses for an empty line: the safe reading.
"""

import math

# A cone cannot move further than this between consecutive scans, or it is not
# the same cone. 0.35 m per tick is 3.5 m/s at the LD06's 10 Hz -- nearly three
# times the fastest this car is allowed to go -- so the gate rejects identity
# swaps without ever rejecting real motion.
MATCH_GATE_M = 0.35

# Steps smaller than this are indistinguishable from cluster jitter, and
# topo_state clamps negative travel to zero -- so feeding it raw noise
# accumulates a positive random walk while the car stands still. The observed
# per-step noise on a median over several matched cones is a few millimetres;
# 8 mm swallows it while costing nothing real: even a slow hand-push covers
# more than that per tick.
DEADBAND_M = 0.008


class Step(object):
    """The car's motion between two scans, in the EARLIER scan's frame."""

    __slots__ = ("forward_m", "lateral_m", "yaw_rad", "pairs")

    def __init__(self, forward_m, lateral_m, yaw_rad, pairs):
        self.forward_m = forward_m
        self.lateral_m = lateral_m
        self.yaw_rad = yaw_rad
        self.pairs = pairs

    def __repr__(self):
        return (f"Step({self.forward_m:+.3f} m fwd, "
                f"{math.degrees(self.yaw_rad):+.2f} deg, {self.pairs} pairs)")


def _match(previous, current, gate_m):
    """Greedy nearest-neighbour pairing, each current cone used once.

    Greedy is enough here for the same reason it is in `fusion.associate`:
    the gate is far tighter than the spacing of the things being matched, so
    the case where greedy and optimal differ is not reachable on a legal
    track.
    """
    pairs = []
    used = set()
    for p in previous:
        best, best_d = None, gate_m
        for j, c in enumerate(current):
            if j in used:
                continue
            d = math.hypot(p.x - c.x, p.y - c.y)
            if d < best_d:
                best, best_d = j, d
        if best is not None:
            used.add(best)
            pairs.append((p, current[best]))
    return pairs


def rigid_step(previous, current, gate_m=MATCH_GATE_M):
    """Two cone sets -> the car's Step between them, or None.

    Solves the 2D rigid fit p1 ~ R p0 + t over the matched pairs (the closed
    form: centre both sets, then the rotation angle is atan2 of the planar
    cross- and dot-products), and converts the SCENE's apparent motion into
    the CAR's actual motion: a car that advances sees the world slide
    backward, so the car's displacement is the scene's, negated and rotated
    into the car's old frame.
    """
    pairs = _match(previous, current, gate_m)
    if not pairs:
        return None

    if len(pairs) == 1:
        p, c = pairs[0]
        # One landmark cannot see rotation. Assume none for this one tick.
        return Step(p.x - c.x, p.y - c.y, 0.0, 1)

    n = float(len(pairs))
    c0x = sum(p.x for p, _ in pairs) / n
    c0y = sum(p.y for p, _ in pairs) / n
    c1x = sum(c.x for _, c in pairs) / n
    c1y = sum(c.y for _, c in pairs) / n

    sdot = sum((p.x - c0x) * (c.x - c1x) + (p.y - c0y) * (c.y - c1y)
               for p, c in pairs)
    scross = sum((p.x - c0x) * (c.y - c1y) - (p.y - c0y) * (c.x - c1x)
                 for p, c in pairs)
    if abs(sdot) < 1e-12 and abs(scross) < 1e-12:
        return None

    # p1 = R p0 + t. The car turned by -angle(R) and moved so that the scene
    # slid by t: car displacement d solves t = -R d, i.e. d = -R^T t.
    ang = math.atan2(scross, sdot)
    cos_a, sin_a = math.cos(ang), math.sin(ang)
    tx = c1x - (cos_a * c0x - sin_a * c0y)
    ty = c1y - (sin_a * c0x + cos_a * c0y)
    dx = -(cos_a * tx + sin_a * ty)
    dy = -(-sin_a * tx + cos_a * ty)
    return Step(dx, dy, -ang, len(pairs))
