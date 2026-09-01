"""Carry a red label across camera dropouts, on a tracked cluster.
Pure, no rclpy.

## Why red labels die today, and why they no longer have to

`fusion.associate` re-matches camera boxes to lidar clusters BY BEARING every
tick, and refuses boxes older than 300 ms -- rightly, because a stale box's
bearing lies once the car moves, and re-matching it would hang the label on
whatever cluster drifted under it. So the moment a red leaves the frame, or
the detector misses a frame, the cluster reverts to UNLABELED and the gate
flickers: the last on-track run recovered the triple on 23 ticks out of 80
in the joint window, and `topo_state` carries the difference on dead
reckoning.

Scan-matched tracking (`ego_motion`) removes the reason for the 300 ms
horizon. A label attached to a TRACKED cluster does not suffer bearing drift:
the cluster carries its label with itself, the position stays lidar-fresh
every tick, and only the colour is remembered. This module widens the class's
time horizon from 300 ms to a few seconds -- for exactly one class.

## Why only red

Blue and yellow already have a geometric substitute (`fill_unlabeled`), and a
wrongly-persisted boundary colour costs a midpoint. Red has no substitute --
geometry cannot tell a gate from a wall -- and a lost red costs the junction.
The asymmetry in stakes is the asymmetry in treatment.

## What keeps it honest

  - Memory can only PRESERVE a label the detector produced, never create one,
    and a fresh camera label always overwrites it.
  - It expires: TTL_S after the camera last confirmed the cone, the memory
    dies, so a misclassification lives seconds, not forever.
  - It is keyed to the tracked cluster. Memory rides the cluster's measured
    position; a cluster that leaves the scene takes its memory with it when
    the TTL lapses, and memory is only ever APPLIED to a cluster standing
    where the remembered red stood.
  - Provenance is visible: a remembered red carries confidence
    `MEMORY_CONFIDENCE`, distinct from any real detection, and the caller
    logs the count separately.
"""

import math

from cone_perception import ego_motion
from cone_perception.cone_classes import CLASS_RED, UNLABELED
from cone_perception.fusion import LabeledCone

# How long a red survives without the camera re-confirming it. Sized for the
# blind mouth at DRIVING speed -- ~1.5 s at 1.2 m/s -- with margin, and short
# enough that a misread orange stops being red before the car has moved a
# car-length past it. A hand-pushed mouth outlasts this; there the latch in
# topo_state still does the carrying, as it always did.
TTL_S = 3.0

# Stamped on a remembered red. Distinct from every real detection (those are
# >= the detector's conf threshold) and from side_assign's geometric 0.0, so
# the three provenances -- measured, remembered, inferred -- stay tellable
# apart in the log.
MEMORY_CONFIDENCE = 0.01


class _Entry(object):
    __slots__ = ("x", "y", "expires_at")

    def __init__(self, x, y, expires_at):
        self.x = x
        self.y = y
        self.expires_at = expires_at


class RedMemory(object):
    """The reds the camera has vouched for recently, riding their clusters."""

    def __init__(self, ttl_s=TTL_S, gate_m=ego_motion.MATCH_GATE_M):
        self.ttl_s = ttl_s
        self.gate_m = gate_m
        self._entries = []

    def apply(self, cones, now):
        """LabeledCones -> (cones, remembered_count), reds restored.

        Order matters and is deliberate: restore first from what was
        remembered, THEN refresh memory from this tick's camera labels -- so a
        cone both freshly seen and remembered is one red, not two, and a fresh
        label re-arms its own TTL.
        """
        self._entries = [e for e in self._entries if e.expires_at > now]

        out = list(cones)
        remembered = 0
        claimed = set()
        for entry in self._entries:
            best, best_d = None, self.gate_m
            for i, cone in enumerate(out):
                if i in claimed:
                    continue
                d = math.hypot(cone.x - entry.x, cone.y - entry.y)
                if d < best_d:
                    best, best_d = i, d
            if best is None:
                # Cluster flickered out this scan. Keep the memory where it
                # was; a 2-point cluster routinely misses a revolution and
                # returns within the match gate.
                continue
            claimed.add(best)
            cone = out[best]
            # The memory follows the cluster, not the other way round.
            entry.x, entry.y = cone.x, cone.y
            if cone.cone_class == UNLABELED:
                out[best] = LabeledCone(
                    cone_class=CLASS_RED, confidence=MEMORY_CONFIDENCE,
                    x=cone.x, y=cone.y,
                    range_stereo=cone.range_stereo,
                    range_bbox=cone.range_bbox,
                    range_lidar=cone.range_lidar,
                    points=cone.points)
                remembered += 1

        for cone in out:
            if cone.cone_class == CLASS_RED and \
                    cone.confidence > MEMORY_CONFIDENCE:
                self._remember(cone, now)
        return out, remembered

    def _remember(self, cone, now):
        for entry in self._entries:
            if math.hypot(cone.x - entry.x, cone.y - entry.y) < self.gate_m:
                entry.x, entry.y = cone.x, cone.y
                entry.expires_at = now + self.ttl_s
                return
        self._entries.append(_Entry(cone.x, cone.y, now + self.ttl_s))

    def __len__(self):
        return len(self._entries)
