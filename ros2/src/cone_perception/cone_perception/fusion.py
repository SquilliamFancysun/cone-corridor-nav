"""Label lidar clusters with the classes the camera sees.

The division of labour is the point: the lidar knows WHERE a thing is and is
good at it; the camera knows WHAT colour it is and is useless at range. So a
cone's position always comes from its cluster and its class always comes from
its box, and the only thing that has to be solved is which box goes with which
cluster.

That is done on horizontal bearing alone, in the camera's frame, comparing a
detection's measured bearing against the bearing predicted FROM each cluster
(which is exact, because the cluster's range is known). No stereo XYZ is
involved: the OAK-D Lite's 7.5 cm baseline gives range error growing as z^2,
metres wide at 5 m, where the LD06 is good to a centimetre. range_stereo stays
NaN, which LabeledCone.msg explicitly permits.

Pure Python. No rclpy, no depthai.
"""

import math

from cone_perception import extrinsics, geometry
from cone_perception.cone_classes import UNLABELED, name_of

# Cone spacing on the track is 0.75 m at its tightest, which subtends 21 deg at
# 2 m and 8.6 deg at 5 m. A 4 deg gate is therefore far too narrow to confuse
# one cone with its neighbour, while still absorbing the lever-arm residual and
# the assumed-zero camera yaw.
#
# If CAMERA_YAW_DEG is wrong by more than this, NOTHING matches -- which is a
# much better failure than everything matching one cone to the left. The
# harness reports matched/unmatched counts so that shows up immediately.
MAX_BEARING_ERR_DEG = 4.0

# Ultralytics on the Pi 5 runs ~3-5 fps against a 10 Hz scan, so detections are
# always somewhat behind the scan they are being matched to. At a hand-pushed
# 0.5 m/s, 300 ms is 15 cm -- a fifth of the tightest cone spacing, tolerable.
# Beyond it, a label is more likely to be attached to the wrong cone than the
# right one, and UNLABELED is the honest answer.
MAX_DETECTION_AGE_S = 0.30


class LabeledCone(object):
    """One cone, shaped like cone_msgs/LabeledCone.

    Not the ROS message -- this module never imports rclpy. The node wrapper
    copies field for field.
    """

    __slots__ = ("cone_class", "confidence", "x", "y",
                 "range_stereo", "range_bbox", "range_lidar", "points")

    def __init__(self, cone_class, confidence, x, y,
                 range_stereo=float("nan"), range_bbox=float("nan"),
                 range_lidar=float("nan"), points=0):
        self.cone_class = cone_class
        self.confidence = confidence
        self.x = x
        self.y = y
        self.range_stereo = range_stereo
        self.range_bbox = range_bbox
        self.range_lidar = range_lidar
        self.points = points

    @property
    def xy(self):
        return (self.x, self.y)

    @property
    def labeled(self):
        return self.cone_class != UNLABELED

    def range_disagreement(self):
        """|range_lidar - range_bbox| in metres, or NaN if bbox range is absent.

        Large and persistent on one cone means the cluster matched to it is not
        that cone -- most often the wall behind it. Nothing else in the pipeline
        can notice that.
        """
        if math.isnan(self.range_bbox) or math.isnan(self.range_lidar):
            return float("nan")
        return abs(self.range_lidar - self.range_bbox)

    def __repr__(self):
        return (f"LabeledCone({name_of(self.cone_class)}, "
                f"({self.x:.2f}, {self.y:.2f}) m, conf={self.confidence:.2f})")


class FusionResult(object):
    """The cones, plus enough counters to tell why fusion is going badly."""

    __slots__ = ("cones", "candidates", "detections", "matched",
                 "out_of_fov", "unmatched_in_fov", "unmatched_detections",
                 "detection_age_s", "stale")

    def __init__(self, cones, candidates, detections, matched, out_of_fov,
                 unmatched_in_fov, unmatched_detections, detection_age_s, stale):
        self.cones = cones
        self.candidates = candidates
        self.detections = detections
        self.matched = matched
        self.out_of_fov = out_of_fov
        self.unmatched_in_fov = unmatched_in_fov
        self.unmatched_detections = unmatched_detections
        self.detection_age_s = detection_age_s
        self.stale = stale

    def as_dict(self):
        return {
            "candidates": self.candidates,
            "detections": self.detections,
            "matched": self.matched,
            "out_of_fov": self.out_of_fov,
            "unmatched_in_fov": self.unmatched_in_fov,
            "unmatched_detections": self.unmatched_detections,
            "detection_age_s": round(self.detection_age_s, 3),
            "stale": self.stale,
        }

    def __repr__(self):
        return (f"FusionResult({self.matched}/{self.candidates} matched, "
                f"{self.unmatched_detections} boxes unused"
                f"{', STALE' if self.stale else ''})")


def associate(candidates, detections, intr, detection_age_s=0.0,
              max_bearing_err_deg=MAX_BEARING_ERR_DEG,
              max_detection_age_s=MAX_DETECTION_AGE_S,
              fov_margin_deg=2.0):
    """ConeCandidates + Detections -> LabeledCones, one per candidate.

    Every candidate comes back, labelled or not. A cluster the camera could not
    explain is still real geometry, and dropping it would hide exactly the case
    worth seeing: a cone the detector missed.

    Matching is greedy over the globally sorted cost list rather than Hungarian,
    which would need scipy. The two differ only when a cheaper pairing forces an
    expensive one elsewhere, and at >=0.75 m cone spacing against a 4 deg gate
    that situation is not reachable -- the gate rejects the second pairing
    before the optimiser would have had to weigh it.
    """
    stale = detection_age_s > max_detection_age_s
    usable = [] if stale else list(detections)

    # Predicted camera-frame bearing per candidate, and whether the camera could
    # have seen it at all. The lidar sweeps ~218 deg, the camera 69 deg, so most
    # candidates fail this and that is normal, not a fault.
    predicted = [geometry.bearing_from_camera(c.x, c.y) for c in candidates]
    eligible = [i for i, b in enumerate(predicted)
                if geometry.in_camera_fov(b, fov_margin_deg)]

    measured = [geometry.detection_bearing(d, intr) for d in usable]
    gate = math.radians(max_bearing_err_deg)

    pairs = []
    for ci in eligible:
        for di, bearing in enumerate(measured):
            cost = abs(geometry.wrap_pi(predicted[ci] - bearing))
            if cost <= gate:
                pairs.append((cost, ci, di))
    pairs.sort()

    taken_c, taken_d = {}, set()
    for _cost, ci, di in pairs:
        if ci in taken_c or di in taken_d:
            continue
        taken_c[ci] = di
        taken_d.add(di)

    cones = []
    for i, cand in enumerate(candidates):
        di = taken_c.get(i)
        if di is None:
            cones.append(LabeledCone(
                cone_class=UNLABELED, confidence=0.0, x=cand.x, y=cand.y,
                range_lidar=cand.range_m, points=cand.points))
            continue
        det = usable[di]
        cones.append(LabeledCone(
            cone_class=det.cls,
            confidence=det.confidence,
            # Position is the lidar's, always. The camera contributes a label
            # and nothing else.
            x=cand.x, y=cand.y,
            range_stereo=float("nan"),
            range_bbox=geometry.range_from_bbox(det, intr),
            range_lidar=cand.range_m,
            points=cand.points,
        ))

    return FusionResult(
        cones=cones,
        candidates=len(candidates),
        detections=len(detections),
        matched=len(taken_c),
        out_of_fov=len(candidates) - len(eligible),
        unmatched_in_fov=len(eligible) - len(taken_c),
        unmatched_detections=len(usable) - len(taken_d),
        detection_age_s=detection_age_s,
        stale=stale,
    )


def startup_warnings():
    """Things worth saying once, at startup, rather than discovering on track."""
    return extrinsics.check_measured()
