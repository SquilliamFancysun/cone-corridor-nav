"""Solve the LD06's bearing convention from cones at known bearings.

The mount sign and the yaw offset are the one piece of geometry that can be
wrong while every number on screen looks reasonable: ranges are unaffected, the
corridor is the right width, and the centerline steers into the wrong boundary
only at a junction. README.md's cone check catches it by eye. This module
catches it by arithmetic, and writes down what it found.

**One cone cannot do it.** With a single observation the sign and the offset
trade off exactly — a cone seen at sensor bearing `a` fits `car = -a + offset`
and `car = +a + offset'` equally well, for offsets that differ. Two cones at
different bearings over-determine the fit by one degree of freedom, so the
residual becomes a real measurement of whether the model holds at all. That is
why `solve_convention` refuses fewer than two poses, and refuses two that sit
too close together.

Pure Python: no pyserial, no foxglove, no car. lidar_view.py collects the scans
and does the talking; everything that can be quietly wrong is in here, where
pytest can reach it.

Angle convention throughout, matching ld06.Scan.to_xy:

    car_bearing = sign * sensor_bearing + angle_offset,  sign = +1 if mirror
                                                                else -1

with car bearings measured counterclockwise from straight ahead — left is
positive, the same sense as y-left in REP-103 and in data/layouts/track_v1.md.
"""

import json
import os
from datetime import datetime, timezone

# The clusterer and the angle helpers moved to cone_perception, because the nav
# stack needs the same "what counts as one object" and a second copy would
# drift. Re-exported here so this module's callers -- and its tests -- keep the
# names they had.
from cone_perception.clustering import (  # noqa: F401  (re-exported)
    GAP_DEG,
    GAP_MM,
    MIN_CLUSTER_POINTS,
    Cluster,
    _make_cluster,
    _median,
    circular_mean,
    circular_spread,
    cluster_scan,
    wrap180,
    wrap360,
)

FORMAT_VERSION = 1

# Beside the tool, not in the data directory: it describes this car's mount, so
# it belongs with the thing that reads it. deploy.sh excludes it from --delete,
# which it must — a redeploy that silently wiped the calibration would leave the
# next session recording an unverified sign.
DEFAULT_FILENAME = "calibration.json"

# Anything this close is the car, not the world. See the chassis note in
# docs/hardware-baseline.md: ~250 mm returns around 184 deg are the vehicle.
CHASSIS_MAX_MM = 400.0

# Two poses closer together than this leave the sign barely determined: the
# residual difference between the two hypotheses shrinks with the separation,
# and noise decides the answer. 45 deg left and 45 deg right gives 90.
MIN_SEPARATION_DEG = 20.0


def find_candidates(clusters, min_mm, max_mm, min_points=MIN_CLUSTER_POINTS):
    """Clusters that could be the calibration cone, best first.

    Ranked by point count, then by nothing else — a second plausible candidate
    means the "nothing else nearby" instruction was not met, and the caller is
    expected to say so rather than quietly take the winner.
    """
    hits = [c for c in clusters
            if min_mm <= c.range_mm <= max_mm and c.points >= min_points]
    hits.sort(key=lambda c: c.points, reverse=True)
    return hits


class Observation(object):
    """One cone pose, measured over many revolutions.

    `spread_deg` across scans is the number that says whether to trust it: the
    car and the cone are both still, so anything above a fraction of a degree
    means something moved or the wrong cluster was picked on some scans.
    """

    __slots__ = ("expected_deg", "bearing_deg", "range_mm", "points",
                 "spread_deg", "scans", "ambiguous_scans")

    def __init__(self, expected_deg, bearing_deg, range_mm, points, spread_deg,
                 scans, ambiguous_scans):
        self.expected_deg = expected_deg
        self.bearing_deg = bearing_deg
        self.range_mm = range_mm
        self.points = points
        self.spread_deg = spread_deg
        self.scans = scans
        self.ambiguous_scans = ambiguous_scans

    def as_dict(self):
        return {
            "expected_car_bearing_deg": self.expected_deg,
            "measured_sensor_bearing_deg": round(self.bearing_deg, 2),
            "range_mm": round(self.range_mm, 1),
            "points_per_scan": round(self.points, 1),
            "bearing_spread_deg": round(self.spread_deg, 2),
            "scans": self.scans,
            "scans_with_rival_cluster": self.ambiguous_scans,
        }

    def __repr__(self):
        return (f"Observation(expect {self.expected_deg:+.0f} deg -> sensor "
                f"{self.bearing_deg:.1f} deg at {self.range_mm:.0f} mm, "
                f"+-{self.spread_deg:.2f} deg over {self.scans} scans)")


def measure_pose(scans, expected_deg, target_mm, tolerance_mm,
                 min_points=MIN_CLUSTER_POINTS):
    """Find the cone in each scan and combine the per-scan bearings.

    Per-scan rather than pooling every point into one cluster pass: pooling
    hides a scan where the wrong object won, and the spread across scans is the
    only evidence available that the same object was found each time.
    """
    bearings, ranges, counts = [], [], []
    ambiguous = 0
    for scan in scans:
        clusters = cluster_scan(scan.angles_deg, scan.ranges_mm)
        hits = find_candidates(clusters, target_mm - tolerance_mm,
                               target_mm + tolerance_mm, min_points)
        if not hits:
            continue
        if len(hits) > 1:
            ambiguous += 1
        bearings.append(hits[0].bearing_deg)
        ranges.append(hits[0].range_mm)
        counts.append(hits[0].points)

    if not bearings:
        return None

    centre = circular_mean(bearings)
    return Observation(
        expected_deg=expected_deg,
        bearing_deg=centre,
        range_mm=_median(ranges),
        points=sum(counts) / float(len(counts)),
        spread_deg=circular_spread(bearings, centre),
        scans=len(bearings),
        ambiguous_scans=ambiguous,
    )


class Solution(object):
    """The convention the observations imply, and how well they imply it.

    `rival_residual_deg` is the same fit under the opposite sign. When the two
    residuals are close the poses did not separate the hypotheses and the answer
    is a coin flip — which is exactly the failure a single cone produces, so it
    is reported rather than left for the reader to infer.
    """

    __slots__ = ("mirror", "angle_offset_deg", "residual_deg", "rival_residual_deg",
                 "separation_deg", "observations")

    def __init__(self, mirror, angle_offset_deg, residual_deg, rival_residual_deg,
                 separation_deg, observations):
        self.mirror = mirror
        self.angle_offset_deg = angle_offset_deg
        self.residual_deg = residual_deg
        self.rival_residual_deg = rival_residual_deg
        self.separation_deg = separation_deg
        self.observations = observations

    @property
    def decisive(self):
        """True when the winning sign beats the other by a clear margin."""
        return self.rival_residual_deg - self.residual_deg >= 5.0

    def flags(self):
        """The command-line form, ready to paste."""
        parts = ["--mirror"] if self.mirror else []
        parts.append(f"--angle-offset {self.angle_offset_deg:.1f}")
        return " ".join(parts)

    def car_bearing(self, sensor_bearing_deg):
        sign = 1.0 if self.mirror else -1.0
        return wrap180(sign * sensor_bearing_deg + self.angle_offset_deg)

    def __repr__(self):
        return (f"Solution(mirror={self.mirror}, "
                f"offset={self.angle_offset_deg:.2f} deg, "
                f"residual={self.residual_deg:.2f} deg)")


def solve_convention(observations, min_separation_deg=MIN_SEPARATION_DEG):
    """Fit sign and yaw to poses at known bearings.

    Raises ValueError when the poses cannot determine the sign — too few, or
    too close together. Returning a confident-looking answer there would defeat
    the entire point of measuring instead of eyeballing.
    """
    if len(observations) < 2:
        raise ValueError(
            "need at least two cone poses: one cannot separate a mirrored sign "
            "from a yaw offset, because both fit a single point exactly")

    separation = max(
        abs(wrap180(a.bearing_deg - b.bearing_deg))
        for i, a in enumerate(observations) for b in observations[i + 1:])
    if separation < min_separation_deg:
        raise ValueError(
            f"cone poses are only {separation:.1f} deg apart in sensor bearing; "
            f"need {min_separation_deg:.0f} deg for the sign to be determined. "
            f"Use one pose well left and one well right.")

    fits = []
    for mirror in (False, True):
        sign = 1.0 if mirror else -1.0
        offsets = [wrap360(o.expected_deg - sign * o.bearing_deg)
                   for o in observations]
        centre = circular_mean(offsets)
        if centre is None:
            continue
        residual = max(abs(wrap180(off - centre)) for off in offsets)
        fits.append((residual, mirror, wrap180(centre)))

    fits.sort(key=lambda f: f[0])
    residual, mirror, offset = fits[0]
    rival = fits[1][0] if len(fits) > 1 else float("inf")
    return Solution(mirror, offset, residual, rival, separation, list(observations))


class Arc(object):
    """A contiguous span of bearings that always returns something very close."""

    __slots__ = ("start_deg", "end_deg", "near_mm", "far_mm", "presence")

    def __init__(self, start_deg, end_deg, near_mm, far_mm, presence):
        self.start_deg = start_deg
        self.end_deg = end_deg
        self.near_mm = near_mm
        self.far_mm = far_mm
        self.presence = presence

    @property
    def width_deg(self):
        return wrap360(self.end_deg - self.start_deg)

    @property
    def mid_deg(self):
        return wrap360(self.start_deg + self.width_deg / 2.0)

    def as_dict(self):
        return {
            "start_deg": round(self.start_deg, 1),
            "end_deg": round(self.end_deg, 1),
            "near_mm": round(self.near_mm),
            "far_mm": round(self.far_mm),
            "presence": round(self.presence, 3),
        }

    def __repr__(self):
        return (f"Arc({self.start_deg:.0f}-{self.end_deg:.0f} deg, "
                f"{self.near_mm:.0f}-{self.far_mm:.0f} mm)")


def chassis_arcs(scans, max_mm=CHASSIS_MAX_MM, bin_deg=1.0, presence=0.8):
    """Bearings where the car sees itself: near returns present in most scans.

    Persistence is the discriminator. A person standing beside the car is near
    too, but drifts and eventually moves; the chassis is in the same bearings on
    every revolution. Worth measuring rather than assuming — the masking limits
    in the DonkeyCar config are only correct for the mount they were written
    for.
    """
    scans = list(scans)
    if not scans:
        return []
    bins = int(round(360.0 / bin_deg))
    seen = [0] * bins
    near = [None] * bins
    far = [None] * bins

    for scan in scans:
        hit = set()
        for angle, dist in zip(scan.angles_deg, scan.ranges_mm):
            if dist < 1 or dist > max_mm:
                continue
            idx = int(wrap360(angle) / 360.0 * bins) % bins
            hit.add(idx)
            near[idx] = dist if near[idx] is None else min(near[idx], dist)
            far[idx] = dist if far[idx] is None else max(far[idx], dist)
        for idx in hit:
            seen[idx] += 1

    threshold = presence * len(scans)
    flags = [seen[i] >= threshold and seen[i] > 0 for i in range(bins)]
    if all(flags):
        return [Arc(0.0, 360.0, min(n for n in near if n is not None),
                    max(f for f in far if f is not None), 1.0)]
    if not any(flags):
        return []

    # Rotate the start so a run that straddles zero is one arc, not two.
    start = next(i for i in range(bins) if flags[i] and not flags[i - 1])
    arcs = []
    run = []
    for step in range(bins + 1):
        idx = (start + step) % bins
        if step < bins and flags[idx]:
            run.append(idx)
            continue
        if run:
            arcs.append(Arc(
                start_deg=run[0] * bin_deg,
                end_deg=(run[-1] + 1) * bin_deg % 360.0,
                near_mm=min(near[i] for i in run),
                far_mm=max(far[i] for i in run),
                presence=min(seen[i] for i in run) / float(len(scans)),
            ))
            run = []
    return arcs


def build_record(solution, arcs=None, mount=None, health=None, notes=None,
                 git_commit=None, target_mm=None):
    """The JSON written to calibration.json and echoed into every session."""
    return {
        "format_version": FORMAT_VERSION,
        "measured_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "git_commit": git_commit,
        "mirror": solution.mirror,
        "angle_offset_deg": round(solution.angle_offset_deg, 2),
        "fit": {
            "residual_deg": round(solution.residual_deg, 2),
            "rival_residual_deg": (None if solution.rival_residual_deg == float("inf")
                                   else round(solution.rival_residual_deg, 2)),
            "separation_deg": round(solution.separation_deg, 1),
            "decisive": solution.decisive,
        },
        "target_range_mm": target_mm,
        "observations": [o.as_dict() for o in solution.observations],
        "chassis_arcs_sensor": [a.as_dict() for a in (arcs or [])],
        "mount": mount,
        "link_health": health,
        "notes": notes,
    }


def save(record, path):
    path = os.path.expanduser(path)
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(record, fh, indent=2)
        fh.write("\n")
    # Atomic: a calibration half-written by a Ctrl+C is worse than none, because
    # the next run would load it without complaint.
    os.replace(tmp, path)
    return path


def load(path):
    """Read calibration.json, or None if it is absent. Malformed is an error."""
    path = os.path.expanduser(path)
    try:
        with open(path) as fh:
            record = json.load(fh)
    except FileNotFoundError:
        return None
    except (OSError, ValueError) as exc:
        raise ValueError(f"{path} is not readable as calibration JSON: {exc}")
    if "mirror" not in record or "angle_offset_deg" not in record:
        raise ValueError(f"{path} has no mirror/angle_offset_deg; re-run --calibrate")
    return record


def default_path(tool_dir=None):
    base = tool_dir or os.path.dirname(os.path.abspath(__file__))
    return os.path.join(base, DEFAULT_FILENAME)
