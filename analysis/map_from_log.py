"""Build a cone map out of a trial log, and check it against the tape.

    python analysis/map_from_log.py data/trials/run.jsonl \
        --layout data/layouts/track_v1.csv

Every tick of a drive_junction run records where the car thinks it is
(`pose_x`, `pose_y`, `pose_yaw_deg`) and where it sees cones relative to itself
(`cones_xy`). Pushing the second through the first puts every sighting in one
frame, and the same cone seen from twenty places collapses to one landmark.

## What this is for

Two questions, and the second is the one that matters.

The pretty one: a plan view of the course the car built for itself, next to the
surveyed truth. That is a report figure.

The load-bearing one: **is the odometry good enough to carry an edge length?**
`cone_perception/odometry.py` is a random walk and says so; `graph_builder`
leans on it for edge lengths and the planner does not read them, so nothing has
ever had to be right. The moment the maze has a loop, `identify()` needs a
pose-derived key and this drift becomes load-bearing. `--layout` answers it now,
cheaply, by nearest-neighbour matching the built map onto the surveyed one and
reporting the residual.

## Why the deadband is a flag here

`ego_motion.DEADBAND_M` exists because `topo_state` clamps negative travel, so
jitter random-walks a distance FLOOR upward while the car stands still. A pose
integrator sums SIGNED steps, so jitter should largely cancel and a deadband
should only under-count slow motion -- which is why `odometry.Pose` defaults it
off. That is an argument, not a measurement. `--deadband` re-integrates the run
both ways from the logged per-tick steps so the residual can settle it.

## What it will not do

A log with `pose_jumps` above zero has been carried, and everything mapped after
the first lift is in a different frame -- the map is valid up to that point and
scrap after it. That is not repairable here: nothing knows the transform across
a lift. Build the figure from a run with no lifts, which is what the re-drive of
an emitted optimal route is.
"""

import argparse
import csv
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, ".."))
_src = os.path.join(_REPO, "src")
if os.path.isdir(_src) and _src not in sys.path:
    sys.path.insert(0, _src)

from cone_perception import ego_motion, odometry
from cone_perception.cone_classes import CLASS_NAMES, UNLABELED, name_of

# Two sightings closer than this are the same cone. A cone is 0.30 m across at
# most (clustering.MAX_CONE_WIDTH_M) and the tightest legal spacing on any
# layout here is 0.50 m, so 0.35 m merges a landmark with itself without ever
# merging it with its neighbour.
MERGE_M = 0.35

# A landmark seen once is a cluster that appeared for one revolution, which is
# what noise looks like. Two is the same floor `clustering` puts on returns.
MIN_SIGHTINGS = 2


class Landmark(object):
    """One cone in the world frame, averaged over every tick that saw it."""

    __slots__ = ("x", "y", "sightings", "classes")

    def __init__(self, x, y, cone_class):
        self.x = x
        self.y = y
        self.sightings = 1
        self.classes = {cone_class: 1}

    def absorb(self, x, y, cone_class):
        # Running mean: the estimate improves with every look, and no tick's
        # reading is privileged over another's.
        n = self.sightings + 1
        self.x += (x - self.x) / n
        self.y += (y - self.y) / n
        self.sightings = n
        self.classes[cone_class] = self.classes.get(cone_class, 0) + 1

    @property
    def cone_class(self):
        """The label seen most often, ignoring UNLABELED unless that is all
        there was. A cone the camera reached twice out of thirty ticks is that
        colour; geometry never disagreed with it, it just never spoke."""
        labelled = {k: v for k, v in self.classes.items() if k != UNLABELED}
        pool = labelled or self.classes
        return max(pool, key=lambda k: pool[k])

    def __repr__(self):
        return (f"Landmark({self.x:+.2f}, {self.y:+.2f}, "
                f"{name_of(self.cone_class)}, x{self.sightings})")


def parse_cones(text):
    """"x,y,class;..." -> [(x, y, class)]. Tolerates an empty field."""
    out = []
    for part in (text or "").split(";"):
        if not part:
            continue
        bits = part.split(",")
        if len(bits) != 3:
            continue
        try:
            out.append((float(bits[0]), float(bits[1]), int(bits[2])))
        except ValueError:
            continue
    return out


def load(path):
    rows = []
    with open(path, encoding="utf-8") as handle:
        for number, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except ValueError:
                print(f"note: line {number} is truncated; ignoring it",
                      file=sys.stderr)
    return rows


def poses(rows, deadband_m=None):
    """One (x, y, yaw_rad) per row.

    Uses the logged pose unless `deadband_m` is given, in which case the run is
    re-integrated from the logged per-tick steps under that deadband. That is
    what makes the deadband question measurable rather than arguable -- and it
    is only possible because `odo_lateral_m` and `odo_yaw_deg` are logged
    beside `odo_forward_m`.
    """
    if deadband_m is None:
        return [(r.get("pose_x", 0.0), r.get("pose_y", 0.0),
                 math.radians(r.get("pose_yaw_deg", 0.0))) for r in rows]

    pose = odometry.Pose()
    out = []
    for row in rows:
        step = ego_motion.Step(row.get("odo_forward_m", 0.0),
                               row.get("odo_lateral_m", 0.0),
                               math.radians(row.get("odo_yaw_deg", 0.0)),
                               row.get("odo_pairs", 0))
        pose.integrate(step if step.pairs else None, deadband_m=deadband_m)
        out.append((pose.x, pose.y, pose.yaw_rad))
    return out


def build(rows, deadband_m=None, merge_m=MERGE_M, min_sightings=MIN_SIGHTINGS,
          stop_at_lift=True):
    """Rows -> (landmarks, path, ticks_used).

    Stops at the first declared lift by default. Everything after one is in a
    different frame and would be scattered across the map as a second, offset
    copy of the course -- which looks like drift and is not.
    """
    landmarks, path, used = [], [], 0
    for row, (px, py, yaw) in zip(rows, poses(rows, deadband_m)):
        if stop_at_lift and row.get("pose_jumps", 0):
            break
        used += 1
        path.append((px, py))
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        for cx, cy, cone_class in parse_cones(row.get("cones_xy", "")):
            wx = px + cx * cos_y - cy * sin_y
            wy = py + cx * sin_y + cy * cos_y
            best, best_d = None, merge_m
            for mark in landmarks:
                d = math.hypot(mark.x - wx, mark.y - wy)
                if d < best_d:
                    best, best_d = mark, d
            if best is None:
                landmarks.append(Landmark(wx, wy, cone_class))
            else:
                best.absorb(wx, wy, cone_class)
    return ([m for m in landmarks if m.sightings >= min_sightings],
            path, used)


def load_layout(path):
    """The surveyed truth. `data/layouts/*.csv` columns: id,color,x_m,y_m,segment."""
    out = []
    with open(path, encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            try:
                out.append((float(row["x_m"]), float(row["y_m"]),
                            (row.get("color") or "").strip()))
            except (KeyError, ValueError, TypeError):
                continue
    return out


def residuals(landmarks, layout, gate_m=1.0):
    """Nearest-neighbour distance from each landmark to a surveyed cone.

    `gate_m` is generous on purpose. This measures DRIFT, and a drift big
    enough to exceed it is the finding -- silently dropping those landmarks
    would report a small residual over the handful that happened to stay put.
    Unmatched ones are counted and reported separately.
    """
    matched, unmatched = [], 0
    for mark in landmarks:
        best = min((math.hypot(mark.x - x, mark.y - y)
                    for x, y, _c in layout), default=None)
        if best is None or best > gate_m:
            unmatched += 1
        else:
            matched.append(best)
    return matched, unmatched


def plot(landmarks, path, layout=(), width=78, height=26):
    """A plan view in text. No matplotlib on the car, and the figure that gets
    looked at is the one that renders where the work is happening."""
    points = ([(m.x, m.y) for m in landmarks] + list(path)
              + [(x, y) for x, y, _c in layout])
    if not points:
        return "  (nothing to plot)"
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    sx = (width - 1) / (x1 - x0) if x1 > x0 else 1.0
    sy = (height - 1) / (y1 - y0) if y1 > y0 else 1.0
    grid = [[" "] * width for _ in range(height)]

    def put(x, y, ch, over=False):
        # World x is forward and world y is left, so the plan view puts x
        # across and y up -- north-up, the way the layout sheets are drawn.
        col = int(round((x - x0) * sx))
        rowi = height - 1 - int(round((y - y0) * sy))
        if 0 <= col < width and 0 <= rowi < height:
            if over or grid[rowi][col] == " ":
                grid[rowi][col] = ch

    for x, y, _c in layout:
        put(x, y, ".")
    for x, y in path:
        put(x, y, "-", over=True)
    for mark in landmarks:
        name = name_of(mark.cone_class)
        put(mark.x, mark.y, name[0].upper() if name in CLASS_NAMES else "?",
            over=True)
    return "\n".join("  " + "".join(row) for row in grid)


def report(rows, path_name, layout=(), deadband_m=None):
    if not rows:
        print(f"{path_name}: no ticks.")
        return 1
    if not any(r.get("cones_xy") for r in rows):
        print(f"{path_name}: no `cones_xy` in this log, so there is nothing to\n"
              "  map. Logs written before that field existed cannot be mapped\n"
              "  retrospectively -- the cone positions were never recorded.")
        return 1

    lifts = max(r.get("pose_jumps", 0) for r in rows)
    landmarks, driven, used = build(rows, deadband_m=deadband_m)

    print(f"{path_name}: {len(rows)} ticks, {used} mapped")
    if lifts:
        print(f"  {lifts} declared lift(s) -- mapping STOPPED at the first.")
        print("  Everything after a lift is in a different frame. Build the")
        print("  figure from a run with no lifts; the re-drive of an emitted")
        print("  optimal route is one.")
    print(f"  {len(landmarks)} landmarks, driven path "
          f"{sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(driven, driven[1:])):.2f} m")
    print()
    print(plot(landmarks, driven, layout))

    if layout:
        matched, unmatched = residuals(landmarks, layout)
        print()
        print("  against the surveyed layout")
        if matched:
            mean = sum(matched) / len(matched)
            ordered = sorted(matched)
            p95 = ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))]
            print(f"    {len(matched)} matched, mean {mean:.3f} m, "
                  f"p95 {p95:.3f} m, worst {max(matched):.3f} m")
        if unmatched:
            print(f"    {unmatched} landmark(s) further than 1.00 m from any "
                  "surveyed cone")
        print()
        print("    This is the number that says whether odometry can carry an")
        print("    edge length. Under ~0.10 m it can; a metre of drift over a")
        print("    20 m course means a pose-derived node key would put two")
        print("    different junctions in the same place.")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Build a cone map from a drive_junction trial log.")
    parser.add_argument("log", help="path to the JSONL written by --log")
    parser.add_argument("--layout", help="surveyed CSV to score against, e.g. "
                                         "data/layouts/track_v1.csv")
    parser.add_argument("--deadband", type=float, default=None,
                        help="re-integrate the run under this deadband in "
                             "metres instead of using the logged pose. "
                             "Pass 0.008 to compare against ego_motion's, "
                             "which is the comparison odometry.Pose's default "
                             "is a prediction about")
    args = parser.parse_args(argv)
    layout = load_layout(args.layout) if args.layout else ()
    return report(load(args.log), args.log, layout, deadband_m=args.deadband)


if __name__ == "__main__":
    sys.exit(main())
