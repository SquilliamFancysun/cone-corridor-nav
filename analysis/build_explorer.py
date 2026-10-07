"""Replay a trial log through the corridor pipeline and write the explorer page.

    python analysis/build_explorer.py                       # explore-run-1854
    python analysis/build_explorer.py data/trials/explore-op-1.jsonl

Writes docs/explorer/index.html: a self-contained page that steps through one
control cycle at a time -- the cones the car saw, the Delaunay triangulation,
the edges that survive the colour and length rules, the centreline, and the
pure pursuit arc.

## What is replayed, and what is not

Every tick of an explore run logs `cones_xy` -- the cones fusion produced, in
base_link, PRE-fill and pre-branch-filter -- and the scan-matched pose. This
script feeds those cones through the same code `drive_junction.py` runs:

    side_assign.fill_unlabeled -> centerline.centerline -> pure_pursuit.steering_angle

with the previous tick's centreline heading as the fill's reference axis, as
on the car. Three junction-only steps are NOT replayed, because they depend on
the topology state machine rather than on the tick's cones: the gate-line mask
on the fill, `junction_exec.keep_branch`, and the gate/goal/wall anchors. So
through a junction the line drawn here can differ from the one the car drove.
The car's own logged lookahead target is carried into the page and drawn
beside the recomputed one, so the difference is visible rather than hidden.

The run is split into legs at each declared lift (`pose_jumps`), because a
carried car's pose is in a new frame; each leg gets its own background map
from `map_from_log.build`.
"""

import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for sub in ("src", "model/capture", ""):
    path = os.path.join(ROOT, sub)
    if path not in sys.path:
        sys.path.insert(0, path)

from cone_perception import extrinsics                          # noqa: E402
from cone_perception.cone_classes import CLASS_NAMES, UNLABELED  # noqa: E402
from cone_perception.fusion import LabeledCone                  # noqa: E402
from cone_nav.corridor import side_assign                       # noqa: E402
from cone_nav.corridor.centerline import centerline, GATE       # noqa: E402
from cone_nav.corridor.delaunay import triangulate, edges_of    # noqa: E402
from cone_nav.control import pure_pursuit                       # noqa: E402

sys.path.insert(0, HERE)
import map_from_log                                             # noqa: E402

DEFAULT_LOG = os.path.join(ROOT, "data", "trials", "explore-run-1854.jsonl")
TEMPLATE = os.path.join(ROOT, "docs", "explorer", "template.html")
OUTPUT = os.path.join(ROOT, "docs", "explorer", "index.html")

LOOKAHEAD_M = 0.8      # --lookahead on the car
HALF_ARC_DEG = 109.0   # chassis blocks the rear 142 deg of the LD06
MAX_RANGE_M = 3.5      # --max-range on the car
EVERY = 2              # keep every Nth tick; 10 Hz -> 5 frames a second
PATH_COLOURS = ("blue", "yellow", "red")
LEAD_IN_TICKS = 30      # after a lift, start this long before the car is re-armed
MIN_MAP_SIGHTINGS = 4   # background map: drop clutter seen only in passing


def r3(v):
    return round(v, 3)


class Frame(object):
    def __init__(self, pose):
        self.x, self.y, self.yaw = pose

    def world(self, p):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return [r3(self.x + p[0] * c - p[1] * s), r3(self.y + p[0] * s + p[1] * c)]


def split_legs(rows):
    """Rows -> [(start, end)] split at each lift, ending a leg at the goal."""
    legs, start, jumps = [], 0, rows[0].get("pose_jumps", 0) if rows else 0
    for i, row in enumerate(rows):
        if row.get("pose_jumps", 0) != jumps:
            legs.append((start, i))
            # Skip the carry itself: start shortly before the car is armed again.
            armed = next((j for j in range(i, len(rows)) if rows[j].get("armed")), i)
            start, jumps = max(i, armed - LEAD_IN_TICKS), row.get("pose_jumps", 0)
    legs.append((start, len(rows)))
    trimmed = []
    for a, b in legs:
        for i in range(a, b):
            if rows[i].get("stop_reason") == "goal reached":
                b = min(b, i + 15)
                break
        if b - a > 20:
            trimmed.append((a, b))
    return trimmed


def replay(rows):
    previous = None
    frames = []
    for k, row in enumerate(rows):
        cones_raw = map_from_log.parse_cones(row.get("cones_xy", ""))
        axis = side_assign.heading_of(previous) if previous is not None else 0.0
        cones = [LabeledCone(cls, 1.0, x, y) for x, y, cls in cones_raw]
        filled, _ = side_assign.fill_unlabeled(cones, reference_heading_rad=axis)
        line = centerline(filled, car_xy=(0.0, 0.0))
        previous = line
        if k % EVERY:
            continue

        pose = Frame((row.get("pose_x", 0.0), row.get("pose_y", 0.0),
                      math.radians(row.get("pose_yaw_deg", 0.0))))
        usable = [c for c in filled
                  if c.cone_class != UNLABELED and CLASS_NAMES[c.cone_class] in PATH_COLOURS]
        pts = [(c.x, c.y) for c in usable]
        tris = triangulate(pts)

        cones_out = []
        for raw, out in zip(cones, filled):
            source = ("cam" if raw.cone_class != UNLABELED
                      else "geo" if out.cone_class != UNLABELED else "none")
            name = CLASS_NAMES[out.cone_class] if out.cone_class != UNLABELED else "unlabeled"
            cones_out.append(pose.world((raw.x, raw.y)) + [name, source])

        axle = extrinsics.REAR_AXLE_IN_BASE[:2]
        pursuit = pure_pursuit.steering_angle(line.points, LOOKAHEAD_M,
                                              extrinsics.WHEELBASE_M, origin=axle)
        logged_target = None
        if row.get("target_x") is not None and not row.get("stop_reason"):
            logged_target = pose.world((row["target_x"], row["target_y"]))

        frames.append({
            "t": round(row.get("t", 0.0), 1),
            "pose": [r3(pose.x), r3(pose.y), round(math.degrees(pose.yaw), 1)],
            "axle": pose.world(axle),
            "cones": cones_out,
            "edges": [[pose.world(pts[i]), pose.world(pts[j])] for i, j in edges_of(tris)],
            "tris": [[pose.world(pts[i]), pose.world(pts[j]), pose.world(pts[m])] for i, j, m in tris],
            "mids": [{"p": pose.world(m.xy), "kind": m.kind,
                      "a": pose.world(pts[m.a]), "b": pose.world(pts[m.b])}
                     for m in (line.midpoints or [])],
            "chain": [pose.world(p) for p in line.points],
            "fallback": bool(line.single_boundary_fallback),
            "target": pose.world(pursuit.target) if pursuit is not None else None,
            "steer": round(math.degrees(pursuit.delta_rad), 1) if pursuit is not None else None,
            "logged_target": logged_target,
            "logged_steer": row.get("steer_deg"),
            "topo": row.get("topo_state", ""),
            "turn": row.get("turn", ""),
            "armed": bool(row.get("armed")),
            "note": row.get("stop_reason", ""),
        })
    return frames


def leg_map(rows):
    marks, _, _ = map_from_log.build(rows, stop_at_lift=False)
    return [{"c": CLASS_NAMES[m.cone_class], "x": r3(m.x), "y": r3(m.y)}
            for m in marks
            if m.cone_class != UNLABELED and m.sightings >= MIN_MAP_SIGHTINGS]


def default_frame(frames):
    def score(f):
        return (f["armed"], sum(1 for m in f["mids"] if m["kind"] == GATE), len(f["chain"]))
    return max(range(len(frames)), key=lambda i: score(frames[i]))


def build(log_path):
    rows = [r for r in map_from_log.load(log_path)]
    legs = []
    for n, (a, b) in enumerate(split_legs(rows), start=1):
        part = rows[a:b]
        frames = replay(part)
        legs.append({
            "name": f"Leg {n}",
            "span": [round(part[0].get("t", 0.0), 1), round(part[-1].get("t", 0.0), 1)],
            "map": leg_map(part),
            "frames": frames,
            "default": default_frame(frames),
        })
    return {"log": os.path.basename(log_path), "legs": legs, "lookahead": LOOKAHEAD_M,
            "range": MAX_RANGE_M, "half_arc": HALF_ARC_DEG}


SHELL = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>html,body{margin:0}</style>
</head>
<body>
%s
</body>
</html>
"""


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("log", nargs="?", default=DEFAULT_LOG)
    parser.add_argument("--out", default=OUTPUT)
    parser.add_argument("--fragment", action="store_true",
                        help="write the page body only, without the html/head shell")
    args = parser.parse_args(argv)

    data = build(args.log)
    with open(TEMPLATE, encoding="utf-8") as handle:
        page = handle.read().replace("__DATA__", json.dumps(data).replace("</", "<\\/"))
    if not args.fragment:
        page = SHELL % page
    with open(args.out, "w", encoding="utf-8") as handle:
        handle.write(page)
    for leg in data["legs"]:
        print(f"{leg['name']}: t {leg['span'][0]}-{leg['span'][1]} s, "
              f"{len(leg['frames'])} frames, {len(leg['map'])} landmarks")
    print(f"wrote {args.out} ({os.path.getsize(args.out) // 1024} KB)")


if __name__ == "__main__":
    main()
