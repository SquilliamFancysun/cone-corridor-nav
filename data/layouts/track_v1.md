# Track v1 — two-fork corridor

The course the car is built to run, and the course the CV dataset is captured
on. Build it once, survey it once, and use the same geometry for training data,
trial ground truth, and `sim/` generation.

A corridor with two **Y-junctions**, each offering a dead end and a correct path,
ending at a green goal cone. The route is *provided* to the vehicle, so the dead
ends do not have to be discovered — they exist to make the route meaningful and
to give us a real failure mode to measure.

## Why forks and not T-junctions

A T forces a 90° turn, and through that turn the corridor swings entirely out of
the OAK-D Lite's ~69° horizontal field of view: the car goes blind at exactly the
moment it most needs to see. Shallow forks avoid this. Branches diverge **±25°**
from the incoming centerline, well inside ±34.5°, so both branches stay in frame
throughout the maneuver, the centerline stays continuous for pure pursuit, and
minimum turning radius stops being a design constraint.

## Frame

Origin at the midpoint of the start line. **x forward along Corridor A, y left,
meters** — the same convention as `cone_msgs/msg/LabeledCone.msg`, so surveyed
positions and perception output are directly comparable.

## Layout

```
       DEAD END A                              DEAD END B
       (1.5 m stub)                            (1.5 m stub)
              ╲                                    ╱
               ╲                                  ╱
                ╲                                ╱
  START ─────────Y──────── Corridor B ──────────Y─────────── ● GOAL
    Corridor A  [J1]          (3.5 m)          [J2]   Corridor C (3 m)
      (3 m)      ↑                              ↑
              ●● orange                      ●● orange
              pair, 1 m back                 pair, 1 m back

           route: LEFT ↗                   route: RIGHT ↘
           (right branch is               (left branch is
            the dead end)                  the dead end)
```

Net path is a gentle S: heading goes 0° → +25° → 0°. Bounding box ≈ 9.5 × 4 m.

If the site is tight, route LEFT at both junctions instead — the course curls
rather than snakes and the footprint drops to roughly 7 × 6 m.

## Parameters

| Parameter | Value | Why |
|---|---|---|
| Corridor width | 1.5 m, uniform | No flare needed; nothing here requires a tight turn |
| Branch divergence | ±25° from incoming centerline | Keeps both branches inside the ~69° HFOV |
| Segment length | A 3 m, B 3.5 m, C 3 m | |
| Cone spacing | 1.5 m on straights, 0.75 m through fork mouths and dead-end walls | Boundary ambiguity bites at the forks; straights tolerate sparse cones |
| Dead-end stub | 1.5 m deep, walled across the end | Long enough that the car commits before the wall is obvious |
| Orange gates | Pairs, straddling the corridor 1.0 m before each fork | `GateEvent.distance` is "meters to gate midpoint"; `gate_detect.py` keys on *pairs*, so orange never goes down singly |
| Green goal | One cone, centered, end of Corridor C | |
| Route | LEFT at J1, RIGHT at J2 | |

**Cone budget** ≈ 45: 18 for the three corridors, 14 for the two fork mouths and
islands, 12 for the two dead ends, 4 orange, 1 green. Short on cones? Spend them
at the forks first and stretch the straights to 2 m — straights are the part the
corridor layer extrapolates well.

## Cone colors at a fork

Each branch keeps blue-left / yellow-right **in its own direction of travel**.
That resolves the island between the branches automatically: its left face (the
left branch's right wall) is yellow, its right face (the right branch's left
wall) is blue. Build the island nose as one yellow and one blue cone side by
side, then taper outward. No fifth class, no ambiguity.

## Cone height

Use **one cone size for the entire track** and record it here and in
`cone_perception/extrinsics.py` as `CONE_HEIGHT_M`. The `range_bbox` channel of
`LabeledCone.msg` is `Z = f * h_real / h_pixels`; a mixed-height track silently
corrupts that estimate for every cone that differs.

- Cone height (base to tip): ____ m
- Cone base width: ____ m

## Survey procedure

Per `data/README.md`, fix the origin and axes **before** the first measurement.
Measure to **cone base centers**. One shared sheet.

Record into `track_v1.csv` with columns:

```
id,color,x_m,y_m,segment
```

where `color` is one of `blue`, `yellow`, `orange`, `green` and matches the class
names used in Roboflow. The CSV holds **measured** positions, not the design
numbers above — lay the track out from this document with a tape, then record
where the cones actually ended up. Photograph the finished track from a fixed
vantage and note where that vantage was, so a rebuild can be checked against the
photo.

This CSV is the D5 deliverable and the ground truth `analysis/` uses for
cross-track error.

## Known consequence for the nav stack

A fork buys continuous visibility at the cost of *ambiguity*. Approaching a Y,
the camera sees two blue walls and two yellow walls at once, so a naive "all blue
cones are my left boundary" rule in `cone_nav/corridor/boundary_split.py` will
fuse two corridors into nonsense.

That is what the orange gate pair is for. `EVENT_GATE_IN_RANGE` is the trigger to
hand off from centerline-following to `cone_nav/guidance/junction_exec.py`, which
uses the provided route to pick which branch's cones to keep. Write the corridor
layer knowing it will sometimes see two corridors, not one.
