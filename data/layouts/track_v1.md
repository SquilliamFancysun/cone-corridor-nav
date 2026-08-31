# Track v1 — two-fork corridor

The course the car is built to run, and the course the CV dataset is captured
on. Build it once, survey it once, and use the same geometry for training data,
trial ground truth, and `sim/` generation.

A corridor with two **Y-junctions**, each offering a dead end and a correct path,
ending at a magenta goal cone. The route is *provided* to the vehicle, so the dead
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
              ●● red                         ●● red
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
| Island nose | 1.78 m past each junction | `half_width / sin(divergence)` — see below |
| Segment length | A 3 m, B 3.5 m, C 3 m | |
| Cone spacing | **0.75 m throughout** | Was 1.5 m on straights. Measured wrong — see [Cone spacing is not a comfort setting](#cone-spacing-is-not-a-comfort-setting) |
| Dead-end stub | 1.5 m deep, walled across the end | Long enough that the car commits before the wall is obvious |
| Red gates | Pairs, straddling the corridor 1.0 m before each fork | `GateEvent.distance` is "meters to gate midpoint"; `gate_detect.py` keys on *pairs*, so red never goes down singly |
| Orange dead ends | The middle cone of each dead-end end wall | Its own class, so a stub is recognisable before the car commits to it |
| Magenta goal | One cone, centered, end of Corridor C | |
| Route | LEFT at J1, RIGHT at J2 | |

## Cone budget

Every corridor segment has exactly one blue wall and one yellow wall, so each
colour needs a cone line as long as the whole driven layout — **blue and yellow
counts are always equal.** Total segment length is 3 + 3.5 + 3 + 1.5 + 1.5 =
12.5 m per colour.

| Colour | Full build | Minimum | What it covers |
|---|---|---|---|
| **Blue** | **18** | 13 | 12.5 m of left-hand wall, in three runs (outer envelope 8 m, J1 island face 1.5 m, Corridor C island face 3 m) + fork densification |
| **Yellow** | **18** | 13 | 12.5 m of right-hand wall, in three runs (4.5 m, 6.5 m, 1.5 m) + fork densification |
| **Red** | **4** | 4 | Two gate pairs, one per fork. Not reducible — see below |
| **Orange** | **2** | 2 | The middle cone of each dead-end end wall |
| **Magenta** | **1** | 1 | The goal |
| | **43** | **33** | |

~~Full build uses 1.5 m spacing on straights; the minimum stretches straights to
2 m and keeps 0.75 m through the fork mouths.~~ **Superseded.** Both figures
predate the sensor-overlap measurement above: at 1.5 m the car cannot follow the
corridor at all, and 2 m is further past the limit. Use 0.75 m throughout, which
raises the blue and yellow counts to roughly 17 per colour per 12.5 m of wall.
1.0 m also works, and 1.25 m passes by a single row; 0.75 m is the one with
margin.

There is no longer a "cut from the straights" option — the straights are the
part that needs the density, because the overlap window is the same 1.8 m deep
wherever the car is. Cut LENGTH instead if cones are short: a 6 m corridor at
0.75 m spacing drives, and a 12 m corridor at 1.5 m does not.

**Red does not scale down.** `gate_detect.py` keys on *pairs*, and
`GateEvent.distance` is the range to a gate's midpoint, so a lone red cone is
not a weak gate — it is an undetectable one. With only two red cones, mark one
junction properly and leave the other unmarked rather than splitting a pair
across both.

**Buy spares of blue and yellow** (2–3 each). Cones get run over.

**Consider 2 extra each of red, orange and magenta** beyond the track. They are
not for the layout — they are for the cone-zoo capture session. The track carries
~36 boundary cones against 4 red, 2 orange and 1 magenta, and that imbalance is
the single biggest threat to the detector on exactly the three classes that
trigger state transitions. Extra cones let you stage them at many ranges at once.

Red and orange are also the closest colors on the track, so shoot them *together*
in the same frame at several ranges. A detector that separates them at 2 m and
merges them at 8 m is the failure that matters, and it only shows up if both are
in shot.

### Dead-end end walls

Each dead end is walled across its 1.5 m width. The two corner positions are
already the last cones of the side walls, so each dead end needs exactly **one
additional cone in the middle** — otherwise the 1.5 m gap reads as corridor and
the car will try to drive through it. That middle cone is **orange**: the dead
end is a class of its own, so a stub is recognisable as a stub rather than
inferred from a gap that failed to open.

Do **not** use red for a dead-end wall — it would fire a false gate, and red is
the colour orange is most likely to be confused with in the first place.

## Cone spacing is not a comfort setting

**Corrected 2026-08-30, from `sim/drive_sim.py`.** This document originally
specified 1.5 m spacing on the straights, reasoning that "straights tolerate
sparse cones". They do not, and the reason has nothing to do with the straights.

The two sensors overlap over a much narrower band than either one's range
suggests:

- The camera cannot see a boundary cone until it is **1.18 m** ahead. A cone
  0.75 m off the corridor axis sits at 32.5 deg or wider before that, outside
  the usable frame (`0.75 / tan(32.5°)`).
- The lidar stops resolving a cone past about **3.0 m**, where it returns fewer
  than two points and one point is indistinguishable from noise
  (`cone_perception/clustering.py`).

So a cone is only *both* locatable and identifiable between 1.18 m and 3.0 m
ahead — a window under 2 m deep. Spacing decides how many cone rows fall inside
it, and the corridor layer needs at least two to form a chain it can steer
along:

| Spacing | Centerline points | Car drives? |
|---|---|---|
| 1.50 m | 4.6 | **no** |
| 1.25 m | 5.0 | yes, by one row |
| 1.00 m | 6.8 | yes |
| 0.75 m | 10.2 | yes |
| 0.50 m | 14.2 | yes |

Re-measured 2026-08-30, after the cone's lidar cross-section was measured rather
than estimated — 7.4 cm, not 6.5 cm (`sim/cone_field.py`). That bought roughly
half a metre of lidar range and moved the cutoff one row, from between 1.0 and
1.25 m to between 1.25 and 1.5 m. **The recommendation does not change: 0.75 m.**
A spacing that passes by a single cone row is not one to build a track on.

At 1.5 m the car does not move, and nothing about that failure points at
spacing: the detector is working, fusion is working, and the centerline simply
comes back too short to steer along.

`cone_nav/corridor/side_assign.py` recovers the near blind spot by giving
unlabelled clusters a side from geometry, and roughly quadruples the usable
midpoints at every spacing — but it cannot see further than the lidar either, so
it does not rescue a sparse track. **Lay the corridor at 1.0 m or tighter.**

This raises the cone budget for a full track build. The 0.75 m figure below for
fork mouths was already right; it is now the number everywhere.

## Cone colors at a fork

Each branch keeps blue-left / yellow-right **in its own direction of travel**.
That resolves the island between the branches automatically: its left face (the
left branch's right wall) is yellow, its right face (the right branch's left
wall) is blue. Build the island nose as one yellow and one blue cone side by
side, then taper outward. No extra class, no ambiguity.

## Cone height

Use **one cone size for the entire track** and record it here and in
`cone_perception/extrinsics.py` as `CONE_HEIGHT_M`. The `range_bbox` channel of
`LabeledCone.msg` is `Z = f * h_real / h_pixels`; a mixed-height track silently
corrupts that estimate for every cone that differs.

- Cone height (base to tip): **0.1778 m** (7 in) — measured 2026-08-28
- Cone base width: ____ m

Recorded in `cone_perception/extrinsics.py` as `CONE_HEIGHT_M`.

The base width is still blank and that is a smaller gap than it looks: it feeds
nothing. `range_bbox` uses the height, and the lidar's view of a cone is gated
loosely on purpose (`cone_perception/clustering.py`) because the camera is what
confirms a cluster is a cone. Measure it when convenient — it would let the
size gate tighten — but nothing is blocked on it.

## Survey procedure

Per `data/README.md`, fix the origin and axes **before** the first measurement.
Measure to **cone base centers**. One shared sheet.

Record into `track_v1.csv` with columns:

```
id,color,x_m,y_m,segment
```

where `color` is one of `blue`, `yellow`, `red`, `orange`, `magenta` and matches
the class names used in Roboflow. The CSV holds **measured** positions, not the
design numbers above — lay the track out from this document with a tape, then record
where the cones actually ended up. Photograph the finished track from a fixed
vantage and note where that vantage was, so a rebuild can be checked against the
photo.

This CSV is the D5 deliverable and the ground truth `analysis/` uses for
cross-track error.

## The fork is a region, not a point

Two branches diverging by ±25° from a 1.5 m corridor do not separate at the
junction. Their inner walls only cross at

```
nose = half_width / sin(divergence) = 0.75 / sin(25°) = 1.78 m
```

past it. Before that point the two corridors overlap and there is no island to
put cones on — laying inner-wall cones from the junction outward builds two
walls that intersect. The island nose belongs at 1.78 m, which is where "build
the island nose as one yellow and one blue cone side by side" actually happens.

**This makes the 1.5 m dead-end stub above impossible as specified.** A 1.5 m
stub ends before the branches have separated, so it never becomes a corridor of
its own — the car cannot be shown a stub it can commit to, because the stub and
the through-branch are still the same widened space. Three ways out, in order of
how little else they disturb:

| Fix | Effect |
|---|---|
| **Stub 2.8 m** | Gives a metre of genuine walled corridor past the nose. Costs ~1.3 m of footprint per dead end and 2 more cones each |
| **Divergence 35°** | Nose moves to 1.31 m, so a 1.5 m stub just works. Branches sit at ±35°, still inside the ~69° HFOV but with much less margin |
| **Narrower corridor at the fork** | 1.2 m corridor puts the nose at 1.42 m. Contradicts "1.5 m, uniform" |

`sim/cone_field.py` builds the 2.8 m version by default and
`island_nose_distance()` is where the arithmetic lives; pass
`track_v1(dead_end_length_m=1.5)` to generate the track exactly as written above
and watch the boundary go ambiguous.

## Superseded at the junctions: see `junction_v2.md`

The two Y-junctions above are laid as **single-origin forks** — both branches
leaving one point, with a red PAIR as a landmark 1 m before it. The car does not
drive that shape well, and the reason is in the arithmetic this document already
contains. Two branches leaving one point have inner walls on the wrong sides of
each other until `half_width / tan(divergence)` past the junction, so through
that whole stretch the routed branch has an outer wall but no blue/yellow
*pair*, hence no corridor midpoint — and `speed_ctrl` stops the car when the
driven line reaches under 1 m ahead.

`data/layouts/junction_v2.md` replaces it with a **staggered fork**: three red
cones instead of two, and each branch starting at its own gate midpoint rather
than at the junction centre. The centre red cone becomes the island nose, so the
divider is physical and at the junction rather than inferred 1.78 m downstream.
The first exit pair moves from 2.06 m past the red line to 0.75 m, and the
measured minimum reach through the mouth is 1.82 m against the 1.0 m floor.

Build junctions from `junction_v2.md`. The corridor segments, colours, cone
height, spacing and survey procedure in this document are unchanged and still
apply — **except** for the advice to densify through a fork, which is wrong at a
v2 junction and is corrected there.

## Known consequence for the nav stack

A fork buys continuous visibility at the cost of *ambiguity*. Approaching a Y,
the camera sees two blue walls and two yellow walls at once, so a naive "all blue
cones are my left boundary" rule in `cone_nav/corridor/boundary_split.py` will
fuse two corridors into nonsense.

That is what the red gate pair is for. `EVENT_GATE_IN_RANGE` is the trigger to
hand off from centerline-following to `cone_nav/guidance/junction_exec.py`, which
uses the provided route to pick which branch's cones to keep. Write the corridor
layer knowing it will sometimes see two corridors, not one.

**Update.** That is what junction v2 does, with one change: the trigger is not
`EVENT_GATE_IN_RANGE` at a red pair but a whole red *triple*, and the handoff is
not to a separate manoeuvre. `cone_nav/guidance/junction_exec.py` filters the
cone list to the routed branch and hands it to the same `centerline` the
corridor uses, so there is no second control stack to hand off to. See
`junction_v2.md`.
