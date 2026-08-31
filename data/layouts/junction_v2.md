# Junction v2 — the staggered fork

The junction the car is built to drive, and the sheet to lay it from. Three
**red** cones across the mouth; the left and centre bracket the left branch's
gate, the centre and right bracket the right branch's. One branch continues as
a corridor, the other is a stub walled with an **orange** cone.

Frame: **origin at the centre red cone, x forward along the incoming corridor,
y left**, metres — the same convention as `cone_msgs/msg/LabeledCone.msg`.

Everything below is generated from `sim/cone_field.track_junction()`:

```
python -m sim.cone_field --layout junction-left --diagram --table --window
```

`sim/test_drive_junction.py` asserts the constraints this document states, so a
geometry change that breaks one fails at the desk rather than on the floor.

## Parameters

| Element | Value | Why |
|---|---|---|
| Red cones | 3, collinear across the mouth at y = +1.35, 0, −1.35 | One tape line to lay and to survey |
| Gate gaps | **1.35 m** each; span 2.70 m | See *Gate width* below — a compromise between two limits, not the corridor width |
| Branch origin | left (0, +0.675), right (0, −0.675) — the gate midpoints | The stagger. See *Why each branch gets its own origin* |
| Divergence | **±20°** | Comfortably inside the ~69° HFOV; the stagger already does the separating |
| First exit row | 0.75 m along the branch, **both walls** | A full blue/yellow pair, so a corridor midpoint exists from row one |
| Last incoming row | 0.75 m before the red line | **No boundary cone on the red line** — one would sit in the mouth the car drives through |
| Spacing | **0.75 m**, and no denser | See *Do not densify at the junction* — this is the opposite of `track_v1.md`'s advice |
| Stub | 2.5 m, orange cone across the end | Long enough that the car commits before the wall is obvious |
| Corridor width | 1.5 m throughout | Unchanged; the gate is deliberately narrower than the corridor |

## Why each branch gets its own origin

`track_v1.md` builds a Y where both branches leave one point. Their inner walls
are then on the wrong sides of each other until

```
half_width / tan(divergence) = 0.75 / tan(20°) = 2.06 m
```

past the junction, so for that whole stretch the routed branch has an outer wall
but no *pair* — and no corridor midpoint. That matters because `speed_ctrl`
stops the car when the driven line reaches less than `MIN_REACH_M = 1.0 m`
ahead, measured from the rear axle at x = −0.362, i.e. when the line's far end
falls below **x = 0.64 m**. A car that stops in a junction mouth does not
restart: the scan does not change while it stands still.

Two origins a gate-gap apart start already separated, so both walls exist from
the first row and the island-nose arithmetic does not arise. Measured at the
tightest moment — gate midpoint at x = 0.64 m:

| | single-origin Y | staggered fork |
|---|---|---|
| First exit **pair** midpoint | 2.06 m past the red line | **0.75 m** |
| Both its cones inside the 1.18–3.0 m sensor overlap? | no | **yes** (1.63 m and 2.03 m) |
| Measured minimum reach through the mouth | — | **1.82 m**, against a 1.0 m floor |

## Gate width

The two limits that bound junction detection are both set by the outer reds'
offset from the axis, and they move in **opposite** directions as the gate
widens. Distances are to the junction line.

      gap   span   in frame   in range   window   ticks at 1.2 m/s
      1.25  2.50   1.96 m     2.73 m     0.77 m   6.4
      1.30  2.60   2.04 m     2.70 m     0.66 m   5.5
      1.35  2.70   2.12 m     2.68 m     0.56 m   4.7
      1.40  2.80   2.20 m     2.65 m     0.46 m   3.8
      1.45  2.90   2.28 m     2.63 m     0.35 m   2.9
      1.50  3.00   2.35 m     2.60 m     0.24 m   2.0
    

Those are the geometric bounds. Driving the sim past a junction and counting
the ticks on which a whole triple is actually recovered gives slightly less,
because the LD06 stops resolving these cones a little inside 3 m:

| gap | predicted window | **measured** | detectable from → to |
|---|---|---|---|
| 1.30 m | 0.66 m | **0.60 m**, 5.0 ticks | 2.64 → 2.04 m |
| 1.35 m | 0.56 m | **0.50 m**, 4.2 ticks | 2.60 → 2.10 m |
| 1.40 m | 0.46 m | **0.40 m**, 3.3 ticks | 2.60 → 2.20 m |
| 1.50 m | 0.24 m | **0.24 m**, 2.0 ticks | 2.58 → 2.34 m |

The span (2 × gap) must also clear `centerline.MAX_PAIR_EDGE_M = 2.5 m`, or the
triangulation pairs the two outer reds and drops a phantom gate midpoint on the
centre cone — inviting the car to drive at the divider. That rules out 1.25 m
and below.

**1.35 m** keeps 0.20 m of margin over the pairing limit and still leaves about
five ticks to see the junction in. 1.50 m — the tidy choice, one corridor width
per gate — leaves two, and in the sim it was the width at which runs started
failing.

## Do not densify at the junction

`track_v1.md` recommends tightening cone spacing through a fork. **Do not do
that here.** Two cones within `clustering.GAP_DEG` (3°) of each other in bearing
merge into one lidar cluster, and a merged outer red is a red that
`gate_detect` never sees.

| Spacing | Nearest boundary cone to an outer red | Whole triples recovered on the approach |
|---|---|---|
| 0.50 m | 0.29 m | **none, on any tick** — the car sails past the fork |
| 0.75 m | 0.53 m | four consecutive ticks |
| 1.00 m | 0.78 m | patchy, but enough |

0.75 m is the spacing the corridor layer needs and the loosest crowding the
junction layer tolerates. It is the only setting that satisfies both.

## The detection window, and what it forces

A whole triple is recoverable only between about 2.60 m and 2.10 m — four ticks
at 1.2 m/s — and below that the outer reds leave the camera frame, come
back UNLABELED, and the junction **disappears while the car is still two metres
short of the mouth**. Three consequences, all of them in
`cone_nav/topology/topo_state.py`:

- The car commits on the **first** whole triple it sees. Sightings are the
  scarce resource here, not confidence; on some layouts the entire approach
  yields one. What guards a commit is `gate_detect` requiring exactly three reds
  with both gaps in range, which a misread orange at a dead-end wall cannot
  satisfy alone.
- It then drives the mouth **blind**, on a latched divider carried forward with
  its own motion. Frozen instead of carried, that latch is metres stale by the
  far side.
- Leaving the manoeuvre is a travelled **distance**, not a clean-looking
  corridor. The corridor the car is still in looks perfectly healthy all the way
  down the approach.

## Scale plan view

Generated. `b`/`y`/`R`/`o`/`M` are blue, yellow, red, orange and the magenta
goal; the routed branch is up and to the right.

```
                                                                        
                                                             b          
                                                       b                
                                                b                       
                                          b                  M          
                                    b                                   
                               R                                  y     
                                                            y           
    b      b      b      b                           y                  
                                               y                        
                                        y                               
                               R                                        
                                        b                               
                                               b                        
    y      y      y      y                           b                  
                                                                        
                               R                     o                  
                                    y                                   
                                          y                             
                                                y                       
                                                                        
                                                                        

```

## Build table

Positions of **cone base centres**, in the junction frame. Offset by the
junction's position in the track frame and paste into
`data/layouts/track_v1.csv`, whose columns these already are. Mirror `y` for a
right-routed junction.

```
id  color    x_m     y_m     segment
1   blue     -3.000  0.750   corridor_a
2   yellow   -3.000  -0.750  corridor_a
3   blue     -2.250  0.750   corridor_a
4   yellow   -2.250  -0.750  corridor_a
5   blue     -1.500  0.750   corridor_a
6   yellow   -1.500  -0.750  corridor_a
7   blue     -0.750  0.750   corridor_a
8   yellow   -0.750  -0.750  corridor_a
9   red      0.000   1.350   junction_1
10  red      0.000   0.000   junction_1
11  red      0.000   -1.350  junction_1
12  blue     0.448   1.636   corridor_b
13  yellow   0.961   0.227   corridor_b
14  blue     1.153   1.893   corridor_b
15  yellow   1.666   0.483   corridor_b
16  blue     1.858   2.149   corridor_b
17  yellow   2.371   0.740   corridor_b
18  blue     2.563   2.406   corridor_b
19  yellow   3.076   0.996   corridor_b
20  blue     3.267   2.662   corridor_b
21  yellow   3.780   1.253   corridor_b
22  blue     0.961   -0.227  dead_end_a
23  yellow   0.448   -1.636  dead_end_a
24  blue     1.666   -0.483  dead_end_a
25  yellow   1.153   -1.893  dead_end_a
26  blue     2.371   -0.740  dead_end_a
27  yellow   1.858   -2.149  dead_end_a
28  orange   2.349   -1.530  dead_end_a
29  magenta  3.289   1.872   goal
```

## Survey

Per `data/README.md`, fix the origin and axes **before** the first measurement.
The origin here is the centre red cone, so place that one first and measure
everything from it. Measure to cone base centres, and record where the cones
actually ended up rather than these design numbers — the CSV holds measured
positions.

Two tolerances are tighter than the rest and are worth a second look with the
tape:

- **The gate gaps**, because the span has only 0.20 m of margin over
  `MAX_PAIR_EDGE_M`. If both gaps come out 1.45 m the span is 2.90 m and still
  fine; if they come out 1.25 m the span is 2.50 m and a phantom gate midpoint
  lands on the divider.
- **The 0.75 m clear band** either side of the red line, because a boundary cone
  crowding a red is what makes the junction undetectable.
