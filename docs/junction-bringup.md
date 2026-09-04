# Junction bring-up

Taking `drive_junction.py` from a green sim to a car that turns where it is
told. Read `data/layouts/junction_v2.md` first — this document assumes the
junction is built to it.

## The risk here is the layout, not the controller

Worth being blunt about, because it decides the order of everything below. The
sim drives both junction directions with 3.4 cm mean cross-track, 12° peak steer
against a 20° limit, and a minimum reach of 1.82 m against the 1.0 m floor. The
control margins are not close.

What the sim cannot tell you is whether **the junction you actually laid can be
seen**. Every failure it produced came from the layout, not the loop:

- gaps at 1.50 m instead of 1.35 m → 2 ticks of visibility instead of 4
- cones spaced 0.5 m near the reds → the triple detected on **zero** ticks
- branches leaving the centre cone instead of their own gate midpoints → the
  car stops in the mouth

And the sim's detector is perfect. The real one confused 6% of oranges for red
in the v1 report, which is the one class error that matters here.

So the bring-up is front-loaded: stages 1–3 prove the track is visible before
anything is allowed to move. Do not skip to stage 5 because the sim is green.

## Stage 0 — at the desk

```sh
uv run --with pytest --with numpy python -m pytest -q      # 712 pass
uv run --with pytest --with numpy python -m pytest sim -q  # 76 pass, 3 known fails

PYTHONPATH=ros2/src/cone_perception:ros2/src/cone_nav:model/capture:. \
  uv run --with numpy python -m sim.drive_sim \
    --track junction-left --route data/routes/junction_left.txt
```

Three failures in `sim/test_drive_sim.py` (`test_base_link_leads_the_axle`,
`test_base_link_rotates_with_the_car`, `test_it_stops_when_the_corridor_runs_out`)
predate this work and fail on a clean checkout. Everything else must be green.

Decide the route before you leave. For a single junction use a one-line file;
`data/routes/junction_left.txt` and `junction_right.txt` are already that.

## Stage 1 — build the junction

From the build table in `junction_v2.md`. Place the **centre red cone first** —
it is the frame origin and the island nose — and measure everything from it.

Two tolerances are tighter than the rest:

| Check | Target | Why it bites |
|---|---|---|
| Both gate gaps | 1.35 m ±0.05 | Span must stay above `MAX_PAIR_EDGE_M` = 2.5 m, and there is only 0.20 m of margin. Too wide costs visibility fast |
| Clear band either side of the red line | 0.75 m, nothing closer than 0.4 m to any red | A boundary cone crowding a red merges with it in the lidar and the junction stops existing |

Everything else — 0.75 m spacing, 1.5 m corridor, ±20° branches — has room.

Photograph the finished junction from a fixed vantage and note where you stood,
so a rebuild can be checked against it.

## Stage 2 — deploy

```sh
./model/capture/deploy.sh                 # or: ./deploy.sh <ssh-host>
```

Pushes the tool, both pure packages, and `data/routes/` → `~/cone_capture_tool/routes/`.
The route file is not optional and is not in the tool directory otherwise.

On the car, before anything opens a device:

```sh
fuser -v /dev/ttyACM0 /dev/ttyUSB0     # must be empty
```

DonkeyCar holds `/dev/ttyACM0`. Stop it. Only one process may hold each device.

## Stage 3 — prove the junction is visible

**This is the stage that matters, and it needs no motion.** Carry or push the
car slowly from about 4 m out, straight down the incoming corridor, through the
mouth.

```sh
cd ~/cone_capture_tool
python drive_junction.py --weights ~/models/best.pt \
    --route routes/junction_left.txt \
    --dry-run --no-deadman --log junction-see.jsonl
```

### Where the car has to be standing

Detection needs no motion at all -- `gate_detect.survey` is a pure function of
one revolution, so a car parked in the right place reports the gate on every
tick. But "the right place" is a band only half a metre deep, and both of its
edges are set by the outer reds sitting 1.35 m off the axis:

| Limit | Distance to the red line | Set by |
|---|---|---|
| Too far | **2.68 m** | `GATE_ARM_RANGE_M` is a SLANT range, and the outer reds reach 3.0 m here |
| Too close | **2.12 m** | The outer reds leave the camera frame and come back UNLABELED |

Park at 3 m and you get **nothing**, with `reds_seen: 1` in the log -- the
centre cone alone -- while all three sit in plain view. Stand the car at about
2.4 m and confirm a gate before you push anything: a static reading is the
cleanest measurement of the track you will get, and it separates a layout fault
from a timing one.

The once-a-second line says which case you are in without opening the log:

```
  idle   duty 0.000  steer   +0.0 deg  4 pts, reach 2.10 m  follow \
         reds 1/3 @ 2.75/3.06/3.06 m  gaps 1.35/1.35  [reds in view, not all three in range]
```

Three reds at 3.06 m with correct gaps is a car standing too far back, not a
mis-laid gate.

`--dry-run` never opens the VESC. Watch the state line — it prints on every
transition — and expect exactly one `follow → approach → traverse → follow`.

Then read the log rather than trusting the impression. The car has no `jq`,
and a check you retype at the bench from a document is a check that gets
skipped, so it is a script:

```sh
~/env/bin/python junction_report.py junction-see.jsonl
```

It prints the table below with the measured value beside each expectation.
`OK`/`CHECK` is a reading aid, not a gate — a run it calls `CHECK` may still be
fine, and one it calls `OK` can still have driven badly.

| What to check | Expect | If not |
|---|---|---|
| Ticks with `gate_live` true | **≥ 4** (the sim gets 4.2 at 1.2 m/s; walking pace gives more) | Read `gate_reason` first — it names the cause. Only then measure the gaps and the clear band |
| `reds_in_view` vs `reds_seen` | equal | `reds_in_view` higher means the car never got inside 2.68 m of the red line |
| `gate_gaps_m` | ≈ `1.35/1.35`, both within 0.05 | This is the car measuring your tape work. Believe it over the tape |
| `gate_range_m` when first live | ≈ 2.6 m, falling to ≈ 2.1 m | A much shorter span means one red is being missed |
| `topo_state` sequence | one clean pass | Two `traverse` blocks means it re-armed on the same junction |
| `branch_cones_dropped` during `traverse` | **> 0** | The filter never bit; the car would be choosing its own branch |
| `topo_note` | `passed` once | `traverse timed out` means it never saw a corridor on the far side — or that `--push-speed` is far below the pace you actually walked |

`--push-speed` is what makes the manoeuvre half of this stage testable. A dry
run pins the commanded duty to zero, and the travel estimate normally comes from
that duty, so without it `travelled_m` stays at zero, TRAVERSE never clears its
distance floor, and the run can only ever end by timing out 20 s later with the
branch filter cutting on a divider frozen where it was first seen. It defaults
to 0.5 m/s; pass the pace you actually walk. It is an assumption about you, not
a measurement of the car, and it is ignored outside `--dry-run`.

If `gate_live` is never true, stop and fix the track. Nothing downstream can
recover from a junction the car cannot see, and the failure mode is that it
drives past and takes whichever branch is longer.

Run it once per direction you intend to drive.

## Stage 4 — steering sign, on a stand

Wheels off the ground.

```sh
python drive_junction.py --weights ~/models/best.pt \
    --route routes/junction_left.txt --steer-only
```

Throttle is pinned to zero; the servo moves. The corridor check is that the
wheels follow a cone walked across the front. **The junction check is different
and is the point of running this again:** with the junction in view and the
route saying `left`, the wheels must turn **left**. A mirrored sign tracks a
straight corridor perfectly and turns the wrong way at the first fork.

Wrong way → add `--invert-steering`, and decide it here, never on the track.

## Stage 5 — first driven run

```sh
python drive_junction.py --weights ~/models/best.pt \
    --route routes/junction_left.txt \
    --max-duty 0.10 --log junction-run-1.jsonl
```

Hold **X** on the F710 to arm. Release and it stops. Start at the duty floor;
raise it only after a clean pass, and drop `--smooth-window` as you do — the
median filter's lag costs more the faster the car moves.

Walk beside the car with a hand near the pad. Release X for any of:

- the car aims at the centre red cone rather than between two cones
- it stops in the mouth (see the diagnosis table — this is the failure the
  geometry is built to prevent, so it means something is off the design)
- the state line shows a second `traverse` on the same junction
- anything you did not expect

Stop after one junction and read the log before running it again.

## When it goes wrong

Keyed on the log, in the order worth checking.

| Symptom | Field to read | Likely cause |
|---|---|---|
| Drives straight past the fork | `gate_live` all false | Layout. Back to stage 1 — gaps too wide, or cones crowding the reds |
| Takes the wrong branch | `turn`, `route_index` | Route file order, or a previous junction consumed an entry it should not have |
| Stops in the mouth | `stop_reason`, `reach_m` | `reach_m` under 1.0 m. Exit corridor starts too far past the red line, or its first row is only one wall |
| Turns too late, clips the divider | `gate_range_m` at commit | Committed near 2.1 m instead of 2.6 m — only the last of the window was seen |
| Manoeuvre never ends | `topo_note` = `traverse timed out`, `travelled_m` | The far-side corridor is not pairing, or `DUTY_TO_MPS` is badly off for this car |
| Ends the turn early, then wanders | `travelled_m` at the `passed` note | `DUTY_TO_MPS` too high. It is a guess; see below |
| Phantom wall across the mouth | `labeled_by_geometry` spiking during `traverse` | Lower `--fill-range-at-junction` below 1.0 |

### The one number likely to need fitting

`speed_ctrl.DUTY_TO_MPS = 12.0` was fitted to nothing. It converts commanded
duty into the travelled distance `topo_state` uses as the floor for deciding a
gate is behind the car. Too high and manoeuvres end early; too low and they
overrun into the timeout.

Fit it from the first driven run: take `travelled_m` at the `passed` note and
compare against the distance actually covered from commit to clear. Scale the
constant by the ratio. The proper fix is the VESC encoder — `VESC_HAS_SENSOR`
is already true and nothing reads it.

## Stage 6 — the goal

The goal is a magenta 3D-printed trophy, 6.88 in tall, standing where the last
corridor ends. `drive_junction.py` reads it, drives at it, and stops
`--goal-stop` metres short — 0.30 m by default, measured from the lidar, which
`hardware-baseline.md` puts at the front edge of the chassis. So the number is
nose-to-trophy.

Nothing here arms until the route is spent. The goal lies past the last junction
by construction, so a magenta seen while a turn is still outstanding is a
misread — and magenta/red is the detector's hardest pair, so that is a real
possibility rather than a theoretical one. `--goal-anywhere` overrides it for
bring-up on a corridor with no junction in it, and says so loudly at startup.

### 6a — can the lidar see the trophy at all?

**Do this first; everything else is void without it.** The scan plane sits at
0.127 m, which is 71% of the way up a 7 in cone and well into the taper — and a
trophy is not a cone in cross-section. If it has a stem or a waist at that
height it may present ~2 cm, which subtends 0.57° at 2 m: under one return,
where `clustering.py` needs two.

Stand the trophy at 1, 2 and 3 m and read points-per-cluster out of
`lidar_view.py`. **Two or more returns at 3 m** and the design holds. Fewer and
stop: the goal would need a camera-only channel (bearing plus `range_bbox`),
which is a different design and not what is built.

### 6b — does the detector find it?

`detect_view.py` with the trophy at 1–3 m. Watch for magenta boxes at all, and
specifically for **magenta read as red** — v1 and v2 both did that on 69% of
instances, and a trophy read as red is a cone that can complete a junction
triple where there is no junction.

### 6c — dry run

    python drive_junction.py --weights ~/models/best.pt --route <route> \
        --goal-anywhere --dry-run --no-deadman --log goal-dry.jsonl

Push the car at the trophy down a short corridor. Travel is measured by scan
matching in a dry run, so the carry through a camera dropout is exercised
honestly at walking pace. Read `goal_reason` first when nothing happens — it
names the fix:

| `goal_reason` | What it means | Where to look |
|---|---|---|
| `no magenta` | The detector is not finding it | 6b, and the weights |
| `magenta in view, beyond arm range` | It IS found; the car is too far back | Walk closer; the arm range is 3.0 m, slant |
| `magenta off the corridor axis` | Found, but not where a goal can be | The tape. `goal_offset_m` says by how much |
| `more than one magenta in range` | Two candidates; it declines to guess | Clear the spare, or find the false positive |

Then watch `goal_state` walk `seeking → run_in → stopped`, `goal_range_m` close
monotonically, and `goal_blind_ticks` stay at or near zero. A run that arrives
with `goal_blind_ticks` high finished on dead reckoning; the summary line says
so, and it means the camera lost the trophy in the last metre.

### 6d — driven

Same command without `--dry-run --no-deadman`, at `--max-duty 0.05`. Confirm:

- the latch fires at `--goal-stop`, and measure where the car actually ends up;
- the trial log's `stop_reason` is empty with `goal_state = stopped`, **not**
  `corridor visible only ... m ahead` — that reason means the reach floor got
  there first and the goal did nothing;
- releasing X and pressing it again drives on, so the trophy can be reset
  without restarting the tool.

### 6e — what it did, 2026-09-01

Full course under power at `--max-duty 0.05`, tool at `f0a275c`
(`data/trials/goal-run-1551.jsonl`, 586 ticks, 359 armed, 10.0 Hz): LEFT at J1,
RIGHT at J2, route fully consumed, then the goal. Run-in opened at **0.99 m** and
the car stopped **0.27 m** from the trophy, `stop_reason` `goal reached`.

The numbers to compare a later run against:

| | value |
|---|---|
| run-in opened | 0.99 m (`RUN_IN_M` = 1.0) |
| stopped at | 0.27 m (`--goal-stop` 0.30, one tick of travel inside it) |
| `goal_hops` | 0 |
| `goal_blind_ticks` through the run-in | 0 |
| range backsteps during run-in | 0 |
| duty through the run-in | 0.050 held, on a 1-point line |

Two earlier dry runs are kept beside it because they are what the design was
corrected against, and both failure signatures are worth recognising again:
`goal-dry.jsonl` shows the stale-axis refusals (`magenta off the corridor axis`
for 26 ticks of a clean approach) and `goal-dry2.jsonl` shows the label
alternating between the trophy and an object 1.17 m behind it, fifteen times.
`goal-dry3.jsonl` is the same course after both fixes.

The default 0.30 m assumes the near-zero coast measured on this car. If the car
overshoots, raise it; do not lower it below 0.20 m, where
`clustering.MIN_CONE_RANGE_M` discards the trophy's return as a chassis leak and
the car would be stopping on a goal it can no longer see.

## Stage 7 — exploring, with you as the reverse

Everything above drives a route someone wrote. `--explore` takes the route away:
the car picks a branch at each junction, and when the branch ends in a wall it
backs the search out and takes the other one. Here **you are the reverse** —
the car stops at the wall and you carry it back. That is not a workaround
bolted on; the decision layer is finished either way, and the map, the plan and
the emitted route are identical to what a self-reversing car produces.

Stage 8 takes the carry out. This stage stays the fallback, and stays the way
to bring up a track before trusting a reverse on it: `--reverse` is off unless
you pass it.

```sh
~/env/bin/python drive_junction.py --weights ~/models/best.pt --explore \
    --invert-steering --max-range 3.5 --lookahead 0.8 \
    --max-duty 0.05 --emit-route routes/optimal.txt --log explore-1.jsonl
```

`--route` and `--explore` are mutually exclusive and neither is a default.

**`--invert-steering --max-range 3.5 --lookahead 0.8` are not optional on this
car** — see *Drive-by-wire* in `hardware-baseline.md`. Omit the first and the car
turns the wrong way at every bend while tracking a straight corridor perfectly;
a run on 2026-09-02 accumulated −167 deg of unwanted right turn that way. Use
`~/env/bin/python`, not `python`: `/usr/bin/python` has no depthai or torch, and
`--help` succeeds either way.

**Deploy first.** All of this is new code, and the car gets it by rsync rather
than by clone, so stage 2's `./model/capture/deploy.sh` is not optional here. The
usual rule applies: commit everything, *then* deploy, *then* run — otherwise the
commit stamped into `VERSION` is unreachable and the run's provenance breaks.

### What is different, and what it costs

**The goal is armed from the first tick.** On a route the goal lies past the last
junction by construction, so a magenta seen with turns outstanding is a misread.
A maze puts the goal wherever it likes, so that rule cannot apply — and magenta
read as red is **15% of instances on v3**, the detector's hardest pair. A misread
mid-course ends the run. The tool says so loudly at startup.

_For the first runs, keep the trophy off the course until the last corridor._
Prove the search works before asking it to survive the goal detector.

**`route_remaining` means something else.** Not entries left to read but branches
**found and not yet tried**, so it GROWS as the car discovers the maze. The log
records which cursor drove the run and `junction_report.py` reads it; a report
that graded an exploring run against a route length would call every backtrack a
fault.

### 7a — can the car see the wall?

**The stage-3 pattern, and for the same reason: no motion needed.** Carry or push
the car down the walled stub.

```sh
python drive_junction.py --weights ~/models/best.pt --explore \
    --dry-run --no-deadman --log explore-see.jsonl

~/env/bin/python junction_report.py explore-see.jsonl
```

The dead-end signal is **geometric first**: the corridor's reach collapsing while
cones are still in view. Orange only shortens the confirmation from twelve ticks
to five, because orange has **recall 0.687** on v3 and **15% of oranges are called
red** — a wall read as a gate is the worst confusion on this track, so it cannot
be the signal.

Read `dead_end_reason` before anything else. The report ranks it over the ticks
that did *not* latch, which is the whole diagnosis:

| `dead_end_reason` | What it means | Where to look |
|---|---|---|
| `corridor reaches X m` | The corridor was genuinely open. Not a fault if the car had not arrived yet | Push further in |
| `only N cones in view` | A blind car, not a wall — the line collapsed because perception did | The camera, the lidar, the light |
| `single-boundary fallback` | The car is confused about one wall | Cone spacing; one side is dropping out |
| `not armed` | Held down deliberately — inside a junction mouth or the goal run-in | Correct; the mouth is allowed to look like a dead end |
| `confirming N/M` | It is working. M is 5 with an orange seen, 12 without | Nothing |

This run also writes the **first log with `pose_x` in it**, which is what
`analysis/map_from_log.py` needs. Nothing on disk before today can be mapped.

### 7b — driven, with recovery

```sh
python drive_junction.py --weights ~/models/best.pt --explore \
    --max-duty 0.05 --emit-route routes/optimal.txt --log explore-1.jsonl
```

At each wall the car prints `[DEAD END]` and stops. Then:

1. **release X.**
2. carry the car back to the junction it came through, facing the way it
   originally approached — about 2.4 m short of the red line, the same place
   stage 3 has you stand.
3. **press X.** The console names the branch it is about to take. It has already
   chosen; the release does not decide anything.

Expect `pose frame broken by the lift` on that line. It means what it says: the
pose cannot see a carry, so edges measured across one are recorded **unmeasured**
rather than wrong. `maze_*` in the log counts them.

### 7c — the plan, and driving it

On exit the tool writes the route from the start to the goal implied by what it
explored — the driven path with its dead ends removed — and prints how many gates
it drove against how many the route holds.

That file is run data: the car worked it out, and nothing in the repo can
regenerate it. `deploy.sh` excludes `routes/` from its `--delete` for exactly
that reason — but **pull it off the car before you redeploy anyway**, along with
the logs. Then carry the car to the start:

```sh
python drive_junction.py --weights ~/models/best.pt \
    --route routes/optimal.txt --log optimal-1.jsonl
```

No new driving code is involved; this is the same tool reading the same route
format a human would have written.

**That run is also the one to build the map from** — a clean single pass with no
lifts in it. But the map is built **off the car**: `deploy.sh` sends
`model/capture/`, the two pure packages and `data/routes/`, and nothing else, so
neither `analysis/` nor `data/layouts/` is over there. Pull the log back first.

```sh
# at the desk, not on the car
scp robocar:cone_capture_tool/optimal-1.jsonl data/trials/
python analysis/map_from_log.py data/trials/optimal-1.jsonl --layout data/layouts/track_v1.csv
```

### 7d — what it did, 2026-09-02

First full exploring run, in failing evening light. Tool at `a52cf51`,
`data/trials/explore-run-1854.jsonl`, 868 ticks / 86.6 s / 10.0 Hz, one junction
with a walled RIGHT branch and the trophy past the LEFT one.

```
 10.0s  follow -> approach -> traverse   turn RIGHT, gate 2.08 m
 27.3s  traverse -> follow               (passed)     now at `right`
 30.3s  DEAD END                         corridor ends 0.85 m ahead (orange wall seen)
 31.8s  released, carried back           re-arming in 0.49 m
 47.5s  follow -> approach -> traverse   turn LEFT,  gate 1.57 m
 53.5s  traverse -> follow               (passed)     now at `left`
 66.5s  goal seeking -> run_in           0.99 m
 68.3s  goal run_in -> stopped           0.21 m
```

It emitted `data/routes/optimal_explore_1854.txt`: **`left`** — two gates driven
while exploring, one on the route, one detour avoided.

| | value |
|---|---|
| whole triples recovered | 68 ticks |
| measured gate gaps | 0.71 / 0.72 m (laid 0.72) |
| dead end named at | 0.85 m, orange corroborating |
| goal run-in opened | 0.99 m (`RUN_IN_M` = 1.0) |
| stopped at | 0.21 m (`--goal-stop` 0.30, one tick of travel inside) |
| ticks with measured odometry | 832 / 868 |
| branch filter peak drop | 13 cones |

**Read the map, not the drive.** The car physically passed a second junction on
its way to the trophy, and the log records no third manoeuvre — it followed the
corridor through rather than detecting a gate there. So the emitted route is
correct for what was MAPPED (3 nodes, 2 edges, 1 dead end) and is one turn
short of describing the course. Re-driving it reproduces the run only because
the second junction is passed by corridor-following either way. A route file is
a claim about the map, and the map is only as complete as the gates that armed.

`min reach through the mouth 0.00 m` still reads CHECK and is expected now: the
reach floor stands down inside a traverse, so a mouth the car can barely see is
crawled rather than stopped in. That is the change that stopped traverses timing
out; see `speed_ctrl.duty`'s `min_reach_m`.

### When it goes wrong

| Symptom | Field to read | Likely cause |
|---|---|---|
| Drives into the wall and never stops | `dead_end_reason` | See the table in 7a. It is a detector or layout fault, not a decision fault |
| Stops in a clear corridor | `dead_end_reason`, `cones` | A perception dropout read as a wall. `only N cones in view` should have caught it — check the count |
| Takes the same branch twice | `explore_path`, `route_remaining` | The latch fired twice on one wall, or X was pressed without the car being moved |
| Stops mid-course at nothing | `goal_state`, `magenta_in_view` | A red misread as magenta. Take the trophy off the course |
| Route emitted with a phantom turn | `maze_nodes` vs junctions driven | A junction recorded twice — the backtrack re-entry was not matched to the same node |
| `no route written` | `goal_state` | The run ended without finding the goal, so there was nothing to plan |

## Stage 8 — the car as its own reverse

Stage 7 ends with you carrying the car back to the junction. This is the same
search with that carry taken out: at a wall the car backs ITSELF down the
branch, recognises the junction from the parent side, and drives through it on
the branch it has not tried.

```sh
~/env/bin/python drive_junction.py --weights ~/models/best.pt --explore \
    --reverse --invert-steering --max-range 3.5 --lookahead 0.8 \
    --max-duty 0.05 --emit-route routes/optimal.txt --log explore-rev-1.jsonl
```

**`--reverse` is off by default, and it requires `--explore`.** With it
off every path is stage 7b's, unchanged — which is what makes it safe to deploy
this branch and still demo the 2026-09-02 run from it. There is no clone on the
car to switch back with.

### What the DonkeyCar reverse already tells you

**This VESC has taken a negative duty before, under manual control.** DonkeyCar's
VESC part sends `set_duty_cycle(throttle * VESC_MAX_SPEED_PERCENT)` and joystick
throttle is −1..1, so pulling the stick back is a negative duty over the same
pyvesc link `VescDriver.drive` uses — the relationship `drive_corridor.VescDriver`
already documents as `duty = throttle * 0.2`. So "does it reverse at all" is
answered, and 8a is a confirmation rather than a gate.

Three things it does **not** answer, and the first is where the day is most
likely to stall:

- **Magnitude.** DonkeyCar reverse ran at up to `0.5 × 0.2 = 0.10` duty. The
  manoeuvre commands **0.05**, the cogging floor. Reversing under a thumb at
  0.10 is no evidence at all about 0.05. That is 8b.
- **From rest.** Manual reverse is usually already rolling, or rocked through
  zero. The manoeuvre starts from a dead stop at a wall. `speed_ctrl.ramp`
  pivots through zero rather than sliding across it, and `VESC_HAS_SENSOR` is
  true so startup is sensored — the case that starts best at low duty — but
  nothing has tried it.
- **Which car.** None of it transfers if that reverse was a different vehicle
  or a PWM/ESC drivetrain rather than `DRIVE_TRAIN_TYPE = "VESC"`.

### The rule for the whole day

**The car cannot see what it is reversing into.** The chassis fills the rear
142° (measured, `hardware-baseline.md`), leaving the forward ~218°. The only
guarantee about the ground behind the car is that it drove over it forward a
moment ago — void the instant anything moves, your own feet included. So:
reverse at the cogging floor, keep the corridor behind the car clear, keep a
hand on X, and note that every reverse carries a hard distance bound.

### 8a — confirm the reverse path *(bench, wheels off the ground)*

```sh
python drive_junction.py --weights ~/models/best.pt --explore --reverse-only
```

Commands `reverse_duty()` while X is held and nothing else — through the real
`VescDriver`, not a bespoke script, so the symmetric clamp is exercised
honestly. It announces itself loudly and refuses to be combined with
`--dry-run` or `--steer-only`.

Confirm the wheels turn **backwards**, that 0.05 duty turns them at all, and
that releasing X stops them. Unloaded on a stand this should just work, given
the manual reverse above; if it does not, the fault is in this tool's path
rather than in the VESC, and the log is the place to look.

`--max-reverse-duty` sweeps the magnitude without editing code. It warns above
twice the cogging floor and refuses a negative — the sign belongs to
`speed_ctrl.reverse_duty` and nowhere else.

### 8b — straight-line reverse *(floor, landmark cones ahead, clear floor behind)*

Same command, wheels down, 2–3 m of clear floor BEHIND the car, `--log`.

**Lay landmark cones ahead of the car.** The stage said "no cones" until 2026-09-03,
and two of the four readouts below cannot be taken that way: `ego_motion` is
scan-to-scan odometry over cone clusters, a step needs two scans with at least
one cone in common, and an empty scene returns None. On a bare floor `odo_pairs`
is 0 every tick and `odo_forward_m` is 0.0 — the run looks healthy and measures
nothing. The car reverses AWAY from cones placed ahead of it, so they stay in the
forward ~218 deg it can see while the rear arc stays empty, which is what "no
cones" was protecting. Space them well clear of `ego_motion.MATCH_GATE_M` (0.35 m)
and keep them inside `--max-range` at the END of the reverse: the first run to try
this had the cones too close together and the fit never locked (60% of ticks
disagreed with the direction of travel).

What to read out:

| | Where | Why it matters |
|---|---|---|
| **Does it move at 0.05 from a standstill?** | your eyes | The real question of the day. Manual reverse ran at 0.10 and usually already rolling; this is half that, from rest, on carpet or asphalt rather than a stand. Sweep `--max-reverse-duty` up until it breaks away reliably and **record what it took** — if it needs 0.08, that goes back into the gains, because `reverse_ctrl`'s loop stiffens with speed |
| **Reverse m/s** | a TAPE and the armed tick count | Settles a live contradiction, not just a missing number. The floor duty 0.05 against a forward-fitted `DUTY_TO_MPS` of 7.5 implies **0.375 m/s**, and `reverse_ctrl.MAX_REVERSE_MPS` says the gains have never been checked above **0.3**. No duty satisfies both — nothing can be commanded under the cogging floor — so the tool says so at startup and this measurement is what resolves it. `DUTY_TO_MPS` is much the weakest of the three. Measure it with a tape — mark the start, hold X, mark the end, divide by armed ticks / 10 Hz — and use summed `odo_forward_m` only as a cross-check: on this car it under-read the reverse by 12-28%, see below |
| Does it track straight or crab? | the floor | Slop and servo trim show here and nowhere else |
| **Does odometry survive?** | `odo_pairs` > 0 | `rigid_step` is sign-agnostic and 0.04 m/tick is well inside the 0.35 m match gate, so it should — and the manoeuvre's distance bound depends on it |

### 8b — what it did, 2026-09-03

Stage 8a first, on a stand: the wheels turned backwards at 0.05 duty and stopped
on release, through the real `VescDriver` (`data/trials/rev-8a.jsonl`). The VESC
takes a negative duty from this tool. That question is closed.

Then four floor runs, and the first of them found a fault that had nothing to do
with reverse. `data/trials/rev-8b.jsonl`: the car backed 120 in but arrived 40 in
to the RIGHT, an arc of radius 5.08 m, 36.9 deg of heading change over 3.05 m —
**with the servo commanded to dead centre on every one of 291 ticks**
(`servo_for(0.0)` is `0.5` whatever `--invert-steering` says). That is a 3.72 deg
mechanical bias at commanded centre: 19% of `MAX_STEER_RAD` spent holding
straight, and 2.42 m of drift over the manoeuvre's own 5.18 m bound, which is
wider than the corridor. The steering was adjusted mechanically before the next
run. Its odometry is also not to be trusted — the cones were too close together
and 40% of ticks disagreed with the direction of travel; the tape is the only
measurement from that run worth keeping.

Three runs after the adjustment, cones respaced:

| | run 1 | run 2 | run 3 |
|---|---|---|---|
| log in `data/trials/` | `rev-8b-run1-105in.jsonl` | `rev-8b-run2-118in.jsonl` | `rev-8b-run3-115in.jsonl` |
| tape | 105 in / 2.667 m | 118 in / 2.997 m | 115 in / 2.921 m |
| armed | 6.02 s | 6.57 s | 6.48 s |
| summed `odo_forward_m`, whole log | −2.084 m | −2.152 m | −2.566 m |
| odometry under-read | 22% | 28% | 12% |
| **speed** | 0.408 m/s | 0.435 m/s | 0.428 m/s |
| yaw over the run | +1.8 deg | **−12.3 deg** | +0.8 deg |

**Settled: the reverse runs at 0.423 m/s, sd 0.014** — three runs, a 3% spread,
measured by tape over the clock and so independent of the odometry. That gives a
reverse-side `DUTY_TO_MPS` of **8.5 ± 0.3** against the 7.5 fitted forward, and it
resolves the contradiction `speed_ctrl.reverse_mps` and the startup banner have
been stating all along — against us. The reverse is **1.41x
`reverse_ctrl.MAX_REVERSE_MPS`**, the speed above which the gains have never been
checked, and no duty can bring it down because 0.05 is already the cogging floor.

The steering fix worked: two of the three runs held within 2 deg of straight, and
median per-tick lateral error fell from 0.0395 m to 0.0075 m.

### What stage 8b leaves open — future work

Four things, in the order they block the rest of stage 8.

**1. The backout's distance bound is longer than the code believes.** Scan-matched
odometry under-read the reverse by 12%, 22% and 28% on three runs of the same
course at the same duty — a mean of 21% with a standard deviation of 10 points.
It is not a scale factor and cannot be corrected by one. `BackoutManoeuvre` bounds
its reverse on travelled distance, so the car runs 14-39% past the bound it thinks
it has, in the one direction it cannot see. **The manoeuvre should not be trusted
on a track until this is closed**, which is why 8e and 8f did not run on
2026-09-03.

The fix worth trying first: for the reverse specifically, the clock is a better
odometer than the scan matcher. The manoeuvre commands a CONSTANT duty and that
duty's speed is now known to 3%, so `elapsed x 0.423` estimates travel better than
`ego_motion` does going backwards. Taking the LARGER of the two — `max(odo_travel,
elapsed x 0.423)` — is the conservative choice for a bound, stops the car sooner,
and leaves forward driving untouched.

**2. The gains are 1.41x out of envelope.** `K_HEADING`/`K_CROSS` = 2.4/0.3 were
swept in sim on 2026-09-02 at `DUTY_TO_MPS` 7.5, which propels `sim/drive_sim.py`
at 0.375 m/s (`speed = duty_now * DUTY_TO_MPS`). The car does 0.423. Neighbouring
cells in that sweep FAIL, and `reverse_ctrl`'s docstring is explicit that a pair
which tracks at 0.2 m/s can oscillate into a wall at 0.5. Re-sweep the sim at the
measured speed before 8d, then settle the gains on the car.

**3. Run 2's −12.3 deg is not explained.** Two runs under 2 deg and one an order of
magnitude worse, same command, same course, minutes apart. Extrapolated over the
5.18 m bound that is roughly 0.6 m of lateral error against nearly none. Until it
is understood, drift over a full backout is not predictable.

**4. The coast is not near-zero in reverse, and is not yet measured.** 0.18 m mean
with a 0.079 m standard deviation across the three runs, and only measurable
through the same odometry that under-reads. `--goal-stop`'s 0.30 m default rests on
"the near-zero coast measured on this car", which was measured FORWARD. The
arrival band is about 0.5 m deep, so a reverse that coasts 0.18-0.26 m after
deciding it has arrived spends a third to a half of that band. Measure it properly:
mark the floor at the instant X is released, and measure that mark against where
the car comes to rest.

**A logging defect found on the way.** `drive_junction.py` passes the drive
pipeline's forward duty suggestion to `status_of`, not `duty_now` — the value the
VESC is actually given. Forward the two converge through `speed_ctrl.ramp` and the
difference has never mattered; in a reverse they differ in SIGN, and the 8b logs
record a positive `duty` for a car that was commanded negative. A backout analysed
from its log will read a forward duty for a reversing car. The fix is a separate
`duty_commanded` field rather than redefining `duty`, so `junction_report.py` and
every existing log keep their meaning.

### 8c — reverse steering sign *(stand, wheels off the ground)*

Stage 4's test in the other direction, and the one failure `reverse_ctrl` has
actually had. **The folk rule is half wrong**: backing up, the cross-track term
keeps its sign and only the heading term flips. Negating the whole law gives a
controller that corrects heading, fights position, and drives the car off the
centreline while its heading trace looks healthy.

With the corridor in view, read `backout_heading_err_deg` and
`backout_cross_track_m` off the log and check the wheels move the way
`reverse_ctrl`'s docstring says they should. Decide it here, never on the track.

### 8d — closed-loop reverse down a straight corridor

Isolates the controller from the manoeuvre. Car at the end of a straight
corridor facing down it; let `reverse_ctrl` regulate on what it can see ahead
for 2–3 m.

**Expect to re-tune here.** `K_HEADING`/`K_CROSS` = 2.4/0.3 were settled in sim
on 2026-09-02 and nothing has measured them on a car. The sweep that produced
them is in `reverse_ctrl`'s comments and is worth reading first: neighbouring
gain cells FAIL, so this is a working point on a marginal system rather than a
broad optimum. The failure mode is a clean-looking second followed by a spin —
watch `backout_cross_track_m` and `backout_heading_err_deg`, and note which one
was growing.

### 8e — the manoeuvre at the relaid junction

**Relay the track, then repeat stages 1 and 3 before anything moves.** New
geometry means new numbers: rebuild to `junction_v2.md`, do the static,
motionless gate-visibility check, and take the gaps from `gate_gaps_m` rather
than off the tape. Nothing downstream survives a junction the car cannot see,
and the whole manoeuvre ends on a gate sighting.

Then the walled branch at `--max-duty 0.05`, hand on X. Expect:

```
DEAD END      corridor ends 0.9 m ahead (orange wall seen)
              backing out N.NN m at most, to take RIGHT
BACKED OUT    now taking RIGHT
```

Release X for anything you did not expect. A release mid-reverse **abandons**
the manoeuvre rather than pausing it, and the run falls back to stage 7b's
carry — X keeps one meaning.

### 8f — full exploring run, hands off

Stage 7b with nobody touching the car.

### Reading the log

`backout_state` is the field to read first when a run ends somewhere odd.

| `backout_state` | Means | Where to look |
|---|---|---|
| `arrived` | Worked. `backout_gate_m` says where it stopped | Compare against the band stage 3 measured — 2.12–2.68 m on a v2 junction |
| `abandoned`, "without seeing the junction" | Reversed its whole bound and never recovered a triple | Stage 3's static check, on the parent side. The car cannot stop at a junction it cannot see |
| `abandoned`, "no corridor to steer on" | Blind for five ticks. It stops rather than reversing straight | Cone spacing behind the wall; the corridor it regulates on is the one AHEAD |
| `abandoned`, "released mid-reverse" | You let go of X | Nothing |
| `abandoned`, "timed out" | 20 s | The car is not moving. Back to 8a/8b |

`backout_travelled_m` against `backout_bound_m` is the other number worth a
glance. In sim the manoeuvre uses 74–78% of its bound; a run that finishes near
100% found the gate by luck and the next one will not.

### What it did in sim, 2026-09-02

Both mirror layouts of `junction-*-blocked`, rear 142° masked, gains 2.4/0.3.
**This is a simulation, and as of 2026-09-03 the MANOEUVRE has still not met a
car.** 8a and 8b have: the VESC reverses on a negative duty from this tool, and
the reverse runs at 0.423 m/s. Everything below was produced at 0.375, and every
row of it is a claim about a controller that has never regulated a real corridor
backwards.

| | LEFT-blocked | RIGHT-blocked |
|---|---|---|
| outcome | goal reached | goal reached |
| path taken | left → wall → **right** | right → wall → **left** |
| backout | 104 ticks / 10.4 s | 110 ticks / 11.0 s |
| reversed | 3.83 m of a 5.18 m bound (74%) | 4.06 m of 5.18 m (78%) |
| stopped at | 2.16 m from the gate, 2 sightings | 2.31 m, 2 sightings |
| cross-track | mean 0.036 m, peak 0.17 m | mean 0.053 m, peak 0.34 m |
| heading error | mean 3.2°, peak 32.2° | mean 3.9°, peak 23.3° |
| cones struck | none | none |

## What to bring back

- `junction-see.jsonl` per direction, and `junction-run-N.jsonl` per driven run,
  into `data/trials/`
- `explore-see.jsonl` and `explore-N.jsonl`, plus the `optimal.txt` the run
  emitted and the `optimal-N.jsonl` from driving it
- the map residual from `map_from_log.py --layout` on that last run — the number
  that says whether odometry can carry an edge length on the real car (the sim
  gives 0.009 m mean; anything under ~0.10 m is fine)
- how many `dead_end_reason` ticks it took to latch each wall, and whether orange
  was seen at all
- the measured gate gaps, from `gate_gaps_m` rather than the tape
- the photograph of the built junction
- whether `--invert-steering` was needed
- **reverse m/s at the floor duty, and whether 0.05 moved the car at all**
- straight-line reverse drift over 2 m, and `odo_pairs` through a reverse
- the gains 8d settled, against the sim's 2.4/0.3
- each backout's `backout_state` and reason, its `backout_travelled_m` against
  its bound, and the `backout_gate_m` it stopped on
- cross-track through each reverse, and which error was growing when one drifted
- whether any backout needed the stage 7b carry after all
- points-per-cluster on the trophy at 1, 2 and 3 m (stage 6a)
- where the car actually stopped against `--goal-stop`, and the
  `goal_blind_ticks` it arrived with

Enough to answer, at the desk, why any run did what it did — and enough to
correct `junction_v2.md` if the track disagrees with it.
