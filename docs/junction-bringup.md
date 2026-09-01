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
uv run --with pytest --with numpy python -m pytest -q      # 373 pass
uv run --with pytest --with numpy python -m pytest sim -q  # 34 pass, 3 known fails

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

## What to bring back

- `junction-see.jsonl` per direction, and `junction-run-N.jsonl` per driven run,
  into `data/trials/`
- the measured gate gaps, from `gate_gaps_m` rather than the tape
- the photograph of the built junction
- whether `--invert-steering` was needed
- points-per-cluster on the trophy at 1, 2 and 3 m (stage 6a)
- where the car actually stopped against `--goal-stop`, and the
  `goal_blind_ticks` it arrived with

Enough to answer, at the desk, why any run did what it did — and enough to
correct `junction_v2.md` if the track disagrees with it.
