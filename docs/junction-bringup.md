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

`--dry-run` never opens the VESC. Watch the state line — it prints on every
transition — and expect exactly one `follow → approach → traverse → follow`.

Then read the log rather than trusting the impression:

```sh
jq -r 'select(.gate_live) | "\(.t)  gate \(.gate_range_m)m  gaps \(.gate_gaps_m)"' junction-see.jsonl
jq -s '[.[] | select(.gate_live)] | length' junction-see.jsonl
```

| What to check | Expect | If not |
|---|---|---|
| Ticks with `gate_live` true | **≥ 4** (the sim gets 4.2 at 1.2 m/s; walking pace gives more) | Stage 1 is wrong. Measure the gaps and the clear band again |
| `gate_gaps_m` | ≈ `1.35/1.35`, both within 0.05 | This is the car measuring your tape work. Believe it over the tape |
| `gate_range_m` when first live | ≈ 2.6 m, falling to ≈ 2.1 m | A much shorter span means one red is being missed |
| `topo_state` sequence | one clean pass | Two `traverse` blocks means it re-armed on the same junction |
| `branch_cones_dropped` during `traverse` | **> 0** | The filter never bit; the car would be choosing its own branch |
| `topo_note` | `passed` once | `traverse timed out` means it never saw a corridor on the far side |

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

## What to bring back

- `junction-see.jsonl` per direction, and `junction-run-N.jsonl` per driven run,
  into `data/trials/`
- the measured gate gaps, from `gate_gaps_m` rather than the tape
- the photograph of the built junction
- whether `--invert-steering` was needed

Enough to answer, at the desk, why any run did what it did — and enough to
correct `junction_v2.md` if the track disagrees with it.
