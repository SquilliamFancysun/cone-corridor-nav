# Generative-AI Usage Log

_Deliverable D11 — maintained from day 1. For each significant use record what
was specified, what came back, what we changed, and how we verified it. Being
specific about what we corrected is the part that reads as engineering judgment._

## The short version

**This repository was written with Claude Code.** That was a deliberate
methodology choice made on day 1, not assistance bolted on at the end. The
question we were interested in was not "can an AI write this" but "what does
the human have to do so that what it writes is *true of the car*" — and the
answer, recorded across 12 days below, is: measure things, and refuse the
model's answer when the measurement disagrees.

Every module in `src/`, every one of the 710 tests, the bring-up manual, the
analysis tooling and this sentence were produced in that collaboration.

## The evidence

| | |
|---|---|
| Commits on the project branches | 117, over 13 days (2026-08-23 → 2026-09-04) |
| Carrying a `Co-Authored-By: Claude` trailer | **113 (96.6%)** — including the first commit in the repository |
| The 4 that do not | one teammate's mp3 playback feature |
| Python | 26,704 lines across 105 files |
| Tests | 710 test functions across 42 files |
| Documentation | 4,303 lines of markdown |
| Cumulative churn, text files | +51,169 / −3,802 |
| Commit-message prose | ~26,000 words — more than the entire docs tree |
| Verification trail | 26 on-car trial logs, 14 deploy tags, 3 training runs with committed curves |

Check the headline number yourself. Read the trailers rather than grepping the
body -- some commit messages discuss the trailer and a plain grep double-counts
them:

```bash
git log --format='%(trailers:key=Co-authored-by,valueonly)' | grep -c Claude
```

Counts are as of this commit; the command is the authority as the history
grows. The trailer is on the commits because they were written that way, not
applied retroactively. `6a5bf15`, the scaffold that created the directory structure,
carries it; so does `b9e424d`, the last measurement before submission.

## What the humans did

The honest counterpart, and the reason the above is a methodology rather than a
boast. None of this is code, and none of it could have been generated:

- **Built the track.** 43 cones laid to a surveyed layout, with a tape measure
  and chalk, and re-laid between runs.
- **Drove the car**, on a deadman, for every one of the 26 trial logs.
- **Measured.** The three reverse runs at 105 / 118 / 115 inches settled the
  reverse speed at 0.423 m/s tape-over-clock — deliberately independent of the
  odometry, which turned out to under-read by 12–28%. The lidar's 142° blind
  arc, the 3.72° servo bias, the 5.04 V rail: all measured, none assumed.
- **Fixed hardware.** The steering bias was corrected mechanically, by hand,
  between two runs.
- **Labelled the dataset.** ~150 bounding boxes drawn by hand to bootstrap v1,
  then corrections over four model-prelabeled sessions — and a deliberate
  decision to hand-label the magenta sessions entirely, because v1 called
  magenta "red" 69% of the time and its proposals would have been corrections
  to undo rather than work saved.
- **Overruled the model**, roughly a dozen recorded times, always on evidence
  from the physical world. Those are the rows below and the examples after them.

## Summary table

| Date | Task | Tool | What we specified | What came back | What we changed | How verified |
|------|------|------|-------------------|----------------|-----------------|--------------|
| 2026-08-23 | Repo scaffold: directory structure, ROS2 package boilerplate, message definitions, doc templates | Claude Code | Proposal doc (architecture §7, repo structure) + "separate sections for CV model and navigation dev" | This repository skeleton | — | colcon build in container; pip install -e on laptop |
| 2026-08-25 | Cone dataset capture tool + track spec (`model/capture/`, `data/layouts/track_v1.md`) | Claude Code | "Data collection script for the CV model: drive with a remote controller, start/stop recording at a reasonable framerate, for a YOLO cone detector" — plus a live SSH probe of the car | Plan (ROS vs. bare-Pi decision, track geometry, Roboflow/training/export steps) and the capture package | Rejected the model's initial T-junction track in favour of Y-junctions, so the corridor never leaves the camera's FOV mid-turn; corrected its claim that motion blur was a problem (the frames it inferred that from were an unrepresentative run); it had pinned autofocus to lens position 0 because it read the value before AF converged | `pytest test_session.py` (14 tests) on the laptop; on-car `--preview` and `--no-joystick --duration 10` → 20 frames at 1920x1080, inter-frame gaps 0.496–0.502 s, 20/20 unique, locked AE/AWB/focus recorded in `session.json`; `prepare_dataset.py` dry-run over 400 real frames |
| 2026-08-27 | Lidar bearing calibration (`model/capture/calibrate.py`, `lidar_view.py --calibrate`) | Claude Code | "Final step of calibrating the lidar and finding the centre before collecting lidar data", against the existing eyeball procedure in `docs/data-collection.md` §6 | A clustering + two-pose solver for the bearing sign and yaw offset, a `calibration.json` every later run loads, and the rewritten runbook step | Kept the model's two-pose design and made it a hard refusal rather than advice — one cone fits both signs exactly, so a one-pose run must not return an answer at all; made an uncalibrated recording a loud warning instead of an error, since `/scan_raw` keeps the sensor's own bearings and the session stays fixable at a desk; caught that `deploy.sh --delete` would wipe the car's calibration, and that `--angle-offset` plus `--mount-yaw` rotate the scan twice | `pytest` (29 new tests, 71 total) including the cases the solver must refuse; a full `--calibrate` dry run against a synthesized LD06 byte stream with a planted convention (mirror=True, offset=−8.0°), recovered to 0.01° |
| 2026-08-27 → 08-29 | Dataset pipeline and three training runs (`model/dataset/`, `model/training/`) | Claude Code | The Roboflow round trip: upload by session, prelabel with v1, export, train, evaluate, and check the class order against `LabeledCone.msg` | Upload/prelabel/export/train/evaluate scripts, the class-order contract, and the committed curves for v1–v3 | Renumbered the classes alphabetically to match what Roboflow assigns rather than fight the tool (`453f0ef`); caught that ultralytics silently *drops* images whose labels are polygons, so they had to be flattened, not just measured (`ffe5922`, `65f403f`); stopped prelabel proposing boxes over the 147 frames a human had already labelled (`cb909a8`, worked example 2) | v3 test mAP50-95 **0.715**, held-out split, per-class report committed at `model/training/v3/report_test.md`; class order asserted against the `.msg` by `test_cone_classes.py` on every run |
| 2026-08-28 → 08-30 | Perception fusion and corridor extraction (`src/cone_perception/`, `src/cone_nav/corridor/`) | Claude Code | Label lidar clusters with camera detections; pair the labelled cones into a corridor and extract a centreline, all pure Python testable in sim | Clustering, camera-to-lidar association, label memory, Delaunay pairing, boundary split, side assignment, centreline | Refused clusters closer than 0.20 m after a phantom at 4–7 cm — the car's own chassis — took a red label and completed a false junction triple on the track (`0c07dc7`, worked example 3); made the model measure the car's geometry and refuse to steer without it rather than shipping a default wheelbase (`922c497`) | Sim: 3.3–3.4 cm mean cross-track, 12.7° peak steer against a 20° limit; on the car, every trial log |
| 2026-08-30 → 09-01 | Junction detection, route execution, goal stop (`src/cone_nav/topology/`, `guidance/`) | Claude Code | Recognise a junction from a triple of red cones, turn as a route file dictates, then find the magenta trophy and stop short of it | Gate detection, the topology state machine, route cursor, goal detection and the run-in/stop controller | Replaced the model's assumed `DUTY_TO_MPS` with one calibrated from the first powered run (`5c6c359`); made the goal refuse a sighting off the corridor axis after a dry run showed the label alternating between the trophy and an object 1.17 m behind it (`f0a275c`, `e2c183d`); rejected 1.5 m gate spacing for 1.35 m after sim showed the wider gate visible for two ticks instead of five (`be0042b`) | Three dry runs kept in `data/trials/` as failure signatures (`goal-dry`, `goal-dry2`, `goal-dry3`), then `goal-run-1551.jsonl`: full route, stopped 0.265 m from the trophy |
| 2026-09-02 | Exploration, mapping and planning (`src/cone_nav/guidance/explore.py`, `planner.py`, `topology/graph_builder.py`, `analysis/map_from_log.py`) | Claude Code | Replace the route file with a search: pick a branch, recognise a dead end, build a graph, and emit the route that avoids what it learned | Frontier search, dead-end detection, incremental graph builder, route emission, and an off-car tool that rebuilds the map from a log | Made dead-end detection geometric first and let orange only *shorten* confirmation from 12 ticks to 5, because the detector's orange recall is 0.687 and 15% of oranges are called red — the model had wanted to trust the colour; rejected "corridor ends 0.00 m ahead" as a dead end, since a corridor that measures zero has vanished, not ended (`7cca921`); made the report stop counting every magenta-free tick as a blind tick, which had produced "carried 336" on a run that never saw the trophy (`5faec9c`, `d1d9385`) | `explore-run-1854.jsonl`: 868 ticks, found a dead end at 0.85 m, took the other branch, reached the goal, emitted `data/routes/optimal_explore_1854.txt` |
| 2026-09-02 → 09-03 | Autonomous reverse back-out (`src/cone_nav/guidance/backout.py`, `control/reverse_ctrl.py`) | Claude Code | Let the car reverse itself out of a dead end instead of being carried, steering backwards down the corridor until the junction is in view again | The manoeuvre, a reverse steering controller, sim coverage on both mirrors, and staged bring-up 8a–8f | **Did not accept the sim result.** Made the doc state that the 0.05 cogging floor and the gain envelope contradict each other (`dbe11fa`), and stopped 8a being called a gate when it is only a confirmation (`e370cc7`). Three tape-measured runs then showed the reverse runs at 1.41× the speed the gains were swept at, and that odometry under-reads it by 12–28% — so the manoeuvre was **not** run on the car, and the README says so | `rev-8a.jsonl` (VESC accepts negative duty), `rev-8b-run{1,2,3}` (0.423 m/s, sd 0.014). `backout_state` is empty on all 12,873 ticks of all 26 logs — this capability is unverified on hardware and is documented as such |
| 2026-09-04 | Submission polish: README, this document, the ROS-package removal | Claude Code | "Make the front page reflect the finished state, remove the stubs and unused files, and make the AI usage statement accurate" | The rewritten README with a "What does not work" section, this rewrite, and the `ros2/` → `src/` move | Directed it to keep `LabeledCone.msg` when it proposed deleting `cone_msgs` wholesale — the file is parsed as the class-order source of truth and asserted against in tests; required the deploy path change to be exercised against a local rsync target rather than eyeballed, since no test covers `deploy.sh` and the car is disassembled | 741 passed / 2 skipped unchanged across the move; `pytest sim` 76 passed with the same 3 known failures; rsync dry run landed 32 modules in the layout the on-car import guard expects, 0 tests leaked |

## Worked examples

Three cases where the generated code was wrong in a way only the physical world
could reveal. They are the argument that the method works: not that the model
was right, but that the loop caught it.

### Example 1 — the gamepad buttons that E-stopped the car (`b5bc168`, 2026-08-27)

**Specified:** a gamepad-triggered recorder for the camera and another for the
lidar, sharing one Logitech F710 with DonkeyCar already running.

**Came back:** working tools. `capture_cones.py` bound record to **A**;
`lidar_view.py` bound it to **B**. Both are reasonable defaults, and both are
wrong on this car, which nothing in the code could have shown.

**What the world said.** joydev gives every reader of `/dev/input/js0` its own
event stream — which is exactly what lets three processes share one gamepad —
but a button bound in two places fires *both* handlers. DonkeyCar binds A to
`emergency_stop` and B to `toggle_manual_recording`. So every camera record
toggle E-stopped the car, and every lidar toggle wrote junk MockCamera tubs
beside the session. Caught on the car during desk checkout, in DonkeyCar's own
log:

```
INFO:donkeycar.parts.controller:button: A state: 1
WARNING:donkeycar.parts.controller:E-Stop!!!
```

**Changed:** both tools moved to **X (index 2)**, which is absent from
DonkeyCar's map — and deliberately the *same* button for both, so one press
starts and stops the camera and lidar sessions together and a paired capture
stays aligned.

**Verified:** indices confirmed physically with `joystick.py --probe-buttons`
(X=2, A=0, B=1, start=7), and two further F710 traps written into
`docs/hardware-baseline.md`, including that the MODE button silently swaps the
left stick and the D-pad so steering dies while throttle keeps working.

### Example 2 — burying careful work under cheap work (`cb909a8`, 2026-08-28)

**Specified:** a model-assisted labelling step — train v1 on a hand-labelled
seed, then have it propose boxes on the rest for a human to correct.

**Came back:** `roboflow_prelabel.py`, which ran over every frame in `frames/`.
Correct against the brief, and about to do real damage: `frames/` includes the
147 images that went up as the *hand-label* batch. Uploading v1's guesses onto
them would have buried careful work under cheap work — a second, worse
annotation arriving on images already done properly.

**Changed:** `roboflow_upload.py` now writes `<session>/uploaded.json` as it
goes and prelabel skips those frames unless `--include-uploaded`. The record
lives beside the frames rather than in git on purpose: `images/` is gitignored
and prelabel can only run where the frames are, so the two travel together or
not at all.

A second bug surfaced in the same review: `upload_session` returned a *failure
count*, so a partial upload would have recorded frames that never arrived and
prelabel would then have skipped them forever. It returns the failures
themselves now.

**Verified:** backfilled for the batch already sent. The selection was
deterministic — evenly spaced over the sorted kept frames — so reproducing it
from the limits used recovers exactly 100 + 7 + 20 + 20 = 147, matching the
manifest.

### Example 3 — the cone that was the car (`0c07dc7`, 2026-08-31)

**Specified:** cluster the lidar returns into cone candidates, masking the arc
the chassis occupies.

**Came back:** clustering that masked the measured chassis arc. Correct, and
insufficient: the mask is built from a *measured* arc, and it leaks on the
revolutions where the arc was not measured.

**What the world said.** The leaked returns cluster into a phantom object 4–7 cm
from the lidar — the car's own body. On the track it took a **red** label from
the camera and completed a spurious junction triple, gaps 1.54 / 1.48 m, both
inside the pair window. The car committed a junction manoeuvre at a gate that
did not exist. No sim scenario would have produced this: it requires a real
chassis, a real mask, and a real detector willing to put red on it.

**Changed:** refuse candidates closer than 0.20 m — beyond the measured chassis
returns, and closer than any cone the car has not already hit.

**Verified:** `MIN_CONE_RANGE_M = 0.20` is enforced in `clustering.py`, and the
incident is preserved as a regression test that carries the story in its own
docstring — `test_a_cluster_at_the_lidar_is_the_car_not_a_cone` feeds a return
at 7 cm alongside a real cone at 1.5 m and asserts only the cone survives.

## A note on what this does not claim

The model did not build the car, drive it, or measure it, and on the evidence
above it was wrong in ways that mattered roughly a dozen times — each time
about something that could only be learned by putting a tape or a wheel on the
ground. What it did was write essentially all of the code, and write it fast
enough that the bottleneck moved to the track, which is where the interesting
part of this project turned out to live.

The one capability this repository does *not* claim — the autonomous back-out —
is the one where the sim was green and the car never got to answer. That is the
whole lesson, and it is why the README has a section for it.
