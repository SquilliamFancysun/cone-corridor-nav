# Generative-AI Usage Log

_Deliverable D11 — maintained from day 1. For each significant use record what
was specified, what came back, what we changed, and how we verified it. Being
specific about what we corrected is the part that reads as engineering judgment._

## Summary table

| Date | Task | Tool | What we specified | What came back | What we changed | How verified |
|------|------|------|-------------------|----------------|-----------------|--------------|
| 2026-08-23 | Repo scaffold: directory structure, ROS2 package boilerplate, message definitions, doc templates | Claude Code | Proposal doc (architecture §7, repo structure) + "separate sections for CV model and navigation dev" | This repository skeleton | — | colcon build in container; pip install -e on laptop |
| 2026-08-25 | Cone dataset capture tool + track spec (`model/capture/`, `data/layouts/track_v1.md`) | Claude Code | "Data collection script for the CV model: drive with a remote controller, start/stop recording at a reasonable framerate, for a YOLO cone detector" — plus a live SSH probe of the car | Plan (ROS vs. bare-Pi decision, track geometry, Roboflow/training/export steps) and the capture package | Rejected the model's initial T-junction track in favour of Y-junctions, so the corridor never leaves the camera's FOV mid-turn; corrected its claim that motion blur was a problem (the frames it inferred that from were an unrepresentative run); it had pinned autofocus to lens position 0 because it read the value before AF converged | `pytest test_session.py` (14 tests) on the laptop; on-car `--preview` and `--no-joystick --duration 10` → 20 frames at 1920x1080, inter-frame gaps 0.496–0.502 s, 20/20 unique, locked AE/AWB/focus recorded in `session.json`; `prepare_dataset.py` dry-run over 400 real frames |
| 2026-08-27 | Lidar bearing calibration (`model/capture/calibrate.py`, `lidar_view.py --calibrate`) | Claude Code | "Final step of calibrating the lidar and finding the centre before collecting lidar data", against the existing eyeball procedure in `docs/data-collection.md` §6 | A clustering + two-pose solver for the bearing sign and yaw offset, a `calibration.json` every later run loads, and the rewritten runbook step | Kept the model's two-pose design and made it a hard refusal rather than advice — one cone fits both signs exactly, so a one-pose run must not return an answer at all; made an uncalibrated recording a loud warning instead of an error, since `/scan_raw` keeps the sensor's own bearings and the session stays fixable at a desk; caught that `deploy.sh --delete` would wipe the car's calibration, and that `--angle-offset` plus `--mount-yaw` rotate the scan twice | `pytest` (29 new tests, 71 total) including the cases the solver must refuse; a full `--calibrate` dry run against a synthesized LD06 byte stream with a planted convention (mirror=True, offset=−8.0°), recovered to 0.01° |

## Worked examples

_Include 2–3 with the actual prompt and the diff between generated and final code._

### Example 1:
