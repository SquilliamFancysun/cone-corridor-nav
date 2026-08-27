# Dataset Card — Cone Detection (3 classes)

_Deliverable D1. Fill in as the dataset is built; don't retrofit at the end._

`prepare_dataset.py` prints the session table after a cull, and
`roboflow_export.py` prints the totals, per-class instance counts and split
rows after an export — both in the shape this file wants. Paste those in rather
than counting by hand; the numbers below should be measurements.

## Composition
- Total images:
- Per class instance counts: blue / yellow / red / orange / magenta:
  - Watch the imbalance: the track carries ~36 boundary cones but only 4 red,
    2 orange and 1 magenta. Keep shooting those three (cone-zoo sessions, slow
    junction passes) until they are within roughly 3:1 of the boundary classes.
  - Red and orange are also the pair the detector is most likely to confuse, so
    they need range and lighting diversity, not just raw counts.
- Split: train / val / test (by *capture session*, not random-by-image, so the
  test set is genuinely unseen conditions). Hold back a whole lighting
  condition rather than skimming frames off each one:

## Capture conditions
- Camera: OAK-D Lite RGB (IMX214), resolution:
- Exposure / white balance: locked per session after a 2 s auto settle; the
  locked values are recorded in each session's `session.json`
- Track: `data/layouts/track_v1.md` (layout version used):
- Sites / surfaces:
- Lighting: (direct sun / shade / overcast — list sessions per condition)
- Range span: (closest–farthest cone distances represented)

## Sessions
_One row per capture session — this table is what defines the split._

| Session dir | Condition | Frames kept / captured | Notes |
|-------------|-----------|------------------------|-------|
|             |           |                        |       |

## Labeling
- Tool:
- Protocol: (box = full cone incl. base; occlusion rule; min box size)
- QA: (who second-checked, what fraction)
- Model-assisted portion: (which images were auto-labeled by v1, then corrected)

## Known gaps
- (e.g., no wet-pavement images; few backlit gate cones; ...)
