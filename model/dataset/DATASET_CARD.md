# Dataset Card — Cone Detection (5 classes)

_Deliverable D1. Fill in as the dataset is built; don't retrofit at the end._

`prepare_dataset.py` prints the session table after a cull, and
`roboflow_export.py` prints the totals, per-class instance counts and split
rows after an export — both in the shape this file wants. Paste those in rather
than counting by hand; the numbers below should be measurements.

Numbers below are from the **v2 export** (`cone-detector-nfjog` version 2,
exported 2026-08-29). Lines marked TODO are the ones nobody has measured yet.

The three `will*` sessions (2026-08-29, 553 frames) are uploaded but not yet
labelled, so they are absent from every count in Composition. Those counts
describe v2 and will not describe v3.

## Composition
- Total images: **1777**
- Per class instance counts: blue **1863** / magenta **370** / orange **2415** /
  red **4105** / yellow **1449** (10202 total)
  - Watch the imbalance: the track carries ~36 boundary cones but only 4 red,
    2 orange and 1 magenta. Keep shooting those three (cone-zoo sessions, slow
    junction passes) until they are within roughly 3:1 of the boundary classes.
  - Red and orange are also the pair the detector is most likely to confuse, so
    they need range and lighting diversity, not just raw counts.
  - **The imbalance has inverted, not resolved.** The cone-zoo sessions
    (eli2–eli6) pushed red and orange *past* the boundary classes: red 4105 and
    orange 2415 against blue 1863 and yellow 1449. Magenta is still 11.1:1
    behind the commonest class. Zoo sessions fix scarcity by staging cones, but
    they stage them in a context the track never produces — see Known gaps.
- Split: train **1614** / valid **100** / test **63**, by *capture session*, not
  random-by-image, so the test set is genuinely unseen conditions. Assignments
  live in `splits.json`, which is the split of record.
  - train counts are **2x** the frames actually uploaded (807): Roboflow applied
    2 augmented outputs per training image when version 2 was generated. Valid
    and test are unaugmented. Do not read the train row as a frame count.

## Capture conditions
- Camera: OAK-D Lite RGB (IMX214), resolution: **1920x1080**, sampled at
  **2.0 Hz** from a 10 fps stream, JPEG quality 95
- Exposure / white balance: locked per session after a 2 s auto settle; the
  locked values are recorded in each session's `session.json`
- Track: `data/layouts/track_v1.md` (layout version used): TODO — eli2–eli6 are
  cone-zoo sessions and did not use the track layout at all
- Sites / surfaces: TODO — session names suggest a parking lot (`lot-sun-A`) and
  EBU2 (`ebu2_test*`); the `eli*` site is not recorded anywhere in the repo
- Lighting: TODO — not recorded per session. The settled exposure values are the
  only evidence on disk, and `lot-sun-A` is a clear outlier at 29985 us / ISO 721
  / 3072 K against ~2000 us / ISO 100 / ~4800 K everywhere else, which reads as
  a much darker scene than its name suggests. Worth resolving before anyone
  cites it as the sunny condition.
- Range span: TODO — not measured. `roboflow_export.py` reports box short side
  at 640x360 as min 2 px, p5 6 px, median 16 px, p95 87 px, max 191 px, which is
  a proxy for it.

## Sessions
_One row per capture session — this table is what defines the split._

| Session dir | Split | Condition | Frames kept / captured | Uploaded | Labeling | Notes |
|-------------|-------|-----------|------------------------|----------|----------|-------|
| 20260827_1053_lot-sun-A | train | TODO | 7 / 28 | 7 | v1-assisted | Settled exposure is a 12x outlier; see Lighting |
| 20260827_1413_ebu2_test | test | TODO | 63 / 83 | 63 | v1-assisted | Held out. The v1↔v2 comparison rests on this session |
| 20260827_1429_ebu2_test_2 | valid | TODO | 259 / 280 | 100 | v1-assisted | Only 7 magenta instances — too few to steer early stopping |
| 20260827_1503_eli1 | train | TODO | 1687 / 1720 | 400 | v1-assisted | 93% of v1's training data came from here |
| 20260828_1313_eli2 | *(null)* | TODO | 189 / — | 0 | none | No `session.json`: capture interrupted. Excluded on purpose |
| 20260828_1320_eli2 | train | cone zoo | 328 / 336 | 80 | hand | `lens_position=0` |
| 20260828_1324_eli3 | train | cone zoo | 342 / 361 | 80 | hand | `lens_position=0` |
| 20260828_1328_eli4 | train | cone zoo | 410 / 424 | 80 | hand | |
| 20260828_1348_eli5 | train | cone zoo | 547 / 549 | 80 | hand | |
| 20260828_1354_eli6 | train | cone zoo | 400 / 402 | 80 | hand | `lens_position=0` |
| 20260829_1718_will1 | train | corridor + goal | 281 / 285 | 281 | hand | Trophy cluster, no gate. Labeling pending |
| 20260829_1724_will2 | test | corridor + goal | 177 / 182 | 177 | hand | Adds the red gate pair and orange stub, so the test set covers all five classes. Labeling pending |
| 20260829_1726_will3 | valid | corridor + goal | 95 / 99 | 95 | hand | Puts real magenta in valid, where early stopping can finally see it. Labeling pending |

Captured − kept is the `prepare_dataset.py` cull (blurry frames and
near-duplicates). Rejects are **moved** to `<session>/_rejected/`, never deleted.

## Labeling
- Tool: Roboflow (`r-william-thatcher-s-workspace/cone-detector-nfjog`)
- Protocol: box = full cone incl. base; see `LABELING.md`. Boxes under 8 px short
  side are meant to be skipped — **1486 of 10202 (15%) are under that floor** in
  the v2 export, so the protocol is not being applied consistently
- QA: TODO — who second-checked, and what fraction
- Model-assisted portion: the four 2026-08-27 sessions were pre-labeled by v1 via
  `roboflow_prelabel.py` and corrected by hand (`_prelabel/` present).
  eli2–eli6 were uploaded raw and labeled entirely by hand, deliberately: v1 had
  never seen a magenta cone and proposes `red` on 69% of them, so its proposals
  would have been corrections to undo rather than work saved

## Known gaps
- **Magenta does not transfer from zoo to track.** Every magenta instance in
  `train` (350) comes from cone-zoo sessions; every magenta in `valid`/`test`
  is on-track. v2 scores **0.00 recall** on the 13 test magenta instances, with
  a confusion breakdown identical to v1's (9 of 13 called red, 4 missed). The
  other four classes appear in both contexts and all improved. This is a domain
  gap, not a volume problem — more zoo data will not close it. On-track frames
  with the goal cone in view are what is missing.
  Addressed on 2026-08-29: the `will*` sessions put a three-cone magenta
  trophy at the end of a track-spec corridor and sweep it from ~11 m to ~1 m,
  so magenta now appears framed by boundary cones instead of staged alone.
  Split across train/test/valid, which is the other half of the fix — v2 could
  not be assessed on magenta because no valid instances of it existed to score
  against. All three share one lighting condition; a second is still owed.
- **254 empty label files in train** (~127 originals before augmentation, ~16%
  of uploaded train frames). Not audited: if those are genuinely empty scenes
  they are fine, and if they are frames someone skipped in the labeler they are
  teaching the model that cones are background.
- **Three cone-zoo sessions report `lens_position=0`** (eli2 at 1320, eli3,
  eli6) where every other session settled between 43 and 95. Not investigated.
  If focus failed to lock, those frames may be soft, which would bear directly
  on the transfer failure above.
- Orange→red confusion is **15%** in v2 (10 of 67) against v1's 6% (1 of 16).
  v1's figure rested on a single error, so v2's is the more trustworthy of the
  two, but 15% is high for the confusion that matters most: a dead end read as
  a gate hands `junction_exec` a wall.
- No wet-pavement, dusk, or night images. No overcast condition recorded.
- Single track layout and single car; nothing tests a second cone size, and
  `CONE_HEIGHT_M` assumes one size across the whole track.
