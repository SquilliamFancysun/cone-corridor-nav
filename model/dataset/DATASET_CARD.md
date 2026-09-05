# Dataset Card — Cone Detection (5 classes)

_Deliverable D1. Fill in as the dataset is built; don't retrofit at the end._

`prepare_dataset.py` prints the session table after a cull, and
`roboflow_export.py` prints the totals, per-class instance counts and split
rows after an export — both in the shape this file wants. Paste those in rather
than counting by hand; the numbers below should be measurements.

Numbers below are the **v3 export** (`cone-detector-nfjog` version 3, exported
2026-08-30), which is the dataset the deployed detector was trained on. They
were recomputed from the label files committed in `labels/` and cross-check
exactly against `labels/manifest.json`.

Fields marked *not recorded* were never captured at the time and cannot be
recovered: the car has been disassembled and the sessions are closed. They are
left visible rather than dropped, because an unmeasured field is a real gap in
a dataset card.

## Composition
- Total images: **2611**
- Per class instance counts: blue **3570** / magenta **1057** / orange **2763** /
  red **4651** / yellow **3339** (**15380** total)
  - Watch the imbalance: the track carries ~36 boundary cones but only 4 red,
    2 orange and 1 magenta. Keep shooting those three (cone-zoo sessions, slow
    junction passes) until they are within roughly 3:1 of the boundary classes.
  - Red and orange are also the pair the detector is most likely to confuse, so
    they need range and lighting diversity, not just raw counts.
  - **The imbalance inverted rather than resolving, and v3 partly walked it
    back.** The cone-zoo sessions (eli2–eli6) pushed red and orange *past* the
    boundary classes in v2. The `will*` corridor sessions added to v3 brought
    blue and yellow back up, so red now leads blue 4651:3570 rather than
    4105:1863. Magenta is the class that improved most — 370 instances in v2,
    **1057** in v3 — and it is the improvement that shows up in the metrics.
- Split: train **2176** / valid **195** / test **240**, by *capture session*, not
  random-by-image, so the test set is genuinely unseen conditions. Assignments
  live in `splits.json`, which is the split of record.
  - train counts are **2x** the frames actually uploaded (1088): Roboflow applied
    2 augmented outputs per training image. Valid and test are unaugmented. Do
    not read the train row as a frame count.

## Capture conditions
- Camera: OAK-D Lite RGB (IMX214), resolution: **1920x1080**, sampled at
  **2.0 Hz** from a 10 fps stream, JPEG quality 95
- Exposure / white balance: locked per session after a 2 s auto settle; the
  locked values are recorded in each session's `session.json`
- Track: `data/layouts/track_v1.md` for the `will*` corridor sessions. eli2–eli6
  are cone-zoo sessions and did not use the track layout at all; the 2026-08-27
  sessions predate the layout being surveyed
- Sites / surfaces: partially recorded. Session names give a parking lot
  (`lot-sun-A`) and EBU2 (`ebu2_test*`); the `eli*` and `will*` sites are *not
  recorded* anywhere in the repo
- Lighting: *not recorded* per session. The settled exposure values are the only
  evidence on disk, and `lot-sun-A` is a clear outlier at 29985 us / ISO 721
  / 3072 K against ~2000 us / ISO 100 / ~4800 K everywhere else, which reads as
  a much darker scene than its name suggests. Nothing should cite it as the
  sunny condition on the strength of the name alone.
- Range span: not measured directly. Box short side over the whole v3 export,
  at the 640x360 reference used throughout this card, is min **2 px**, p5
  **6 px**, median **17 px**, p95 **86 px**, max **191 px** — a proxy for it.

## Sessions
_One row per capture session — this table is what defines the split._

| Session dir | Split | Condition | Frames kept / captured | Uploaded | In v3 (aug.) | Labeling | Notes |
|-------------|-------|-----------|------------------------|----------|--------------|----------|-------|
| 20260827_1053_lot-sun-A | train | not recorded | 7 / 28 | 7 | 14 | v1-assisted | Settled exposure is a 12x outlier; see Lighting |
| 20260827_1413_ebu2_test | test | not recorded | 63 / 83 | 63 | 63 | v1-assisted | Held out. The v1↔v2↔v3 comparison rests on this session |
| 20260827_1429_ebu2_test_2 | valid | not recorded | 259 / 280 | 100 | 100 | v1-assisted | Only 7 magenta instances — too few to steer early stopping on its own |
| 20260827_1503_eli1 | train | not recorded | 1687 / 1720 | 400 | 800 | v1-assisted | 93% of v1's training data came from here |
| 20260828_1313_eli2 | *(null)* | cone zoo | 189 / — | 0 | 0 | none | No `session.json`: capture interrupted. Excluded on purpose |
| 20260828_1320_eli2 | train | cone zoo | 328 / 336 | 80 | 160 | hand | `lens_position=0` |
| 20260828_1324_eli3 | train | cone zoo | 342 / 361 | 80 | 160 | hand | `lens_position=0` |
| 20260828_1328_eli4 | train | cone zoo | 410 / 424 | 80 | 160 | hand | |
| 20260828_1348_eli5 | train | cone zoo | 547 / 549 | 80 | 160 | hand | |
| 20260828_1354_eli6 | train | cone zoo | 400 / 402 | 80 | 160 | hand | `lens_position=0` |
| 20260829_1718_will1 | train | corridor + goal | 281 / 285 | 281 | 562 | hand | Trophy cluster, no gate |
| 20260829_1724_will2 | test | corridor + goal | 177 / 182 | 177 | 177 | hand | Adds the red gate pair and orange stub, so the test set covers all five classes |
| 20260829_1726_will3 | valid | corridor + goal | 95 / 99 | 95 | 95 | hand | Puts real magenta in valid, where early stopping can finally see it |

Captured − kept is the `prepare_dataset.py` cull (blurry frames and
near-duplicates). Rejects are **moved** to `<session>/_rejected/`, never deleted.

The three `will*` sessions were labelled after the v2 export and are what
separates v3 from it. In v2 they were uploaded but unlabelled and therefore
absent from every count.

## Labeling
- Tool: Roboflow (`r-william-thatcher-s-workspace/cone-detector-nfjog`)
- Protocol: box = full cone incl. base; see `LABELING.md`. Boxes under 8 px short
  side are meant to be skipped — **1953 of 15380 (13%) are under that floor** in
  the v3 export, so the protocol is not being applied consistently. It was 15%
  in v2, so this did not improve so much as get diluted by cleaner sessions
- QA: *not recorded* — nobody second-checked a defined fraction, and there is no
  inter-annotator agreement figure. This is the weakest part of the card
- Model-assisted portion: the four 2026-08-27 sessions were pre-labeled by v1 via
  `roboflow_prelabel.py` and corrected by hand (`_prelabel/` present).
  eli2–eli6 and the `will*` sessions were uploaded raw and labeled entirely by
  hand, deliberately: v1 had never seen a magenta cone and proposes `red` on 69%
  of them, so its proposals would have been corrections to undo rather than work
  saved

## What v3 bought

Against the same held-out test split, mean mAP50-95 went **0.514 (v1) → 0.562
(v2) → 0.715 (v3)**, and nearly all of the last jump is magenta:

| | v2 | v3 |
|---|---|---|
| magenta test instances | 13 | 233 |
| magenta recall | **0.00** | **0.684** |
| magenta mAP50-95 | 0.026 | 0.653 |
| magenta called red | 9 of 13 (69%) | 35 of 233 (15%) |

The v2 row is why the `will*` sessions were shot. Full per-class figures are in
`model/training/v3/report_test.md`.

## Known gaps
- **Magenta transfers now, but only from one lighting condition.** In v2 every
  magenta instance in `train` came from cone-zoo sessions, every magenta in
  `valid`/`test` was on-track, and v2 scored 0.00 recall on the 13 test
  instances. The `will*` sessions put a three-cone magenta trophy at the end of
  a track-spec corridor and sweep it from ~11 m to ~1 m, so magenta now appears
  framed by boundary cones instead of staged alone — and recall went to 0.684.
  All three `will*` sessions share one lighting condition; a second is still
  owed, and 32% of magenta is still being missed.
- **412 empty label files** (384 in train, 27 in test, 1 in valid). The train
  figure is 192 originals before augmentation, **~18% of the 1088 uploaded
  train frames**, up from ~16% in v2. Not audited: if those are genuinely empty scenes they
  are fine, and if they are frames someone skipped in the labeler they are
  teaching the model that cones are background. The count grew with the dataset
  rather than being addressed.
- **Three cone-zoo sessions report `lens_position=0`** (eli2 at 1320, eli3,
  eli6) where every other session settled between 43 and 95. Not investigated.
  If focus failed to lock, those frames may be soft.
- **Orange is the class that did not improve.** Orange→red confusion is **15%**
  in v3 (10 of 67), unchanged from v2, and orange mAP50-95 is 0.583 against
  yellow's 0.836. This is the confusion that matters most on the track: a dead
  end read as a gate hands `junction_exec` a wall. It is also why dead-end
  detection is geometric first and uses orange only to shorten confirmation —
  see `src/cone_nav/topology/dead_end.py`.
- Only 67 orange instances in the test split, so that 15% rests on 10 errors.
  Treat it as a signal, not a precise rate.
- No wet-pavement, dusk, or night images. No overcast condition recorded.
- Single track layout and single car; nothing tests a second cone size, and
  `CONE_HEIGHT_M` assumes one size across the whole track.
