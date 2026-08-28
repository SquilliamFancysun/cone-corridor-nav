# Labeling and training protocol

The steps between `prepare_dataset.py` and a `.blob` on the car. Fill
`DATASET_CARD.md` in as you go rather than reconstructing it afterwards.

## Roboflow project setup

Object Detection project. Create the classes **in this order**, because the
index Roboflow assigns has to match the constants in
`cone_msgs/msg/LabeledCone.msg`:

| id | name |
|----|-----------|
| 0 | `blue` |
| 1 | `yellow` |
| 2 | `red` |
| 3 | `orange` |
| 4 | `magenta` |

**Verify the order in the exported `data.yaml`.** Do not assume it — a silent
permutation here turns into a detector that calls every gate cone a boundary,
and it is invisible until the car drives into a dead end.

`roboflow_export.py` does that check against `LabeledCone.msg` and refuses to
sync anything when it fails; `train.py` repeats it before the first epoch. Both
read the ids out of the `.msg` rather than trusting a copy.

## Upload one batch per session

Name each batch after its session directory, then assign train/valid/test **at
batch granularity**. Roboflow's default random split puts near-identical frames
on both sides of the boundary, which inflates mAP and hides exactly the
generalization failure the test set exists to catch. Hold back a whole lighting
condition rather than skimming frames off each one.

Decide the split in [`splits.json`](splits.json), then:

```bash
export ROBOFLOW_API_KEY=...
cd model/dataset
python roboflow_upload.py \
    --workspace WS --project PROJ --dry-run     # then without --dry-run
```

It refuses to upload a session nobody has assigned a split to, and renames
frames to `<session>__<frame>.jpg` on the way up. Frame numbers restart every
session, so without the prefix they collide — and the prefix survives the round
trip, which is what lets the export script prove afterwards that no session
straddles two splits.

For the first hand-label batch, `--limit 150` takes evenly-spaced frames across
each drive rather than one end of it.

## Box protocol

- Box the **full cone including its base**. The base is what the surveyed
  ground-truth positions in `data/layouts/track_v1.csv` refer to, and the
  `range_bbox` estimate (`Z = f * h_real / h_pixels`) assumes full height.
- Label occluded cones down to **~40% visible**; skip below that.
- Skip boxes under **~8 px**.
- Label cones all the way to the horizon, not just the near ones — range
  diversity is the point.

Record whatever you actually did in `DATASET_CARD.md`, including who
second-checked and what fraction.

## Model-assisted labeling

Hand-label ~150 images spread across every session and condition, train v1 on
those, then let v1 propose boxes for the rest and human-correct. Note in the
dataset card which images were auto-labeled — the corrections are the
interesting part.

Roboflow's own Label Assist wants the model hosted there; we train locally, so:

```bash
python roboflow_prelabel.py \
    --weights ../training/v1/weights/best.pt \
    --session 20260827_1503_eli1 --workspace WS --project PROJ
```

The proposals go into a `<session>-auto` batch tagged `auto-labeled`, so the
fraction the card asks for is a filter in the app rather than a guess. Run it
with `--no-upload` first: it prints per-class proposal counts, which tells you
whether v1 finds magenta at all before you spend upload quota finding out.

## Preprocessing and augmentation

- Auto-orient on.
- Resize to 640x640 **letterbox / "fit", not stretch** — matches how DepthAI
  letterboxes at inference.
- **No hue or saturation jitter.** Color *is* the class signal here; hue jitter
  teaches the model that blue and yellow are interchangeable. This is the single
  most damaging default to leave on.
- Fine: brightness ±15%, exposure ±10%, slight blur, noise, mosaic.
- Horizontal flip is fine — the class is the cone's color, and left/right
  corridor semantics are resolved downstream in `cone_nav`, not by the detector.

Export YOLOv8 format, then pull it down with the script rather than by hand:

```bash
python roboflow_export.py \
    --workspace WS --project PROJ --version 3
```

It downloads into `model/dataset/export/` (gitignored), then checks the class
order against `LabeledCone.msg`, checks that no capture session appears in two
splits, and prints per-class instance counts and box-size percentiles in the
shape `DATASET_CARD.md` asks for. Only if all of that passes does it sync the
labels, `data.yaml` and a `manifest.json` into `model/dataset/labels/`, which is
in git — committing a broken export would make it the record of what the model
trained on.

## Training

YOLOv8**n** — nano is effectively required for the Myriad X. Colab T4 is the
recommended runner; `device=mps` on the Mac works for a smoke test.

```bash
cd model/training
python train.py \
    --data ../dataset/export/PROJ-v3/data.yaml --name v1
```

`train.py` wraps `yolo detect train` with the settings above and pins the
augmentation that matters: `hsv_h` and `hsv_s` are 0 and there is no flag to
raise them. It also stamps `train_config.json` next to the curves — commit,
dataset export sha, library versions — so a curve in the report can be traced
to what produced it.

Commit `results.csv`, the curves and `train_config.json` to `model/training/`
(D2 wants the curves). Weights attach to a GitHub Release — `.gitignore` blocks
`*.pt`.

**Report per-class mAP50-95, not just the average.** Red and orange are the pair
most likely to confuse — nearest colors, opposite meanings — and magenta has the
fewest instances on the track; a single averaged number hides both. `evaluate.py`
prints the per-class table, flags any class with no instances in the split (an
em-dash, not a zero — nothing was measured), and reads the confusion matrix out
in words, orange-vs-yellow both directions.

## Export to the OAK-D

Use the Luxonis converter at `tools.luxonis.com` (or the `blobconverter` CLI):
upload `best.pt`, select YOLOv8 detection, `imgsz=640`, **6 SHAVEs**. It strips
the detection head into the DepthAI `YoloDetectionNetwork` form and returns the
`.blob` plus a JSON of anchors/masks/classes.

Commit that JSON to `model/export/` — `yolo_node.py` needs it at runtime and it
is small. Export at 416 as well and benchmark both on-car: the OAK-D here
negotiates USB 2.0, so 640 may be too slow for the control loop. That
measurement belongs in the D3 perception characterization.

## The thing to check at the end

Run `best.pt` over held-out frames from a session that was never trained on and
confirm all five classes separate — particularly red vs. orange, and that
magenta is found at all.

```bash
python evaluate.py \
    --weights v1/weights/best.pt --data ../dataset/export/PROJ-v3/data.yaml \
    --split test --images ../dataset/images/<held-out-session>/frames
```

The `--images` sweep needs no labels: it reports per-class detection counts and
confidence spread per session and writes annotated samples, so "magenta is never
detected here" shows up as a line rather than as a car that drives past the goal.
