# Labeling and training protocol

The steps between `prepare_dataset.py` and a `.blob` on the car. Fill
`DATASET_CARD.md` in as you go rather than reconstructing it afterwards.

## Roboflow project setup

Object Detection project. Create the classes **in this order**, because the
index Roboflow assigns has to match the constants in
`cone_msgs/msg/LabeledCone.msg`:

| id | name |
|----|--------|
| 0 | `blue` |
| 1 | `yellow` |
| 2 | `orange` |
| 3 | `green` |

**Verify the order in the exported `data.yaml`.** Do not assume it — a silent
permutation here turns into a detector that calls every gate cone a boundary,
and it is invisible until the car drives into a dead end.

## Upload one batch per session

Name each batch after its session directory, then assign train/valid/test **at
batch granularity**. Roboflow's default random split puts near-identical frames
on both sides of the boundary, which inflates mAP and hides exactly the
generalization failure the test set exists to catch. Hold back a whole lighting
condition rather than skimming frames off each one.

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
those, then use Label Assist for the rest and human-correct. Note in the dataset
card which images were auto-labeled — the corrections are the interesting part.

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

Export YOLOv8 format. Labels, `data.yaml` and the split files go in
`model/dataset/labels/`; images stay out of git.

## Training

YOLOv8**n** — nano is effectively required for the Myriad X. Colab T4 is the
recommended runner; `device=mps` on the Mac works for a smoke test.

```
yolo detect train model=yolov8n.pt data=data.yaml imgsz=640 \
     epochs=100 batch=16 patience=20 project=model/training name=v1
```

Commit `results.csv`, the curves and the config to `model/training/` (D2 wants
the curves). Weights attach to a GitHub Release — `.gitignore` blocks `*.pt`.

**Report per-class mAP50-95, not just the average.** Orange and yellow are the
pair most likely to confuse under warm low-angle sun, and green has the fewest
instances on the track; a single averaged number hides both.

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
confirm all four classes separate — particularly orange vs. yellow, and that
green is found at all.
