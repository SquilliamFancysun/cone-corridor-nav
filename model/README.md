# CV Model Development

Everything about the cone detector that happens **off-car**. The on-car runtime
(DepthAI spatial detection node) lives in `ros2/src/cone_perception/`.

## Layout

```
capture/     On-car capture tool: gamepad-triggered OAK-D recorder (see its README)
dataset/
  images/    NOT in git (gitignored). Canonical copy lives in the team's
             shared Drive / Roboflow project — link it here once it exists.
  labels/    YOLO-format labels + train/val/test split files. IN git.
  prepare_dataset.py   Pull sessions off the car, dedupe, cull, contact-sheet
  LABELING.md          Roboflow setup, box protocol, augmentation, training, export
  DATASET_CARD.md
training/    Training scripts, configs, and exported training curves (D2)
export/      .pt -> ONNX -> OAK-D .blob conversion scripts
```

`capture/` runs on the Pi but produces a dataset, so it lives here with the rest
of D1 rather than in `ros2/src/`. It deliberately does not use ROS: the dataset
is an off-car deliverable, and rosbag round-tripping would add a lossy re-encode
for no benefit. Its DepthAI pipeline is the reference camera configuration that
`cone_perception/yolo_node.py` should mirror at inference time — if capture and
inference disagree on white balance, the detector sees different colors than it
trained on.

## Classes

| id | class  | role                                          |
|----|--------|-----------------------------------------------|
| 0  | blue   | left corridor boundary                        |
| 1  | yellow | right corridor boundary                       |
| 2  | orange | junction gate — always placed in pairs        |
| 3  | green  | goal marker                                   |

The class *is* the color; left/right are relative to the direction of travel, so
the same corridor driven in reverse still has blue on its own left. Class ids
must match `cone_msgs/msg/LabeledCone.msg` — that file is the source of truth,
and the Roboflow project's class order must be verified against it in the
exported `data.yaml` rather than assumed.

Because color carries the class, **never augment with hue or saturation jitter**
— it teaches the model that blue and yellow are interchangeable.

## Pipeline

0. Build and survey the track (`data/layouts/track_v1.md`) — capture happens on
   the course we actually intend to run
1. Drive the track and record with `capture/` (Person C, day 2)
2. Hand-label 100–150 images; train v1; auto-label the rest; correct (day 3)
3. Retrain v2 on the full corrected set (day 4)
4. Export: `.pt` → ONNX → OAK-D blob (6 SHAVEs) for on-device inference
5. Trained weights/blobs attach to a GitHub Release, not to git
