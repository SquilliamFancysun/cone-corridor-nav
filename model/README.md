# CV Model Development

Everything about the cone detector that happens **off-car**. The on-car runtime
(DepthAI spatial detection node) lives in `ros2/src/cone_perception/`.

## Layout

```
dataset/
  images/    NOT in git (gitignored). Canonical copy lives in the team's
             shared Drive / Roboflow project — link it here once it exists.
  labels/    YOLO-format labels + train/val/test split files. IN git.
  DATASET_CARD.md
training/    Training scripts, configs, and exported training curves (D2)
export/      .pt -> ONNX -> OAK-D .blob conversion scripts
```

## Classes

| id | class    | color  | role              |
|----|----------|--------|-------------------|
| 0  | boundary | orange | corridor boundary |
| 1  | gate     | blue   | junction gate     |
| 2  | goal     | green  | goal marker       |

Class ids must match `cone_msgs/msg/LabeledCone.msg` — that file is the source
of truth.

## Pipeline

1. Photograph cones on-site: varied light, angle, range (Person C, day 2)
2. Hand-label 100–150 images; train v1; auto-label the rest; correct (day 3)
3. Retrain v2 on the full corrected set (day 4)
4. Export: `.pt` → ONNX → OAK-D blob (6 SHAVEs) for on-device inference
5. Trained weights/blobs attach to a GitHub Release, not to git
