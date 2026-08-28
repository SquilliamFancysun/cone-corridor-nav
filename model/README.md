# CV Model Development

Everything about the cone detector that happens **off-car**. The on-car runtime
(DepthAI spatial detection node) lives in `ros2/src/cone_perception/`.

## Layout

```
capture/     On-car capture tool: gamepad-triggered OAK-D recorder (see its README)
cone_classes.py  Class order, parsed from LabeledCone.msg, and the checks against it
runtime.py       Device selection and run provenance, shared by the scripts below
dataset/
  images/    NOT in git (gitignored). Canonical copy lives in the team's
             shared Drive / Roboflow project — link it here once it exists.
  labels/    YOLO labels, data.yaml and manifest.json from the last export. IN git.
  export/    Downloaded Roboflow exports. NOT in git; re-download them.
  prepare_dataset.py     Pull sessions off the car, dedupe, cull, contact-sheet
  splits.json            Which session goes to which split — the split lives here
  roboflow_upload.py     Upload sessions as batches, one split per session
  roboflow_prelabel.py   Propose boxes with v1 for correction (model-assisted step)
  roboflow_export.py     Download a version, check it, sync labels into git
  LABELING.md            Roboflow setup, box protocol, augmentation, training, export
  DATASET_CARD.md
training/    train.py, evaluate.py, and one directory per run (D2 curves)
export/      .pt -> ONNX -> OAK-D .blob conversion scripts
```

`capture/` runs on the Pi but produces a dataset, so it lives here with the rest
of D1 rather than in `ros2/src/`. It deliberately does not use ROS: the dataset
is an off-car deliverable, and rosbag round-tripping would add a lossy re-encode
for no benefit. Its DepthAI pipeline is the reference camera configuration that
`cone_perception/yolo_node.py` should mirror at inference time — if capture and
inference disagree on white balance, the detector sees different colors than it
trained on.

## Off-car environment

Everything here except `capture/` runs on a laptop, against a venv in `model/.venv`
(gitignored):

```bash
cd model
/path/to/python3.11-or-newer -m venv .venv   # NOT bare `python3` on macOS — see below
source .venv/bin/activate
pip install -r requirements.txt
```

**Python 3.11 is a hard floor** (numpy 2.3.5). macOS ships 3.9 as `python3` and
it wins the PATH even inside a conda `(base)` shell; the resulting venv fails
with `Could not find a version that satisfies the requirement roboflow==1.4.1`,
which looks like a wrong pin and is actually a pip too old to read the release
metadata. Verify with `python -V` after activating, before believing any
install error. The car runs 3.11.2; this machine's miniforge is 3.13.

`capture/` is the exception — it runs on the car against `~/env` and deliberately
depends on nothing that is not already there.

## Classes

| id | class   | role                                         |
|----|---------|----------------------------------------------|
| 0  | blue    | left corridor boundary                       |
| 1  | magenta | goal marker                                  |
| 2  | orange  | dead end — the wall across a stub            |
| 3  | red     | junction gate — always placed in pairs       |
| 4  | yellow  | right corridor boundary                      |

The ids are alphabetical by name — the order Roboflow assigns. They carry no
meaning beyond agreeing with the dataset; see the note in `LabeledCone.msg`.

The class *is* the color; left/right are relative to the direction of travel, so
the same corridor driven in reverse still has blue on its own left. Class ids
must match `cone_msgs/msg/LabeledCone.msg` — that file is the source of truth,
and the Roboflow project's class order must be verified against it in the
exported `data.yaml` rather than assumed.

Because color carries the class, **never augment with hue or saturation jitter**
— it teaches the model that blue and yellow are interchangeable.

**Red and orange are the pair that matters.** They are the two most similar
colors on the track and they mean opposite things: a dead end read as a gate
hands the car off to `junction_exec` at a wall, and a gate read as a dead end
misses the junction. Report their per-class numbers separately and look at the
confusion between them specifically — `evaluate.py` calls that pair out by name.

## Pipeline

0. Build and survey the track (`data/layouts/track_v1.md`) — capture happens on
   the course we actually intend to run
1. Drive the track and record with `capture/` (Person C, day 2)
2. Cull with `dataset/prepare_dataset.py`, decide the split in `dataset/splits.json`,
   upload with `dataset/roboflow_upload.py`
3. Hand-label 100–150 images; train v1 with `training/train.py`; propose boxes for
   the rest with `dataset/roboflow_prelabel.py`; correct them in Roboflow (day 3)
4. Re-export with `dataset/roboflow_export.py` — which is where the class order
   and the split are verified — and retrain v2 on the full corrected set (day 4)
5. Check with `training/evaluate.py --split test`, per class, on a held-out session
6. Export: `.pt` → ONNX → OAK-D blob (6 SHAVEs) for on-device inference
7. Trained weights/blobs attach to a GitHub Release, not to git
