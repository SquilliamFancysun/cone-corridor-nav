# Training (D2)

Two scripts and the runs they produce. The protocol they implement — why nano,
why 640, why no hue jitter — is in
[`../dataset/LABELING.md`](../dataset/LABELING.md); this file is how to run it.

```
train.py      yolov8n training with the project's constraints wired in
evaluate.py   per-class metrics, confusion read-out, qualitative sweep
<name>/       one directory per run: curves, results.csv, train_config.json
```

## Run it

```bash
cd model/training
uv run --with ultralytics python train.py \
    --data ../dataset/export/<project>-v<N>/data.yaml --name v1
```

`--dry-run` checks the dataset and prints the resolved config without training —
worth doing before booking a GPU. Device is picked automatically: CUDA, else
MPS, else CPU. Colab's T4 is the recommended runner; `--device mps` on the Mac
is a smoke test, not a run.

Then the number that counts, on a session no training run has seen:

```bash
uv run --with ultralytics python evaluate.py \
    --weights v1/weights/best.pt \
    --data ../dataset/export/<project>-v<N>/data.yaml --split test
```

`--images <dir>` adds a qualitative sweep over unlabeled frames — detection
counts and confidence spread per class, per session, plus a few annotated
samples. That is the end-of-LABELING.md check: do all five classes separate on
footage the model never saw, is magenta found at all, and are red and orange
told apart at range.

Both write a markdown report into the run directory.

## What the scripts refuse to do

- **Hue or saturation jitter.** Ultralytics defaults to `hsv_h=0.015,
  hsv_s=0.7`, which teaches the model that a blue cone is a yellow cone in
  different light. Color *is* the class here. There is no flag; changing it
  means editing `COLOR_SAFE_AUGMENTATION` in `train.py` and saying why in the
  commit message. `--hsv-v` (brightness) is exposed, because brightness is not
  the class signal.
- **Train on a dataset whose class order disagrees with
  `cone_msgs/msg/LabeledCone.msg`.** Checked before the first epoch, and again
  against the weights' own names in `evaluate.py`.
- **Silently sidestep an existing run directory.** Ultralytics would write to
  `v1_2`; this errors and makes you choose a name, because two runs called
  something like `v1` in a report is a mess nobody untangles later.

## What goes in git

`results.csv`, `args.yaml`, `train_config.json`, the curves, and the reports —
D2 wants the curves, and `train_config.json` is what makes a curve
attributable: commit, dataset export sha, ultralytics and torch versions, the
full resolved config, and the dataset manifest if the export came from
`roboflow_export.py`.

Weights do not: `*.pt` is gitignored. Attach `best.pt` to a GitHub Release, then
convert it to a `.blob` for the OAK-D — see the export section of
[`../dataset/LABELING.md`](../dataset/LABELING.md).
