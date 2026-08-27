"""Train the cone detector, with the project's constraints wired in rather than
left to whoever types the command.

Four of them, and each has already cost someone a run somewhere:

* **No hue or saturation jitter, ever.** Color *is* the class here — blue is
  "left boundary" and yellow is "right boundary" and nothing else distinguishes
  them. Ultralytics defaults to hsv_h=0.015, hsv_s=0.7, which teaches the model
  that a blue cone is a yellow cone in different light. There is no flag to
  turn that back on; edit this file and explain yourself in the commit message.
* **Class order checked against `cone_msgs/msg/LabeledCone.msg`** before a
  single epoch runs, because a permutation costs a whole training run.
* **Nano by default, 640 by default.** The Myriad X on the OAK-D runs the
  exported blob; anything larger than yolov8n does not fit the budget.
* **Provenance written next to the weights** — commit, dataset export, library
  versions. The curves are deliverable D2 and an unattributable curve is not.

    uv run --with ultralytics python train.py \
        --data ../dataset/export/cone-v3/data.yaml --name v1

    uv run --with ultralytics python train.py --data ... --name v1 --dry-run

Horizontal flip stays on: the class is the cone's color, and left/right corridor
semantics are resolved in cone_nav, not by the detector.

Requires: ultralytics.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cone_classes import check_order, load_data_yaml  # noqa: E402
from runtime import dirty_worktree, git_commit, pick_device, sha256, versions  # noqa: E402
from evaluate import class_warnings, format_rows, per_class_rows  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))

# Not exposed as flags. See the docstring.
COLOR_SAFE_AUGMENTATION = {
    "hsv_h": 0.0,   # hue jitter would make blue and yellow interchangeable
    "hsv_s": 0.0,   # so would saturation jitter
    "degrees": 0.0,  # cones sit on the ground; the camera does not roll
    "flipud": 0.0,
    "fliplr": 0.5,   # safe: color carries the class, not the side of the frame
}

# The blob export path only has these two shapes benchmarked on-car.
BLOB_IMGSZ = (416, 640)


def check_dataset(data_path):
    if not os.path.exists(data_path):
        raise SystemExit(f"error: no data.yaml at {data_path}")
    names, doc = load_data_yaml(data_path)
    print("class order: " + ", ".join(f"{i}={n}" for i, n in enumerate(names)))
    mismatch = check_order(names)
    if mismatch:
        raise SystemExit(
            "error: " + mismatch
            + "\n\nRun model/dataset/roboflow_export.py, which fails on this before"
              "\nit ever reaches training."
        )
    print("matches cone_msgs/msg/LabeledCone.msg — ok")
    for key in ("train", "val"):
        if key not in doc:
            raise SystemExit(f"error: {data_path} has no '{key}' split")
    if "test" not in doc:
        print("warning: no test split in data.yaml. The held-out session is the only "
              "honest\n         number this project produces — see DATASET_CARD.md.")
    return names


def build_config(args, class_names):
    config = {
        "model": args.model,
        "data": os.path.abspath(args.data),
        "imgsz": args.imgsz,
        "epochs": args.epochs,
        "batch": args.batch,
        "patience": args.patience,
        "seed": args.seed,
        "workers": args.workers,
        "device": args.device,
        "project": os.path.abspath(args.project),
        "name": args.name,
        # Always true: stamp_provenance() has already created this directory, and
        # without it ultralytics would sidestep into <name>2. Collisions are
        # caught by the explicit check in main() instead, which can explain itself.
        "exist_ok": True,
        "pretrained": True,
        "val": True,
        "plots": True,
        "hsv_v": args.hsv_v,
        "mosaic": args.mosaic,
        "close_mosaic": args.close_mosaic,
        "scale": args.scale,
        "translate": args.translate,
    }
    config.update(COLOR_SAFE_AUGMENTATION)
    return config


def dataset_manifest(data_path):
    """The committed export manifest, but only if it describes *this* export.

    Attaching the manifest of some other export would be worse than attaching
    none: it would look like provenance and be wrong.
    """
    path = os.path.normpath(os.path.join(HERE, "..", "dataset", "labels", "manifest.json"))
    try:
        with open(path) as fh:
            manifest = json.load(fh)
    except (OSError, ValueError):
        return None
    location = os.path.abspath(manifest.get("location", ""))
    if location != os.path.dirname(os.path.abspath(data_path)):
        return None
    return manifest


def stamp_provenance(run_dir, config, args, class_names):
    """Everything needed to say what produced these weights, written before the
    run rather than after, so a crashed run still leaves the record behind."""
    os.makedirs(run_dir, exist_ok=True)
    stamp = {
        "config": config,
        "class_names": list(class_names),
        "data_yaml_sha256": sha256(args.data),
        "git_commit": git_commit(),
        "git_dirty": dirty_worktree(),
        "versions": versions(),
        "augmentation_note": (
            "hue and saturation jitter are pinned to 0: color is the class signal"
        ),
    }
    stamp["dataset_manifest"] = dataset_manifest(args.data)
    if stamp["dataset_manifest"] is None:
        print("note: no matching model/dataset/labels/manifest.json — this run will "
              "not be\n      traceable to a Roboflow version. Export with "
              "roboflow_export.py to get one.")
    path = os.path.join(run_dir, "train_config.json")
    with open(path, "w") as fh:
        json.dump(stamp, fh, indent=2, sort_keys=True)
        fh.write("\n")
    if stamp["git_dirty"]:
        print(f"warning: uncommitted changes — {stamp['git_commit']} does not fully "
              "describe\n         the code that produced this run.")
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--data", required=True, help="data.yaml from roboflow_export.py")
    parser.add_argument("--name", default="v1", help="run name (default: v1)")
    parser.add_argument("--model", default="yolov8n.pt",
                        help="default: yolov8n.pt — nano is effectively required "
                             "for the Myriad X")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch", type=int, default=16,
                        help="-1 lets ultralytics pick from available memory")
    parser.add_argument("--patience", type=int, default=20)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--device", default=None, help="default: cuda, else mps, else cpu")
    parser.add_argument("--project", default=HERE,
                        help="default: model/training (D2 wants the curves in git)")
    parser.add_argument("--exist-ok", action="store_true",
                        help="write into an existing run directory")
    parser.add_argument("--resume", action="store_true", help="resume from last.pt")
    parser.add_argument("--hsv-v", type=float, default=0.2,
                        help="brightness jitter (default: 0.2). Value only — hue and "
                             "saturation stay at 0")
    parser.add_argument("--mosaic", type=float, default=1.0)
    parser.add_argument("--close-mosaic", type=int, default=10,
                        help="epochs at the end with mosaic off (default: 10)")
    parser.add_argument("--scale", type=float, default=0.5)
    parser.add_argument("--translate", type=float, default=0.1)
    parser.add_argument("--dry-run", action="store_true",
                        help="check the dataset and print the config; train nothing")
    args = parser.parse_args(argv)

    class_names = check_dataset(args.data)
    args.device = pick_device(args.device)
    config = build_config(args, class_names)

    if "yolov8n" not in os.path.basename(args.model):
        print(f"warning: {args.model} is not yolov8n. The OAK-D's Myriad X runs the "
              "exported\n         blob; anything larger has not been shown to fit "
              "the control loop.")
    if args.imgsz not in BLOB_IMGSZ:
        print(f"warning: imgsz {args.imgsz} is neither 416 nor 640 — those are the two "
              "shapes\n         the blob export and on-car benchmark cover.")
    if args.device == "cpu":
        print("warning: training on CPU. Fine for a smoke test with --epochs 1; "
              "use Colab's T4\n         for a real run.")

    print("\nconfig:")
    for key in sorted(config):
        print(f"  {key}: {config[key]}")
    print("\nhue and saturation jitter are pinned to 0 — color is the class signal.")

    run_dir = os.path.join(config["project"], args.name)
    if args.dry_run:
        print(f"\n(dry run — would train into {run_dir})")
        return 0
    if os.path.exists(run_dir) and not (args.exist_ok or args.resume):
        raise SystemExit(
            f"error: {run_dir} already exists. Pick another --name, or pass "
            "--exist-ok\n       to overwrite it. Ultralytics would silently write "
            f"to {args.name}2 instead,\n       which is how two runs end up with the "
            "same name in the report."
        )

    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit(
            "error: needs ultralytics.\n"
            "       uv run --with ultralytics python train.py --data ... --name ..."
        )

    stamp = stamp_provenance(run_dir, config, args, class_names)
    print(f"provenance -> {stamp}")

    model = YOLO(args.model)
    if args.resume:
        model = YOLO(os.path.join(run_dir, "weights", "last.pt"))
        model.train(resume=True)
    else:
        model.train(**config)

    print("\n" + "=" * 68)
    metrics = model.val(data=os.path.abspath(args.data), split="val",
                        imgsz=args.imgsz, device=args.device, verbose=False,
                        project=run_dir, name="val", exist_ok=True)
    rows = per_class_rows(metrics, list(class_names))
    print("Validation, per class — report these, not the average:\n")
    print("\n".join(format_rows(rows)))
    notes = class_warnings(rows)
    if notes:
        print()
        for note in notes:
            print(f"  * {note}")

    print("\n" + "=" * 68)
    print(f"run -> {run_dir}")
    print("Commit results.csv, args.yaml, train_config.json and the curves. Weights\n"
          "are gitignored — attach best.pt to a GitHub Release.\n")
    print("Then the number that actually counts, on the held-out session:\n")
    print(f"  uv run --with ultralytics python evaluate.py \\\n"
          f"      --weights {os.path.join(run_dir, 'weights', 'best.pt')} \\\n"
          f"      --data {os.path.abspath(args.data)} --split test")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
