"""Validate a trained detector and report the numbers that actually decide things.

Ultralytics prints a per-class table and moves on. What D2 and D3 need out of
it, and what this script produces:

* **Per-class mAP50-95, never just the average.** Red and orange are the pair
  most likely to confuse — nearest colors on the track, opposite meanings — and
  magenta has the fewest instances; one averaged number hides both failures.
* **The confusion matrix read out in words** — specifically orange-vs-yellow
  both ways, and how much of each class is being missed entirely.
* **A qualitative sweep over a session that was never trained on** (`--images`),
  which is the end-of-LABELING.md check: do all five classes separate, is
  magenta found at all.

Everything lands in a markdown report next to the weights, because the report
is a deliverable and reconstructing these numbers a week later is not fun.

    python evaluate.py \
        --weights v1/weights/best.pt --data ../dataset/export/cone-v3/data.yaml

    python evaluate.py --weights v1/weights/best.pt \
        --data ... --images ../dataset/images/20260827_1053_lot-sun-A/frames

Requires the off-car venv: see ../requirements.txt.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cone_classes import check_order, load_data_yaml, session_of  # noqa: E402
from runtime import pick_device  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def tidy_path(path):
    """Relative when that is shorter and readable, absolute when it escapes the tree."""
    relative = os.path.relpath(path, HERE)
    return path if relative.startswith("..") and relative.count("..") > 2 else relative

# Pairs worth calling out by name, and why they are the ones that bite. Read as
# (true class, predicted class): "a real X the model called Y".
WATCH_PAIRS = (
    ("orange", "red", "a dead end read as a gate hands off to junction_exec at a "
                      "wall — the worst confusion on the track"),
    ("red", "orange", "a gate read as a dead end misses the junction entirely"),
    ("orange", "yellow", "warm low-angle sun washes orange toward yellow; a dead "
                         "end read as a boundary is a wall the car drives into"),
    ("magenta", "red", "the goal read as a gate — the car passes the finish"),
)


def load_model(weights):
    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit(
            "error: needs ultralytics.\n"
            "       pip install -r ../requirements.txt  (torch first — see that file)"
        )
    if not os.path.exists(weights):
        raise SystemExit(f"error: no weights at {weights}")
    return YOLO(weights)


def model_class_names(model):
    names = getattr(model, "names", None) or {}
    if isinstance(names, dict):
        return [str(names[k]).lower() for k in sorted(names, key=int)]
    return [str(n).lower() for n in names]


def per_class_rows(metrics, class_names):
    """One row per class: precision, recall, mAP50, mAP50-95, instances.

    Classes with no instances in the split come back with map=None rather than
    a zero — a zero reads like "the model failed", and the truth is "nothing
    was measured", which is a different problem with a different fix.
    """
    box = metrics.box
    index = getattr(box, "ap_class_index", None)
    # `or []` would raise here: these are numpy arrays, not lists.
    present = [] if index is None else [int(c) for c in index]
    nt = getattr(metrics, "nt_per_class", None)
    rows = []
    for cls, name in enumerate(class_names):
        instances = None
        if nt is not None and cls < len(nt):
            instances = int(nt[cls])
        if cls in present:
            i = present.index(cls)
            rows.append({
                "cls": cls, "name": name, "instances": instances,
                "p": float(box.p[i]), "r": float(box.r[i]),
                "map50": float(box.ap50[i]), "map": float(box.ap[i]),
            })
        else:
            rows.append({"cls": cls, "name": name, "instances": instances,
                         "p": None, "r": None, "map50": None, "map": None})
    return rows


def format_rows(rows):
    lines = [f"{'class':<9}{'inst':>7}{'P':>9}{'R':>9}{'mAP50':>9}{'mAP50-95':>10}"]

    def cell(value, width, fmt="{:.3f}"):
        return ("{:>%d}" % width).format("—" if value is None else fmt.format(value))

    for row in rows:
        lines.append(
            f"{row['name']:<9}"
            + cell(row["instances"], 7, "{:d}")
            + cell(row["p"], 9) + cell(row["r"], 9)
            + cell(row["map50"], 9) + cell(row["map"], 10)
        )
    measured = [r["map"] for r in rows if r["map"] is not None]
    if measured:
        lines.append(f"{'mean':<9}{'':>7}{'':>9}{'':>9}{'':>9}"
                     f"{sum(measured) / len(measured):>10.3f}")
    return lines


def class_warnings(rows, weakest_gap=0.15):
    """The per-class reading an averaged mAP would have hidden."""
    notes = []
    measured = [r for r in rows if r["map"] is not None]
    for row in rows:
        if row["map"] is None:
            notes.append(f"{row['name']}: no instances in this split — this class was "
                         "not evaluated at all. Nothing here says whether it works.")
        elif row["instances"] is not None and row["instances"] < 30:
            plural = "" if row["instances"] == 1 else "s"
            notes.append(f"{row['name']}: only {row['instances']} instance{plural}; its "
                         "mAP moves a lot on a handful of boxes.")
    if len(measured) > 1:
        best = max(measured, key=lambda r: r["map"])
        worst = min(measured, key=lambda r: r["map"])
        if best["map"] - worst["map"] > weakest_gap:
            notes.append(
                f"{worst['name']} trails {best['name']} by "
                f"{best['map'] - worst['map']:.3f} mAP50-95 "
                f"({worst['map']:.3f} vs {best['map']:.3f}). The average is "
                "carrying it.")
    for row in measured:
        if row["r"] is not None and row["r"] < 0.7:
            notes.append(f"{row['name']}: recall {row['r']:.2f} — roughly "
                         f"{(1 - row['r']) * 100:.0f}% of them are being missed. "
                         "Downstream that is a gap in the corridor, not a wrong label.")
    return notes


def confusion_notes(metrics, class_names):
    """Read the confusion matrix out loud.

    Ultralytics orients it [predicted, true] with a trailing background row and
    column, so matrix[p][t] is 'true class t called p'. The background column is
    a false positive out of nowhere; the background row is a miss.
    """
    matrix = getattr(metrics, "confusion_matrix", None)
    matrix = getattr(matrix, "matrix", None)
    if matrix is None:
        return ["confusion matrix unavailable from this ultralytics version — "
                "the per-class table above still stands."], None
    try:
        grid = [[float(v) for v in row] for row in matrix]
    except TypeError:
        return ["confusion matrix could not be read."], None
    n = len(class_names)
    if len(grid) < n:
        return ["confusion matrix has an unexpected shape."], None
    has_background = len(grid) > n

    notes = []
    for cls, name in enumerate(class_names):
        total = sum(grid[p][cls] for p in range(len(grid)))
        if total <= 0:
            continue
        wrong = [(grid[p][cls], class_names[p]) for p in range(n) if p != cls]
        wrong = [(count, other) for count, other in wrong if count > 0]
        missed = grid[n][cls] if has_background else 0.0
        parts = []
        if wrong:
            count, other = max(wrong)
            parts.append(f"{count:.0f} called {other} ({count / total * 100:.0f}%)")
        if missed > 0:
            parts.append(f"{missed:.0f} missed entirely "
                         f"({missed / total * 100:.0f}%)")
        if parts:
            notes.append(f"{name} ({total:.0f} true): " + "; ".join(parts))

    for true_name, pred_name, why in WATCH_PAIRS:
        if true_name not in class_names or pred_name not in class_names:
            continue
        t, p = class_names.index(true_name), class_names.index(pred_name)
        count = grid[p][t]
        total = sum(grid[q][t] for q in range(len(grid)))
        if total > 0 and count > 0:
            notes.append(f"WATCH {true_name} -> {pred_name}: {count:.0f} of "
                         f"{total:.0f} ({count / total * 100:.0f}%) — {why}")
    return notes, grid


def qualitative(model, images_dir, class_names, args):
    """Detection counts and confidence spread on frames with no ground truth.

    Deliberately not a metric. It answers the question LABELING.md ends on —
    do the four classes separate on a session the model never saw — for footage
    that was never labeled, which is most of what exists.
    """
    exts = (".jpg", ".jpeg", ".png")
    paths = []
    for root, _, files in os.walk(images_dir):
        if os.path.basename(root) in ("_rejected", "_prelabel"):
            continue
        paths.extend(sorted(os.path.join(root, f) for f in files
                            if f.lower().endswith(exts)))
    if not paths:
        raise SystemExit(f"error: no images under {images_dir}")
    if args.limit:
        step = max(1, len(paths) // args.limit)
        paths = paths[::step][:args.limit]

    print(f"\nqualitative sweep: {len(paths)} frames from {images_dir}")
    per_class = {name: [] for name in class_names}
    per_session = {}
    empty = 0
    save_dir = os.path.join(args.out_dir, "qualitative")
    saved = 0
    for start in range(0, len(paths), args.batch):
        chunk = paths[start:start + args.batch]
        results = model.predict(chunk, imgsz=args.imgsz, conf=args.conf,
                                device=args.device, verbose=False)
        for path, result in zip(chunk, results):
            session = session_of(path) or os.path.basename(os.path.dirname(
                os.path.dirname(path)))
            counts = per_session.setdefault(session, {n: 0 for n in class_names})
            boxes = result.boxes
            if len(boxes) == 0:
                empty += 1
            for i in range(len(boxes)):
                cls = int(boxes.cls[i])
                if 0 <= cls < len(class_names):
                    per_class[class_names[cls]].append(float(boxes.conf[i]))
                    counts[class_names[cls]] += 1
            if saved < args.save_samples:
                os.makedirs(save_dir, exist_ok=True)
                result.save(filename=os.path.join(save_dir, os.path.basename(path)))
                saved += 1

    lines = [f"{'class':<9}{'boxes':>8}{'conf p5':>10}{'median':>9}{'p95':>9}"]
    for name in class_names:
        scores = sorted(per_class[name])
        if not scores:
            lines.append(f"{name:<9}{0:>8}{'—':>10}{'—':>9}{'—':>9}")
            continue

        def at(fraction):
            return scores[min(len(scores) - 1, int(fraction * len(scores)))]

        lines.append(f"{name:<9}{len(scores):>8}{at(0.05):>10.2f}"
                     f"{at(0.5):>9.2f}{at(0.95):>9.2f}")
    lines.append(f"\n{empty} of {len(paths)} frames had no detection at all "
                 f"(conf >= {args.conf})")
    for name in class_names:
        if not per_class[name]:
            lines.append(f"WARNING: {name} was never detected here. If this session "
                         f"contains {name} cones, that is the finding.")
    if len(per_session) > 1:
        lines.append("\nper session:")
        lines.append(f"{'session':<32}" + "".join(f"{n:>9}" for n in class_names))
        for session in sorted(per_session):
            lines.append(f"{session:<32}"
                         + "".join(f"{per_session[session][n]:>9}"
                                   for n in class_names))
    if saved:
        lines.append(f"\n{saved} annotated samples -> {save_dir}")
    return lines


def write_report(path, sections):
    with open(path, "w") as fh:
        for heading, body in sections:
            if heading:
                fh.write(f"\n## {heading}\n\n")
            if isinstance(body, list):
                fh.write("```\n" + "\n".join(body) + "\n```\n")
            else:
                fh.write(body.rstrip() + "\n")
    print(f"\nreport -> {path}")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--weights", required=True)
    parser.add_argument("--data", default=None,
                        help="data.yaml; omit only with --images (no metrics then)")
    parser.add_argument("--split", default="test", choices=("train", "val", "test"),
                        help="default: test — the split no training run has seen")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", default=None, help="default: cuda, else mps, else cpu")
    parser.add_argument("--conf", type=float, default=0.25,
                        help="confidence floor for the qualitative sweep (default: 0.25)")
    parser.add_argument("--iou", type=float, default=0.6, help="NMS IoU for validation")
    parser.add_argument("--images", default=None,
                        help="directory of unlabeled frames for a qualitative sweep")
    parser.add_argument("--limit", type=int, default=200,
                        help="max frames in the qualitative sweep (default: 200)")
    parser.add_argument("--save-samples", type=int, default=12,
                        help="annotated frames to write out (default: 12)")
    parser.add_argument("--out-dir", default=None,
                        help="default: the run directory two levels above the weights")
    parser.add_argument("--report", default=None, help="default: <out-dir>/report_<split>.md")
    args = parser.parse_args(argv)

    args.device = pick_device(args.device)
    model = load_model(args.weights)
    class_names = model_class_names(model)
    if not class_names:
        raise SystemExit(f"error: {args.weights} carries no class names")

    mismatch = check_order(class_names)
    if mismatch:
        raise SystemExit("error: " + mismatch + "\n\nThese weights cannot be deployed "
                         "as they are; the ROS side reads the ids literally.")

    args.out_dir = args.out_dir or os.path.dirname(
        os.path.dirname(os.path.abspath(args.weights)))
    os.makedirs(args.out_dir, exist_ok=True)

    sections = [(None, f"# Evaluation — {tidy_path(os.path.abspath(args.weights))}\n\n"
                       f"- split: `{args.split}`\n"
                       f"- data: `{args.data}`\n"
                       f"- device: `{args.device}`, imgsz {args.imgsz}\n")]

    if args.data:
        dataset_names, _ = load_data_yaml(args.data)
        mismatch = check_order(dataset_names)
        if mismatch:
            raise SystemExit("error: " + mismatch)
        print(f"validating on the {args.split} split, device {args.device}")
        metrics = model.val(data=args.data, split=args.split, imgsz=args.imgsz,
                            batch=args.batch, device=args.device, iou=args.iou,
                            plots=True, verbose=False, project=args.out_dir,
                            name=f"val_{args.split}", exist_ok=True)
        rows = per_class_rows(metrics, class_names)
        table = format_rows(rows)
        print("\n" + "\n".join(table))
        sections.append((f"Per-class metrics ({args.split})", table))

        notes = class_warnings(rows)
        if notes:
            print("\n" + "\n".join(f"  * {n}" for n in notes))
            sections.append(("What the average hides",
                             "\n".join(f"- {n}" for n in notes)))

        confusion, _ = confusion_notes(metrics, class_names)
        if confusion:
            print("\nconfusion:")
            print("\n".join(f"  * {n}" for n in confusion))
            sections.append(("Confusion", "\n".join(f"- {n}" for n in confusion)))
    elif not args.images:
        raise SystemExit("error: pass --data (metrics), --images (qualitative), or both")

    if args.images:
        lines = qualitative(model, args.images, class_names, args)
        print("\n" + "\n".join(lines))
        sections.append((f"Qualitative sweep — {os.path.basename(args.images.rstrip('/'))}",
                         lines))

    report = args.report or os.path.join(args.out_dir, f"report_{args.split}.md")
    write_report(report, sections)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
