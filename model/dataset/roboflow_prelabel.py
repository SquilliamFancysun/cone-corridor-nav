"""Pre-label a session with the v1 weights and upload the boxes for correction.

The model-assisted step from LABELING.md: hand-label ~150 images, train v1 on
those, then let v1 propose boxes for everything else and correct them by hand.
Roboflow's own Label Assist wants the model hosted there; we train locally with
ultralytics, so this runs `best.pt` here and uploads image + annotation
together.

These boxes are *proposals*. They go up tagged `auto-labeled` and into their own
batch so that (a) a labeler knows to look harder at them and (b) the dataset
card can honestly report which fraction of the set was machine-labeled and
corrected — LABELING.md asks for that number, and the corrections are the
interesting part.

    python roboflow_prelabel.py \
        --weights ../training/v1/weights/best.pt --session 20260827_1503_eli1

The workspace and project come from model/roboflow.json; see roboflow_config.py.

`--no-upload` writes the label files and stops, which is also the fastest way to
see whether v1 finds magenta at all, and whether it is telling red and orange
apart, before spending upload quota on it.

Requires the off-car venv: see ../requirements.txt. The CUDA build of torch
makes this pass over a full session a great deal less tedious.
"""

import argparse
import os
import shutil
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cone_classes import resolve_class_names, staged_name  # noqa: E402
from runtime import pick_device  # noqa: E402
from roboflow_upload import (DEFAULT_IMAGES_DIR, DEFAULT_SPLITS_FILE, connect,  # noqa: E402
                             kept_frames, load_splits)
from cone_classes import read_uploaded, record_uploaded  # noqa: E402

PRELABEL_DIRNAME = "_prelabel"


def write_label(result, path, conf):
    """YOLO txt: one `cls xc yc w h` line per box, all normalized.

    Written straight from the boxes rather than via save_txt so the confidence
    floor and the filename are ours — the label file has to be named after the
    staged image or Roboflow will not pair them.
    """
    lines = []
    boxes = result.boxes
    kept_conf = []
    for i in range(len(boxes)):
        score = float(boxes.conf[i])
        if score < conf:
            continue
        cls = int(boxes.cls[i])
        xc, yc, w, h = (float(v) for v in boxes.xywhn[i])
        lines.append(f"{cls} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}")
        kept_conf.append((cls, score))
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("\n".join(lines) + ("\n" if lines else ""))
    return kept_conf


def predict_session(model, session, session_dir, args, class_names):
    paths = kept_frames(session_dir)

    # Frames a human already labelled are not candidates for machine proposals.
    # Without this the hand-label batch comes back a second time as v1's guesses,
    # and the careful work is buried under the cheap work.
    if not args.include_uploaded:
        already = read_uploaded(session_dir)
        if already:
            before = len(paths)
            paths = [p for p in paths if os.path.basename(p) not in already]
            print(f"{session}: skipping {before - len(paths)} frames already "
                  f"uploaded (--include-uploaded to override)")

    if args.limit and len(paths) > args.limit:
        # Evenly spaced, matching roboflow_upload.py. Truncating instead would
        # hand back one contiguous stretch of the drive -- the first 300 frames
        # of a 1600-frame session are one corner in one light, and a batch
        # corrected from that teaches v2 about that corner.
        step = len(paths) / float(args.limit)
        paths = [paths[int(i * step)] for i in range(args.limit)]
    if not paths:
        print(f"{session}: no frames left to pre-label, skipping")
        return None

    out_dir = os.path.join(session_dir, PRELABEL_DIRNAME)
    if os.path.isdir(out_dir) and args.clean:
        shutil.rmtree(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    print(f"\n{session}: predicting on {len(paths)} frames "
          f"(conf >= {args.conf}, device {args.device})")
    per_class = {name: 0 for name in class_names}
    empty = 0
    pairs = []
    for start in range(0, len(paths), args.batch):
        chunk = paths[start:start + args.batch]
        results = model.predict(chunk, imgsz=args.imgsz, conf=args.conf,
                                device=args.device, verbose=False)
        for path, result in zip(chunk, results):
            image_name = staged_name(session, os.path.basename(path))
            label_path = os.path.join(out_dir, os.path.splitext(image_name)[0] + ".txt")
            found = write_label(result, label_path, args.conf)
            if not found:
                empty += 1
            for cls, _ in found:
                if 0 <= cls < len(class_names):
                    per_class[class_names[cls]] += 1
            pairs.append((path, image_name, label_path))
        if args.progress:
            print(f"  ...{min(start + args.batch, len(paths))}/{len(paths)}",
                  file=sys.stderr)

    total = sum(per_class.values())
    print(f"  {total} boxes over {len(paths)} frames "
          f"({total / float(len(paths)):.1f} per frame); {empty} frames with none")
    for name in class_names:
        print(f"    {name:<7} {per_class[name]:>6}")
    for name in class_names:
        if per_class[name] == 0:
            print(f"  WARNING: v1 proposed no {name} at all here. Either this "
                  f"session has none,\n           or {name} is under-trained — "
                  "check before trusting these boxes.")
    return pairs


def upload_pairs(project, session, split, pairs, args, class_names):
    labelmap = {i: name for i, name in enumerate(class_names)}
    batch = session + args.batch_suffix
    tags = list(args.tag) + [session, os.path.basename(os.path.dirname(
        os.path.dirname(os.path.abspath(args.weights))))]
    print(f"  uploading to batch {batch!r}, split {split}")
    ok, failed = 0, []
    with tempfile.TemporaryDirectory(prefix="rf-prelabel-") as stage_dir:
        for i, (path, image_name, label_path) in enumerate(pairs, start=1):
            link = os.path.join(stage_dir, image_name)
            try:
                os.symlink(os.path.abspath(path), link)
            except (OSError, NotImplementedError):
                shutil.copy2(path, link)
            kwargs = dict(image_path=link, annotation_path=label_path,
                          batch_name=batch, split=split, tag_names=tags,
                          num_retry_uploads=args.retries)
            try:
                try:
                    project.single_upload(annotation_labelmap=labelmap, **kwargs)
                except TypeError:
                    # Older SDK: no labelmap parameter. The txt carries bare class
                    # indices, so Roboflow reads them against the project's own
                    # class order — which is exactly the permutation risk this
                    # repo keeps warning about. Verify in the app before labeling.
                    project.single_upload(**kwargs)
                    if i == 1:
                        print("  note: SDK has no annotation_labelmap; class indices "
                              "are being read\n        against the project's class "
                              "order. Spot-check one image in Roboflow.")
                ok += 1
            except Exception as exc:
                failed.append((image_name, str(exc).strip()))
            if i % 25 == 0 or i == len(pairs):
                print(f"  ...{i}/{len(pairs)} uploaded, {len(failed)} failed",
                      file=sys.stderr)
    for name, err in failed[:5]:
        print(f"  FAILED {name}: {err}")
    if len(failed) > 5:
        print(f"  ... and {len(failed) - 5} more failures")
    return ok, failed


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--weights", required=True, help="v1 .pt to propose boxes with")
    parser.add_argument("--session", action="append", required=True,
                        help="session to pre-label; repeatable")
    parser.add_argument("--images-dir", default=DEFAULT_IMAGES_DIR)
    parser.add_argument("--splits-file", default=DEFAULT_SPLITS_FILE)
    parser.add_argument("--workspace", default=None)
    parser.add_argument("--project", default=None)
    parser.add_argument("--api-key", default=None, help="default: $ROBOFLOW_API_KEY")
    parser.add_argument("--conf", type=float, default=0.20,
                        help="confidence floor for a proposed box (default: 0.20 — "
                             "lower than inference on purpose; a spurious box is "
                             "faster to delete than a missing one is to draw)")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default=None, help="default: cuda, else mps, else cpu")
    parser.add_argument("--batch", type=int, default=16, help="frames per predict call")
    parser.add_argument("--limit", type=int, default=None,
                        help="at most N frames per session, evenly spaced across "
                             "the drive — same sampling as roboflow_upload.py")
    parser.add_argument("--include-uploaded", action="store_true",
                        help="also propose on frames already uploaded; by default "
                             "those are skipped, since a human has labelled them")
    parser.add_argument("--batch-suffix", default="-auto",
                        help="appended to the session name for the Roboflow batch")
    parser.add_argument("--tag", action="append", default=["auto-labeled"])
    parser.add_argument("--retries", type=int, default=3)
    parser.add_argument("--progress", action="store_true",
                        help="print prediction progress to stderr")
    parser.add_argument("--clean", action="store_true",
                        help="delete an existing _prelabel/ first")
    parser.add_argument("--no-upload", action="store_true",
                        help="write label files, upload nothing")
    args = parser.parse_args(argv)

    if not os.path.exists(args.weights):
        raise SystemExit(f"error: no weights at {args.weights}")
    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit(
            "error: needs ultralytics.\n"
            "       pip install -r ../requirements.txt  (torch first — see that file)"
        )

    args.device = pick_device(args.device)
    model = YOLO(args.weights)
    class_names, source = resolve_class_names()

    trained = getattr(model, "names", None) or {}
    trained_names = [str(trained[k]).lower() for k in sorted(trained, key=int)] \
        if isinstance(trained, dict) else [str(n).lower() for n in trained]
    if trained_names and tuple(trained_names) != tuple(class_names):
        raise SystemExit(
            f"error: {args.weights} was trained on {trained_names}, but "
            f"{os.path.basename(source)}\n       says {list(class_names)}. Uploading "
            "these boxes would scramble the classes."
        )

    splits = load_splits(args.splits_file)
    images_dir = os.path.expanduser(args.images_dir)
    project = None if args.no_upload else connect(args)

    total_ok, total_failed = 0, 0
    for session in args.session:
        session_dir = os.path.join(images_dir, session)
        if not os.path.isdir(session_dir):
            raise SystemExit(f"error: no such session directory: {session_dir}")
        split = splits.get(session)
        if split is None and not args.no_upload:
            raise SystemExit(f"error: {session} has no split in "
                             f"{os.path.basename(args.splits_file)}")
        pairs = predict_session(model, session, session_dir, args, class_names)
        if pairs and not args.no_upload:
            ok, failed = upload_pairs(project, session, split, pairs, args, class_names)
            total_ok += ok
            total_failed += len(failed)
            if ok:
                # Proposals are uploads too. Without this the next round -- v2
                # proposing over what v1 already covered -- would land a second
                # set of machine boxes on frames a human has just finished
                # correcting, which is the same burial this skip exists to stop.
                failed_names = {name for name, _ in failed}
                sent = [p for p, image_name, _ in pairs
                        if image_name not in failed_names]
                total = record_uploaded(session_dir, sent, split)
                print(f"  recorded {len(sent)} frames in {session}/uploaded.json "
                      f"({total} total)")

    print("\n" + "=" * 68)
    if args.no_upload:
        print(f"Label files written to <session>/{PRELABEL_DIRNAME}/. Nothing uploaded.")
        return 0
    print(f"uploaded {total_ok}, failed {total_failed}")
    print("\nThese are proposals: correct them in Roboflow, then record the "
          "auto-labeled\nfraction in DATASET_CARD.md under Labeling.")
    return 1 if total_failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
