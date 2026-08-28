"""Download a labeled Roboflow version and check it before anyone trains on it.

Downloading is the easy part. The checks are the point:

* **Class order** against `cone_msgs/msg/LabeledCone.msg`. LABELING.md says to
  verify this in the exported data.yaml rather than assume it; this is that
  verification, and it is fatal.
* **Session leakage.** Uploads are named `<session>__<frame>`, so the exported
  filenames still say where each image came from. If one session's frames show
  up in two splits, the split was assigned per image somewhere and the held-out
  numbers are inflated.
* **Class balance and box sizes**, printed in the shape DATASET_CARD.md asks
  for, so the card gets filled in from measurements instead of memory.

Images stay out of git; labels, the class list and the manifest are synced into
`model/dataset/labels/`, which is committed.

    export ROBOFLOW_API_KEY=...
    python roboflow_export.py --version 3

The workspace and project come from model/roboflow.json; --workspace / --project
and $ROBOFLOW_WORKSPACE / $ROBOFLOW_PROJECT still override it. See
roboflow_config.py.

Already have the export on disk?

    python roboflow_export.py --location export/cone-3 --no-download

Requires the off-car venv: see ../requirements.txt. Only the roboflow SDK is
needed to download; --no-download runs on pyyaml and pillow alone.
"""

import argparse
import datetime
import json
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cone_classes import (SPLITS, check_order, load_data_yaml,  # noqa: E402
                          resolve_class_names, session_of)

import roboflow_config  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_EXPORT_DIR = os.path.join(HERE, "export")
DEFAULT_LABELS_DIR = os.path.join(HERE, "labels")
IMAGE_EXTS = (".jpg", ".jpeg", ".png")

# The box protocol in LABELING.md: skip boxes under ~8 px.
MIN_BOX_PX = 8
# The card's rule of thumb: keep the rare classes within ~3:1 of the common ones.
MAX_IMBALANCE = 3.0


def download(args):
    try:
        from roboflow import Roboflow
    except ImportError:
        raise SystemExit(
            "error: needs the roboflow SDK to download.\n"
            "       pip install -r ../requirements.txt\n"
            "       (or pass --location <dir> --no-download for an export you already have)"
        )
    key = args.api_key or os.environ.get("ROBOFLOW_API_KEY")
    if not key:
        raise SystemExit("error: no API key. export ROBOFLOW_API_KEY=...")
    workspace, project_id = roboflow_config.slugs(args)
    if not args.version:
        raise SystemExit("error: --version is required (the Roboflow version number)")

    location = args.location or os.path.join(
        DEFAULT_EXPORT_DIR, f"{project_id}-v{args.version}")
    print(f"downloading {workspace}/{project_id} v{args.version} ({args.format}) "
          f"-> {location}")
    version = (Roboflow(api_key=key)
               .workspace(workspace)
               .project(project_id)
               .version(int(args.version)))
    dataset = version.download(args.format, location=location, overwrite=True)
    return getattr(dataset, "location", location)


def yaml_key(split):
    """Ultralytics reads the validation split from `val:`; the directory is `valid/`."""
    return "val" if split == "valid" else split


def split_dirs(location, split):
    base = os.path.join(location, split)
    return os.path.join(base, "images"), os.path.join(base, "labels")


def list_files(directory, exts=None):
    if not os.path.isdir(directory):
        return []
    return sorted(
        os.path.join(directory, n) for n in os.listdir(directory)
        if exts is None or n.lower().endswith(exts)
    )


def image_size(path):
    try:
        from PIL import Image
    except ImportError:
        return None
    try:
        with Image.open(path) as img:
            return img.size
    except OSError:
        return None


def read_boxes(label_path):
    """[(cls, w, h)] from a YOLO txt, box rows and polygon rows alike.

    Roboflow exports a segmentation-style row -- `cls x1 y1 x2 y2 ...` -- when the
    annotations are polygons, which is what its Smart Polygon tool produces even
    in a detection project. Reading parts[3:5] as a width and height there is not
    a crash, it is a wrong number: those are the second vertex's coordinates. The
    box-size percentiles this feeds go into DATASET_CARD.md, so a silently wrong
    one is worse than a refusal.

    Polygons are reduced to their bounding extents, which is exactly what
    ultralytics does when it trains a detector on segment labels -- so the numbers
    reported here describe the boxes the model will actually be trained on.

    The set of row kinds comes back too, because a file holding both kinds is one
    ultralytics will not load at all. See report().
    """
    boxes, bad, kinds = [], 0, set()
    with open(label_path, encoding="utf-8") as fh:
        for line in fh:
            parts = line.split()
            if not parts:
                continue
            try:
                cls = int(float(parts[0]))
                if len(parts) == 5:
                    boxes.append((cls, float(parts[3]), float(parts[4])))
                    kinds.add("box")
                elif len(parts) > 5 and len(parts) % 2 == 1:
                    # cls + an even number of coordinates: a polygon.
                    xs = [float(v) for v in parts[1::2]]
                    ys = [float(v) for v in parts[2::2]]
                    boxes.append((cls, max(xs) - min(xs), max(ys) - min(ys)))
                    kinds.add("segment")
                else:
                    bad += 1
            except ValueError:
                bad += 1
    return boxes, bad, kinds


def scan_split(location, split, class_names):
    images_dir, labels_dir = split_dirs(location, split)
    images = list_files(images_dir, IMAGE_EXTS)
    labels = list_files(labels_dir, (".txt",))
    if not images and not labels:
        return None

    dims = image_size(images[0]) if images else None
    counts = {name: 0 for name in class_names}
    unknown_ids = {}
    px_short_sides = []
    tiny = 0
    empty_labels = 0
    malformed = 0
    mixed_rows = []
    sessions = {}
    label_stems = set()

    for path in labels:
        label_stems.add(os.path.splitext(os.path.basename(path))[0])
        boxes, bad, kinds = read_boxes(path)
        malformed += bad
        if len(kinds) > 1:
            mixed_rows.append(os.path.basename(path))
        if not boxes:
            empty_labels += 1
        for cls, w, h in boxes:
            if 0 <= cls < len(class_names):
                counts[class_names[cls]] += 1
            else:
                unknown_ids[cls] = unknown_ids.get(cls, 0) + 1
            if dims:
                short = min(w * dims[0], h * dims[1])
                px_short_sides.append(short)
                if short < MIN_BOX_PX:
                    tiny += 1

    for path in images:
        session = session_of(path) or "(unprefixed)"
        sessions[session] = sessions.get(session, 0) + 1

    image_stems = {os.path.splitext(os.path.basename(p))[0] for p in images}
    return {
        "split": split,
        "images": len(images),
        "labels": len(labels),
        "unlabeled_images": sorted(image_stems - label_stems)[:5],
        "n_unlabeled": len(image_stems - label_stems),
        "orphan_labels": len(label_stems - image_stems),
        "empty_labels": empty_labels,
        "malformed": malformed,
        "mixed_rows": mixed_rows,
        "counts": counts,
        "unknown_ids": unknown_ids,
        "sessions": sessions,
        "dims": dims,
        "px_short_sides": px_short_sides,
        "tiny": tiny,
    }


def percentiles(values):
    if not values:
        return "n/a"
    ordered = sorted(values)

    def at(fraction):
        return ordered[min(len(ordered) - 1, int(fraction * len(ordered)))]

    return (f"min {ordered[0]:.0f}px  p5 {at(0.05):.0f}px  median {at(0.5):.0f}px  "
            f"p95 {at(0.95):.0f}px  max {ordered[-1]:.0f}px")


def report(scans, class_names):
    """Everything DATASET_CARD.md asks for, plus the problems worth failing on."""
    problems = []

    print("\n" + "=" * 68)
    print("Splits\n")
    print(f"{'split':<8}{'images':>8}{'labels':>8}{'empty':>8}  sessions")
    for scan in scans:
        print(f"{scan['split']:<8}{scan['images']:>8}{scan['labels']:>8}"
              f"{scan['empty_labels']:>8}  "
              f"{', '.join(sorted(scan['sessions'])) or '-'}")

    # Leakage: the whole reason uploads carry a session prefix.
    where = {}
    for scan in scans:
        for session in scan["sessions"]:
            where.setdefault(session, []).append(scan["split"])
    straddling = {s: v for s, v in where.items() if len(v) > 1 and s != "(unprefixed)"}
    if straddling:
        problems.append("session leakage across splits:\n" + "\n".join(
            f"    {s} appears in {', '.join(v)}" for s, v in sorted(straddling.items())))
    if "(unprefixed)" in where:
        print("\nnote: some images have no <session>__ prefix — uploaded outside "
              "roboflow_upload.py.\n      Leakage cannot be checked for those.")

    # Ultralytics refuses to load a label file that mixes a `cls x y w h` row
    # with a polygon row, and it refuses quietly: the image is dropped with one
    # "corrupt" line in a scroll of loading output, and training continues on
    # whatever is left. read_boxes reads both kinds happily, so without this the
    # counts printed below would describe images the model never sees -- and
    # DATASET_CARD.md would record them. Roboflow produces the mix when some
    # annotations on an image were drawn with Smart Polygon and the rest as
    # plain boxes.
    mixed_total = sum(len(scan["mixed_rows"]) for scan in scans)
    if mixed_total:
        detail = "\n".join(
            f"    {scan['split']}: {len(scan['mixed_rows'])} of {scan['labels']} "
            f"(e.g. {scan['mixed_rows'][0]})"
            for scan in scans if scan["mixed_rows"])
        problems.append(
            f"{mixed_total} label file(s) mix box and polygon rows. ultralytics "
            "drops each one\n    silently as 'corrupt', so they would be counted "
            "here and never trained on:\n" + detail +
            "\n    Fix in Roboflow: re-draw so every annotation on those images is "
            "the same\n    kind (this is a detection project, so boxes), then cut "
            "a new version.")

    print("\n" + "=" * 68)
    print("Instances per class\n")
    header = f"{'split':<8}" + "".join(f"{name:>9}" for name in class_names) + f"{'total':>9}"
    print(header)
    totals = {name: 0 for name in class_names}
    for scan in scans:
        row = f"{scan['split']:<8}"
        for name in class_names:
            row += f"{scan['counts'][name]:>9}"
            totals[name] += scan["counts"][name]
        row += f"{sum(scan['counts'].values()):>9}"
        print(row)
    row = f"{'ALL':<8}" + "".join(f"{totals[n]:>9}" for n in class_names)
    print(row + f"{sum(totals.values()):>9}")

    for name in class_names:
        if totals[name] == 0:
            problems.append(f"class {name!r} has no instances anywhere in the export")
    for scan in scans:
        for name in class_names:
            if totals[name] and scan["counts"][name] == 0:
                print(f"\nwarning: no {name} instances in the {scan['split']} split — "
                      f"its mAP for {name}\n         will be meaningless.")

    common = max(totals.values()) if totals else 0
    for name in class_names:
        if totals[name] and common / float(totals[name]) > MAX_IMBALANCE:
            print(f"\nwarning: {name} is {common / float(totals[name]):.1f}:1 behind the "
                  f"commonest class\n         (target is roughly {MAX_IMBALANCE:.0f}:1). "
                  "Shoot more — cone-zoo frames,\n         slow junction passes.")

    unknown = {}
    for scan in scans:
        for cls, n in scan["unknown_ids"].items():
            unknown[cls] = unknown.get(cls, 0) + n
    if unknown:
        problems.append("label files contain class ids outside 0..%d: %s"
                        % (len(class_names) - 1,
                           ", ".join(f"{c} ({n} boxes)" for c, n in sorted(unknown.items()))))

    all_px = [px for scan in scans for px in scan["px_short_sides"]]
    tiny = sum(scan["tiny"] for scan in scans)
    print("\n" + "=" * 68)
    if all_px:
        dims = next((s["dims"] for s in scans if s["dims"]), None)
        print(f"Box short side at {dims[0]}x{dims[1]}: {percentiles(all_px)}")
        if tiny:
            print(f"  {tiny} boxes under {MIN_BOX_PX}px — the protocol says skip those. "
                  "A few are fine;\n  a lot means someone was labeling specks near the "
                  "horizon.")
    else:
        print("Box size stats skipped (install pillow to get them).")

    malformed = sum(scan["malformed"] for scan in scans)
    if malformed:
        problems.append(f"{malformed} malformed label lines")
    unlabeled = sum(scan["n_unlabeled"] for scan in scans)
    if unlabeled:
        print(f"\nnote: {unlabeled} images have no label file. Roboflow omits the file "
              "for a\n      genuinely empty image, so this is only a problem if you "
              "expected boxes.")
    orphans = sum(scan["orphan_labels"] for scan in scans)
    if orphans:
        problems.append(f"{orphans} label files have no matching image")

    print("\n" + "=" * 68)
    print("Paste into DATASET_CARD.md:\n")
    print(f"- Total images: {sum(s['images'] for s in scans)}")
    print("- Per class instance counts: "
          + " / ".join(f"{name} {totals[name]}" for name in class_names))
    print("- Split: " + ", ".join(f"{s['split']} {s['images']}" for s in scans))
    print("\n| Session dir | Split | Images |")
    print("|-------------|-------|--------|")
    for scan in scans:
        for session, n in sorted(scan["sessions"].items()):
            print(f"| {session} | {scan['split']} | {n} |")
    return problems


def sync_labels(location, scans, class_names, labels_dir, meta):
    """Copy labels + a manifest into git. Images stay out; the labels are the work."""
    for split in [s["split"] for s in scans]:
        dest = os.path.join(labels_dir, split)
        if os.path.isdir(dest):
            shutil.rmtree(dest)
        os.makedirs(dest)
        _, src = split_dirs(location, split)
        for path in list_files(src, (".txt",)):
            shutil.copy2(path, os.path.join(dest, os.path.basename(path)))

    # Forward slashes, always: this file is committed and then read on Windows,
    # macOS and in Colab. os.path.relpath hands back backslashes on Windows, which
    # the other two read as part of the name rather than as a separator.
    rel_location = os.path.relpath(location, labels_dir).replace(os.sep, "/")
    yaml_path = os.path.join(labels_dir, "data.yaml")
    # encoding and newline are pinned for the same reason the separator is: this
    # is the committed record, and the Windows defaults (cp1252, CRLF) write a
    # different file than the Mac does from identical input. The em dash below
    # is what makes that visible -- it lands as a mojibake byte.
    with open(yaml_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("# Written by roboflow_export.py — the committed record of the\n"
                 "# dataset this model was trained on. Images are gitignored, so the\n"
                 "# paths point at a local export; re-download it with\n"
                 "#   python roboflow_export.py --version N\n")
        fh.write(f"path: {rel_location}\n")
        for scan in scans:
            fh.write(f"{yaml_key(scan['split'])}: {scan['split']}/images\n")
        fh.write(f"nc: {len(class_names)}\n")
        fh.write("names:\n")
        for i, name in enumerate(class_names):
            fh.write(f"  {i}: {name}\n")

    manifest = {
        "exported_utc": datetime.datetime.now(datetime.timezone.utc)
                                 .replace(microsecond=0).isoformat(),
        "class_names": list(class_names),
        "class_order_source": meta["class_order_source"],
        "roboflow": meta["roboflow"],
        "location": os.path.abspath(location),
        "splits": [
            {
                "split": scan["split"],
                "images": scan["images"],
                "labels": scan["labels"],
                "instances": scan["counts"],
                "sessions": scan["sessions"],
            }
            for scan in scans
        ],
    }
    with open(os.path.join(labels_dir, "manifest.json"), "w",
              encoding="utf-8", newline="\n") as fh:
        json.dump(manifest, fh, indent=2, sort_keys=True)
        fh.write("\n")
    print(f"\nsynced labels + manifest.json -> {labels_dir}")


def normalize_label_file(label_path):
    """Rewrite one label file so every row is a detection row: cls cx cy w h.

    Roboflow's polygon tools emit segmentation rows -- `cls x1 y1 x2 y2 ...` --
    and a file can hold both kinds at once when some cones were boxed and others
    traced. Ultralytics accepts a file that is all one or all the other and
    **skips the image entirely** when a file mixes them: "ignoring corrupt
    image/label: labels mix segment and detection rows". It is a warning in a
    wall of startup output, and what it costs is silent -- the first real export
    here lost 60% of the validation split that way, and every metric printed was
    computed on what survived.

    Polygons collapse to their bounding extents, which is what ultralytics itself
    would have done had the file been pure. Returns (rows, converted).
    """
    rows, converted = [], 0
    with open(label_path) as fh:
        lines = fh.readlines()

    for line in lines:
        parts = line.split()
        if not parts:
            continue
        cls = parts[0]
        if len(parts) == 5:
            rows.append(f"{cls} {' '.join(parts[1:])}")
            continue
        if len(parts) > 5 and len(parts) % 2 == 1:
            xs = [float(v) for v in parts[1::2]]
            ys = [float(v) for v in parts[2::2]]
            x0, x1 = min(xs), max(xs)
            y0, y1 = min(ys), max(ys)
            rows.append(f"{cls} {(x0 + x1) / 2:.6f} {(y0 + y1) / 2:.6f} "
                        f"{x1 - x0:.6f} {y1 - y0:.6f}")
            converted += 1
            continue
        raise ValueError(f"{label_path}: row with {len(parts)} fields is neither "
                         f"a box nor a polygon")

    if converted:
        tmp = label_path + ".tmp"
        with open(tmp, "w") as fh:
            fh.write("\n".join(rows) + "\n")
        os.replace(tmp, label_path)
    return len(rows), converted


def normalize_labels(location):
    """Flatten every label in the export to detection rows. Returns a summary.

    Done to the export rather than to labels/ because the export is what
    ultralytics reads, and the export is disposable -- re-download it and this
    runs again. Also drops the label caches: ultralytics keys them by a hash of
    the label files, and a stale cache would preserve the very exclusions this
    exists to undo.
    """
    files = converted_files = converted_rows = 0
    for split in SPLITS:
        _, labels_dir = split_dirs(location, split)
        if not os.path.isdir(labels_dir):
            continue
        for name in sorted(os.listdir(labels_dir)):
            if not name.endswith(".txt"):
                continue
            files += 1
            _, converted = normalize_label_file(os.path.join(labels_dir, name))
            if converted:
                converted_files += 1
                converted_rows += converted
        cache = os.path.join(os.path.dirname(labels_dir), "labels.cache")
        if os.path.exists(cache):
            os.remove(cache)
    return {"files": files, "converted_files": converted_files,
            "converted_rows": converted_rows}


def project_slug_from_location(location):
    """Recover the project slug from an export directory named <project>-v<N>.

    The last resort for a --no-download run, where no slug was ever supplied:
    this script named the directory, so the slug is still in it.
    """
    name = os.path.basename(os.path.normpath(location))
    head, sep, tail = name.rpartition("-v")
    return head if sep and tail.isdigit() and head else None


def rewrite_data_yaml(location, class_names, scans):
    """Point data.yaml at absolute paths, in the class order the .msg dictates.

    Roboflow writes `train: ../train/images`, which only resolves from wherever
    it happened to be unzipped. Absolute paths mean a Colab notebook and a Mac
    smoke test read the same file and mean the same thing.
    """
    path = os.path.join(location, "data.yaml")
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("# Rewritten by roboflow_export.py: absolute paths, class order\n"
                 "# verified against cone_msgs/msg/LabeledCone.msg.\n")
        fh.write(f"path: {os.path.abspath(location)}\n")
        for scan in scans:
            fh.write(f"{yaml_key(scan['split'])}: {scan['split']}/images\n")
        fh.write(f"nc: {len(class_names)}\n")
        fh.write("names:\n")
        for i, name in enumerate(class_names):
            fh.write(f"  {i}: {name}\n")
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--workspace", default=None)
    parser.add_argument("--project", default=None)
    parser.add_argument("--version", default=None, help="Roboflow dataset version number")
    parser.add_argument("--api-key", default=None, help="default: $ROBOFLOW_API_KEY")
    parser.add_argument("--format", default="yolov8", help="export format (default: yolov8)")
    parser.add_argument("--location", default=None,
                        help="where the export lives (default: export/<project>-v<N>)")
    parser.add_argument("--no-download", action="store_true",
                        help="check an export already on disk")
    parser.add_argument("--labels-dir", default=DEFAULT_LABELS_DIR)
    parser.add_argument("--no-sync-labels", action="store_true",
                        help="skip copying labels into git")
    args = parser.parse_args(argv)

    if args.no_download:
        location = args.location
        if not location:
            raise SystemExit("error: --no-download needs --location")
    else:
        location = download(args)
    location = os.path.abspath(os.path.expanduser(location))
    if not os.path.isdir(location):
        raise SystemExit(f"error: no export at {location}")

    yaml_path = os.path.join(location, "data.yaml")
    if not os.path.exists(yaml_path):
        raise SystemExit(f"error: {location} has no data.yaml — is this a YOLO export?")
    dataset_names, _ = load_data_yaml(yaml_path)
    truth, source = resolve_class_names()

    print(f"\nclass order in data.yaml: "
          + ", ".join(f"{i}={n}" for i, n in enumerate(dataset_names)))
    mismatch = check_order(dataset_names)
    if mismatch:
        raise SystemExit("\nerror: " + mismatch + "\n\nNothing was synced. Fix the "
                         "project, cut a new version, re-export.")
    print(f"matches {os.path.relpath(source, os.path.dirname(HERE))} — ok")

    class_names = tuple(truth)

    norm = normalize_labels(location)
    if norm["converted_rows"]:
        print(f"\nnormalised {norm['converted_rows']} polygon rows to boxes across "
              f"{norm['converted_files']}/{norm['files']} label files.")
        print("  Ultralytics skips any image whose label file mixes polygon and box "
              "rows, which is\n  a warning rather than an error and silently shrinks "
              "the split it validates on.")

    scans = [s for s in (scan_split(location, split, class_names) for split in SPLITS) if s]
    if not scans:
        raise SystemExit(f"error: no train/valid/test directories under {location}")

    problems = report(scans, class_names)
    rewrite_data_yaml(location, class_names, scans)

    if problems:
        print("\n" + "=" * 68)
        print("PROBLEMS — do not train on this until they are resolved:\n")
        for problem in problems:
            print(f"  * {problem}")
        print("\nLabels were NOT synced into git: committing a broken export makes it\n"
              "the record of what the model trained on.")
        return 1

    if not args.no_sync_labels:
        # Resolved, not args: the slugs arrive as often from roboflow.json or
        # $ROBOFLOW_* as from a flag, and recording the bare args wrote null for
        # both. data.yaml tells the next person to "re-download it with
        # --workspace ... --project ...", so a manifest that does not say which
        # project is a dead end -- the slugs then have to be dug out of
        # Roboflow's own README, or the account.
        config = roboflow_config.load()
        workspace, _ = roboflow_config.resolve("workspace", args, config)
        project, _ = roboflow_config.resolve("project", args, config)
        meta = {
            "class_order_source": os.path.abspath(source),
            "roboflow": {
                "workspace": workspace,
                "project": project or project_slug_from_location(location),
                "version": args.version,
                "format": args.format,
                "labels_normalized": norm,
            },
        }
        sync_labels(location, scans, class_names, os.path.expanduser(args.labels_dir), meta)

    print("\n" + "=" * 68)
    print("Export checks passed. Train with:\n")
    print(f"  cd model/training && python train.py \\\n"
          f"      --data {os.path.join(location, 'data.yaml')} --name v1")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
