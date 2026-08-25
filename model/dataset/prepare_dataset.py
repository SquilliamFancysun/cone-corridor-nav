"""Pull capture sessions off the car and cull them into a labelable dataset.

Runs on a laptop. Three jobs: rsync sessions down, drop frames that are not
worth a labeler's time, and print the numbers that DATASET_CARD.md asks for.

Rejected frames are MOVED to <session>/_rejected/, never deleted — the contact
sheet is there so you can check the script's judgement and put anything back.

    uv run --with pillow --with numpy python prepare_dataset.py --pull robocar
    uv run --with pillow --with numpy python prepare_dataset.py --dry-run

Requires: pillow, numpy.
"""

import argparse
import json
import os
import shutil
import subprocess
import sys

try:
    import numpy as np
    from PIL import Image
except ImportError:
    raise SystemExit(
        "error: needs pillow and numpy.\n"
        "       uv run --with pillow --with numpy python prepare_dataset.py ..."
    )

DEFAULT_IMAGES_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "images")
REJECT_DIRNAME = "_rejected"


def dhash(path, size=16):
    """64-bit difference hash of a downscaled grayscale frame.

    Compares adjacent-pixel gradients rather than absolute values, so it keys on
    scene structure and shrugs off the exposure wobble between frames.
    """
    with Image.open(path) as img:
        small = img.convert("L").resize((size + 1, size), Image.BILINEAR)
    pixels = np.asarray(small, dtype=np.int16)
    bits = (pixels[:, 1:] > pixels[:, :-1]).flatten()
    value = 0
    for bit in bits:
        value = (value << 1) | int(bit)
    return value


def hamming(a, b):
    return bin(a ^ b).count("1")


def blur_score(path, width=512):
    """Variance of the Laplacian: low means smeared.

    Scale-dependent, which is why every frame is resized to the same width
    first and why the script prints the distribution instead of trusting the
    default threshold.
    """
    with Image.open(path) as img:
        gray = img.convert("L")
        height = max(1, round(gray.height * width / gray.width))
        gray = gray.resize((width, height), Image.BILINEAR)
    a = np.asarray(gray, dtype=np.float32)
    lap = (
        -4 * a[1:-1, 1:-1]
        + a[:-2, 1:-1] + a[2:, 1:-1]
        + a[1:-1, :-2] + a[1:-1, 2:]
    )
    return float(lap.var())


def contact_sheet(paths, out_path, cols=8, thumb=200, limit=96):
    """Grid of thumbnails, so a human can check the cull in one glance."""
    if not paths:
        return None
    step = max(1, len(paths) // limit)
    chosen = paths[::step][:limit]
    rows = (len(chosen) + cols - 1) // cols
    with Image.open(chosen[0]) as first:
        cell_h = max(1, round(first.height * thumb / first.width))
    tiles = []
    for path in chosen:
        with Image.open(path) as img:
            tiles.append(img.convert("RGB").resize((thumb, cell_h), Image.BILINEAR))
    sheet = Image.new("RGB", (cols * thumb, rows * cell_h), (24, 24, 24))
    for i, tile in enumerate(tiles):
        sheet.paste(tile, ((i % cols) * thumb, (i // cols) * cell_h))
    sheet.save(out_path, quality=85)
    return out_path


def percentiles(values):
    if not values:
        return "n/a"
    arr = np.asarray(values)
    p = np.percentile(arr, [1, 5, 50, 95])
    return f"min {arr.min():.0f}  p1 {p[0]:.0f}  p5 {p[1]:.0f}  median {p[2]:.0f}  p95 {p[3]:.0f}"


def pull(host, remote_root, dest):
    os.makedirs(dest, exist_ok=True)
    remote = f"{host}:{remote_root.rstrip('/')}/"
    print(f"rsync {remote} -> {dest}/")
    subprocess.run(
        ["rsync", "-av", "--exclude", REJECT_DIRNAME, remote, dest + "/"],
        check=True,
    )


def frame_paths(session_dir):
    frames_dir = os.path.join(session_dir, "frames")
    if not os.path.isdir(frames_dir):
        return []
    return [
        os.path.join(frames_dir, n)
        for n in sorted(os.listdir(frames_dir))
        if n.lower().endswith((".jpg", ".jpeg", ".png"))
    ]


def process_session(session_dir, args):
    paths = frame_paths(session_dir)
    name = os.path.basename(session_dir.rstrip("/"))
    if not paths:
        print(f"\n{name}: no frames found, skipping")
        return None

    print(f"\n{name}: {len(paths)} frames")

    kept, dup, blurry = [], [], []
    blur_values = []
    last_hash = None

    for i, path in enumerate(paths):
        if args.progress and i % 100 == 0 and i:
            print(f"  ...{i}/{len(paths)}", file=sys.stderr)
        score = blur_score(path)
        blur_values.append(score)
        if score < args.blur_threshold:
            blurry.append(path)
            continue
        digest = dhash(path)
        if last_hash is not None and hamming(digest, last_hash) <= args.dup_distance:
            dup.append(path)
            continue
        last_hash = digest
        kept.append(path)

    print(f"  blur (variance of Laplacian): {percentiles(blur_values)}")
    print(f"  threshold {args.blur_threshold} -> {len(blurry)} blurred")
    print(f"  dHash distance <= {args.dup_distance} -> {len(dup)} near-duplicates")
    print(f"  KEPT {len(kept)} / {len(paths)}"
          f"  ({100.0 * len(kept) / len(paths):.0f}%)")

    manifest_path = os.path.join(session_dir, "session.json")
    if os.path.exists(manifest_path):
        with open(manifest_path) as fh:
            meta = json.load(fh).get("capture", {})
        if meta:
            print(f"  camera: {meta.get('width')}x{meta.get('height')}, "
                  f"{meta.get('lock_mode', 'unknown lock')}")

    if args.dry_run:
        print("  (dry run — nothing moved)")
        return {"session": name, "total": len(paths), "kept": len(kept),
                "blurry": len(blurry), "dup": len(dup)}

    reject_dir = os.path.join(session_dir, REJECT_DIRNAME)
    if blurry or dup:
        os.makedirs(reject_dir, exist_ok=True)
    for path in blurry + dup:
        shutil.move(path, os.path.join(reject_dir, os.path.basename(path)))

    sheet = contact_sheet(kept, os.path.join(session_dir, "contact_sheet.jpg"))
    if sheet:
        print(f"  contact sheet: {sheet}")
    if blurry or dup:
        rejected = sorted(
            os.path.join(reject_dir, n) for n in os.listdir(reject_dir)
            if n.lower().endswith((".jpg", ".jpeg", ".png"))
        )
        contact_sheet(rejected, os.path.join(session_dir, "contact_sheet_rejected.jpg"))
        print(f"  rejected sheet: {os.path.join(session_dir, 'contact_sheet_rejected.jpg')}")

    return {"session": name, "total": len(paths), "kept": len(kept),
            "blurry": len(blurry), "dup": len(dup)}


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--images-dir", default=DEFAULT_IMAGES_DIR,
                        help="local dataset image root (default: model/dataset/images)")
    parser.add_argument("--pull", metavar="HOST", default=None,
                        help="rsync sessions from this ssh host first, e.g. robocar")
    parser.add_argument("--remote-root", default="~/cone_capture",
                        help="capture root on the car (default: ~/cone_capture)")
    parser.add_argument("--session", action="append", default=None,
                        help="only process this session dir; repeatable")
    parser.add_argument("--blur-threshold", type=float, default=40.0,
                        help="reject below this variance of Laplacian; check the "
                             "printed distribution before trusting it (default: 40)")
    parser.add_argument("--dup-distance", type=int, default=6,
                        help="reject if dHash is within this Hamming distance of the "
                             "last kept frame (default: 6; 0 disables)")
    parser.add_argument("--dry-run", action="store_true",
                        help="report what would be moved, move nothing")
    parser.add_argument("--progress", action="store_true")
    args = parser.parse_args(argv)

    images_dir = os.path.expanduser(args.images_dir)
    if args.pull:
        pull(args.pull, args.remote_root, images_dir)

    if not os.path.isdir(images_dir):
        raise SystemExit(f"error: {images_dir} does not exist (use --pull first?)")

    names = args.session or sorted(
        n for n in os.listdir(images_dir)
        if os.path.isdir(os.path.join(images_dir, n)) and not n.startswith(".")
    )
    if not names:
        raise SystemExit(f"error: no session directories in {images_dir}")

    rows = [r for r in (process_session(os.path.join(images_dir, n), args) for n in names) if r]

    print("\n" + "=" * 68)
    print("Paste into model/dataset/DATASET_CARD.md:\n")
    print("| Session dir | Condition | Frames kept / captured | Notes |")
    print("|-------------|-----------|------------------------|-------|")
    for r in rows:
        print(f"| {r['session']} |  | {r['kept']} / {r['total']} | "
              f"{r['blurry']} blurred, {r['dup']} duplicate |")
    total_kept = sum(r["kept"] for r in rows)
    print(f"\nTotal kept: {total_kept} frames across {len(rows)} sessions")
    if total_kept < 1500:
        print("Below the ~1500-2500 target — plan another capture day.")
    print("\nCheck each contact_sheet.jpg before uploading. Upload ONE BATCH PER")
    print("SESSION to Roboflow and assign train/valid/test by batch, or near-identical")
    print("frames leak across the split and inflate mAP.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
