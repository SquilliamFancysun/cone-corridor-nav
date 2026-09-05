"""Upload culled capture sessions to Roboflow, one batch per session.

Runs after prepare_dataset.py has done the culling. Three things it does that
the web uploader will not:

1. **One batch per session, split assigned per session.** The split lives in
   splits.json, not in Roboflow's random assignment — see that file for why.
2. **Renames on the way up** to `<session>__<frame>.jpg`. Frames are numbered
   per session, so `000123.jpg` collides across sessions; the prefix also
   survives the round trip, which is what lets roboflow_export.py prove no
   session leaked across the split.
3. **Refuses to upload a session nobody has assigned a split to.** Deciding
   that after the images are already in the project is how frames end up in
   the wrong place.

    export ROBOFLOW_API_KEY=...
    python roboflow_upload.py --dry-run
    python roboflow_upload.py

Workspace and project come from model/roboflow.json (see roboflow_config.py);
--workspace / --project and $ROBOFLOW_* still override it. The API key never
goes in that file -- it is committed.

Only the *kept* frames go up — `_rejected/` is skipped, since those were culled
precisely so nobody spends labeling time on them.

Requires the off-car venv: see ../requirements.txt.
"""

import argparse
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cone_classes import (SPLITS, record_uploaded,  # noqa: E402
                          resolve_class_names, staged_name)

import roboflow_config  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_IMAGES_DIR = os.path.join(HERE, "images")
DEFAULT_SPLITS_FILE = os.path.join(HERE, "splits.json")
IMAGE_EXTS = (".jpg", ".jpeg", ".png")


def load_splits(path):
    with open(path, encoding="utf-8") as fh:
        doc = json.load(fh)
    sessions = doc.get("sessions", {})
    if not sessions:
        raise SystemExit(f"error: {path} has no 'sessions' map")
    bad = {n: v for n, v in sessions.items() if v is not None and v not in SPLITS}
    if bad:
        raise SystemExit(
            f"error: {path}: split must be one of {', '.join(SPLITS)} or null; got "
            + ", ".join(f"{n}={v!r}" for n, v in bad.items())
        )
    return sessions


def kept_frames(session_dir):
    """Frames still in frames/ — the cull moved the rest to _rejected/."""
    frames_dir = os.path.join(session_dir, "frames")
    if not os.path.isdir(frames_dir):
        return []
    return [
        os.path.join(frames_dir, n)
        for n in sorted(os.listdir(frames_dir))
        if n.lower().endswith(IMAGE_EXTS)
    ]


def stage(paths, session, stage_dir):
    """Symlink frames under their session-prefixed upload names.

    Symlinks rather than copies: the uploader opens the file and sends the
    basename, so this costs nothing and never risks a stale second copy of the
    dataset drifting out of sync with images/.
    """
    staged = []
    for path in paths:
        link = os.path.join(stage_dir, staged_name(session, os.path.basename(path)))
        try:
            os.symlink(os.path.abspath(path), link)
        except (OSError, NotImplementedError):
            import shutil
            shutil.copy2(path, link)
        staged.append(link)
    return staged


def connect(args):
    try:
        from roboflow import Roboflow
    except ImportError:
        raise SystemExit(
            "error: needs the roboflow SDK.\n"
            "       pip install -r ../requirements.txt"
        )
    key = args.api_key or os.environ.get("ROBOFLOW_API_KEY")
    if not key:
        raise SystemExit(
            "error: no API key. export ROBOFLOW_API_KEY=... (Roboflow > Settings >\n"
            "       API Keys). Do not put it in a file in this repo."
        )
    workspace, project_id = roboflow_config.slugs(args)
    project = Roboflow(api_key=key).workspace(workspace).project(project_id)
    check_project_classes(project)
    return project


def check_project_classes(project):
    """Warn early if the project's class order already disagrees with the .msg.

    Advisory only: an empty project has no classes yet, and Roboflow reports
    them unordered in some API versions. The authoritative check is on the
    exported data.yaml in roboflow_export.py.
    """
    truth, source = resolve_class_names(quiet=True)
    try:
        classes = list(getattr(project, "classes", None) or [])
    except Exception:
        return
    if not classes:
        return
    unknown = [c for c in classes if str(c).lower() not in truth]
    if unknown:
        print(f"warning: project has classes not in {os.path.basename(source)}: "
              f"{', '.join(map(str, unknown))}")
    missing = [c for c in truth if c not in [str(x).lower() for x in classes]]
    if missing:
        print(f"note: project has no {', '.join(missing)} yet — create the classes "
              f"in the order {', '.join(truth)}")


def upload_session(project, session, split, paths, args):
    print(f"\n{session}  ->  batch {session!r}, split {split}, {len(paths)} frames")
    if args.dry_run:
        for path in paths[:3]:
            print(f"  would upload {staged_name(session, os.path.basename(path))}")
        if len(paths) > 3:
            print(f"  ... and {len(paths) - 3} more")
        return len(paths), []

    tags = list(args.tag) + [session]
    ok, failed = 0, []
    with tempfile.TemporaryDirectory(prefix="rf-upload-") as stage_dir:
        for i, link in enumerate(stage(paths, session, stage_dir), start=1):
            try:
                project.upload(
                    link,
                    batch_name=session,
                    split=split,
                    tag_names=tags,
                    num_retry_uploads=args.retries,
                )
                ok += 1
            except Exception as exc:  # network, quota, duplicate — all worth naming
                failed.append((os.path.basename(link), str(exc).strip()))
            if i % 25 == 0 or i == len(paths):
                print(f"  ...{i}/{len(paths)} uploaded, {len(failed)} failed",
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
    parser.add_argument("--workspace", default=None, help="Roboflow workspace url slug")
    parser.add_argument("--project", default=None, help="Roboflow project url slug")
    parser.add_argument("--api-key", default=None, help="default: $ROBOFLOW_API_KEY")
    parser.add_argument("--images-dir", default=DEFAULT_IMAGES_DIR)
    parser.add_argument("--splits-file", default=DEFAULT_SPLITS_FILE)
    parser.add_argument("--session", action="append", default=None,
                        help="only upload this session; repeatable (default: all "
                             "sessions assigned a split in splits.json)")
    parser.add_argument("--limit", type=int, default=None,
                        help="upload at most N evenly-spaced frames per session — "
                             "how you get the first ~150 hand-label batch without "
                             "sampling one end of the drive")
    parser.add_argument("--tag", action="append", default=[],
                        help="extra Roboflow tag; repeatable")
    parser.add_argument("--retries", type=int, default=3)
    parser.add_argument("--dry-run", action="store_true",
                        help="resolve everything and upload nothing")
    args = parser.parse_args(argv)

    images_dir = os.path.expanduser(args.images_dir)
    splits = load_splits(args.splits_file)

    on_disk = sorted(
        n for n in os.listdir(images_dir)
        if os.path.isdir(os.path.join(images_dir, n)) and not n.startswith(".")
    )
    unlisted = [n for n in on_disk if n not in splits]
    if unlisted:
        print(f"error: {len(unlisted)} session(s) on disk are not in "
              f"{os.path.basename(args.splits_file)}:")
        for name in unlisted:
            print(f'    "{name}": null,')
        raise SystemExit("\nAdd them with a split of train, valid or test first. "
                         "Assigning\nthe split after the images are up is how frames "
                         "end up in the wrong one.")

    wanted = args.session or on_disk
    plan = []
    for session in wanted:
        session_dir = os.path.join(images_dir, session)
        if not os.path.isdir(session_dir):
            raise SystemExit(f"error: no such session directory: {session_dir}")
        split = splits.get(session)
        if split is None:
            print(f"skipping {session}: split is null in "
                  f"{os.path.basename(args.splits_file)}")
            continue
        paths = kept_frames(session_dir)
        if not paths:
            print(f"skipping {session}: no frames in frames/")
            continue
        if args.limit and len(paths) > args.limit:
            step = len(paths) / float(args.limit)
            paths = [paths[int(i * step)] for i in range(args.limit)]
        plan.append((session, split, paths))

    if not plan:
        raise SystemExit("error: nothing to upload")

    project = None if args.dry_run else connect(args)

    total_ok, total_failed = 0, 0
    for session, split, paths in plan:
        ok, failed = upload_session(project, session, split, paths, args)
        total_ok += ok
        total_failed += len(failed)
        if ok and not args.dry_run:
            # Which frames went up, so roboflow_prelabel.py can leave them alone:
            # proposing machine boxes over a frame a human hand-labelled is how
            # the careful work gets buried under the cheap work.
            failed_names = {name for name, _ in failed}
            sent = [p for p in paths
                    if staged_name(session, os.path.basename(p)) not in failed_names]
            total = record_uploaded(os.path.join(args.images_dir, session), sent, split)
            print(f"  recorded {len(sent)} frames in {session}/uploaded.json "
                  f"({total} total)")

    print("\n" + "=" * 68)
    by_split = {}
    for session, split, paths in plan:
        by_split.setdefault(split, []).append((session, len(paths)))
    for split in SPLITS:
        rows = by_split.get(split, [])
        if rows:
            frames = sum(n for _, n in rows)
            print(f"{split:<6} {frames:>5} frames  "
                  f"({', '.join(s for s, _ in rows)})")
    if not by_split.get("test"):
        print("\nNo test session in this upload. The test split has to be a whole "
              "session\nnobody trained on, or the held-out numbers mean nothing.")
    if args.dry_run:
        print("\n(dry run — nothing uploaded)")
        return 0
    print(f"\nuploaded {total_ok}, failed {total_failed}")
    print("\nNext: label in Roboflow (box the full cone including its base — see")
    print("LABELING.md), then roboflow_export.py, which is where the class order")
    print("gets verified against cone_msgs/msg/LabeledCone.msg.")
    return 1 if total_failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
