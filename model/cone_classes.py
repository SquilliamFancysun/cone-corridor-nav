"""The class list, and the one file that defines it.

`cone_msgs/msg/LabeledCone.msg` is the source of truth for class ids — those
constants are what the ROS side switches on. Roboflow assigns its indices from
the order classes were created in the project, and nothing connects the two but
a convention. A permutation between them is invisible in training metrics and
shows up on the track as a car that reads a junction gate as a wall.

So nothing downstream trusts a hardcoded order for anything that matters: the
names are parsed out of the .msg, and every dataset that arrives is checked
against them.

Also home to the upload filename convention, because the split-by-session rule
in DATASET_CARD.md is only checkable after the Roboflow round trip if the
filenames still say which session each frame came from.
"""

import os
import re

REPO_MSG_PATH = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "ros2", "src", "cone_msgs", "msg", "LabeledCone.msg",
))

# Mirrors capture/session.py. Used only when the .msg is out of reach — a Colab
# runtime with just model/ uploaded, say — and the fallback is always announced.
FALLBACK_CLASS_NAMES = ("blue", "yellow", "red", "orange", "magenta")

SPLITS = ("train", "valid", "test")

STAGE_SEP = "__"

_CONST_RE = re.compile(r"^\s*uint8\s+CLASS_([A-Za-z_]+)\s*=\s*(\d+)")


def class_names_from_msg(msg_path=None):
    """Class names in id order, parsed from LabeledCone.msg.

    Raises if the ids are not a gapless 0..N-1 run: a gap means the message
    grew a class the dataset does not know about yet.
    """
    path = msg_path or REPO_MSG_PATH
    found = {}
    with open(path) as fh:
        for line in fh:
            match = _CONST_RE.match(line)
            if match:
                found[int(match.group(2))] = match.group(1).lower()
    if not found:
        raise ValueError(f"{path}: no 'uint8 CLASS_* = <id>' constants found")
    expected = list(range(len(found)))
    if sorted(found) != expected:
        raise ValueError(f"{path}: class ids are {sorted(found)}, expected {expected}")
    return tuple(found[i] for i in expected)


def resolve_class_names(msg_path=None, quiet=False):
    """(names, where_they_came_from) — the .msg when reachable, else the fallback."""
    try:
        path = msg_path or REPO_MSG_PATH
        return class_names_from_msg(path), path
    except (OSError, ValueError) as exc:
        if not quiet:
            print(f"warning: cannot read class order from the .msg ({exc})")
            print("warning: falling back to the hardcoded order in model/cone_classes.py")
        return FALLBACK_CLASS_NAMES, "hardcoded fallback in model/cone_classes.py"


def check_order(names, msg_path=None, quiet=False):
    """Compare a dataset's class list against the message.

    Returns None when they agree, or a multi-line explanation of the mismatch.
    Callers are expected to treat that as fatal — this is the check LABELING.md
    means by "verify the order in the exported data.yaml, do not assume it".
    """
    truth, source = resolve_class_names(msg_path, quiet=quiet)
    got = tuple(str(n).strip().lower() for n in names)
    if got == truth:
        return None

    def enumerate_names(seq):
        return ", ".join(f"{i}={n}" for i, n in enumerate(seq))

    lines = [
        f"class order does not match {source}",
        f"  expected: {enumerate_names(truth)}",
        f"  dataset:  {enumerate_names(got)}",
    ]
    if sorted(got) == sorted(truth):
        lines.append("  Same names, different order — this is exactly the silent")
        lines.append("  permutation. Fix the class order in the Roboflow project")
        lines.append("  (Classes tab) and re-export; do not remap it downstream.")
    else:
        missing = [n for n in truth if n not in got]
        extra = [n for n in got if n not in truth]
        if missing:
            lines.append(f"  missing from the dataset: {', '.join(missing)}")
        if extra:
            lines.append(f"  not a cone class: {', '.join(extra)}")
    return "\n".join(lines)


def load_data_yaml(path):
    """(names, parsed_doc) from a YOLO data.yaml. Names come back in id order."""
    try:
        import yaml
    except ImportError:
        raise SystemExit(
            "error: needs pyyaml to read data.yaml.\n"
            "       uv run --with pyyaml ..."
        )
    with open(path) as fh:
        doc = yaml.safe_load(fh)
    if not isinstance(doc, dict) or "names" not in doc:
        raise SystemExit(f"error: {path} has no 'names' — is it a YOLO data.yaml?")
    names = doc["names"]
    if isinstance(names, dict):
        names = [names[key] for key in sorted(names, key=int)]
    return list(names), doc


def staged_name(session, filename):
    """Upload filename that carries its session: 20260827_1503_eli1__000123.jpg

    Frames are numbered per session, so `000123.jpg` collides the moment two
    sessions share a Roboflow project. The prefix survives Roboflow's own
    `_jpg.rf.<hash>` suffix, which is what makes the leakage check in
    roboflow_export.py possible at all.
    """
    return f"{session}{STAGE_SEP}{filename}"


def session_of(path):
    """Session name recovered from an exported image filename, or None."""
    base = os.path.basename(path)
    if STAGE_SEP not in base:
        return None
    return base.split(STAGE_SEP, 1)[0]
