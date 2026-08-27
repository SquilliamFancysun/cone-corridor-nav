"""Session bookkeeping for capture: directory naming, frame pacing, manifests.

Pure Python, no depthai, no pyserial, no foxglove. capture_cones.py owns the
camera and lidar_view.py owns the lidar; both hand data to this module, which
means everything here is exercisable by pytest on a laptop with no car attached.

Both sensors share the session naming and manifest shape on purpose: a camera
session and a lidar session recorded on the same run should be recognisably the
same thing, and provenance should look identical in both.
"""

import json
import os
import subprocess
import time
from datetime import datetime, timezone

TOOL_VERSION = "1"

# Colors are the classes. Kept here so the capture side and the dataset side
# agree without importing ROS. Order matters: it is the class-id order in
# cone_msgs/msg/LabeledCone.msg and must match the Roboflow project.
CLASS_NAMES = ("blue", "yellow", "orange", "green")


def session_dir_name(label, when=None):
    """Build a session directory name: 20260826_1432_lot-sun-A.

    The name carries the capture session, which is the unit train/val/test are
    split on (see model/dataset/DATASET_CARD.md) — so it has to survive all the
    way to the Roboflow batch name. Kept filesystem- and URL-safe.
    """
    when = when or datetime.now()
    safe = "".join(c if c.isalnum() or c in "-_" else "-" for c in label).strip("-")
    if not safe:
        raise ValueError("session label must contain at least one alphanumeric character")
    return f"{when:%Y%m%d_%H%M}_{safe}"


class FrameClock:
    """Decides which of the camera's frames to keep, at a target rate.

    The camera runs faster than we save (auto-exposure stays responsive that
    way). Deadlines advance by a fixed period rather than from each frame's
    actual arrival, so a late frame does not push every later frame late — the
    cadence stays locked instead of drifting.
    """

    def __init__(self, rate_hz):
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")
        self.period = 1.0 / rate_hz
        self._next = None

    def should_keep(self, t):
        """True if a frame arriving at monotonic time `t` should be saved."""
        if self._next is None:
            self._next = t + self.period
            return True
        if t < self._next:
            return False
        # If we fell far behind (a stall, or recording was paused), resync to
        # now rather than firing a burst to "catch up" on missed deadlines.
        if t > self._next + self.period:
            self._next = t + self.period
        else:
            self._next += self.period
        return True

    def reset(self):
        self._next = None


def git_commit(path=None):
    """Short commit of the repo this script came from, or None off-repo.

    The car gets the tool by rsync, not by clone, so this is expected to be
    None on-car; the deploy script stamps the commit into the meta instead.
    """
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=path or os.path.dirname(os.path.abspath(__file__)),
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return out.stdout.strip() or None if out.returncode == 0 else None


class _SessionBase:
    """Shared skeleton: the directory, the meta, and the provenance block.

    Split out so the lidar and camera manifests cannot drift apart in the parts
    that identify a run — only in the parts that describe a sensor.
    """

    tool_name = "unknown"

    def __init__(self, root, label, meta=None, when=None):
        self.name = session_dir_name(label, when)
        self.label = label
        self.dir = os.path.join(os.path.expanduser(root), self.name)
        self.meta = dict(meta or {})
        self.closed = False

    def _header(self, notes):
        return {
            "session": self.name,
            "label": self.label,
            "created_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "tool": {
                "name": self.tool_name,
                "version": TOOL_VERSION,
                "git_commit": self.meta.get("git_commit") or git_commit(),
            },
            "capture": {k: v for k, v in self.meta.items() if k != "git_commit"},
            "notes": notes,
        }

    def _write_manifest(self, notes):
        with open(os.path.join(self.dir, "session.json"), "w") as fh:
            json.dump(self.manifest(notes), fh, indent=2)
            fh.write("\n")
        self.closed = True
        return self.dir

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False


class SessionWriter(_SessionBase):
    """Writes one capture session: frames/*.jpg plus a session.json manifest.

    Frames are written as they arrive rather than buffered, so a session that
    ends in a crash or a yanked battery still leaves usable images behind.
    """

    tool_name = "capture_cones.py"

    def __init__(self, root, label, meta=None, when=None):
        super().__init__(root, label, meta, when)
        self.frames_dir = os.path.join(self.dir, "frames")
        os.makedirs(self.frames_dir, exist_ok=False)
        self.frames = []
        self._t0 = None

    @property
    def count(self):
        return len(self.frames)

    def add_frame(self, jpeg_bytes, t=None, **frame_meta):
        """Write one JPEG and record it. Returns the filename written."""
        if self.closed:
            raise RuntimeError("session already closed")
        t = time.monotonic() if t is None else t
        if self._t0 is None:
            self._t0 = t
        filename = f"{len(self.frames):06d}.jpg"
        with open(os.path.join(self.frames_dir, filename), "wb") as fh:
            fh.write(jpeg_bytes)
        record = {"file": filename, "t": round(t - self._t0, 4)}
        record.update(frame_meta)
        self.frames.append(record)
        return filename

    def manifest(self, notes=None):
        manifest = self._header(notes)
        manifest.update({
            "classes": list(CLASS_NAMES),
            "frame_count": len(self.frames),
            "duration_s": round(self.frames[-1]["t"], 3) if self.frames else 0.0,
            "frames": self.frames,
        })
        # Key order is load-bearing only for readability: classes and capture
        # come before the long per-frame list.
        ordered = ("session", "label", "created_utc", "tool", "classes",
                   "capture", "notes", "frame_count", "duration_s", "frames")
        return {k: manifest[k] for k in ordered}

    def close(self, notes=None):
        """Write session.json. Safe to call twice; the second call is a no-op."""
        if self.closed:
            return self.dir
        return self._write_manifest(notes)


class ScanSessionWriter(_SessionBase):
    """Writes one lidar session: scans.jsonl plus a session.json manifest.

    The MCAP alongside it is written by foxglove's own sink, which is why this
    class does not know about it — session.py stays importable with nothing
    installed. Scans are appended as they arrive, same rationale as add_frame:
    a yanked battery costs the last scan, not the session.

    scans.jsonl is deliberately redundant with the MCAP's raw channel. It is
    the zero-dependency path: the replay harness and a quick `python -c` on any
    machine can read it without foxglove or mcap installed.
    """

    tool_name = "lidar_view.py"

    def __init__(self, root, label, meta=None, when=None, jsonl=True):
        super().__init__(root, label, meta, when)
        os.makedirs(self.dir, exist_ok=False)
        self.count = 0
        self.health = {}
        self._t0 = None
        self._last_t = 0.0
        self._speeds = []
        self._jsonl = open(os.path.join(self.dir, "scans.jsonl"), "w") if jsonl else None

    @property
    def jsonl_path(self):
        return os.path.join(self.dir, "scans.jsonl")

    @property
    def mcap_path(self):
        return os.path.join(self.dir, "scans.mcap")

    def add_scan(self, scan):
        """Append one revolution. Returns its time relative to the first scan."""
        if self.closed:
            raise RuntimeError("session already closed")
        if self._t0 is None:
            self._t0 = scan.t
        t = round(scan.t - self._t0, 4)
        self._last_t = t
        self.count += 1
        self._speeds.append(scan.speed_hz)
        if self._jsonl is not None:
            json.dump({
                "t": t,
                "speed_hz": round(scan.speed_hz, 3),
                # 0.01 deg is the sensor's own resolution; more digits would be
                # invented precision and a third larger on disk.
                "angles_deg": [round(a, 2) for a in scan.angles_deg],
                "ranges_mm": list(scan.ranges_mm),
                "intensities": list(scan.intensities),
            }, self._jsonl, separators=(",", ":"))
            self._jsonl.write("\n")
        return t

    def set_health(self, **stats):
        """Link health for the manifest: packet counts, CRC drops, throughput.

        Recorded per session so a run that looks wrong later can be checked
        against the baseline in docs/hardware-baseline.md without guessing.
        """
        self.health.update(stats)

    def manifest(self, notes=None):
        manifest = self._header(notes)
        speeds = self._speeds
        manifest.update({
            "scan_count": self.count,
            "duration_s": round(self._last_t, 3),
            "mean_rotation_hz": round(sum(speeds) / len(speeds), 3) if speeds else 0.0,
            "health": self.health,
        })
        return manifest

    def close(self, notes=None):
        """Write session.json and close the jsonl. Safe to call twice."""
        if self.closed:
            return self.dir
        if self._jsonl is not None:
            self._jsonl.close()
        return self._write_manifest(notes)
