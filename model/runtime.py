"""Bits every off-car script needs: which accelerator to use, and provenance.

The provenance helpers exist because the training runs are a deliverable. A
curve with no record of which commit, which dataset export and which library
version produced it cannot be defended in the report, and it certainly cannot
be reproduced four days later when the numbers have moved.
"""

import hashlib
import os
import platform
import subprocess


def pick_device(requested=None):
    """'0' for CUDA, 'mps' on Apple silicon, else 'cpu'.

    Ultralytics takes the same strings. mps is fine for a smoke test and slow
    for a real run — Colab's T4 is the recommended runner.
    """
    if requested:
        return requested
    try:
        import torch
    except ImportError:
        return "cpu"
    if torch.cuda.is_available():
        return "0"
    mps = getattr(torch.backends, "mps", None)
    if mps is not None and mps.is_available():
        return "mps"
    return "cpu"


def git_commit(path=None):
    """Short commit of the repo this script came from, or None off-repo."""
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=path or os.path.dirname(os.path.abspath(__file__)),
            capture_output=True, text=True, check=True,
        )
        return out.stdout.strip() or None
    except (OSError, subprocess.CalledProcessError):
        return None


def dirty_worktree(path=None):
    """True if there are uncommitted changes — a run stamped with a commit that
    does not describe the code that produced it is worse than an unstamped one."""
    try:
        out = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=path or os.path.dirname(os.path.abspath(__file__)),
            capture_output=True, text=True, check=True,
        )
        return bool(out.stdout.strip())
    except (OSError, subprocess.CalledProcessError):
        return False


def sha256(path, chunk=1 << 16):
    digest = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(chunk), b""):
            digest.update(block)
    return digest.hexdigest()


def versions():
    """What was installed at run time. Ultralytics defaults move between releases."""
    info = {"python": platform.python_version(), "platform": platform.platform()}
    for name in ("ultralytics", "torch"):
        try:
            module = __import__(name)
            info[name] = getattr(module, "__version__", "unknown")
        except ImportError:
            info[name] = None
    try:
        import torch
        info["cuda"] = torch.version.cuda if torch.cuda.is_available() else None
    except ImportError:
        info["cuda"] = None
    return info
