"""Tests for the hardware-free half of capture. Runs on a laptop, no car."""

import json
from datetime import datetime

import pytest

from ld06 import Scan
from session import (
    CLASS_NAMES,
    FrameClock,
    ScanSessionWriter,
    SessionWriter,
    session_dir_name,
)


class TestSessionDirName:
    def test_encodes_time_and_label(self):
        when = datetime(2026, 8, 26, 14, 32)
        assert session_dir_name("lot-sun-A", when) == "20260826_1432_lot-sun-A"

    def test_sanitizes_unsafe_characters(self):
        when = datetime(2026, 8, 26, 14, 32)
        name = session_dir_name("lot sun/A (2)", when)
        assert name == "20260826_1432_lot-sun-A--2"
        assert "/" not in name and " " not in name

    def test_rejects_empty_label(self):
        with pytest.raises(ValueError):
            session_dir_name("///")


class TestFrameClock:
    def test_keeps_first_frame(self):
        assert FrameClock(2.0).should_keep(100.0) is True

    def test_downsamples_10hz_to_2hz(self):
        clock = FrameClock(2.0)
        kept = [t / 10.0 for t in range(50) if clock.should_keep(t / 10.0)]
        assert len(kept) == 10
        gaps = [b - a for a, b in zip(kept, kept[1:])]
        assert all(abs(gap - 0.5) < 1e-6 for gap in gaps)

    def test_cadence_does_not_drift_after_a_late_frame(self):
        # A frame arriving slightly late must not push every later deadline late.
        clock = FrameClock(2.0)
        clock.should_keep(0.0)
        assert clock.should_keep(0.52) is True
        assert clock.should_keep(0.99) is False
        assert clock.should_keep(1.00) is True

    def test_resyncs_instead_of_bursting_after_a_stall(self):
        clock = FrameClock(2.0)
        clock.should_keep(0.0)
        assert clock.should_keep(30.0) is True
        # Deadlines are re-anchored to now, not replayed for the missed minute.
        assert clock.should_keep(30.1) is False
        assert clock.should_keep(30.5) is True

    def test_reset_makes_next_frame_immediate(self):
        clock = FrameClock(2.0)
        clock.should_keep(0.0)
        assert clock.should_keep(0.1) is False
        clock.reset()
        assert clock.should_keep(0.1) is True

    def test_rejects_nonpositive_rate(self):
        with pytest.raises(ValueError):
            FrameClock(0)


class TestSessionWriter:
    def test_writes_frames_and_manifest(self, tmp_path):
        writer = SessionWriter(str(tmp_path), "lot-sun-A", {"width": 1920})
        writer.add_frame(b"\xff\xd8jpeg-one", t=10.0, seq=1)
        writer.add_frame(b"\xff\xd8jpeg-two", t=10.5, seq=6)
        path = writer.close(notes="overcast, second lap")

        frames = sorted((tmp_path / writer.name / "frames").iterdir())
        assert [f.name for f in frames] == ["000000.jpg", "000001.jpg"]
        assert frames[0].read_bytes() == b"\xff\xd8jpeg-one"

        manifest = json.loads((tmp_path / writer.name / "session.json").read_text())
        assert manifest["frame_count"] == 2
        assert manifest["label"] == "lot-sun-A"
        assert manifest["notes"] == "overcast, second lap"
        assert manifest["capture"]["width"] == 1920
        assert manifest["classes"] == list(CLASS_NAMES)
        # Timestamps are relative to the first frame, so a session is readable
        # without knowing when the script happened to start.
        assert manifest["frames"][0]["t"] == 0.0
        assert manifest["frames"][1]["t"] == 0.5
        assert manifest["duration_s"] == 0.5
        assert path == str(tmp_path / writer.name)

    def test_close_is_idempotent(self, tmp_path):
        writer = SessionWriter(str(tmp_path), "x")
        writer.add_frame(b"a", t=0.0)
        writer.close()
        writer.close()
        assert json.loads((tmp_path / writer.name / "session.json").read_text())["frame_count"] == 1

    def test_manifest_does_not_consume_meta(self, tmp_path):
        writer = SessionWriter(str(tmp_path), "x", {"git_commit": "abc123", "width": 4})
        first = writer.manifest()
        second = writer.manifest()
        assert first == second
        assert first["tool"]["git_commit"] == "abc123"
        assert "git_commit" not in first["capture"]

    def test_rejects_frames_after_close(self, tmp_path):
        writer = SessionWriter(str(tmp_path), "x")
        writer.close()
        with pytest.raises(RuntimeError):
            writer.add_frame(b"a")

    def test_context_manager_closes(self, tmp_path):
        with SessionWriter(str(tmp_path), "x") as writer:
            writer.add_frame(b"a", t=0.0)
        assert (tmp_path / writer.name / "session.json").exists()


def make_scan(t, n=4, speed_hz=10.0):
    return Scan(t=t, angles_deg=[i * 90.0 for i in range(n)],
                ranges_mm=[1000 + i for i in range(n)],
                intensities=[200] * n, speed_hz=speed_hz)


class TestScanSessionWriter:
    def test_writes_scans_and_manifest(self, tmp_path):
        writer = ScanSessionWriter(str(tmp_path), "lot-A", {"port": "/dev/ttyUSB0"})
        writer.add_scan(make_scan(100.0))
        writer.add_scan(make_scan(100.1))
        writer.set_health(packets_ok=800, packets_bad=1)
        path = writer.close(notes="first lap")

        manifest = json.loads((tmp_path / writer.name / "session.json").read_text())
        assert manifest["scan_count"] == 2
        assert manifest["duration_s"] == 0.1
        assert manifest["mean_rotation_hz"] == 10.0
        assert manifest["capture"]["port"] == "/dev/ttyUSB0"
        assert manifest["health"] == {"packets_ok": 800, "packets_bad": 1}
        assert manifest["tool"]["name"] == "lidar_view.py"
        assert manifest["notes"] == "first lap"
        assert path == str(tmp_path / writer.name)

    def test_jsonl_is_one_scan_per_line(self, tmp_path):
        writer = ScanSessionWriter(str(tmp_path), "x")
        writer.add_scan(make_scan(5.0))
        writer.add_scan(make_scan(5.1))
        writer.close()

        lines = (tmp_path / writer.name / "scans.jsonl").read_text().splitlines()
        assert len(lines) == 2
        first = json.loads(lines[0])
        # Times are relative to the first scan, as in the camera manifest.
        assert first["t"] == 0.0
        assert json.loads(lines[1])["t"] == 0.1
        assert first["ranges_mm"] == [1000, 1001, 1002, 1003]
        assert len(first["angles_deg"]) == 4

    def test_scans_survive_a_session_that_never_closes(self, tmp_path):
        # The yanked-battery case: no manifest, but the scans are on disk.
        writer = ScanSessionWriter(str(tmp_path), "x")
        writer.add_scan(make_scan(0.0))
        writer._jsonl.flush()
        assert (tmp_path / writer.name / "scans.jsonl").read_text().count("\n") == 1

    def test_no_jsonl_when_disabled(self, tmp_path):
        writer = ScanSessionWriter(str(tmp_path), "x", jsonl=False)
        writer.add_scan(make_scan(0.0))
        writer.close()
        assert not (tmp_path / writer.name / "scans.jsonl").exists()
        assert json.loads((tmp_path / writer.name / "session.json").read_text())["scan_count"] == 1

    def test_close_is_idempotent(self, tmp_path):
        writer = ScanSessionWriter(str(tmp_path), "x")
        writer.add_scan(make_scan(0.0))
        writer.close()
        writer.close()
        assert json.loads((tmp_path / writer.name / "session.json").read_text())["scan_count"] == 1

    def test_rejects_scans_after_close(self, tmp_path):
        writer = ScanSessionWriter(str(tmp_path), "x")
        writer.close()
        with pytest.raises(RuntimeError):
            writer.add_scan(make_scan(0.0))

    def test_refuses_to_reuse_a_directory(self, tmp_path):
        writer = ScanSessionWriter(str(tmp_path), "x", when=datetime(2026, 8, 26, 14, 32))
        writer.close()
        with pytest.raises(FileExistsError):
            ScanSessionWriter(str(tmp_path), "x", when=datetime(2026, 8, 26, 14, 32))
