"""CLI contract tests for drive_junction audio controls."""

from contextlib import redirect_stderr
import io
from pathlib import Path
import sys
import tempfile
import unittest


CAPTURE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(CAPTURE_DIR))

import audio_playback
import drive_junction


class DriveJunctionAudioArgsTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.route = Path(self.temp_dir.name) / "route.txt"
        self.route.write_text("left\n", encoding="utf-8")

    def tearDown(self):
        self.temp_dir.cleanup()

    def parse(self, *extra):
        return drive_junction.parse_args(
            ["--route", str(self.route), *extra])

    def test_defaults_select_bundled_audio_and_default_sink(self):
        args = self.parse()

        self.assertEqual(
            args.drive_audio, str(audio_playback.DEFAULT_DRIVE_AUDIO))
        self.assertEqual(
            args.goal_audio, str(audio_playback.DEFAULT_GOAL_AUDIO))
        self.assertEqual(args.audio_volume, 1.0)
        self.assertIsNone(args.audio_target)
        self.assertFalse(args.no_audio)

    def test_every_audio_setting_can_be_overridden(self):
        args = self.parse(
            "--drive-audio", "/tmp/alternate drive.mp3",
            "--goal-audio", "/tmp/alternate goal.mp3",
            "--audio-volume", "0.35",
            "--audio-target", "usb-speaker",
            "--no-audio",
        )

        self.assertEqual(args.drive_audio, "/tmp/alternate drive.mp3")
        self.assertEqual(args.goal_audio, "/tmp/alternate goal.mp3")
        self.assertEqual(args.audio_volume, 0.35)
        self.assertEqual(args.audio_target, "usb-speaker")
        self.assertTrue(args.no_audio)

    def test_volume_bounds_are_inclusive(self):
        self.assertEqual(self.parse("--audio-volume", "0").audio_volume, 0.0)
        self.assertEqual(self.parse("--audio-volume", "1").audio_volume, 1.0)

    def test_volume_outside_bounds_is_rejected(self):
        for value in ("-0.01", "1.01"):
            with self.subTest(value=value):
                with redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit):
                        self.parse("--audio-volume", value)


if __name__ == "__main__":
    unittest.main()
