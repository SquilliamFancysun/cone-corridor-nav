"""Hardware-free tests for the on-car audio process lifecycle."""

from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest


CAPTURE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(CAPTURE_DIR))

from audio_playback import (
    AudioController,
    CLOSED,
    DRIVING,
    GOAL,
    STOPPED,
    update_for_deadman,
    update_for_goal,
)


class FakeProcess:
    def __init__(self):
        self.returncode = None
        self.terminated = False
        self.killed = False

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminated = True
        self.returncode = -15

    def kill(self):
        self.killed = True
        self.returncode = -9

    def wait(self, timeout=None):
        if self.returncode is None:
            raise subprocess.TimeoutExpired("pw-play", timeout)
        return self.returncode


class FakePopen:
    def __init__(self):
        self.calls = []
        self.processes = []

    def __call__(self, argv, **kwargs):
        process = FakeProcess()
        self.calls.append((argv, kwargs))
        self.processes.append(process)
        return process


class RecordingAudio:
    def __init__(self):
        self.events = []

    def start_driving(self):
        self.events.append(DRIVING)

    def goal_reached(self):
        self.events.append(GOAL)

    def stop(self):
        self.events.append(STOPPED)


def wait_for(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("timed out waiting for audio worker")


class AudioControllerTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        root = Path(self.temp_dir.name)
        self.drive = root / "drive.mp3"
        self.goal = root / "goal.mp3"
        self.drive.touch()
        self.goal.touch()
        self.popen = FakePopen()
        self.warnings = []
        self.audio = AudioController(
            drive_audio=self.drive,
            goal_audio=self.goal,
            volume=0.4,
            target="usb-speaker",
            popen_factory=self.popen,
            warn=self.warnings.append,
        )

    def tearDown(self):
        self.audio.close()
        self.temp_dir.cleanup()

    def test_driving_starts_once_and_builds_safe_argv(self):
        self.audio.start_driving()
        self.audio.start_driving()
        wait_for(lambda: len(self.popen.calls) == 1)

        argv, kwargs = self.popen.calls[0]
        self.assertEqual(
            argv,
            ["pw-play", "--volume", "0.400", "--target", "usb-speaker",
             str(self.drive)],
        )
        self.assertEqual(kwargs["stdout"], subprocess.DEVNULL)
        self.assertEqual(self.audio.state, DRIVING)

    def test_goal_stops_driving_track_and_plays_once(self):
        self.audio.start_driving()
        wait_for(lambda: len(self.popen.calls) == 1)
        driving_process = self.popen.processes[0]

        self.audio.goal_reached()
        self.audio.goal_reached()
        wait_for(lambda: len(self.popen.calls) == 2)

        self.assertTrue(driving_process.terminated)
        self.assertEqual(self.popen.calls[1][0][-1], str(self.goal))
        self.assertEqual(self.audio.state, GOAL)

    def test_stop_terminates_current_track(self):
        self.audio.start_driving()
        wait_for(lambda: len(self.popen.calls) == 1)
        process = self.popen.processes[0]

        self.audio.stop()
        wait_for(lambda: process.terminated)

        self.assertEqual(self.audio.state, STOPPED)

    def test_close_is_idempotent_and_stops_process(self):
        self.audio.start_driving()
        wait_for(lambda: len(self.popen.calls) == 1)
        process = self.popen.processes[0]

        self.audio.close()
        self.audio.close()

        self.assertTrue(process.terminated)
        self.assertEqual(self.audio.state, CLOSED)

    def test_missing_file_warns_instead_of_starting_player(self):
        self.audio.close()
        missing = Path(self.temp_dir.name) / "missing.mp3"
        self.audio = AudioController(
            drive_audio=missing,
            goal_audio=self.goal,
            popen_factory=self.popen,
            warn=self.warnings.append,
        )

        self.audio.start_driving()
        wait_for(lambda: bool(self.warnings))

        self.assertEqual(self.popen.calls, [])
        self.assertIn("audio file not found", self.audio.last_error)

    def test_disabled_controller_is_a_no_op(self):
        self.audio.close()
        self.audio = AudioController(
            drive_audio=self.drive,
            goal_audio=self.goal,
            enabled=False,
            popen_factory=self.popen,
        )

        self.audio.start_driving()
        self.audio.goal_reached()
        self.audio.stop()
        self.audio.close()

        self.assertEqual(self.popen.calls, [])
        self.assertEqual(self.audio.state, CLOSED)

    def test_rejects_invalid_volume(self):
        self.audio.close()
        with self.assertRaises(ValueError):
            AudioController(
                drive_audio=self.drive,
                goal_audio=self.goal,
                volume=1.1,
                popen_factory=self.popen,
            )


class DriveEventIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.audio = RecordingAudio()

    def test_deadman_rising_edge_starts_driving_track(self):
        update_for_deadman(self.audio, armed=True, was_armed=False)
        self.assertEqual(self.audio.events, [DRIVING])

    def test_deadman_falling_edge_stops_driving_track(self):
        update_for_deadman(self.audio, armed=False, was_armed=True)
        self.assertEqual(self.audio.events, [STOPPED])

    def test_release_after_goal_does_not_cut_off_finish_clip(self):
        update_for_deadman(
            self.audio, armed=False, was_armed=True, goal_stopped=True)
        self.assertEqual(self.audio.events, [])

    def test_only_transition_into_stopped_plays_finish_clip(self):
        update_for_goal(self.audio, goal_stopped=True, state_changed=True)
        update_for_goal(self.audio, goal_stopped=True, state_changed=False)
        update_for_goal(self.audio, goal_stopped=False, state_changed=True)
        self.assertEqual(self.audio.events, [GOAL])


if __name__ == "__main__":
    unittest.main()
