"""Nonblocking audio playback for the on-car driving tools.

The navigation loop must never wait for audio. ``AudioController`` therefore
owns ``pw-play`` in a worker thread and exposes small, idempotent state-change
methods to the driving code. A missing file, speaker, or player is reported as
a warning and does not propagate into vehicle control.
"""

from pathlib import Path
import queue
import subprocess
import threading


HERE = Path(__file__).resolve().parent
DEFAULT_DRIVE_AUDIO = HERE / "audio" / "DrivingSound.mp3"
DEFAULT_GOAL_AUDIO = HERE / "audio" / "EndSound.mp3"

STOPPED = "stopped"
DRIVING = "driving"
GOAL = "goal"
CLOSED = "closed"

_CLOSE = object()


class AudioController:
    """Play driving and goal audio without blocking the caller.

    Public state changes only enqueue work. Starting a process, stopping it,
    and waiting for it to exit all happen on the private worker thread.

    ``popen_factory`` and ``warn`` are injectable so the lifecycle can be
    tested without opening a real audio device.
    """

    def __init__(self, drive_audio=DEFAULT_DRIVE_AUDIO,
                 goal_audio=DEFAULT_GOAL_AUDIO, volume=1.0, target=None,
                 player="pw-play", enabled=True,
                 popen_factory=subprocess.Popen, warn=print):
        volume = float(volume)
        if not 0.0 <= volume <= 1.0:
            raise ValueError("audio volume must be between 0.0 and 1.0")

        self.drive_audio = Path(drive_audio).expanduser()
        self.goal_audio = Path(goal_audio).expanduser()
        self.volume = volume
        self.target = target
        self.player = player
        self.enabled = bool(enabled)
        self._popen = popen_factory
        self._warn_callback = warn

        self._commands = queue.Queue()
        self._lock = threading.Lock()
        self._desired = STOPPED
        self._closed = False
        self._last_error = ""
        self._thread = None

        if self.enabled:
            self._thread = threading.Thread(
                target=self._run, name="audio-playback", daemon=True)
            self._thread.start()

    @property
    def state(self):
        """The most recently requested state."""
        with self._lock:
            return CLOSED if self._closed else self._desired

    @property
    def last_error(self):
        with self._lock:
            return self._last_error

    def start_driving(self):
        """Start the main track unless it is already requested."""
        self._request(DRIVING)

    def goal_reached(self):
        """Stop the main track and play the finish track exactly once."""
        self._request(GOAL)

    def stop(self):
        """Stop playback unless silence is already requested."""
        self._request(STOPPED)

    def close(self, timeout=2.0):
        """Stop playback and finish the worker thread.

        This is the only waiting method and is intended for the driving tool's
        ``finally`` block, after motor shutdown is already underway.
        """
        with self._lock:
            if self._closed:
                return
            self._closed = True
        if not self.enabled or self._thread is None:
            return
        self._commands.put(_CLOSE)
        self._thread.join(timeout=timeout)
        if self._thread.is_alive():
            self._warn("audio worker did not stop within %.1f s" % timeout)

    def _request(self, state):
        if not self.enabled:
            return
        with self._lock:
            if self._closed or self._desired == state:
                return
            self._desired = state
        self._commands.put(state)

    def _run(self):
        process = None
        active_state = STOPPED

        while True:
            try:
                command = self._commands.get(timeout=0.1)
            except queue.Empty:
                if process is not None and process.poll() is not None:
                    if process.returncode:
                        self._warn(
                            "audio player exited with status %s while %s"
                            % (process.returncode, active_state))
                    process = None
                    active_state = STOPPED
                continue

            if process is not None:
                self._stop_process(process)
                process = None
                active_state = STOPPED

            if command is _CLOSE:
                return
            if command == STOPPED:
                continue

            path = self.drive_audio if command == DRIVING else self.goal_audio
            if not path.is_file():
                self._warn("audio file not found: %s" % path)
                continue

            argv = [self.player, "--volume", "%.3f" % self.volume]
            if self.target:
                argv.extend(("--target", str(self.target)))
            argv.append(str(path))

            try:
                process = self._popen(argv, stdout=subprocess.DEVNULL)
                active_state = command
            except (OSError, subprocess.SubprocessError) as exc:
                self._warn("could not start audio player: %s" % exc)

    def _stop_process(self, process):
        if process.poll() is not None:
            return
        try:
            process.terminate()
            process.wait(timeout=0.5)
        except subprocess.TimeoutExpired:
            process.kill()
            try:
                process.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                self._warn("audio player did not exit after being killed")
        except OSError as exc:
            self._warn("could not stop audio player: %s" % exc)

    def _warn(self, message):
        text = "warning:  audio: %s" % message
        with self._lock:
            self._last_error = text
        try:
            self._warn_callback(text)
        except Exception:
            # Diagnostics must not become a failure path into vehicle control.
            pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

