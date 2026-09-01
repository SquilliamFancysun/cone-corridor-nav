"""Autonomous corridor following. Runs on the car, and drives it.

    python drive_corridor.py --weights ~/models/best.pt --dry-run
    python drive_corridor.py --weights ~/models/best.pt --steer-only
    python drive_corridor.py --weights ~/models/best.pt

This is `fusion_view.py` with a steering wheel attached. The perception half is
imported from it rather than copied, so there is exactly one definition of the
pipeline and a divergence between what you watched in Foxglove and what the car
then did is not possible.

## Safety, which is most of this file

Nothing here moves unless a human is holding a button down. `--dry-run` computes
and logs everything and never opens the VESC at all; `--steer-only` moves the
servo with the throttle pinned to zero, which is how the steering sign gets
checked on a stand. Full driving is the third thing you run, not the first.

The motor is commanded to zero on every path out of the loop: deadman released,
scan gone stale, centerline empty, exception, Ctrl-C, normal exit. `finally`
does it again in case any of those was missed.

## Ownership

Only one process can hold each device. Before running this, DonkeyCar must be
stopped -- it holds `/dev/ttyACM0` -- and so must anything else on the camera or
the lidar. Check with `fuser -v /dev/ttyACM0 /dev/ttyUSB0`.

## The detector runs on its own thread

`fusion_view.py` calls the detector inline, which is right for a tool you watch
and wrong for a loop that steers: at ~111 ms per frame the control loop would
inherit the inference period and drop scans. Here a background thread owns the
camera and publishes the newest DetectionSet; the control loop reads whatever is
current and lets `fusion.MAX_DETECTION_AGE_S` reject anything too old. That
bound is 300 ms and inference is ~111 ms, so in normal running nothing is
rejected -- the thread exists to decouple the rates, not to paper over them.
"""

import argparse
import json
import math
import os
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if not os.path.isdir(os.path.join(_HERE, "cone_perception")):
    _REPO = os.path.normpath(os.path.join(_HERE, "..", ".."))
    for _pkg in ("cone_perception", "cone_nav"):
        _src = os.path.join(_REPO, "ros2", "src", _pkg)
        if os.path.isdir(_src) and _src not in sys.path:
            sys.path.insert(0, _src)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import detectors
import fusion_view
import oakd
from cone_nav.control import pure_pursuit, speed_ctrl
from cone_nav.corridor.centerline import centerline
from cone_nav.corridor.side_assign import fill_unlabeled, heading_of
from cone_perception import clustering, extrinsics, fusion
from joystick import Joystick, JoystickNotFound
from lidar_view import BAUD, DEFAULT_PORT, open_serial

DEFAULT_PORT_WS = 8769

# The VESC, by a path that is tied to the device rather than to enumeration
# order. `/dev/ttyACM0` is POSITIONAL -- docs/hardware-baseline.md records that
# adding or moving anything on the bus renumbers it -- and this car reboots at
# the track like any other. myconfig_capture.py still names ttyACM0 because
# DonkeyCar's config is not ours to churn mid-session; lidar_view.py already
# uses the by-id form for the LD06, and this is the same rule applied to the one
# device that can drive the car.
#
# Falls back to the positional name if the by-id link is absent, so this still
# starts on a car whose serial number differs from ours -- with the reasoning
# recorded here rather than the fallback looking like the intended path.
DEFAULT_VESC_BY_ID = ("/dev/serial/by-id/"
                      "usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00")
FALLBACK_VESC_PORT = "/dev/ttyACM0"


def default_vesc_port():
    return (DEFAULT_VESC_BY_ID if os.path.exists(DEFAULT_VESC_BY_ID)
            else FALLBACK_VESC_PORT)

# The F710 button held to enable autonomy. X, per docs/hardware-baseline.md:
# DonkeyCar binds A to emergency_stop and B to toggle_manual_recording, and
# every process reading /dev/input/js0 sees every press. X is the index this
# project reserves for its own tools.
DEADMAN_BUTTON = 2

# A scan older than this and the car is steering on history. Two revolutions at
# 10 Hz: one missed scan is a hiccup, two in a row is a fault.
MAX_SCAN_AGE_S = 0.25

DRIVE_STATUS_SCHEMA = {
    "type": "object",
    "title": "DriveStatus",
    "properties": {
        "armed": {"type": "boolean", "description": "deadman held"},
        "mode": {"type": "string"},
        "duty": {"type": "number"},
        "steer_deg": {"type": "number", "description": "commanded, left positive"},
        "steer_normalised": {"type": "number", "description": "raw, before smoothing"},
        "steer_commanded": {"type": "number", "description": "after the median filter -- what the servo got"},
        "servo": {"type": "number", "description": "what the VESC was told, 0-1"},
        "lookahead_m": {"type": "number"},
        "target_x": {"type": "number"},
        "target_y": {"type": "number"},
        "reach_m": {"type": "number"},
        "stop_reason": {"type": "string"},
        "short_line": {"type": "boolean"},
        "cones": {"type": "integer"},
        "labeled_by_camera": {"type": "integer"},
        "labeled_by_geometry": {"type": "integer"},
        "candidates": {"type": "integer", "description": "lidar clusters offered to fusion"},
        "detections": {"type": "integer", "description": "boxes offered to fusion"},
        "out_of_fov": {"type": "integer", "description": "clusters the camera structurally could not see"},
        "unmatched_in_fov": {"type": "integer", "description": "clusters IN frame that got no box -- a detector miss or a bearing error"},
        "unmatched_detections": {"type": "integer", "description": "boxes no cluster claimed"},
        "detections_stale": {"type": "boolean", "description": "the boxes were older than max_detection_age"},
        "centerline_points": {"type": "integer"},
        "single_boundary_fallback": {"type": "boolean"},
        "scan_age_s": {"type": "number"},
        "detection_age_s": {"type": "number"},
        "loop_hz": {"type": "number"},
    },
}


class ThreadedDetector(threading.Thread):
    """Owns the camera, runs inference, publishes the newest DetectionSet.

    The loop never waits on this. `latest()` returns whatever the last completed
    inference produced, and the age carried inside the DetectionSet is what
    decides whether it is still usable -- so a slow frame degrades labelling
    rather than stalling the car.
    """

    daemon = True

    def __init__(self, detector, queue):
        super().__init__(name="detector")
        self.detector = detector
        self.queue = queue
        self._latest = None
        self._lock = threading.Lock()
        # NOT `self._stop`. threading.Thread has a private `_stop()` METHOD,
        # and Thread.join -> _wait_for_tstate_lock calls it once the thread has
        # finished. An attribute of that name shadows it, so join() raises
        # `TypeError: 'Event' object is not callable` -- on the way out of a
        # tool, from inside `finally`, after the run is over. Under torch it
        # comes out as a C++ std::terminate and a SIGABRT over the top of the
        # run's own summary. Observed on the car; see test_the_stop_flag_does_
        # not_shadow_threads_own_stop.
        self._stopping = threading.Event()
        self.frames = 0
        self.last_error = None

    def run(self):
        while not self._stopping.is_set():
            frame = self.queue.tryGet()
            if frame is None:
                time.sleep(0.005)
                continue
            captured = time.monotonic()
            try:
                detections = self.detector.detect(frame.getCvFrame(), captured)
            except Exception as exc:  # a bad frame must not take the car down
                self.last_error = str(exc)
                continue
            with self._lock:
                self._latest = detections
                self.frames += 1

    def latest(self):
        with self._lock:
            return self._latest

    def stop(self):
        self._stopping.set()


class Deadman:
    """The F710 button that has to be held for the car to move.

    Absence of the pad is treated as "not held", not as an error: a receiver
    knocked out mid-run must stop the car, and the joydev read simply stops
    producing events when that happens.
    """

    def __init__(self, path="/dev/input/js0", button=DEADMAN_BUTTON):
        self.button = button
        self.held = False
        self.present = False
        self.joystick = None
        try:
            self.joystick = Joystick(path)
            self.present = True
        except JoystickNotFound as exc:
            self.reason = str(exc)

    def poll(self):
        if self.joystick is None or not self.joystick.connected:
            self.held = False
            return False
        for event in self.joystick.poll():
            if event.is_button and event.number == self.button:
                self.held = bool(event.value)
        if not self.joystick.connected:
            self.held = False
        return self.held

    def close(self):
        if self.joystick is not None:
            self.joystick.close()


class VescDriver:
    """Steering and throttle to the VESC, and a stop that always works.

    Mirrors DonkeyCar's VESC part, whose constants live in
    `myconfig_capture.py`: servo = angle * 0.5 + 0.5, duty = throttle * 0.2.
    Those are this car's tuned values and are reused rather than re-derived.

    THE STEERING SIGN IS NOT KNOWN UNTIL IT IS TESTED. Our convention is left
    positive (REP-103, and every frame in this repo); which way that turns the
    wheels depends on how the servo is installed. `--invert-steering` exists for
    exactly that, and `--steer-only` on a stand is how you find out which you
    need. Do not skip it: a mirrored sign tracks a straight corridor perfectly
    and turns the wrong way at the first bend.
    """

    def __init__(self, port, steering_scale=0.5, steering_offset=0.5,
                 max_duty_percent=0.2, invert_steering=False, baudrate=115200,
                 timeout=0.05, has_sensor=True):
        from pyvesc import VESC

        self.steering_scale = steering_scale
        self.steering_offset = steering_offset
        self.max_duty_percent = max_duty_percent
        self.invert = -1.0 if invert_steering else 1.0
        self.last_servo = steering_offset
        self.vesc = VESC(serial_port=port, has_sensor=has_sensor,
                         start_heartbeat=True, baudrate=baudrate,
                         timeout=timeout)

    def servo_for(self, normalised_steer):
        value = self.invert * normalised_steer * self.steering_scale + self.steering_offset
        return min(1.0, max(0.0, value))

    def drive(self, normalised_steer, duty):
        servo = self.servo_for(normalised_steer)
        self.vesc.set_servo(servo)
        self.last_servo = servo
        # duty here is already an absolute duty fraction from speed_ctrl, not
        # DonkeyCar's -1..1 throttle, so max_duty_percent is a ceiling rather
        # than a scale factor. Applying it as a scale would silently run the
        # car at a fifth of the commanded speed.
        self.vesc.set_duty_cycle(min(duty, self.max_duty_percent))
        return servo

    def stop(self):
        """Zero throttle, wheels centred. Safe to call repeatedly and after a
        failure -- every exception path calls it, including ones where the
        serial link may already be gone."""
        try:
            self.vesc.set_duty_cycle(0.0)
            self.vesc.set_servo(self.steering_offset)
        except Exception:
            pass

    def close(self):
        self.stop()
        try:
            self.vesc.stop_heartbeat()
        except Exception:
            pass


class TrialLog:
    """One JSON object per control tick, for analysis/ and for the report.

    Written as it goes rather than buffered: a run that ends by someone lunging
    for the car is exactly the run worth having the data from.
    """

    def __init__(self, path):
        self.path = path
        self.handle = open(path, "w", encoding="utf-8") if path else None
        self.rows = 0

    def write(self, record):
        if self.handle is None:
            return
        self.handle.write(json.dumps(record) + "\n")
        self.rows += 1
        if self.rows % 20 == 0:
            self.handle.flush()

    def close(self):
        if self.handle is not None:
            self.handle.flush()
            self.handle.close()


def drive_pipeline(scan, detection_set, calibration, intr, args, now,
                   axis_rad=0.0):
    """One revolution -> cones, centerline, steering, duty.

    The perception half is `fusion_view.pipeline_once` unchanged. What is added
    is the near-field fill and the two control layers, and they are added HERE
    rather than inside fusion_view so that the viewing tool stays a viewing
    tool.
    """
    candidates = clustering.cone_candidates(scan, calibration,
                                            max_range_m=args.max_range)
    age = detection_set.age(now) if detection_set is not None else float("inf")
    detections = detection_set.detections if detection_set is not None else []
    result = fusion.associate(candidates, detections, intr,
                              detection_age_s=age,
                              max_bearing_err_deg=args.bearing_gate,
                              max_detection_age_s=args.max_detection_age)

    cones, filled = result.cones, 0
    if not args.no_fill:
        cones, filled = fill_unlabeled(cones, reference_heading_rad=axis_rad,
                                       fill_in_fov=args.no_camera)

    line = centerline(cones, car_xy=(0.0, 0.0))
    axle = extrinsics.REAR_AXLE_IN_BASE
    pursuit = pure_pursuit.steering_angle(line.points, args.lookahead,
                                          extrinsics.WHEELBASE_M, origin=axle)
    duty = speed_ctrl.duty(pursuit, line, max_duty=args.max_duty, origin=axle)
    return result, cones, filled, line, pursuit, duty


def status_of(result, cones, filled, line, pursuit, duty, servo, armed, args,
              scan_age, detection_age, loop_hz, commanded=0.0):
    return {
        "armed": bool(armed),
        "mode": args.mode,
        "duty": round(duty.duty, 4),
        "steer_deg": round(math.degrees(pursuit.delta_rad), 2) if pursuit else 0.0,
        "steer_normalised": round(pursuit.normalised, 4) if pursuit else 0.0,
        "steer_commanded": round(commanded, 4),
        "servo": round(servo, 4),
        "lookahead_m": args.lookahead,
        "target_x": round(pursuit.target[0], 3) if pursuit else 0.0,
        "target_y": round(pursuit.target[1], 3) if pursuit else 0.0,
        "reach_m": round(duty.reach_m, 3),
        "stop_reason": duty.reason,
        "short_line": bool(pursuit.short_line) if pursuit else False,
        "cones": len(cones),
        "labeled_by_camera": result.matched,
        "labeled_by_geometry": filled,
        # fusion.py's own diagnostic counters, which its docstring promises the
        # harness reports. fusion_view.py logs them to Foxglove; they belong in
        # the trial log too, because a cone that is missing from the scene and a
        # cone that is present but unlabelled are different faults with
        # different fixes and `labeled_by_camera` alone cannot separate them.
        # `unmatched_in_fov` is the one to read first: a cluster the camera
        # could see and did not explain is a detector miss or a bearing error,
        # and nothing else in the record says so.
        "candidates": result.candidates,
        "detections": result.detections,
        "out_of_fov": result.out_of_fov,
        "unmatched_in_fov": result.unmatched_in_fov,
        "unmatched_detections": result.unmatched_detections,
        "detections_stale": bool(result.stale),
        "centerline_points": len(line.points),
        "single_boundary_fallback": bool(line.single_boundary_fallback),
        "scan_age_s": round(scan_age, 3),
        "detection_age_s": round(detection_age, 3) if detection_age != float("inf") else -1.0,
        "loop_hz": round(loop_hz, 1),
    }


def pad_health(deadman):
    """The bench-line tag for a deadman pad that died mid-run.

    A USB over-current trip disconnects the F710 along with everything else,
    and it re-enumerates as a NEW input device -- the running tool still holds
    the dead one, so every press after that moment reads as 'not held'. That
    is the safe reading and the silent one: the operator stands there pressing
    X at a line that says 'idle'. Observed on the track 2026-09-01. The pad
    cannot be reacquired mid-run (the fd is gone); the fix is a restart, and
    the tag says so.
    """
    if deadman is None or not deadman.present or deadman.joystick is None:
        return ""
    if deadman.joystick.connected:
        return ""
    return "PAD LOST -- X does nothing; restart the tool"


def camera_health(detection_age_s, max_age_s, grace_s=3.0):
    """The bench-line tag for a detector that has stopped producing.

    Exists because of a run where the detector thread died on an X_LINK_ERROR
    five seconds in and the operator pushed the whole course reading
    '[no reds]' -- which was true, and useless: the camera had been gone for
    forty seconds and nothing on the once-a-second line said so. The traceback
    printed once and scrolled away. A stale detector and an empty scene are
    different facts, and the line must not let them read the same.

    Returns '' while frames are fresh. The grace period covers startup, where
    the first inference legitimately takes a couple of seconds.
    """
    if detection_age_s <= max(grace_s, max_age_s):
        return ""
    if detection_age_s == float("inf"):
        return "CAMERA: NO FRAMES YET"
    return f"CAMERA STALE {detection_age_s:.0f}s -- labels are dead reckoning"


def require_vehicle_geometry():
    """Refuse to run without the two measurements pure pursuit needs.

    Fatal, not a warning, and for the same reason `fusion_view.resolve_calibration`
    is fatal about the lidar's bearing sign: this tool ACTUATES. A guessed
    wheelbase produces steering that looks entirely reasonable in the logs and
    is wrong by an unknown factor at every angle.
    """
    problems = extrinsics.check_vehicle_measured()
    if problems:
        lines = "\n".join(f"       - {p}" for p in problems)
        raise SystemExit(
            "error: the vehicle geometry has never been measured.\n" + lines +
            "\n\n       Take a tape to the car and record both in\n"
            "       ros2/src/cone_perception/cone_perception/extrinsics.py:\n"
            "         REAR_AXLE_IN_BASE = (x, y, z)  # rear axle in base_link;\n"
            "                                        # base_link is AT THE LIDAR,\n"
            "                                        # so x is negative\n"
            "         WHEELBASE_M = 0.00             # front axle to rear axle\n"
            "       Then re-run deploy.sh.")


def build_parser(description="Autonomous cone-corridor following."):
    """Every argument the two driving scripts share.

    Split out of `parse_args` so `drive_junction.py` can add `--route` to the
    same parser rather than restating twenty-five options that must stay in
    step with these. Pure refactor: `parse_args` below is what it always was.
    """
    parser = argparse.ArgumentParser(description=description)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true",
                      help="compute and log everything; never open the VESC")
    mode.add_argument("--steer-only", action="store_true",
                      help="move the servo, throttle pinned at zero. Run this "
                           "on a stand before anything else")

    parser.add_argument("--weights", default="best.pt")
    parser.add_argument("--detector", default="ultralytics",
                        choices=("ultralytics", "blob"))
    parser.add_argument("--imgsz", type=int, default=detectors.DEFAULT_IMGSZ)
    parser.add_argument("--conf", type=float, default=detectors.DEFAULT_CONF)
    parser.add_argument("--device", default=None)
    parser.add_argument("--camera-fps", type=float, default=15.0)
    parser.add_argument("--no-camera", action="store_true",
                        help="drive on the lidar alone: every unlabelled "
                             "cluster gets a side from geometry. Valid on a "
                             "plain corridor, WRONG at a fork")

    parser.add_argument("--lookahead", type=float, default=1.0,
                        help="metres. 1.0 is the sim's robust pick; shorter "
                             "weaves, longer cuts corners")
    parser.add_argument("--max-duty", type=float,
                        default=speed_ctrl.DEFAULT_MAX_DUTY,
                        help="duty cycle ceiling. Start at the floor")
    parser.add_argument("--invert-steering", action="store_true",
                        help="flip the servo sign. Decide this on a stand with "
                             "--steer-only, never on the track")
    parser.add_argument("--smooth-window", type=int,
                        default=pure_pursuit.SMOOTH_WINDOW,
                        help="median window on the steering command. 5 suits a "
                             "slow first run; drop it as --max-duty goes up, "
                             "since the lag costs more the faster the car moves")
    parser.add_argument("--no-fill", action="store_true",
                        help="disable near-field geometric side assignment")

    parser.add_argument("--vesc-port", default=default_vesc_port(),
                        help="defaults to the VESC's by-id path, which survives "
                             "a reboot renumbering ttyACM*")
    parser.add_argument("--port", default=DEFAULT_PORT, help="LD06 serial port")
    parser.add_argument("--joystick", default="/dev/input/js0")
    parser.add_argument("--no-deadman", action="store_true",
                        help="run without the F710. Refused unless --dry-run")

    parser.add_argument("--max-range", type=float,
                        default=clustering.MAX_CONE_RANGE_M)
    parser.add_argument("--bearing-gate", type=float,
                        default=fusion.MAX_BEARING_ERR_DEG)
    parser.add_argument("--max-detection-age", type=float,
                        default=fusion.MAX_DETECTION_AGE_S)
    parser.add_argument("--bins", type=int, default=fusion_view.DEFAULT_BINS)
    parser.add_argument("--ws-port", type=int, default=DEFAULT_PORT_WS)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--no-live", action="store_true")
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--calibration", default=None)
    parser.add_argument("--log", default=None,
                        help="path for the per-tick JSONL trial log")

    return parser


def finalise_args(parser, args):
    """The mode derivation and the refusals, shared with drive_junction.py."""
    args.mode = ("dry-run" if args.dry_run else
                 "steer-only" if args.steer_only else "drive")
    if args.no_deadman and not args.dry_run:
        parser.error("--no-deadman is only allowed with --dry-run")
    return args


def parse_args(argv=None):
    parser = build_parser()
    return finalise_args(parser, parser.parse_args(argv))


def announce(args, record, intr, detector_name):
    print(f"mode      {args.mode.upper()}"
          + ("   (no VESC will be opened)" if args.dry_run else "")
          + ("   (throttle pinned to zero)" if args.steer_only else ""))
    print(f"vehicle   wheelbase {extrinsics.WHEELBASE_M} m, rear axle at "
          f"{extrinsics.REAR_AXLE_IN_BASE} in base_link")
    print(f"control   lookahead {args.lookahead} m, max duty {args.max_duty}, "
          f"steering {'INVERTED' if args.invert_steering else 'normal'}")
    print(f"lidar     mirror={record['mirror']} "
          f"angle_offset={record['angle_offset_deg']} deg")
    print(f"detector  {detector_name}, gate {args.bearing_gate} deg")
    if args.no_camera:
        print("warning:  --no-camera: EVERY unlabelled cluster is being given a "
              "side by geometry.\n"
              "          Correct on a plain corridor. Wrong at a fork.")
    if args.no_fill:
        print("warning:  --no-fill: the near blind spot is unlabelled, so the "
              "corridor will be\n"
              "          short unless the cones are spaced under 1 m.")
    for warning in fusion.startup_warnings():
        print(f"warning:  {warning}")


def main(argv=None):
    args = parse_args(argv)
    require_vehicle_geometry()

    record = fusion_view.resolve_calibration(args)

    detector = detectors.build(args.detector, weights=args.weights,
                               imgsz=args.imgsz, conf=args.conf,
                               device=args.device)
    device, (width, height) = oakd.open_camera(args.camera_fps)
    intr = oakd.camera_intrinsics(device, width, height)
    announce(args, record, intr, detector.name)

    deadman = Deadman(args.joystick)
    if not deadman.present and not args.no_deadman:
        device.close()
        raise SystemExit(
            f"error: no joystick at {args.joystick}. The deadman is not "
            "optional --\n       nothing may drive itself with no way to stop "
            "it. Switch the F710\n       on, or pass --dry-run --no-deadman to "
            "compute without actuating.")

    handle = open_serial(args.port, BAUD)
    reader = fusion_view.LidarReader(handle)
    reader.start()

    q_preview = device.getOutputQueue("preview", maxSize=1, blocking=False)
    oakd.lock_camera(device.getInputQueue("control"))
    detector_thread = ThreadedDetector(detector, q_preview)
    detector_thread.start()

    sinks = fusion_view.Sinks()
    drive_ch = sinks.channel("/drive_status", DRIVE_STATUS_SCHEMA)
    if not sinks.available:
        print(f"warning:  no Foxglove sink ({sinks.reason}); running headless")

    server = None
    if sinks.available and not args.no_live:
        server = sinks.start_server(args.host, args.ws_port)
        print(f"\nFoxglove: ws://<car-ip>:{args.ws_port}  (desktop app)\n")

    vesc = None
    if not args.dry_run:
        try:
            vesc = VescDriver(args.vesc_port,
                              invert_steering=args.invert_steering)
        except Exception as exc:
            reader.stop(); handle.close(); detector_thread.stop(); device.close()
            raise SystemExit(
                f"error: could not open the VESC on {args.vesc_port}: {exc}\n"
                "       Is DonkeyCar still running? Only one process may hold "
                "that port.\n       Check with: fuser -v /dev/ttyACM0")

    log = TrialLog(args.log)
    if args.log:
        print(f"logging every tick to {args.log}")
    print("hold X on the F710 to arm. Release to stop. Ctrl-C to quit.\n")

    started = time.monotonic()
    axis_rad = 0.0
    duty_now = 0.0
    steer_history = []
    last_scan_at = started
    last_report = started
    loops = 0

    try:
        while True:
            now = time.monotonic()
            if args.duration and now - started >= args.duration:
                break

            armed = deadman.poll() if deadman.present else bool(args.no_deadman)

            scan = reader.take()
            if scan is None:
                # No new revolution. If it has been too long the car is steering
                # on history, so stop -- but keep looping, because the lidar
                # usually comes back and a stopped car can recover.
                if vesc is not None and now - last_scan_at > MAX_SCAN_AGE_S:
                    vesc.stop()
                    duty_now = 0.0
                time.sleep(0.005)
                continue

            scan_age = now - last_scan_at
            last_scan_at = now
            loops += 1

            detection_set = detector_thread.latest()
            result, cones, filled, line, pursuit, duty = drive_pipeline(
                scan, detection_set, record, intr, args, now, axis_rad)
            axis_rad = heading_of(line, default=axis_rad)

            # Every gate that can stop the car, in one place.
            target_duty = duty.duty
            if not armed:
                target_duty = 0.0
            if args.steer_only or args.dry_run:
                target_duty = 0.0
            duty_now = speed_ctrl.ramp(duty_now, target_duty)

            # Median-filter before the servo ever sees it. The raw command is
            # quiet with discrete slams -- see SMOOTH_WINDOW in pure_pursuit --
            # and those slams are the chain flickering, not the corridor moving.
            steer_history, steer = pure_pursuit.smooth(
                steer_history,
                pursuit.normalised if pursuit is not None else None,
                window=args.smooth_window)
            servo = 0.0
            if vesc is not None:
                if armed:
                    servo = vesc.drive(steer, duty_now)
                else:
                    vesc.stop()
                    servo = vesc.last_servo

            detection_age = (detection_set.age(now)
                             if detection_set is not None else float("inf"))
            elapsed = now - started
            status = status_of(result, cones, filled, line, pursuit, duty,
                               servo, armed, args, scan_age, detection_age,
                               loops / elapsed if elapsed else 0.0,
                               commanded=steer)

            wall = time.time()
            if sinks.available:
                sinks.log_transforms(wall)
                sinks.log_scan(scan, wall, args.bins, record["mirror"],
                               record["angle_offset_deg"])
                sinks.log_cones(cones, wall)
                sinks.log_centerline(line, wall)
                drive_ch.log(status, log_time=int(wall * 1e9))
            log.write(dict(status, t=round(elapsed, 3), wall=wall))

            if now - last_report >= 1.0:
                last_report = now
                flag = "ARMED " if armed else "idle  "
                health = camera_health(detection_age,
                                       args.max_detection_age)
                health = " / ".join(t for t in (health, pad_health(deadman))
                                    if t)
                print(f"  {flag} duty {duty_now:.3f}  steer "
                      f"{status['steer_deg']:+6.1f} deg  "
                      f"{len(line.points)} pts, reach {duty.reach_m:.2f} m  "
                      f"cones {result.matched}cam/{filled}geo/{len(cones)}"
                      + (f"  [{duty.reason}]" if duty.reason else "")
                      + (f"  !! {health}" if health else ""))
    except KeyboardInterrupt:
        print("\nstopping")
    except Exception:
        # Stop the car before the traceback prints, not after.
        if vesc is not None:
            vesc.stop()
        raise
    finally:
        if vesc is not None:
            vesc.close()
        reader.stop()
        detector_thread.stop()
        # Join before closing the port. stop() only sets a flag, and the reader
        # spends most of its life blocked in handle.read(); closing underneath
        # it makes pyserial read a None fd and raise TypeError out of a thread,
        # which prints a traceback over the run's own summary. The read timeout
        # is 50 ms, so this returns promptly or the thread was already gone.
        reader.join(timeout=1.0)
        handle.close()
        device.close()
        deadman.close()
        log.close()
        if server is not None:
            server.stop()
        if log.path:
            print(f"wrote {log.rows} ticks to {log.path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
