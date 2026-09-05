"""Gamepad-triggered cone image capture from the OAK-D Lite. Runs on the car.

Owns the camera; DonkeyCar must be running with CAMERA_TYPE="MOCK" (see
myconfig_capture.py) so it drives the VESC without claiming the OAK-D. Only one
process can hold the device.

The DepthAI pipeline here is the reference camera configuration: whatever
detectors.py does at inference time should match it, or the detector sees
different colors than it trained on.

    python capture_cones.py --session-label lot-sun-A
"""

import argparse
import os
import sys
import time

import depthai as dai

from joystick import DEFAULT_DEVICE, Joystick, JoystickNotFound
from session import FrameClock, SessionWriter

RESOLUTIONS = {
    "1080p": (dai.ColorCameraProperties.SensorResolution.THE_1080_P, 1920, 1080),
    "4k": (dai.ColorCameraProperties.SensorResolution.THE_4_K, 3840, 2160),
}


def build_pipeline(args):
    """ColorCamera -> hardware MJPEG encoder -> host.

    Encoding on-device matters twice over: the Pi never touches cv2.imencode,
    and the OAK-D here negotiates USB 2.0 (UsbSpeed.HIGH), where raw 1080p
    frames would not fit in the available bandwidth at any useful rate.
    """
    sensor_res, width, height = RESOLUTIONS[args.resolution]
    pipeline = dai.Pipeline()

    cam = pipeline.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam.setResolution(sensor_res)
    cam.setFps(args.camera_fps)
    cam.setInterleaved(False)
    # Native 16:9 all the way through. DonkeyCar's part squeezes this into 4:3;
    # we keep the sensor's real geometry so training frames and the deployed
    # letterbox agree.
    cam.setPreviewSize(416, 234)
    cam.setPreviewKeepAspectRatio(True)

    encoder = pipeline.create(dai.node.VideoEncoder)
    encoder.setDefaultProfilePreset(
        args.camera_fps, dai.VideoEncoderProperties.Profile.MJPEG
    )
    encoder.setQuality(args.quality)
    cam.video.link(encoder.input)

    xout_jpeg = pipeline.create(dai.node.XLinkOut)
    xout_jpeg.setStreamName("jpeg")
    encoder.bitstream.link(xout_jpeg.input)

    # Small side-channel purely to read back what auto-exposure/AWB settled on.
    # Encoded frames do not reliably carry those fields.
    xout_meta = pipeline.create(dai.node.XLinkOut)
    xout_meta.setStreamName("meta")
    cam.preview.link(xout_meta.input)

    xin_ctrl = pipeline.create(dai.node.XLinkIn)
    xin_ctrl.setStreamName("control")
    xin_ctrl.out.link(cam.inputControl)

    return pipeline, width, height


def frame_settings(frame):
    """Exposure/ISO/WB/focus off one preview frame."""
    settings = {}
    for key, getter in (
        ("exposure_us", "getExposureTime"),
        ("iso", "getSensitivity"),
        ("wb_kelvin", "getColorTemperature"),
        ("lens_position", "getLensPosition"),
    ):
        try:
            value = getattr(frame, getter)()
        except (AttributeError, RuntimeError):
            continue
        if key == "exposure_us":
            value = int(value.total_seconds() * 1e6)
        settings[key] = value
    return settings


def read_settings(q_meta, timeout_s=2.0):
    """Settings from the most recent preview frame, draining anything stale.

    Taking the first frame off the queue would report state from before
    auto-exposure converged.
    """
    deadline = time.monotonic() + timeout_s
    latest = None
    while time.monotonic() < deadline:
        frame = q_meta.tryGet()
        if frame is None:
            if latest is not None:
                break
            time.sleep(0.02)
            continue
        latest = frame
    return frame_settings(latest) if latest is not None else {}


def settle(q_jpeg, q_meta, seconds):
    """Run AE/AWB/AF free for a while; report where they ended up.

    Autofocus reports lens_position 0 until it has actually converged, so the
    last non-zero position seen during settling is what gets pinned — reading a
    single frame at the end can catch it mid-hunt and pin the lens to 0, which
    is not a focus distance, it is "no answer yet".
    """
    deadline = time.monotonic() + seconds
    latest = {}
    last_focus = None
    while time.monotonic() < deadline:
        q_jpeg.tryGet()
        frame = q_meta.tryGet()
        if frame is not None:
            latest = frame_settings(frame)
            if latest.get("lens_position"):
                last_focus = latest["lens_position"]
        time.sleep(0.02)
    if last_focus is not None:
        latest["lens_position"] = last_focus
    return latest


def apply_camera_control(q_ctrl, args, settled):
    """Pin exposure, white balance and focus for the session.

    The four classes ARE colors, so an AWB algorithm that re-balances between a
    sunlit and a shaded stretch will shift cone hue underneath the labels. One
    settled-then-locked setting per session removes that. Focus is pinned for
    the same reason: autofocus hunting mid-run yields soft frames at random.
    """
    ctrl = dai.CameraControl()
    mode = []
    if args.exposure_us is not None or args.iso is not None:
        exposure = args.exposure_us if args.exposure_us is not None else 8000
        iso = args.iso if args.iso is not None else 400
        ctrl.setManualExposure(exposure, iso)
        mode.append(f"manual exposure {exposure}us @ ISO {iso}")
    else:
        ctrl.setAutoExposureLock(True)
        mode.append("AE locked")

    if args.wb_kelvin is not None:
        ctrl.setManualWhiteBalance(args.wb_kelvin)
        mode.append(f"manual WB {args.wb_kelvin}K")
    else:
        ctrl.setAutoWhiteBalanceLock(True)
        mode.append("AWB locked")

    # There is no "autofocus lock" — pinning the position autofocus already
    # settled on is the equivalent. A reported 0 means AF has not converged
    # rather than "focused at 0", so leave it hunting instead of pinning a lens
    # position that was never a real answer.
    focus = args.focus if args.focus is not None else settled.get("lens_position")
    if focus:
        ctrl.setManualFocus(int(focus))
        mode.append(f"focus pinned at {int(focus)}")
    else:
        mode.append("focus left on autofocus (never converged while settling)")

    q_ctrl.send(ctrl)
    return ", ".join(mode)


def tool_commit():
    """Commit stamped in by deploy.sh, since the car has no git repo."""
    version_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "VERSION")
    try:
        with open(version_file) as fh:
            return fh.read().strip() or None
    except OSError:
        return None


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--session-label", default="session",
                        help="describes the conditions, e.g. lot-sun-A; becomes the "
                             "directory name and the Roboflow batch name")
    parser.add_argument("--out-root", default="~/cone_capture")
    parser.add_argument("--rate", type=float, default=2.0,
                        help="frames saved per second (default: 2)")
    parser.add_argument("--camera-fps", type=float, default=10.0,
                        help="sensor rate; runs faster than --rate so auto-exposure "
                             "stays responsive (default: 10)")
    parser.add_argument("--resolution", choices=sorted(RESOLUTIONS), default="1080p")
    parser.add_argument("--quality", type=int, default=95, help="MJPEG quality 1-100")
    parser.add_argument("--settle", type=float, default=3.0,
                        help="seconds of auto AE/AWB/AF before locking; point the "
                             "camera at the track while it runs (default: 3)")
    parser.add_argument("--auto", action="store_true",
                        help="leave AE/AWB running free; NOT recommended for dataset "
                             "capture, colors will drift across the session")
    parser.add_argument("--exposure-us", type=int, default=None)
    parser.add_argument("--iso", type=int, default=None)
    parser.add_argument("--wb-kelvin", type=int, default=None)
    parser.add_argument("--focus", type=int, default=None,
                        help="lens position 0-255; default pins whatever autofocus "
                             "settled on, so it cannot hunt mid-session")
    parser.add_argument("--device", default=DEFAULT_DEVICE, help="joystick device node")
    parser.add_argument("--record-button", type=int, default=2,
                        help="joystick button that toggles recording; find it with "
                             "`python joystick.py --probe-buttons`. Default 2 = X, "
                             "which DonkeyCar leaves unbound — A is its E-Stop and B "
                             "its tub recorder, and every process on the pad sees "
                             "the same press (default: 2)")
    parser.add_argument("--no-joystick", action="store_true",
                        help="record immediately without a gamepad; needs --duration")
    parser.add_argument("--duration", type=float, default=None,
                        help="stop after this many seconds of recording")
    parser.add_argument("--preview", action="store_true",
                        help="settle, save one frame, print the locked settings, exit")
    parser.add_argument("--out", default="preview.jpg", help="path for --preview")
    parser.add_argument("--notes", default=None, help="free text into session.json")
    args = parser.parse_args(argv)

    if args.no_joystick and args.duration is None and not args.preview:
        parser.error("--no-joystick needs --duration (nothing would ever stop it)")
    if args.rate > args.camera_fps:
        parser.error(f"--rate {args.rate} exceeds --camera-fps {args.camera_fps}")
    return args


def main(argv=None):
    args = parse_args(argv)

    joystick = None
    if not args.no_joystick and not args.preview:
        try:
            joystick = Joystick(args.device)
        except JoystickNotFound as exc:
            raise SystemExit(
                f"error: {exc}\n"
                f"       (or run with --no-joystick --duration N to capture without one)"
            )

    pipeline, width, height = build_pipeline(args)

    with dai.Device(pipeline) as device:
        q_jpeg = device.getOutputQueue("jpeg", maxSize=8, blocking=False)
        q_meta = device.getOutputQueue("meta", maxSize=4, blocking=False)
        q_ctrl = device.getInputQueue("control")

        usb_speed = device.getUsbSpeed().name
        print(f"OAK-D up: {width}x{height} @ {args.camera_fps} fps, USB {usb_speed}")
        if usb_speed not in ("SUPER", "SUPER_PLUS"):
            print("  note: USB 2.0 link — on-device MJPEG keeps this well inside budget")

        print(f"Settling auto-exposure/white-balance/focus for {args.settle:.1f}s...")
        # Point the camera at the track while this runs — whatever it settles on
        # is what the whole session is locked to.
        settled = settle(q_jpeg, q_meta, args.settle)
        if args.auto:
            lock_mode = "AE/AWB/focus free-running (--auto)"
        else:
            lock_mode = apply_camera_control(q_ctrl, args, settled)
            time.sleep(0.4)
        settings = read_settings(q_meta) or settled
        print(f"Camera: {lock_mode}")
        if settings:
            print("  " + "  ".join(f"{k}={v}" for k, v in settings.items()))
        if settings.get("exposure_us", 0) > 15000 and args.exposure_us is None:
            print(f"  warning: {settings['exposure_us']}us is a long shutter. Fine if "
                  f"you are indoors or at dusk; outdoors it suggests something is "
                  f"wrong. --exposure-us/--iso override it.")

        if args.preview:
            frame = None
            deadline = time.monotonic() + 5.0
            while frame is None and time.monotonic() < deadline:
                frame = q_jpeg.tryGet()
                if frame is None:
                    time.sleep(0.02)
            if frame is None:
                raise SystemExit("error: no frame arrived within 5s")
            with open(os.path.expanduser(args.out), "wb") as fh:
                fh.write(frame.getData().tobytes())
            print(f"Wrote {args.out} — scp it back and look at it before a real run.")
            return 0

        meta = {
            "width": width,
            "height": height,
            "camera_fps": args.camera_fps,
            "target_rate_hz": args.rate,
            "jpeg_quality": args.quality,
            "camera": "OAK-D Lite RGB (IMX214)",
            "usb_speed": usb_speed,
            "lock_mode": lock_mode,
            "settled": settings,
            "track": "data/layouts/track_v1.md",
            "git_commit": tool_commit(),
        }

        writer = None
        clock = FrameClock(args.rate)
        recording = False
        recorded_s = 0.0
        started_at = None
        exit_code = 0

        def start():
            nonlocal recording, started_at, writer
            if writer is None:
                writer = SessionWriter(args.out_root, args.session_label, meta)
                print(f"\nSession: {writer.dir}")
            clock.reset()
            started_at = time.monotonic()
            recording = True

        def stop():
            nonlocal recording, recorded_s
            if recording and started_at is not None:
                recorded_s += time.monotonic() - started_at
            recording = False

        if args.no_joystick:
            start()
        else:
            print(f"\nReady. Button {args.record_button} toggles recording. Ctrl+C to finish.")

        try:
            while True:
                if joystick is not None:
                    if not joystick.connected:
                        print("\nJoystick disconnected — stopping recording.")
                        stop()
                        break
                    for event in joystick.poll():
                        if event.pressed and event.number == args.record_button:
                            stop() if recording else start()
                            state = "REC" if recording else "paused"
                            count = writer.count if writer else 0
                            print(f"\r{state:>6}  {count} frames" + " " * 20)

                frame = q_jpeg.tryGet()
                if frame is not None and recording:
                    now = time.monotonic()
                    if clock.should_keep(now):
                        writer.add_frame(
                            frame.getData().tobytes(),
                            t=now,
                            seq=frame.getSequenceNum(),
                        )
                        elapsed = recorded_s + (now - started_at)
                        sys.stdout.write(
                            f"\rREC ● {writer.name}  {writer.count} frames  "
                            f"{elapsed:6.1f}s"
                        )
                        sys.stdout.flush()

                if args.duration is not None and recording:
                    if recorded_s + (time.monotonic() - started_at) >= args.duration:
                        stop()
                        break

                time.sleep(0.005)
        except KeyboardInterrupt:
            stop()
            print("\nInterrupted.")
        finally:
            if joystick is not None:
                joystick.close()
            if writer is not None:
                path = writer.close(args.notes)
                print(f"\nWrote {writer.count} frames to {path}")
                print("Record the session and its conditions in "
                      "model/dataset/DATASET_CARD.md while you still remember them.")
            else:
                print("\nNothing recorded.")
                exit_code = 1

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
