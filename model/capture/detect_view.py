"""Live view of what the cone detector sees. Runs on the car, or at a desk.

    python detect_view.py --weights ~/models/best.pt

Then connect Foxglove Studio to ws://<car-ip>:8768 -- the car's IP or its
.local name, never the ssh alias. Port 8768, so this sits alongside
lidar_view.py (8765), depth_view.py (8766) and fusion_view.py (8767), though
the three camera tools all want a device only one process can hold.

This is the eyeball check and nothing more: does the model, on this camera,
right now, put the right coloured box on the right cone. evaluate.py answers
that question with numbers on a labelled split; that is the honest one, and
this is the one you can run while walking a cone around in front of the car.

Boxes are drawn in the cone's OWN colour, so a mislabel needs no legend to
spot -- a blue box on an orange cone is wrong at a glance. The pair that
matters is red and orange (a dead end read as a gate hands the car off to
junction_exec at a wall); walk those past the lens at a few ranges.

No lidar, no calibration, no ROS. With --frames it needs no camera either, so
a session pulled off the car can be replayed through the real model at a desk:

    python detect_view.py --weights ../training/v3/weights/best.pt \
        --frames ../dataset/images/20260827_1413_ebu2/frames --window
"""

import argparse
import glob
import os
import sys
import time
from contextlib import ExitStack

# On the car everything sits in one directory: deploy.sh drops cone_perception/
# beside this file. In a git checkout it lives under ros2/src/, and --frames is
# meant to be run at a desk, so find it there too rather than making the tool
# car-only.
_HERE = os.path.dirname(os.path.abspath(__file__))
if not os.path.isdir(os.path.join(_HERE, "cone_perception")):
    _SRC = os.path.normpath(os.path.join(_HERE, "..", "..", "ros2", "src",
                                         "cone_perception"))
    if os.path.isdir(_SRC) and _SRC not in sys.path:
        sys.path.insert(0, _SRC)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import detectors
import oakd
from cone_perception.cone_classes import (
    CLASS_BLUE,
    CLASS_MAGENTA,
    CLASS_NAMES,
    CLASS_ORANGE,
    CLASS_RED,
    CLASS_YELLOW,
    name_of,
)

DEFAULT_PORT_WS = 8768

# BGR, because cv2. Each box is drawn the colour of the cone it claims to be,
# which is what makes a wrong label visible rather than something you decode
# from a legend. Deliberately not imported from fusion_view's CONE_RGBA: that
# module pulls in the lidar and the whole corridor layer, and this tool is
# camera-only on purpose.
BOX_BGR = {
    CLASS_BLUE: (240, 90, 40),
    CLASS_MAGENTA: (215, 40, 190),
    CLASS_ORANGE: (25, 130, 245),
    CLASS_RED: (40, 40, 230),
    CLASS_YELLOW: (25, 215, 240),
}
UNKNOWN_BGR = (160, 160, 160)

STATUS_SCHEMA = {
    "type": "object",
    "title": "DetectStatus",
    "properties": {
        "frames": {"type": "integer"},
        "fps": {"type": "number", "description": "frames through the model per second"},
        "inference_ms": {"type": "number"},
        "boxes": {"type": "integer", "description": "detections in the latest frame"},
        "lowest_conf": {"type": "number",
                        "description": "weakest box on screen; near --conf means marginal"},
        "clipped": {"type": "integer",
                    "description": "boxes touching a frame edge; their bearing and range both lie"},
        "empty_frames": {"type": "integer",
                         "description": "frames with no detection at all, cumulative"},
    },
}
for _name in CLASS_NAMES:
    STATUS_SCHEMA["properties"][_name] = {
        "type": "integer", "description": f"{_name} boxes seen so far"}


def load_cv2():
    """cv2, or a refusal that names the fix. It is already in ~/env on the car."""
    try:
        import cv2
    except ImportError:
        raise SystemExit(
            "error: detect_view.py draws with cv2 and cannot find it.\n"
            "       ~/env/bin/pip install opencv-python-headless   (on the car)\n"
            "       pip install -r ../requirements.txt             (off-car)")
    return cv2


def check_weights_class_order(detector):
    """Refuse weights whose class order disagrees with the deployed constants.

    The same gate evaluate.py applies, for the same reason and one more. A
    permutation is invisible in training metrics, and here it would be worse
    than invisible: the tool would draw a blue box on a yellow cone and look
    exactly like a model that cannot tell the two apart. Checking the order
    first is what makes everything drawn afterwards mean something.
    """
    model = getattr(detector, "model", None)
    names = getattr(model, "names", None)
    if not names:
        return None
    if isinstance(names, dict):
        got = tuple(str(names[k]).lower() for k in sorted(names, key=int))
    else:
        got = tuple(str(n).lower() for n in names)
    if got == tuple(CLASS_NAMES):
        return got
    listing = ", ".join(f"{i}={n}" for i, n in enumerate(got))
    expected = ", ".join(f"{i}={n}" for i, n in enumerate(CLASS_NAMES))
    raise SystemExit(
        "error: these weights disagree with cone_perception/cone_classes.py\n"
        f"         weights:  {listing}\n"
        f"         expected: {expected}\n"
        "       Every box drawn here would carry another class's colour, so\n"
        "       there would be nothing useful to look at. Fix the class order\n"
        "       in the Roboflow project and retrain; never remap downstream.")


def draw_boxes(cv2, image, detections, conf_floor=None):
    """Boxes and labels onto a BGR image, in place. Normalised in, pixels out.

    Returns the number drawn. Coordinates come from geometry.Detection, which
    is centre-form and normalised precisely so this works the same whether the
    boxes came from ultralytics on the host or the blob on the device.
    """
    height, width = image.shape[:2]
    for det in detections:
        colour = BOX_BGR.get(det.cls, UNKNOWN_BGR)
        x1 = int(round((det.u - det.w / 2.0) * width))
        y1 = int(round((det.v - det.h / 2.0) * height))
        x2 = int(round((det.u + det.w / 2.0) * width))
        y2 = int(round((det.v + det.h / 2.0) * height))
        # A marginal box gets a thin outline instead of a thick one. It is
        # still drawn -- a cone the model is unsure about is the interesting
        # case, not the one to hide.
        weak = conf_floor is not None and det.confidence < conf_floor
        cv2.rectangle(image, (x1, y1), (x2, y2), colour, 1 if weak else 2)
        # The asterisk is the clipped flag: a box cut off by the frame edge has
        # a centre that has moved inward and a height cut short, so fusion's
        # bearing and range_from_bbox both lie about it.
        label = (f"{name_of(det.cls)} {det.confidence:.2f}"
                 + ("*" if det.clipped else ""))
        _draw_label(cv2, image, label, x1, y1, colour)
    return len(detections)


def _draw_label(cv2, image, text, x, y, colour):
    """Text on a filled chip of the box colour, flipped below the box near the top."""
    font, size, pad = cv2.FONT_HERSHEY_SIMPLEX, 0.4, 3
    (text_w, text_h), _ = cv2.getTextSize(text, font, size, 1)
    top = y - text_h - 2 * pad
    if top < 0:
        top = y
    cv2.rectangle(image, (x, top), (x + text_w + 2 * pad, top + text_h + 2 * pad),
                  colour, -1)
    cv2.putText(image, text, (x + pad, top + text_h + pad - 1), font, size,
                _text_bgr(colour), 1, cv2.LINE_AA)


def _text_bgr(bgr):
    """Black on a light chip, white on a dark one. White on yellow is unreadable."""
    b, g, r = bgr
    return (0, 0, 0) if 0.114 * b + 0.587 * g + 0.299 * r > 140 else (255, 255, 255)


def draw_hud(cv2, image, lines):
    """A few lines top-left, on a dark backdrop so they survive a bright frame."""
    font, size, step = cv2.FONT_HERSHEY_SIMPLEX, 0.42, 15
    overlay = image.copy()
    cv2.rectangle(overlay, (0, 0), (image.shape[1], 6 + step * len(lines)),
                  (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.45, image, 0.55, 0, image)
    for i, line in enumerate(lines):
        cv2.putText(image, line, (6, 15 + i * step), font, size,
                    (255, 255, 255), 1, cv2.LINE_AA)
    return image


def render(cv2, frame, detections, hud=(), scale=2, conf_floor=None):
    """The annotated picture: upscale first, then draw, so the text stays crisp.

    416x234 is what the model sees and it is too small to read a label on. The
    upscale is for the human only -- nothing measured comes off this image.
    """
    image = frame if scale == 1 else cv2.resize(
        frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    draw_boxes(cv2, image, detections, conf_floor=conf_floor)
    if hud:
        draw_hud(cv2, image, hud)
    return image


class Tally:
    """Running per-class counts, which is the summary worth printing at the end.

    Counts, not metrics: this tool has no ground truth. A class that never
    appears across a session that contained it is still a finding -- the same
    question evaluate.py's qualitative sweep asks, asked live.
    """

    def __init__(self):
        self.per_class = {name: 0 for name in CLASS_NAMES}
        self.frames = 0
        self.empty = 0
        self.boxes = 0

    def add(self, detections):
        self.frames += 1
        self.boxes += len(detections)
        if not detections:
            self.empty += 1
        for det in detections:
            name = name_of(det.cls)
            if name in self.per_class:
                self.per_class[name] += 1

    def summary(self):
        counts = "  ".join(f"{n}={self.per_class[n]}" for n in CLASS_NAMES)
        return (f"{self.frames} frames, {self.boxes} boxes, "
                f"{self.empty} with nothing found\n  {counts}")

    def missing(self):
        return [n for n in CLASS_NAMES if self.per_class[n] == 0]


class CameraSource:
    """The OAK-D preview stream, configured exactly as capture_cones.py was."""

    name = "camera"

    def __init__(self, fps, settle_s):
        self.device, self.size = oakd.open_camera(fps)
        self.queue = self.device.getOutputQueue("preview", maxSize=1, blocking=False)
        width, height = self.size
        print(f"OAK-D {self.device.getMxId()}  USB {self.device.getUsbSpeed().name}  "
              f"{width}x{height} @ {fps:g} fps")
        print(f"settling exposure and white balance for {settle_s:g}s "
              f"-- point the camera at the track")
        oakd.lock_camera(self.device.getInputQueue("control"), settle_s)

    def frames(self):
        while True:
            packet = self.queue.tryGet()
            if packet is None:
                time.sleep(0.002)
                continue
            yield packet.getCvFrame(), time.monotonic()

    def close(self):
        self.device.close()


class FrameDirSource:
    """Recorded jpgs, at a desk, through the real model.

    Not detectors.ReplayDetector: that replays the LABELS recorded beside a
    session, which answers a different question. Here the frames are the input
    and the model is genuinely running, which is the point of the tool.
    """

    name = "frames"

    def __init__(self, path, fps, loop=False):
        self.paths = sorted(p for pattern in ("*.jpg", "*.jpeg", "*.png")
                            for p in glob.glob(os.path.join(path, pattern)))
        if not self.paths:
            # A capture session keeps its frames in <session>/frames/ and a
            # Roboflow export keeps them in <split>/images/, so pointing at the
            # directory above either is the obvious mistake to make.
            for subdir in ("frames", "images"):
                nested = os.path.join(path, subdir)
                if os.path.isdir(nested):
                    raise SystemExit(f"error: no images directly in {path}\n"
                                     f"       Did you mean {nested}?")
            raise SystemExit(f"error: no .jpg/.jpeg/.png under {path}")
        self.interval = 1.0 / fps if fps > 0 else 0.0
        self.loop = loop
        print(f"{len(self.paths)} frames from {path}"
              + ("  (looping)" if loop else ""))

    def frames(self):
        cv2 = load_cv2()
        while True:
            for path in self.paths:
                image = cv2.imread(path)
                if image is None:
                    print(f"\nwarning: cannot read {path}, skipping")
                    continue
                yield image, time.monotonic()
                if self.interval:
                    time.sleep(self.interval)
            if not self.loop:
                return

    def close(self):
        pass


class Sinks:
    """Foxglove channels for the live server and the MCAP file.

    Degrades to unavailable rather than taking the run down -- same contract as
    lidar_view.py and depth_view.py. With --window there is somewhere else to
    look, so it is not fatal here either.
    """

    def __init__(self, frame_id="camera"):
        self.frame_id = frame_id
        self.available = False
        self.reason = None
        try:
            import foxglove
            try:
                from foxglove import messages as schemas
            except ImportError:  # SDKs before the messages/schemas rename
                from foxglove import schemas
            from foxglove.channels import CompressedImageChannel
        except ImportError as exc:
            self.reason = str(exc)
            return

        self._foxglove = foxglove
        self._s = schemas
        self.image_ch = CompressedImageChannel("/detections")
        self.status_ch = foxglove.Channel("/detect_status", schema=STATUS_SCHEMA)
        self.available = True

    def open_mcap(self, path):
        return self._foxglove.open_mcap(path)

    def start_server(self, host, port):
        return self._foxglove.start_server(name="cone-car detections", host=host,
                                           port=port)

    def log_image(self, jpeg, wall_s):
        self.image_ch.log(self._s.CompressedImage(
            timestamp=self._s.Timestamp.from_epoch_secs(wall_s),
            frame_id=self.frame_id, data=jpeg, format="jpeg",
        ), log_time=int(wall_s * 1e9))

    def log_status(self, status, wall_s):
        self.status_ch.log(status, log_time=int(wall_s * 1e9))


def hud_lines(tally, fps, inference_s, detections):
    counts = "  ".join(f"{n[:3]} {tally.per_class[n]}" for n in CLASS_NAMES)
    return [
        f"{fps:4.1f} fps   {inference_s * 1000:5.0f} ms   {len(detections)} boxes",
        counts,
    ]


def status_of(tally, fps, inference_s, detections):
    confidences = [d.confidence for d in detections]
    status = {
        "frames": tally.frames,
        "fps": round(fps, 2),
        "inference_ms": round(inference_s * 1000.0, 1),
        "boxes": len(detections),
        "lowest_conf": round(min(confidences), 3) if confidences else 0.0,
        "clipped": sum(1 for d in detections if d.clipped),
        "empty_frames": tally.empty,
    }
    status.update(tally.per_class)
    return status


def show_window(cv2, image, title="detect_view"):
    """True to keep going, False when the window asked to stop."""
    try:
        cv2.imshow(title, image)
    except cv2.error as exc:
        raise SystemExit(
            f"error: --window cannot open a display ({exc}).\n"
            "       Over ssh there is none -- drop --window and watch it in\n"
            "       Foxglove instead. opencv-python-headless also cannot do\n"
            "       this; the full opencv-python can.")
    return (cv2.waitKey(1) & 0xFF) not in (ord("q"), 27)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--weights", default="best.pt",
                        help="detector weights (default: best.pt beside this tool)")
    parser.add_argument("--detector", default="ultralytics",
                        choices=("ultralytics", "blob"),
                        help="where the boxes come from (default: ultralytics)")
    parser.add_argument("--frames", default=None,
                        help="a directory of recorded jpgs to run instead of the "
                             "camera; no hardware needed")
    parser.add_argument("--loop", action="store_true",
                        help="with --frames, start over at the end")
    parser.add_argument("--imgsz", type=int, default=detectors.DEFAULT_IMGSZ)
    parser.add_argument("--conf", type=float, default=detectors.DEFAULT_CONF,
                        help=f"confidence floor (default: {detectors.DEFAULT_CONF}); "
                             "boxes within 0.1 of it are drawn thin")
    parser.add_argument("--device", default=None,
                        help="torch device for ultralytics; default is CPU on the car")
    parser.add_argument("--camera-fps", type=float, default=10.0,
                        help="sensor rate, or the replay rate with --frames (default: 10)")
    parser.add_argument("--settle", type=float, default=2.0,
                        help="seconds of auto exposure/WB before locking (default: 2)")
    parser.add_argument("--scale", type=int, default=2,
                        help="upscale the 416x234 preview this much for viewing "
                             "(default: 2)")
    parser.add_argument("--quality", type=int, default=80,
                        help="JPEG quality for /detections (default: 80)")
    parser.add_argument("--host", default="0.0.0.0",
                        help="Foxglove bind address; the SDK default of 127.0.0.1 is "
                             "unreachable from the laptop (default: 0.0.0.0)")
    parser.add_argument("--ws-port", type=int, default=DEFAULT_PORT_WS,
                        help=f"Foxglove port; {DEFAULT_PORT_WS} leaves 8765/8766/8767 "
                             f"to the other tools (default: {DEFAULT_PORT_WS})")
    parser.add_argument("--window", action="store_true",
                        help="also open a cv2 window; needs a display, so not over ssh")
    parser.add_argument("--no-live", action="store_true",
                        help="no Foxglove server; use with --window, --save-dir or --mcap")
    parser.add_argument("--mcap", default=None,
                        help="record /detections and /detect_status for replay off-car")
    parser.add_argument("--save-dir", default=None,
                        help="write every annotated frame here as a jpg; the "
                             "capture_cones.py --preview habit -- run it over ssh, "
                             "scp the frames back and look at them")
    parser.add_argument("--save-every", type=int, default=1,
                        help="with --save-dir, keep every Nth frame (default: 1)")
    parser.add_argument("--duration", type=float, default=None,
                        help="stop after this many seconds")
    args = parser.parse_args(argv)

    if args.scale < 1:
        parser.error("--scale must be at least 1")
    if args.save_every < 1:
        parser.error("--save-every must be at least 1")
    if args.no_live and not (args.window or args.mcap or args.save_dir):
        parser.error("--no-live without --window, --save-dir or --mcap leaves "
                     "nothing to look at")
    return args


def main(argv=None):
    args = parse_args(argv)
    cv2 = load_cv2()

    detector = detectors.build(args.detector, weights=args.weights,
                               imgsz=args.imgsz, conf=args.conf,
                               device=args.device)
    names = check_weights_class_order(detector)
    print(f"detector  {detector.name}  {args.weights}  "
          f"imgsz {args.imgsz}  conf {args.conf}")
    if names:
        print(f"classes   {', '.join(names)}  (matches cone_classes.py)")

    sinks = Sinks()
    live = sinks.available and not args.no_live
    if not sinks.available and not (args.window or args.save_dir):
        raise SystemExit(
            f"error: foxglove-sdk is not available ({sinks.reason}), and neither\n"
            f"       --window nor --save-dir was given, so there is nowhere to draw.\n"
            f"       ~/env/bin/pip install foxglove-sdk   (needs Python 3.10+)\n"
            f"       Or --save-dir OUT, and scp the frames back.")

    tally = Tally()
    with ExitStack() as stack:
        if args.frames:
            source = FrameDirSource(args.frames, args.camera_fps, loop=args.loop)
        else:
            source = CameraSource(args.camera_fps, args.settle)
        stack.callback(source.close)

        if args.mcap and sinks.available:
            path = os.path.expanduser(args.mcap)
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            try:
                stack.enter_context(sinks.open_mcap(path))
            except FileExistsError:
                raise SystemExit(f"error: {path} already exists.\n"
                                 f"       Pick another --mcap path; recordings are "
                                 f"never overwritten.")
            print(f"recording {path}")
        elif args.mcap:
            # Not fatal -- --save-dir or --window may still be giving you
            # something to look at -- but silently dropping a recording someone
            # asked for is worse than the run being one flag short.
            print(f"warning: --mcap needs foxglove-sdk ({sinks.reason}); "
                  f"nothing will be recorded.")
        if live:
            server = sinks.start_server(args.host, args.ws_port)
            stack.callback(server.stop)
            print(f"\nFoxglove: ws://<car-ip>:{args.ws_port}  "
                  f"(desktop app, not the browser)")
            print("  Image panel -> /detections, Raw Messages -> /detect_status\n")
        if args.save_dir:
            os.makedirs(args.save_dir, exist_ok=True)
            print(f"writing annotated frames to {args.save_dir}")
        if args.window:
            print("window open -- q or Esc to quit\n")

        saved = 0
        started = time.monotonic()
        last_report = started
        recent = []
        try:
            for frame, captured_at in source.frames():
                detection_set = detector.detect(frame, captured_at)
                detections = detection_set.detections
                tally.add(detections)

                now = time.monotonic()
                # Rolling fps over the last couple of seconds. A cumulative
                # average hides the moment inference starts falling behind.
                recent.append(now)
                recent = [t for t in recent if now - t <= 2.0]
                fps = ((len(recent) - 1) / (recent[-1] - recent[0])
                       if len(recent) > 1 else 0.0)

                image = render(cv2, frame, detections,
                               hud=hud_lines(tally, fps,
                                             detection_set.inference_s, detections),
                               scale=args.scale, conf_floor=args.conf + 0.1)

                wall = time.time()
                if sinks.available and (live or args.mcap):
                    ok, buffer = cv2.imencode(
                        ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), args.quality])
                    if ok:
                        sinks.log_image(bytes(buffer), wall)
                    sinks.log_status(
                        status_of(tally, fps, detection_set.inference_s, detections),
                        wall)
                if args.save_dir and (tally.frames - 1) % args.save_every == 0:
                    cv2.imwrite(
                        os.path.join(args.save_dir, f"{tally.frames - 1:06d}.jpg"),
                        image, [int(cv2.IMWRITE_JPEG_QUALITY), args.quality])
                    saved += 1
                if args.window and not show_window(cv2, image):
                    break

                if now - last_report >= 1.0:
                    last_report = now
                    sys.stdout.write(
                        f"\r  {tally.frames} frames  {fps:4.1f} fps  "
                        f"{detection_set.inference_s * 1000:5.0f} ms  "
                        f"{len(detections)} boxes" + " " * 8)
                    sys.stdout.flush()
                if args.duration is not None and now - started >= args.duration:
                    break
        except KeyboardInterrupt:
            print("\ninterrupted")
        finally:
            if args.window:
                cv2.destroyAllWindows()

    print(f"\n{tally.summary()}")
    if args.save_dir and saved:
        print(f"  {saved} annotated frames -> {args.save_dir}")
    if tally.frames == 0:
        print("  No frames arrived at all.")
        return 1
    missing = tally.missing()
    if missing:
        # Same reading as evaluate.py's qualitative sweep: absence is a finding
        # only if the class was in front of the camera, and only you know that.
        print(f"  never detected: {', '.join(missing)} -- a finding if those "
              f"cones were in shot.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
