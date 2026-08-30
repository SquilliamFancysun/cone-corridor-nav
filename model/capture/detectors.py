"""Where cone boxes come from. One interface, three sources.

Everything downstream consumes a `DetectionSet` -- a list of
cone_perception.geometry.Detection plus the time it was captured -- and knows
nothing about how it was produced. That is what lets the blob land later as a
swap rather than a rewrite, and what lets the whole fusion stack run off a
recording with no camera attached.

The backends live here, in model/capture/, rather than in cone_perception,
because they import depthai and ultralytics. cone_perception stays pure so it
can be unit-tested on a laptop and reused verbatim by a ROS node.

    ultralytics   best.pt on the Pi's CPU. ~8.6 fps at 416, measured on the
                  Pi 5 with v3 weights (111 ms/frame). Too slow to drive on;
                  fast enough to validate fusion by hand-pushing.
    blob          NOT IMPLEMENTED. The OAK-D runs the model on its own Myriad X
                  at camera rate. Needs model/export/ populated first.
    replay        Recorded frames plus their labels. No hardware at all.
"""

import glob
import os
import time

from cone_perception.geometry import Detection, intrinsics_from_hfov

# The detector's input resolution. 416 rather than 640 because the OAK-D here
# negotiates USB 2.0 and the Pi 5 runs the model on its CPU; both make the
# larger input a bad trade. model/dataset/LABELING.md asks for both to be
# benchmarked on-car -- that measurement belongs in the D3 characterization.
DEFAULT_IMGSZ = 416

# Deliberately low, matching roboflow_prelabel.py's default and for the same
# reason: a spurious box costs one mislabelled cluster, which the bearing gate
# usually rejects anyway, while a missing box costs a cone.
DEFAULT_CONF = 0.25


class DetectionSet(object):
    """Detections and when they were captured, on time.monotonic()'s clock.

    The timestamp is not decoration. Ultralytics on the Pi runs several times
    slower than the lidar, so by the time a scan is fused the boxes are already
    old, and fusion.associate refuses to use ones that are too old. Recording
    the capture time -- not the time fusion happened to run -- is what makes
    that check mean anything.
    """

    __slots__ = ("detections", "captured_at", "frame", "inference_s")

    def __init__(self, detections, captured_at, frame=None, inference_s=0.0):
        self.detections = detections
        self.captured_at = captured_at
        self.frame = frame
        self.inference_s = inference_s

    def age(self, now=None):
        return (time.monotonic() if now is None else now) - self.captured_at

    def __len__(self):
        return len(self.detections)


def _to_detection(cls, confidence, x1, y1, x2, y2, width, height):
    """Pixel corners -> a normalised Detection, with the clip flag set.

    `clipped` matters twice over: a box cut off by the frame edge has a centre
    that has moved inward, so its bearing points somewhere the cone is not, and
    a height that is cut short, so range_from_bbox would under-report.
    """
    u = ((x1 + x2) / 2.0) / width
    v = ((y1 + y2) / 2.0) / height
    w = abs(x2 - x1) / width
    h = abs(y2 - y1) / height
    clipped = x1 <= 1.0 or y1 <= 1.0 or x2 >= width - 1.0 or y2 >= height - 1.0
    return Detection(cls=int(cls), confidence=float(confidence),
                     u=u, v=v, w=w, h=h, clipped=clipped)


class UltralyticsDetector(object):
    """v1 best.pt, run on the host CPU against the OAK-D's preview stream.

    The camera configuration mirrors capture_cones.py deliberately: exposure,
    white balance and focus are settled then LOCKED, so the detector sees at
    inference the same colours it trained on. A camera left on auto drifts
    between frames, and blue/yellow is exactly the discrimination that drift
    costs.
    """

    name = "ultralytics"

    def __init__(self, weights, imgsz=DEFAULT_IMGSZ, conf=DEFAULT_CONF,
                 device=None):
        try:
            from ultralytics import YOLO
        except ImportError:
            raise SystemExit(
                "error: --detector ultralytics needs the ultralytics package.\n"
                "       ~/env/bin/pip install ultralytics\n"
                "       Or use --detector replay, which needs nothing.")
        if not os.path.exists(weights):
            raise SystemExit(
                f"error: no weights at {weights}\n"
                "       Weights are gitignored; fetch best.pt from the release\n"
                "       or point --weights at model/training/v1/weights/best.pt.")
        self.model = YOLO(weights)
        self.imgsz = imgsz
        self.conf = conf
        self.device = device
        self._warned = False
        self._frames = 0

    def detect(self, frame, captured_at):
        """BGR ndarray -> DetectionSet."""
        started = time.monotonic()
        results = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf,
                                     device=self.device, verbose=False)
        height, width = frame.shape[:2]
        out = []
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
                out.append(_to_detection(int(box.cls[0]), float(box.conf[0]),
                                         x1, y1, x2, y2, width, height))
        elapsed = time.monotonic() - started
        # The first predict() is ultralytics warming up -- it allocates and runs
        # the graph once, and on the Pi that frame alone costs ~1.5 s against a
        # ~0.11 s steady state. Judging the run by it fired this warning on every
        # single run and told everyone to try --imgsz 320 for nothing, so the
        # warmup frame is measured but never used as the verdict.
        self._frames += 1
        if self._frames > 1 and elapsed > 0.5 and not self._warned:
            self._warned = True
            print(f"warning: inference is taking {elapsed:.2f} s per frame. "
                  "Every label will be stale; try --imgsz 320.")
        return DetectionSet(out, captured_at, frame=frame, inference_s=elapsed)


class ReplayDetector(object):
    """Recorded frames and their YOLO txt labels. No hardware.

    Reads a capture session laid out the way prepare_dataset.py leaves one:
    frames/NNNNNN.jpg beside labels in _prelabel/NNNNNN.txt. That means a
    session already run through roboflow_prelabel.py can be replayed through the
    full fusion stack at a desk.
    """

    name = "replay"

    def __init__(self, session_dir, labels_subdir="_prelabel"):
        self.frames = sorted(glob.glob(os.path.join(session_dir, "frames", "*.jpg")))
        if not self.frames:
            raise SystemExit(
                f"error: no frames under {session_dir}/frames/\n"
                "       --detector replay wants a capture_cones.py session.")
        self.labels_dir = os.path.join(session_dir, labels_subdir)
        self.index = 0

    def detect(self, frame, captured_at):
        """`frame` is ignored -- replay supplies its own."""
        if self.index >= len(self.frames):
            return DetectionSet([], captured_at)
        path = self.frames[self.index]
        self.index += 1
        label = os.path.join(
            self.labels_dir,
            os.path.splitext(os.path.basename(path))[0] + ".txt")
        return DetectionSet(self._read(label), captured_at)

    def _read(self, path):
        """YOLO txt: `cls xc yc w h`, all normalised. Missing file means no cones."""
        out = []
        try:
            with open(path, encoding="utf-8") as fh:
                for line in fh:
                    parts = line.split()
                    if len(parts) < 5:
                        continue
                    cls, u, v, w, h = (float(p) for p in parts[:5])
                    clipped = (u - w / 2 <= 0.002 or u + w / 2 >= 0.998
                               or v - h / 2 <= 0.002 or v + h / 2 >= 0.998)
                    out.append(Detection(cls=int(cls), confidence=1.0, u=u, v=v,
                                         w=w, h=h, clipped=clipped))
        except OSError:
            return []
        return out


class BlobDetector(object):
    """The OAK-D running the model itself. Not built yet.

    Kept as a named backend rather than left out, so the seam it plugs into is
    visible and the error says what is actually missing. See
    model/dataset/LABELING.md for the tools.luxonis.com round trip.
    """

    name = "blob"

    def __init__(self, *args, **kwargs):
        raise SystemExit(
            "error: --detector blob is not implemented.\n"
            "       model/export/ is empty: there is no .blob and no anchors\n"
            "       JSON, so the OAK-D cannot run the detector on-device.\n"
            "       See model/dataset/LABELING.md. Use --detector ultralytics.")


def build(name, weights=None, session=None, imgsz=DEFAULT_IMGSZ,
          conf=DEFAULT_CONF, device=None):
    if name == "ultralytics":
        return UltralyticsDetector(weights, imgsz=imgsz, conf=conf, device=device)
    if name == "replay":
        return ReplayDetector(session)
    if name == "blob":
        return BlobDetector()
    raise SystemExit(f"error: unknown detector {name!r}")


def replay_intrinsics(width, height):
    """Intrinsics for a recording, where there is no device to ask."""
    return intrinsics_from_hfov(width, height)
