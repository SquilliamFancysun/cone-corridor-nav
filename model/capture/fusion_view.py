"""Live camera+lidar fusion and corridor extraction. Runs on the car.

    python fusion_view.py --weights ~/models/best.pt

Then connect Foxglove Studio to ws://<car-ip>:8767 -- the car's IP or its
.local name, never the ssh alias. Port 8767, so this can run alongside
lidar_view.py (8765) and depth_view.py (8766), though all three want hardware
that only one process can hold at a time.

Owns BOTH sensors. That is the point of the tool rather than an implementation
detail: model/capture/session.py records each sample's timestamp relative to
its own session's first sample, so two separately-recorded sessions cannot be
aligned to better than the one-second resolution of created_utc. Fusing a
camera stream against a lidar stream needs them on one clock, and one process
is how you get one clock.

What it draws:

    /scan          the raw revolution, as lidar_view.py draws it
    /cones         every cluster, coloured by its label, grey when UNLABELED
    /centerline    the chosen chain, with the branches it rejected dimmed
    /detections    the camera frame with its boxes
    /fusion_status counters -- this is the panel that says WHY it is going badly

Camera ownership is exclusive: DonkeyCar must be on myconfig_capture.py
(CAMERA_TYPE="MOCK"), and capture_cones.py / depth_view.py must not be running.
"""

import argparse
import json
import math
import os
import sys
import threading
import time

# On the car everything sits in one directory: deploy.sh drops cone_perception/
# and cone_nav/ beside this file, so the imports below just resolve. In a git
# checkout they live under ros2/src/, and --detector replay is meant to be run
# at a desk, so find them there too rather than making the tool car-only.
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
import oakd
from calibrate import load as load_calibration
from cone_nav.corridor import boundary_split
from cone_nav.corridor.centerline import CORRIDOR, centerline
from cone_perception import clustering, extrinsics, fusion
from cone_perception.cone_classes import (
    CLASS_BLUE,
    CLASS_MAGENTA,
    CLASS_ORANGE,
    CLASS_RED,
    CLASS_YELLOW,
    UNLABELED,
)
from ld06 import LD06Decoder, ScanAssembler, bin_scan
from lidar_view import BAUD, DEFAULT_PORT, open_serial

DEFAULT_PORT_WS = 8767
DEFAULT_BINS = 450

# Drawn colours match the cones, so a mislabelled cone is obvious at a glance
# rather than something you decode from a legend. UNLABELED is grey and
# deliberately dull: it is context, not a finding.
CONE_RGBA = {
    CLASS_BLUE: (0.15, 0.35, 0.95, 1.0),
    CLASS_YELLOW: (0.95, 0.85, 0.10, 1.0),
    CLASS_RED: (0.90, 0.15, 0.15, 1.0),
    CLASS_ORANGE: (0.95, 0.50, 0.10, 1.0),
    CLASS_MAGENTA: (0.85, 0.15, 0.75, 1.0),
    UNLABELED: (0.45, 0.45, 0.45, 0.6),
}

STATUS_SCHEMA = {
    "type": "object",
    "title": "FusionStatus",
    "properties": {
        "candidates": {"type": "integer", "description": "lidar clusters that could be cones"},
        "detections": {"type": "integer", "description": "boxes from the camera"},
        "matched": {"type": "integer"},
        "out_of_fov": {"type": "integer",
                       "description": "clusters the camera could not have seen; normally most of them"},
        "unmatched_in_fov": {"type": "integer",
                             "description": "clusters the camera should have seen and did not"},
        "unmatched_detections": {"type": "integer",
                                 "description": "boxes with no cluster; a cone beyond lidar range"},
        "detection_age_s": {"type": "number"},
        "stale": {"type": "boolean", "description": "detections too old to trust"},
        "inference_s": {"type": "number"},
        "scan_hz": {"type": "number"},
        "centerline_points": {"type": "integer"},
        "single_boundary_fallback": {"type": "boolean"},
        "corridor_half_width": {"type": "number"},
        "gates": {"type": "integer"},
        "points_per_cluster": {"type": "number",
                               "description": "observed; the number that says how far the lidar really sees"},
        "crc_drop_rate": {"type": "number"},
    },
}


class Sinks:
    """Foxglove channels feeding the live server and the MCAP file.

    Degrades to a no-op if foxglove-sdk is missing rather than taking the run
    down with it -- same contract as lidar_view.py and depth_view.py.
    """

    def __init__(self, frame_id="base_link"):
        self.frame_id = frame_id
        self.available = False
        self.reason = None
        try:
            import foxglove
            try:
                from foxglove import messages as schemas
            except ImportError:  # SDKs before the messages/schemas rename
                from foxglove import schemas
            from foxglove.channels import (FrameTransformChannel,
                                           LaserScanChannel, SceneUpdateChannel)
        except ImportError as exc:
            self.reason = str(exc)
            return

        self._foxglove = foxglove
        self._s = schemas
        self.scan_ch = LaserScanChannel("/scan")
        self.cones_ch = SceneUpdateChannel("/cones")
        self.line_ch = SceneUpdateChannel("/centerline")
        self.tf_ch = FrameTransformChannel("/tf")
        self.status_ch = foxglove.Channel("/fusion_status", schema=STATUS_SCHEMA)
        self.available = True

    def open_mcap(self, path):
        return self._foxglove.open_mcap(path)

    def start_server(self, host, port):
        return self._foxglove.start_server(name="cone-car fusion", host=host,
                                           port=port)

    def _stamp(self, wall_s):
        return self._s.Timestamp.from_epoch_secs(wall_s)

    def _identity_pose(self):
        return self._s.Pose(
            position=self._s.Vector3(x=0.0, y=0.0, z=0.0),
            orientation=self._s.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

    def log_transforms(self, wall_s):
        """base_link is the lidar, so the camera is the one that moves.

        See cone_perception/extrinsics.py: this is not a convention chosen here,
        it is the one every recorded session already used.
        """
        stamp = self._stamp(wall_s)
        identity = self._s.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.tf_ch.log(self._s.FrameTransform(
            timestamp=stamp, parent_frame_id="base_link", child_frame_id="lidar",
            translation=self._s.Vector3(x=0.0, y=0.0, z=0.0), rotation=identity))
        cam_x, cam_y, cam_z = extrinsics.CAMERA_IN_BASE
        half = math.radians(extrinsics.CAMERA_YAW_DEG) / 2.0
        self.tf_ch.log(self._s.FrameTransform(
            timestamp=stamp, parent_frame_id="base_link", child_frame_id="camera",
            translation=self._s.Vector3(x=cam_x, y=cam_y, z=cam_z),
            rotation=self._s.Quaternion(x=0.0, y=0.0, z=math.sin(half),
                                        w=math.cos(half))))

    def log_scan(self, scan, wall_s, bins, mirror, angle_offset):
        self.scan_ch.log(self._s.LaserScan(
            timestamp=self._stamp(wall_s),
            frame_id="lidar",
            start_angle=0.0,
            end_angle=2 * math.pi * (bins - 1) / bins,
            ranges=bin_scan(scan, bins=bins, mirror=mirror,
                            angle_offset_deg=angle_offset),
            intensities=[],
        ), log_time=int(wall_s * 1e9))

    def log_cones(self, cones, wall_s):
        s = self._s
        spheres = []
        for cone in cones:
            r, g, b, a = CONE_RGBA.get(cone.cone_class, CONE_RGBA[UNLABELED])
            # Labelled cones are drawn larger. At a glance the picture should
            # say how much of the field the camera actually explained.
            size = 0.16 if cone.labeled else 0.10
            spheres.append(s.SpherePrimitive(
                pose=s.Pose(
                    position=s.Vector3(x=cone.x, y=cone.y, z=0.0),
                    orientation=s.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
                size=s.Vector3(x=size, y=size, z=size),
                color=s.Color(r=r, g=g, b=b, a=a)))
        entity = s.SceneEntity(timestamp=self._stamp(wall_s),
                               frame_id=self.frame_id, id="cones",
                               lifetime=None, frame_locked=False,
                               spheres=spheres)
        self.cones_ch.log(s.SceneUpdate(deletions=[], entities=[entity]),
                          log_time=int(wall_s * 1e9))

    def log_centerline(self, result, wall_s):
        """The chosen chain bright, everything it rejected dim.

        Drawing the rejected branches is the whole reason a fork is debuggable:
        a chain that took the wrong branch looks identical to a correct one
        unless you can see what it was choosing between.
        """
        s = self._s
        lines = []
        if len(result.points) >= 2:
            lines.append(s.LinePrimitive(
                type=s.LinePrimitiveLineType.LineStrip,
                pose=self._identity_pose(), thickness=0.05,
                scale_invariant=False,
                points=[s.Point3(x=x, y=y, z=0.05) for x, y in result.points],
                color=s.Color(r=0.1, g=0.95, b=0.4, a=1.0),
                colors=[], indices=[]))

        chosen = {p for p in result.points}
        rejected = [m for m in result.midpoints
                    if m.kind == CORRIDOR and m.xy not in chosen]
        spheres = [s.SpherePrimitive(
            pose=s.Pose(position=s.Vector3(x=m.x, y=m.y, z=0.05),
                        orientation=s.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
            size=s.Vector3(x=0.07, y=0.07, z=0.07),
            color=s.Color(r=0.1, g=0.6, b=0.35, a=0.35)) for m in rejected]
        # Gate midpoints are landmarks, not steps on the line, so they get their
        # own marker rather than being threaded into the chain.
        spheres += [s.SpherePrimitive(
            pose=s.Pose(position=s.Vector3(x=m.x, y=m.y, z=0.05),
                        orientation=s.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
            size=s.Vector3(x=0.13, y=0.13, z=0.13),
            color=s.Color(r=1.0, g=0.3, b=0.3, a=0.9)) for m in result.gates]

        entity = s.SceneEntity(timestamp=self._stamp(wall_s),
                               frame_id=self.frame_id, id="centerline",
                               lifetime=None, frame_locked=False,
                               lines=lines, spheres=spheres)
        self.line_ch.log(s.SceneUpdate(deletions=[], entities=[entity]),
                         log_time=int(wall_s * 1e9))

    def log_status(self, status, wall_s):
        self.status_ch.log(status, log_time=int(wall_s * 1e9))


class LidarReader(threading.Thread):
    """Serial -> Scans, on its own thread so inference never stalls the port.

    A blocked read backs up in the kernel buffer and the scans that come out
    afterwards are old. The camera is the slow half here, and it must not be
    allowed to make the lidar the slow half too.
    """

    daemon = True

    def __init__(self, handle):
        super().__init__(name="ld06")
        self.handle = handle
        self.decoder = LD06Decoder()
        self.assembler = ScanAssembler()
        self.latest = None
        self.count = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            data = self.handle.read(4096)
            if not data:
                continue
            now = time.monotonic()
            for packet in self.decoder.feed(data):
                scan = self.assembler.add(packet, now)
                if scan is not None:
                    with self._lock:
                        self.latest = scan
                        self.count += 1

    def take(self):
        """The newest complete revolution, or None. Consumed once."""
        with self._lock:
            scan, self.latest = self.latest, None
            return scan

    def stop(self):
        self._stop.set()


def pipeline_once(scan, detection_set, calibration, intr, args, now):
    """One revolution -> labelled cones and a centerline. The whole algorithm.

    Deliberately the only place the layers are wired together, and deliberately
    free of I/O: this is what a ROS node will call, unchanged.
    """
    candidates = clustering.cone_candidates(
        scan, calibration, max_range_m=args.max_range)
    age = detection_set.age(now) if detection_set is not None else float("inf")
    detections = detection_set.detections if detection_set is not None else []
    result = fusion.associate(candidates, detections, intr, detection_age_s=age,
                              max_bearing_err_deg=args.bearing_gate,
                              max_detection_age_s=args.max_detection_age)
    bounds = boundary_split.split(result.cones, max_range_m=args.max_range)
    line = centerline(result.cones, car_xy=(0.0, 0.0))
    return result, bounds, line


def status_of(result, line, reader, detection_set, elapsed):
    points = [c.points for c in result.cones]
    return {
        "candidates": result.candidates,
        "detections": result.detections,
        "matched": result.matched,
        "out_of_fov": result.out_of_fov,
        "unmatched_in_fov": result.unmatched_in_fov,
        "unmatched_detections": result.unmatched_detections,
        "detection_age_s": round(result.detection_age_s, 3)
        if result.detection_age_s != float("inf") else -1.0,
        "stale": bool(result.stale),
        "inference_s": round(detection_set.inference_s, 3) if detection_set else 0.0,
        "scan_hz": round(reader.count / elapsed, 2) if elapsed else 0.0,
        "centerline_points": len(line.points),
        "single_boundary_fallback": bool(line.single_boundary_fallback),
        "corridor_half_width": round(line.corridor_half_width, 3),
        "gates": len(line.gates),
        "points_per_cluster": round(sum(points) / len(points), 2) if points else 0.0,
        "crc_drop_rate": round(reader.decoder.drop_rate, 5),
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Live camera+lidar fusion and corridor extraction.")
    parser.add_argument("--detector", default="ultralytics",
                        choices=("ultralytics", "blob", "replay"),
                        help="where cone boxes come from")
    parser.add_argument("--weights", default="best.pt",
                        help="detector weights, for --detector ultralytics")
    parser.add_argument("--replay-session", default=None,
                        help="a capture_cones.py session directory, for --detector replay")
    parser.add_argument("--replay-scans", default=None,
                        help="a lidar_view.py scans.jsonl to replay against")
    parser.add_argument("--imgsz", type=int, default=detectors.DEFAULT_IMGSZ)
    parser.add_argument("--conf", type=float, default=detectors.DEFAULT_CONF)
    parser.add_argument("--device", default=None,
                        help="torch device for ultralytics; default is CPU on the car")
    parser.add_argument("--port", default=DEFAULT_PORT, help="LD06 serial port")
    parser.add_argument("--camera-fps", type=float, default=15.0)
    parser.add_argument("--max-range", type=float,
                        default=clustering.MAX_CONE_RANGE_M,
                        help="metres; past ~3 m a cone is one lidar return or none")
    parser.add_argument("--bearing-gate", type=float,
                        default=fusion.MAX_BEARING_ERR_DEG,
                        help="degrees of bearing error allowed when matching")
    parser.add_argument("--max-detection-age", type=float,
                        default=fusion.MAX_DETECTION_AGE_S,
                        help="seconds; older boxes are ignored rather than misapplied")
    parser.add_argument("--bins", type=int, default=DEFAULT_BINS)
    parser.add_argument("--ws-port", type=int, default=DEFAULT_PORT_WS)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--no-live", action="store_true",
                        help="no Foxglove server; useful for a headless smoke test")
    parser.add_argument("--duration", type=float, default=None,
                        help="seconds to run, then exit")
    parser.add_argument("--calibration", default=None,
                        help="path to calibration.json; default is beside this tool")
    return parser.parse_args(argv)


def resolve_calibration(args):
    """The lidar's bearing convention, or a refusal to guess at it.

    Hard error rather than lidar_view.py's warning, and the difference is
    deliberate. lidar_view.py RECORDS: a session captured against an unverified
    sign stores the sensor's own bearings and is fixable at a desk. This tool
    INTERPRETS -- it decides which side of the car a cone is on -- and there is
    no fixing that afterwards.
    """
    path = args.calibration or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "calibration.json")
    record = load_calibration(path)
    if record is None:
        raise SystemExit(
            f"error: no calibration at {path}\n"
            "       Run `python lidar_view.py --calibrate` first. Without the\n"
            "       bearing sign every cone lands on the wrong side of the car,\n"
            "       which looks fine on a straight and fails at a junction.")
    return record


def announce(record, intr, detector, args):
    print(f"lidar     mirror={record['mirror']} "
          f"angle_offset={record['angle_offset_deg']} deg "
          f"({len(record.get('chassis_arcs_sensor') or [])} chassis arcs masked)")
    print(f"camera    fx={intr.fx:.1f} cx={intr.cx:.1f} "
          f"{intr.width}x{intr.height}, hfov {extrinsics.CAMERA_HFOV_DEG} deg")
    print(f"extrinsic camera at {extrinsics.CAMERA_IN_BASE} m in base_link "
          f"(base_link is the lidar)")
    print(f"detector  {detector.name}, gate {args.bearing_gate} deg, "
          f"stale past {args.max_detection_age} s")
    for warning in fusion.startup_warnings():
        print(f"warning:  {warning}")


def main(argv=None):
    args = parse_args(argv)
    record = resolve_calibration(args)

    if args.detector == "replay":
        return run_replay(args, record)

    detector = detectors.build(args.detector, weights=args.weights,
                               imgsz=args.imgsz, conf=args.conf,
                               device=args.device)
    device, (width, height) = oakd.open_camera(args.camera_fps)
    intr = oakd.camera_intrinsics(device, width, height)
    announce(record, intr, detector, args)

    handle = open_serial(args.port, BAUD)
    reader = LidarReader(handle)
    reader.start()

    sinks = Sinks()
    if not sinks.available:
        print(f"warning:  no Foxglove sink ({sinks.reason}); running headless")

    q_preview = device.getOutputQueue("preview", maxSize=1, blocking=False)
    q_ctrl = device.getInputQueue("control")
    oakd.lock_camera(q_ctrl)

    server = None
    if sinks.available and not args.no_live:
        server = sinks.start_server(args.host, args.ws_port)
        print(f"\nFoxglove: ws://<car-ip>:{args.ws_port}  (desktop app, not the browser)\n")

    started = time.monotonic()
    detection_set = None
    last_report = started
    try:
        while True:
            if args.duration and time.monotonic() - started >= args.duration:
                break

            frame = q_preview.tryGet()
            if frame is not None:
                captured = time.monotonic()
                detection_set = detector.detect(frame.getCvFrame(), captured)

            scan = reader.take()
            if scan is None:
                time.sleep(0.005)
                continue

            now = time.monotonic()
            result, bounds, line = pipeline_once(scan, detection_set, record,
                                                 intr, args, now)
            wall = time.time()
            if sinks.available:
                sinks.log_transforms(wall)
                sinks.log_scan(scan, wall, args.bins, record["mirror"],
                               record["angle_offset_deg"])
                sinks.log_cones(result.cones, wall)
                sinks.log_centerline(line, wall)
                sinks.log_status(
                    status_of(result, line, reader, detection_set,
                              now - started), wall)

            if now - last_report >= 2.0:
                last_report = now
                print(f"  {result.matched}/{result.candidates} cones labelled, "
                      f"{len(line.points)} centerline points, "
                      f"{bounds.counts()}")
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        reader.stop()
        handle.close()
        device.close()
        if server is not None:
            server.stop()
    return 0


def run_replay(args, record):
    """Re-run a recording through the identical pipeline, with no hardware.

    The scans come from a lidar_view.py session's scans.jsonl and the boxes from
    a capture_cones.py session's _prelabel labels. Those two were recorded by
    separate processes and so cannot be aligned in time -- which is exactly the
    limitation this tool exists to remove -- so replay pairs them by INDEX and
    reports no meaningful detection age. It is for exercising the algorithm, not
    for measuring fusion quality.
    """
    if not args.replay_scans:
        raise SystemExit("error: --detector replay needs --replay-scans PATH")
    detector = detectors.build("replay", session=args.replay_session)
    intr = detectors.replay_intrinsics(416, 234)
    announce(record, intr, detector, args)

    processed = 0
    with open(args.replay_scans, encoding="utf-8") as fh:
        for line_no, raw in enumerate(fh):
            raw = raw.strip()
            if not raw:
                continue
            doc = json.loads(raw)
            scan = _scan_from_json(doc)
            detection_set = detector.detect(None, time.monotonic())
            result, bounds, line = pipeline_once(
                scan, detection_set, record, intr, args, time.monotonic())
            processed += 1
            if processed % 20 == 0:
                print(f"  scan {line_no}: {result.matched}/{result.candidates} "
                      f"labelled, {len(line.points)} centerline points")
    print(f"replayed {processed} revolutions")
    return 0


def _scan_from_json(doc):
    from ld06 import Scan
    return Scan(t=doc.get("t", 0.0), angles_deg=doc["angles_deg"],
                ranges_mm=doc["ranges_mm"],
                intensities=doc.get("intensities") or [0] * len(doc["angles_deg"]),
                speed_hz=doc.get("speed_hz", 10.0))


if __name__ == "__main__":
    sys.exit(main())
