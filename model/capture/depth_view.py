"""Live Foxglove view of the OAK-D Lite's stereo depth. Runs on the car.

    python depth_view.py

Then connect Foxglove Studio to ws://<car-ip>:8766 — the car's IP or its
.local name, never the ssh alias, which only ssh can resolve. Port 8766, not
8765, so this and lidar_view.py can stream at the same time.

Owns the camera, exactly as capture_cones.py does: only one process can hold
the OAK-D, so DonkeyCar must be running with CAMERA_TYPE="MOCK" (see
myconfig_capture.py) and capture_cones.py must not be running. There is no
sharing the device; the second opener gets an X_LINK_DEVICE_ALREADY_IN_USE.

Points are emitted in the car frame (x forward, y left, z up), not the camera's
optical frame, so /depth/points and lidar_view.py's /scan land in the same 3D
panel with the same convention and can be compared directly.
"""

import argparse
import os
import sys
import time
from contextlib import ExitStack

import depthai as dai
import numpy as np

# The OAK-D Lite's mono sensors are OV7251: 480p and 400p only, no 720p. 400p
# is the smaller of the two and the depth map is bandwidth, not detail — see
# --usb-speed in the banner if frames stall.
MONO_RES = {
    "400p": (dai.MonoCameraProperties.SensorResolution.THE_400_P, 640, 400),
    "480p": (dai.MonoCameraProperties.SensorResolution.THE_480_P, 640, 480),
}

STATUS_SCHEMA = {
    "type": "object",
    "title": "DepthLinkHealth",
    "properties": {
        "usb_speed": {"type": "string"},
        "frames": {"type": "integer"},
        "fps": {"type": "number"},
        "valid_fraction": {"type": "number",
                           "description": "pixels with a depth return, 0 means no stereo match"},
        "min_m": {"type": "number"},
        "median_m": {"type": "number"},
        "max_m": {"type": "number"},
        "points": {"type": "integer"},
    },
}


def build_pipeline(args):
    """LEFT + RIGHT mono -> StereoDepth -> host, as uint16 millimetres.

    Depth comes out registered to the *right* rectified mono frame by default,
    which is why the intrinsics read back below are RIGHT's. Aligning to RGB
    instead is one call (--align-rgb) but reprojects into a different, narrower
    FOV, so the default keeps the raw stereo geometry.
    """
    sensor_res, width, height = MONO_RES[args.mono_resolution]
    pipeline = dai.Pipeline()

    left = pipeline.create(dai.node.MonoCamera)
    right = pipeline.create(dai.node.MonoCamera)
    for cam, socket in ((left, dai.CameraBoardSocket.LEFT),
                        (right, dai.CameraBoardSocket.RIGHT)):
        cam.setBoardSocket(socket)
        cam.setResolution(sensor_res)
        cam.setFps(args.fps)

    stereo = pipeline.create(dai.node.StereoDepth)
    # HIGH_DENSITY fills more of the frame than HIGH_ACCURACY, which is what
    # you want for a demo and for cone-sized blobs; accuracy mode leaves holes.
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    # Left-right check throws away matches the two views disagree on. It is the
    # difference between a clean cone and a smear of speckle at its edges.
    stereo.setLeftRightCheck(True)
    # The Lite's baseline is 7.5 cm, so the near limit is ~35 cm at 400p.
    # Extended disparity halves that at the cost of range; subpixel is the
    # opposite trade. Neither is on by default.
    stereo.setExtendedDisparity(args.extended)
    stereo.setSubpixel(args.subpixel)
    if args.align_rgb:
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
    left.out.link(stereo.left)
    right.out.link(stereo.right)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("depth")
    # Latest frame wins. A depth map queued behind three stale ones is a live
    # view that lies about where the car is.
    xout.input.setBlocking(False)
    xout.input.setQueueSize(1)
    stereo.depth.link(xout.input)

    return pipeline, width, height


def open_device(pipeline):
    """Start the device, or exit with the diagnosis that usually applies."""
    try:
        return dai.Device(pipeline)
    except RuntimeError as exc:
        message = str(exc)
        hint = ("       ls /dev/serial/by-id and check the USB-C cable — a USB 2.0\n"
                "       cable links the OAK-D at 480 M and nothing says why.\n"
                "       See docs/hardware-baseline.md.")
        if "ALREADY_IN_USE" in message or "already" in message.lower():
            hint = ("       Something else already holds the camera. Only one process\n"
                    "       can: stop capture_cones.py, and make sure DonkeyCar is on\n"
                    "       myconfig_capture.py (CAMERA_TYPE=\"MOCK\").")
        raise SystemExit(f"error: cannot open the OAK-D\n       {message}\n{hint}")


def intrinsics(device, socket, width, height):
    """fx, fy, cx, cy for the frame depth is registered to."""
    matrix = device.readCalibration().getCameraIntrinsics(socket, width, height)
    return matrix[0][0], matrix[1][1], matrix[0][2], matrix[1][2]


class Projector:
    """Depth image -> car-frame points, with the per-pixel rays precomputed.

    The rays never change, only the depths, so the trig happens once at startup
    instead of thirty times a second on a Pi.
    """

    def __init__(self, width, height, fx, fy, cx, cy, step):
        us = np.arange(0, width, step, dtype=np.float32)
        vs = np.arange(0, height, step, dtype=np.float32)
        grid_u, grid_v = np.meshgrid(us, vs)
        # Optical frame is x-right, y-down, z-forward; the car frame is
        # x-forward, y-left, z-up (REP-103, same as /scan). Hence the swap and
        # the two negations, done here so nothing downstream has to know.
        self.left = -(grid_u - cx) / fx
        self.up = -(grid_v - cy) / fy
        self.step = step
        self.shape = grid_u.shape

    def points(self, depth_mm, max_m):
        """Nx4 float32 of (x, y, z, range) in metres, invalid pixels dropped."""
        forward = depth_mm[::self.step, ::self.step].astype(np.float32) * 0.001
        # 0 is "no stereo match", not "a surface at the lens". Points at the
        # origin would draw a solid blob at the car in every frame.
        keep = (forward > 0.0) & (forward < max_m)
        x = forward[keep]
        out = np.empty((x.size, 4), dtype=np.float32)
        out[:, 0] = x
        out[:, 1] = x * self.left[keep]
        out[:, 2] = x * self.up[keep]
        out[:, 3] = x
        return out


class Sinks:
    """Foxglove channels feeding the live server and the MCAP file.

    Degrades to a no-op if foxglove-sdk is missing rather than taking the run
    down with it — same contract as lidar_view.py.
    """

    def __init__(self, frame_id="camera", parent_frame="base_link", mount=(0.0, 0.0, 0.0)):
        self.frame_id = frame_id
        self.parent_frame = parent_frame
        self.mount = mount
        self.available = False
        self.reason = None
        try:
            import foxglove
            try:
                from foxglove import messages as schemas
            except ImportError:  # SDKs before the messages/schemas rename
                from foxglove import schemas
            from foxglove.channels import (CompressedImageChannel, FrameTransformChannel,
                                           PointCloudChannel, RawImageChannel)
        except ImportError as exc:
            self.reason = str(exc)
            return

        self._foxglove = foxglove
        self._s = schemas
        self.depth_ch = RawImageChannel("/depth/image")
        self.color_ch = CompressedImageChannel("/depth/colorized")
        self.cloud_ch = PointCloudChannel("/depth/points")
        self.tf_ch = FrameTransformChannel("/tf")
        self.status_ch = foxglove.Channel("/depth_status", schema=STATUS_SCHEMA)
        self._fields = self._point_fields()
        self.available = True

    def _point_fields(self):
        """x, y, z and a copy of range, so the 3D panel can color by distance."""
        s = self._s
        numeric = s.PackedElementFieldNumericType
        # The SDK spells this member Float32; older builds shouted it. Getting
        # it wrong is a TypeError at construction, not a bad point cloud.
        f32 = getattr(numeric, "Float32", None) or numeric.FLOAT32
        return [s.PackedElementField(name=name, offset=offset, type=f32)
                for name, offset in (("x", 0), ("y", 4), ("z", 8), ("range", 12))]

    def open_mcap(self, path):
        return self._foxglove.open_mcap(path)

    def start_server(self, host, port):
        return self._foxglove.start_server(name="cone-car depth", host=host, port=port)

    def _stamp(self, wall_s):
        return self._s.Timestamp.from_epoch_secs(wall_s)

    def log_transform(self, wall_s):
        """base_link -> camera, so depth sits where the lens actually is."""
        x, y, z = self.mount
        self.tf_ch.log(self._s.FrameTransform(
            timestamp=self._stamp(wall_s),
            parent_frame_id=self.parent_frame,
            child_frame_id=self.frame_id,
            translation=self._s.Vector3(x=x, y=y, z=z),
            rotation=self._s.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ))

    def log_depth(self, depth_mm, wall_s):
        """The measurement itself: uint16 millimetres, nothing rescaled."""
        height, width = depth_mm.shape
        self.depth_ch.log(self._s.RawImage(
            timestamp=self._stamp(wall_s),
            frame_id=self.frame_id,
            width=width,
            height=height,
            encoding="16UC1",
            step=width * 2,
            data=depth_mm.tobytes(),
        ), log_time=int(wall_s * 1e9))

    def log_colorized(self, jpeg, wall_s):
        self.color_ch.log(self._s.CompressedImage(
            timestamp=self._stamp(wall_s),
            frame_id=self.frame_id,
            data=jpeg,
            format="jpeg",
        ), log_time=int(wall_s * 1e9))

    def log_points(self, points, wall_s):
        s = self._s
        self.cloud_ch.log(s.PointCloud(
            timestamp=self._stamp(wall_s),
            frame_id=self.frame_id,
            pose=s.Pose(position=s.Vector3(x=0.0, y=0.0, z=0.0),
                        orientation=s.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
            point_stride=16,
            fields=self._fields,
            data=points.tobytes(),
        ), log_time=int(wall_s * 1e9))
        self.log_transform(wall_s)

    def log_status(self, status, wall_s):
        self.status_ch.log(status, log_time=int(wall_s * 1e9))


def colorize(depth_mm, max_m):
    """Depth as a JPEG heat map, or None if cv2 is missing.

    Foxglove's Image panel renders 16UC1 as near-black without a hand-set value
    range, which is a poor first impression of a working sensor. This channel is
    the one you point at during a demo; /depth/image is the one you measure from.
    """
    try:
        import cv2
    except ImportError:
        return None
    clipped = np.clip(depth_mm, 0, int(max_m * 1000)).astype(np.float32)
    # Invert so near is hot: the default ramp puts the cone in front of the car
    # at the cold end, which reads backwards.
    scaled = (255.0 - clipped * (255.0 / (max_m * 1000.0))).astype(np.uint8)
    scaled[depth_mm == 0] = 0  # no return stays black, rather than "very close"
    colored = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
    ok, buffer = cv2.imencode(".jpg", colored, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    return bytes(buffer) if ok else None


def frame_stats(depth_mm):
    """Valid fraction and the range spread, in metres."""
    valid = depth_mm[depth_mm > 0]
    if valid.size == 0:
        return {"valid_fraction": 0.0, "min_m": 0.0, "median_m": 0.0, "max_m": 0.0}
    return {
        "valid_fraction": round(float(valid.size) / depth_mm.size, 4),
        "min_m": round(float(valid.min()) * 0.001, 3),
        "median_m": round(float(np.median(valid)) * 0.001, 3),
        "max_m": round(float(valid.max()) * 0.001, 3),
    }


def run_selftest(args):
    """A few frames of depth checked against the hardware baseline, then exit."""
    pipeline, width, height = build_pipeline(args)
    with open_device(pipeline) as device:
        speed = device.getUsbSpeed().name
        mxid = device.getMxId()
        queue = device.getOutputQueue("depth", maxSize=1, blocking=False)
        frames, stats = 0, {}
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            packet = queue.tryGet()
            if packet is None:
                time.sleep(0.005)
                continue
            frames += 1
            stats = frame_stats(packet.getFrame())
        elapsed = time.monotonic() - t0

    fps = frames / elapsed if elapsed else 0.0
    print(f"OAK-D {mxid}  USB {speed}  {width}x{height}  {frames} frames  {fps:.1f} fps")
    if not stats:
        print("  The device opened but produced no depth frames.")
        print("  Both mono cameras must enumerate; check the ribbon seating.")
        return 1
    print(f"valid {stats['valid_fraction'] * 100:.1f}% of pixels  "
          f"min {stats['min_m']:.2f} m  median {stats['median_m']:.2f} m  "
          f"max {stats['max_m']:.2f} m")

    ok = True
    if speed != "SUPER":
        # Not fatal for 400p depth, but it is the documented baseline and the
        # cable is the usual cause.
        print(f"  warning: link is {speed}, not SUPER. docs/hardware-baseline.md "
              f"expects 5 Gbps.\n"
              f"  A USB 2.0 cable links at 480 M and nothing in lsusb says why.")
        ok = False
    if stats["valid_fraction"] < 0.2:
        print(f"  warning: only {stats['valid_fraction'] * 100:.1f}% of pixels matched. "
              f"Point the car at\n  something textured a metre or two away — a blank "
              f"wall or open sky gives stereo\n  nothing to match on.")
        ok = False
    if fps < args.fps * 0.5:
        print(f"  warning: {fps:.1f} fps against a requested {args.fps}.")
        ok = False
    if ok:
        print("  matches docs/hardware-baseline.md.")
    return 0 if ok else 1


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
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="0.0.0.0",
                        help="Foxglove server bind address; the SDK default of "
                             "127.0.0.1 is unreachable from the laptop (default: 0.0.0.0)")
    parser.add_argument("--port-ws", type=int, default=8766,
                        help="Foxglove server port; 8766 so lidar_view.py keeps "
                             "8765 and both can stream at once (default: 8766)")
    parser.add_argument("--fps", type=float, default=10.0,
                        help="mono camera rate; 10 matches the lidar's revolution "
                             "rate and keeps the live stream near 2 MB/s, which is "
                             "what campus WiFi carries without stuttering")
    parser.add_argument("--mono-resolution", default="400p", choices=sorted(MONO_RES),
                        help="OAK-D Lite mono sensors do 400p and 480p only")
    parser.add_argument("--max-range", type=float, default=10.0,
                        help="metres; caps the color ramp and drops farther points")
    parser.add_argument("--cloud-step", type=int, default=4,
                        help="take every Nth pixel for the point cloud; 1 is every "
                             "pixel and roughly 16x the data (default: 4)")
    parser.add_argument("--no-cloud", action="store_true", help="skip /depth/points")
    parser.add_argument("--no-colorized", action="store_true",
                        help="skip the JPEG heat map, leaving only 16-bit /depth/image")
    parser.add_argument("--extended", action="store_true",
                        help="extended disparity: halves the ~35 cm near limit, "
                             "costs far range")
    parser.add_argument("--subpixel", action="store_true",
                        help="subpixel disparity: finer far-range steps, costs near range")
    parser.add_argument("--align-rgb", action="store_true",
                        help="register depth to the color camera instead of the right "
                             "mono; narrower FOV, and the color camera's 4:3 aspect "
                             "makes the reprojected point cloud approximate")
    parser.add_argument("--mount-x", type=float, default=0.0,
                        help="camera position in base_link, metres")
    parser.add_argument("--mount-y", type=float, default=0.0)
    parser.add_argument("--mount-z", type=float, default=0.0)
    parser.add_argument("--mcap", default=None,
                        help="also record every topic to this MCAP file, for replay")
    parser.add_argument("--duration", type=float, default=None,
                        help="stop after this many seconds")
    parser.add_argument("--no-live", action="store_true",
                        help="skip the Foxglove server; useful with --mcap")
    parser.add_argument("--selftest", action="store_true",
                        help="3s of depth checked against the hardware baseline, then exit")
    args = parser.parse_args(argv)

    if args.cloud_step < 1:
        parser.error("--cloud-step must be at least 1")
    if args.max_range <= 0:
        parser.error("--max-range must be positive")
    if args.no_live and not args.mcap:
        parser.error("--no-live without --mcap would produce nothing")
    if args.extended and args.subpixel:
        parser.error("--extended and --subpixel are mutually exclusive on this device")
    return args


def main(argv=None):
    args = parse_args(argv)

    if args.selftest:
        return run_selftest(args)

    sinks = Sinks(mount=(args.mount_x, args.mount_y, args.mount_z))
    if not sinks.available:
        raise SystemExit(
            f"error: foxglove-sdk is not available ({sinks.reason}).\n"
            f"       ~/env/bin/pip install foxglove-sdk   (needs Python 3.10+)")

    pipeline, width, height = build_pipeline(args)
    socket = dai.CameraBoardSocket.RGB if args.align_rgb else dai.CameraBoardSocket.RIGHT

    with ExitStack() as stack:
        device = stack.enter_context(open_device(pipeline))
        queue = device.getOutputQueue("depth", maxSize=1, blocking=False)
        # Intrinsics wait for the first frame: --align-rgb reprojects into a
        # different geometry, and the frame itself is the only honest source of
        # what size the depth map actually came out.
        projector = None

        print(f"OAK-D {device.getMxId()}  USB {device.getUsbSpeed().name}  "
              f"{width}x{height} @ {args.fps:g} fps  "
              f"(commit {tool_commit() or 'unknown'})")

        if args.mcap:
            path = os.path.expanduser(args.mcap)
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            try:
                stack.enter_context(sinks.open_mcap(path))
            except FileExistsError:
                # The SDK refuses to overwrite, which is the right default for a
                # recording — but the bare OSError names neither the file nor why.
                raise SystemExit(f"error: {path} already exists.\n"
                                 f"       Pick another --mcap path; recordings are "
                                 f"never overwritten.")
            print(f"Recording: {path}")
        if not args.no_live:
            server = sinks.start_server(args.host, args.port_ws)
            stack.callback(server.stop)
            print(f"Foxglove: ws://{args.host}:{args.port_ws}  "
                  f"(connect Studio to ws://<car>:{args.port_ws})")
            print("  3D panel -> /depth/points, Image panel -> /depth/colorized")

        frames = 0
        stats = {}
        points_out = 0
        t_start = time.monotonic()
        last_status = 0.0
        try:
            while True:
                packet = queue.tryGet()
                if packet is None:
                    time.sleep(0.002)
                else:
                    depth_mm = packet.getFrame()
                    wall = time.time()
                    frames += 1
                    if projector is None:
                        rows, cols = depth_mm.shape
                        fx, fy, cx, cy = intrinsics(device, socket, cols, rows)
                        projector = Projector(cols, rows, fx, fy, cx, cy, args.cloud_step)
                        print(f"depth {cols}x{rows}  intrinsics fx={fx:.1f} fy={fy:.1f} "
                              f"cx={cx:.1f} cy={cy:.1f}")
                    sinks.log_depth(depth_mm, wall)
                    if not args.no_colorized:
                        jpeg = colorize(depth_mm, args.max_range)
                        if jpeg is not None:
                            sinks.log_colorized(jpeg, wall)
                        else:
                            # cv2 is in ~/env, so this only fires off-car.
                            args.no_colorized = True
                            print("\nnote: cv2 not available; /depth/colorized disabled.")
                    if not args.no_cloud:
                        points = projector.points(depth_mm, args.max_range)
                        points_out = len(points)
                        sinks.log_points(points, wall)
                    else:
                        sinks.log_transform(wall)
                    stats = frame_stats(depth_mm)

                now = time.monotonic()
                if now - last_status >= 1.0 and stats:
                    elapsed = now - t_start
                    status = dict(stats)
                    status.update({
                        "usb_speed": device.getUsbSpeed().name,
                        "frames": frames,
                        "fps": round(frames / elapsed, 1) if elapsed else 0.0,
                        "points": points_out,
                    })
                    sinks.log_status(status, time.time())
                    last_status = now
                    sys.stdout.write(
                        f"\r  live  {frames} frames  {status['fps']:.1f} fps  "
                        f"{stats['valid_fraction'] * 100:5.1f}% valid  "
                        f"median {stats['median_m']:5.2f} m  {points_out} points" + " " * 6)
                    sys.stdout.flush()

                if args.duration is not None and now - t_start >= args.duration:
                    break
        except KeyboardInterrupt:
            print("\nInterrupted.")

        elapsed = time.monotonic() - t_start
        print(f"\n{frames} frames in {elapsed:.1f}s "
              f"({frames / elapsed if elapsed else 0:.1f} fps)")
        if frames == 0:
            print("  No depth frames arrived. Run --selftest.")
            return 1
        if args.mcap:
            print(f"Drag {os.path.expanduser(args.mcap)} into Foxglove to scrub it.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
