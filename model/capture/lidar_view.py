"""Live Foxglove view of the LD06 lidar, with gamepad-triggered recording.

Runs on the car in its own pane, alongside DonkeyCar driving by wire — the
lidar is not the camera, so nothing here contends with capture_cones.py and all
three can run at once.

    python lidar_view.py --session-label lot-A

Then connect Foxglove Studio on the laptop to ws://robocar:8765.

Not a ROS node, for the same reason capture_cones.py is not one: a recording
that needs the class container up to replay is a recording Person B cannot use
on a laptop. ld06.py decodes the wire format into a plain Scan, which is what
cone_perception/lidar_cluster.py will consume — so when the nav stack does want
a ROS topic, that node is a thin rclpy wrapper, not a second decoder.

Only one process may hold the serial port. Do not run the container's lidar
driver at the same time as this.
"""

import argparse
import math
import os
import sys
import time
from contextlib import ExitStack

from ld06 import LD06Decoder, ScanAssembler, bin_scan
from session import ScanSessionWriter

try:
    from joystick import DEFAULT_DEVICE, Joystick, JoystickNotFound
except ImportError:  # pragma: no cover - joystick.py ships beside this file
    DEFAULT_DEVICE = "/dev/input/js0"
    Joystick = JoystickNotFound = None

# By-id, never ttyUSB0: the positional name renumbers the moment anything else
# joins the bus. See docs/hardware-baseline.md.
DEFAULT_PORT = ("/dev/serial/by-id/"
                "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0")
BAUD = 230400
DEFAULT_BINS = 450  # ~0.8 deg, close to the sensor's own angular resolution

# Declared rather than left to the SDK's placeholder, so the MCAP is
# self-describing: /scan_raw is the lossless copy of the scan, and a copy that
# only Foxglove can interpret is not much of an archive.
RAW_SCHEMA = {
    "type": "object",
    "title": "LD06RawScan",
    "description": "One revolution, sensor bearings uncorrected for mount sign or yaw.",
    "properties": {
        "t": {"type": "number", "description": "seconds since the first scan"},
        "speed_hz": {"type": "number"},
        "angles_deg": {"type": "array", "items": {"type": "number"},
                       "description": "sensor bearing, clockwise from above"},
        "ranges_mm": {"type": "array", "items": {"type": "integer"},
                      "description": "0 means no return, not a hit at the origin"},
        "intensities": {"type": "array", "items": {"type": "integer"}},
    },
}

STATUS_SCHEMA = {
    "type": "object",
    "title": "LidarLinkHealth",
    "properties": {
        "bytes_per_s": {"type": "integer"},
        "packets_per_s": {"type": "number"},
        "packets_ok": {"type": "integer"},
        "packets_bad": {"type": "integer"},
        "crc_drop_rate": {"type": "number"},
        "scans": {"type": "integer"},
        "points_per_scan": {"type": "integer"},
        "recording": {"type": "boolean"},
    },
}


def open_serial(port, baud):
    """Open the lidar port, or exit with the diagnosis that usually applies."""
    try:
        import serial
    except ImportError:
        raise SystemExit(
            "error: needs pyserial.\n"
            "       source ~/env/bin/activate  (donkeycar already depends on it)\n"
            "       or: pip install pyserial")
    try:
        return serial.Serial(port, baud, timeout=0.05)
    except serial.SerialException as exc:
        raise SystemExit(
            f"error: cannot open {port}\n"
            f"       {exc}\n"
            f"       ls -l /dev/serial/by-id/ to see what is actually attached.\n"
            f"       A spinning motor and lit LEDs prove nothing: a charge-only\n"
            f"       micro-USB cable powers the LD06 while carrying no data, and\n"
            f"       the port then never appears. See docs/hardware-baseline.md.")


class Sinks:
    """Foxglove channels feeding the live server and the MCAP file.

    Degrades to a no-op if foxglove-sdk is missing rather than taking the run
    down with it: a failed pip install at the track must not cost the data.
    scans.jsonl is written either way.
    """

    def __init__(self, frame_id="lidar", parent_frame="base_link", mount=(0.0, 0.0, 0.0),
                 mount_yaw=0.0):
        self.frame_id = frame_id
        self.parent_frame = parent_frame
        self.mount = mount
        self.mount_yaw = mount_yaw
        self.available = False
        self.reason = None
        try:
            import foxglove
            try:
                from foxglove import messages as schemas
            except ImportError:  # SDKs before the messages/schemas rename
                from foxglove import schemas
            from foxglove.channels import FrameTransformChannel, LaserScanChannel
        except ImportError as exc:
            self.reason = str(exc)
            return

        self._foxglove = foxglove
        self._s = schemas
        self.scan_ch = LaserScanChannel("/scan")
        self.tf_ch = FrameTransformChannel("/tf")
        self.raw_ch = foxglove.Channel("/scan_raw", schema=RAW_SCHEMA)
        self.status_ch = foxglove.Channel("/lidar_status", schema=STATUS_SCHEMA)
        self.available = True

    def open_mcap(self, path):
        return self._foxglove.open_mcap(path)

    def start_server(self, host, port):
        return self._foxglove.start_server(name="cone-car lidar", host=host, port=port)

    def _stamp(self, wall_s):
        return self._s.Timestamp.from_epoch_secs(wall_s)

    def log_transform(self, wall_s):
        """base_link -> lidar, so the scan sits where the sensor actually is."""
        x, y, z = self.mount
        half = self.mount_yaw / 2.0
        self.tf_ch.log(self._s.FrameTransform(
            timestamp=self._stamp(wall_s),
            parent_frame_id=self.parent_frame,
            child_frame_id=self.frame_id,
            translation=self._s.Vector3(x=x, y=y, z=z),
            rotation=self._s.Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half)),
        ))

    def log_scan(self, scan, wall_s, bins, mirror, angle_offset):
        log_time = int(wall_s * 1e9)
        ranges = bin_scan(scan, bins=bins, mirror=mirror, angle_offset_deg=angle_offset)
        self.scan_ch.log(self._s.LaserScan(
            timestamp=self._stamp(wall_s),
            frame_id=self.frame_id,
            start_angle=0.0,
            end_angle=2 * math.pi * (bins - 1) / bins,
            ranges=ranges,
            intensities=[],
        ), log_time=log_time)
        # Binning is for what gets drawn. The raw bearings go in untouched, so
        # the MCAP is not a lossy version of scans.jsonl.
        self.raw_ch.log({
            "t": round(scan.t, 4),
            "speed_hz": round(scan.speed_hz, 3),
            "angles_deg": [round(a, 2) for a in scan.angles_deg],
            "ranges_mm": list(scan.ranges_mm),
            "intensities": list(scan.intensities),
        }, log_time=log_time)
        self.log_transform(wall_s)

    def log_status(self, status, wall_s):
        self.status_ch.log(status, log_time=int(wall_s * 1e9))


def health(decoder, elapsed, scans, points):
    """The numbers docs/hardware-baseline.md calls a healthy LD06."""
    return {
        "bytes_per_s": int(decoder.bytes_in / elapsed) if elapsed else 0,
        "packets_per_s": round(decoder.packets_ok / elapsed, 1) if elapsed else 0.0,
        "packets_ok": decoder.packets_ok,
        "packets_bad": decoder.packets_bad,
        "crc_drop_rate": round(decoder.drop_rate, 5),
        "scans": scans,
        "points_per_scan": points,
    }


def run_selftest(port, seconds):
    """Three seconds of link stats, then exit. The §3 re-verification check."""
    handle = open_serial(port, BAUD)
    time.sleep(0.3)
    handle.reset_input_buffer()
    decoder, assembler = LD06Decoder(), ScanAssembler()
    scans, last = 0, 0
    t0 = time.monotonic()
    speeds = []
    while time.monotonic() - t0 < seconds:
        for packet in decoder.feed(handle.read(4096)):
            speeds.append(packet.speed_hz)
            scan = assembler.add(packet)
            if scan is not None:
                scans += 1
                last = len(scan)
    handle.close()
    elapsed = time.monotonic() - t0
    stats = health(decoder, elapsed, scans, last)
    hz = sum(speeds) / len(speeds) if speeds else 0.0

    print(f"{stats['bytes_per_s']} B/s  {stats['packets_per_s']} packets/s  "
          f"{hz:.2f} Hz  {scans} scans  {last} points/scan")
    print(f"CRC: {decoder.packets_ok} ok, {decoder.packets_bad} bad "
          f"({stats['crc_drop_rate'] * 100:.2f}%)")

    if decoder.packets_ok == 0:
        # The failure that costs the most time on this car, and it does not
        # look like a data problem: the motor spins and both LEDs light.
        print("  The port opened but produced no valid packets.")
        if decoder.bytes_in == 0:
            print("  Not a single byte arrived. The LD06 is powered but not "
                  "talking — suspect a\n  charge-only micro-USB cable, which "
                  "spins the motor and lights both LEDs\n  while carrying no "
                  "data. Test the cable on a phone or USB stick.")
        else:
            print(f"  {decoder.bytes_in} bytes arrived but none decoded "
                  f"({decoder.packets_bad} failed CRC).\n"
                  f"  Wrong baud, or something else is on this port.")
        return 1

    ok = True
    if not 9.9 <= hz <= 10.1:
        print(f"  warning: rotation {hz:.2f} Hz is outside the 9.9-10.1 baseline. "
              f"Check the lidar's 5 V rail — the hub, not the Pi.")
        ok = False
    if decoder.drop_rate > 0.01:
        print(f"  warning: {stats['crc_drop_rate'] * 100:.1f}% of packets fail CRC. "
              f"Suspect the cable before the code.")
        ok = False
    if stats["bytes_per_s"] < 15000:
        print(f"  warning: {stats['bytes_per_s']} B/s is below the ~19 KB/s baseline.")
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
    parser.add_argument("--session-label", default="session",
                        help="describes the conditions, e.g. lot-A; becomes the "
                             "session directory name")
    parser.add_argument("--out-root", default="~/lidar_capture")
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help="lidar serial device; by-id by default because "
                             "ttyUSB0 is positional and renumbers")
    parser.add_argument("--host", default="0.0.0.0",
                        help="Foxglove server bind address; the SDK default of "
                             "127.0.0.1 is unreachable from the laptop (default: 0.0.0.0)")
    parser.add_argument("--port-ws", type=int, default=8765, help="Foxglove server port")
    parser.add_argument("--bins", type=int, default=DEFAULT_BINS,
                        help=f"angular bins for the drawn scan (default: {DEFAULT_BINS})")
    parser.add_argument("--mirror", action="store_true",
                        help="flip the bearing sign; set this from the cone check "
                             "in README.md, not from reasoning about the mount")
    parser.add_argument("--angle-offset", type=float, default=0.0,
                        help="degrees of yaw between the lidar's zero and forward")
    parser.add_argument("--mount-x", type=float, default=0.0,
                        help="lidar position in base_link, metres")
    parser.add_argument("--mount-y", type=float, default=0.0)
    parser.add_argument("--mount-z", type=float, default=0.0)
    parser.add_argument("--mount-yaw", type=float, default=0.0, help="degrees")
    parser.add_argument("--device", default=DEFAULT_DEVICE, help="joystick device node")
    parser.add_argument("--record-button", type=int, default=1,
                        help="joystick button that toggles recording; 1 = B, kept "
                             "off capture_cones.py's A so both tools can share a pad")
    parser.add_argument("--no-joystick", action="store_true",
                        help="record immediately without a gamepad; needs --duration")
    parser.add_argument("--duration", type=float, default=None,
                        help="stop after this many seconds of recording")
    parser.add_argument("--no-live", action="store_true",
                        help="skip the Foxglove server; record only")
    parser.add_argument("--no-mcap", action="store_true", help="skip the MCAP file")
    parser.add_argument("--no-jsonl", action="store_true", help="skip scans.jsonl")
    parser.add_argument("--selftest", action="store_true",
                        help="3s of link stats against the hardware baseline, then exit")
    parser.add_argument("--dump-raw", default=None,
                        help="also write raw serial bytes here, for test fixtures")
    parser.add_argument("--notes", default=None, help="free text into session.json")
    args = parser.parse_args(argv)

    if args.no_joystick and args.duration is None and not args.selftest:
        parser.error("--no-joystick needs --duration (nothing would ever stop it)")
    if args.no_mcap and args.no_jsonl:
        parser.error("--no-mcap and --no-jsonl together would record nothing")
    if args.bins < 8:
        parser.error(f"--bins {args.bins} is too coarse to be worth drawing")
    return args


def main(argv=None):
    args = parse_args(argv)

    if args.selftest:
        return run_selftest(args.port, 3.0)

    joystick = None
    if not args.no_joystick:
        if Joystick is None:
            raise SystemExit("error: joystick.py not importable; use --no-joystick --duration N")
        try:
            joystick = Joystick(args.device)
        except JoystickNotFound as exc:
            raise SystemExit(
                f"error: {exc}\n"
                f"       (or run with --no-joystick --duration N to record without one)")

    sinks = Sinks(mount=(args.mount_x, args.mount_y, args.mount_z),
                  mount_yaw=math.radians(args.mount_yaw))
    if not sinks.available:
        if args.no_jsonl:
            raise SystemExit(
                f"error: foxglove-sdk is not available ({sinks.reason}), so the MCAP\n"
                f"       cannot be written, and --no-jsonl disables the other half.\n"
                f"       This run would record nothing. Drop --no-jsonl, or:\n"
                f"       pip install foxglove-sdk   (needs Python 3.10+)")
        print(f"note: foxglove-sdk not available ({sinks.reason}).")
        print("      No live view and no MCAP; scans.jsonl still records.")
        print("      pip install foxglove-sdk   (needs Python 3.10+)")

    handle = open_serial(args.port, BAUD)
    decoder, assembler = LD06Decoder(), ScanAssembler()
    # Scans carry monotonic time; Foxglove wants wall clock. One offset, taken
    # once, keeps the two consistent across the session.
    wall_offset = time.time() - time.monotonic()

    meta = {
        "port": args.port,
        "baud": BAUD,
        "lidar": "LDRobot LD06",
        "direction": "cw_native",
        "mirrored": args.mirror,
        "angle_offset_deg": args.angle_offset,
        "mount": {"x": args.mount_x, "y": args.mount_y, "z": args.mount_z,
                  "yaw_deg": args.mount_yaw},
        "bins": args.bins,
        "track": "data/layouts/track_v1.md",
        "git_commit": tool_commit(),
    }

    with ExitStack() as stack:
        stack.callback(handle.close)
        if joystick is not None:
            stack.callback(joystick.close)
        raw_dump = None
        if args.dump_raw:
            raw_dump = stack.enter_context(open(os.path.expanduser(args.dump_raw), "wb"))

        if sinks.available and not args.no_live:
            server = sinks.start_server(args.host, args.port_ws)
            stack.callback(server.stop)
            print(f"Foxglove: ws://{args.host}:{args.port_ws}  "
                  f"(connect Studio to ws://<car>:{args.port_ws})")

        time.sleep(0.3)
        handle.reset_input_buffer()

        writer = None
        mcap = None
        recording = False
        recorded_s = 0.0
        started_at = None
        exit_code = 0
        t_start = time.monotonic()
        last_status = 0.0
        last_scan_len = 0
        scans_seen = 0

        def start():
            nonlocal recording, started_at, writer, mcap
            if writer is None:
                writer = ScanSessionWriter(args.out_root, args.session_label, meta,
                                           jsonl=not args.no_jsonl)
                if sinks.available and not args.no_mcap:
                    mcap = stack.enter_context(sinks.open_mcap(writer.mcap_path))
                print(f"\nSession: {writer.dir}")
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
            print(f"\nReady. Button {args.record_button} toggles recording. "
                  f"Ctrl+C to finish.")
            print("Live view runs whether or not you are recording.")

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
                            print(f"\r{state:>6}  {count} scans" + " " * 24)

                chunk = handle.read(4096)
                if chunk:
                    if raw_dump is not None:
                        raw_dump.write(chunk)
                    for packet in decoder.feed(chunk):
                        scan = assembler.add(packet)
                        if scan is None:
                            continue
                        scans_seen += 1
                        last_scan_len = len(scan)
                        wall = scan.t + wall_offset
                        # The live view is always on; recording is what the
                        # button gates. Watching the scan is how you decide
                        # whether a run is worth recording at all.
                        if sinks.available:
                            sinks.log_scan(scan, wall, args.bins, args.mirror,
                                           args.angle_offset)
                        if recording:
                            elapsed = recorded_s + (time.monotonic() - started_at)
                            writer.add_scan(scan)
                            sys.stdout.write(
                                f"\rREC ● {writer.name}  {writer.count} scans  "
                                f"{elapsed:6.1f}s  {scan.speed_hz:5.2f} Hz")
                            sys.stdout.flush()

                now = time.monotonic()
                if sinks.available and now - last_status >= 1.0:
                    stats = health(decoder, now - t_start, scans_seen, last_scan_len)
                    stats["recording"] = recording
                    sinks.log_status(stats, now + wall_offset)
                    last_status = now
                    if not recording:
                        sys.stdout.write(
                            f"\r   live  {scans_seen} scans  "
                            f"{stats['packets_per_s']:.0f} pkt/s  "
                            f"{stats['crc_drop_rate'] * 100:.2f}% CRC drops" + " " * 8)
                        sys.stdout.flush()

                if args.duration is not None and recording:
                    if recorded_s + (time.monotonic() - started_at) >= args.duration:
                        stop()
                        break

                if not chunk:
                    time.sleep(0.002)
        except KeyboardInterrupt:
            stop()
            print("\nInterrupted.")
        finally:
            elapsed = time.monotonic() - t_start
            if writer is not None:
                writer.set_health(**health(decoder, elapsed, scans_seen, last_scan_len))
                path = writer.close(args.notes)
                print(f"\nWrote {writer.count} scans to {path}")
                if decoder.drop_rate > 0.01:
                    print(f"  warning: {decoder.drop_rate * 100:.1f}% of packets failed "
                          f"CRC this session. Check the cable before trusting the data.")
                if mcap is not None:
                    print("Open scans.mcap in Foxglove to scrub it.")
                if not args.no_jsonl:
                    print("scans.jsonl replays with no dependencies installed.")
            else:
                print("\nNothing recorded.")
                exit_code = 1

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
