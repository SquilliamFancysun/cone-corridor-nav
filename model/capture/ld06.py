"""LD06 lidar wire protocol: bytes in, full-revolution scans out.

Pure Python, no pyserial and no foxglove — this module never touches the port,
which means the parts that can be wrong quietly (CRC, angle interpolation, the
revolution boundary) are exercisable by pytest on a laptop with no car attached.
lidar_view.py owns the serial port and hands the bytes here.

The Scan produced here is meant to be the representation cone_perception's
lidar_cluster.py consumes, so that when the nav stack needs the lidar as a ROS
topic the node is a thin rclpy wrapper rather than a second decoder.

Frame format (47 bytes, fixed):

    54 2c  speed:u16  start:u16  [dist:u16 intensity:u8] x12  end:u16  ts:u16  crc:u8

Angles are hundredths of a degree, distances millimetres, speed degrees/second.
"""

import math
import time

HEADER = 0x54
VERLEN = 0x2C  # version 1, 12 points per packet — the only variant the LD06 emits
POINTS_PER_PACKET = 12
PACKET_LEN = 11 + 3 * POINTS_PER_PACKET  # 47
_MAGIC = bytes((HEADER, VERLEN))


def _build_crc_table():
    """LD06's CRC8: poly 0x4D, init 0x00, MSB-first, no reflection, no final xor.

    Built rather than pasted as a 256-entry literal so the polynomial stays
    visible — a pasted table is unreviewable and impossible to tell from a
    typo'd one.
    """
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return tuple(table)


CRC_TABLE = _build_crc_table()


def crc8(data):
    """CRC over the first 46 bytes of a packet; the 47th is the expected value."""
    crc = 0
    for byte in data:
        crc = CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


class Packet(object):
    """One decoded 12-point packet.

    `points` is [(angle_deg, dist_mm, intensity), ...] with angles already
    interpolated across the packet and wrapped into [0, 360).
    """

    __slots__ = ("speed_dps", "start_angle", "end_angle", "points", "timestamp_ms")

    def __init__(self, speed_dps, start_angle, end_angle, points, timestamp_ms):
        self.speed_dps = speed_dps
        self.start_angle = start_angle
        self.end_angle = end_angle
        self.points = points
        self.timestamp_ms = timestamp_ms

    @property
    def speed_hz(self):
        return self.speed_dps / 360.0

    def __repr__(self):
        return (f"Packet({self.start_angle:.2f}->{self.end_angle:.2f} deg, "
                f"{len(self.points)} pts, {self.speed_hz:.2f} Hz)")


def decode_packet(buf):
    """Decode 47 bytes into a Packet, or None if the CRC fails.

    Assumes buf[0:2] is already known to be the magic. Callers count the Nones:
    a marginal cable or a browning-out motor rail shows up as a rising CRC drop
    rate long before it shows up as an obviously broken picture.
    """
    if crc8(buf[:PACKET_LEN - 1]) != buf[PACKET_LEN - 1]:
        return None

    speed = buf[2] | (buf[3] << 8)
    start_raw = buf[4] | (buf[5] << 8)
    end_raw = buf[-5] | (buf[-4] << 8)
    timestamp = buf[-3] | (buf[-2] << 8)

    # A packet that crosses 0 has end < start. Unwrap before interpolating,
    # otherwise the twelve points inside it fan backwards across the whole
    # circle instead of spanning the ~5 degrees they actually cover.
    span_end = end_raw + 36000 if end_raw < start_raw else end_raw
    step = (span_end - start_raw) / float(POINTS_PER_PACKET - 1)

    points = []
    for i in range(POINTS_PER_PACKET):
        off = 6 + 3 * i
        dist = buf[off] | (buf[off + 1] << 8)
        intensity = buf[off + 2]
        angle = ((start_raw + step * i) % 36000) / 100.0
        points.append((angle, dist, intensity))

    return Packet(speed / 1.0, start_raw / 100.0, end_raw / 100.0, points, timestamp)


class LD06Decoder(object):
    """Byte stream -> packets, resynchronising over garbage.

    Serial gives us arbitrary chunk boundaries and, on a bad link, occasional
    dropped bytes. On a CRC failure we advance a single byte and hunt for the
    next magic rather than discarding the buffer, so one corrupt packet costs
    one packet instead of everything queued behind it.
    """

    def __init__(self):
        self._buf = bytearray()
        self.packets_ok = 0
        self.packets_bad = 0
        self.bytes_in = 0
        self.bytes_discarded = 0

    @property
    def drop_rate(self):
        total = self.packets_ok + self.packets_bad
        return self.packets_bad / float(total) if total else 0.0

    def feed(self, data):
        """Add bytes, return whatever complete packets they completed."""
        self.bytes_in += len(data)
        self._buf.extend(data)
        out = []

        while True:
            start = self._buf.find(_MAGIC)
            if start < 0:
                # Keep a trailing byte: the magic may straddle this chunk and
                # the next one.
                keep = 1 if self._buf and self._buf[-1] == HEADER else 0
                self.bytes_discarded += len(self._buf) - keep
                del self._buf[:len(self._buf) - keep]
                return out
            if start:
                self.bytes_discarded += start
                del self._buf[:start]
            if len(self._buf) < PACKET_LEN:
                return out

            packet = decode_packet(self._buf[:PACKET_LEN])
            if packet is None:
                self.packets_bad += 1
                self.bytes_discarded += 1
                del self._buf[:1]  # not a real packet; resync past this magic
                continue

            self.packets_ok += 1
            out.append(packet)
            del self._buf[:PACKET_LEN]


class Scan(object):
    """One revolution. Angles in degrees as the sensor reports them.

    Deliberately stores the sensor's own bearings rather than a corrected
    heading: the mount sign and yaw offset are applied at the point of use (see
    to_xy) and recorded in the manifest, so a session captured with the sign
    backwards is fixable at a desk instead of re-driven.
    """

    __slots__ = ("t", "angles_deg", "ranges_mm", "intensities", "speed_hz")

    def __init__(self, t, angles_deg, ranges_mm, intensities, speed_hz):
        self.t = t
        self.angles_deg = angles_deg
        self.ranges_mm = ranges_mm
        self.intensities = intensities
        self.speed_hz = speed_hz

    def __len__(self):
        return len(self.angles_deg)

    def to_xy(self, mirror=False, angle_offset_deg=0.0, min_mm=1):
        """Cartesian points in the ROS/Foxglove convention: x forward, y left.

        The LD06's bearing increases clockwise viewed from above; REP-103
        measures counterclockwise about +z. Mounting the unit upside down
        mirrors it again. Both collapse into one sign, which `mirror` flips —
        see the angle-convention section of model/capture/README.md. Returns
        [(x_m, y_m), ...] dropping zero-distance points, which are the LD06's
        "no return", not a hit at the origin.
        """
        sign = 1.0 if mirror else -1.0
        out = []
        for angle, dist in zip(self.angles_deg, self.ranges_mm):
            if dist < min_mm:
                continue
            theta = math.radians(sign * angle + angle_offset_deg)
            r = dist / 1000.0
            out.append((r * math.cos(theta), r * math.sin(theta)))
        return out

    def __repr__(self):
        return f"Scan(t={self.t:.3f}, {len(self)} pts, {self.speed_hz:.2f} Hz)"


class ScanAssembler(object):
    """Packets -> Scans, cut at the revolution boundary.

    The boundary is where the reported bearing stops increasing. The LD06 has
    no revolution counter, so this is the only signal available; a dropped
    packet costs a few points, never a whole scan.
    """

    def __init__(self, min_points=100):
        # A "scan" of a handful of points is a resync artefact, not a
        # revolution. Emitting it would put a near-empty frame into the
        # recording that the clusterer then has to defend against.
        self.min_points = min_points
        self._angles = []
        self._ranges = []
        self._intensities = []
        self._speeds = []
        self._last_angle = None
        self._t0 = None
        # We always join the stream mid-rotation, so the first revolution we
        # accumulate is a partial one by construction — how partial depends on
        # where the sensor happened to be. Dropping it costs 100 ms at startup
        # and keeps every scan in a recording a whole revolution, which is what
        # the clusterer is entitled to assume.
        self._dropped_partial = False

    def add(self, packet, t=None):
        """Add one packet. Returns a Scan when this packet closed a revolution."""
        t = time.monotonic() if t is None else t
        scan = None
        if self._last_angle is not None and packet.start_angle < self._last_angle:
            scan = self._emit(t)
        self._last_angle = packet.start_angle

        if self._t0 is None:
            self._t0 = t
        for angle, dist, intensity in packet.points:
            self._angles.append(angle)
            self._ranges.append(dist)
            self._intensities.append(intensity)
        self._speeds.append(packet.speed_hz)
        return scan

    def _emit(self, t):
        angles, ranges = self._angles, self._ranges
        intensities, speeds = self._intensities, self._speeds
        self._angles, self._ranges = [], []
        self._intensities, self._speeds = [], []
        started = self._t0
        self._t0 = None
        if not self._dropped_partial:
            self._dropped_partial = True
            return None
        if len(angles) < self.min_points:
            return None
        return Scan(
            t=started if started is not None else t,
            angles_deg=angles,
            ranges_mm=ranges,
            intensities=intensities,
            speed_hz=sum(speeds) / len(speeds) if speeds else 0.0,
        )


def bin_scan(scan, bins=450, mirror=False, angle_offset_deg=0.0):
    """Resample a scan onto `bins` equally-spaced bearings, 0..2pi.

    Foxglove's LaserScan assumes ranges sit at equal angular steps between
    start_angle and end_angle, and the LD06's points do not: rotation jitters,
    and packets straddle the boundary. Binning is only for what gets drawn —
    the raw per-point bearings go into the recording untouched.

    Returns metres, with NaN where nothing was measured, so gaps render as gaps
    rather than as a wall at the origin.
    """
    sign = 1.0 if mirror else -1.0
    out = [float("nan")] * bins
    for angle, dist in zip(scan.angles_deg, scan.ranges_mm):
        if dist < 1:
            continue
        theta = math.radians(sign * angle + angle_offset_deg) % (2 * math.pi)
        idx = int(theta / (2 * math.pi) * bins) % bins
        metres = dist / 1000.0
        # Nearest return wins a contested bin: for corridor following an
        # under-reported obstacle is the dangerous error.
        if math.isnan(out[idx]) or metres < out[idx]:
            out[idx] = metres
    return out
