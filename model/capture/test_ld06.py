"""Tests for the LD06 wire protocol. Runs on a laptop, no car.

Packets are synthesized with real CRCs by build_packet(), so these exercise the
same path a live port does. One recorded fixture (fixtures/ld06_sample.bin, made
with `lidar_view.py --dump-raw`) pins the decoder against hardware that actually
exists — synthesized bytes only ever prove the decoder agrees with itself.
"""

import os

import pytest

from ld06 import (
    HEADER,
    PACKET_LEN,
    POINTS_PER_PACKET,
    VERLEN,
    LD06Decoder,
    Scan,
    ScanAssembler,
    bin_scan,
    crc8,
    decode_packet,
)

FIXTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "fixtures", "ld06_sample.bin")


def build_packet(start_deg=0.0, end_deg=5.5, speed_dps=3600, timestamp_ms=0,
                 points=None):
    """A valid 47-byte packet. points is [(dist_mm, intensity)] x12."""
    points = points or [(1000 + 10 * i, 200) for i in range(POINTS_PER_PACKET)]
    assert len(points) == POINTS_PER_PACKET
    body = bytearray((HEADER, VERLEN))
    body += int(speed_dps).to_bytes(2, "little")
    body += int(round(start_deg * 100)).to_bytes(2, "little")
    for dist, intensity in points:
        body += int(dist).to_bytes(2, "little")
        body.append(intensity)
    body += int(round(end_deg * 100)).to_bytes(2, "little")
    body += int(timestamp_ms).to_bytes(2, "little")
    body.append(crc8(body))
    assert len(body) == PACKET_LEN
    return bytes(body)


class TestCrc:
    def test_table_matches_published_ld06_values(self):
        # First entries of the table in LDRobot's own SDK. If the polynomial is
        # wrong these differ and every packet would be rejected.
        from ld06 import CRC_TABLE
        assert CRC_TABLE[:8] == (0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE)

    def test_round_trips_a_built_packet(self):
        packet = build_packet()
        assert crc8(packet[:-1]) == packet[-1]


class TestDecodePacket:
    def test_decodes_speed_angles_and_points(self):
        packet = build_packet(start_deg=10.0, end_deg=15.5, speed_dps=3600,
                              timestamp_ms=1234)
        decoded = decode_packet(packet)
        assert decoded is not None
        assert decoded.speed_hz == pytest.approx(10.0)
        assert decoded.start_angle == pytest.approx(10.0)
        assert decoded.end_angle == pytest.approx(15.5)
        assert decoded.timestamp_ms == 1234
        assert len(decoded.points) == POINTS_PER_PACKET
        assert decoded.points[0][1] == 1000
        assert decoded.points[0][2] == 200

    def test_interpolates_angles_evenly_across_the_packet(self):
        decoded = decode_packet(build_packet(start_deg=10.0, end_deg=21.0))
        angles = [p[0] for p in decoded.points]
        assert angles[0] == pytest.approx(10.0)
        assert angles[-1] == pytest.approx(21.0)
        gaps = [b - a for a, b in zip(angles, angles[1:])]
        assert all(gap == pytest.approx(1.0) for gap in gaps)

    def test_unwraps_a_packet_that_crosses_zero(self):
        # end < start. The twelve points must span ~6 degrees across the seam,
        # not fan backwards over the whole circle.
        decoded = decode_packet(build_packet(start_deg=357.0, end_deg=3.0))
        angles = [p[0] for p in decoded.points]
        assert angles[0] == pytest.approx(357.0)
        assert angles[-1] == pytest.approx(3.0)
        assert all(0.0 <= a < 360.0 for a in angles)
        unwrapped = [a + 360 if a < 180 else a for a in angles]
        gaps = [b - a for a, b in zip(unwrapped, unwrapped[1:])]
        assert all(gap == pytest.approx(6.0 / 11, abs=1e-6) for gap in gaps)

    def test_rejects_a_corrupted_packet(self):
        packet = bytearray(build_packet())
        packet[10] ^= 0xFF
        assert decode_packet(bytes(packet)) is None


class TestDecoderStream:
    def test_reassembles_packets_split_across_reads(self):
        stream = build_packet(start_deg=0.0) + build_packet(start_deg=6.0)
        decoder = LD06Decoder()
        got = []
        for i in range(0, len(stream), 7):  # chunk boundaries land mid-packet
            got.extend(decoder.feed(stream[i:i + 7]))
        assert len(got) == 2
        assert decoder.packets_ok == 2
        assert decoder.packets_bad == 0

    def test_skips_leading_garbage(self):
        decoder = LD06Decoder()
        got = decoder.feed(b"\x00\x11\x22garbage" + build_packet())
        assert len(got) == 1
        assert decoder.bytes_discarded == 10

    def test_one_corrupt_packet_costs_one_packet(self):
        corrupt = bytearray(build_packet(start_deg=6.0))
        corrupt[-1] ^= 0xFF
        decoder = LD06Decoder()
        got = decoder.feed(build_packet(start_deg=0.0) + bytes(corrupt)
                           + build_packet(start_deg=12.0))
        # The good packets on either side survive; only the bad one is lost.
        assert len(got) == 2
        assert [round(p.start_angle) for p in got] == [0, 12]
        assert decoder.packets_bad == 1
        assert decoder.drop_rate == pytest.approx(1 / 3.0)

    def test_holds_a_trailing_header_byte_across_chunks(self):
        packet = build_packet()
        decoder = LD06Decoder()
        assert decoder.feed(packet[:1]) == []   # a lone 0x54, magic not yet proven
        assert len(decoder.feed(packet[1:])) == 1
        assert decoder.bytes_discarded == 0

    def test_does_not_emit_until_the_packet_is_complete(self):
        packet = build_packet()
        decoder = LD06Decoder()
        assert decoder.feed(packet[:-1]) == []
        assert len(decoder.feed(packet[-1:])) == 1


class TestScanAssembler:
    def _revolution(self, assembler, start=0.0, step=6.0, t=0.0):
        scans = []
        angle = start
        while angle < 360.0:
            packet = decode_packet(build_packet(start_deg=angle % 360.0,
                                                end_deg=(angle + step) % 360.0))
            scan = assembler.add(packet, t=t)
            if scan is not None:
                scans.append(scan)
            angle += step
        return scans

    def test_emits_one_scan_per_revolution(self):
        assembler = ScanAssembler()
        assert self._revolution(assembler) == []        # first revolution fills up
        assert self._revolution(assembler, t=0.1) == []  # closes it, dropped as partial
        scans = self._revolution(assembler, t=0.2)
        assert len(scans) == 1
        assert len(scans[0]) == 60 * POINTS_PER_PACKET
        assert scans[0].speed_hz == pytest.approx(10.0)

    def test_drops_the_first_partial_revolution(self):
        # Joining the stream mid-rotation, the first revolution is short. It
        # must not reach a recording, where a half scan reads as a corridor
        # that abruptly ends.
        assembler = ScanAssembler(min_points=10)
        for angle in range(180, 360, 6):  # start halfway round
            assembler.add(decode_packet(build_packet(start_deg=float(angle),
                                                     end_deg=float(angle + 6) % 360.0)))
        first = self._revolution(assembler)
        assert first == []
        full = self._revolution(assembler)
        assert len(full) == 1
        assert len(full[0]) == 60 * POINTS_PER_PACKET

    def test_scan_timestamp_is_the_start_of_the_revolution(self):
        assembler = ScanAssembler()
        self._revolution(assembler, t=1.0)
        self._revolution(assembler, t=5.0)
        scans = self._revolution(assembler, t=9.0)
        assert scans[0].t == pytest.approx(5.0)

    def test_discards_a_stub_revolution(self):
        # A resync mid-rotation leaves a handful of points that are not a scan.
        # Past the initial partial, so this is min_points doing the work.
        assembler = ScanAssembler(min_points=100)
        self._revolution(assembler)
        self._revolution(assembler)
        assembler.add(decode_packet(build_packet(start_deg=350.0, end_deg=355.0)))
        scans = self._revolution(assembler)
        assert scans == []


class TestGeometry:
    def _scan(self, angle_deg):
        return Scan(t=0.0, angles_deg=[angle_deg], ranges_mm=[1000],
                    intensities=[200], speed_hz=10.0)

    def test_sensor_bearing_maps_to_ros_convention(self):
        # LD06 counts clockwise from above; REP-103 counts counterclockwise with
        # x forward and y left. So a sensor bearing of 90 deg is on the RIGHT.
        (x, y), = self._scan(90.0).to_xy()
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(-1.0)

    def test_mirror_flips_the_bearing_sign(self):
        # This is the flag that upside-down mounting sets. Same reading, other side.
        (_, plain), = self._scan(90.0).to_xy()
        (_, mirrored), = self._scan(90.0).to_xy(mirror=True)
        assert mirrored == pytest.approx(-plain)

    def test_offset_rotates_the_whole_scan(self):
        # The offset is a yaw correction in the car frame, applied after the
        # sign: a return the sensor calls "straight ahead" moves to the left.
        (x, y), = self._scan(0.0).to_xy(angle_offset_deg=90.0)
        assert x == pytest.approx(0.0, abs=1e-9)
        assert y == pytest.approx(1.0)

    def test_drops_no_return_points(self):
        scan = Scan(t=0.0, angles_deg=[0.0, 10.0], ranges_mm=[0, 1000],
                    intensities=[0, 200], speed_hz=10.0)
        assert len(scan.to_xy()) == 1

    def test_bin_scan_places_a_return_at_its_bearing(self):
        # Sensor 90 deg -> ROS -90 deg -> three quarters of the way round.
        bins = bin_scan(self._scan(90.0), bins=360)
        assert bins[270] == pytest.approx(1.0)
        assert sum(1 for b in bins if b == b) == 1  # the rest are NaN

    def test_bin_scan_keeps_the_nearest_return_in_a_contested_bin(self):
        # Sensor 350 and 351 deg both land in ROS bin 0 (bearings 10 and 9 deg).
        scan = Scan(t=0.0, angles_deg=[350.0, 351.0], ranges_mm=[3000, 1500],
                    intensities=[200, 200], speed_hz=10.0)
        bins = bin_scan(scan, bins=8)
        assert bins[0] == pytest.approx(1.5)


@pytest.mark.skipif(not os.path.exists(FIXTURE), reason="no recorded fixture yet")
class TestRecordedFixture:
    """Pins the decoder against bytes the real sensor produced.

    Capture with: python lidar_view.py --dump-raw fixtures/ld06_sample.bin --duration 3
    """

    def test_decodes_the_recording_cleanly(self):
        with open(FIXTURE, "rb") as fh:
            raw = fh.read()
        decoder = LD06Decoder()
        packets = decoder.feed(raw)
        assert decoder.packets_ok > 100
        assert decoder.drop_rate < 0.01
        speeds = [p.speed_hz for p in packets]
        assert 9.0 < sum(speeds) / len(speeds) < 11.0   # hardware-baseline.md

    def test_assembles_full_revolutions(self):
        with open(FIXTURE, "rb") as fh:
            raw = fh.read()
        assembler = ScanAssembler()
        scans = [s for p in LD06Decoder().feed(raw)
                 for s in [assembler.add(p)] if s is not None]
        assert len(scans) > 5
        # ~4500 points/s at ~10 Hz.
        assert all(300 < len(s) < 700 for s in scans)
