"""The driving tool's testable half: the actuator mapping and the safety gates.

Everything here runs without a car. What it covers is what can be wrong
silently -- a steering sign, a mode that actuates when it should not, a deadman
that fails open. The parts that cannot be tested at a desk (does the servo
actually turn that way) are exactly the parts `--steer-only` on a stand exists
for, and the sign test below only pins our own convention, not the car's.
"""

import pytest

import drive_corridor


class FakeVesc:
    """Stands in for pyvesc.VESC, recording what it was told."""

    def __init__(self):
        self.servo = None
        self.duty = None
        self.calls = []
        self.heartbeat = True

    def set_servo(self, value):
        self.servo = value
        self.calls.append(("servo", value))

    def set_duty_cycle(self, value):
        self.duty = value
        self.calls.append(("duty", value))

    def stop_heartbeat(self):
        self.heartbeat = False


def driver(invert=False, vesc=None):
    """A VescDriver with its serial link replaced. __init__ opens a real port,
    so it is bypassed rather than mocked at the import site."""
    d = drive_corridor.VescDriver.__new__(drive_corridor.VescDriver)
    d.steering_scale = 0.5
    d.steering_offset = 0.5
    d.max_duty_percent = 0.2
    d.invert = -1.0 if invert else 1.0
    d.last_servo = 0.5
    d.vesc = vesc if vesc is not None else FakeVesc()
    return d


# --- the steering mapping -----------------------------------------------

def test_straight_ahead_is_servo_centre():
    assert driver().servo_for(0.0) == pytest.approx(0.5)


def test_full_left_and_right_reach_the_servo_limits():
    d = driver()
    assert d.servo_for(1.0) == pytest.approx(1.0)
    assert d.servo_for(-1.0) == pytest.approx(0.0)


def test_the_mapping_matches_donkeycar_s_constants():
    """servo = angle * 0.5 + 0.5, from myconfig_capture.py. If this drifts, the
    car handles differently under our code than under manual control."""
    for value in (-1.0, -0.5, 0.0, 0.25, 1.0):
        assert driver().servo_for(value) == pytest.approx(value * 0.5 + 0.5)


def test_inverting_mirrors_the_command_about_centre():
    normal, inverted = driver(), driver(invert=True)
    for value in (-1.0, -0.3, 0.3, 1.0):
        assert (normal.servo_for(value) - 0.5) == pytest.approx(
            -(inverted.servo_for(value) - 0.5))


def test_inverting_does_not_move_centre():
    """A sign flip must not introduce a trim offset."""
    assert driver(invert=True).servo_for(0.0) == pytest.approx(0.5)


def test_the_servo_command_is_clamped_to_its_range():
    """pure_pursuit already saturates, but a scale change here must not be able
    to command the servo past its stops."""
    d = driver()
    d.steering_scale = 5.0
    assert 0.0 <= d.servo_for(1.0) <= 1.0
    assert 0.0 <= d.servo_for(-1.0) <= 1.0


def test_a_left_steer_and_a_right_steer_land_on_opposite_sides_of_centre():
    """Our convention is left positive. Which physical direction that is on
    this car is what --steer-only on a stand decides; this only pins that the
    two are not the same."""
    d = driver()
    assert d.servo_for(0.6) > 0.5 > d.servo_for(-0.6)


# --- the throttle -------------------------------------------------------

def test_duty_is_capped_not_scaled():
    """speed_ctrl hands over an absolute duty, not DonkeyCar's -1..1 throttle.
    Multiplying by max_duty_percent would run the car at a fifth of the
    commanded speed and look like a mysteriously feeble motor."""
    fake = FakeVesc()
    driver(vesc=fake).drive(0.0, 0.10)
    assert fake.duty == pytest.approx(0.10)


def test_duty_above_the_ceiling_is_clipped():
    fake = FakeVesc()
    driver(vesc=fake).drive(0.0, 0.9)
    assert fake.duty == pytest.approx(0.2)


def test_stop_zeroes_the_throttle_and_centres_the_wheels():
    fake = FakeVesc()
    d = driver(vesc=fake)
    d.drive(0.8, 0.1)
    d.stop()
    assert fake.duty == 0.0
    assert fake.servo == pytest.approx(0.5)


def test_stop_survives_a_dead_serial_link():
    """Every exception path calls stop(), including ones where the link is
    already gone. It must not raise on the way out and mask the real error."""
    class Broken(FakeVesc):
        def set_duty_cycle(self, value):
            raise OSError("device disconnected")

    driver(vesc=Broken()).stop()  # must not raise


def test_close_stops_before_dropping_the_heartbeat():
    fake = FakeVesc()
    d = driver(vesc=fake)
    d.drive(0.5, 0.1)
    d.close()
    assert fake.duty == 0.0
    assert fake.heartbeat is False


# --- the modes ----------------------------------------------------------

def test_dry_run_and_steer_only_are_mutually_exclusive():
    with pytest.raises(SystemExit):
        drive_corridor.parse_args(["--dry-run", "--steer-only"])


def test_the_default_mode_is_the_one_that_drives():
    assert drive_corridor.parse_args([]).mode == "drive"


@pytest.mark.parametrize("flag,mode", [("--dry-run", "dry-run"),
                                       ("--steer-only", "steer-only")])
def test_the_safe_modes_are_named(flag, mode):
    assert drive_corridor.parse_args([flag]).mode == mode


def test_running_without_a_deadman_is_refused_unless_nothing_can_move():
    """--no-deadman exists for computing at a desk. It must never combine with
    a mode that opens the VESC."""
    with pytest.raises(SystemExit):
        drive_corridor.parse_args(["--no-deadman"])
    with pytest.raises(SystemExit):
        drive_corridor.parse_args(["--no-deadman", "--steer-only"])
    assert drive_corridor.parse_args(["--no-deadman", "--dry-run"]).mode == "dry-run"


def test_the_default_lookahead_is_the_one_the_sim_chose():
    assert drive_corridor.parse_args([]).lookahead == pytest.approx(1.0)


# --- the deadman --------------------------------------------------------

class FakeJoystick:
    def __init__(self, events):
        self.connected = True
        self._events = list(events)

    def poll(self):
        events, self._events = self._events, []
        return events

    def close(self):
        self.connected = False


class FakeEvent:
    def __init__(self, number, value):
        self.number = number
        self.value = value
        self.is_button = True


def deadman_with(joystick):
    d = drive_corridor.Deadman.__new__(drive_corridor.Deadman)
    d.button = drive_corridor.DEADMAN_BUTTON
    d.held = False
    d.present = True
    d.joystick = joystick
    return d


def test_the_deadman_starts_unheld():
    """It must fail closed: a tool that comes up armed would drive on startup."""
    assert deadman_with(FakeJoystick([])).held is False


def test_pressing_and_releasing_the_button():
    d = deadman_with(FakeJoystick([FakeEvent(drive_corridor.DEADMAN_BUTTON, 1)]))
    assert d.poll() is True
    d.joystick._events = [FakeEvent(drive_corridor.DEADMAN_BUTTON, 0)]
    assert d.poll() is False


def test_it_stays_held_across_a_poll_with_no_events():
    """joydev only reports transitions, so a held button is silence."""
    d = deadman_with(FakeJoystick([FakeEvent(drive_corridor.DEADMAN_BUTTON, 1)]))
    assert d.poll() is True
    assert d.poll() is True


def test_other_buttons_do_not_arm_it():
    """DonkeyCar binds A and B, and every process on the pad sees every press."""
    for other in (0, 1, 3, 7):
        if other == drive_corridor.DEADMAN_BUTTON:
            continue
        d = deadman_with(FakeJoystick([FakeEvent(other, 1)]))
        assert d.poll() is False


def test_a_yanked_receiver_disarms():
    """The failure that matters most: the pad vanishing must stop the car, not
    leave it holding the last state it saw."""
    d = deadman_with(FakeJoystick([FakeEvent(drive_corridor.DEADMAN_BUTTON, 1)]))
    assert d.poll() is True
    d.joystick.connected = False
    assert d.poll() is False


def test_no_joystick_at_all_is_never_armed():
    d = drive_corridor.Deadman.__new__(drive_corridor.Deadman)
    d.button, d.held, d.present, d.joystick = 2, False, False, None
    assert d.poll() is False


# --- the device path ----------------------------------------------------

def test_the_vesc_port_prefers_the_stable_by_id_path(monkeypatch):
    """ttyACM0 is positional and this car reboots at the track. A renumbered
    node would either fail to open or -- worse on a bus with two CDC devices --
    open the wrong one."""
    monkeypatch.setattr(drive_corridor.os.path, "exists", lambda p: True)
    assert drive_corridor.default_vesc_port() == drive_corridor.DEFAULT_VESC_BY_ID
    assert drive_corridor.default_vesc_port().startswith("/dev/serial/by-id/")


def test_it_falls_back_to_the_positional_name_when_by_id_is_absent():
    """A different VESC reports a different serial, so the by-id name differs.
    Better to start on the positional path than to refuse outright."""
    import os as _os
    real = _os.path.exists
    try:
        _os.path.exists = lambda p: False
        assert drive_corridor.default_vesc_port() == drive_corridor.FALLBACK_VESC_PORT
    finally:
        _os.path.exists = real
