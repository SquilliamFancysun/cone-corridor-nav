"""Non-blocking Linux joydev reader for the F710, with no third-party deps.

`evdev` is not installed on the car and `pygame` wants an SDL video driver we
do not have over SSH, so this reads /dev/input/js0 directly. joydev hands each
open file its own event stream, which is what lets this run alongside DonkeyCar
holding the same pad open to drive the VESC.

Run it directly to find out what a button is called:
    python joystick.py --probe-buttons
"""

import argparse
import os
import struct
import time

# struct js_event: __u32 time; __s16 value; __u8 type; __u8 number;
_EVENT_FORMAT = "IhBB"
_EVENT_SIZE = struct.calcsize(_EVENT_FORMAT)

TYPE_BUTTON = 0x01
TYPE_AXIS = 0x02
TYPE_INIT = 0x80

DEFAULT_DEVICE = "/dev/input/js0"


class JoystickEvent:
    __slots__ = ("time_ms", "value", "type", "number")

    def __init__(self, time_ms, value, type_, number):
        self.time_ms = time_ms
        self.value = value
        self.type = type_
        self.number = number

    @property
    def is_button(self):
        return bool(self.type & TYPE_BUTTON)

    @property
    def is_axis(self):
        return bool(self.type & TYPE_AXIS)

    @property
    def is_init(self):
        return bool(self.type & TYPE_INIT)

    @property
    def pressed(self):
        return self.is_button and self.value == 1

    def __repr__(self):
        kind = "button" if self.is_button else "axis"
        return f"<{kind} {self.number} = {self.value}{' init' if self.is_init else ''}>"


class JoystickNotFound(OSError):
    """The device node does not exist — usually the receiver is unplugged."""


class Joystick:
    """Non-blocking reader. `poll()` returns the events since the last call."""

    def __init__(self, path=DEFAULT_DEVICE, include_init=False):
        self.path = path
        self.include_init = include_init
        try:
            self._fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except FileNotFoundError as exc:
            raise JoystickNotFound(
                f"{path} does not exist — is the F710 receiver plugged in and the "
                f"pad switched on? (check: ls /dev/input/js*)"
            ) from exc
        self.connected = True

    def poll(self):
        """Drain and return pending events. Never blocks.

        joydev replays the full initial state as TYPE_INIT events on open; those
        are filtered out by default so a button already held at startup does not
        read as a fresh press.
        """
        events = []
        while self.connected:
            try:
                data = os.read(self._fd, _EVENT_SIZE)
            except BlockingIOError:
                break
            except OSError:
                # Receiver yanked mid-session.
                self.connected = False
                break
            if not data or len(data) < _EVENT_SIZE:
                break
            time_ms, value, type_, number = struct.unpack(_EVENT_FORMAT, data)
            event = JoystickEvent(time_ms, value, type_, number)
            if event.is_init and not self.include_init:
                continue
            events.append(event)
        return events

    def close(self):
        if getattr(self, "_fd", None) is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None
        self.connected = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument(
        "--probe-buttons",
        action="store_true",
        help="print each button press with its index, so you can pick one for --record-button",
    )
    parser.add_argument("--axes", action="store_true", help="also print axis motion")
    args = parser.parse_args()

    if not args.probe_buttons and not args.axes:
        args.probe_buttons = True

    print(f"Reading {args.device}. Press buttons; Ctrl+C to stop.")
    print("The F710 renumbers its buttons between the X and D switch positions,")
    print("so probe with the switch where you will actually drive it.\n")
    try:
        with Joystick(args.device) as js:
            while js.connected:
                for event in js.poll():
                    if event.is_button and args.probe_buttons and event.pressed:
                        print(f"  button {event.number} pressed"
                              f"   -->  --record-button {event.number}")
                    elif event.is_axis and args.axes and abs(event.value) > 3000:
                        print(f"  axis {event.number} = {event.value}")
                time.sleep(0.01)
            print("Device disconnected.")
    except JoystickNotFound as exc:
        raise SystemExit(f"error: {exc}")
    except KeyboardInterrupt:
        print("\nDone.")


if __name__ == "__main__":
    main()
