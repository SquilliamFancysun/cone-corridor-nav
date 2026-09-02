"""Given turn list -> per-junction command as gates are passed. Pure, no rclpy.

The route is *provided* to the vehicle (README: "left at the first junction,
right at the second"); planning one is a nice-to-have. So this module is two
small things: read the file a human wrote, and hold a finger on the line the car
is currently executing.

## Why the parser refuses rather than skips

`load_route` raises on anything it does not recognise, naming the line number,
and `drive_junction.py` treats a missing or empty file as fatal. That matches
`fusion_view.resolve_calibration`, and for the same reason: this file steers a
car. A typo that silently becomes "no turn here" is a car that drives into a
dead end at the first junction and reports nothing unusual in the log.

## Why the cursor cannot be advanced by distance

There is no odometry anywhere in this repo -- every tick is computed fresh in
base_link and nothing integrates. So a route entry is consumed when
`topo_state` says a gate was passed, and never on a timer or a travelled
distance. `advance()` is deliberately dumb about *when*; owning that decision is
`topo_state`'s job, and the debounce lives there.
"""

LEFT = "left"
RIGHT = "right"
TURNS = (LEFT, RIGHT)


def parse_route(text):
    """Route text -> list of turns. One per line, `#` comments, blank lines OK.

    Split out from `load_route` so the format can be tested without a file, and
    so a route embedded in a test or a sim invocation goes through exactly the
    same validation as one read from disk.
    """
    turns = []
    for number, raw in enumerate(text.splitlines(), start=1):
        line = raw.split("#", 1)[0].strip().lower()
        if not line:
            continue
        if line not in TURNS:
            raise ValueError(
                f"line {number}: {raw.strip()!r} is not a turn. "
                f"Every non-comment line must be one of {list(TURNS)}.")
        turns.append(line)
    return turns


def load_route(path):
    """Read and validate a route file.

    An empty route is an error, not an empty plan. A file with no turns in it
    is far more likely to be an unfinished edit than a deliberate statement
    that the track has no junctions -- and if it really has none,
    `drive_corridor.py` is the tool for that.
    """
    try:
        with open(path, encoding="utf-8") as handle:
            text = handle.read()
    except OSError as exc:
        raise ValueError(f"cannot read route file {path!r}: {exc}") from exc

    turns = parse_route(text)
    if not turns:
        raise ValueError(
            f"route file {path!r} contains no turns. Expected one of "
            f"{list(TURNS)} per line.")
    return turns


class RouteCursor(object):
    """Which junction the car is executing, and what to do at it.

    `current` is None once the route is spent. That is not an error state: a
    car that has taken every turn it was given is in the last corridor, and
    `topo_state` reads a None here as "stop arming for junctions" so a red
    triple glimpsed past the final turn cannot restart the machine.
    """

    __slots__ = ("turns", "index")

    def __init__(self, turns):
        self.turns = list(turns)
        self.index = 0

    @property
    def current(self):
        if self.exhausted:
            return None
        return self.turns[self.index]

    @property
    def exhausted(self):
        return self.index >= len(self.turns)

    @property
    def remaining(self):
        return max(0, len(self.turns) - self.index)

    def advance(self):
        """Consume the current turn. Idempotent past the end.

        Returns the turn that was consumed, or None if the route was already
        spent -- so a caller that double-fires on one gate cannot walk the
        index off into the next junction's entry.
        """
        if self.exhausted:
            return None
        turn = self.turns[self.index]
        self.index += 1
        return turn

    def __repr__(self):
        done = "spent" if self.exhausted else f"at {self.current}"
        return f"RouteCursor({self.index}/{len(self.turns)}, {done})"
