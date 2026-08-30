"""Speed control: slow down approaching junctions. Pure function, no rclpy.

## Why this talks in duty cycle and not in metres per second

`set_duty_cycle` on the VESC is open loop. It commands a fraction of bus voltage
to the motor, and the speed that results depends on the battery's state of
charge, the surface, and whether the car is pointing up a slope. Naming the
argument `speed_mps` would put a number in the trial logs and in the report that
reads like a measurement and is not one.

The VESC has an encoder (`VESC_HAS_SENSOR = True` in `myconfig_capture.py`), so
closed-loop `set_rpm` and an honest m/s is available and is the right follow-up.
Until then this module is deliberately, visibly open loop.

## The floor is not zero

Below roughly 4-5% duty a brushless motor under load cogs rather than turns: it
draws current, hums, and the car does not move. So "slow down" has a bottom, and
below it the only two honest commands are the floor and a stop. A derating
formula that slides smoothly to 0.01 would leave the car stalled and buzzing
mid-corridor with a log full of plausible-looking small numbers -- which is why
`MIN_MOVE_DUTY` exists and why `duty()` snaps to it rather than through it.

Both duty constants here are starting guesses, to be replaced by what the bench
test in the plan's Phase 3 actually shows this car doing.
"""

from cone_nav.control.pure_pursuit import clamp

# Full speed for autonomous running. Well under DonkeyCar's
# VESC_MAX_SPEED_PERCENT of 0.2, because that number is the ceiling for a human
# holding a controller who can see the whole track, and this is a first
# autonomous run alongside a person walking.
DEFAULT_MAX_DUTY = 0.10

# Below this the motor cogs instead of turning. See the module docstring.
MIN_MOVE_DUTY = 0.05

# How much of the speed a hard turn gives up. At full lock the car runs at
# (1 - 0.5) = half its straight-line duty.
STEER_DERATE = 0.5

# The centerline has to reach this far before the car may move at all. Shorter
# than this and the car would be committing to a corridor it can only see the
# first metre of -- and per `cone_perception/clustering.py`, a lidar cluster past
# ~3 m is one return or none, so a line this short usually means the cones ahead
# have dropped out rather than that the corridor ended.
MIN_REACH_M = 1.0

# Reach at which the car may run at full duty. Between MIN_REACH_M and this it
# scales linearly, so the car eases off as the corridor ahead thins out instead
# of running flat out into the last thing it can see.
FULL_REACH_M = 2.5

# The single-boundary fallback in `centerline.py` infers the line by offsetting
# one wall by half the corridor width. It is a real answer and worth driving on,
# but it is inferred from one side rather than measured between two, so it gets
# a fixed penalty rather than being treated as an equal.
FALLBACK_DERATE = 0.6

# Most this may change per control tick. At 10 Hz, 0.02 means roughly half a
# second from a standstill to full duty -- fast enough to be responsive, slow
# enough that one dropped frame is a dip rather than a lurch.
MAX_DUTY_STEP = 0.02


class DutyResult(object):
    """The commanded duty, and the reason when it is zero.

    `reason` is the field worth logging. A stopped car looks identical from the
    outside whatever stopped it, and the difference between "no line" and "line
    too short" is the difference between a perception bug and a corridor that
    genuinely ran out of cones.
    """

    __slots__ = ("duty", "reason", "reach_m")

    def __init__(self, duty, reason, reach_m):
        self.duty = duty
        self.reason = reason
        self.reach_m = reach_m

    @property
    def moving(self):
        return self.duty > 0.0

    def __repr__(self):
        tail = "" if self.moving else f" ({self.reason})"
        return f"DutyResult({self.duty:.3f}, reach {self.reach_m:.2f} m{tail})"


def reach_of(line, origin=(0.0, 0.0)):
    """How far down the corridor the centerline actually extends, in metres.

    Along-track distance to the far end, not arc length: what matters is how far
    ahead the car can see, and a line that wanders sideways has not bought any
    more of that.
    """
    if not line.points:
        return 0.0
    return max(0.0, line.points[-1][0] - origin[0])


def duty(pursuit, line, max_duty=DEFAULT_MAX_DUTY, origin=(0.0, 0.0)):
    """Centerline and steering command -> duty cycle. Never raises.

    Zero is a first-class answer here, not a failure: `pursuit is None` means
    `steering_angle` found nothing steerable, and holding the last good throttle
    through that is how a car drives itself into cones it stopped being able to
    see.
    """
    reach = reach_of(line, origin)

    if pursuit is None:
        return DutyResult(0.0, "no steerable target", reach)
    if len(line.points) < 2:
        return DutyResult(0.0, "centerline too short", reach)
    if reach < MIN_REACH_M:
        return DutyResult(0.0, f"corridor visible only {reach:.2f} m ahead", reach)

    scale = 1.0
    scale *= 1.0 - STEER_DERATE * abs(pursuit.normalised)
    if line.single_boundary_fallback:
        scale *= FALLBACK_DERATE
    if reach < FULL_REACH_M:
        scale *= (reach - MIN_REACH_M) / (FULL_REACH_M - MIN_REACH_M)

    commanded = max_duty * clamp(scale, 0.0, 1.0)

    # Snap through the dead band rather than sliding into it. See the module
    # docstring: a duty between zero and the floor is a stalled motor, not a
    # slow car.
    commanded = max(commanded, MIN_MOVE_DUTY)
    return DutyResult(min(commanded, max_duty), "", reach)


def ramp(previous, target, max_step=MAX_DUTY_STEP):
    """Rate-limit a duty change. Pure -- the caller holds `previous`.

    Only the rise is limited. A commanded stop takes effect immediately,
    because every path that produces one is either a safety condition or a
    perception dropout, and neither is improved by easing into it.
    """
    if target <= 0.0:
        return 0.0
    if target > previous + max_step:
        return previous + max_step
    return target
