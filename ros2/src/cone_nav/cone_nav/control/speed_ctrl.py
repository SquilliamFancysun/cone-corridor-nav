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
# Duty cycle -> metres per second. MEASURED, at last: the first powered run
# (2026-09-01, junction-live.jsonl) carried scan-matched odometry alongside
# the commanded duty, and the median over the armed ticks was 0.38 m/s at
# duty 0.05 -- 7.5, where the original guess said 12. One battery, one
# surface, one duty point, so it is a first calibration rather than a curve;
# refine it from any future run's odo_forward_m column. It matters less than
# it used to -- the state machine now prefers the measured step and falls back
# here only on a tick with no cones shared between scans -- but the sim's
# vehicle model still runs on it.
DUTY_TO_MPS = 7.5

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
#
# One caller is allowed past it, via `duty(min_reach_m=...)`: the goal run-in,
# where the line legitimately ends a few tens of centimetres ahead because the
# course does. See `cone_nav/guidance/goal_stop.py`.
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


def duty(pursuit, line, max_duty=DEFAULT_MAX_DUTY, origin=(0.0, 0.0),
         min_reach_m=MIN_REACH_M, min_points=2):
    """Centerline and steering command -> duty cycle. Never raises.

    Zero is a first-class answer here, not a failure: `pursuit is None` means
    `steering_angle` found nothing steerable, and holding the last good throttle
    through that is how a car drives itself into cones it stopped being able to
    see.

    `min_reach_m` and `min_points` exist for exactly one caller and default to
    leaving both refusals where they have always been. `cone_nav/guidance/goal_stop.py` passes
    zero over the last metre of a goal approach, where the reach rule would
    otherwise stop the car at 0.64 m from the trophy -- for a bookkeeping reason,
    and unrecoverably, since the scan does not change while the car stands still.
    The rule's own rationale does not describe that situation: there is no
    corridor left to commit to, only a lidar-ranged point being closed on. Every
    other caller gets today's behaviour, and passing these anywhere else means
    standing a safety rule down without that argument.

    `min_points` goes with it. At the very end of a course the corridor's last
    midpoint passes behind the car and the driven line is the goal anchor alone;
    two points is the right floor for a CORRIDOR, whose single midpoint says
    nothing trustworthy, and the wrong one for a measured object the car is
    closing on.
    """
    reach = reach_of(line, origin)

    if pursuit is None:
        return DutyResult(0.0, "no steerable target", reach)
    if len(line.points) < min_points:
        return DutyResult(0.0, "centerline too short", reach)
    if reach < min_reach_m:
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


# Reverse runs at the cogging floor and no faster. It is the direction with no
# lidar behind it (the chassis blocks 142 deg aft, measured), so the car is
# driving over ground it has to REMEMBER is clear rather than see is clear --
# and `reverse_ctrl`'s loop stiffens with speed on gains nothing has measured.
#
# Whether the car MOVES here is the open question, and it is a different one
# from whether the VESC accepts a negative duty. It does: DonkeyCar has driven
# this car backwards under manual control, which is the same
# `set_duty_cycle(throttle * VESC_MAX_SPEED_PERCENT)` call. But that ran at up
# to 0.10 and usually already rolling, and this is half of it from a dead stop.
# `docs/junction-bringup.md` stage 8b sweeps it; if the car needs more than the
# floor to break away in reverse, that number belongs here and the reverse
# gains want re-checking against it.
MAX_REVERSE_DUTY = MIN_MOVE_DUTY


def reverse_duty(max_reverse_duty=MAX_REVERSE_DUTY):
    """The duty to command while backing up. Negative.

    A separate function rather than a sign threaded through `duty()`, because
    every refusal `duty()` makes is about a corridor AHEAD -- reach, steerable
    target, point count -- and none of them describes a car reversing down a
    corridor it can no longer see. Reusing it would mean standing all three
    down, which is `goal_stop`'s trick and needs `goal_stop`'s argument; there
    is no such argument here.

    There is no derating either. Reverse already runs at the floor, and the
    floor is where derating stops meaning anything: below it the motor cogs
    rather than turns.
    """
    return -abs(max_reverse_duty)


def ramp(previous, target, max_step=MAX_DUTY_STEP):
    """Rate-limit a duty change. Pure -- the caller holds `previous`.

    Only the rise is limited, in whichever direction is being commanded. A
    commanded stop takes effect immediately, because every path that produces
    one is either a safety condition or a perception dropout, and neither is
    improved by easing into it.

    Zero is the pivot rather than a value to ramp through: a car asked to
    reverse while still rolling forward must reach zero at once and start
    again, not slide across through a duty that means nothing. That also keeps
    the old contract exactly -- `target <= 0` used to mean stop, and for every
    caller that never commands a negative it still does.
    """
    if target == 0.0:
        return 0.0
    if target < 0.0:
        if previous > 0.0:
            return 0.0
        return max(target, previous - max_step)
    if previous < 0.0:
        return 0.0
    if target > previous + max_step:
        return previous + max_step
    return target
