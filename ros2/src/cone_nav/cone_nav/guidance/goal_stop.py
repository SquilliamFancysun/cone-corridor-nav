"""Drive at the goal, then stop at it and stay stopped. Pure, no rclpy.

This is the whole of "goal navigation", and like `junction_exec` there is no
second control stack under it: `centerline`, `pure_pursuit` and `speed_ctrl` run
exactly as they do on a plain corridor. What changes is that one point is
threaded onto the driven line, and that the car is told to stop when it arrives.

The steering half is nearly free. The goal sits centered at the end of the
corridor, so the corridor was already pointing the car at it; the anchor exists
because a magenta cone forms no midpoints of its own (`midpoint_graph` pairs only
blue with yellow, and `test_orange_and_magenta_do_not_pair_with_anything` pins
that), so without one the line simply ends at the last cone row and the goal is
not on it.

## Why the reach floor has to move, and why that is safe

`speed_ctrl.MIN_REACH_M` stops the car when the driven line reaches less than a
metre ahead. With the goal anchored as the line's far point, reach is
`goal_x + 0.362` (the rear axle sits that far behind base_link), so that rule
halts the car at **goal_x ~= 0.64 m** -- before any sane stop range is reached,
and with `"corridor visible only 0.99 m ahead"` in the log rather than an
arrival. Worse, it is unrecoverable: `test_the_line_never_runs_short_through_the_
mouth` records the reason, which is that the scan does not change while the car
stands still, so a car stopped by a perception rule cannot un-stop itself.

So the caller relaxes that floor while this state machine is in `RUN_IN`, and
only then. The rule's own rationale -- do not commit to a corridor you can see
one metre of -- does not describe this situation: the car is closing on a
confirmed, lidar-ranged point between 1.0 m and the stop range, and there is
nothing left to commit to. `pure_pursuit.MIN_TARGET_M` still guards the steering
geometry, and with the axle 0.362 m behind the lidar the target never crosses
behind the pivot.

## Why the identity is debounced and the trigger is not

`CONFIRM_TICKS` sightings are required before the machine will act on a goal at
all, and then the stop fires on the first tick inside `stop_range_m` with no
further confirmation. That split is deliberate. Debouncing the identity is cheap
-- the trophy is dead ahead, centered and in frame for the whole approach, so
sightings are plentiful here in a way they emphatically are not at a junction
(see `topo_state`, which commits on a single sighting because some layouts yield
exactly one). Debouncing the TRIGGER would be expensive and pointless: at the
0.05 duty floor the car covers 3.8 cm per tick, so three ticks of confirmation is
11 cm of overshoot bought for nothing.

## Carrying the goal through a dropout

If magenta flickers off during the run-in the anchor vanishes, the line collapses
to whatever the corridor still offers, and the car stops on `"no steerable
target"` at the one place it cannot restart. So the goal is carried forward with
the car's own motion, by the same arithmetic `topo_state` uses for the divider,
and fed scan-matched odometry on the car rather than a duty estimate.

`topo_state`'s docstring refuses this for the GATE anchor, and is right to: "a
point 1.2 m stale is a point behind the car." It is admissible here only because
it is bounded to `max_blind_ticks` -- about a second, a third of a metre at
run-in speed -- after which the latch drops and the car stops honestly. A live
sighting always beats the carried estimate, and `blind_ticks` is logged so a run
that finished on dead reckoning is visible as one rather than passing for a clean
arrival.
"""

import math

SEEKING = "seeking"
RUN_IN = "run_in"
STOPPED = "stopped"

# How close the car's nose gets to the trophy, in metres, measured in base_link
# -- which IS the nose: `docs/hardware-baseline.md` puts the lidar at the front
# edge of the chassis.
#
# The floor on this is not braking. Measured on the car, deceleration at these
# speeds is near-instant and coast is negligible; what bounds it is
# `clustering.MIN_CONE_RANGE_M = 0.20`, below which a return is treated as the
# chassis arc leaking and the trophy stops being a cluster at all. 0.30 m leaves
# 10 cm of margin, so the tick that fires the stop is looking at a MEASURED
# cluster: at the 0.05 duty floor ticks land 3.8 cm apart, so the firing tick
# sees the goal somewhere in 0.30-0.34 m and the one before it at ~0.34 m.
STOP_RANGE_M = 0.30

# Where the machine takes over the speed law, in metres. Above this the ordinary
# corridor rules run untouched. Chosen to sit clear of the 0.64 m at which the
# reach floor would otherwise bite, so the relaxation covers the whole of the
# stretch that needs it and nothing more -- about 0.7 m of travel.
RUN_IN_M = 1.0

# Consecutive sightings before the goal is acted on. See the module docstring on
# why this is consecutive and why the trigger gets no such treatment.
CONFIRM_TICKS = 3

# How long the goal may be carried without the camera re-confirming it, in ticks.
# ~1 s at the measured 9.9 Hz, about 0.38 m at the duty floor. Past this the
# latch drops rather than steering the car at a remembered point.
GOAL_BLIND_TICKS = 10


class GoalLatch(object):
    """Where the car is in its approach to the goal, and whether to stop.

    Owns the arrival decision so that the invariant that matters -- the car stops
    once, stays stopped, and only a deliberate release restarts it -- lives in
    one place and is testable without a car.
    """

    __slots__ = ("stop_range_m", "run_in_m", "confirm_ticks", "max_blind_ticks",
                 "state", "goal_xy", "blind_ticks", "note", "_sightings",
                 "_live")

    def __init__(self, stop_range_m=STOP_RANGE_M, run_in_m=RUN_IN_M,
                 confirm_ticks=CONFIRM_TICKS,
                 max_blind_ticks=GOAL_BLIND_TICKS):
        self.stop_range_m = stop_range_m
        self.run_in_m = run_in_m
        self.confirm_ticks = confirm_ticks
        self.max_blind_ticks = max_blind_ticks
        self.state = SEEKING
        self.goal_xy = None
        self.blind_ticks = 0
        self.note = ""
        self._sightings = 0
        self._live = False

    # --- what the pipeline reads ---------------------------------------

    @property
    def stopped(self):
        """Is the car being held at the goal? The caller zeroes duty on this."""
        return self.state == STOPPED

    @property
    def run_in(self):
        """May the caller relax `speed_ctrl`'s reach floor this tick?

        True only in the final stretch. See the module docstring -- this is a
        safety rule being stood down, and it is stood down over 0.7 m of travel
        toward a point the lidar is measuring, not for the whole approach.
        """
        return self.state == RUN_IN

    @property
    def confirmed(self):
        """Has the goal been seen enough times to be acted on?"""
        return (self._sightings >= self.confirm_ticks
                or self.state in (RUN_IN, STOPPED))

    @property
    def anchor_ok(self):
        """May the goal be threaded into the driven line this tick?

        Unlike `topo_state.anchor_ok` this does not demand a live sighting, and
        the module docstring gives the reason: the alternative is the line
        collapsing in the last metre, where the car cannot recover. The blind
        budget is what keeps it honest.
        """
        return (self.confirmed and self.goal_xy is not None
                and self.blind_ticks <= self.max_blind_ticks)

    @property
    def range_m(self):
        """Range to the goal -- measured if it was seen this tick, else carried."""
        if self.goal_xy is None:
            return None
        return math.hypot(self.goal_xy[0], self.goal_xy[1])

    # --- the transition ------------------------------------------------

    def _carry_forward(self, travel_m, yaw_delta_rad):
        """Move the carried goal into THIS tick's base_link.

        The car went `travel_m` along its own x and turned `yaw_delta_rad`, so
        anything fixed to the ground moved the opposite way in its frame. The
        same expression as `topo_state._carry_forward`.
        """
        if self.goal_xy is None:
            return
        x, y = self.goal_xy[0] - travel_m, self.goal_xy[1]
        cos_t, sin_t = math.cos(yaw_delta_rad), math.sin(yaw_delta_rad)
        self.goal_xy = (x * cos_t + y * sin_t, -x * sin_t + y * cos_t)

    def update(self, goal, armed, travel_m=0.0, yaw_delta_rad=0.0):
        """One tick. `goal` is `goal_detect`'s answer, `armed` the caller's.

        `armed` is the guard this module cannot apply itself: the goal lies at
        the end of the route by construction, so `drive_junction.py` passes
        `RouteCursor.exhausted` and a magenta glimpsed at the first junction
        never reaches the state machine. It gates ENTRY only -- once stopped, the
        car stays stopped whatever else changes, and `release()` is the only way
        out.
        """
        self.note = ""
        self._live = goal is not None

        if self.state == STOPPED:
            # Sticky by design. Nothing observed after arrival may restart the
            # car: not the goal dropping out, not the route, not a fresh
            # sighting somewhere else.
            return self.state

        if not armed:
            self._reset("")
            return self.state

        self._carry_forward(travel_m, yaw_delta_rad)
        if goal is not None:
            # A live sighting always beats a carried estimate.
            self.goal_xy = (goal.x, goal.y)
            self.blind_ticks = 0
            self._sightings += 1
        else:
            self.blind_ticks += 1
            # Consecutive, and only while nothing has been committed to yet.
            # Once the machine is in RUN_IN the blind budget is what governs.
            if self.state == SEEKING:
                self._sightings = 0

        if self.state == RUN_IN and self.blind_ticks > self.max_blind_ticks:
            self._reset("goal lost during run-in")
            return self.state

        if not self.confirmed or self.goal_xy is None:
            return self.state

        range_m = self.range_m
        if range_m <= self.stop_range_m:
            self.state = STOPPED
            self.note = ("goal reached" if self._live
                         else "goal reached (carried)")
        elif range_m <= self.run_in_m:
            self.state = RUN_IN
        return self.state

    def release(self):
        """Clear a stop so the car may drive again.

        Called on a deliberate operator action -- `drive_junction.py` releases on
        a deadman rising edge -- so a trophy can be reset and the run repeated
        without restarting the tool and reopening the camera and the lidar.
        """
        self._reset("released")

    def _reset(self, note):
        self.state = SEEKING
        self.goal_xy = None
        self.blind_ticks = 0
        self.note = note
        self._sightings = 0

    def __repr__(self):
        where = ("%.2f m" % self.range_m) if self.goal_xy is not None else "-"
        return f"GoalLatch({self.state}, {where})"
