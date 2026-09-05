"""Backing the car out of a dead end, to where it can see the junction again.

Pure, no rclpy. The manoeuvre that replaces the operator in
`docs/junction-bringup.md` stage 7b, which today reads: release X, *carry the
car* back to the junction it came through, facing the way it originally
approached, press X. This module does the carrying.

## Why it terminates on a gate sighting and not on a distance

The obvious design is odometric: remember how far the car drove into the
branch, back out that far, stop. It is wrong in the way that matters, because
the distance the car wants is not the one it measured. `topo_state` fires
`passed` somewhere past the gate line and the pose snapshot is taken there, so
the recorded edge starts from a point that is neither the junction nor the place
the car has to end up. Backing out an edge length lands the car in the mouth.

What the car actually needs is a pose it can SEE the whole junction from,
because that is by construction a pose the `follow -> approach -> traverse`
machine can drive from. And it can recognise one rather than dead-reckon to it:
`gate_detect.survey` is a pure function of one revolution, so a car parked in
the right place reports the gate on every tick. So the manoeuvre reverses until
it recovers a whole triple, and stops on the first one that is not the mouth.

Recovering a triple is a NARROWER condition than it sounds, and that is what
makes it a good stopping rule. `docs/junction-bringup.md` stage 3 measures the
band it happens in as about half a metre deep, with both edges set by the outer
reds: too far and they fall outside `GATE_ARM_RANGE_M`, too close and they leave
the camera frame and come back unlabeled. The car cannot see three reds from
anywhere except a workable re-approach pose, so "I can see the junction" and "I
am somewhere I can drive from" are the same measurement. It is self-calibrating
for free, which a relaid track needs.

## Why not stop at the range it committed from

That was the first design, and it does not work, for a reason worth keeping:
**the commit range is not reachable on the way out.** Driving in, the car
commits at the FAR edge of that band, the moment a triple first appears. Backing
out, it enters the band at the near edge and climbs -- and the reds leave arm
range again before it gets back to where it committed. Measured on
`junction-left-blocked`: a clean ten-tick window from 2.24 m to 2.56 m against a
commit range of about 2.7, so the car reversed through a perfectly good
re-approach pose without stopping, and off the end of the corridor.

So `commit_range_m` sizes the distance bound, where it is genuinely useful, and
the arrival test is a floor that rejects the mouth and nothing more.

## The distance bound is a bound, never the target

It still needs one, because the terminating condition is a PERCEPTION event and
the car is reversing into an arc it cannot see -- the chassis fills the rear
142 deg (`docs/hardware-baseline.md`). A manoeuvre that waits forever for a gate
that never comes is a car reversing off the end of the track. So the budget
stops it and hands back to the operator, and `abandoned` is a normal outcome
rather than a failure: it is exactly today's behaviour, which works.

## Why the mouth does not trigger a false arrival

Reversing out of the branch, the car passes THROUGH the junction, and for
several ticks it is sitting in the mouth looking at all three reds from a metre
away. Gate liveness alone would arrive there -- inside the junction, pointing
down the branch it just left. The range floor is what rejects it, and is the
reason the test is `range >= commit_range_m` rather than `junction is not None`.
"""

import math

from cone_nav.control import reverse_ctrl, speed_ctrl

IDLE = "idle"
BACKING = "backing"
ARRIVED = "arrived"
ABANDONED = "abandoned"

# Sightings of the gate at range, WITHIN a window, before the manoeuvre stops.
# A count inside the window, not a run of consecutive ticks -- the distinction
# `dead_end.py` makes at length, for the same reason, having got it wrong first
# time round too.
#
# Measured: sweeping the car across the band a triple is recoverable from, on
# the sim's own junction, a whole triple comes back on roughly one tick in
# three -- and which ones depends on a few centimetres of position and a few
# degrees of heading. The band is not narrow so much as SPECKLED. Requiring
# three in a row asks for a run that does not occur, and the car reverses
# through a perfectly good re-approach pose and off the end of its bound.
#
# Two rather than one because a single frame is cheap to get wrong, and twelve
# because that is about how long the car spends in the band at the cogging
# floor: 0.4 m of band at 0.375 m/s is 11 ticks at 10 Hz.
ARRIVE_CONFIRM_TICKS = 2
ARRIVE_WINDOW_TICKS = 12

# How long the manoeuvre may steer on a remembered error. `corridor_error`
# returns None on a line too short to have a direction, and the module it comes
# from is explicit that zero error must NOT be substituted -- zero commands a
# straight reverse, which is the one thing a car with an unknown heading must
# not do. So the last good command is held briefly, and then the car stops.
MAX_BLIND_TICKS = 5

# The largest one-tick change in either error that can be believed.
#
# Crossing back through the junction, the centerline ahead of the car flips
# between two readings of the scene -- the branch it is leaving and the parent
# corridor it is entering -- and the errors invert wholesale rather than move.
# Measured on junction-left-blocked: cross-track +1.96 m to -2.03 m and heading
# +40 deg to -71 deg between consecutive ticks, three ticks of it, with the law
# commanding full lock in alternating directions throughout.
#
# The car cannot travel that far in 0.04 m, so neither is a measurement of
# anything. Reversing at the cogging floor it turns a couple of degrees a tick
# and moves four centimetres, so both thresholds sit an order of magnitude
# above anything real and still catch the flip.
#
# A rejected tick holds the last good command, which is what the servo would
# have done anyway; obeying the flip is what puts the car into a cone.
MAX_HEADING_JUMP_RAD = math.radians(30.0)
MAX_CROSS_TRACK_JUMP_M = 0.5

# How many consecutive implausible readings before the manoeuvre believes them.
#
# This is the other half of the rule and it is not optional. Backing out of a
# branch, the car really does change which corridor it is regulating against --
# it leaves the branch and enters the parent -- and the true reference on the
# far side of that legitimately differs by more than the gate allows. A gate
# with no way back locks the manoeuvre onto a stale reading of a corridor the
# car has left, and it never recovers, because the disagreement it is
# rejecting is the very thing that would clear it. Measured: the car steered
# neatly, on the wrong corridor, all the way to the end of its bound.
#
# So a jump seen ONCE is noise and a jump that persists is the world. The flip
# through a junction mouth lasts three ticks; a new corridor lasts.
REFERENCE_RESEAT_TICKS = 4

# Slack on the distance bound, over and above the edge and the commit range.
BUDGET_MARGIN_M = 1.0

# The gap between the gate line and where `topo_state` calls a gate passed --
# its CLEAR_PAST_GATE_M, restated rather than imported so that guidance does not
# take a dependency on topology for one float. The recorded edge starts at that
# snapshot, so the car has to reverse this much further than the edge before it
# is even back at the junction. Measured in sim without it, the manoeuvre used
# 87-90% of its bound, which is no margin at all for a track laid to different
# dimensions.
CLEARED_PAST_GATE_M = 0.5

# The only thing the arrival range has to do: reject a triple seen from inside
# the mouth. Backing out, the car passes THROUGH the junction and spends a few
# ticks looking at all three reds from under a metre away, pointing down the
# branch it just left.
#
# It sits deliberately well BELOW the band a triple can actually be recovered
# from -- 2.12 m at the near edge on the v2 junction, 2.24 m measured in sim on
# a narrower one. It is not a target and must never bind on a real sighting:
# the camera's own frame sets where those begin. See the module docstring.
MOUTH_CLEAR_RANGE_M = 1.2

# Safety net, in ticks, mirroring `topo_state.MAX_TRAVERSE_TICKS`. 20 s at
# 10 Hz, against a manoeuvre that should take a few seconds.
MAX_BACKOUT_TICKS = 200


class BackoutManoeuvre(object):
    """Reverse to the junction, on the errors the car can still see ahead.

    Shaped like `goal_stop.GoalLatch` and `dead_end.DeadEndLatch`: the caller
    drives it a tick at a time and reads a command and a reason off it. It owns
    no hardware and decides nothing about the search -- `ExplorePolicy` has
    already chosen the branch by the time this begins, and would have chosen the
    same one if a person did the moving.
    """

    __slots__ = ("state", "reason", "turn", "budget_m", "commit_range_m",
                 "travelled_m", "ticks", "confirm", "blind_ticks",
                 "steer_normalised", "duty", "gate_range_m",
                 "heading_err_rad", "cross_track_m", "_has_reference",
                 "rejected_ticks",
                 "max_reverse_duty", "arrive_confirm_ticks", "max_blind_ticks",
                 "max_ticks", "arrive_window_ticks", "_window",
                 "k_heading", "k_cross")

    def __init__(self, max_reverse_duty=speed_ctrl.MAX_REVERSE_DUTY,
                 arrive_confirm_ticks=ARRIVE_CONFIRM_TICKS,
                 arrive_window_ticks=ARRIVE_WINDOW_TICKS,
                 max_blind_ticks=MAX_BLIND_TICKS,
                 max_ticks=MAX_BACKOUT_TICKS,
                 k_heading=reverse_ctrl.K_HEADING,
                 k_cross=reverse_ctrl.K_CROSS):
        # Carried rather than left to `reverse_ctrl.steer`'s defaults, which
        # are bound at def time and so cannot be swept by rebinding the module
        # attribute -- a sweep that does that measures one gain pair twelve
        # times and reports it as twelve results.
        self.k_heading = k_heading
        self.k_cross = k_cross
        self.max_reverse_duty = max_reverse_duty
        self.arrive_confirm_ticks = arrive_confirm_ticks
        self.arrive_window_ticks = arrive_window_ticks
        self.max_blind_ticks = max_blind_ticks
        self.max_ticks = max_ticks
        self.state = IDLE
        self.reason = ""
        self.turn = None
        self.budget_m = 0.0
        self.commit_range_m = 0.0
        self._clear_run()

    def _clear_run(self):
        self.travelled_m = 0.0
        self.ticks = 0
        self.confirm = 0
        self._window = []
        self.blind_ticks = 0
        self.steer_normalised = 0.0
        self.duty = 0.0
        self.gate_range_m = 0.0
        self.heading_err_rad = 0.0
        self.cross_track_m = 0.0
        self._has_reference = False
        self.rejected_ticks = 0

    # --- what the caller reads -----------------------------------------

    @property
    def active(self):
        return self.state == BACKING

    @property
    def arrived(self):
        return self.state == ARRIVED

    @property
    def abandoned(self):
        return self.state == ABANDONED

    @property
    def arrive_range_m(self):
        """The gate range below which a sighting is the mouth, not an arrival.

        A floor, not a target. See MOUTH_CLEAR_RANGE_M.
        """
        return MOUTH_CLEAR_RANGE_M

    @property
    def bound_m(self):
        """The distance bound. A bound, not a target; see the module docstring.

        The edge the car drove in, plus the commit range -- because the edge is
        measured from the `passed` snapshot rather than from the junction, so it
        is short of the whole journey by roughly that much -- plus slack.

        This is where `commit_range_m` earns its keep. It is a poor arrival
        test (see the module docstring) and a good scale for how much further
        than the recorded edge the car has to travel.
        """
        return (self.budget_m + max(self.commit_range_m, MOUTH_CLEAR_RANGE_M)
                + CLEARED_PAST_GATE_M + BUDGET_MARGIN_M)

    # --- the two events ------------------------------------------------

    def begin(self, turn, budget_m=0.0, commit_range_m=0.0):
        """Start backing out, to take `turn` at the junction behind the car.

        `turn` is what `ExplorePolicy.dead_end()` returned. None means the
        search is spent and there is nothing to back out TO, so the manoeuvre
        refuses rather than reversing to no purpose.
        """
        self._clear_run()
        if turn is None:
            self.state = IDLE
            self.reason = "nothing left to explore; not backing out"
            return self.state
        self.turn = turn
        self.budget_m = max(0.0, budget_m)
        self.commit_range_m = max(0.0, commit_range_m)
        self.state = BACKING
        self.reason = f"backing out to take {turn}"
        return self.state

    def update(self, line, junction, travel_m=0.0, armed=True):
        """One tick. `line` is the forward centerline, `junction` the survey's.

        Both are what the car sees AHEAD, which is the whole reason this can run
        while the path itself is behind the car and invisible.
        """
        if self.state != BACKING:
            return self.state

        self.ticks += 1
        self.travelled_m += abs(travel_m)

        # The deadman keeps one meaning. A release mid-reverse is an operator
        # saying stop, not pause, so the manoeuvre gives up and the run falls
        # back to the carry it would have used anyway.
        if not armed:
            return self._abandon("released mid-reverse")

        # A refused tick is evidence-against rather than a reset. See
        # ARRIVE_CONFIRM_TICKS: the band is speckled, and wiping the count on
        # every bad tick makes it unreachable while the evidence is plainly
        # there.
        seen = self._arriving(junction)
        self._window.append(seen)
        del self._window[:-self.arrive_window_ticks]
        self.confirm = sum(1 for hit in self._window if hit)
        # `seen` as well as the count, so the range this stops on and reports
        # is one the car measured THIS tick rather than one carried from
        # wherever it was a second ago.
        if seen and self.confirm >= self.arrive_confirm_ticks:
            self.duty = 0.0
            self.steer_normalised = 0.0
            self.state = ARRIVED
            self.reason = (f"whole junction in view at "
                           f"{self.gate_range_m:.2f} m, "
                           f"{self.confirm} sightings in "
                           f"{len(self._window)} ticks")
            return self.state

        if self.travelled_m >= self.bound_m:
            return self._abandon(
                f"backed out {self.travelled_m:.2f} m without seeing the "
                f"junction (bound {self.bound_m:.2f} m)")
        if self.ticks >= self.max_ticks:
            return self._abandon("backout timed out")

        if not self._steer(line):
            return self._abandon(
                f"no corridor to steer on for {self.blind_ticks} ticks")

        self.duty = speed_ctrl.reverse_duty(self.max_reverse_duty)
        self.reason = (f"backing out to take {self.turn}, "
                       f"{self.travelled_m:.2f}/{self.bound_m:.2f} m")
        return self.state

    def release(self):
        """Back to idle, once the caller has acted on the outcome."""
        self.state = IDLE
        self.turn = None
        self.reason = ""
        self._clear_run()

    # --- internals -----------------------------------------------------

    def _arriving(self, junction):
        """Is this tick a sighting of the junction from far enough back?"""
        if junction is None or self.turn is None:
            self.gate_range_m = 0.0
            return False
        try:
            self.gate_range_m = junction.range_for(self.turn)
        except ValueError:
            self.gate_range_m = 0.0
            return False
        return self.gate_range_m >= self.arrive_range_m

    def _steer(self, line):
        """Set the steer command. False when the car has been blind too long.

        Two different failures, and only one of them ends the manoeuvre. No
        corridor at all is a car that cannot see, and it stops. A corridor that
        disagrees with the last one is a car whose reference has jumped, and it
        holds -- then believes the new one if it persists.
        """
        error = reverse_ctrl.corridor_error(line)
        if error is None:
            # Hold the last command rather than straightening; see
            # MAX_BLIND_TICKS.
            self.blind_ticks += 1
            return self.blind_ticks <= self.max_blind_ticks
        if not self._believable(error):
            self.rejected_ticks += 1
            if self.rejected_ticks <= REFERENCE_RESEAT_TICKS:
                return True
            # It has persisted. The corridor changed; the reading did not lie.
        self.blind_ticks = 0
        self.rejected_ticks = 0
        self.heading_err_rad, self.cross_track_m = error
        self._has_reference = True
        command = reverse_ctrl.steer(self.heading_err_rad, self.cross_track_m,
                                     reference="backout",
                                     k_heading=self.k_heading,
                                     k_cross=self.k_cross)
        self.steer_normalised = command.normalised
        return True

    def _believable(self, error):
        """Could the car have moved this far since the last accepted tick?

        See MAX_HEADING_JUMP_RAD. Only ever asked of a tick that already HAS a
        reference, so the first one is believed by definition -- there is
        nothing to have jumped from.
        """
        if not self._has_reference:
            return True
        heading, cross = error
        return (abs(heading - self.heading_err_rad) <= MAX_HEADING_JUMP_RAD
                and abs(cross - self.cross_track_m) <= MAX_CROSS_TRACK_JUMP_M)

    def _abandon(self, reason):
        self.duty = 0.0
        self.steer_normalised = 0.0
        self.state = ABANDONED
        self.reason = reason
        return self.state

    def __repr__(self):
        if self.state != BACKING:
            return f"BackoutManoeuvre({self.state}, {self.reason or '-'})"
        return (f"BackoutManoeuvre(backing to {self.turn}, "
                f"{self.travelled_m:.2f}/{self.bound_m:.2f} m, "
                f"confirm {self.confirm}/{self.arrive_confirm_ticks})")
