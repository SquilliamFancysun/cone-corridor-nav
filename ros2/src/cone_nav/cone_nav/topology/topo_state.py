"""Discrete state machine: which edge / which node the car is on. Pure, no rclpy.

Three states. `FOLLOW` is a plain corridor and is what `drive_corridor.py`
already does; `APPROACH` is a junction in view but not yet committed to;
`TRAVERSE` is the manoeuvre itself.

## The detection window, and why commitment is not a distance

A junction is visible for a surprisingly short stretch, and the two limits that
bound it move in OPPOSITE directions as the gate widens. With gaps of `g` the
outer reds sit `g` off the axis, so measuring to the junction line they:

  - enter lidar range at  sqrt(3.0^2 - g^2)   -- wider gate, nearer
  - leave the camera frame at  g / tan(32.5)  -- wider gate, further

    gap    span   window   ticks at 1.2 m/s
    1.30   2.60   0.66 m   5.5
    1.35   2.70   0.56 m   4.7
    1.50   3.00   0.24 m   2.0

and the span must clear `MAX_PAIR_EDGE_M` (2.5 m) or the triangulation puts a
phantom gate midpoint on the centre cone, which rules out anything at or below
1.25 m gaps. `data/layouts/junction_v2.md` lays the track at 1.35 m for that
reason; 1.50 m looks tidier on paper and leaves two ticks to decide in.

A window that thin is why commitment is NOT a distance threshold here. There is
no room between "detectable" and "committed" for a metre-based margin, and the
route has already decided which way to go -- there is nothing to defer. So the
machine commits on a COUNT of detections instead.

That count is one, and it is worth being plain about why, because a debounce is
the obvious thing to want here. Measured against the sim, across the whole
approach the car gets:

    20 deg divergence, 1.35 m gaps    sightings on ticks 1,2,3,4,5
    25 deg divergence, 1.35 m gaps    sightings on ticks 0 and 4
    20 deg divergence, 1.45 m gaps    a sighting on tick 1, and no other

An outer red merges into a neighbouring branch cone in the lidar whenever the
two fall inside `clustering.GAP_DEG` of each other in bearing, and requiring
three good recoveries at once turns three marginal detections into one very
marginal one. On the layouts above, demanding two sightings lost the junction
outright: the car sailed past, followed whichever branch gave the longer chain,
and drove into the dead end.

So sightings are the scarce resource, not confidence, and none can be spent on
confirmation. What guards against a spurious commit is `gate_detect.detect`
itself -- exactly three reds, all inside 3 m, both gaps inside the corridor-pair
window -- which a misread orange at a dead-end wall cannot satisfy alone. The
window and count are kept as constants because they are the first thing to raise
if a layout ever makes sightings plentiful.

## The blind period, and why the latch exists

Below `g / tan(32.5)` the outer reds come back UNLABELED, out of
`boundary_split`'s red bucket and invisible to `gate_detect`. The junction
therefore DISAPPEARS while the car is still short of the mouth, not once it is
through. A machine that read that disappearance as "passed" would drop the
branch filter and consume a route entry at the worst possible moment. So the car
latches what it saw and drives the rest of the mouth blind.

## What the latch may and may not be used for

Nothing in this repo tracks odometry, so a latched cone position is in a frame
that moves out from under it -- a gate latched at 1.5 m is still recorded at
1.5 m a second later, when it is really at 0.3 m. That makes the latch usable
for some things and not others:

  - The branch half-plane, yes -- but only if it is dead-reckoned. A junction
    latched at 2.6 m is still recorded at 2.6 m three metres later, by which
    point the cut it defines is behind the car and pointing the wrong way. Left
    frozen it made the divergence sweep in `sim/drive_sim.py` non-monotonic:
    20 deg passed while 15, 25 and 30 deg each failed in one direction, which
    is the signature of a run passing by luck rather than by design. So the
    divider and the axis are carried forward each tick with the car's own
    motion. This is dead reckoning, which nothing else in this repo does; it is
    tolerable here only because it runs for a few seconds and feeds a half-plane
    with 0.2-0.4 m of slack, never a position the car steers at.
  - The gate ANCHOR, yes -- but only once there was something honest to carry
    it with, and only while it is still in front of the car. This entry used to
    read "no", on the grounds that "a point 1.2 m stale is a point behind the
    car". That was correct about a FROZEN anchor and it was the only option:
    the sentence above it said "nothing in this repo tracks odometry", and when
    that is true a latched point is simply wrong by however far the car has
    driven.

    `cone_perception/odometry.py` made it false. The anchor is now carried by
    the same measured step the divider is, and `goal_stop` has been carrying
    the trophy the same way since it was written -- bounded, on measured
    motion, with a live sighting always winning.

    Refusing it was costing the whole manoeuvre. Measured on the car
    2026-09-02 (`data/trials/explore-4.jsonl`): across a 186-tick traverse the
    triple was recovered on **5 ticks**, so for 181 of them there was no gate
    to steer at and the car drove the corridor line instead -- `target_y`
    pinned at +0.03 m while taking a branch that diverges 20 degrees. Straight
    ahead at a junction mouth is the centre red cone, and that is what it hit.

What ends the carry is geometry rather than a timer: the anchor is dropped once
it is no longer ahead of the car, because by then the car is inside the mouth
and the exit corridor is the better guide anyway. `MAX_ANCHOR_BLIND_TICKS` is a
net under that, for the case where the odometry itself has stopped reporting.

## Leaving TRAVERSE

Not "the reds are gone" -- they went while the car was still 2 m short of the
mouth. Nor "the corridor looks healthy": it looks healthy for the whole
approach too, because the corridor the car is still IN is a perfectly good
corridor. Measured in `sim/drive_sim.py`, that predicate alone ended the
manoeuvre at tick 9 of 65, with the junction still 2.2 m ahead -- the branch
filter switched off before the fork, the car followed whichever branch happened
to give the longer chain, and on `--track junction-right` it drove into the
dead end and stalled.

So the floor is a travelled DISTANCE, and the corridor check only confirms it:

    travelled since commit  >=  gate range at commit + CLEAR_PAST_GATE_M
    and no junction in view
    and the corridor is two-sided again
    held for PASSED_CONFIRM_TICKS

`travel_m` is supplied by the caller, which is honest about what it is: in the
sim and on the car it is the commanded duty times `DUTY_TO_MPS`, and that
constant is a guess. It does not need to be accurate -- it needs to be monotone
and roughly right, so that a detector dropout cannot end the manoeuvre early and
a stalled car cannot end it at all. `myconfig_capture.py` sets
`VESC_HAS_SENSOR = True` and nothing in this repo reads the encoder yet; that is
where this number should come from once something does.

`COMMIT_CONFIRM_TICKS`, `PASSED_CONFIRM_TICKS` and `MIN_REACQUIRE_POINTS` are
first guesses to be settled in `sim/drive_sim.py`, not measured values.
"""

import math

FOLLOW = "follow"
APPROACH = "approach"
TRAVERSE = "traverse"

# Sightings needed to commit, and how many ticks they may be spread over. One:
# see the module docstring -- on some layouts the whole approach yields a single
# recovered triple, and a car that waits for a second one drives past the fork.
COMMIT_CONFIRM_TICKS = 1
COMMIT_WINDOW_TICKS = 4

# Consecutive ticks of "clean corridor, no junction" before the manoeuvre is
# declared over. ~0.3 s at the measured 9.9 Hz. A one-tick detector dropout must
# never consume a route entry.
PASSED_CONFIRM_TICKS = 3

# A reacquired corridor has to be more than the two points a fallback can
# manufacture from one wall.
MIN_REACQUIRE_POINTS = 3

# How far past the latched gate the car must have travelled before the mouth
# counts as behind it. Enough to carry the reds past the rear axle at 0.36 m,
# with room for the travel estimate being a duty-cycle guess.
CLEAR_PAST_GATE_M = 0.5

# A carried anchor is dropped once it is no longer this far ahead of base_link.
# Past that the car is in the mouth, the point it would steer at is beside or
# behind it, and the exit corridor is the better guide -- which is the same
# reasoning the old "no carried anchor" rule rested on, kept and made
# geometric. `pure_pursuit` would refuse a target behind the axle anyway; this
# stops the line being threaded with a point that is merely useless first.
MIN_ANCHOR_X_M = 0.0

# A net under the geometric test above, not the thing that normally ends the
# carry. It only bites if the odometry stops reporting, in which case the
# carried anchor stops being moved and the x test can no longer retire it.
# 6 s at ~10 Hz, against a mouth that takes 5-7 s at the duty floor.
MAX_ANCHOR_BLIND_TICKS = 60

# If the corridor never comes back, stop trusting a latch this old and hand
# control back to plain corridor following without consuming a route entry.
# 20 s at ~10 Hz -- a loose safety net, not a bound: the travel floor above is
# what actually decides when the manoeuvre is over, and a derated crawl through
# a mouth legitimately takes 40-50 ticks.
MAX_TRAVERSE_TICKS = 200


def corridor_reacquired(line, min_points=MIN_REACQUIRE_POINTS):
    """Is this line evidence of being back in a normal two-sided corridor?

    A single-boundary fallback is explicitly not: it is what the car produces
    while it is confused about one wall, which is the state the mouth of a
    junction puts it in.
    """
    if line is None:
        return False
    return (len(line.points) >= min_points
            and not line.single_boundary_fallback)


class TopoState(object):
    """Where the car is in the route, and what the pipeline should do about it.

    Owns the `RouteCursor` rather than letting the caller advance it, so the
    invariant that matters -- exactly one advance per confirmed gate pass --
    lives in one place and is testable on its own.
    """

    __slots__ = ("cursor", "state", "latched", "latched_turn", "live",
                 "max_traverse_ticks",
                 "_confirm", "_seen", "_traverse_ticks", "blind_ticks", "note",
                 "travelled_m", "commit_range_m", "divider_xy", "axis_rad",
                 "anchor_xy", "_sightings")

    def __init__(self, cursor, max_traverse_ticks=MAX_TRAVERSE_TICKS):
        # The traverse safety net is a TIME bound sized for a driving car:
        # 20 s crosses any mouth under power several times over. A hand-pushed
        # dry run with measured odometry covers the same distance floor in
        # minutes, honestly -- measured 2026-09-01: the floor cleared at tick
        # 188 of 200 at walking pace, leaving twelve ticks to confirm the exit.
        # So the bound is a parameter: the caller that knows the car is being
        # pushed may size it for a push. It remains a bound, never the thing
        # that ends a healthy traverse.
        self.max_traverse_ticks = max_traverse_ticks
        self.cursor = cursor
        self.state = FOLLOW
        self.latched = None
        self.latched_turn = None
        self.live = None
        self.blind_ticks = 0
        self.note = ""
        self.travelled_m = 0.0
        self.commit_range_m = 0.0
        self.divider_xy = None
        self.axis_rad = 0.0
        self.anchor_xy = None
        self._sightings = []
        self._confirm = 0
        self._seen = 0
        self._traverse_ticks = 0

    # --- what the pipeline reads ---------------------------------------

    @property
    def engaged(self):
        """Should the branch filter run this tick?"""
        return self.state in (APPROACH, TRAVERSE)

    @property
    def past_gate(self):
        """Has the car travelled beyond the gate line it committed to?

        `engaged` is what holds the dead-end detector down, because a junction
        mouth legitimately looks like a short one-sided corridor and calling it
        a wall would reverse the car out of a junction it was driving
        correctly. But that excuse expires at the gate line. Past it the car is
        inside the branch it chose, and a wall ahead is a wall -- while
        `engaged` stays true for the rest of the traverse, which is the whole
        clearance distance plus `CLEAR_PAST_GATE_M`.

        On a stub shorter than `commit_range_m + CLEAR_PAST_GATE_M` that gap
        swallows the entire branch: the car meets the wall still inside the
        manoeuvre, with the detector silenced. Measured 2026-09-02, J1
        committed at 2.71 m and so needed 3.21 m of travel against a 2.5 m
        stub.
        """
        return (self.state == TRAVERSE and self.commit_range_m > 0.0
                and self.travelled_m >= self.commit_range_m)

    @property
    def junction(self):
        """The junction to act on: this tick's if seen, else the latched one."""
        return self.live if self.live is not None else self.latched

    @property
    def anchor_ok(self):
        """May the gate midpoint be threaded into the driven line this tick?

        Yes while there is one and it is still ahead of the car. See the module
        docstring: it is carried on measured motion rather than frozen, which
        is what makes a latched anchor usable at all, and it retires on
        geometry rather than on a timer.
        """
        return (self.engaged and self.anchor_xy is not None
                and self.anchor_xy[0] > MIN_ANCHOR_X_M
                and self.blind_ticks <= MAX_ANCHOR_BLIND_TICKS)

    @property
    def turn(self):
        return self.latched_turn if self.latched_turn else self.cursor.current

    # --- the transition ------------------------------------------------

    def _carry_forward(self, travel_m, yaw_delta_rad):
        """Move the latched divider and axis into THIS tick's base_link.

        The car went `travel_m` along its own x and turned `yaw_delta_rad`, so
        everything fixed to the ground moved the opposite way in its frame.
        """
        cos_t, sin_t = math.cos(yaw_delta_rad), math.sin(yaw_delta_rad)

        def moved(point):
            x, y = point[0] - travel_m, point[1]
            return (x * cos_t + y * sin_t, -x * sin_t + y * cos_t)

        if self.anchor_xy is not None:
            self.anchor_xy = moved(self.anchor_xy)
        if self.divider_xy is None:
            return
        self.divider_xy = moved(self.divider_xy)
        self.axis_rad -= yaw_delta_rad

    def update(self, junction, corridor_line=None, travel_m=0.0,
               yaw_delta_rad=0.0):
        """One tick. `corridor_line` is the PREVIOUS tick's unanchored line.

        Previous rather than current because the pipeline needs the state to
        decide how to build this tick's line -- the same one-tick feedback
        `drive_corridor.py` already uses for `axis_rad`.

        `travel_m` and `yaw_delta_rad` are how far the car moved and turned
        since the last tick. See the module docstring: the distance is a
        duty-cycle estimate used as a floor rather than as a measurement, and
        the pair together carry the latched divider forward while the reds are
        out of frame.
        """
        self.live = junction
        self.note = ""
        self.blind_ticks = 0 if junction is not None else self.blind_ticks + 1
        self._sightings.append(junction)
        del self._sightings[:-COMMIT_WINDOW_TICKS]
        self._carry_forward(travel_m, yaw_delta_rad)
        if junction is not None:
            # A live sighting always beats a carried-forward estimate.
            self.divider_xy = (junction.centre.x, junction.centre.y)
            self.axis_rad = junction.axis_rad
            turn = self.turn
            if turn is not None:
                gate = junction.gate_for(turn)
                self.anchor_xy = (gate[0], gate[1])

        if self.state == FOLLOW:
            self._follow(junction)
        elif self.state == APPROACH:
            self._approach(junction)
        else:
            self._traverse(junction, corridor_line, travel_m)
        return self.state

    def _seen_recently(self):
        """The most recent live junction inside the sliding window, and how
        many ticks of that window saw one at all."""
        live = [j for j in self._sightings if j is not None]
        return (live[-1] if live else None), len(live)

    def _follow(self, junction):
        if junction is not None and self.cursor.current is not None:
            self.state = APPROACH

    def _approach(self, junction):
        turn = self.cursor.current
        recent, count = self._seen_recently()
        if recent is None or turn is None:
            # Nothing seen in the whole window, so there was no junction and
            # nothing was committed -- no route entry to protect. Drop back.
            self.state = FOLLOW
            return
        if count >= COMMIT_CONFIRM_TICKS:
            # `recent` may be a tick or two old, which is why the divider and
            # axis this drives are the carried-forward ones rather than its own.
            junction = recent
            self.latched = junction
            self.latched_turn = turn
            self.state = TRAVERSE
            self.commit_range_m = junction.range_for(turn)
            self.travelled_m = 0.0
            self._confirm = 0
            self._traverse_ticks = 0

    def _traverse(self, junction, corridor_line, travel_m=0.0):
        self._traverse_ticks += 1
        self.travelled_m += max(0.0, travel_m)

        cleared = self.travelled_m >= self.commit_range_m + CLEAR_PAST_GATE_M
        if cleared and junction is None and corridor_reacquired(corridor_line):
            self._confirm += 1
        else:
            self._confirm = 0

        if self._confirm >= PASSED_CONFIRM_TICKS:
            self.cursor.advance()
            self._reset("passed")
            return

        if self._traverse_ticks >= self.max_traverse_ticks:
            # The corridor never came back. Keep the route entry -- the turn was
            # not demonstrably taken -- and stop steering on a stale latch.
            self._reset("traverse timed out; route entry kept")

    def _reset(self, note):
        self.state = FOLLOW
        self.latched = None
        self.latched_turn = None
        self.note = note
        self._confirm = 0
        self._seen = 0
        self._traverse_ticks = 0
        self.travelled_m = 0.0
        self.commit_range_m = 0.0
        self.divider_xy = None
        self.axis_rad = 0.0
        self.anchor_xy = None

    def __repr__(self):
        return f"TopoState({self.state}, turn={self.turn}, {self.cursor!r})"
