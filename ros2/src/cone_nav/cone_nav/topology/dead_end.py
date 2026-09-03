"""Deciding that the corridor has ended in a wall. Pure, no rclpy.

The car already behaves correctly at a dead end: `speed_ctrl` stops it with
`"corridor visible only 0.42 m ahead"` and it sits there. What is missing is the
car *knowing* that is what happened, because a stop is where exploration has to
make a decision and a `stop_reason` string is not a decision.

So this module turns that stop into a named, latched event that
`guidance/explore.py` can act on.

## Why the orange cone is corroboration and not the signal

A dead end is walled with an orange cone, so reading the wall off the detector
is the obvious design. The measured numbers say otherwise. On the v3 test split
(`model/training/v3/report_test.md`) orange has **recall 0.687** -- roughly a
third of them missed on any given frame -- and, worse, **15% of oranges are
called red**. Red is the junction class. A dead end misread as a gate is the one
confusion `docs/junction-bringup.md` calls "the worst on the track", because it
hands a wall to `junction_exec` as something to drive through.

The geometry does not have that problem. At a wall the corridor pairing simply
runs out and the driven line collapses, and that is measured by
`speed_ctrl.reach_of` from lidar clusters whose positions are good to a
centimetre and owe nothing to the classifier. So the primary signal is
geometric, orange is corroboration, and the machine works with orange absent --
only more slowly.

## The false positive that actually matters

Not a wall misread. A **perception dropout**: cones vanish, the line collapses,
reach falls to zero, and a naive rule declares a dead end in the middle of a
clear corridor -- sending the car reversing out of a corridor that was never
blocked. Three things guard it, and all three are needed:

  - a dead end is *made of cones*, so a collapsed line with almost nothing in
    view is a blind car, not a wall. `min_cones` refuses that case outright.
  - **the line itself has to have survived.** A wall is a corridor that stops
    SHORT -- the last blue/yellow pair is still there, still pairing, still
    producing a midpoint, and reach lands somewhere around 0.8 m. A line with
    no points at all is not a short corridor, it is the absence of one, and
    reading it as a wall makes `reach` say the corridor "ends 0.00 m ahead",
    which is a contradiction rather than a measurement. Cone count alone does
    not catch this: measured 2026-09-02 (`explore-3.jsonl`), plenty of clusters
    were in view and the pairing still produced nothing.
  - a single-boundary fallback line is what the car produces while it is
    confused about one wall, which is a description of a dropout. Refused --
    **unless an orange says otherwise**. That exception is not a softening, it
    is the correction to an over-strict rule. A car at a dead end is close to
    the wall and usually angled, so one boundary legitimately leaves the usable
    arc: it has ARRIVED, not got confused. Measured 2026-09-02
    (`data/trials/explore-6.jsonl`): an orange was present on 110 of 110 ticks
    at a wall, tracking smoothly from 1.44 m to 0.81 m and sitting within 4 cm
    of the centreline -- and 63 of those ticks were refused as a fallback. A
    well-placed orange is positive evidence that the missing wall is the end of
    the corridor rather than a dropout, which is the distinction this guard
    exists to draw and could not draw alone.
  - the signal must hold across a WINDOW of ticks -- a count within it, not a
    run of consecutive ones. That distinction is the whole of this paragraph
    and it was got wrong first time round.

    Consecutive confirmation was copied from `goal_stop`, whose own docstring
    says its sightings are "plentiful here in a way they emphatically are not
    at a junction". A wall is the junction case. Measured 2026-09-02
    (`data/trials/explore-7.jsonl`): 19 of 114 ticks at a wall passed every
    geometric test, six in any ten -- and the longest CONSECUTIVE run was
    **two**, against a rule needing five. It could not fire, ever, while the
    evidence was plainly there. `reach` flickers 0.98, 1.33, 1.40, 1.83, 1.91,
    0.98 as the centerline finds a longer or shorter chain, and one good tick
    in three is what that looks like.

    `topo_state` met this first and answered it the same way, for the same
    reason -- "sightings are the scarce resource, not confidence" -- with
    `COMMIT_WINDOW_TICKS` and a count inside it.

    A stopped car's scan does not change, so waiting still costs nothing: this
    remains the rare state machine that can afford to be slow. What it cannot
    afford is to require a steadiness the perception does not have.

## Where this must not run

Two places where the corridor legitimately ends and the car is right to be
there, and in both the caller holds this machine down rather than the machine
guessing:

  - **through a junction mouth.** `topo_state`'s own docstring records that the
    corridor "looks healthy for the whole approach" and then does not: the mouth
    is exactly a stretch of short, one-sided line. `TopoState.engaged` is the
    flag.
  - **the goal run-in.** `goal_stop` deliberately stands the reach floor down
    over the last metre because the course really has ended. A dead end
    declared there would back the car away from the trophy it just reached.
"""

from cone_nav.control.speed_ctrl import MIN_REACH_M, reach_of

CLEAR = "clear"
DEAD_END = "dead_end"

# The window the evidence is counted over. 2 s at the measured 9.9 Hz -- long
# enough to ride out the reach flicker, short enough that a wall is named while
# the car is still short of it.
WINDOW_TICKS = 20

# Evidence ticks needed inside that window, with an orange seen too.
#
# Sized from measured runs rather than chosen. Peak density in any 20-tick
# window, across every driven run of 2026-09-02: at a wall 20/20, 10/20 and
# 9/20; in a healthy corridor **0/20 on all of them**. The geometric test
# simply does not pass while a corridor is open, which is what makes a count
# safe where it would otherwise invite false positives.
CONFIRM_TICKS = 5

# And without one. More, because the whole case for the geometric signal is
# that it stands alone -- but still inside the worst measured wall density, so
# a run where the detector never finds the orange can still name the wall.
LONE_CONFIRM_TICKS = 8

# Fewer clusters than this in view and a collapsed line is a blind car, not a
# wall. A dead end presents its own wall plus both corridor sides.
MIN_CONES = 4

# How near an orange must be to count as this corridor's wall rather than
# something across the field.
WALL_RANGE_M = 2.5

# And how far off the car's axis it may sit. This is a POSITION test doing a
# classification job, and it is worth saying why.
#
# A dead-end wall cone stands across the end of a corridor, near its
# centreline. A boundary cone stands on the wall, at the corridor half-width --
# `centerline.DEFAULT_HALF_WIDTH_M`, 0.75 m. Those are different places, so an
# orange at 0.75 m off the axis is far more likely to be a misread yellow than
# a wall.
#
# That is not hypothetical. Measured 2026-09-02 in a setting sun
# (`data/trials/explore-5.jsonl`): orange was 27% of all boundary-ish sightings
# on a track carrying ONE orange cone per dead end -- 647 of them -- while
# yellow ran depleted against blue, 787 to 955. The missing yellows were being
# called orange.
#
# 0.5 m keeps a wall cone the car meets while offset or angled, and rejects a
# boundary cone at 0.75 m with a quarter-metre of margin either way.
#
# This only stops a misread ACCELERATING the confirmation from twelve ticks to
# five. It cannot restore the wall the misread destroyed: a cone labelled
# orange leaves `boundary_split`'s yellow bucket, so the corridor loses that
# side, and `side_assign.fill_unlabeled` will not repaint a cone that already
# carries a class. A confident wrong label costs more than a missing one, and
# the fix for that is the detector, not this constant.
WALL_OFFSET_M = 0.5

# How far the car must travel after a release before this may latch again.
#
# Releasing the latch does not move the car, and a stopped car's scan does not
# change -- which is the same property that lets the machine be slow and
# careful, working against it here. Measured 2026-09-02 (`explore-2.jsonl`):
# X was re-pressed at the wall, the unchanged scan re-confirmed twelve ticks
# later, and the run recorded THREE dead ends for two real walls, each one
# also spending a `cursor.dead_end()` the search had not earned.
#
# A travel floor rather than a tick timer, because what makes the next latch
# trustworthy is new evidence, not elapsed time. 0.5 m is enough to leave a
# wall the car stopped ~0.8 m from. An operator-assisted lift reads as zero
# travel -- `rigid_step` cannot see a carry -- so a car put back at the
# junction stays suppressed until it drives away, which is exactly right.
REARM_TRAVEL_M = 0.5


class DeadEndLatch(object):
    """Has the corridor ended? Latched, and released only by the deadman.

    Mirrors `goal_stop.GoalLatch`: a confirm count, a sticky latch, and a
    `release()` on the deadman's rising edge so the operator can clear a wrong
    call and drive on without restarting the tool.
    """

    __slots__ = ("state", "reason", "confirm", "oranges_seen", "reach_m",
                 "min_reach_m", "min_cones", "confirm_ticks",
                 "lone_confirm_ticks", "rearm_travel_m", "_travel_since",
                 "window_ticks", "_window")

    def __init__(self, min_reach_m=MIN_REACH_M, min_cones=MIN_CONES,
                 confirm_ticks=CONFIRM_TICKS,
                 lone_confirm_ticks=LONE_CONFIRM_TICKS,
                 rearm_travel_m=REARM_TRAVEL_M, window_ticks=WINDOW_TICKS):
        self.rearm_travel_m = rearm_travel_m
        self._travel_since = rearm_travel_m
        self.window_ticks = window_ticks
        # (evidence, orange) per tick, most recent last.
        self._window = []
        self.min_reach_m = min_reach_m
        self.min_cones = min_cones
        self.confirm_ticks = confirm_ticks
        self.lone_confirm_ticks = lone_confirm_ticks
        self.state = CLEAR
        self.reason = ""
        self.confirm = 0
        self.oranges_seen = 0
        self.reach_m = 0.0

    @property
    def latched(self):
        return self.state == DEAD_END

    def release(self):
        """Clear the latch. The deadman's rising edge, same as `GoalLatch`.

        Also starts the re-arm travel floor: the car is still at the wall it
        just named, so without one the next twelve ticks name it again.
        """
        self.state = CLEAR
        self.reason = ""
        self.confirm = 0
        self.oranges_seen = 0
        self._window = []
        self._travel_since = 0.0

    def update(self, line, cones, oranges=(), armed=True, origin=(0.0, 0.0),
               travel_m=0.0):
        """One tick.

        `armed` is the caller holding the machine down where the corridor is
        allowed to end -- through a junction mouth and over the goal run-in.
        See the module docstring. `oranges` is `boundary_split.split().dead_ends`,
        already range-sorted; it is corroboration only. `travel_m` is this
        tick's measured travel, which only the re-arm floor reads.
        """
        if self.latched:
            return self.state

        self._travel_since += abs(travel_m)
        if self._travel_since < self.rearm_travel_m:
            self._clear()
            self.reason = (f"re-arming in "
                           f"{self.rearm_travel_m - self._travel_since:.2f} m")
            return self.state

        if not armed:
            self._clear()
            self.reason = "not armed"
            return self.state

        self.reach_m = reach_of(line, origin) if line is not None else 0.0

        wall = self._wall_ahead(oranges)
        blocked = self._refusal(line, cones, wall)

        # A refused tick is counted as evidence-against rather than wiping the
        # window. That is the change: the reach flicker means a wall produces
        # roughly one good tick in three, and resetting on every bad one made
        # the count unreachable while the evidence was plainly there.
        self._window.append((not blocked, wall))
        del self._window[:-self.window_ticks]
        self.confirm = sum(1 for good, _ in self._window if good)
        self.oranges_seen = sum(1 for good, o in self._window if good and o)

        needed = (self.confirm_ticks if self.oranges_seen
                  else self.lone_confirm_ticks)
        if self.confirm >= needed:
            self.state = DEAD_END
            corroborated = ("orange wall seen" if self.oranges_seen
                            else "no orange, geometry alone")
            self.reason = (f"corridor ends {self.reach_m:.2f} m ahead "
                           f"({corroborated})")
        elif blocked:
            # Say what stopped THIS tick, with the tally beside it, so the
            # reason field still names the fault and no longer implies the
            # count went back to zero.
            self.reason = f"{blocked}  [{self.confirm}/{needed}]"
        else:
            self.reason = f"confirming {self.confirm}/{needed}"
        return self.state

    def _clear(self):
        self.confirm = 0
        self.oranges_seen = 0
        self._window = []

    # --- the refusals ---------------------------------------------------

    def _refusal(self, line, cones, wall=False):
        """Why this tick is not evidence of a wall, or "" if it is.

        `wall` is whether an orange sits where the end of this corridor would
        be. It buys exactly one refusal -- see the fallback below.
        """
        if line is None:
            return "no line"
        if len(cones) < self.min_cones:
            # The dropout case. A wall is made of cones; a blind car is not.
            return f"only {len(cones)} cones in view"
        # No exception here, deliberately. Zero points is not a corridor that
        # stopped short, it is the absence of one, and `reach` then reads "ends
        # 0.00 m ahead" -- a contradiction that already went out as a decision
        # once today. An orange cannot make that mean anything, so it does not
        # get to.
        if len(line.points) < 2:
            # A wall is a corridor that stops short, so a corridor still has to
            # be there to have stopped. No points is the pairing failing, and
            # it reports as "ends 0.00 m ahead" -- which is what fired twice on
            # 2026-09-02 in place of the real wall a metre further on.
            return f"line collapsed to {len(line.points)} point(s)"
        if line.single_boundary_fallback and not wall:
            # See the module docstring. The guard is right about a dropout and
            # wrong about an arrival, and a well-placed orange is what tells
            # them apart. With no orange it still refuses.
            return "single-boundary fallback, and no orange to say otherwise"
        if self.reach_m >= self.min_reach_m:
            return f"corridor reaches {self.reach_m:.2f} m"
        return ""

    def _wall_ahead(self, oranges):
        """Is one of these oranges positioned like this corridor's wall?

        `oranges` is range-sorted, so the first one inside the range gate is
        the nearest candidate and the rest cannot be nearer.
        """
        for cone in oranges:
            if cone.x < 0.0 or cone.x > WALL_RANGE_M:
                continue
            if abs(cone.y) <= WALL_OFFSET_M:
                return True
        return False

    def __repr__(self):
        return (f"DeadEndLatch({self.state}, reach {self.reach_m:.2f} m, "
                f"confirm {self.confirm}, {self.reason})")
