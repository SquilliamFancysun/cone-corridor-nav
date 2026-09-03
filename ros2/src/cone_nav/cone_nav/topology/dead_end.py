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
    confused about one wall, which is a description of a dropout. Refused.
  - the signal must hold for several consecutive ticks. A stopped car's scan
    does not change, so waiting costs nothing and buys everything: this is the
    rare state machine that can afford to be slow, and it should be.

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

# Consecutive ticks the geometric signal must hold with an orange seen too.
# ~0.5 s at the measured 9.9 Hz.
CONFIRM_TICKS = 5

# And without one. Longer, because the whole case for the geometric signal is
# that it stands alone -- but a car that has already stopped is not paying for
# the wait, and half of these ticks are a chance for the detector's 69% recall
# to land one orange and take the faster path.
LONE_CONFIRM_TICKS = 12

# Fewer clusters than this in view and a collapsed line is a blind car, not a
# wall. A dead end presents its own wall plus both corridor sides.
MIN_CONES = 4

# How near an orange must be to count as this corridor's wall rather than
# something across the field, and how far off the car's axis it may sit.
WALL_RANGE_M = 2.5
WALL_OFFSET_M = 1.2

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
                 "lone_confirm_ticks", "rearm_travel_m", "_travel_since")

    def __init__(self, min_reach_m=MIN_REACH_M, min_cones=MIN_CONES,
                 confirm_ticks=CONFIRM_TICKS,
                 lone_confirm_ticks=LONE_CONFIRM_TICKS,
                 rearm_travel_m=REARM_TRAVEL_M):
        self.rearm_travel_m = rearm_travel_m
        self._travel_since = rearm_travel_m
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
            self.confirm = 0
            self.oranges_seen = 0
            self.reason = (f"re-arming in "
                           f"{self.rearm_travel_m - self._travel_since:.2f} m")
            return self.state

        if not armed:
            self.confirm = 0
            self.oranges_seen = 0
            self.reason = "not armed"
            return self.state

        self.reach_m = reach_of(line, origin) if line is not None else 0.0

        blocked = self._refusal(line, cones)
        if blocked:
            self.confirm = 0
            self.oranges_seen = 0
            self.reason = blocked
            return self.state

        if self._wall_ahead(oranges):
            self.oranges_seen += 1
        self.confirm += 1

        needed = (self.confirm_ticks if self.oranges_seen
                  else self.lone_confirm_ticks)
        if self.confirm >= needed:
            self.state = DEAD_END
            corroborated = ("orange wall seen" if self.oranges_seen
                            else "no orange, geometry alone")
            self.reason = (f"corridor ends {self.reach_m:.2f} m ahead "
                           f"({corroborated})")
        else:
            self.reason = f"confirming {self.confirm}/{needed}"
        return self.state

    # --- the refusals ---------------------------------------------------

    def _refusal(self, line, cones):
        """Why this tick is not evidence of a wall, or "" if it is."""
        if line is None:
            return "no line"
        if len(cones) < self.min_cones:
            # The dropout case. A wall is made of cones; a blind car is not.
            return f"only {len(cones)} cones in view"
        if len(line.points) < 2:
            # A wall is a corridor that stops short, so a corridor still has to
            # be there to have stopped. No points is the pairing failing, and
            # it reports as "ends 0.00 m ahead" -- which is what fired twice on
            # 2026-09-02 in place of the real wall a metre further on.
            return f"line collapsed to {len(line.points)} point(s)"
        if line.single_boundary_fallback:
            return "single-boundary fallback"
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
