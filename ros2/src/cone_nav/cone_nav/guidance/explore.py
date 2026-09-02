"""Choosing a turn at a junction the car has never seen. Pure, no rclpy.

`route_exec.RouteCursor` answers "which way here?" by reading the next line of a
file a human wrote. This module answers the same question with no file: it picks
a branch, remembers that it picked it, and when the branch turns out to be a dead
end it goes back and takes one it has not tried.

## The seam this sits behind

`topo_state.TopoState` reaches its cursor through five members and nothing else:

    .current      the turn to take at the next junction, or None
    .advance()    consume it -- called on exactly one confirmed gate pass
    .exhausted    no turn is outstanding
    .index        how many junctions are behind the car
    .remaining    how many are still to come

That is the whole interface, which is why swapping a file-backed cursor for a
policy that decides as it goes needs no change to the state machine at all. It is
also why the future shape of this project is a series of implementations of these
five members rather than a series of edits to `topo_state`: a wall follower, a
Trémaux marker, or an A* replay over an already-built graph are each a different
class here.

Two members are added by this module and back-filled onto `RouteCursor`:

    .dead_end()   the corridor the car is in has ended. See below
    .goal_armed   may a magenta stop the car yet?

`dead_end` is the genuinely new one. The old interface could only say *advance*,
because a provided route has exactly one outcome per junction; exploring has two,
and they are not the same event. A confirmed pass means the turn worked. A dead
end means it did not, and the difference is the whole of the search.

`goal_armed` exists because the two implementations disagree about it for a real
reason rather than a stylistic one. On a provided route the goal lies past the
last junction *by construction*, so a magenta seen with turns outstanding is a
misread -- and magenta/red is the detector's hardest pair. In a maze the goal is
wherever it is; that is the point. So exploration arms it everywhere, and accepts
the false-positive risk `drive_junction.py --goal-anywhere` warns about, because
here it is inherent rather than a bring-up shortcut.

## Depth-first, and why no localisation is needed

The search is a plain DFS over a stack of decisions. What makes that sound with
no pose estimate and no map is that a maze without loops is a TREE: the path of
turns taken from the start identifies where the car is, uniquely, with no
geometry involved. "Have I been at this junction before?" -- the question that
makes SLAM hard -- is answered by the stack rather than by recognising a place.

That property is exactly what stops holding when the maze has a loop, at which
point two different paths reach one junction and the stack starts lying. This
module does not detect that case and cannot. `graph_builder.identify` is where
that would be caught, and its key is deliberately opaque so it can grow a
pose-derived component without anything here changing.

## What backing out costs the caller

`dead_end()` only decides; it does not move the car. Between the call and the
next `advance()` the vehicle has to physically get back to the junction, which
`cone_nav/control/reverse_ctrl.py` does under power and an operator does by
picking the car up. Neither is visible from here, and that is deliberate: the
search is correct either way, and the graph it builds does not record which one
happened.
"""

from cone_nav.guidance.route_exec import LEFT, TURNS


class _Choice(object):
    """One junction the car has committed to, and what it has left to try."""

    __slots__ = ("taken", "untried")

    def __init__(self, taken, untried):
        self.taken = taken
        self.untried = list(untried)

    def __repr__(self):
        left = "/".join(self.untried) if self.untried else "none"
        return f"_Choice(took {self.taken}, untried {left})"


class ExplorePolicy(object):
    """Depth-first exploration of an unknown junction tree.

    Implements the cursor interface `topo_state.TopoState` consumes, so it drops
    in wherever a `RouteCursor` goes.
    """

    __slots__ = ("first", "stack", "pending", "_resuming", "_dead_ends", "note")

    def __init__(self, first=LEFT):
        if first not in TURNS:
            raise ValueError(
                f"{first!r} is not a turn. Expected one of {list(TURNS)}.")
        self.first = first
        self.stack = []
        self.pending = first
        self._resuming = None
        self._dead_ends = 0
        self.note = ""

    # --- the interface topo_state reads --------------------------------

    @property
    def current(self):
        """The turn to take at the next junction, or None when the tree is spent."""
        return self.pending

    @property
    def exhausted(self):
        """No turn outstanding.

        For a route this means every provided turn was taken. Here it means the
        search has nowhere left to go: every junction behind the car has had
        both branches tried and all of them ended. On a solvable maze the run
        finishes at the goal long before this is true, so seeing it in a log is
        a finding -- either the goal was missed or the layout has no route.
        """
        return self.pending is None

    @property
    def index(self):
        """Junctions behind the car. The log's `route_index`."""
        return len(self.stack)

    @property
    def remaining(self):
        """Branches known to be untried. The log's `route_remaining`.

        Not a count of junctions left -- the car cannot know that -- but of the
        choices it could still come back for. It rises as the car finds
        junctions and falls as it exhausts them, which is what makes it a
        readable progress figure in a maze rather than a countdown.
        """
        return sum(len(choice.untried) for choice in self.stack)

    @property
    def goal_armed(self):
        """Always. The goal can be anywhere in a maze -- see the module docstring."""
        return True

    # --- the two events ------------------------------------------------

    def advance(self):
        """A gate was passed on `current`. Record the choice and move on.

        Idempotent past the end, matching `RouteCursor.advance`, so a caller
        that double-fires on one gate cannot corrupt the stack.
        """
        if self.pending is None:
            return None
        taken = self.pending

        if self._resuming is not None:
            # Re-entering a junction the car backed out of. The choice object
            # already had this branch removed from `untried` when it was
            # popped, so pushing it back records both branches as tried.
            self._resuming.taken = taken
            self.stack.append(self._resuming)
            self._resuming = None
        else:
            self.stack.append(
                _Choice(taken, [t for t in TURNS if t != taken]))

        self.pending = self.first
        self.note = ""
        return taken

    def dead_end(self):
        """The corridor the car is in has ended. Choose where to resume.

        Unwinds the stack to the deepest junction with a branch left to try and
        returns that branch, which becomes `current`. Junctions whose branches
        are both spent are discarded on the way past -- their whole subtree is
        known to be closed, so the car will never be sent back into one.

        Returns None when nothing is left anywhere, which sets `exhausted`.
        """
        self._dead_ends += 1
        self._resuming = None

        while self.stack:
            choice = self.stack.pop()
            if choice.untried:
                self.pending = choice.untried.pop(0)
                self._resuming = choice
                self.note = f"dead end; backing out to try {self.pending}"
                return self.pending

        self.pending = None
        self.note = ("dead end with no junction behind the car; "
                     "nothing left to explore")
        return None

    # --- what the log and the graph read -------------------------------

    @property
    def path(self):
        """The turns taken to get here, outermost first.

        This is the car's position in a tree-shaped maze, and it is what
        `graph_builder` identifies a node by. It is not a pose and says nothing
        about where the junction is in metres.
        """
        return [choice.taken for choice in self.stack]

    @property
    def dead_ends(self):
        return self._dead_ends

    def __repr__(self):
        where = "spent" if self.exhausted else f"at {self.pending}"
        return (f"ExplorePolicy({self.index} deep, {where}, "
                f"{self.remaining} untried, {self._dead_ends} dead ends)")
