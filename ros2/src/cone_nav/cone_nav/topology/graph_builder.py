"""The map the car builds while exploring. Pure, no rclpy.

A node is a place where the car had a choice; an edge is the corridor it drove
to get from one to the next. That is the whole structure, and it is deliberately
topological: there are no metres in it that anything steers by.

## What identifies a node, and why that is the interesting decision

`identify()` turns the path of turns taken so far into a node key. Today it
returns that path, which is sound for exactly one reason: **a maze without loops
is a tree, and in a tree the route from the start names the place uniquely.**
"Have I been at this junction before?" -- the question that makes SLAM hard -- is
answered by the traversal history, with no pose estimate, no landmark matching,
and nothing that can drift.

The moment the maze has a loop that stops being true: two different paths reach
one junction, the map records them as two nodes, and every count built on it is
wrong. Nothing here detects that, and nothing here can.

So the key is opaque on purpose. Growing this to a looped maze means giving
`identify()` a pose-derived component -- cluster the junction's world position
out of `cone_perception/odometry.py` and use that when two paths land close
enough to be the same place -- and *nothing else in this module changes*, because
nothing else looks inside a key. That is the entire reason for the indirection,
and it is worth the one extra function today.

## What the lengths are for, honestly

Edges carry a length from integrated scan-matched odometry. In a tree it changes
no decision: there is exactly one route to any node, so the shortest one is the
only one. The lengths are recorded because they are what the report plots and
what a looped maze would need, not because the planner reads them. Saying so here
is cheaper than someone later trusting a number that has never been on the
critical path of anything.
"""

JUNCTION = "junction"
DEAD_END = "dead_end"
GOAL = "goal"


def identify(path):
    """Turns taken from the start -> an opaque node key.

    See the module docstring. The return value is a hashable token and callers
    must not read structure out of it, so that a looped maze can change what
    goes into it without changing anything that holds one.
    """
    return tuple(path)


class Node(object):
    __slots__ = ("key", "kind")

    def __init__(self, key, kind=JUNCTION):
        self.key = key
        self.kind = kind

    def __repr__(self):
        where = "/".join(self.key) if self.key else "start"
        return f"Node({where}, {self.kind})"


class Edge(object):
    """A corridor driven, from one node to the next."""

    __slots__ = ("turn", "to_key", "length_m")

    def __init__(self, turn, to_key, length_m=0.0):
        self.turn = turn
        self.to_key = to_key
        self.length_m = length_m

    def __repr__(self):
        return f"Edge({self.turn} -> {self.to_key}, {self.length_m:.2f} m)"


class MazeMap(object):
    """Junctions and the corridors between them, accumulated during a run.

    Fed from the two events `topo_state` already produces -- a confirmed gate
    pass and a dead end -- plus the goal latch. It never sees the vehicle and
    never decides anything; `cone_nav/guidance/planner.py` is what reads it.
    """

    __slots__ = ("nodes", "edges", "root")

    def __init__(self):
        self.root = identify([])
        self.nodes = {self.root: Node(self.root)}
        self.edges = {}

    # --- recording -----------------------------------------------------

    def _node(self, key, kind=JUNCTION):
        node = self.nodes.get(key)
        if node is None:
            node = self.nodes[key] = Node(key, kind)
        return node

    def record_pass(self, path, turn, length_m=0.0):
        """A gate was passed at `path`, taking `turn`. Returns the new key.

        Idempotent: driving the same junction twice -- which happens every time
        the car backs out of a dead end and takes the other branch -- must not
        duplicate the edge or double the length. The second traversal's length
        is kept, being the one measured on a car that was not about to stop.
        """
        here = identify(path)
        self._node(here, JUNCTION).kind = JUNCTION
        there = identify(list(path) + [turn])
        self._node(there)

        edges = self.edges.setdefault(here, [])
        for edge in edges:
            if edge.turn == turn:
                edge.length_m = length_m
                return there
        edges.append(Edge(turn, there, length_m))
        return there

    def record_dead_end(self, path):
        """The corridor at `path` ended in a wall."""
        self._node(identify(path)).kind = DEAD_END

    def record_goal(self, path):
        """The goal was found at `path`."""
        self._node(identify(path)).kind = GOAL

    # --- reading -------------------------------------------------------

    def neighbours(self, key):
        return list(self.edges.get(key, ()))

    def kind_of(self, key):
        node = self.nodes.get(key)
        return node.kind if node is not None else None

    def find(self, kind):
        """Every node of a kind, in insertion order."""
        return [n.key for n in self.nodes.values() if n.kind == kind]

    @property
    def goal_key(self):
        found = self.find(GOAL)
        return found[0] if found else None

    def summary(self):
        """One line for the log and the console."""
        return (f"{len(self.nodes)} nodes, "
                f"{sum(len(e) for e in self.edges.values())} edges, "
                f"{len(self.find(DEAD_END))} dead ends")

    def __repr__(self):
        return f"MazeMap({self.summary()})"
