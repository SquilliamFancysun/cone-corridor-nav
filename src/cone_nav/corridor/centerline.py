"""Delaunay pairing -> midpoints -> centerline fit.
Fallback: offset half-width from a single visible boundary. Pure function, no rclpy.

Why triangulate rather than walk the two boundaries in order: at a Y-junction
the camera sees two blue walls and two yellow walls at once (see the closing
section of data/layouts/track_v1.md), and a walk built around "nearest blue
ahead, nearest yellow ahead" will happily pair cones from two different
corridors and steer into the island between them.

A triangulation has no notion of left and right. It produces candidate edges
from geometry alone, and the colour rule becomes a filter applied afterwards --
which is also what makes the SECOND filter possible. Red gate cones are placed
in PAIRS straddling the corridor, so the midpoint of a junction mouth is the
midpoint between two cones of the SAME colour. A boundary walk cannot express
that at all; here it is one more line over the same edge list.

Both filters, one structure:

    blue <-> yellow edges  ->  corridor midpoints
    red  <-> red    edges  ->  junction-mouth midpoints
"""

import math

from cone_perception.cone_classes import CLASS_BLUE, CLASS_RED, CLASS_YELLOW

from cone_nav.corridor.delaunay import edges_of, triangulate

# Corridor width is 1.5 m uniform and red gate pairs straddle it, so any edge
# meaningfully longer than that joins cones on opposite sides of the island at a
# fork, or two unrelated corridors. Delaunay will happily produce such an edge;
# it is geometrically valid and physically nonsense.
MAX_PAIR_EDGE_M = 2.5

# ...and nothing much narrower than the corridor is a corridor either. The two
# branches of a fork meet at an island nose where a blue and a yellow cone sit
# side by side, roughly 0.3 m apart (data/layouts/track_v1.md: "one yellow and
# one blue cone side by side, then taper outward"). Pairing THOSE puts a
# midpoint on the island tip and invites the car to drive at it. The corridor is
# 1.5 m wide, so 0.6 m rejects every nose pair with room to spare and cannot
# reach a real one.
MIN_PAIR_EDGE_M = 0.6

# Half the surveyed corridor width, from data/layouts/track_v1.md.
DEFAULT_HALF_WIDTH_M = 0.75

# A chain has to BEGIN near the car to be drivable. Without this, the longest
# chain in view can be one that starts several metres away down a branch the car
# is not in, which is a perfectly good line to nowhere.
START_RADIUS_M = 3.0

CORRIDOR = "corridor"
GATE = "gate"


class Midpoint(object):
    """A point on the driveable line, and the cone pair that produced it."""

    __slots__ = ("x", "y", "kind", "a", "b", "width_m")

    def __init__(self, x, y, kind, a, b, width_m):
        self.x = x
        self.y = y
        self.kind = kind
        self.a = a
        self.b = b
        self.width_m = width_m

    @property
    def xy(self):
        return (self.x, self.y)

    def __repr__(self):
        return f"Midpoint(({self.x:.2f}, {self.y:.2f}), {self.kind})"


class CenterlineResult(object):
    """The chosen line, plus everything needed to see why it was chosen."""

    __slots__ = ("points", "corridor_half_width", "single_boundary_fallback",
                 "midpoints", "adjacency", "gates")

    def __init__(self, points, corridor_half_width, single_boundary_fallback,
                 midpoints, adjacency, gates):
        self.points = points
        self.corridor_half_width = corridor_half_width
        self.single_boundary_fallback = single_boundary_fallback
        self.midpoints = midpoints
        self.adjacency = adjacency
        self.gates = gates

    def __repr__(self):
        tail = " (single-boundary)" if self.single_boundary_fallback else ""
        return f"CenterlineResult({len(self.points)} points{tail})"


def midpoint_graph(cones, max_edge_m=MAX_PAIR_EDGE_M,
                   min_edge_m=MIN_PAIR_EDGE_M):
    """LabeledCones -> (midpoints, adjacency).

    Adjacency is by SHARED CONE: two midpoints connect when the cone pairs that
    produced them have a cone in common. Consecutive midpoints down a corridor
    always do -- blue_i/yellow_i and blue_i/yellow_i+1 share blue_i -- and that
    is what turns a cloud of midpoints into something walkable.

    Shared *triangle* was the first rule tried and it does not survive a fork.
    Where the corridor widens between the junction and the island nose the
    triangulation goes irregular, the two branches' midpoint chains stop sharing
    triangles with the corridor behind them, and the graph falls into
    disconnected components: the branch the car is actually in becomes
    unreachable and the fork stops being visible as a branch at all. Sharing a
    cone is looser, and the looseness is exactly right here -- the cones at a
    junction genuinely do belong to both branches.
    """
    usable = [c for c in cones
              if c.cone_class in (CLASS_BLUE, CLASS_YELLOW, CLASS_RED)]
    points = [(c.x, c.y) for c in usable]
    triangles = triangulate(points)
    if not triangles:
        return [], {}

    midpoints = []
    edge_cones = []
    for i, j in edges_of(triangles):
        ci, cj = usable[i], usable[j]
        pair = {ci.cone_class, cj.cone_class}
        if pair == {CLASS_BLUE, CLASS_YELLOW}:
            kind = CORRIDOR
        elif ci.cone_class == CLASS_RED and cj.cone_class == CLASS_RED:
            kind = GATE
        else:
            continue

        width = math.hypot(ci.x - cj.x, ci.y - cj.y)
        if not min_edge_m <= width <= max_edge_m:
            continue

        midpoints.append(Midpoint(
            x=(ci.x + cj.x) / 2.0, y=(ci.y + cj.y) / 2.0,
            kind=kind, a=i, b=j, width_m=width))
        edge_cones.append({i, j})

    # Only corridor midpoints are linked. A gate midpoint is a landmark, not a
    # step: the red pair straddles the corridor, so its midpoint already lies on
    # the centerline the corridor pairs describe, and threading it into the
    # chain adds a near-duplicate point whose adjacency depends on which
    # triangles the reds happened to land in. gate_detect.py wants them as
    # events, which is what CenterlineResult.gates is for.
    adjacency = {i: set() for i in range(len(midpoints))}
    for i in range(len(midpoints)):
        if midpoints[i].kind != CORRIDOR:
            continue
        for j in range(i + 1, len(midpoints)):
            if midpoints[j].kind != CORRIDOR:
                continue
            if edge_cones[i] & edge_cones[j]:
                adjacency[i].add(j)
                adjacency[j].add(i)
    return midpoints, adjacency


def _longest_forward_chain(midpoints, adjacency, car_xy,
                           start_radius_m=START_RADIUS_M):
    """Longest chain of corridor midpoints moving steadily away from the car.

    Restricting steps to increasing distance-from-car turns the graph into a
    DAG, which matters for two reasons: longest path in a general graph is
    NP-hard and would need a search budget nobody can reason about, and an
    unconstrained walk can double back down the corridor it just came up. The
    track's branches diverge by 25 deg, so distance from the car increases along
    every real path and the constraint costs nothing.

    A chain may only BEGIN within `start_radius_m` of the car; past that a
    midpoint can only continue a chain, never start one. Otherwise the longest
    line in view can be one that starts four metres away in a branch the car is
    not in.

    At a fork this returns the longer branch. When junction_exec lands it
    replaces this function with "the branch the route names" -- same graph,
    same midpoints, different choice.
    """
    nodes = [i for i, m in enumerate(midpoints) if m.kind == CORRIDOR]
    if not nodes:
        return []

    dist = {i: math.hypot(midpoints[i].x - car_xy[0], midpoints[i].y - car_xy[1])
            for i in nodes}

    # Length of the longest chain ENDING at each node, computed in increasing
    # distance order so every predecessor is final by the time it is read.
    # None means no chain can end here yet -- it is too far out to be a start
    # and nothing has reached it.
    best_len = {i: (1 if dist[i] <= start_radius_m else None) for i in nodes}
    best_prev = {i: None for i in nodes}

    for node in sorted(nodes, key=lambda i: dist[i]):
        if best_len[node] is None:
            continue
        for nxt in adjacency[node]:
            if nxt not in best_len or dist[nxt] <= dist[node]:
                continue
            if best_len[nxt] is None or best_len[node] + 1 > best_len[nxt]:
                best_len[nxt] = best_len[node] + 1
                best_prev[nxt] = node

    reachable = [i for i in nodes if best_len[i] is not None]
    if not reachable:
        return []
    # Longest wins; among equals the one that reaches furthest, so a tie between
    # two branches is broken by which actually goes somewhere.
    end = max(reachable, key=lambda i: (best_len[i], dist[i]))

    chain = []
    node = end
    while node is not None:
        chain.append(node)
        node = best_prev[node]
    chain.reverse()
    return chain


def _quantile(ordered, q):
    """Linear-interpolated quantile of an already-sorted list.

    The corridor half-width is taken at q=0.25 rather than as a mean, a median
    or a minimum, and the choice is measured rather than aesthetic. A
    triangulation pairs each blue cone with several yellows, so the chain
    carries both perpendicular pairings (the actual corridor width) and diagonal
    ones (longer). On the 1.5 m corridor of track_v1, with the fork in view:

        estimator   straight   at J1    truth
        min           0.75      0.50     0.75   <- one spurious narrow pair
        p25           0.75      0.75     0.75
        median        0.75      0.83     0.75
        mean          0.89      0.84     0.75   <- diagonals inflate it

    p25 sits below the diagonals and above the occasional too-narrow pairing
    thrown up in the ambiguous zone between a junction and its island nose.
    """
    if not ordered:
        return float("nan")
    k = (len(ordered) - 1) * q
    lo = int(k)
    hi = min(lo + 1, len(ordered) - 1)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (k - lo)


def _single_boundary_line(boundary, half_width, to_left):
    """Offset a lone visible wall by half the corridor width.

    `to_left` says which side the driveable space is on: a blue wall is the
    corridor's LEFT edge, so the corridor lies to its right, and vice versa for
    yellow. Getting this backwards steers the car into the cones it can see,
    which is why it is a named argument rather than a sign buried in a formula.
    """
    if len(boundary) < 2:
        return []
    first, last = boundary[0], boundary[-1]
    dx, dy = last.x - first.x, last.y - first.y
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return []
    dx, dy = dx / length, dy / length
    # Left normal of the direction of travel.
    nx, ny = -dy, dx
    sign = 1.0 if to_left else -1.0
    return [(c.x + sign * nx * half_width, c.y + sign * ny * half_width)
            for c in boundary]


def centerline(cones, car_xy=(0.0, 0.0), half_width=DEFAULT_HALF_WIDTH_M,
               max_edge_m=MAX_PAIR_EDGE_M):
    """LabeledCones -> the line to drive, near to far, in base_link.

    Falls back to offsetting a single visible boundary when the pairing produces
    nothing usable, flagging that in the result so the consumer knows the line
    is inferred from one side rather than measured between two.
    """
    midpoints, adjacency = midpoint_graph(cones, max_edge_m=max_edge_m)
    gates = [m for m in midpoints if m.kind == GATE]
    chain = _longest_forward_chain(midpoints, adjacency, car_xy)

    if len(chain) >= 2:
        corridor_widths = [midpoints[i].width_m for i in chain]
        measured = (_quantile(sorted(corridor_widths), 0.25) / 2.0
                    if corridor_widths else half_width)
        return CenterlineResult(
            points=[midpoints[i].xy for i in chain],
            corridor_half_width=measured,
            single_boundary_fallback=False,
            midpoints=midpoints,
            adjacency=adjacency,
            gates=gates,
        )

    left = sorted([c for c in cones if c.cone_class == CLASS_BLUE],
                  key=lambda c: math.hypot(c.x - car_xy[0], c.y - car_xy[1]))
    right = sorted([c for c in cones if c.cone_class == CLASS_YELLOW],
                   key=lambda c: math.hypot(c.x - car_xy[0], c.y - car_xy[1]))
    # Prefer whichever wall is better represented; a two-cone fit through the
    # nearer wall beats a two-cone fit through a wall glimpsed at the far edge.
    points = []
    if len(left) >= len(right):
        points = _single_boundary_line(left, half_width, to_left=False)
    if not points:
        points = _single_boundary_line(right, half_width, to_left=True)
    if not points and left:
        points = _single_boundary_line(left, half_width, to_left=False)

    return CenterlineResult(
        points=points,
        corridor_half_width=half_width,
        single_boundary_fallback=bool(points),
        midpoints=midpoints,
        adjacency=adjacency,
        gates=gates,
    )
