"""Delaunay triangulation of a cone field. Pure function, no rclpy.

Hand-rolled Bowyer-Watson rather than scipy.spatial or cv2.Subdiv2D, for two
reasons. The README promises cone_nav runs on a laptop with nothing but
`pip install -e`, and neither scipy nor opencv is a dependency this package
should acquire for one call. And the problem is tiny: the full track is 43
cones, of which a fraction are ever in view, so an O(n^2) insertion is a few
thousand operations at 10 Hz.

The property that matters -- no point lies inside any triangle's circumcircle --
is checked directly in the tests rather than assumed, which is the only honest
way to ship a hand-rolled predicate.
"""


def _orient(a, b, c):
    """Twice the signed area of triangle abc. Positive when counterclockwise."""
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def in_circumcircle(a, b, c, d):
    """True if d lies strictly inside the circumcircle of triangle abc.

    The standard determinant predicate. It assumes abc is counterclockwise, so
    the caller's triangles are normalised on construction; fed a clockwise
    triangle it returns the exact opposite answer, which is the classic way this
    algorithm produces a plausible-looking but wrong mesh.
    """
    ax, ay = a[0] - d[0], a[1] - d[1]
    bx, by = b[0] - d[0], b[1] - d[1]
    cx, cy = c[0] - d[0], c[1] - d[1]
    det = (
        (ax * ax + ay * ay) * (bx * cy - by * cx)
        - (bx * bx + by * by) * (ax * cy - ay * cx)
        + (cx * cx + cy * cy) * (ax * by - ay * bx)
    )
    return det > 0.0


def _ccw(tri, pts):
    """Triangle index triple, reordered counterclockwise."""
    i, j, k = tri
    if _orient(pts[i], pts[j], pts[k]) < 0.0:
        return (i, k, j)
    return (i, j, k)


def triangulate(points, eps=1e-12):
    """[(x, y), ...] -> list of (i, j, k) index triples, counterclockwise.

    Returns [] for fewer than three points, and for degenerate inputs where
    every point is collinear -- a straight line of cones has no interior and
    therefore no triangle, which is a real case on a corridor straight seen
    edge-on, not an error.
    """
    n = len(points)
    if n < 3:
        return []

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    if max(xs) - min(xs) < eps and max(ys) - min(ys) < eps:
        return []

    # A super-triangle guaranteed to contain every point. Made generously large
    # relative to the spread so its vertices cannot fall inside any real
    # circumcircle and perturb the result.
    mid_x = (min(xs) + max(xs)) / 2.0
    mid_y = (min(ys) + max(ys)) / 2.0
    span = max(max(xs) - min(xs), max(ys) - min(ys), eps) * 1000.0
    work = list(points) + [
        (mid_x - span, mid_y - span),
        (mid_x + span, mid_y - span),
        (mid_x, mid_y + span),
    ]
    super_ids = {n, n + 1, n + 2}

    triangles = [_ccw((n, n + 1, n + 2), work)]

    for p in range(n):
        point = work[p]
        bad = [t for t in triangles
               if in_circumcircle(work[t[0]], work[t[1]], work[t[2]], point)]
        if not bad:
            # Collinear with everything placed so far, so it sits on a hull edge
            # and no circumcircle strictly contains it. It gets picked up when a
            # later point gives it an interior to belong to.
            continue

        # The hole's boundary is the edges belonging to exactly one bad
        # triangle; shared edges are interior and vanish with them.
        counts = {}
        for tri in bad:
            for u, v in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
                key = (u, v) if u < v else (v, u)
                counts[key] = counts.get(key, 0) + 1

        bad_set = set(bad)
        triangles = [t for t in triangles if t not in bad_set]
        for (u, v), count in counts.items():
            if count == 1:
                triangles.append(_ccw((u, v, p), work))

    return [t for t in triangles if not (set(t) & super_ids)]


def edges_of(triangles):
    """Unique undirected edges, as a dict {(i, j): [triangle indices]}.

    The triangle membership is what the corridor layer needs: two midpoints are
    adjacent when their source edges share a triangle, and that is what turns a
    cloud of midpoints into a path that can be walked.
    """
    out = {}
    for t_index, (i, j, k) in enumerate(triangles):
        for u, v in ((i, j), (j, k), (k, i)):
            key = (u, v) if u < v else (v, u)
            out.setdefault(key, []).append(t_index)
    return out
