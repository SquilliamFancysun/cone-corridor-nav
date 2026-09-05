"""The triangulation, checked against its defining property rather than trusted.

A hand-rolled Bowyer-Watson can produce a mesh that looks right in a plot and
is not Delaunay, and the corridor layer's adjacency depends on it being
Delaunay. So the empty-circumcircle property is verified exhaustively on every
fixture, which is cheap at these sizes and is the only check that actually
means anything.
"""

import random

import pytest

from cone_nav.corridor.delaunay import edges_of, in_circumcircle, triangulate


def assert_is_delaunay(points, triangles):
    """No point may lie strictly inside any triangle's circumcircle."""
    for tri in triangles:
        a, b, c = (points[i] for i in tri)
        for i, p in enumerate(points):
            if i in tri:
                continue
            assert not in_circumcircle(a, b, c, p), (
                f"point {i} {p} is inside the circumcircle of {tri}")


def test_a_single_triangle():
    points = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
    assert len(triangulate(points)) == 1


def test_a_square_becomes_two_triangles():
    points = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    triangles = triangulate(points)
    assert len(triangles) == 2
    assert_is_delaunay(points, triangles)


def test_every_triangle_is_counterclockwise():
    """in_circumcircle assumes it, and silently inverts if it is wrong."""
    points = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.5, 0.4)]
    for i, j, k in triangulate(points):
        a, b, c = points[i], points[j], points[k]
        area = (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
        assert area > 0


def test_a_corridor_of_cones_is_delaunay():
    points = []
    for i in range(6):
        points.append((i * 1.5, 0.75))
        points.append((i * 1.5, -0.75))
    triangles = triangulate(points)
    assert triangles
    assert_is_delaunay(points, triangles)


@pytest.mark.parametrize("seed", range(12))
def test_random_fields_are_delaunay(seed):
    rng = random.Random(seed)
    points = [(rng.uniform(0, 8), rng.uniform(-3, 3)) for _ in range(18)]
    triangles = triangulate(points)
    assert triangles
    assert_is_delaunay(points, triangles)


def test_no_super_triangle_vertex_survives():
    points = [(rng, rng * 0.3) for rng in (0.0, 1.0, 2.0, 3.0)] + [(1.5, 2.0)]
    for tri in triangulate(points):
        assert all(0 <= i < len(points) for i in tri)


def test_fewer_than_three_points_has_no_triangles():
    assert triangulate([]) == []
    assert triangulate([(0.0, 0.0)]) == []
    assert triangulate([(0.0, 0.0), (1.0, 0.0)]) == []


def test_collinear_cones_have_no_interior():
    """A corridor wall seen edge-on. Not an error -- there is simply no triangle."""
    points = [(float(i), 0.0) for i in range(5)]
    assert triangulate(points) == []


def test_coincident_points_do_not_hang():
    points = [(0.0, 0.0), (0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
    triangulate(points)


def test_edges_carry_their_triangle_membership():
    """Shared-triangle membership is what makes midpoints walkable."""
    points = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    triangles = triangulate(points)
    edges = edges_of(triangles)
    # The square's diagonal is interior, so it belongs to both triangles.
    shared = [e for e, tris in edges.items() if len(tris) == 2]
    assert len(shared) == 1
    # Every other edge is on the hull and belongs to exactly one.
    assert sum(1 for tris in edges.values() if len(tris) == 1) == 4


def test_edge_keys_are_normalised():
    points = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
    for i, j in edges_of(triangulate(points)):
        assert i < j


def test_a_large_field_stays_fast_enough_for_the_loop():
    """The whole track is 43 cones and this runs at 10 Hz."""
    import time
    rng = random.Random(0)
    points = [(rng.uniform(0, 10), rng.uniform(-4, 4)) for _ in range(43)]
    start = time.monotonic()
    triangulate(points)
    assert time.monotonic() - start < 0.1
