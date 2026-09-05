"""The map is written once per junction and read once per run, so the tests are
about the writes that happen more than once.

Backing out of a dead end and taking the other branch drives the SAME junction
twice. A map that recorded that as two junctions would count every later node
twice over, and the error would not show up until the planner emitted a route
with a phantom turn in it.
"""

from cone_nav.guidance.route_exec import LEFT, RIGHT
from cone_nav.topology.graph_builder import (
    DEAD_END,
    GOAL,
    JUNCTION,
    MazeMap,
    identify,
)


# --- keys ---------------------------------------------------------------

def test_the_start_is_the_empty_path():
    assert identify([]) == ()


def test_a_key_is_hashable_so_it_can_index_the_map():
    {identify([LEFT, RIGHT]): 1}


def test_keys_from_equal_paths_are_equal():
    """The whole tree argument rests on this: same route, same place."""
    assert identify([LEFT, RIGHT]) == identify([LEFT, RIGHT])
    assert identify([LEFT, RIGHT]) != identify([RIGHT, LEFT])


# --- recording ----------------------------------------------------------

def test_a_new_map_holds_only_the_start():
    maze = MazeMap()
    assert list(maze.nodes) == [maze.root]
    assert maze.neighbours(maze.root) == []


def test_passing_a_gate_adds_the_node_beyond_it():
    maze = MazeMap()
    key = maze.record_pass([], LEFT, length_m=2.0)
    assert key == identify([LEFT])
    assert maze.kind_of(maze.root) == JUNCTION
    edges = maze.neighbours(maze.root)
    assert len(edges) == 1
    assert edges[0].turn == LEFT
    assert edges[0].length_m == 2.0


def test_both_branches_of_one_junction_are_two_edges_from_one_node():
    maze = MazeMap()
    maze.record_pass([], LEFT)
    maze.record_pass([], RIGHT)
    assert len(maze.neighbours(maze.root)) == 2
    assert len(maze.nodes) == 3


def test_re_driving_a_junction_does_not_duplicate_the_edge():
    """The backtrack case: the car passes this gate once going in and once
    coming back to take the other branch."""
    maze = MazeMap()
    maze.record_pass([], LEFT, length_m=2.0)
    maze.record_pass([], LEFT, length_m=2.4)
    edges = maze.neighbours(maze.root)
    assert len(edges) == 1
    assert edges[0].length_m == 2.4


def test_a_dead_end_is_a_kind_not_a_deletion():
    """The node stays in the map. The planner has to be able to see that the
    branch was tried, and the report has to be able to plot it."""
    maze = MazeMap()
    maze.record_pass([], LEFT)
    maze.record_dead_end([LEFT])
    assert maze.kind_of(identify([LEFT])) == DEAD_END
    assert identify([LEFT]) in maze.nodes


def test_the_goal_is_findable_by_kind():
    maze = MazeMap()
    maze.record_pass([], RIGHT)
    maze.record_goal([RIGHT])
    assert maze.goal_key == identify([RIGHT])
    assert maze.find(GOAL) == [identify([RIGHT])]


def test_no_goal_reads_as_none_rather_than_raising():
    assert MazeMap().goal_key is None


def test_summary_counts_what_the_console_line_shows():
    maze = MazeMap()
    maze.record_pass([], LEFT, length_m=2.0)
    maze.record_dead_end([LEFT])
    maze.record_pass([], RIGHT, length_m=2.1)
    assert maze.summary() == "3 nodes, 2 edges, 1 dead ends"


# --- unmeasured edges ---------------------------------------------------

def test_an_edge_with_no_length_is_unmeasured_not_zero():
    """What an operator-assisted backtrack produces: the car was carried, the
    pose could not see it, and a length measured across that is wrong rather
    than merely absent."""
    maze = MazeMap()
    maze.record_pass([], LEFT)
    edge = maze.neighbours(maze.root)[0]
    assert edge.length_m is None
    assert not edge.measured


def test_the_summary_says_when_lengths_are_missing():
    """On the line everyone reads, not in a field nobody opens."""
    maze = MazeMap()
    maze.record_pass([], LEFT)
    assert "1 edge(s) unmeasured" in maze.summary()


def test_an_unmeasured_re_drive_does_not_erase_a_measured_length():
    """The car drives a junction twice every time it backs out of one. The
    second pass may be the carried one; it must not delete what the first
    pass actually measured."""
    maze = MazeMap()
    maze.record_pass([], LEFT, length_m=2.0)
    maze.record_pass([], LEFT, length_m=None)
    assert maze.neighbours(maze.root)[0].length_m == 2.0


def test_a_measured_re_drive_replaces_an_unmeasured_length():
    maze = MazeMap()
    maze.record_pass([], LEFT)
    maze.record_pass([], LEFT, length_m=2.4)
    assert maze.neighbours(maze.root)[0].length_m == 2.4
