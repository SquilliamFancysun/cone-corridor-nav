"""What the planner is for is pruning, so that is what these test.

The explored run drives every dead end. The planned run drives none of them.
The tests build maps that look like real explorations -- detours included -- and
assert the route comes back without them.
"""

import pytest

from cone_nav.guidance.planner import (
    NoRouteError,
    format_route,
    route_to,
    route_to_goal,
    saving,
    write_route,
)
from cone_nav.guidance.route_exec import LEFT, RIGHT, load_route, parse_route
from cone_nav.topology.graph_builder import MazeMap, identify


def explored_maze():
    """The three-junction tree from test_explore.py, as the car would map it.

    j1-LEFT dead ends. j1-RIGHT opens j2. j2-LEFT dead ends. j2-RIGHT is the
    goal. Driven order is exactly what an ExplorePolicy produces.
    """
    maze = MazeMap()
    maze.record_pass([], LEFT, length_m=2.5)
    maze.record_dead_end([LEFT])
    maze.record_pass([], RIGHT, length_m=2.6)
    maze.record_pass([RIGHT], LEFT, length_m=2.4)
    maze.record_dead_end([RIGHT, LEFT])
    maze.record_pass([RIGHT], RIGHT, length_m=2.7)
    maze.record_goal([RIGHT, RIGHT])
    return maze


# --- routing ------------------------------------------------------------

def test_the_start_needs_no_turns():
    assert route_to(MazeMap(), MazeMap().root) == []


def test_the_planned_route_drops_every_dead_end():
    """Four gates were driven; two of them were mistakes."""
    assert route_to_goal(explored_maze()) == [RIGHT, RIGHT]


def test_the_search_agrees_with_the_key_on_a_tree():
    """In a tree the key IS the route, so a disagreement means the edges and
    the keys have drifted apart -- which is the map being wrong, not the
    planner. Worth pinning while both are cheap to compare."""
    maze = explored_maze()
    assert route_to_goal(maze) == list(maze.goal_key)


def test_a_route_to_a_dead_end_still_resolves():
    """Not useful to drive, but the report plots these and an exception here
    would take the whole figure out."""
    maze = explored_maze()
    assert route_to(maze, identify([RIGHT, LEFT])) == [RIGHT, LEFT]


def test_a_node_that_is_not_in_the_map_raises():
    with pytest.raises(NoRouteError):
        route_to(explored_maze(), identify([LEFT, LEFT, LEFT]))


def test_a_map_with_no_goal_refuses_rather_than_planning_nothing():
    """An empty route file would send the car down the first corridor taking
    no turn at all, which looks like a successful run until it is not."""
    maze = MazeMap()
    maze.record_pass([], LEFT)
    with pytest.raises(NoRouteError) as excinfo:
        route_to_goal(maze)
    assert "no goal" in str(excinfo.value)


# --- what the plan is worth ---------------------------------------------

def test_saving_counts_the_detours_the_plan_avoids():
    driven = [LEFT, RIGHT, LEFT, RIGHT]
    planned, drove, avoided = saving(explored_maze(), driven)
    assert (planned, drove, avoided) == (2, 4, 2)


# --- the file the car actually drives -----------------------------------

def test_a_generated_route_parses_as_a_hand_written_one():
    """The round trip is the point: a planner bug becomes a parse error at the
    desk instead of a wrong turn on the track."""
    text = format_route([RIGHT, RIGHT])
    assert parse_route(text) == [RIGHT, RIGHT]


def test_the_note_is_commented_out_so_it_cannot_be_parsed_as_a_turn():
    text = format_route([LEFT], note="explored 2026-09-01\n4 gates driven")
    assert parse_route(text) == [LEFT]
    assert "explored 2026-09-01" in text


def test_a_written_route_loads_through_the_normal_loader(tmp_path):
    path = tmp_path / "optimal.txt"
    write_route(route_to_goal(explored_maze()), str(path),
                note="from an exploration run")
    assert load_route(str(path)) == [RIGHT, RIGHT]
