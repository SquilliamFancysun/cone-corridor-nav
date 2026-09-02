"""The search is the thing being protected.

A route file can be read back and checked by eye; an exploration policy cannot,
so the invariant that matters is that the stack never sends the car somewhere it
has already proved is closed, and never forgets somewhere it has not. Both
failures look identical on the track -- a car driving in circles -- and only the
stack can tell them apart.

The other theme is that `ExplorePolicy` and `RouteCursor` really are
interchangeable. `topo_state` is written against five members and two events; a
test that pins that contract is what lets the state machine stay untouched.
"""

import pytest

from cone_nav.guidance.explore import ExplorePolicy
from cone_nav.guidance.route_exec import LEFT, RIGHT, RouteCursor


# --- the starting state -------------------------------------------------

def test_it_starts_pointed_at_its_first_choice():
    policy = ExplorePolicy()
    assert policy.current == LEFT
    assert policy.index == 0
    assert policy.remaining == 0
    assert not policy.exhausted


def test_the_first_choice_is_settable():
    assert ExplorePolicy(first=RIGHT).current == RIGHT


def test_a_first_choice_that_is_not_a_turn_raises():
    """Same refusal as the route parser, for the same reason: this steers a car."""
    with pytest.raises(ValueError):
        ExplorePolicy(first="straight")


# --- advancing ----------------------------------------------------------

def test_passing_a_gate_records_the_choice_and_the_alternative():
    policy = ExplorePolicy()
    assert policy.advance() == LEFT
    assert policy.index == 1
    assert policy.path == [LEFT]
    # The branch not taken is the thing worth remembering.
    assert policy.remaining == 1


def test_it_resets_to_its_first_choice_at_the_next_junction():
    policy = ExplorePolicy(first=RIGHT)
    policy.advance()
    assert policy.current == RIGHT


def test_untried_branches_accumulate_down_a_chain():
    policy = ExplorePolicy()
    policy.advance()
    policy.advance()
    policy.advance()
    assert policy.index == 3
    assert policy.remaining == 3
    assert policy.path == [LEFT, LEFT, LEFT]


# --- dead ends ----------------------------------------------------------

def test_a_dead_end_takes_the_branch_it_did_not_try():
    policy = ExplorePolicy()
    policy.advance()                      # took LEFT at junction 1
    assert policy.dead_end() == RIGHT
    assert policy.current == RIGHT


def test_backing_out_pops_the_junction_until_the_car_re_passes_it():
    """The car is behind the gate again, so the junction is not behind it."""
    policy = ExplorePolicy()
    policy.advance()
    policy.dead_end()
    assert policy.index == 0
    policy.advance()
    assert policy.index == 1


def test_re_entering_a_junction_marks_both_branches_tried():
    """The failure this guards: pushing a fresh choice on the way back in,
    which would re-offer the branch that just dead-ended and loop forever."""
    policy = ExplorePolicy()
    policy.advance()
    policy.dead_end()
    policy.advance()
    assert policy.path == [RIGHT]
    assert policy.remaining == 0


def test_a_second_dead_end_unwinds_past_the_spent_junction():
    """Both branches of junction 1 now end, so it is closed and the search
    must go further back, not offer LEFT again."""
    policy = ExplorePolicy()
    policy.advance()                      # j1, LEFT
    policy.advance()                      # j2, LEFT
    assert policy.dead_end() == RIGHT     # back to j2, try RIGHT
    policy.advance()                      # j2, RIGHT -- j2 now closed
    assert policy.dead_end() == RIGHT     # past j2, back to j1
    assert policy.path == []
    assert policy.index == 0


def test_a_dead_end_with_nothing_behind_it_exhausts_the_search():
    policy = ExplorePolicy()
    assert policy.dead_end() is None
    assert policy.exhausted
    assert policy.current is None
    assert "nothing left to explore" in policy.note


def test_an_exhausted_search_stays_exhausted():
    policy = ExplorePolicy()
    policy.advance()
    policy.dead_end()
    policy.advance()
    assert policy.dead_end() is None
    assert policy.exhausted
    assert policy.advance() is None
    assert policy.index == 0


def test_dead_ends_are_counted():
    policy = ExplorePolicy()
    policy.advance()
    policy.dead_end()
    assert policy.dead_ends == 1


def test_the_note_names_the_branch_it_is_backing_out_to():
    policy = ExplorePolicy()
    policy.advance()
    policy.dead_end()
    assert RIGHT in policy.note


# --- a whole small maze -------------------------------------------------

def test_it_explores_a_three_junction_tree_without_repeating_itself():
    """Layout: j1-LEFT dead ends. j1-RIGHT opens j2. j2-LEFT dead ends.
    j2-RIGHT is the way through. The car must never be sent down a branch it
    has already closed."""
    policy = ExplorePolicy()
    visited = []

    policy.advance()                      # j1, LEFT
    visited.append(policy.path[:])
    assert policy.dead_end() == RIGHT

    policy.advance()                      # j1, RIGHT
    visited.append(policy.path[:])
    policy.advance()                      # j2, LEFT
    visited.append(policy.path[:])
    assert policy.dead_end() == RIGHT     # j2's other branch

    policy.advance()                      # j2, RIGHT
    visited.append(policy.path[:])

    assert visited == [[LEFT], [RIGHT], [RIGHT, LEFT], [RIGHT, RIGHT]]
    # Every branch tried exactly once, and the search is not exhausted --
    # the car is through, which is how a real run ends.
    assert policy.remaining == 0
    assert not policy.exhausted


# --- the interface both implementations satisfy -------------------------

CURSOR_MEMBERS = ("current", "exhausted", "index", "remaining", "goal_armed")
CURSOR_EVENTS = ("advance", "dead_end")


@pytest.mark.parametrize("cursor",
                         [RouteCursor([LEFT, RIGHT]), ExplorePolicy()])
def test_both_cursors_expose_what_topo_state_reads(cursor):
    for name in CURSOR_MEMBERS:
        getattr(cursor, name)
    for name in CURSOR_EVENTS:
        assert callable(getattr(cursor, name))


def test_a_route_arms_the_goal_only_once_it_is_spent():
    cursor = RouteCursor([LEFT])
    assert not cursor.goal_armed
    cursor.advance()
    assert cursor.goal_armed


def test_exploring_arms_the_goal_from_the_start():
    """The goal is wherever the maze puts it, which is the point."""
    assert ExplorePolicy().goal_armed


def test_a_route_cursor_keeps_its_entry_on_a_dead_end():
    """The turn was not demonstrably taken, so it is not consumed -- the same
    rule topo_state's traverse timeout follows."""
    cursor = RouteCursor([LEFT, RIGHT])
    assert cursor.dead_end() is None
    assert cursor.current == LEFT
    assert cursor.remaining == 2


def test_both_cursors_report_the_path_that_got_them_here():
    """graph_builder identifies a node by this, so one builder serves both."""
    route = RouteCursor([LEFT, RIGHT])
    assert route.path == []
    route.advance()
    assert route.path == [LEFT]

    policy = ExplorePolicy()
    policy.advance()
    assert policy.path == [LEFT]
