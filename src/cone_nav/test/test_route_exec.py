"""The route file is a human-written input that steers a car.

So the tests are mostly about the parser refusing things, and about the cursor
advancing exactly once per junction. A cursor that double-advances skips a turn
silently, which on the track looks identical to a detector failure and is much
harder to diagnose.
"""

import os

import pytest

from cone_nav.guidance.route_exec import (
    LEFT,
    RIGHT,
    RouteCursor,
    load_route,
    parse_route,
)


# --- parsing ------------------------------------------------------------

def test_a_plain_route_parses_in_order():
    assert parse_route("left\nright\nright\n") == [LEFT, RIGHT, RIGHT]


def test_comments_and_blank_lines_are_ignored():
    text = ("# the route for track_v1\n"
            "\n"
            "left    # first junction\n"
            "\n"
            "   right\n")
    assert parse_route(text) == [LEFT, RIGHT]


def test_case_and_surrounding_space_do_not_matter():
    assert parse_route("  LEFT  \n\tRight\n") == [LEFT, RIGHT]


def test_an_unknown_turn_raises_and_names_the_line():
    """The line number is the whole point of the message: a route file is
    edited by hand between runs and 'straight' is the obvious thing to type."""
    with pytest.raises(ValueError) as excinfo:
        parse_route("left\nstraight\nright\n")
    assert "line 2" in str(excinfo.value)
    assert "straight" in str(excinfo.value)


def test_a_typo_is_not_silently_skipped():
    """The dangerous failure mode. 'lft' must not parse as a one-turn route --
    that is a car that drives past the junction it was told to turn at."""
    with pytest.raises(ValueError):
        parse_route("lft\nright\n")


# --- loading ------------------------------------------------------------

def test_the_shipped_route_file_loads():
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "..", "..", "data", "routes",
                        "route_v1.txt")
    assert load_route(os.path.normpath(path)) == [LEFT, RIGHT]


def test_a_missing_file_raises(tmp_path):
    with pytest.raises(ValueError) as excinfo:
        load_route(str(tmp_path / "nope.txt"))
    assert "cannot read" in str(excinfo.value)


def test_a_file_with_only_comments_raises(tmp_path):
    """An empty route is an unfinished edit, not a statement that the track has
    no junctions. drive_corridor.py is the tool for a track with none."""
    path = tmp_path / "empty.txt"
    path.write_text("# nothing yet\n\n", encoding="utf-8")
    with pytest.raises(ValueError) as excinfo:
        load_route(str(path))
    assert "no turns" in str(excinfo.value)


# --- the cursor ---------------------------------------------------------

def test_the_cursor_starts_on_the_first_turn():
    cursor = RouteCursor([LEFT, RIGHT])
    assert cursor.current == LEFT
    assert cursor.remaining == 2
    assert not cursor.exhausted


def test_advancing_returns_the_turn_it_consumed():
    cursor = RouteCursor([LEFT, RIGHT])
    assert cursor.advance() == LEFT
    assert cursor.current == RIGHT


def test_the_cursor_is_spent_after_the_last_turn():
    cursor = RouteCursor([LEFT])
    cursor.advance()
    assert cursor.exhausted
    assert cursor.current is None
    assert cursor.remaining == 0


def test_advancing_past_the_end_is_harmless():
    """topo_state fires advance() on a confirmed gate pass. If the car sees one
    more red triple than the route knows about, the index must not run away."""
    cursor = RouteCursor([LEFT])
    cursor.advance()
    assert cursor.advance() is None
    assert cursor.advance() is None
    assert cursor.index == 1


def test_the_cursor_does_not_alias_the_list_it_was_given():
    turns = [LEFT, RIGHT]
    cursor = RouteCursor(turns)
    turns.append(LEFT)
    assert cursor.remaining == 2
