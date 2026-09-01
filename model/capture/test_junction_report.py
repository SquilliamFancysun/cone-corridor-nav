"""The stage-3 diagnosis, which is the only thing that reads a failed run.

`junction_report.py` is what docs/junction-bringup.md points at when the
junction is not detected, so a wrong message here does not merely mislead -- it
sends someone out to re-lay a track that is fine. Each test below pins one
cause to the one instruction that fixes it.
"""

import json

import junction_report
from cone_nav.topology import gate_detect


def rows_with(reason, reds_seen, reds_in_view, gaps="1.35/1.35", ticks=20):
    """A run in which every tick failed the same way."""
    return [{"t": i * 0.1, "gate_live": False, "gate_reason": reason,
             "reds_seen": reds_seen, "reds_in_view": reds_in_view,
             "red_gaps_m": gaps, "topo_state": "follow"}
            for i in range(ticks)]


def diagnosis(rows):
    best = max(r.get("reds_seen", 0) for r in rows)
    in_view = max(r.get("reds_in_view", 0) for r in rows)
    reasons = {}
    for row in rows:
        reasons[row["gate_reason"]] = reasons.get(row["gate_reason"], 0) + 1
    dominant = max(reasons, key=lambda k: reasons[k])
    return junction_report._diagnose_reds(rows, best, in_view, dominant)


def test_a_car_too_far_back_is_not_blamed_on_the_track():
    """The failure this exists for. `reds_seen` counts SLANT range, so three
    cones in plain view log as one, and the old count-based message sent you off
    to measure the clear band around a gate that was laid correctly."""
    text = diagnosis(rows_with(gate_detect.DISTANCE, reds_seen=1, reds_in_view=3))
    assert "too" in text and "far back" in text
    assert "2.68" in text and "2.12" in text
    assert "clear band" not in text


def test_a_fourth_red_is_named_as_a_detector_fault():
    """gate_detect wants EXACTLY three, so a misread orange makes the junction
    undetectable in a way that measuring the gaps cannot explain."""
    text = diagnosis(rows_with(gate_detect.EXTRA, reds_seen=4, reds_in_view=4))
    assert "EXACTLY three" in text
    assert "detect_view.py" in text
    assert "gaps" not in text


def test_no_reds_at_all_points_at_the_detector_not_the_layout():
    text = diagnosis(rows_with(gate_detect.NO_REDS, 0, 0, gaps=""))
    assert "ZERO reds" in text
    assert "detect_view.py" in text


def test_a_gap_failure_quotes_the_car_s_own_measurement():
    """The point of measuring gaps on ticks that did NOT arm: on a run being
    diagnosed there are no ticks that did."""
    text = diagnosis(rows_with(gate_detect.GAPS, 3, 3, gaps="2.90/2.90"))
    assert "gaps" in text
    assert "2.90/2.90" in text


def test_a_missing_red_still_points_at_the_clear_band():
    text = diagnosis(rows_with(gate_detect.CROWDED, 2, 2, gaps=""))
    assert "clear band" in text


def test_an_old_log_without_the_reason_field_still_diagnoses():
    """Logs written before gate_reason existed fall back to the counts. The
    tool has to keep reading them -- the runs already on disk are the ones worth
    comparing a new run against."""
    text = junction_report._diagnose_reds(
        rows_with("", 1, 3), best=1, in_view=3, reason="")
    assert "far back" in text


def test_the_gaps_are_read_from_ticks_that_never_armed(tmp_path, capsys):
    """End to end: a run that recovered no triple still reports the tape work,
    because `red_gaps_m` is measured whether or not the gate armed."""
    path = tmp_path / "run.jsonl"
    path.write_text("".join(
        json.dumps(r) + "\n"
        for r in rows_with(gate_detect.DISTANCE, 1, 3, gaps="1.36/1.34")))

    assert junction_report.report(junction_report.load(str(path)), str(path)) == 0
    out = capsys.readouterr().out
    assert "the car is standing too far back" in out
    assert "1.36" in out or "1.35" in out
    assert "no gap readings" not in out


def test_a_clean_run_is_not_diagnosed_at_all(tmp_path, capsys):
    """A run that saw the junction must not print a failure message; the
    diagnosis block is for runs with no live tick."""
    path = tmp_path / "good.jsonl"
    rows = [{"t": i * 0.1, "gate_live": True, "gate_reason": "",
             "reds_seen": 3, "reds_in_view": 3, "gate_gaps_m": "1.35/1.35",
             "red_gaps_m": "1.35/1.35", "gate_range_m": 2.4,
             "topo_state": "approach", "branch_cones_dropped": 4}
            for i in range(10)]
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))

    junction_report.report(junction_report.load(str(path)), str(path))
    out = capsys.readouterr().out
    assert "too far back" not in out
    assert "ZERO reds" not in out
    assert "whole triples recovered" in out


def test_the_expectations_scale_with_the_gap_the_track_was_laid_with(tmp_path, capsys):
    """The v2 sheet's 1.35 m gaps assume lidar reach this car does not have;
    a re-laid 1.10 m gate must not drown stage 3 in false CHECKs."""
    path = tmp_path / "narrow.jsonl"
    rows = [{"t": i * 0.1, "gate_live": True, "gate_reason": "",
             "reds_seen": 3, "reds_in_view": 3, "gate_gaps_m": "1.10/1.10",
             "red_gaps_m": "1.10/1.10", "gate_range_m": 2.0,
             "topo_state": "approach", "branch_cones_dropped": 2}
            for i in range(10)]
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))

    junction_report.main([str(path), "--expect-gap", "1.10"])
    out = capsys.readouterr().out
    assert "expect 1.10 +-0.05" in out
    assert "OK    left  gap  1.10" in out
    # And the window hints follow the gap: 1.10/tan(32.5) = 1.73 near.
    assert "expect ~1.73" in out


def test_the_too_far_back_message_names_the_laid_gap():
    rows = rows_with(gate_detect.DISTANCE, 1, 3, gaps="1.10/1.10")
    text = junction_report._diagnose_reds(rows, 1, 3, gate_detect.DISTANCE,
                                          gap_m=1.10)
    assert "1.10 m off the axis" in text
    assert "2.79" in text          # sqrt(3.0^2 - 1.10^2)
