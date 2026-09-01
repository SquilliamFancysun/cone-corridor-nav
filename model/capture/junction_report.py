"""Read a drive_junction.py trial log and answer the stage-3 questions.

    python junction_report.py junction-see.jsonl

`docs/junction-bringup.md` stage 3 is a table of things to check in the log
before anything is allowed to move. This runs that table. It exists because the
car has no `jq`, and because a check you have to retype at the bench from a
document is a check that gets skipped.

Reports rather than judges wherever it can: the numbers are printed next to what
was expected, and OK / CHECK is a reading aid, not a gate. A run this calls
CHECK may still be fine, and a run it calls OK can still have driven badly.
"""

import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if not os.path.isdir(os.path.join(_HERE, "cone_nav")):
    _REPO = os.path.normpath(os.path.join(_HERE, "..", ".."))
    for _pkg in ("cone_perception", "cone_nav"):
        _src = os.path.join(_REPO, "ros2", "src", _pkg)
        if os.path.isdir(_src) and _src not in sys.path:
            sys.path.insert(0, _src)

from cone_nav.topology import gate_detect

# From data/layouts/junction_v2.md and the sim, for comparison only.
EXPECT_LIVE_TICKS = 4          # 4.2 at 1.2 m/s; a pushed car should beat it
EXPECT_GAP_M = 1.35
GAP_TOLERANCE_M = 0.05
EXPECT_FIRST_SEEN_M = 2.60
EXPECT_LAST_SEEN_M = 2.10


def load(path):
    rows = []
    with open(path, encoding="utf-8") as handle:
        for number, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except ValueError:
                # A run killed mid-write leaves a partial last line. That is
                # the run worth reading, so keep what parsed.
                print(f"note: line {number} is truncated; ignoring it",
                      file=sys.stderr)
    return rows


def mark(ok):
    return "OK   " if ok else "CHECK"


def _diagnose_reds(rows, best, in_view, reason=""):
    """Why the triple never armed, and what to go and do about it.

    Keyed on `gate_reason` where the log carries one, because that string was
    produced by the same `gate_detect.survey` call that made the decision, and a
    diagnosis re-derived here from counts is a second opinion that can disagree
    with the first. The count-based branches below are the fallback for logs
    written before that field existed.

    `best` is the most reds ever inside gate_detect's arm range; `in_view` is
    the most ever detected at ANY range. The two differing is the diagnosis
    easiest to mistake for a track fault -- the arm range is a SLANT range, so a
    car 2.75 m from the red line has its outer reds at 3.06 m and counts one,
    while three sit in plain view.
    """
    if in_view == 0 or reason == gate_detect.NO_REDS:
        return ("          ZERO reds at any point. The detector is not calling\n"
                "          red at all -- wrong weights, or the cones read as\n"
                "          orange. Check with detect_view.py before touching\n"
                "          the track.")
    if reason == gate_detect.EXTRA or best > 3:
        return (f"          {max(best, 4)} reds in range at once, and gate_detect\n"
                "          wants EXACTLY three. Something else is being called\n"
                "          red -- most likely the orange dead-end cone, which the\n"
                "          v1 report confused 6% of the time. Check detect_view.py.")
    if reason == gate_detect.DISTANCE or (in_view >= 3 > best):
        return ("          All three reds WERE detected, but never all three\n"
                "          inside the 3.0 m arm range at once. The car is too\n"
                "          far back: the outer reds are 1.35 m off the axis, so\n"
                "          all three are in range only within 2.68 m of the red\n"
                "          line, and the camera loses them past 2.12 m. Stand\n"
                "          the car between those two and try again.")
    if reason == gate_detect.GAPS:
        gaps = _worst_gaps(rows)
        measured = f" Measured {gaps[0]:.2f}/{gaps[1]:.2f}." if gaps else ""
        return ("          All three were seen together in range, but the gaps\n"
                "          failed gate_detect's window. Both must be inside\n"
                f"          0.6-2.5 m, and the span must exceed 2.5 m.{measured}")
    if reason == gate_detect.CROWDED or best < 3:
        return ("          Reds ARE detected but never all three at once. One\n"
                "          cone is merging with a neighbour in the lidar or\n"
                "          leaving frame early: check the 0.4 m clear band\n"
                "          around each red, and that the gaps are not > 1.40 m.")
    return ("          Three reds in range with good gaps, and still no gate.\n"
            "          That is not a detection failure -- read the topo_state\n"
            "          sequence below, and check the route file was loaded.")


def _worst_gaps(rows):
    """The gap pair furthest from the design value, for the message above.

    `red_gaps_m` is measured whether or not the triple armed, which is what
    makes it usable here -- `gate_gaps_m` exists only on ticks that succeeded,
    and on a run being diagnosed there are none.
    """
    pairs = []
    for row in rows:
        text = row.get("red_gaps_m") or ""
        try:
            pairs.append(tuple(float(v) for v in text.split("/")))
        except ValueError:
            continue
    pairs = [p for p in pairs if len(p) == 2]
    if not pairs:
        return None
    return max(pairs, key=lambda p: max(abs(v - EXPECT_GAP_M) for v in p))


def gaps_of(rows, field="gate_gaps_m"):
    out = []
    for row in rows:
        text = row.get(field) or ""
        try:
            left, right = (float(v) for v in text.split("/"))
        except ValueError:
            continue
        out.append((left, right))
    return out


def transitions(rows):
    out = []
    previous = None
    for row in rows:
        state = row.get("topo_state", "")
        if state != previous:
            out.append((row.get("t", 0.0), previous, state,
                        row.get("gate_range_m", 0.0)))
            previous = state
    return out


def report(rows, path):
    if not rows:
        print(f"{path}: no ticks. The run wrote nothing.")
        return 1

    live = [r for r in rows if r.get("gate_live")]
    traverse = [r for r in rows if r.get("topo_state") == "traverse"]
    duration = rows[-1].get("t", 0.0)
    hz = len(rows) / duration if duration else 0.0

    print(f"{path}: {len(rows)} ticks over {duration:.1f} s ({hz:.1f} Hz)")
    print()

    print("  the junction was seen")
    print(f"    {mark(len(live) >= EXPECT_LIVE_TICKS)} whole triples recovered "
          f"{len(live):>4} ticks      expect >= {EXPECT_LIVE_TICKS}")
    if live:
        first, last = live[0], live[-1]
        print(f"    ..... first seen at gate {first.get('gate_range_m', 0):.2f} m"
              f"          expect ~{EXPECT_FIRST_SEEN_M:.2f}")
        print(f"    ..... last  seen at gate {last.get('gate_range_m', 0):.2f} m"
              f"          expect ~{EXPECT_LAST_SEEN_M:.2f}")
    else:
        print("    ..... the triple was NEVER recovered.")
    if not live or any("reds_seen" in r for r in rows):
        best = max((r.get("reds_seen", 0) for r in rows), default=0)
        in_view = max((r.get("reds_in_view", 0) for r in rows), default=0)
        held = sum(1 for r in rows if r.get("reds_seen", 0) == 3)
        print(f"    ..... most reds in arm range at once {best}, on "
              f"{held} tick(s) exactly three")
        if in_view:
            print(f"    ..... most reds detected at ANY range {in_view}"
                  + ("        <-- the car is standing too far back"
                     if in_view > best else ""))
        # Why gate_detect declined, straight from the same code path that
        # declined. Counting them ranks the causes rather than reporting
        # whichever happened to be last.
        reasons = {}
        for row in rows:
            if row.get("gate_live"):
                continue
            reason = row.get("gate_reason") or ""
            if reason:
                reasons[reason] = reasons.get(reason, 0) + 1
        ranked = sorted(reasons.items(), key=lambda kv: -kv[1])
        for reason, count in ranked[:3]:
            print(f"    ..... {count:>4} ticks  {reason}")
        if not live:
            print(_diagnose_reds(rows, best, in_view,
                                 ranked[0][0] if ranked else ""))

    # Prefer the ticks that actually armed, but fall back to every tick that
    # measured three reds. The tape work is worth reporting on a run that
    # recovered no triple at all -- on those runs it is the number that says
    # whether to go and re-lay the gate or to go and stand somewhere else.
    pairs = gaps_of(live, "gate_gaps_m") or gaps_of(rows, "red_gaps_m")
    print()
    print("  the car's own measurement of your tape work")
    if pairs:
        for index, label in ((0, "left  gap"), (1, "right gap")):
            values = [p[index] for p in pairs]
            mean = sum(values) / len(values)
            ok = abs(mean - EXPECT_GAP_M) <= GAP_TOLERANCE_M
            print(f"    {mark(ok)} {label} {mean:>5.2f} m  "
                  f"(min {min(values):.2f}, max {max(values):.2f})   "
                  f"expect {EXPECT_GAP_M:.2f} +-{GAP_TOLERANCE_M:.2f}")
    else:
        print("    ..... no gap readings -- three reds were never in view "
              "at once")

    print()
    print("  the manoeuvre")
    moves = transitions(rows)
    for when, was, now, gate in moves:
        if was is None:
            continue
        print(f"    {when:>6.2f}s  {was} -> {now}"
              + (f"   gate {gate:.2f} m" if gate else ""))
    passes = [r for r in rows if r.get("topo_note") == "passed"]
    timeouts = [r for r in rows if "timed out" in (r.get("topo_note") or "")]
    traverses = sum(1 for _t, was, now, _g in moves if now == "traverse")
    print(f"    {mark(traverses == 1)} entered the manoeuvre {traverses} time(s)"
          "        expect 1 per junction")
    print(f"    {mark(len(passes) == 1)} confirmed passes    {len(passes)}"
          "                  expect 1 per junction")
    if timeouts:
        print("    CHECK traverse TIMED OUT -- the corridor never came back on")
        print("          the far side, or DUTY_TO_MPS is badly off for this car")

    print()
    print("  the branch filter")
    dropped = max((r.get("branch_cones_dropped", 0) for r in traverse),
                  default=0)
    print(f"    {mark(dropped > 0)} cones dropped, peak {dropped:>3}"
          "              expect > 0, else the filter never bit")
    if traverse:
        reach = min(r.get("reach_m", 0.0) for r in traverse)
        print(f"    {mark(reach > 1.0)} min reach through the mouth "
              f"{reach:>5.2f} m   expect > 1.00 (the stop floor)")
        fallback = sum(1 for r in traverse if r.get("single_boundary_fallback"))
        print(f"    ..... single-boundary ticks {fallback:>3} of {len(traverse)}"
              "          sim gets 0")

    reasons = {}
    for row in rows:
        reason = row.get("stop_reason") or ""
        if reason:
            reasons[reason] = reasons.get(reason, 0) + 1
    if reasons:
        print()
        print("  ticks the car would not have moved on")
        for reason, count in sorted(reasons.items(), key=lambda kv: -kv[1]):
            print(f"    {count:>4}  {reason}")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Stage-3 checks over a drive_junction.py trial log.")
    parser.add_argument("log", help="path to the JSONL written by --log")
    args = parser.parse_args(argv)
    return report(load(args.log), args.log)


if __name__ == "__main__":
    sys.exit(main())
