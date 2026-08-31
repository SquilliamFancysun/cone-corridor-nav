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
import sys

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


def gaps_of(rows):
    out = []
    for row in rows:
        text = row.get("gate_gaps_m") or ""
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
        print("    ..... the triple was NEVER recovered. This is a layout")
        print("          problem, not a control one -- check the gate gaps and")
        print("          that no boundary cone is within 0.4 m of a red.")

    pairs = gaps_of(live)
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
        print("    ..... no gap readings -- the triple was never recovered")

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
