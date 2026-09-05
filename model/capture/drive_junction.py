"""Autonomous junction navigation. Runs on the car, and drives it.

    python drive_junction.py --weights ~/models/best.pt --route route_v1.txt --dry-run
    python drive_junction.py --weights ~/models/best.pt --route route_v1.txt --steer-only
    python drive_junction.py --weights ~/models/best.pt --route route_v1.txt

`drive_corridor.py` with a route in its hand. The perception half, the control
half, the VESC driver, the deadman, the trial log and every argument are
imported from it rather than restated, so there is one definition of each. What
this file adds is three lines in the middle of the pipeline:

    junction = gate_detect.survey(cones).junction  # a red triple in range?
    cones    = junction_exec.keep_branch(...)     # drop the other branch
    line     = junction_exec.junction_line(...)   # aim at the gate

...and three more for the goal, which is the same idea again:

    goal = goal_detect.survey(cones).goal         # a magenta in range?
    goal_latch.update(goal, route_spent, ...)     # seeking / run-in / stopped
    line = junction_exec.junction_line(line, ...) # aim at it, same helper

Everything downstream of that -- `centerline`, `pure_pursuit`, `speed_ctrl`,
`VescDriver` -- is byte-identical to plain corridor following. Neither a junction
nor a goal is a second control stack; both are a cone list and an anchor.

## What is copied, and why that is a cost

`main()` is a near-copy of `drive_corridor.main()`, and that duplication is
deliberate but not free: every safety path in it -- deadman released, scan gone
stale, exception, Ctrl-C, `finally` -- now exists twice, and a fix to one does
not reach the other. It is done this way because `drive_corridor.py` works and
is the fallback if this does not; threading junction state through its loop
would put unproven state in the one program known to drive the car. **If both
scripts are still here once this one has run the track, merge them.**

## Read before running

`data/layouts/junction_v2.md` is the geometry this expects, and the tolerances
in it are real: a triple laid 1.5 m apart instead of 1.35 m is visible for two
ticks instead of five, and boundary cones crowded within 0.4 m of a red make the
junction undetectable at any range.
"""

import math
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if not os.path.isdir(os.path.join(_HERE, "cone_perception")):
    _REPO = os.path.normpath(os.path.join(_HERE, "..", ".."))
    _src = os.path.join(_REPO, "src")
    if os.path.isdir(_src) and _src not in sys.path:
        sys.path.insert(0, _src)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import audio_playback
import detectors
import drive_corridor
import fusion_view
import oakd
from cone_nav.control import pure_pursuit, reverse_ctrl, speed_ctrl
from cone_nav.corridor.centerline import centerline
from cone_nav.corridor import side_assign
from cone_nav.corridor.side_assign import fill_unlabeled, heading_of
from cone_nav.corridor.boundary_split import split
from cone_nav.guidance import backout as backout_mod
from cone_nav.guidance import goal_stop, junction_exec, planner
from cone_nav.guidance.explore import ExplorePolicy
from cone_nav.guidance.route_exec import RouteCursor, load_route
from cone_nav.topology import (dead_end, gate_detect, goal_detect,
                               graph_builder, topo_state)
from cone_perception import (clustering, ego_motion, extrinsics, fusion,
                             geometry,
                             label_memory, odometry)
from drive_corridor import (
    DEFAULT_PORT_WS,
    MAX_SCAN_AGE_S,
    Deadman,
    ThreadedDetector,
    TrialLog,
    VescDriver,
    build_parser,
    finalise_args,
    require_vehicle_geometry,
)
from lidar_view import BAUD, open_serial

# The corridor schema plus what the state machine adds. Spelled as a merge so a
# field added to DriveStatus for the corridor shows up here too.
JUNCTION_STATUS_SCHEMA = {
    "type": "object",
    "title": "JunctionStatus",
    "properties": dict(
        drive_corridor.DRIVE_STATUS_SCHEMA["properties"],
        topo_state={"type": "string", "description": "follow / approach / traverse"},
        turn={"type": "string", "description": "the turn being executed"},
        route_index={"type": "integer", "description": "junctions consumed"},
        route_remaining={"type": "integer"},
        gate_live={"type": "boolean", "description": "a whole triple seen THIS tick"},
        reds_seen={"type": "integer", "description": "red cones in arm range, whether or not they formed a triple"},
        reds_in_view={"type": "integer", "description": "red cones at ANY range. Above reds_seen means the car is too far back"},
        reds_m={"type": "string", "description": "range to every red, nearest first"},
        red_gaps_m={"type": "string", "description": "the two gaps between three reds, measured whether or not they armed"},
        gate_reason={"type": "string", "description": "why no triple this tick"},
        labeled_by_memory={"type": "integer", "description": "reds restored from a tracked cluster's remembered label"},
        gate_range_m={"type": "number", "description": "to the chosen gate midpoint"},
        gate_gaps_m={"type": "string", "description": "the two measured gate widths"},
        branch_cones_dropped={"type": "integer"},
        blind_ticks={"type": "integer", "description": "ticks since the last triple"},
        travelled_m={"type": "number", "description": "since commit"},
        odo_forward_m={"type": "number", "description": "this tick's scan-matched travel; the dry run's odometry"},
        odo_pairs={"type": "integer", "description": "cones the odometry step was fitted on; 0 = no measurement"},
        odo_lateral_m={"type": "number", "description": "this tick's scan-matched sideways travel"},
        odo_yaw_deg={"type": "number", "description": "this tick's scan-matched turn, left positive"},
        pose_x={"type": "number", "description": "integrated position in the frame the run started in. A random walk -- nothing steers by it"},
        pose_y={"type": "number"},
        pose_yaw_deg={"type": "number"},
        pose_measured={"type": "integer", "description": "ticks the pose was advanced by a real measurement. Below `t`*rate means the run has blind stretches in it"},
        pose_jumps={"type": "integer", "description": "declared lifts. Anything measured across one is in a different frame and comes back unmeasured"},
        boxes={"type": "string", "description": "this tick's CAMERA boxes as class,bearing_deg,confidence;... -- the inputs fusion chose between. cones_xy records what it decided; without these a cluster that came back the wrong colour cannot be diagnosed"},
        cones_xy={"type": "string", "description": "this tick's cones in base_link as x,y,class;... -- what analysis/map_from_log.py turns into a map. Pre-fill and pre-branch-filter, the same list the odometry is fitted on"},
        wall_state={"type": "string", "description": "the orange run-in: seeking / run_in / stopped. A dead end named by ARRIVAL rather than by the corridor running out"},
        wall_range_m={"type": "number", "description": "to the wall cone, measured if seen this tick else carried"},
        wall_reason={"type": "string", "description": "why no wall cone was accepted -- the same reasons goal_reason gives, aimed at orange"},
        dead_end_state={"type": "string", "description": "clear / dead_end"},
        dead_end_reason={"type": "string", "description": "why the corridor is or is not judged to have ended. The first field to read when a backtrack fires or fails to"},
        dead_end_reach_m={"type": "number", "description": "reach of the UNANCHORED corridor line, which is what the decision is made on"},
        cursor={"type": "string", "description": "route / explore -- what decided the turns. route_index and route_remaining mean different things in each"},
        explore_path={"type": "string", "description": "turns taken to reach where the car is -- the maze node's identity"},
        maze_nodes={"type": "integer"},
        maze_edges={"type": "integer"},
        maze_dead_ends={"type": "integer"},
        backout_state={"type": "string", "description": "idle / backing / arrived / abandoned -- the reverse out of a dead end. Empty when --reverse is off"},
        backout_reason={"type": "string", "description": "what the manoeuvre is doing or why it stopped. The first field to read when a run ends somewhere unexpected"},
        backout_travelled_m={"type": "number", "description": "distance reversed so far, unsigned"},
        backout_bound_m={"type": "number", "description": "the distance bound. Reaching it is an abandon, never a healthy ending"},
        backout_gate_m={"type": "number", "description": "range to the gate midpoint this tick, once a whole triple is back in view. Where it stopped, against the band it had to stop inside"},
        backout_heading_err_deg={"type": "number", "description": "corridor axis in the car frame, left positive -- NOT the car's heading. The reverse law's h"},
        backout_cross_track_m={"type": "number", "description": "car's offset from the centreline, left positive. The reverse law's y; a drifting reverse is diagnosed from which of the two grew"},
        backout_blind_ticks={"type": "integer", "description": "consecutive ticks with no corridor to steer on. The manoeuvre holds its last command briefly and then stops"},
        topo_note={"type": "string"},
        goal_state={"type": "string", "description": "seeking / run_in / stopped"},
        goal_range_m={"type": "number", "description": "to the goal, measured if seen this tick else carried"},
        goal_reason={"type": "string", "description": "why no goal was accepted this tick"},
        goal_offset_m={"type": "number", "description": "the candidate's offset from the corridor axis"},
        goal_bearing_deg={"type": "number", "description": "bearing to the nearest magenta in the CAR frame, left positive. Disagreeing with goal_offset_m is the signature of a bad axis"},
        magenta_in_view={"type": "integer", "description": "magenta cones at ANY range"},
        goal_armed={"type": "boolean", "description": "the route is spent, so a goal may stop the car"},
        goal_blind_ticks={"type": "integer", "description": "ticks the goal has been carried rather than seen"},
        goal_hops={"type": "integer", "description": "sightings refused for being too far from the tracked goal -- the label on a different object"},
        goal_note={"type": "string"},
    ),
}

JUNCTION_SCHEMA = {
    "type": "object",
    "title": "Junction",
    "properties": {
        "reds": {"type": "string"},
        "left_gate": {"type": "string"},
        "right_gate": {"type": "string"},
        "chosen": {"type": "string"},
    },
}


def drive_pipeline(scan, detection_set, calibration, intr, args, now,
                   axis_rad=0.0, topo=None, previous_line=None,
                   travel_m=0.0, yaw_delta_rad=0.0, red_memory=None,
                   goal_latch=None, goal_armed=False, dead_end_latch=None,
                   backing_out=False, wall_latch=None):
    """One revolution -> cones, centerline, steering, duty, plus the manoeuvre.

    Identical to `drive_corridor.drive_pipeline` up to the fill and after the
    centerline. The junction block in the middle is the whole difference.
    """
    candidates = clustering.cone_candidates(scan, calibration,
                                            max_range_m=args.max_range)
    age = detection_set.age(now) if detection_set is not None else float("inf")
    detections = detection_set.detections if detection_set is not None else []
    result = fusion.associate(candidates, detections, intr,
                              detection_age_s=age,
                              max_bearing_err_deg=args.bearing_gate,
                              max_detection_age_s=args.max_detection_age)

    # A red the camera vouched for recently stays red while its tracked
    # cluster stands where it stood -- position lidar-fresh, colour remembered,
    # everything expiring. This is what turns 23 flickering sightings in a
    # window into a triple that HOLDS through detector misses and frame exits.
    cones, remembered = ((red_memory.apply(result.cones, now))
                         if red_memory is not None else (result.cones, 0))
    filled = 0
    # Surveyed rather than merely detected, because "saw two reds and one merged
    # into a branch cone", "saw no red at all" and "saw all three from a metre
    # too far back" are different problems with different fixes, and a log that
    # only records whole triples cannot tell them apart. `survey.reason` is the
    # first field to read when stage 3 finds nothing.
    survey = gate_detect.survey(cones, axis_rad=axis_rad)
    junction = survey.junction
    if topo is not None:
        if backing_out:
            # Reversing takes the car back THROUGH the junction, which rises
            # into view ahead of a car pointing the wrong way down it and
            # moving away from it. Left running, `_follow` would arm an
            # approach on that sighting and `_approach` would commit a traverse
            # on the very branch the search is backing out to try. The survey
            # still runs above -- the manoeuvre's own ending is read off it.
            topo.hold("backing out")
        else:
            topo.update(junction, previous_line, travel_m=travel_m,
                        yaw_delta_rad=yaw_delta_rad)

    # Read off the same PRE-FILL, pre-branch-filter list the reds are. The fill
    # only ever paints blue and yellow so it cannot invent a goal, but
    # `keep_branch` can DELETE one, and a goal read after it would silently
    # depend on which way the route happened to turn.
    # NOT `axis_rad`. It is a one-tick feedback that holds its last value when
    # the centerline dies, and at the goal the centerline always dies -- which on
    # 2026-09-01 froze it 50-64 deg wrong and refused 26 ticks of a clean
    # approach. See goal_detect.trusted_axis.
    goal_survey = goal_detect.survey(
        cones, axis_rad=goal_detect.trusted_axis(previous_line, axis_rad))
    if goal_latch is not None:
        goal_latch.update(goal_survey.goal, goal_armed, travel_m=travel_m,
                          yaw_delta_rad=yaw_delta_rad)

    engaged = topo is not None and topo.engaged
    if not args.no_fill:
        # Full fill range everywhere, with a +-GATE_BAND_M mask along the gate
        # line instead of the old radius shrink. The shrink protected the reds
        # by starving the corridor -- the out-of-frame rows between 1.0 and
        # 1.18 m went unlabelled for the whole engaged period and the mouth
        # line thinned exactly where it was needed. The band excludes every
        # red (the build rules keep 0.75 m clear either side of the red line)
        # and nothing else, at any gap width. Sourced from the carried divider
        # while engaged -- it survives the reds leaving frame -- else from the
        # nearest labelled red.
        line_mask = side_assign.gate_line_of(
            topo.divider_xy if engaged else None,
            topo.axis_rad if engaged else axis_rad,
            survey.reds, axis_rad)
        cones, filled = fill_unlabeled(
            cones, reference_heading_rad=axis_rad,
            fill_in_fov=args.no_camera, gate_line=line_mask)

    dropped = 0
    if engaged and topo.junction is not None:
        junction_exec.select(topo.junction, topo.turn)
        # The divider and axis come from the machine, not from the latched
        # junction: through the blind period those are carried forward with the
        # car's motion, and the latched pair are stale by metres.
        cones, dropped = junction_exec.keep_branch(
            cones, topo.divider_xy, topo.axis_rad, topo.turn)

    corridor_line = centerline(cones, car_xy=(0.0, 0.0))
    line = corridor_line
    # The anchor comes from `topo`, not from `select()`, because `topo` is what
    # carries it forward on measured motion. `select()` reads whichever junction
    # object is current -- live or latched -- and a latched one is frozen in a
    # frame the car has driven out of.
    if topo is not None and topo.anchor_ok:
        line = junction_exec.junction_line(corridor_line, topo.anchor_xy)
    # The wall gets the same anchor the trophy does, and for the same reason:
    # an orange forms no midpoint, so without one the driven line stops at the
    # last boundary pair and the cone the car is closing on is not on its path.
    if wall_latch is not None and wall_latch.anchor_ok:
        line = junction_exec.junction_line(line, wall_latch.goal_xy)
    if goal_latch is not None and goal_latch.anchor_ok:
        # The same helper as the gate anchor, for the same reason: a magenta
        # cone forms no midpoints, so without this the line stops at the last
        # cone row and the goal is not on the driven path at all.
        line = junction_exec.junction_line(line, goal_latch.goal_xy)

    axle = extrinsics.REAR_AXLE_IN_BASE
    pursuit = pure_pursuit.steering_angle(line.points, args.lookahead,
                                          extrinsics.WHEELBASE_M, origin=axle)
    # Both of the speed law's refusals stand down for the goal run-in, and for
    # nothing else. Left in place they halt the car ~0.64 m from the trophy --
    # before any stop range can fire, and unrecoverably, because the scan does
    # not change while the car stands still. See cone_nav/guidance/goal_stop.py.
    # Either run-in relaxes the speed law. Both are a confirmed, lidar-ranged
    # point being closed on with no corridor left to commit to, which is the
    # whole of `goal_stop`'s argument for standing the floor down.
    run_in = ((goal_latch is not None and goal_latch.run_in)
              or (wall_latch is not None and wall_latch.run_in))
    # The reach floor stands down in exactly two places, and this is the second.
    #
    # Inside a TRAVERSE the car has already committed: the gate is latched, the
    # turn is chosen, and `keep_branch` is cutting the other fork away. The
    # floor's own rationale -- do not commit to a corridor you can see one
    # metre of -- does not describe that situation, which is the identical
    # argument `goal_stop` makes for the run-in.
    #
    # What it costs to leave in place is not a slower crossing, it is a
    # deadlock. Measured 2026-09-02 (`explore-3.jsonl`): reach sat at 0.70 m
    # through a mouth, duty went to zero, and because travel is now MEASURED by
    # scan matching a stopped car accrues none -- so `travelled_m` never
    # cleared the pass floor and the traverse timed out 20 s later, keeping a
    # route entry the car had physically driven through. A stopped car cannot
    # recover on its own here: the scan does not change while it stands still.
    #
    # `min_points` stays at 2. At the goal the line legitimately shrinks to the
    # anchor alone; a one-point line in a junction mouth is a car that is
    # confused, not one that has arrived. "no steerable target" and "centerline
    # too short" both still stop the car, so what changes is only that a SHORT
    # BUT REAL corridor keeps it crawling at the cogging floor.
    in_mouth = topo is not None and topo.state == topo_state.TRAVERSE
    duty = speed_ctrl.duty(pursuit, line, max_duty=args.max_duty, origin=axle,
                           min_reach_m=(0.0 if run_in or in_mouth
                                        else speed_ctrl.MIN_REACH_M),
                           min_points=1 if run_in else 2)
    # The wall, approached the way the trophy is. Neither `goal_detect.survey`
    # nor `GoalLatch` is about magenta -- one takes the bucket as an argument
    # now, and the other only ever tracked a point. What they give a dead end is
    # a POSITIVE signal with a known stop distance, in place of inferring a wall
    # from a corridor that ran out.
    #
    # It does NOT replace the geometric latch below. Orange is the weakest class
    # in the dataset -- 0.687 recall on v3 -- and a wall whose cone is missed
    # must still be named, or the operator gets a stopped car that decided
    # nothing: no dead_end(), no map entry, no banner. Measured across four
    # runs, the geometric signal fired 9-20 of 20 ticks at a wall and 0 of 20 in
    # a corridor, which is a sound thing to fall back on.
    wall_survey = None
    if wall_latch is not None:
        wall_survey = goal_detect.survey(
            cones, axis_rad=goal_detect.trusted_axis(previous_line, axis_rad),
            candidates=split(cones).dead_ends)
        wall_latch.update(wall_survey.goal,
                          armed=not engaged and not backing_out,
                          travel_m=travel_m, yaw_delta_rad=yaw_delta_rad)

    # `--wall-arrival-only` drops the geometric path and leaves the
    # arrival path alone: `wall_latch` still runs above and still calls
    # `dead_end_latch.force()`, so the latch object stays live and
    # everything downstream is unchanged. Skipping `update` rather than
    # passing `armed=False` because a held-down latch still counts
    # travel and still writes a reason, and this is meant to be off.
    if dead_end_latch is not None and not args.wall_arrival_only:
        # On the UNANCHORED line: an anchor is a point threaded onto the driven
        # line, so judging reach from `line` would credit the corridor with a
        # gate or a trophy it does not contain. Held down wherever the corridor
        # is ALLOWED to end -- through a mouth, and over the goal run-in. See
        # cone_nav/topology/dead_end.py.
        # Armed outside the manoeuvre, and again once the car is past the gate
        # line: see `TopoState.past_gate`. Still held down over the goal
        # run-in, where the course really has ended.
        past = topo is not None and topo.past_gate
        dead_end_latch.update(
            corridor_line, cones, oranges=split(cones).dead_ends,
            armed=(not engaged or past) and not run_in and not backing_out,
            origin=axle, travel_m=travel_m)
    return (result, cones, filled, line, pursuit, duty, corridor_line,
            junction, dropped, survey, remembered, goal_survey, wall_survey)


def status_of(base, topo, junction, dropped, survey=None, goal_survey=None,
              goal_latch=None, goal_armed=False, dead_end_latch=None,
              wall_latch=None, wall_survey=None,
              pose=None, maze=None, backout=None):
    """The corridor status record, plus what the manoeuvre is doing."""
    gaps = ""
    live = topo.live if topo else None
    if live is not None:
        gaps = "%.2f/%.2f" % live.gaps_m
    turn = topo.turn if topo else None
    return dict(
        base,
        topo_state=topo.state if topo else "",
        turn=turn or "",
        route_index=topo.cursor.index if topo else 0,
        route_remaining=topo.cursor.remaining if topo else 0,
        gate_live=live is not None,
        reds_seen=len(survey.in_arm) if survey else 0,
        reds_in_view=len(survey.reds) if survey else 0,
        reds_m=("/".join("%.2f" % r for r in survey.ranges_m)
                if survey else ""),
        red_gaps_m=("%.2f/%.2f" % survey.gaps_m
                    if survey and survey.gaps_m else ""),
        gate_reason=survey.reason if survey else "",
        gate_range_m=round(topo.junction.range_for(turn), 3)
        if topo and topo.junction is not None and turn else 0.0,
        gate_gaps_m=gaps,
        branch_cones_dropped=dropped,
        blind_ticks=topo.blind_ticks if topo else 0,
        travelled_m=round(topo.travelled_m, 3) if topo else 0.0,
        topo_note=topo.note if topo else "",
        # The goal, on the same principle as `gate_reason`: a trophy at 3.4 m,
        # a trophy off to one side and no trophy at all are different problems
        # with different fixes, and none of them is visible in `goal_state`.
        goal_state=goal_latch.state if goal_latch else "",
        goal_range_m=(round(goal_latch.range_m, 3)
                      if goal_latch and goal_latch.range_m is not None else 0.0),
        goal_reason=goal_survey.reason if goal_survey else "",
        goal_offset_m=(round(goal_survey.offset_m, 3)
                       if goal_survey and goal_survey.offset_m is not None
                       else 0.0),
        goal_bearing_deg=(round(goal_survey.bearing_deg, 1)
                          if goal_survey and goal_survey.bearing_deg is not None
                          else 0.0),
        magenta_in_view=len(goal_survey.magenta) if goal_survey else 0,
        goal_armed=bool(goal_armed),
        goal_blind_ticks=goal_latch.blind_ticks if goal_latch else 0,
        goal_hops=goal_latch.hops if goal_latch else 0,
        goal_note=goal_latch.note if goal_latch else "",
        # The dead end, on the same principle as `gate_reason` and
        # `goal_reason`: a corridor that reaches too far, a car with nothing in
        # view and a wall are different states, and `dead_end_state` shows none
        # of them.
        wall_state=wall_latch.state if wall_latch else "",
        wall_range_m=(round(wall_latch.range_m, 3)
                      if wall_latch and wall_latch.range_m is not None else 0.0),
        wall_reason=wall_survey.reason if wall_survey else "",
        dead_end_state=dead_end_latch.state if dead_end_latch else "",
        dead_end_reason=dead_end_latch.reason if dead_end_latch else "",
        dead_end_reach_m=(round(dead_end_latch.reach_m, 3)
                          if dead_end_latch else 0.0),
        pose_x=round(pose.x, 3) if pose else 0.0,
        pose_y=round(pose.y, 3) if pose else 0.0,
        pose_yaw_deg=round(pose.yaw_deg, 1) if pose else 0.0,
        pose_measured=pose.measured if pose else 0,
        pose_jumps=pose.jumps if pose else 0,
        cursor=topo.cursor.label if topo else "",
        explore_path="/".join(topo.cursor.path) if topo else "",
        maze_nodes=len(maze.nodes) if maze else 0,
        maze_edges=(sum(len(e) for e in maze.edges.values()) if maze else 0),
        maze_dead_ends=(len(maze.find(graph_builder.DEAD_END))
                        if maze else 0),
        # The manoeuvre. `backout_state` is the field to read first when a run
        # ends somewhere unexpected: `abandoned` with a reason names its own
        # cause, and `arrived` beside a `gate_range_m` says where it stopped
        # against the band it had to stop inside.
        backout_state=backout.state if backout else "",
        backout_reason=backout.reason if backout else "",
        backout_travelled_m=round(backout.travelled_m, 3) if backout else 0.0,
        backout_bound_m=round(backout.bound_m, 3) if backout else 0.0,
        backout_gate_m=round(backout.gate_range_m, 3) if backout else 0.0,
        backout_heading_err_deg=(round(math.degrees(backout.heading_err_rad), 2)
                                 if backout else 0.0),
        backout_cross_track_m=(round(backout.cross_track_m, 3)
                               if backout else 0.0),
        backout_blind_ticks=backout.blind_ticks if backout else 0,
    )


def parse_args(argv=None):
    parser = build_parser(description="Autonomous cone-junction navigation.")
    parser.add_argument("--route",
                        help="path to a route file: one 'left' or 'right' per "
                             "junction, in order. See data/routes/. Mutually "
                             "exclusive with --explore")
    parser.add_argument("--explore", action="store_true",
                        help="decide each junction on the spot instead of "
                             "reading a route: take a branch, and if it dead-"
                             "ends back out and take the other. Builds a map "
                             "as it goes; see --emit-route")
    parser.add_argument("--explore-first", default="left",
                        choices=["left", "right"],
                        help="which branch --explore tries first at a junction "
                             "it has not seen before (default: left)")
    parser.add_argument("--reverse", action="store_true",
                        help="back the car out of a dead end under power "
                             "instead of stopping for someone to carry it. "
                             "Exploring runs only; see docs/junction-bringup.md "
                             "stage 8")
    parser.add_argument("--reverse-only", action="store_true",
                        help="command a steady reverse while X is held and "
                             "nothing else. The bench check that this VESC "
                             "reverses on a negative duty at all -- stage 8a. "
                             "Wheels off the ground the first time")
    parser.add_argument("--max-reverse-duty", type=float,
                        default=speed_ctrl.MAX_REVERSE_DUTY,
                        help="duty magnitude while backing out (default "
                             "%(default)s, the cogging floor). Reverse is the "
                             "direction with no lidar behind it; raise this "
                             "only with a reason")
    parser.add_argument("--emit-route", metavar="PATH",
                        help="on exit, write the route from the start to the "
                             "goal implied by what was explored -- the driven "
                             "path with its dead ends removed. Drive it with "
                             "--route")
    parser.add_argument("--goal-stop", type=float,
                        default=goal_stop.STOP_RANGE_M,
                        help="metres from the lidar -- the front of the car -- "
                             "at which the magenta goal stops the run. The "
                             "floor is clustering.MIN_CONE_RANGE_M (0.20), "
                             "below which the trophy stops being a cluster")
    parser.add_argument("--wall-stop", type=float,
                        default=goal_stop.STOP_RANGE_M,
                        help="metres from the lidar at which the car stops "
                             "short of a dead-end wall it has driven up to. "
                             "Same default as --goal-stop; raise it if the "
                             "car ends up crowded among the wall cones with "
                             "no room to back out")
    parser.add_argument("--wall-arrival-only", action="store_true",
                        help="name a dead end ONLY by driving up to an orange, "
                             "never from the corridor running out. Turns off "
                             "the geometric latch entirely, leaving the same "
                             "machinery the goal uses -- one cone, on the "
                             "corridor axis, in arm range, driven at and "
                             "stopped short of. Costs the walls whose orange "
                             "the detector misses (0.687 recall on v3): those "
                             "stop the car on the reach floor and name "
                             "nothing, so no dead_end(), no map entry, no "
                             "backout. Use it when geometry is firing in open "
                             "corridor and a missed wall is the cheaper fault")
    parser.add_argument("--goal-anywhere", action="store_true",
                        help="arm the goal stop even while the route still has "
                             "turns left. FOR BRING-UP on a corridor with no "
                             "junction; on the track it lets a misread red stop "
                             "the car mid-course")
    parser.add_argument("--drive-audio",
                        default=str(audio_playback.DEFAULT_DRIVE_AUDIO),
                        help="main track latched on by the first X press and "
                             "played until the goal")
    parser.add_argument("--goal-audio",
                        default=str(audio_playback.DEFAULT_GOAL_AUDIO),
                        help="finish clip played once when the goal stops the "
                             "car")
    parser.add_argument("--audio-volume", type=float, default=1.0,
                        help="pw-play stream volume from 0.0 to 1.0. The USB "
                             "speaker's own sink volume still applies")
    parser.add_argument("--audio-target", default=None,
                        help="optional PipeWire sink name. Defaults to the "
                             "system's selected output")
    parser.add_argument("--no-audio", action="store_true",
                        help="run the original driving behavior without "
                             "starting an audio player")
    args = finalise_args(parser, parser.parse_args(argv))

    if args.reverse_only:
        if args.dry_run or args.steer_only:
            parser.error("--reverse-only is a mode: it cannot be combined with "
                         "--dry-run or --steer-only.")
        args.mode = "reverse-only"
    if args.reverse_only and args.reverse:
        parser.error("--reverse-only drives a bare reverse and never reaches a "
                     "junction, so --reverse has nothing to do. Pass one.")
    if args.reverse and not args.explore:
        parser.error("--reverse backs out of a dead end to take the branch the "
                     "search has not tried,\n       which only --explore "
                     "decides. A route says which way to turn already.")
    if args.max_reverse_duty <= 0.0:
        parser.error("--max-reverse-duty is a magnitude and must be positive; "
                     "the sign is applied by speed_ctrl.reverse_duty.")
    if args.max_reverse_duty > speed_ctrl.MIN_MOVE_DUTY * 2:
        print(f"WARNING: --max-reverse-duty {args.max_reverse_duty} is well "
              f"over the cogging floor of {speed_ctrl.MIN_MOVE_DUTY}.\n"
              "         Reverse is the direction with no lidar behind it and "
              "reverse_ctrl's loop\n         stiffens with speed on gains "
              "nothing has measured on a car.")

    if not 0.0 <= args.audio_volume <= 1.0:
        parser.error("--audio-volume must be between 0.0 and 1.0")

    if args.wall_stop < clustering.MIN_CONE_RANGE_M:
        parser.error(
            f"--wall-stop {args.wall_stop} is inside "
            f"{clustering.MIN_CONE_RANGE_M} m, where a return is discarded as "
            "the chassis arc leaking and\n       the wall cone stops being a "
            "cluster at all.")

    if args.goal_stop < clustering.MIN_CONE_RANGE_M:
        parser.error(
            f"--goal-stop {args.goal_stop} is inside "
            f"{clustering.MIN_CONE_RANGE_M} m, where a return is discarded as "
            "the chassis arc\n       leaking and the trophy stops being a "
            "cluster at all. The car would drive\n       at a goal it can no "
            "longer see and stop on dead reckoning, if at all.")

    if args.no_camera:
        # drive_corridor only warns. Here it is fatal: the fork is exactly the
        # case its own help text calls out, and red cannot be inferred from
        # geometry at all.
        parser.error("--no-camera cannot work at a junction: the gate is red, "
                     "and geometry cannot tell red from a boundary cone. It is "
                     "valid on a plain corridor -- use drive_corridor.py.")
    if bool(args.route) == bool(args.explore):
        parser.error(
            "pass exactly one of --route and --explore. A route says which way "
            "to turn at\n       each junction; --explore decides that on the "
            "spot. There is no sensible\n       reading of both, and neither "
            "is a default.")

    args.route_turns = []
    if args.route:
        try:
            args.route_turns = load_route(args.route)
        except ValueError as exc:
            parser.error(str(exc))
    if args.emit_route and not args.explore:
        parser.error(
            "--emit-route needs --explore. On a provided route the emitted "
            "file would be\n       the route it was given, minus any branch "
            "that turned out to be a wall --\n       which is a finding to "
            "read in the log, not a file to drive.")
    return args


def dry_run_travel(step, deadband_m=ego_motion.DEADBAND_M):
    """(travel_m, yaw_delta_rad) for the state machine, from a measured Step.

    The dry run's odometry. Replaces two generations of fiction: the duty
    estimate (identically zero with the throttle pinned) and an assumed push
    speed (exactly as honest as the operator's pace was close to it --
    measured 0.13 m/s against an assumed 0.5 on 2026-09-01, which declared
    the junction passed 1.94 m before the gate). The deadband stops cluster
    jitter from random-walking `travelled_m` upward while the car stands
    still, since `topo_state` clamps negative travel.

    None -- no cones in common between scans -- reads as no motion, the same
    safe convention an empty centerline gets.
    """
    if step is None:
        return 0.0, 0.0
    travel = step.forward_m if abs(step.forward_m) > deadband_m else 0.0
    return travel, step.yaw_rad


RULE = "=" * 66


def banner(headline, detail=""):
    """Print a state change so it survives a 1 Hz status stream.

    The bench line prints every second, and the transitions used to print at
    the same indent and weight -- so the one event worth reacting to, the car
    changing its mind about what it is doing, scrolled past looking exactly
    like the ninety lines around it. Blank lines and a rule are what make it
    findable at a glance while walking beside a moving car, which is the only
    condition this output is ever read in.
    """
    print("\n" + RULE)
    print("  " + headline)
    if detail:
        print("  " + detail)
    print(RULE + "\n")


def reverse_speed_warning(duty):
    """The line to print when a commanded reverse exceeds the gains' ceiling.

    Empty when it does not. See `speed_ctrl.reverse_mps`: the cogging floor and
    `reverse_ctrl.MAX_REVERSE_MPS` cannot both be satisfied, so this is a
    standing condition to be measured out of existence at stage 8b rather than
    a fault to be fixed by turning a number down.
    """
    mps = speed_ctrl.reverse_mps(duty)
    if mps <= reverse_ctrl.MAX_REVERSE_MPS:
        return ""
    return (f"           duty {duty} is about {mps:.2f} m/s on a FORWARD-fitted "
            f"DUTY_TO_MPS, over the\n"
            f"           {reverse_ctrl.MAX_REVERSE_MPS} m/s above which the "
            f"reverse gains have never been checked.\n"
            f"           Nothing below the cogging floor can be commanded, so "
            f"this stands until\n"
            f"           stage 8b measures reverse m/s directly.")


def announce(args):
    if args.reverse_only:
        print(RULE)
        print("  REVERSE ONLY   the car will drive BACKWARDS while X is held,")
        print(f"                 at duty {args.max_reverse_duty}, with nothing "
              "steering it.")
        print("                 Wheels off the ground the first time. Nothing "
              "is behind")
        print("                 the car that the car can see -- the chassis "
              "blanks the")
        print("                 rear 142 deg. See docs/junction-bringup.md "
              "stage 8a.")
        warning = reverse_speed_warning(args.max_reverse_duty)
        if warning:
            print(warning.replace("           ", "                 "))
        print(RULE + "\n")
        return
    if args.explore:
        print(f"explore   no route: deciding at each junction, trying "
              f"{args.explore_first} first")
        print("           a dead end backs the search out and takes the other "
              "branch")
        if args.reverse:
            print(f"           at a wall the car BACKS ITSELF OUT under "
                  f"power at duty {args.max_reverse_duty} -- no carry")
            print("           the car cannot see behind it. Keep the corridor "
                  "behind it clear")
            warning = reverse_speed_warning(args.max_reverse_duty)
            if warning:
                print(warning)
        else:
            print("           at a wall the car stops for you to carry it "
                  "back (stage 7b)")
        if args.emit_route:
            print(f"           the route to the goal will be written to "
                  f"{args.emit_route}")
    else:
        turns = ", ".join(args.route_turns)
        print(f"route     {len(args.route_turns)} junction(s): {turns}")
        print(f"           from {args.route}")
    armed_when = ("   (ARMED FROM THE START)"
                  if args.goal_anywhere or args.explore else
                  "   (armed once the route is spent)")
    print(f"goal      stop {args.goal_stop} m from the lidar" + armed_when)
    if args.explore and not args.goal_anywhere:
        print("           armed throughout because a maze puts the goal "
              "wherever it likes.\n"
              "           Magenta read as red is the detector's hardest pair "
              "(15% on v3), so a\n"
              "           false stop mid-course is the failure to watch for.")
    if args.no_audio:
        print("audio     disabled")
    else:
        print(f"audio     {args.drive_audio} while driving")
        print(f"          {args.goal_audio} at the goal, volume "
              f"{args.audio_volume:.2f}, target "
              f"{args.audio_target or 'default sink'}")
    if args.goal_anywhere:
        print("warning:  --goal-anywhere: any confirmed magenta may stop the "
              "car, including one\n"
              "          seen before the last turn. Magenta and red are the "
              "detector's\n"
              "          hardest pair -- v1 and v2 called 69% of magenta RED -- "
              "so on a track\n"
              "          with junctions this can end the run mid-course. "
              "Bring-up only.")
    if args.dry_run:
        print("dry run   travel is MEASURED by scan matching over the cones; "
              "push at any pace,\n          pause freely. No push-speed "
              "assumption is in play.")


def main(argv=None):
    args = parse_args(argv)
    require_vehicle_geometry()

    record = fusion_view.resolve_calibration(args)

    detector = detectors.build(args.detector, weights=args.weights,
                               imgsz=args.imgsz, conf=args.conf,
                               device=args.device)
    device, (width, height) = oakd.open_camera(args.camera_fps)
    intr = oakd.camera_intrinsics(device, width, height)
    drive_corridor.announce(args, record, intr, detector.name)
    announce(args)

    deadman = Deadman(args.joystick)
    if not deadman.present and not args.no_deadman:
        device.close()
        raise SystemExit(
            f"error: no joystick at {args.joystick}. The deadman is not "
            "optional --\n       nothing may drive itself with no way to stop "
            "it. Switch the F710\n       on, or pass --dry-run --no-deadman to "
            "compute without actuating.")

    handle = open_serial(args.port, BAUD)
    reader = fusion_view.LidarReader(handle)
    reader.start()

    q_preview = device.getOutputQueue("preview", maxSize=1, blocking=False)
    oakd.lock_camera(device.getInputQueue("control"))
    detector_thread = ThreadedDetector(detector, q_preview)
    detector_thread.start()

    sinks = fusion_view.Sinks()
    drive_ch = sinks.channel("/drive_status", JUNCTION_STATUS_SCHEMA)
    junction_ch = sinks.channel("/junction", JUNCTION_SCHEMA)
    if not sinks.available:
        print(f"warning:  no Foxglove sink ({sinks.reason}); running headless")

    server = None
    if sinks.available and not args.no_live:
        server = sinks.start_server(args.host, args.ws_port)
        print(f"\nFoxglove: ws://<car-ip>:{args.ws_port}  (desktop app)\n")

    vesc = None
    if not args.dry_run:
        try:
            vesc = VescDriver(args.vesc_port,
                              invert_steering=args.invert_steering)
        except Exception as exc:
            reader.stop(); handle.close(); detector_thread.stop(); device.close()
            raise SystemExit(
                f"error: could not open the VESC on {args.vesc_port}: {exc}\n"
                "       Is DonkeyCar still running? Only one process may hold "
                "that port.\n       Check with: fuser -v /dev/ttyACM0")

    log = TrialLog(args.log)
    if args.log:
        print(f"logging every tick to {args.log}")
    print("hold X on the F710 to arm. Release to stop. Ctrl-C to quit.\n")

    # A pushed car covers the traverse's distance floor at walking pace, and
    # the travel it accrues is measured -- so the dry run gets a walker's
    # clock. Under power the driving bound stands.
    red_memory = label_memory.RedMemory()
    goal_latch = goal_stop.GoalLatch(stop_range_m=args.goal_stop)
    dead_end_latch = dead_end.DeadEndLatch()
    # The same latch the trophy uses, aimed at the orange. It never asks what
    # colour the point is; `goal_detect.survey(candidates=...)` decides that.
    wall_latch = goal_stop.GoalLatch(stop_range_m=args.wall_stop)
    backout = (backout_mod.BackoutManoeuvre(
        max_reverse_duty=args.max_reverse_duty) if args.reverse else None)
    # `topo.commit_range_m` is zeroed by `_reset` the instant a gate is called
    # passed, and a dead end comes many seconds after that -- so the range the
    # car committed from has to be caught while it still exists. It sizes the
    # backout's distance bound; see cone_nav/guidance/backout.py.
    last_commit_range_m = 0.0
    backouts = 0
    # The pose is read by the map and by the report, and by nothing that
    # steers. See cone_perception/odometry.py on why that line matters.
    pose = odometry.Pose()
    maze = graph_builder.MazeMap()
    last_node_pose = pose.snapshot()
    # `ExplorePolicy` and `RouteCursor` implement the same five members and two
    # events, which is why swapping them needs no change to `topo_state`.
    cursor = (ExplorePolicy(first=args.explore_first) if args.explore
              else RouteCursor(args.route_turns))
    topo = topo_state.TopoState(
        cursor,
        max_traverse_ticks=(topo_state.MAX_TRAVERSE_TICKS * 3
                            if args.dry_run else
                            topo_state.MAX_TRAVERSE_TICKS))
    started = time.monotonic()
    axis_rad = 0.0
    duty_now = 0.0
    steer_history = []
    previous_line = None
    previous_cones = None
    odo_step = None
    travel_m = 0.0
    yaw_delta_rad = 0.0
    last_scan_at = started
    last_report = started
    last_state = topo.state
    last_goal_state = goal_latch.state
    was_armed = False
    was_dead_end = False
    was_at_wall = False
    was_at_goal = False
    driven_turns = []
    loops = 0
    audio = audio_playback.AudioController(
        drive_audio=args.drive_audio,
        goal_audio=args.goal_audio,
        volume=args.audio_volume,
        target=args.audio_target,
        enabled=not args.no_audio,
    )

    try:
        while True:
            now = time.monotonic()
            if args.duration and now - started >= args.duration:
                break

            armed = deadman.poll() if deadman.present else bool(args.no_deadman)

            # Releasing X and pressing it again clears a goal stop, so the
            # trophy can be reset and the run repeated without restarting the
            # tool -- which would mean reopening the camera and the lidar. A
            # RISING edge, deliberately: holding X through the arrival must not
            # drive the car into the trophy it just stopped at.
            if armed and not was_armed and goal_latch.stopped:
                goal_latch.release()
                banner("GOAL RELEASED   driving again")

            # And clears a dead end, which is what makes an operator-assisted
            # backtrack possible before `reverse_ctrl` exists: the car stops at
            # the wall, the search has ALREADY taken the other branch (the
            # cursor moved when the latch fired), someone carries the car back
            # to the junction, and X re-pressed drives it down the branch it
            # has not tried. The map records the same thing either way.
            if armed and not was_armed and dead_end_latch.latched:
                dead_end_latch.release()
                # The car has been carried somewhere between the release and
                # now, and `rigid_step` cannot see a move that large -- it
                # finds no cone within MATCH_GATE_M and returns None, which
                # every other tick correctly reads as no motion. So the pose
                # would silently omit the whole lift and every edge measured
                # afterwards would start from the wrong place. Poisoning the
                # frame makes those come back unmeasured instead of wrong.
                pose.mark_discontinuity()
                # The pose is not the only thing the lift invalidated. Every
                # scan-to-scan feedback the loop carries is now wrong, and two
                # of them feed the very thing the car is about to do --
                # recognise the junction again.
                #
                # `axis_rad` is the worst. It orders the reds left-to-right in
                # `gate_detect.survey` and picks which wall a cone is on in
                # `fill_unlabeled`, and `heading_of` HOLDS its last value
                # whenever the line dies -- which at a wall it always has. So
                # it reaches the re-approach carrying a heading from inside the
                # branch, 20-25 deg off the corridor. Zero is the car's own
                # frame and the right prior, for the same reason
                # `goal_detect.trusted_axis` falls back to it: a car placed in
                # a corridor is aligned with it.
                #
                # `red_memory` is metres out of date and re-binds on a 0.20 m
                # gate sized for ONE tick of travel, so until its 3 s TTL runs
                # out it can paint red onto whatever now stands where a
                # different cone used to be.
                #
                # Then the odometry landmarks, the corridor line `topo_state`
                # reads next tick, and the steering median.
                axis_rad = 0.0
                wall_latch.release()
                red_memory.forget()
                previous_cones = None
                previous_line = None
                steer_history = []
                banner(
                    "DEAD END RELEASED   now taking "
                    f"{(topo.cursor.current or '-').upper()}",
                    "pose frame broken by the lift; carried perception state "
                    "cleared")

            # All three read `was_armed` before it is overwritten below, so
            # the audio latches on the same rising edge the goal and the dead
            # end release on.
            audio_playback.update_for_deadman(audio, armed, was_armed)
            was_armed = armed

            scan = reader.take()
            if scan is None:
                if vesc is not None and now - last_scan_at > MAX_SCAN_AGE_S:
                    vesc.stop()
                    duty_now = 0.0
                time.sleep(0.005)
                continue

            scan_age = now - last_scan_at
            last_scan_at = now
            loops += 1

            detection_set = detector_thread.latest()
            # The goal lies past the last junction by construction, so a
            # magenta seen while a turn is still outstanding is a misread and
            # must not be allowed to stop the car. --goal-anywhere overrides
            # this for bring-up on a corridor with no junction in it.
            # Read before `drive_pipeline`, which calls `topo.update` and so
            # may consume the turn and move the path out from under them.
            path_before = list(topo.cursor.path)
            turn_before = topo.turn

            # `goal_armed` is the cursor's own answer: a route arms the goal
            # once it is spent, exploring arms it throughout. See
            # cone_nav/guidance/explore.py.
            goal_armed = args.goal_anywhere or topo.cursor.goal_armed
            # Reversing, the car points the wrong way down a junction it is
            # moving away from, and a magenta ahead of it is not an arrival.
            # Both machines stand down inside drive_pipeline; the gate survey
            # still runs, because the manoeuvre's own ending is read off it.
            backing_out = backout is not None and backout.active
            if backing_out:
                goal_armed = False
            (result, cones, filled, line, pursuit, duty, corridor_line,
             junction, dropped, survey, remembered,
             goal_survey, wall_survey) = drive_pipeline(
                scan, detection_set, record, intr, args, now, axis_rad,
                topo=topo, previous_line=previous_line, travel_m=travel_m,
                yaw_delta_rad=yaw_delta_rad, red_memory=red_memory,
                goal_latch=goal_latch, goal_armed=goal_armed,
                dead_end_latch=dead_end_latch, wall_latch=wall_latch, backing_out=backing_out)
            previous_line = corridor_line
            axis_rad = heading_of(line, default=axis_rad)

            # Scan-matched odometry, fed to topo_state NEXT tick -- the same
            # one-tick feedback previous_line and axis_rad already use. The
            # pre-fill cone list is the feature set: unlabeled clusters are
            # landmarks too, and the fill's repainting never moves a point.
            odo_step = (ego_motion.rigid_step(previous_cones, result.cones)
                        if previous_cones is not None else None)
            previous_cones = result.cones
            pose.integrate(odo_step)

            # The two events the map is built from, and the one the search acts
            # on. Both keys are taken BEFORE the machine that consumes them
            # moves: `topo_state` advances the cursor inside `update`, so
            # `path_before` and `turn_before` are captured at the top of the
            # tick, and the dead end is recorded before `cursor.dead_end()`
            # unwinds the stack out from under it.
            # Caught while it exists; see where `last_commit_range_m` is
            # declared. `_reset` zeroes it on the same tick `passed` is set.
            if topo.commit_range_m > 0.0:
                last_commit_range_m = topo.commit_range_m

            if topo.note == "passed":
                driven_turns.append(turn_before)
                maze.record_pass(path_before, turn_before, length_m=(
                    odometry.distance_between(last_node_pose, pose.snapshot())))
                last_node_pose = pose.snapshot()

            # A completed wall run-in is a dead end, named by arrival rather
            # than by the corridor running out. It feeds the same event, so
            # everything downstream -- the search, the map, the banner -- is
            # unchanged and cannot tell which signal got there first.
            wall_reached = wall_latch.stopped and not was_at_wall
            was_at_wall = wall_latch.stopped
            if wall_reached and not dead_end_latch.latched:
                dead_end_latch.force(
                    f"drove up to the wall, stopped "
                    f"{wall_latch.range_m:.2f} m short (orange run-in)")

            if dead_end_latch.latched and not was_dead_end:
                maze.record_dead_end(topo.cursor.path)
                resume = topo.cursor.dead_end()
                # Before `last_node_pose` moves: how far the car drove from the
                # junction to this wall. It is a BOUND on the reverse, not its
                # target -- see cone_nav/guidance/backout.py.
                edge_m = odometry.distance_between(last_node_pose,
                                                   pose.snapshot())
                last_node_pose = pose.snapshot()
                if backout is not None and resume is not None:
                    backout.begin(resume, budget_m=edge_m,
                                  commit_range_m=last_commit_range_m)
                    backouts += 1
                    steer_history = []
                    duty_now = 0.0
                    banner(
                        f"*** DEAD END ***   {dead_end_latch.reason}",
                        f"backing out {backout.bound_m:.2f} m at most, to "
                        f"take {resume.upper()}")
                else:
                    banner(
                        f"*** DEAD END ***   {dead_end_latch.reason}",
                        (f"back out to the last junction and take "
                         f"{resume.upper()}" if resume else
                         "nothing left to explore -- every branch has been "
                         "tried"))
            was_dead_end = dead_end_latch.latched

            if goal_latch.stopped and not was_at_goal:
                maze.record_goal(topo.cursor.path)
            was_at_goal = goal_latch.stopped

            target_duty = duty.duty
            if not armed:
                target_duty = 0.0
            if args.steer_only or args.dry_run:
                target_duty = 0.0
            if goal_latch.stopped:
                # The run is over. Held here rather than inside `speed_ctrl` so
                # that every gate which can stop this car stays in one place,
                # the way `armed` and --steer-only already are.
                target_duty = 0.0
            if dead_end_latch.latched:
                # `speed_ctrl`'s reach floor has almost certainly stopped the
                # car already -- that collapse is what the latch fired on. This
                # is here so the stop is stated rather than incidental, and so
                # it holds if a future reach rule would not.
                target_duty = 0.0

            steer_override = None
            if backing_out:
                # The manoeuvre owns both commands while it runs. It is driven
                # here rather than in `drive_pipeline` so that everything which
                # decides this car's throttle stays in one place, next to
                # `armed` and --steer-only.
                #
                # It gets the UNANCHORED line, for the same reason the dead-end
                # detector does: an anchor is a point threaded onto the driven
                # line, and steering a reverse on a gate anchor would regulate
                # against a point the car is deliberately moving away from.
                backout.update(corridor_line, junction, travel_m=travel_m,
                               armed=armed)
                target_duty = backout.duty
                steer_override = backout.steer_normalised
            elif args.reverse_only:
                # Stage 8a: a bare commanded reverse, nothing steering it.
                target_duty = (speed_ctrl.reverse_duty(args.max_reverse_duty)
                               if armed else 0.0)
                steer_override = 0.0

            duty_now = speed_ctrl.ramp(duty_now, target_duty)

            steer_history, steer = pure_pursuit.smooth(
                steer_history,
                pursuit.normalised if pursuit is not None else None,
                window=args.smooth_window)
            if steer_override is not None:
                # Unsmoothed, and the history cleared at both edges: a median
                # window straddling a direction change blends two laws that
                # disagree about the sign of the heading term.
                steer = steer_override
                steer_history = []

            if backout is not None and backout.arrived:
                # Back where it can see the whole junction. Releasing the dead
                # end restarts that latch's own re-arm travel floor, which is
                # what stops the wall behind the car being named again the
                # moment it drives forward.
                dead_end_latch.release()
                backout.release()
                steer_history = []
                last_node_pose = pose.snapshot()
                banner(f"BACKED OUT   now taking "
                       f"{(topo.cursor.current or '-').upper()}",
                       "pose frame intact -- the car did the moving, so this "
                       "edge is measured")
            elif backout is not None and backout.abandoned:
                # Hand back to the operator, which is exactly the behaviour
                # this manoeuvre replaces. The dead end stays latched, so X
                # released and pressed again resumes the carry.
                banner(f"BACKOUT ABANDONED   {backout.reason}",
                       "carry the car back to the junction and press X, as in "
                       "stage 7b")
                backout.release()
            servo = 0.0
            if vesc is not None:
                if armed:
                    servo = vesc.drive(steer, duty_now)
                else:
                    vesc.stop()
                    servo = vesc.last_servo

            # What topo_state is told next tick. Measured scan odometry
            # whenever a step exists -- it does not care whether the wheels or
            # a hand moved the car, and it replaced two generations of guesses
            # in the dry run. Under power the old open-loop duty estimate
            # (speed_ctrl.DUTY_TO_MPS, an admitted guess, least trustworthy at
            # the cogging floor) survives only as the fallback for a tick with
            # no cones in common between scans.
            if odo_step is not None or args.dry_run:
                travel_m, yaw_delta_rad = dry_run_travel(odo_step)
            else:
                speed = duty_now * speed_ctrl.DUTY_TO_MPS
                travel_m = speed * scan_age
                delta = pursuit.delta_rad if pursuit is not None else 0.0
                yaw_delta_rad = (travel_m * math.tan(delta)
                                 / extrinsics.WHEELBASE_M)

            detection_age = (detection_set.age(now)
                             if detection_set is not None else float("inf"))
            elapsed = now - started
            base = drive_corridor.status_of(
                result, cones, filled, line, pursuit, duty, servo, armed, args,
                scan_age, detection_age, loops / elapsed if elapsed else 0.0,
                commanded=steer)
            status = status_of(base, topo, junction, dropped, survey,
                               dead_end_latch=dead_end_latch, pose=pose,
                               wall_latch=wall_latch, wall_survey=wall_survey,
                               maze=maze, backout=backout,
                               goal_survey=goal_survey, goal_latch=goal_latch,
                               goal_armed=goal_armed)
            if goal_latch.stopped:
                # Say what actually stopped the car. Once latched the line is
                # the goal anchor alone, so `speed_ctrl` reports "centerline too
                # short" -- true, incidental, and the first field an analyst
                # reads. An arrival must not be filed under a perception fault.
                status["stop_reason"] = "goal reached"

            status["labeled_by_memory"] = remembered
            # The map's raw material. Pre-fill and pre-branch-filter, which is
            # the list `ego_motion` is fitted on: the fill only repaints and
            # `keep_branch` DELETES, so a map built after either would lose
            # exactly the cones a junction is made of. ~14 bytes a cone, so a
            # 600-tick run grows by a few hundred KB against a log already
            # half a megabyte.
            # The camera side of the same tick. `cones_xy` records what fusion
            # DECIDED; without the boxes it was deciding between, a cluster
            # that came back the wrong colour cannot be diagnosed -- which box
            # claimed it, and how far off in bearing it was, are exactly the
            # two numbers missing. Bearing in the CAMERA frame, degrees, left
            # positive, so it lines up with a cluster's own bearing.
            status["boxes"] = ";".join(
                f"{d.cls},{math.degrees(geometry.detection_bearing(d, intr)):.1f}"
                f",{d.confidence:.2f}"
                for d in (detection_set.detections if detection_set else ()))
            status["cones_xy"] = ";".join(
                f"{c.x:.3f},{c.y:.3f},{c.cone_class}" for c in result.cones)
            status["odo_forward_m"] = round(odo_step.forward_m, 4) if odo_step else 0.0
            status["odo_lateral_m"] = round(odo_step.lateral_m, 4) if odo_step else 0.0
            status["odo_yaw_deg"] = (round(math.degrees(odo_step.yaw_rad), 3)
                                     if odo_step else 0.0)
            status["odo_pairs"] = odo_step.pairs if odo_step else 0

            wall = time.time()
            if sinks.available:
                sinks.log_transforms(wall)
                sinks.log_scan(scan, wall, args.bins, record["mirror"],
                               record["angle_offset_deg"])
                sinks.log_cones(cones, wall)
                sinks.log_centerline(line, wall)
                drive_ch.log(status, log_time=int(wall * 1e9))
                if junction is not None:
                    junction_ch.log({
                        "reds": "; ".join(
                            f"({c.x:.2f}, {c.y:.2f})"
                            for c in (junction.left, junction.centre,
                                      junction.right)),
                        "left_gate": "(%.2f, %.2f)" % junction.left_gate,
                        "right_gate": "(%.2f, %.2f)" % junction.right_gate,
                        "chosen": topo.turn or "",
                    }, log_time=int(wall * 1e9))
            log.write(dict(status, t=round(elapsed, 3), wall=wall))

            # State changes are rare and each one matters, so they print when
            # they happen rather than waiting for the once-a-second line.
            if topo.state != last_state or topo.note:
                banner(
                    f"{last_state.upper()} -> {topo.state.upper()}"
                    + (f"   turn {topo.turn.upper()}" if topo.turn else ""),
                    f"gate {status['gate_range_m']:.2f} m, "
                    f"{topo.cursor.remaining} branch(es) left"
                    + (f"   ({topo.note})" if topo.note else ""))
                last_state = topo.state

            # The arrival is the one event this whole tool exists to produce,
            # so it prints when it happens rather than waiting up to a second.
            goal_state_changed = goal_latch.state != last_goal_state
            audio_playback.update_for_goal(
                audio, goal_stopped=goal_latch.stopped,
                state_changed=goal_state_changed)
            if goal_state_changed:
                banner(
                    ("*** GOAL REACHED ***" if goal_latch.stopped else
                     f"GOAL {last_goal_state.upper()} -> "
                     f"{goal_latch.state.upper()}"),
                    f"{status['goal_range_m']:.2f} m"
                    + (f", carried {goal_latch.blind_ticks} ticks"
                       if goal_latch.blind_ticks else "")
                    + (f"   ({goal_latch.note})" if goal_latch.note else "")
                    + ("   release X and press it again to drive on"
                       if goal_latch.stopped else ""))
                last_goal_state = goal_latch.state

            if now - last_report >= 1.0:
                last_report = now
                flag = "ARMED " if armed else "idle  "
                # The reds line is what stage 3 is actually watching, so it says
                # where they are and why they did not arm rather than only how
                # many were countable. Standing still in front of the mouth,
                # this is the whole diagnosis without opening the log.
                reds = f"reds {len(survey.in_arm)}/{len(survey.reds)}"
                if survey.reds:
                    reds += " @ " + status["reds_m"] + " m"
                if survey.gaps_m:
                    reds += f"  gaps {status['red_gaps_m']}"
                if survey.reason:
                    reds += f"  [{survey.reason}]"
                # Only once the goal is the live question -- while turns remain
                # this would be a column of 'no magenta' scrolling past the
                # thing the operator is actually watching.
                goal_line = ""
                if goal_armed or goal_survey.magenta:
                    goal_line = f"  goal {goal_latch.state}"
                    if goal_latch.range_m is not None:
                        goal_line += f" @ {goal_latch.range_m:.2f} m"
                    if goal_latch.blind_ticks:
                        goal_line += f" (carried {goal_latch.blind_ticks})"
                    if goal_survey.reason:
                        goal_line += f" [{goal_survey.reason}]"
                    if not goal_armed:
                        goal_line += " [disarmed: route not spent]"
                health = drive_corridor.camera_health(
                    detection_age, args.max_detection_age)
                health = " / ".join(
                    t for t in (health, drive_corridor.pad_health(deadman))
                    if t)
                # The dead end says why it declined, beside the gate and the
                # goal saying the same. Without it the only way to tell "the
                # corridor is genuinely still open" from "held down inside a
                # mouth" from "the pairing collapsed" is to pull the log --
                # which is no use to someone watching a car drive at a wall.
                # Suppressed once latched: `stop_reason` already says that.
                # The run-in first when it is live: "closing on the wall at
                # 0.74 m" is a different thing to know than why the geometric
                # signal has not fired, and it is the one that is about to act.
                if dead_end_latch.latched:
                    wall = ""
                elif wall_latch.state != goal_stop.SEEKING:
                    wall = (f"  WALL {wall_latch.state} "
                            f"{status['wall_range_m']:.2f} m")
                else:
                    wall = (f"  wall? [{dead_end_latch.reason}]"
                            if dead_end_latch.reason else "")
                print(f"  {flag} duty {duty_now:.3f}  steer "
                      f"{status['steer_deg']:+6.1f} deg  "
                      f"{len(line.points)} pts, reach {duty.reach_m:.2f} m  "
                      f"{topo.state} {reds}"
                      + (f"/{topo.turn}" if topo.engaged else "")
                      + goal_line + wall
                      # status['stop_reason'], not duty.reason: once latched
                      # the line is the anchor alone and the speed law says
                      # "centerline too short", which is true, incidental, and
                      # not what stopped the car. The log already says so; the
                      # console must not disagree with it.
                      + (f"  [{status['stop_reason']}]"
                         if status["stop_reason"] else "")
                      + (f"  !! {health}" if health else ""))
    except KeyboardInterrupt:
        print("\nstopping")
    except Exception:
        if vesc is not None:
            vesc.stop()
        raise
    finally:
        if vesc is not None:
            vesc.close()
        audio.close()
        reader.stop()
        detector_thread.stop()
        reader.join(timeout=1.0)
        handle.close()
        device.close()
        deadman.close()
        log.close()
        if server is not None:
            server.stop()
        if log.path:
            print(f"wrote {log.rows} ticks to {log.path}")
        if args.explore:
            # `remaining` means something different here: branches the car
            # found and did not try, not route entries left unread. A run that
            # ends with some is a partial map, which is honest and still
            # plannable -- as long as the goal is in it.
            print(f"maze      {maze.summary()}, "
                  f"{len(driven_turns)} gate(s) driven")
            print(f"           ended at "
                  f"{'/'.join(topo.cursor.path) or 'the start'}")
            if topo.cursor.remaining:
                print(f"           {topo.cursor.remaining} branch(es) found "
                      "but never tried")
        elif topo.cursor.remaining:
            print(f"warning:  {topo.cursor.remaining} junction(s) of the route "
                  "were never taken")

        if goal_latch.stopped:
            print(f"reached the goal, stopped {goal_latch.range_m:.2f} m from it"
                  + (" ON A CARRIED ESTIMATE -- the camera had lost it"
                     if "carried" in goal_latch.note else ""))
        elif not args.explore and not topo.cursor.remaining:
            print("warning:  the route was completed but the goal was never "
                  "reached")

        if args.emit_route:
            # Written here rather than on arrival so that a run ended by Ctrl-C
            # or by a lunge for the car still emits whatever it had mapped --
            # which is the run most worth having the file from.
            try:
                turns = planner.route_to_goal(maze)
            except planner.NoRouteError as exc:
                print(f"warning:  no route written to {args.emit_route}: {exc}")
            else:
                _, drove, avoided = planner.saving(maze, driven_turns)
                planner.write_route(
                    turns, args.emit_route,
                    note=(f"Explored {time.strftime('%Y-%m-%d %H:%M')}. "
                          f"{maze.summary()}.\n"
                          f"Driven {drove} gates while exploring; this route "
                          f"is {len(turns)}, avoiding {avoided}."))
                print(f"wrote {args.emit_route}: "
                      f"{', '.join(turns) or '(no turns)'}")
                print(f"           {drove} gate(s) driven exploring, "
                      f"{len(turns)} on this route -- {avoided} avoided")
    return 0


if __name__ == "__main__":
    sys.exit(main())
