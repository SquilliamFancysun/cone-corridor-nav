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
    for _pkg in ("cone_perception", "cone_nav"):
        _src = os.path.join(_REPO, "ros2", "src", _pkg)
        if os.path.isdir(_src) and _src not in sys.path:
            sys.path.insert(0, _src)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import audio_playback
import detectors
import drive_corridor
import fusion_view
import oakd
from cone_nav.control import pure_pursuit, speed_ctrl
from cone_nav.corridor.centerline import centerline
from cone_nav.corridor import side_assign
from cone_nav.corridor.side_assign import fill_unlabeled, heading_of
from cone_nav.corridor.boundary_split import split
from cone_nav.guidance import goal_stop, junction_exec, planner
from cone_nav.guidance.explore import ExplorePolicy
from cone_nav.guidance.route_exec import RouteCursor, load_route
from cone_nav.topology import (dead_end, gate_detect, goal_detect,
                               graph_builder, topo_state)
from cone_perception import (clustering, ego_motion, extrinsics, fusion,
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
        cones_xy={"type": "string", "description": "this tick's cones in base_link as x,y,class;... -- what analysis/map_from_log.py turns into a map. Pre-fill and pre-branch-filter, the same list the odometry is fitted on"},
        dead_end_state={"type": "string", "description": "clear / dead_end"},
        dead_end_reason={"type": "string", "description": "why the corridor is or is not judged to have ended. The first field to read when a backtrack fires or fails to"},
        dead_end_reach_m={"type": "number", "description": "reach of the UNANCHORED corridor line, which is what the decision is made on"},
        cursor={"type": "string", "description": "route / explore -- what decided the turns. route_index and route_remaining mean different things in each"},
        explore_path={"type": "string", "description": "turns taken to reach where the car is -- the maze node's identity"},
        maze_nodes={"type": "integer"},
        maze_edges={"type": "integer"},
        maze_dead_ends={"type": "integer"},
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
                   goal_latch=None, goal_armed=False, dead_end_latch=None):
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

    gate_xy, dropped = None, 0
    if engaged and topo.junction is not None:
        gate_xy, _divider = junction_exec.select(topo.junction, topo.turn)
        # The divider and axis come from the machine, not from the latched
        # junction: through the blind period those are carried forward with the
        # car's motion, and the latched pair are stale by metres.
        cones, dropped = junction_exec.keep_branch(
            cones, topo.divider_xy, topo.axis_rad, topo.turn)

    corridor_line = centerline(cones, car_xy=(0.0, 0.0))
    line = corridor_line
    if engaged and topo.anchor_ok and gate_xy is not None:
        line = junction_exec.junction_line(corridor_line, gate_xy)
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
    run_in = goal_latch is not None and goal_latch.run_in
    duty = speed_ctrl.duty(pursuit, line, max_duty=args.max_duty, origin=axle,
                           min_reach_m=0.0 if run_in else speed_ctrl.MIN_REACH_M,
                           min_points=1 if run_in else 2)
    if dead_end_latch is not None:
        # On the UNANCHORED line: an anchor is a point threaded onto the driven
        # line, so judging reach from `line` would credit the corridor with a
        # gate or a trophy it does not contain. Held down wherever the corridor
        # is ALLOWED to end -- through a mouth, and over the goal run-in. See
        # cone_nav/topology/dead_end.py.
        dead_end_latch.update(
            corridor_line, cones, oranges=split(cones).dead_ends,
            armed=not engaged and not run_in, origin=axle, travel_m=travel_m)
    return (result, cones, filled, line, pursuit, duty, corridor_line,
            junction, dropped, survey, remembered, goal_survey)


def status_of(base, topo, junction, dropped, survey=None, goal_survey=None,
              goal_latch=None, goal_armed=False, dead_end_latch=None,
              pose=None, maze=None):
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

    if not 0.0 <= args.audio_volume <= 1.0:
        parser.error("--audio-volume must be between 0.0 and 1.0")

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


def announce(args):
    if args.explore:
        print(f"explore   no route: deciding at each junction, trying "
              f"{args.explore_first} first")
        print("           a dead end backs the search out and takes the other "
              "branch")
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
                print("  [goal released] X re-pressed -- driving again")

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
                print(f"  [dead end released] X re-pressed -- now taking "
                      f"{topo.cursor.current or '-'}")
                print("                      pose frame broken by the lift; "
                      "edges across it are unmeasured")

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
            (result, cones, filled, line, pursuit, duty, corridor_line,
             junction, dropped, survey, remembered,
             goal_survey) = drive_pipeline(
                scan, detection_set, record, intr, args, now, axis_rad,
                topo=topo, previous_line=previous_line, travel_m=travel_m,
                yaw_delta_rad=yaw_delta_rad, red_memory=red_memory,
                goal_latch=goal_latch, goal_armed=goal_armed,
                dead_end_latch=dead_end_latch)
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
            if topo.note == "passed":
                driven_turns.append(turn_before)
                maze.record_pass(path_before, turn_before, length_m=(
                    odometry.distance_between(last_node_pose, pose.snapshot())))
                last_node_pose = pose.snapshot()

            if dead_end_latch.latched and not was_dead_end:
                maze.record_dead_end(topo.cursor.path)
                resume = topo.cursor.dead_end()
                last_node_pose = pose.snapshot()
                print(f"  [DEAD END] {dead_end_latch.reason}")
                print("             " + (
                    f"back out to the last junction and take {resume}"
                    if resume else
                    "nothing left to explore -- every branch has been tried"))
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
            duty_now = speed_ctrl.ramp(duty_now, target_duty)

            steer_history, steer = pure_pursuit.smooth(
                steer_history,
                pursuit.normalised if pursuit is not None else None,
                window=args.smooth_window)
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
                               maze=maze,
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
                print(f"  [{last_state} -> {topo.state}] turn "
                      f"{topo.turn or '-'}, gate "
                      f"{status['gate_range_m']:.2f} m, "
                      f"{topo.cursor.remaining} left"
                      + (f"  ({topo.note})" if topo.note else ""))
                last_state = topo.state

            # The arrival is the one event this whole tool exists to produce,
            # so it prints when it happens rather than waiting up to a second.
            goal_state_changed = goal_latch.state != last_goal_state
            audio_playback.update_for_goal(
                audio, goal_stopped=goal_latch.stopped,
                state_changed=goal_state_changed)
            if goal_state_changed:
                print(f"  [goal {last_goal_state} -> {goal_latch.state}] "
                      f"{status['goal_range_m']:.2f} m"
                      + (f", carried {goal_latch.blind_ticks} ticks"
                         if goal_latch.blind_ticks else "")
                      + (f"  ({goal_latch.note})" if goal_latch.note else ""))
                if goal_latch.stopped:
                    print("  GOAL REACHED -- release X and press it again to "
                          "drive on.")
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
                print(f"  {flag} duty {duty_now:.3f}  steer "
                      f"{status['steer_deg']:+6.1f} deg  "
                      f"{len(line.points)} pts, reach {duty.reach_m:.2f} m  "
                      f"{topo.state} {reds}"
                      + (f"/{topo.turn}" if topo.engaged else "")
                      + goal_line
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
