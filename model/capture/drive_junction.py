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

Everything downstream of that -- `centerline`, `pure_pursuit`, `speed_ctrl`,
`VescDriver` -- is byte-identical to plain corridor following. A junction is not
a second control stack; it is a filtered cone list.

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

import detectors
import drive_corridor
import fusion_view
import oakd
from cone_nav.control import pure_pursuit, speed_ctrl
from cone_nav.corridor.centerline import centerline
from cone_nav.corridor import side_assign
from cone_nav.corridor.side_assign import fill_unlabeled, heading_of
from cone_nav.guidance import junction_exec
from cone_nav.guidance.route_exec import RouteCursor, load_route
from cone_nav.topology import gate_detect, topo_state
from cone_perception import clustering, extrinsics, fusion
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
        gate_range_m={"type": "number", "description": "to the chosen gate midpoint"},
        gate_gaps_m={"type": "string", "description": "the two measured gate widths"},
        branch_cones_dropped={"type": "integer"},
        blind_ticks={"type": "integer", "description": "ticks since the last triple"},
        travelled_m={"type": "number", "description": "since commit; a duty estimate"},
        topo_note={"type": "string"},
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
                   travel_m=0.0, yaw_delta_rad=0.0):
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

    cones, filled = result.cones, 0
    # Surveyed rather than merely detected, because "saw two reds and one merged
    # into a branch cone", "saw no red at all" and "saw all three from a metre
    # too far back" are different problems with different fixes, and a log that
    # only records whole triples cannot tell them apart. `survey.reason` is the
    # first field to read when stage 3 finds nothing.
    survey = gate_detect.survey(result.cones, axis_rad=axis_rad)
    junction = survey.junction
    if topo is not None:
        topo.update(junction, previous_line, travel_m=travel_m,
                    yaw_delta_rad=yaw_delta_rad)

    engaged = topo is not None and topo.engaged
    if not args.no_fill:
        # An outer red drops out of frame at ~2.1 m and comes back UNLABELED
        # while still inside the fill's 2.0 m reach, so geometry paints it into
        # a wall across the mouth the car is trying to drive through. Pulling
        # the fill in to 1.0 m while a junction is engaged puts the reds outside
        # it -- an outer red 1.0 m ahead is 1.68 m away, being 1.35 m off the
        # axis -- while still covering the near blind spot, which is 0.75 m out.
        #
        # Engaged is not the only time that matters. Seen on the track
        # 2026-08-31, standing in FOLLOW 0.9 m from a gate after a traverse
        # timeout: the centre red was labelled in frame, both outer reds were
        # past the frame edge, and the fill painted them blue and yellow -- a
        # fake corridor whose midpoint was the centre cone, with the centerline
        # aiming the car straight at the island. FOLLOW near a gate happens
        # before the first sighting and after every pass or timeout, so the
        # trigger is a labelled red within the fill's own reach, not the
        # state machine: painting requires an out-of-frame red inside the fill
        # range, and a labelled red that close means its siblings are too. A
        # red glimpsed at 3.5 m must NOT pull the fill in -- that starves the
        # corridor of its near-field labels a full straightaway early.
        near_gate = bool(survey.reds) and (
            min(survey.ranges_m) <= side_assign.MAX_FILL_RANGE_M)
        fill_range = (args.fill_range_at_junction if engaged or near_gate
                      else side_assign.MAX_FILL_RANGE_M)
        cones, filled = fill_unlabeled(
            cones, reference_heading_rad=axis_rad,
            fill_in_fov=args.no_camera, max_range_m=fill_range)

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

    axle = extrinsics.REAR_AXLE_IN_BASE
    pursuit = pure_pursuit.steering_angle(line.points, args.lookahead,
                                          extrinsics.WHEELBASE_M, origin=axle)
    duty = speed_ctrl.duty(pursuit, line, max_duty=args.max_duty, origin=axle)
    return (result, cones, filled, line, pursuit, duty, corridor_line,
            junction, dropped, survey)


def status_of(base, topo, junction, dropped, survey=None):
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
    )


def parse_args(argv=None):
    parser = build_parser(description="Autonomous cone-junction navigation.")
    parser.add_argument("--route", required=True,
                        help="path to a route file: one 'left' or 'right' per "
                             "junction, in order. See data/routes/")
    parser.add_argument("--push-speed", type=float, default=0.5,
                        help="metres per second to ASSUME the car is being "
                             "pushed at, during --dry-run only. The travel "
                             "estimate normally comes from the commanded duty, "
                             "which a dry run pins to zero -- so without this "
                             "the manoeuvre can never clear its distance floor "
                             "and stage 3 can only ever time out. An "
                             "assumption, not a measurement")
    parser.add_argument("--fill-range-at-junction", type=float, default=1.0,
                        help="metres. While a junction is engaged, geometric "
                             "side assignment is pulled in to this range, so an "
                             "out-of-frame red is not painted into a wall "
                             "across the mouth. 2.0 restores the corridor value")
    args = finalise_args(parser, parser.parse_args(argv))

    if args.no_camera:
        # drive_corridor only warns. Here it is fatal: the fork is exactly the
        # case its own help text calls out, and red cannot be inferred from
        # geometry at all.
        parser.error("--no-camera cannot work at a junction: the gate is red, "
                     "and geometry cannot tell red from a boundary cone. It is "
                     "valid on a plain corridor -- use drive_corridor.py.")
    try:
        args.route_turns = load_route(args.route)
    except ValueError as exc:
        parser.error(str(exc))
    return args


def announce(args):
    turns = ", ".join(args.route_turns)
    print(f"route     {len(args.route_turns)} junction(s): {turns}")
    print(f"           from {args.route}")
    if args.dry_run and args.no_deadman:
        print(f"warning:  --no-deadman: travel is assumed at {args.push_speed} "
              "m/s CONTINUOUSLY, moving or not.\n"
              "          The carried divider, the axis and the exit-distance "
              "floor are fiction\n"
              "          whenever your pace differs -- a paused car burns the "
              "traverse budget\n"
              "          and a resumed one cuts the exit corridor on a stale "
              "divider. For a\n"
              "          pushed stage-3 run, switch the F710 on, drop "
              "--no-deadman, and hold X\n"
              "          exactly while the car is actually moving.")


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

    topo = topo_state.TopoState(RouteCursor(args.route_turns))
    started = time.monotonic()
    axis_rad = 0.0
    duty_now = 0.0
    steer_history = []
    previous_line = None
    travel_m = 0.0
    yaw_delta_rad = 0.0
    last_scan_at = started
    last_report = started
    last_state = topo.state
    loops = 0

    try:
        while True:
            now = time.monotonic()
            if args.duration and now - started >= args.duration:
                break

            armed = deadman.poll() if deadman.present else bool(args.no_deadman)

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
            (result, cones, filled, line, pursuit, duty, corridor_line,
             junction, dropped, survey) = drive_pipeline(
                scan, detection_set, record, intr, args, now, axis_rad,
                topo=topo, previous_line=previous_line, travel_m=travel_m,
                yaw_delta_rad=yaw_delta_rad)
            previous_line = corridor_line
            axis_rad = heading_of(line, default=axis_rad)

            target_duty = duty.duty
            if not armed:
                target_duty = 0.0
            if args.steer_only or args.dry_run:
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

            # What topo_state is told next tick. Open loop, from the duty the
            # car was just given -- see speed_ctrl.DUTY_TO_MPS on how little
            # that number is worth and why it is still enough here.
            #
            # Except in a dry run, where the duty is pinned to zero and that
            # estimate is therefore zero on every tick -- so `travelled_m` never
            # rises, TRAVERSE never clears its distance floor, and the manoeuvre
            # can only ever end by timing out 20 s later with the divider still
            # frozen where it was first seen. Stage 3 of docs/junction-bringup.md
            # is a dry run and expects one clean pass, so a pushed car needs a
            # travel estimate that does not come from the motor. --push-speed is
            # that estimate. It is an assumption about the person pushing, not a
            # measurement, and it is confined to --dry-run: --steer-only runs on
            # a stand where the true answer is zero.
            speed = (args.push_speed if args.dry_run and armed
                     else duty_now * speed_ctrl.DUTY_TO_MPS)
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
            status = status_of(base, topo, junction, dropped, survey)

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
                print(f"  {flag} duty {duty_now:.3f}  steer "
                      f"{status['steer_deg']:+6.1f} deg  "
                      f"{len(line.points)} pts, reach {duty.reach_m:.2f} m  "
                      f"{topo.state} {reds}"
                      + (f"/{topo.turn}" if topo.engaged else "")
                      + (f"  [{duty.reason}]" if duty.reason else ""))
    except KeyboardInterrupt:
        print("\nstopping")
    except Exception:
        if vesc is not None:
            vesc.stop()
        raise
    finally:
        if vesc is not None:
            vesc.close()
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
        if topo.cursor.remaining:
            print(f"warning:  {topo.cursor.remaining} junction(s) of the route "
                  "were never taken")
    return 0


if __name__ == "__main__":
    sys.exit(main())
