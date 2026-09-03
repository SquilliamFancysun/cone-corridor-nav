"""Closed-loop driving against a synthetic cone field. No hardware, no ROS.

    python -m sim.drive_sim --track curve

`cone_field.py` is deliberately not a simulator -- it answers "place the car
here, what would the sensors return". This module is the loop that closes around
it: take that answer, run the real pipeline over it, steer, move the car, ask
again. It is the only place the perception layers and the control layers meet
before they meet on the track.

## What it is for, and what it is not for

It is for the questions that are expensive at the track and free here: does the
steering sign point the right way through a corner, does the lookahead hold a
line without weaving, does the speed law stall the car mid-corridor, does the
centerline survive being viewed from a car that is moving rather than a car
someone placed. Every gain settled here is a track run not spent.

It is NOT a prediction of how the car will behave. The vehicle model is a
kinematic bicycle -- no tyre slip, no mass, no servo lag, no motor dynamics --
and `DUTY_TO_MPS` below is a guess. Timing, latency and grip all have to be
found on the real car. What transfers is geometry and logic, which is most of
what is currently unwritten and all of what is easy to get backwards.

Everything downstream of `synth_scan` is the production code path: the same
`clustering.cone_candidates`, the same `fusion.associate`, the same
`centerline`, the same `pure_pursuit` and `speed_ctrl` that `drive_corridor.py`
runs on the car.
"""

import argparse
import math
import sys

from cone_nav.control import pure_pursuit, speed_ctrl
from cone_nav.guidance import goal_stop, junction_exec
from cone_nav.corridor.boundary_split import split
from cone_nav.guidance.explore import ExplorePolicy
from cone_nav.guidance.route_exec import RouteCursor, load_route
from cone_nav.topology import dead_end, gate_detect, goal_detect, topo_state
from cone_nav.corridor.centerline import centerline
from cone_nav.corridor import side_assign
from cone_nav.corridor.side_assign import fill_unlabeled, heading_of
from cone_perception import clustering, fusion, label_memory
from cone_perception import extrinsics
from cone_perception.cone_classes import CLASS_NAMES
from cone_perception.geometry import intrinsics_from_hfov

from sim import cone_field
from sim.cone_field import (
    IDENTITY_CALIBRATION,
    Pose,
    cones_in_car_frame,
    synth_detections,
    synth_scan,
)

# Colour name -> class id, built from the message's own ordering rather than
# spelled out again. See cone_perception/cone_classes.py on why that list is
# duplicated once and only once.
CLASS_IDS = {name: i for i, name in enumerate(CLASS_NAMES)}

# The preview the detector actually runs on, from capture_cones.py.
PREVIEW_W, PREVIEW_H = 416, 234

# Duty cycle -> metres per second. Imported rather than kept here, so the sim
# and drive_junction.py cannot disagree about how far the car thinks it went.
DUTY_TO_MPS = speed_ctrl.DUTY_TO_MPS


# Half the car's width plus a cone's base radius. Closer than this and the car
# has hit the cone.
STRIKE_CLEARANCE_M = 0.15

# The outcome of a run that ended the way the course intends. Distinct from
# "reached the end", which is a PROXIMITY test against the layout's own last
# midpoint and would score a car that merely coasted to a halt nearby -- the
# thing this whole feature exists to stop being the answer.
GOAL_OUTCOME = "stopped at the goal"

# How close to the last ideal centerline point counts as finishing. Derived from
# the speed law's own stopping rule plus half a car, not chosen: see the comment
# at the check itself.
FINISH_RADIUS_M = speed_ctrl.MIN_REACH_M + 0.4

# Control period. Matches the LD06's ~10 Hz revolution rate, which is what
# actually clocks the loop on the car -- `drive_corridor.py` runs when a scan
# completes, not on a timer.
DT_S = 0.1

# Ticks between a scan being taken and its steering command reaching the servo.
#
# Without this the sim is only half a model, and misleadingly so: a lag-free
# plant tracks better the shorter the lookahead, so a sweep says "use the
# smallest value" -- and the car built to that advice weaves. Corner-cutting and
# weaving are the two ends of the pure-pursuit tradeoff and only one of them is
# visible without latency.
#
# Two ticks is where the real numbers land: a scan completes over ~100 ms and
# the detector adds ~111 ms of inference (model/capture/README.md:130), so a
# command is acting on a picture roughly 200 ms old.
DEFAULT_LATENCY_TICKS = 2

# Starting lookahead, metres. Chosen for robustness rather than for the best
# number in any one row of the sweep: on the s-bend it is within a centimetre of
# optimal everywhere from 0.6 to 2.4 m/s, so it survives DUTY_TO_MPS -- a guess
# -- being wrong by a factor of four. Below about 0.8 m the car weaves once it
# is moving properly; above about 1.5 m it cuts corners at any speed.
DEFAULT_LOOKAHEAD_M = 1.0

# First-order time constant of the steering servo, seconds. A guess of the same
# character as DUTY_TO_MPS -- it sets how fast commanded angle becomes actual
# angle, and a real servo under load is slower than an unloaded one.
SERVO_TAU_S = 0.08


class Vehicle(object):
    """Kinematic bicycle, stated at the REAR AXLE.

    The axle is the reference point because that is the point pure pursuit's
    geometry is derived about (see `control/pure_pursuit.py`). base_link -- where
    the sensors are, and the frame every perception layer speaks -- is derived
    from it, which is the same relationship the real car has and the same one
    that has to be got right there.
    """

    __slots__ = ("x", "y", "heading_rad", "wheelbase_m", "rear_axle_in_base")

    def __init__(self, wheelbase_m, rear_axle_in_base, x=0.0, y=0.0,
                 heading_rad=0.0):
        self.x = x
        self.y = y
        self.heading_rad = heading_rad
        self.wheelbase_m = wheelbase_m
        self.rear_axle_in_base = rear_axle_in_base

    def base_pose(self):
        """Where base_link is, in the layout frame.

        `rear_axle_in_base` is the axle expressed in base_link, so base_link
        expressed in the axle frame is its negation, rotated into the layout.
        """
        ax, ay = self.rear_axle_in_base[0], self.rear_axle_in_base[1]
        cos_h, sin_h = math.cos(self.heading_rad), math.sin(self.heading_rad)
        return Pose(x=self.x - (ax * cos_h - ay * sin_h),
                    y=self.y - (ax * sin_h + ay * cos_h),
                    heading_deg=math.degrees(self.heading_rad))

    def step(self, delta_rad, speed_mps, dt=DT_S):
        self.x += speed_mps * math.cos(self.heading_rad) * dt
        self.y += speed_mps * math.sin(self.heading_rad) * dt
        self.heading_rad += (speed_mps / self.wheelbase_m) * math.tan(delta_rad) * dt


class Tick(object):
    """One control cycle, kept so a failed run can be read rather than guessed at."""

    __slots__ = ("t", "x", "y", "heading_deg", "cones", "labeled", "filled",
                 "centerline_points", "reach_m", "delta_rad", "duty", "reason",
                 "fallback", "topo", "turn", "dropped", "gate_range_m",
                 "goal_state", "goal_range_m", "goal_reason")

    def __init__(self, **kw):
        for slot in self.__slots__:
            setattr(self, slot, kw.get(slot))


class SimResult(object):
    __slots__ = ("outcome", "ticks", "cones_passed", "cone_total",
                 "struck_cone", "distance_m", "max_abs_steer_deg",
                 "mean_xtrack_m", "max_xtrack_m")

    def __init__(self, outcome, ticks, cones_passed, cone_total, struck_cone,
                 distance_m, max_abs_steer_deg, mean_xtrack_m=0.0,
                 max_xtrack_m=0.0):
        self.outcome = outcome
        self.ticks = ticks
        self.cones_passed = cones_passed
        self.cone_total = cone_total
        self.struck_cone = struck_cone
        self.distance_m = distance_m
        self.max_abs_steer_deg = max_abs_steer_deg
        self.mean_xtrack_m = mean_xtrack_m
        self.max_xtrack_m = max_xtrack_m

    @property
    def completed(self):
        # Both count as finishing the course. They are different achievements
        # and `stopped_at_goal` tells them apart: one is the car recognising the
        # trophy, the other is it running out of corridor near it.
        return self.outcome in ("reached the end", GOAL_OUTCOME)

    @property
    def stopped_at_goal(self):
        return self.outcome == GOAL_OUTCOME

    def __repr__(self):
        return (f"SimResult({self.outcome}, {self.distance_m:.1f} m, "
                f"{self.cones_passed}/{self.cone_total} cones passed)")


def layout_centerline(layout, width_tolerance=0.4,
                      exclude_prefixes=("dead_end",)):
    """The true centerline of a synthetic track, for scoring against.

    Each blue cone is paired with its nearest yellow, and the pair's midpoint is
    a point on the ideal path. Pairs that are not roughly a corridor apart are
    dropped -- at a fork the nearest yellow to a blue can be across the island.

    Ground truth, not perception: this is the line the car SHOULD have driven,
    and it is what makes cross-track error mean anything. The car never sees it.
    """
    from sim.cone_field import CORRIDOR_WIDTH_M
    # The ordering walk below is nearest-unvisited, which at a fork happily
    # wanders up the dead end and then jumps across to the routed branch. A
    # stub is by definition not on the ideal path, so drop it before pairing
    # rather than trying to untangle the walk afterwards.
    layout = [c for c in layout
              if not any(str(c.segment).startswith(pre)
                         for pre in exclude_prefixes)]
    blues = [c for c in layout if c.color == "blue"]
    yellows = [c for c in layout if c.color == "yellow"]
    points = []
    for b in blues:
        if not yellows:
            break
        y = min(yellows, key=lambda y: math.hypot(b.x - y.x, b.y - y.y))
        width = math.hypot(b.x - y.x, b.y - y.y)
        if abs(width - CORRIDOR_WIDTH_M) > width_tolerance:
            continue
        points.append(((b.x + y.x) / 2.0, (b.y + y.y) / 2.0))
    # Order along the path by walking nearest-unvisited from the start.
    if not points:
        return []
    ordered = [min(points, key=lambda p: math.hypot(p[0], p[1]))]
    remaining = [p for p in points if p is not ordered[0]]
    while remaining:
        last = ordered[-1]
        nxt = min(remaining, key=lambda p: math.hypot(p[0] - last[0],
                                                     p[1] - last[1]))
        ordered.append(nxt)
        remaining.remove(nxt)
    return ordered


def cross_track_error(point, line):
    """Perpendicular distance from a point to the ideal centerline polyline."""
    if len(line) < 2:
        return 0.0
    return min(_point_segment_distance(point, line[i], line[i + 1])
               for i in range(len(line) - 1))


def _point_segment_distance(point, a, b):
    """Distance from `point` to the segment ab."""
    abx, aby = b[0] - a[0], b[1] - a[1]
    denom = abx * abx + aby * aby
    if denom < 1e-12:
        return math.hypot(point[0] - a[0], point[1] - a[1])
    t = ((point[0] - a[0]) * abx + (point[1] - a[1]) * aby) / denom
    t = max(0.0, min(1.0, t))
    return math.hypot(point[0] - (a[0] + t * abx), point[1] - (a[1] + t * aby))


def _strike(vehicle, layout, clearance=STRIKE_CLEARANCE_M):
    """The first cone the car body is touching, or None.

    The body is modelled as the segment from the rear axle to base_link at the
    nose -- crude, but it is the difference between "the car passed close" and
    "the car drove over it", which a single point at the axle cannot tell.
    """
    base = vehicle.base_pose()
    axle = (vehicle.x, vehicle.y)
    nose = (base.x, base.y)
    for cone in layout:
        if _point_segment_distance((cone.x, cone.y), axle, nose) < clearance:
            return cone
    return None


def observe(layout, pose, intr, dropout=(), seed=None, detector_hfov=None):
    """Layout + pose -> what the two sensors would report, in base_link."""
    local = cones_in_car_frame(layout, pose)
    scan = synth_scan(local, seed=seed)
    detections = synth_detections(local, intr, CLASS_IDS, dropout=dropout,
                                  hfov_deg=detector_hfov)
    return scan, detections


def pipeline(scan, detections, intr, detection_age_s=0.0, fill_sides=True,
             reference_heading_rad=0.0, fill_in_fov=False,
             topo=None, previous_line=None, travel_m=0.0,
             yaw_delta_rad=0.0, red_memory=None, now_s=0.0,
             goal_latch=None, goal_armed=False):
    """The production perception path, exactly as `fusion_view.pipeline_once`.

    Kept as its own function so a divergence between the sim and the car is a
    diff between two short functions rather than something to be discovered on
    the track.

    Returns `(fusion_result, cones, line, filled, corridor_line, dropped)`.
    `cones` is separate from `fusion_result.cones` because after the fill they
    are no longer the same list, and a status panel reading `result.matched`
    alongside a centerline built from filled cones would overstate what the
    camera did.

    `corridor_line` is the line BEFORE the gate anchor is threaded in. It is
    what `topo_state` reads next tick to decide whether the corridor has been
    reacquired, and the anchored line cannot answer that -- the anchor
    guarantees two points whether or not the car can see a corridor.

    Passing `topo` turns on junction handling. That is the whole of it: three
    steps around the same `centerline` call the plain corridor uses. Passing
    `goal_latch` turns on the goal the same way, and for the same reason -- the
    goal is one more anchor on the same line, not a second controller.
    """
    candidates = clustering.cone_candidates(scan, IDENTITY_CALIBRATION)
    result = fusion.associate(candidates, detections, intr,
                              detection_age_s=detection_age_s)

    # Mirrors drive_junction.drive_pipeline tick for tick: remembered reds
    # first, then the survey on the PRE-fill list, then the band-masked fill.
    # This sim used to detect on the post-fill list and fill at a different
    # range than the car -- the 2026-08-31 gap sweep was biased by exactly
    # that, which is why a divergence here is a bug and not a simplification.
    cones, _remembered = ((red_memory.apply(result.cones, now_s))
                          if red_memory is not None else (result.cones, 0))
    survey = gate_detect.survey(cones, axis_rad=reference_heading_rad)
    if topo is not None:
        topo.update(survey.junction, previous_line, travel_m=travel_m,
                    yaw_delta_rad=yaw_delta_rad)
    engaged = topo is not None and topo.engaged

    # Read off the same PRE-FILL, pre-branch-filter list the reds are read from.
    # The fill only ever paints blue and yellow, so it cannot invent a goal --
    # but keep_branch can DELETE one, and a goal read after it would silently
    # depend on which way the route happened to turn.
    # NOT `reference_heading_rad` -- see goal_detect.trusted_axis. That feedback
    # holds its last value once the centerline dies, and the centerline always
    # dies at the goal.
    goal_survey = goal_detect.survey(
        cones,
        axis_rad=goal_detect.trusted_axis(previous_line, reference_heading_rad))
    if goal_latch is not None:
        goal_latch.update(goal_survey.goal, goal_armed, travel_m=travel_m,
                          yaw_delta_rad=yaw_delta_rad)

    filled = 0
    if fill_sides:
        line_mask = side_assign.gate_line_of(
            topo.divider_xy if engaged else None,
            topo.axis_rad if engaged else reference_heading_rad,
            survey.reds, reference_heading_rad)
        cones, filled = fill_unlabeled(
            cones, reference_heading_rad=reference_heading_rad,
            fill_in_fov=fill_in_fov, gate_line=line_mask)

    dropped = 0
    if topo is not None:
        if topo.engaged and topo.junction is not None:
            junction_exec.select(topo.junction, topo.turn)
            # The divider and axis come from the machine, not from the latched
            # junction: while the reds are out of frame those are carried
            # forward with the car's motion and the latched pair are stale.
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
    if goal_latch is not None and goal_latch.anchor_ok:
        # The same helper, for the same reason: a magenta cone forms no
        # midpoints, so without this the line simply stops at the last cone row
        # and the goal is not on the path at all.
        line = junction_exec.junction_line(line, goal_latch.goal_xy)
    return result, cones, line, filled, corridor_line, dropped, goal_survey


def simulate(layout, wheelbase_m, rear_axle_in_base, lookahead_m=1.5,
             max_duty=speed_ctrl.DEFAULT_MAX_DUTY, max_time_s=60.0,
             dropout=(), seed=None, start=None, stall_ticks=20,
             detection_age_s=0.0, fill_sides=True,
             latency_ticks=DEFAULT_LATENCY_TICKS, servo_tau_s=SERVO_TAU_S,
             fill_in_fov=False, smooth_window=pure_pursuit.SMOOTH_WINDOW,
             route=None, goal_stop_m=goal_stop.STOP_RANGE_M,
             goal_anywhere=False, cursor=None):
    """Drive the layout. Returns a SimResult; never raises on a bad run.

    `route` is a list of turns; passing one turns on junction handling with the
    same `TopoState` and `RouteCursor` `drive_junction.py` uses, so a gain tuned
    here is tuned against the code the car runs.

    `cursor` supplies that decision-maker directly instead, so an
    `ExplorePolicy` can be driven against the same vehicle model -- which is
    where the exploration policy gets tested, since a maze it has not seen is
    exactly what a synthetic layout is. Passing one turns junction handling on
    the same way a route does.

    The goal latch is always running, armed on the same condition the car uses:
    no turn still outstanding. `goal_anywhere` forces it on even while the route
    has turns left, mirroring the car's bring-up flag -- it is there to be tested
    against, not to drive a track with.
    """
    intr = intrinsics_from_hfov(PREVIEW_W, PREVIEW_H)
    vehicle = Vehicle(wheelbase_m, rear_axle_in_base, **(start or {}))

    if cursor is None and route:
        cursor = RouteCursor(route)
    topo = topo_state.TopoState(cursor) if cursor is not None else None
    red_memory = label_memory.RedMemory() if cursor is not None else None
    goal_latch = goal_stop.GoalLatch(stop_range_m=goal_stop_m)
    dead_end_latch = dead_end.DeadEndLatch()
    previous_line = None
    last_travel = 0.0
    last_yaw = 0.0

    ticks = []
    axis_rad = 0.0
    duty_now = 0.0
    # The commands in flight between perception and the servo.
    pipeline_delay = [0.0] * max(0, int(latency_ticks))
    actual_delta = 0.0
    steer_history = []
    stalled = 0
    distance = 0.0
    max_steer = 0.0
    outcome = "ran out of time"

    # Where the corridor actually ends, in layout coordinates. Counting cones
    # passed instead makes the finish line depend on cone SPACING -- a dense
    # track leaves four cones alongside at the end where a sparse one leaves
    # two, so the same successful run reads as complete at 0.75 m and as a
    # failure at 0.5 m.
    ideal = layout_centerline(layout)
    finish = ideal[-1] if ideal else None

    steps = int(max_time_s / DT_S)
    for i in range(steps):
        pose = vehicle.base_pose()
        scan, detections = observe(layout, pose, intr, dropout=dropout,
                                   seed=seed)
        # Armed exactly as `drive_junction.py` arms it: the goal lies at the
        # end of the route by construction, so a magenta seen while a turn is
        # still outstanding is a misread and must not stop the car.
        goal_armed = goal_anywhere or topo is None or topo.cursor.goal_armed
        result, cones, line, filled, corridor_line, dropped, goal_survey = pipeline(
            scan, detections, intr, detection_age_s, fill_sides=fill_sides,
            reference_heading_rad=axis_rad, fill_in_fov=fill_in_fov,
            topo=topo, previous_line=previous_line, travel_m=last_travel,
            yaw_delta_rad=last_yaw, red_memory=red_memory, now_s=i * DT_S,
            goal_latch=goal_latch, goal_armed=goal_armed)
        previous_line = corridor_line
        # The corridor the car is in rotates relative to the car through a
        # bend, so the side split follows last frame's line rather than
        # assuming the corridor runs straight ahead forever.
        axis_rad = heading_of(line, default=axis_rad)

        pursuit = pure_pursuit.steering_angle(
            line.points, lookahead_m, wheelbase_m, origin=rear_axle_in_base)
        # The reach floor stands down for the goal run-in and inside a
        # TRAVERSE, and nowhere else. At the goal, left in place it halts the
        # car 0.64 m from the trophy -- before any stop range can trigger, and
        # unrecoverably, since the scan does not change while the car stands
        # still. In a mouth it deadlocks the manoeuvre instead: duty goes to
        # zero, a stopped car accrues no measured travel, and the traverse
        # times out on a gate it drove through. Same rule, same reason, two
        # places the car has already committed. See goal_stop.py, and keep this
        # identical to drive_junction.drive_pipeline.
        in_mouth = topo is not None and topo.state == topo_state.TRAVERSE
        target = speed_ctrl.duty(pursuit, line, max_duty=max_duty,
                                 origin=rear_axle_in_base,
                                 min_reach_m=(0.0 if goal_latch.run_in or in_mouth
                                              else speed_ctrl.MIN_REACH_M),
                                 min_points=1 if goal_latch.run_in else 2)
        duty_now = speed_ctrl.ramp(
            duty_now, 0.0 if goal_latch.stopped else target.duty)

        # Median-filter the command exactly as drive_corridor.py does, so a
        # gain chosen here is chosen against the same signal the car acts on.
        steer_history, commanded = pure_pursuit.smooth(
            steer_history,
            pursuit.normalised * pure_pursuit.MAX_STEER_RAD if pursuit else None,
            window=smooth_window)
        # Age the command through the perception-to-actuator delay, then let the
        # servo chase it rather than snapping to it.
        pipeline_delay.append(commanded)
        delayed = pipeline_delay.pop(0) if pipeline_delay else commanded
        alpha = 1.0 if servo_tau_s <= 0 else min(1.0, DT_S / servo_tau_s)
        actual_delta += (delayed - actual_delta) * alpha
        delta = actual_delta
        max_steer = max(max_steer, abs(delta))

        ticks.append(Tick(
            t=i * DT_S, x=vehicle.x, y=vehicle.y,
            heading_deg=math.degrees(vehicle.heading_rad),
            cones=len(cones), labeled=result.matched, filled=filled,
            centerline_points=len(line.points),
            reach_m=target.reach_m, delta_rad=delta, duty=duty_now,
            reason=target.reason, fallback=bool(line.single_boundary_fallback),
            topo=topo.state if topo else "", turn=topo.turn if topo else "",
            dropped=dropped,
            gate_range_m=(topo.junction.range_for(topo.turn)
                          if topo and topo.junction and topo.turn else 0.0),
            goal_state=goal_latch.state,
            goal_range_m=goal_latch.range_m or 0.0,
            goal_reason=goal_survey.reason))

        speed = duty_now * DUTY_TO_MPS
        heading_before = vehicle.heading_rad
        vehicle.step(delta, speed, DT_S)
        last_yaw = vehicle.heading_rad - heading_before
        # What topo_state will be told next tick. The same duty-cycle estimate
        # drive_junction.py has to use, so the state machine is exercised
        # against the quality of information it will really get.
        last_travel = speed * DT_S
        distance += last_travel

        hit = _strike(vehicle, layout)
        if hit is not None:
            outcome = f"struck a {hit.color} cone in {hit.segment}"
            return _finish(outcome, ticks, vehicle, layout, hit, distance,
                           max_steer)

        if goal_latch.stopped:
            # The run ended the way the course intends. Checked after the strike
            # test so a car that stopped ON the trophy still reports the strike.
            outcome = GOAL_OUTCOME
            break

        # A car commanding zero for two seconds is not going to start again on
        # its own: the centerline it is refusing to drive on is computed from a
        # scan taken where it is standing.
        # The dead-end latch, on the same inputs and the same holds as the
        # car's. Run before the stall check so a wall is reported as a wall
        # rather than as the generic zero-duty stop it also is -- which is the
        # entire difference this module exists to make.
        # Armed outside the manoeuvre and again past the gate line, and fed
        # the measured travel the re-arm floor counts. Keep identical to
        # drive_junction.drive_pipeline.
        engaged = topo is not None and topo.engaged
        past = topo is not None and topo.past_gate
        dead_end_latch.update(
            corridor_line, cones, oranges=split(cones).dead_ends,
            armed=(not engaged or past) and not goal_latch.run_in,
            origin=rear_axle_in_base, travel_m=last_travel)
        if dead_end_latch.latched:
            if topo is not None:
                resume = topo.cursor.dead_end()
                outcome = (f"dead end: {dead_end_latch.reason}"
                           + (f"; would take {resume}" if resume else
                              "; nothing left to explore"))
            else:
                outcome = f"dead end: {dead_end_latch.reason}"
            break

        stalled = stalled + 1 if duty_now <= 0.0 else 0
        if stalled >= stall_ticks:
            outcome = f"stopped: {target.reason or 'zero duty'}"
            break

        # Tied to MIN_REACH_M rather than a free constant, because stopping
        # short of the last cone row is the DESIGNED behaviour: speed_ctrl
        # refuses to move once the corridor ahead is under MIN_REACH_M, and at
        # the final row there is no more corridor. A finish radius tighter than
        # that scores every correct run as a failure, and does it
        # non-monotonically -- which reads as controller instability and is
        # purely an artefact of the scoring.
        #
        # It is suspended once a goal has been confirmed, because then stopping
        # short is no longer the designed behaviour -- the car is closing on the
        # trophy and has another 1.1 m to travel. Left armed, this radius ends
        # every run at 1.17 m from the goal and the stop can never be observed
        # at all. `confirmed` goes false again if the latch gives up, so a run
        # that loses the goal still has an ending.
        if (finish is not None and not goal_latch.confirmed
                and math.hypot(vehicle.x - finish[0],
                               vehicle.y - finish[1]) < FINISH_RADIUS_M):
            outcome = "reached the end"
            break

    return _finish(outcome, ticks, vehicle, layout, None, distance, max_steer)


def _cones_passed(vehicle, layout):
    """How many cones are behind the car, in its own frame."""
    pose = vehicle.base_pose()
    return sum(1 for cone in layout if pose.to_car((cone.x, cone.y))[0] < 0.0)


def _finish(outcome, ticks, vehicle, layout, hit, distance, max_steer):
    ideal = layout_centerline(layout)
    # Skip the first few ticks: the car starts from rest on whatever line it
    # happens to be on, and that settling transient is not tracking error.
    scored = [cross_track_error((t.x, t.y), ideal) for t in ticks[5:]]
    return SimResult(
        outcome=outcome, ticks=ticks,
        cones_passed=_cones_passed(vehicle, layout), cone_total=len(layout),
        struck_cone=hit, distance_m=distance,
        max_abs_steer_deg=math.degrees(max_steer),
        mean_xtrack_m=sum(scored) / len(scored) if scored else 0.0,
        max_xtrack_m=max(scored) if scored else 0.0)


# --- tracks -------------------------------------------------------------

# What the corridor is actually built at. See the spacing section of
# cone_nav/corridor/side_assign.py: past 1.0 m the sensor overlap holds too few
# cone rows and the car cannot follow the corridor at all.
DEFAULT_SPACING_M = 0.75


def build_track(name, spacing=DEFAULT_SPACING_M):
    if name == "straight":
        return cone_field.straight_corridor(length=8.0, spacing=spacing)
    if name == "curve":
        return cone_field.curved_corridor(radius=4.0, sweep_deg=70.0,
                                          spacing=spacing, left=True)
    if name == "curve-right":
        return cone_field.curved_corridor(radius=4.0, sweep_deg=70.0,
                                          spacing=spacing, left=False)
    if name == "s-bend":
        return cone_field.s_bend_corridor(radius=4.0, sweep_deg=45.0,
                                          spacing=spacing)
    if name == "track_v1":
        return cone_field.track_v1()
    if name == "junction-left":
        return cone_field.track_junction("left", spacing=spacing)
    if name == "junction-right":
        return cone_field.track_junction("right", spacing=spacing)
    if name == "junction-left-blocked":
        # The mirror of junction-left: the branch the car is sent down is the
        # walled stub. `track_junction` walls whichever branch is NOT the
        # argument, so asking for the opposite one produces a layout where a
        # car doing everything correctly still arrives at a wall -- which is
        # the only way to test `dead_end` on a car that is not already lost.
        return cone_field.track_junction("right", spacing=spacing)
    if name == "junction-right-blocked":
        return cone_field.track_junction("left", spacing=spacing)
    raise SystemExit(f"unknown track {name!r}")


TRACKS = ("straight", "curve", "curve-right", "s-bend", "track_v1",
          "junction-left", "junction-right",
          "junction-left-blocked", "junction-right-blocked")


# --- reporting ----------------------------------------------------------

def plot(result, layout, width=72, height=22):
    """A rough ASCII picture of where the car went. No matplotlib on the car."""
    xs = [c.x for c in layout] + [t.x for t in result.ticks]
    ys = [c.y for c in layout] + [t.y for t in result.ticks]
    if not xs:
        return ""
    x0, x1 = min(xs) - 0.5, max(xs) + 0.5
    y0, y1 = min(ys) - 0.5, max(ys) + 0.5
    span_x = max(x1 - x0, 1e-6)
    span_y = max(y1 - y0, 1e-6)

    grid = [[" "] * width for _ in range(height)]

    def put(x, y, ch):
        col = int((x - x0) / span_x * (width - 1))
        # y is left-positive, so rows run the other way to read like a map.
        row = int((y1 - y) / span_y * (height - 1))
        if 0 <= row < height and 0 <= col < width:
            grid[row][col] = ch

    for cone in layout:
        put(cone.x, cone.y, {"blue": "b", "yellow": "y", "red": "R",
                             "orange": "o", "magenta": "M"}.get(cone.color, "?"))
    for tick in result.ticks:
        put(tick.x, tick.y, ".")
    if result.ticks:
        put(result.ticks[-1].x, result.ticks[-1].y, "#")
    if result.struck_cone is not None:
        put(result.struck_cone.x, result.struck_cone.y, "X")
    return "\n".join("".join(row) for row in grid)


def summarise(result):
    lines = [
        f"outcome        {result.outcome}",
        f"distance       {result.distance_m:.2f} m over {len(result.ticks)} ticks",
        f"cones passed   {result.cones_passed}/{result.cone_total}",
        f"peak steer     {result.max_abs_steer_deg:.1f} deg",
        (f"cross-track    {result.mean_xtrack_m * 100:.1f} cm mean, "
         f"{result.max_xtrack_m * 100:.1f} cm max"),
    ]
    if result.ticks:
        labeled = [t.labeled for t in result.ticks]
        cones = [t.cones for t in result.ticks]
        pts = [t.centerline_points for t in result.ticks]
        fallbacks = sum(1 for t in result.ticks if t.fallback)
        filled = sum(t.filled for t in result.ticks) / len(result.ticks)
        lines += [
            (f"cones/tick     {sum(cones) / len(cones):.1f} seen, "
             f"{sum(labeled) / len(labeled):.1f} by camera, "
             f"{filled:.1f} by geometry"),
            f"centerline     {sum(pts) / len(pts):.1f} points/tick",
            f"fallback       {fallbacks} ticks on a single boundary",
        ]
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Closed-loop cone corridor sim.")
    parser.add_argument("--track", default="curve", choices=TRACKS)
    parser.add_argument("--spacing", type=float, default=DEFAULT_SPACING_M,
                        help="cone spacing in metres, as actually laid out")
    parser.add_argument("--lookahead", type=float, default=DEFAULT_LOOKAHEAD_M,
                        help="metres")
    parser.add_argument("--max-duty", type=float,
                        default=speed_ctrl.DEFAULT_MAX_DUTY)
    # Defaults come from the measured car, not from a copy kept here. A sim
    # tuned against different geometry than the car it advises is worse than no
    # sim: it produces confident gains for a vehicle that does not exist.
    parser.add_argument("--wheelbase", type=float,
                        default=extrinsics.WHEELBASE_M,
                        help="metres; defaults to the measured car")
    parser.add_argument("--rear-axle-x", type=float,
                        default=(extrinsics.REAR_AXLE_IN_BASE or (-0.25,))[0],
                        help="metres; the rear axle's x in base_link, negative")
    parser.add_argument("--max-time", type=float, default=60.0)
    parser.add_argument("--dropout", default="",
                        help="comma-separated colours the camera never sees")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--sweep-lookahead", action="store_true",
                        help="try a range of lookaheads and report each")
    parser.add_argument("--latency-ticks", type=int,
                        default=DEFAULT_LATENCY_TICKS,
                        help="ticks between a scan and its command reaching "
                             "the servo; 0 for a lag-free plant")
    parser.add_argument("--servo-tau", type=float, default=SERVO_TAU_S,
                        help="steering servo time constant, seconds")
    parser.add_argument("--no-camera-fill", action="store_true",
                        help="disable geometric side assignment for the near "
                             "cones the camera cannot see")
    parser.add_argument("--route", default=None,
                        help="path to a route file (see data/routes/). Turns "
                             "on junction handling; required for a junction "
                             "track and meaningless without one")
    parser.add_argument("--goal-stop", type=float,
                        default=goal_stop.STOP_RANGE_M,
                        help="metres from base_link at which the magenta goal "
                             "stops the run. Sweep it here before committing a "
                             "value to the car")
    parser.add_argument("--explore", action="store_true",
                        help="decide each junction on the spot instead of "
                             "reading a route, and report the dead end when "
                             "a branch turns out to be a wall. Use with a "
                             "*-blocked track to exercise the search")
    parser.add_argument("--explore-first", default="left",
                        choices=["left", "right"],
                        help="which branch --explore tries first (default: "
                             "left)")
    parser.add_argument("--goal-anywhere", action="store_true",
                        help="arm the goal even while the route has turns left")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args(argv)

    layout = build_track(args.track, args.spacing)
    dropout = tuple(c.strip() for c in args.dropout.split(",") if c.strip())
    axle = (args.rear_axle_x, 0.0, 0.0)
    if args.route and args.explore:
        raise SystemExit("error: pass one of --route and --explore, not both. "
                         "A route says which way\n       to turn; --explore "
                         "decides on the spot.")
    route = load_route(args.route) if args.route else None
    if args.track.startswith("junction-") and route is None and not args.explore:
        raise SystemExit("error: a junction track needs --route or --explore. "
                         "Without one the car\n       has no way to choose a "
                         "branch.\n"
                         "       try --route data/routes/route_v1.txt, "
                         "or --explore")

    if args.sweep_lookahead:
        print(f"{'lookahead':>10}  {'outcome':<28} {'dist':>6}  "
              f"{'xtrack mean':>11}  {'xtrack max':>10}  {'peak':>6}")
        for lookahead in (0.8, 1.0, 1.2, 1.5, 1.8, 2.2, 2.6):
            res = simulate(layout, args.wheelbase, axle, lookahead_m=lookahead,
                           max_duty=args.max_duty, max_time_s=args.max_time,
                           dropout=dropout, seed=args.seed,
                           fill_sides=not args.no_camera_fill,
                           latency_ticks=args.latency_ticks,
                           servo_tau_s=args.servo_tau, route=route,
                           cursor=(ExplorePolicy(first=args.explore_first)
                                   if args.explore else None),
                           goal_stop_m=args.goal_stop,
                           goal_anywhere=args.goal_anywhere)
            print(f"{lookahead:>10.1f}  {res.outcome:<28} "
                  f"{res.distance_m:>5.1f}m  "
                  f"{res.mean_xtrack_m * 100:>10.1f}cm  "
                  f"{res.max_xtrack_m * 100:>9.1f}cm  "
                  f"{res.max_abs_steer_deg:>5.1f}d")
        return 0

    result = simulate(layout, args.wheelbase, axle, lookahead_m=args.lookahead,
                      max_duty=args.max_duty, max_time_s=args.max_time,
                      dropout=dropout, seed=args.seed,
                      fill_sides=not args.no_camera_fill,
                      latency_ticks=args.latency_ticks,
                      servo_tau_s=args.servo_tau, route=route,
                      cursor=(ExplorePolicy(first=args.explore_first)
                              if args.explore else None),
                      goal_stop_m=args.goal_stop,
                      goal_anywhere=args.goal_anywhere)
    print(f"track {args.track}, {len(layout)} cones, "
          f"lookahead {args.lookahead} m, max duty {args.max_duty}\n")
    print(summarise(result))
    if not args.no_plot:
        print()
        print(plot(result, layout))
    return 0 if result.completed else 1


if __name__ == "__main__":
    sys.exit(main())
