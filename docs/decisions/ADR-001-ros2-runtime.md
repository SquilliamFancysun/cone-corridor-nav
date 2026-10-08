# ADR-001: ROS 2 for the v2 runtime

**Status:** Accepted, conditional on Checkpoint B · **Date:** 2026-10-07 · **Decision:** Will Thatcher · **Analysis and checkpoint criteria:** Claude (proposed; adjust as needed)

## Decision

v2's on-car runtime will be built on ROS 2. The pure algorithm packages from v1 ([`src/cone_perception`](../../src/cone_perception), [`src/cone_nav`](../../src/cone_nav)) carry over unchanged and stay ROS-free. ROS provides transport, timestamps, transforms, recording, drivers and process management.

The commitment gets confirmed or reversed at two checkpoints during setup. They are placed so that reversing costs, at most, the platform setup and some launch files.

## Checkpoint log

| Checkpoint | Status | Date | Notes |
|---|---|---|---|
| A — platform tripwire | Pending | | |
| B — inflection point | Pending | | |

Record each outcome here with the measured numbers. If a checkpoint fails, set this ADR's status to *Reversed* and record the reason, then write a superseding ADR.

## Context

- v1 ran as a single 10 Hz host loop ([`model/capture/drive_junction.py`](../../model/capture/drive_junction.py)), with visualization through the Foxglove SDK ([`model/capture/fusion_view.py`](../../model/capture/fusion_view.py)) and JSONL tick logs. The algorithm layer never imported `rclpy`, and its 741 tests run on a laptop with no ROS installed.
- v1 was first scaffolded as ROS 2 ament packages for the class's `robocar_team2` container ([`hardware-baseline.md`](../hardware-baseline.md)). That packaging was removed on 2026-09-04 because the car ran the tool as a host process ([`README.md`](../../README.md), "Design rule that everything depends on"). Dropping it was about how the class car was deployed, not a judgment on ROS. That history is also why containers are only the fallback here.
- v2 replaces nearly all of that loop's I/O:
  - OAK-D → Insta360 X2
  - LD06 → STL-27L
  - Direct VESC control and the F710 deadman → Pixhawk 6C Mini (ArduPilot Rover) over MAVLink
  - New: IMU, steering encoder, EKF and MPC
- The runtime has to be rewritten either way, which makes now the cheapest time to adopt ROS.
- An EKF plus a delay-compensated MPC is multi-rate and driven by timestamps. v1's single tick doesn't fit that.

## Why ROS 2

- Every message carries a timestamp, and tf2 can look up transforms at a given time. Both are needed for lidar scan deskewing and for delay compensation in the MPC.
- rosbag2 records to MCAP and can replay runs. MATLAB's `ros2bagreader` reads MCAP bags (R2024a and later), so system ID can be done in MATLAB directly from car bags.
- The Pixhawk is reachable through MAVROS or ArduPilot's DDS interface, and community lidar drivers exist.
- Operations get simpler: one launch command starts everything, node health is visible through `ros2 topic hz` and diagnostics, and YAML parameters replace the growing list of CLI flags (v1 needed `--invert-steering --max-range 3.5 --lookahead 0.8 --max-duty 0.05` on every run).
- robot_localization and the Nav2 controllers can serve as reference baselines for the EKF and MPC comparison.
- ROS is the standard vocabulary for GNC and autonomy roles.

## Known costs, accepted

- **Pi 5 platform friction.** ROS 2 targets Ubuntu, while the AI HAT's supported path is Raspberry Pi OS.
- **Failures move off the laptop.** Problems shift from code that can be tested on a laptop to the on-car environment: QoS mismatches, unsourced workspaces, discovery problems.
- **Loss of the deterministic tick.** Mitigated by the fixed-rate tick node (see below).
- **rclpy overhead.** Every message is serialized, and there's no zero-copy transfer between Python nodes.
- **Analysis tools expect JSONL.** [`junction_report.py`](../../model/capture/junction_report.py), [`map_from_log.py`](../../analysis/map_from_log.py) and the [`sim/`](../../sim) harness read JSONL, not bags.
- **Time.** Roughly 2–4 weekends of infrastructure work that isn't GNC work.

## Initial configuration (revisit at Checkpoint A)

- **Distro:** ROS 2 Jazzy on Ubuntu 24.04, rather than Lyrical on 26.04. It's more mature and is the most-tested path for both Hailo and ArduPilot.
- **Networking:** ROS runs on the car only. The laptop connects through foxglove_bridge over WebSocket, so DDS traffic never crosses Wi-Fi.
- **Pixhawk link:** MAVROS first; a pymavlink-based custom node is the fallback.
- **Nodes:** six coarse nodes. Each is a thin wrapper around a plain module:
  - Pixhawk bridge
  - Lidar
  - Camera + detector (one process)
  - Perception → `LabeledConeArray` (the message definition already exists in [`src/cone_msgs`](../../src/cone_msgs/msg))
  - Estimator (own EKF) → odom + TF
  - Guidance/control on a fixed-rate tick
- **Images:** never passed between Python nodes. The camera node publishes detections plus a throttled, compressed preview for viewing.
- **Tick log:** the tick node publishes its per-tick decision record as a message. A bag → JSONL exporter keeps `junction_report.py`, `map_from_log.py` and the sim harness working.
- **Reference stacks:** robot_localization and Nav2 are used as baselines only, never in the control loop.

## Design rule that keeps the exit cheap

v1's rule extends to drivers: algorithm and driver logic live in plain Python modules, and `rclpy` appears only in thin `*_node.py` wrappers. Reverting means replacing the wrappers with a host loop and the Foxglove SDK. Nothing else has to change.

## Checkpoint A: platform tripwire

**When:** now, on the bench, with only the Pi 5 and AI HAT. It runs in parallel with BOM Phases 1–2, and no car is needed.
**Timebox:** one weekend.

**Pass (all of these):**
1. Ubuntu 24.04, Jazzy and HailoRT are installed by a script that reproduces the setup on a fresh SD card.
2. A stock Hailo model-zoo YOLO (`.hef`) runs inside a ROS node at 10 Hz or faster on live camera frames, from the X2 or any USB camera as a stand-in.
   Compiling the cone detector for Hailo is separate work that's needed with or without ROS, so it isn't part of this test.
3. The laptop reaches foxglove_bridge over the travel router and can see those detections.

**If it fails:** spend one more weekend on Raspberry Pi OS with Jazzy in Docker and the Hailo device passed through. If that also fails, revert to ROS-free before writing any car code.

## Checkpoint B: the inflection point

**When:** at the end of BOM Phase 3, once the Pi commands the car through the Pixhawk and before perception (Phase 4).

**Why here:** by this point every ROS-specific integration risk has been tested on the real car, but no algorithm code has been wrapped yet. Everything built up to here (drivers, Pixhawk configuration, wiring) would be needed in a ROS-free build too.

**Rule before B:** write only drivers, bridges, launch files and config. Don't wrap the `src/` algorithms in nodes until B passes.

**Pass (all of these):**
1. **Pixhawk link:** MAVROS on Jazzy shows heartbeat, mode, armed state, RC channels and battery over TELEM2.
2. **Command path:** a Pi node drives steering and throttle through the Pixhawk, gated by the transmitter's arm switch.
3. **Command-loss safety:** killing the command node makes the car stop on its own. Measure and record the stop time.
4. **Bring-up:** a single command, or systemd at boot, starts the whole stack. No per-process terminals.
5. **Recording:** the bag from a teleop run opens in Foxglove and in MATLAB `ros2bagreader`.
6. **Timing headroom:** a fixed-rate tick node holds 50 Hz while the Checkpoint A detector runs alongside it. Log the jitter. CPU stays under about 70%, with no thermal throttling.
7. **Field network:** the laptop sees the whole stack through foxglove_bridge only.

**Budget:** no more than 3 weekends of ROS-specific effort from the start through Checkpoint B. Hardware, firmware and Pixhawk configuration that a ROS-free build would also need don't count toward this.

**If it passes:** commit fully. Wrap the `src/` packages and retire the v1 host loop.
**If it fails:** revert to ROS-free. The drivers and modules carry over.

## After Checkpoint B

From here on, problems get solved inside ROS rather than by leaving it.

Watch item for Phase 4: if perception can't keep up with the lidar rate, consolidate perception into one process or a C++ composable node.

## References

- Pros-and-cons analysis: Claude chat session, 2026-10-07 (not in the repo; this ADR is the record of it)
- v1 runtime and design rule: [`README.md`](../../README.md), [`model/capture/drive_junction.py`](../../model/capture/drive_junction.py), [`model/capture/fusion_view.py`](../../model/capture/fusion_view.py)
- v1 ROS history: [`docs/ai-usage.md`](../ai-usage.md) (2026-08-23 scaffold, 2026-09-04 removal), [`docs/hardware-baseline.md`](../hardware-baseline.md)
- Phase plan: Robocar v2 BOM (Claude Docs artifact, outside the repo)
- [ROS 2 Lyrical Luth release announcement](https://discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021)
- [ArduPilot ROS 2 interfaces](https://ardupilot.org/dev/docs/ros2-interfaces.html)
- [MathWorks `ros2bagreader`](https://www.mathworks.com/help/ros/ref/ros2bagreader.html)
- [Hailo community: AI HAT+ on Ubuntu 24.04](https://community.hailo.ai/t/installing-ai-hat-26-tops-on-raspberry-pi-5-with-ubuntu-24-04/16714)
