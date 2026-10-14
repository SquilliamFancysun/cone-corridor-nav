# CLAUDE.md

This repo holds two cars. **v1** is the MAE 148 cone-corridor car, submitted as
the [`demo-v1`](https://github.com/SquilliamFancysun/cone-corridor-nav/tree/demo-v1)
tag and described in [`README.md`](README.md). **v2** is a personal build in
progress: a Tamiya XV-02RS Pro with a VESC, a Pixhawk 6C Mini (ArduPilot Rover)
and a Raspberry Pi 5. ROS 2 is the planned runtime, conditional on ADR-001's
two checkpoints; the Pi still runs Raspberry Pi OS today.

## Before any v2 work, read

1. [`docs/v2-setup.md`](docs/v2-setup.md): every v2 decision with its status,
   the configuration values, the phase plan, superseded choices and open items.
2. [`docs/decisions/`](docs/decisions/): ADRs. ADR-001 (ROS 2, conditional on
   two checkpoints) sets what may be built before Checkpoint B.
3. [`docs/v2-harness.md`](docs/v2-harness.md): wiring, connectors and the
   ArduPilot/VESC settings that depend on them.
4. [`docs/field-network.md`](docs/field-network.md): how the laptop reaches the car.

Items 2–4 live on open PRs #6, #7 and #4 until they merge. If they aren't on
your branch, read them with `git show origin/docs/adr-001-ros2-runtime:docs/decisions/ADR-001-ros2-runtime.md`,
`git show origin/docs/v2-harness:docs/v2-harness.md` and
`git show origin/docs/v2-pointer:docs/field-network.md`.

If those files disagree, the order is ADR, then harness, then `v2-setup.md`.
A row marked **Recommended** or **Open** in `v2-setup.md` isn't settled: ask
before building on it.

## Rules

- **Algorithm code never imports `rclpy`.** Algorithm and driver logic are plain
  Python modules; `rclpy` appears only in thin `*_node.py` wrappers. Keep
  `src/` ROS-free.
- **Until ADR-001 Checkpoint B passes**, write only drivers, bridges, launch
  files and config. Don't wrap `src/` packages in nodes.
- **Timestamps at measurement, SI units everywhere.** Raw servo microseconds and
  motor RPM stay inside the Pixhawk bridge.
- **One ground tie between the drive and stack packs** (the CAN cable's GND).
  Never suggest a second path, such as the VESC's USB into the Pi on the car.
- **Locking connectors only.** No Raspberry Pi GPIO, no new friction-fit joints.
- **Never write the AS5600's one-time-programmable registers** (ZPOS, MPOS, MANG).
- **Li-ion, not LiPo,** for any cutoff or charge value.
- **Decisions are recorded, not edited away.** A reversed ADR gets a new ADR.
  A changed decision in `v2-setup.md` gets its status and date updated and the
  old choice moved to Superseded. Measured values go in the harness bring-up
  log or the ADR checkpoint log.
- **The BOM is a Claude Docs page outside the repo.** Don't recreate it here;
  say what should change in it.
- **Secrets stay out.** The field Wi-Fi password is not in the repo.

## Working in the repo

- Tests: `python -m pytest` from the repo root (the README records 741
  passed, 2 skipped). `pytest.ini` limits it to `src`, `model/capture` and `analysis`,
  and `conftest.py` puts the packages on the path, so ROS isn't needed; pytest
  and numpy are. The sim runs separately with `python -m pytest sim`, which has
  3 known failures in `test_drive_sim.py` (see the README).
- Work on a branch and open a PR. GitHub is the system of record: merged PRs on
  `main`, with open PRs carrying work in progress.
- Commit subjects are plain sentences that say what changed and why (see
  `git log`).
