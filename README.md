# Cone-Defined Corridor Navigation

MAE 148 final project — a car that drives itself through a course marked out by
traffic cones, where the cones are not obstacles to dodge but the thing that
tells the car where the road is.

> **Scope, stated up front:** our baseline executes a route *provided* to the
> vehicle ("left at the first junction, right at the second"); autonomous route
> planning is a nice-to-have.

## Repository layout

```
model/       CV model development (off-car): dataset, training, OAK-D blob export
ros2/src/    Everything that runs on the car — a ROS2 workspace source tree
  cone_msgs/         Custom message definitions (the interfaces between layers)
  cone_perception/   yolo_node, lidar_cluster, associate  → labeled cone list
  cone_nav/          corridor / topology / guidance / control layers
sim/         Synthetic cone-field generator + replay harness (no hardware needed)
analysis/    Perception characterization, trial analysis, plotting scripts
data/        Surveyed layouts (ground truth) and trial logs
docs/        Report, slides, AI usage log, verified hardware baseline
```

**Collecting data?** [`docs/data-collection.md`](docs/data-collection.md) is the
step-by-step runbook: build the track, preflight the hardware, run the three
panes, pull the sessions off the car.

Mapping to the proposal's deliverables: `model/` → D1–D2, `analysis/` → D3 + D6,
`ros2/src/` → D4, `data/layouts/` → D5, `docs/` → D7 + D11.

## Design rule that everything depends on

**Algorithm code never imports `rclpy`.** Corridor extraction, cone pairing, the
gate state machine, graph code, planners, and pure pursuit are plain Python
modules inside `cone_nav/`. ROS nodes are thin wrappers that subscribe, call the
pure function, and publish. This is what lets:

- Person B develop and unit-test everything against `sim/` on a laptop with no
  ROS installed
- pytest run on any machine: `pip install -e ros2/src/cone_nav && pytest`
- the replay harness feed recorded logs through the exact code that runs on-car

## Working on it

### Which branch is live

`hardware-baseline` is the branch the car runs. Everything on-car — the capture
tool, perception, nav, the trial logs — lands there. `main` is the earlier
milestone and is a week or more behind it at any given time; branch new work off
`hardware-baseline`, not off `main`.

Do not confuse the branch with [`docs/hardware-baseline.md`](docs/hardware-baseline.md),
which is the *hardware* record — port map, cabling, device checks. The names
collide, and grepping the repo for "hardware-baseline" finds the document, never
the branch. That is a large part of how a collaborator once concluded the
branch did not exist.

### First time on this repo

The repo is private, so a fresh clone needs credentials before anything else:

```bash
gh auth login            # then: git clone / git fetch work normally
git fetch origin
git switch hardware-baseline
```

If `git fetch` prompts for a password or returns 403, stop and fix auth. Do not
work around it by copying files off the car — the car is a deploy target, not a
source of truth, and it has no git clone at all (see
[`model/capture/deploy.sh`](model/capture/deploy.sh)).

### Recovering the revision a car is running

`deploy.sh` stamps `model/capture/VERSION` on the car with three fields:

```
<full sha> <branch> <deploy tag>
```

Fetch it **by branch or by tag, never by the sha**:

```bash
git fetch origin hardware-baseline          # the branch it was deployed from
git fetch origin tag deploy/20260901-155652 # or the exact deploy
```

`git fetch origin <sha>` does not work against GitHub. It refuses to serve a
commit it has not advertised, and the error reads as though the commit does not
exist. It is fetchable — you just have to ask for it by a name.

**On the Mac (or any laptop, no ROS required):**

```bash
pip install -e ros2/src/cone_nav
pytest ros2/src/cone_nav
python -m sim.generate --help    # synthetic cone fields
```

**On the Pi (inside the class ROS2 container):** clone the repo, mount `ros2/src`
into the container's workspace `src/`, then:

```bash
colcon build --packages-select cone_msgs cone_perception cone_nav
source install/setup.bash
```

`build/`, `install/`, and `log/` are gitignored — build products stay in the
container, never in the repo.

**Before a data run:** [`docs/data-collection.md`](docs/data-collection.md) walks
the whole procedure end to end. It leans on
[`docs/hardware-baseline.md`](docs/hardware-baseline.md), which records the
verified USB port map, the cables each link actually requires, and a four-step
check that every device is not just enumerated but working — re-run that after
any change to cabling, ports, or power.

## What is deliberately NOT in git

- **Dataset images** (`model/dataset/images/`) — large binaries; the labels,
  splits, and dataset card ARE in git. See `model/README.md` for where images live.
- **Model weights** (`*.pt`, `*.onnx`, `*.blob`) — attach the trained model to a
  GitHub Release instead; training configs and curves ARE in git.
- **Driving audio** (`model/capture/audio/`) — 7 MB of mp3 on the `audio-v1`
  Release. `deploy.sh` fetches it onto the car when the car does not have it;
  the code that plays it IS in git.
- **Rosbags** (`*.db3`, `*.mcap`) — trial CSV summaries and analysis outputs ARE
  in git.
