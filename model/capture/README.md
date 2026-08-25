# Cone capture

Gamepad-triggered image capture from the OAK-D Lite, for building the D1
dataset. Runs on the Pi; no ROS involved.

## Why it is not a ROS node

The dataset is an off-car deliverable. Going through `image_transport` and a
rosbag would add a lossy re-encode, couple capture rate to topic rate, and
require the class container to be up — all for no benefit. What the pipeline
here *is* good for is being copied: the DepthAI configuration in
`capture_cones.py` is the reference the deployed `cone_perception/yolo_node.py`
should match, so the detector sees at inference the same colors it trained on.

## The camera-ownership problem

Only one process can hold the OAK-D. DonkeyCar's `OakD` part claims it and
hardcodes 640x480 (`donkeycar/parts/oak_d.py:31`), which is both too small and
the wrong aspect ratio for training data.

So DonkeyCar runs as the **drive-by-wire stack only** — it reads the F710 and
drives the VESC — with `CAMERA_TYPE="MOCK"` so it never opens the camera.
`myconfig_capture.py` does that; everything else in it is copied from the live
`myconfig.py` so the car handles the same as always.

Both processes read `/dev/input/js0` at once, which is fine: joydev gives each
open file its own event stream.

## Deploy

```
./deploy.sh            # defaults to the `robocar` ssh host
```

Copies the tool to `~/cone_capture_tool/`, `myconfig_capture.py` to `~/mycar/`,
and stamps the git commit into `VERSION` (the car has no clone, so the commit
recorded in each `session.json` comes from there).

Nothing to install: `depthai`, `cv2` and `donkeycar` are already in `~/env`, and
`joystick.py` avoids `evdev` (not installed) and `pygame` (wants an SDL video
driver we do not have over SSH).

## Run

Two panes on the car:

```
# 1 — driving. Releases the camera, writes no tubs.
source ~/env/bin/activate && cd ~/mycar
python manage.py drive --myconfig=myconfig_capture.py

# 2 — capture
source ~/env/bin/activate && cd ~/cone_capture_tool
python capture_cones.py --session-label lot-sun-A
```

Press the record button to start and stop; it toggles. Ctrl+C finishes the
session and writes `session.json`.

### Preflight

1. **Plug in the F710 receiver** and switch the pad on. `ls /dev/input/js0`
   must show the device — without it `capture_cones.py` exits with an
   explanation rather than starting.
2. `python joystick.py --probe-buttons`, press the button you want, pass the
   index it prints as `--record-button N`. The F710 renumbers its buttons
   between the X and D switch positions, so probe with the switch where you
   will actually drive.
3. `python capture_cones.py --preview --out /tmp/preview.jpg`, then `scp` it
   back **and look at it**: right colors, expected framing, and is a cone at 5 m
   more than ~20 px tall?

### Settings are locked, on purpose

The four classes *are* colors. An AWB algorithm that re-balances between a sunlit
and a shaded stretch shifts cone hue underneath the labels, and autofocus that
hunts mid-run produces soft frames at random. So the script runs auto exposure,
white balance and focus free for `--settle` seconds, then pins all three for the
rest of the session and records the values in `session.json`.

**Point the camera at the track while it settles** — whatever it lands on is what
the session is locked to.

The startup banner prints the locked values. If `exposure_us` comes back above
~15000 outdoors, something is wrong and the script says so; `--exposure-us` and
`--iso` override. `--auto` disables locking entirely, which is not recommended
for dataset capture.

### Useful flags

| Flag | Default | |
|---|---|---|
| `--session-label` | `session` | Describes the conditions. Becomes the directory name and the Roboflow batch name |
| `--rate` | 2 | Frames saved per second |
| `--camera-fps` | 10 | Sensor rate. Deliberately faster than `--rate` so auto-exposure stays responsive |
| `--record-button` | 0 | Toggle button index |
| `--no-joystick --duration N` | — | Record without a gamepad, for smoke tests |
| `--resolution` | `1080p` | Or `4k` |
| `--notes` | — | Free text into `session.json` |

## Output

```
~/cone_capture/20260826_1432_lot-sun-A/
  frames/000000.jpg ...      1920x1080, ~430 KB each
  session.json               settings, per-frame timestamps, commit, notes
```

One session directory per run of the script; the toggle pauses and resumes
within it. That is deliberate — a session means "one set of conditions", and
sessions are the unit `DATASET_CARD.md` splits train/val/test on.

Roughly 2 Hz x 4 minutes = 480 frames = 200 MB. The card has 29 GB free.

Pull and cull with `../dataset/prepare_dataset.py --pull robocar`.

## Tests

`session.py` is pure Python — naming, the sampling clock, manifest assembly —
so the parts that can be wrong quietly are testable with no car:

```
uv run --with pytest python -m pytest test_session.py -q
```

`joystick.py` and `capture_cones.py` need the hardware; verify those with
`--probe-buttons` and `--preview` on the car.
