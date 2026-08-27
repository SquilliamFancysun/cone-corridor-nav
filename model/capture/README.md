# Cone capture

Two on-car tools, both gamepad-triggered, both running on the Pi with no ROS
involved:

- **`capture_cones.py`** — image capture from the OAK-D Lite, for building the
  D1 dataset.
- **`lidar_view.py`** — live Foxglove view of the LD06 plus scan recording, for
  developing the centerfinding algorithm against real track data.

They own different devices, so they run side by side.

This file is the reference for the two tools. For the procedure — build the
track, preflight, run all three panes, pull the data — see
[`docs/data-collection.md`](../../docs/data-collection.md).

## Deploy

```
./deploy.sh            # defaults to the `robocar` ssh host
```

Copies both tools to `~/cone_capture_tool/`, `myconfig_capture.py` to
`~/mycar/`, and stamps the git commit into `VERSION` (the car has no clone, so
the commit recorded in each `session.json` comes from there).

Commit first, then deploy, then capture. Amending or rebasing after a deploy
makes the commit in every `session.json` unreachable and silently breaks
provenance.

Nothing to install for the camera: `depthai`, `cv2` and `donkeycar` are already
in `~/env`, and `joystick.py` avoids `evdev` (not installed) and `pygame` (wants
an SDL video driver we do not have over SSH). The lidar tool wants one package —
see [Install](#install) below.

## Camera: images for the dataset

### Why it is not a ROS node

The dataset is an off-car deliverable. Going through `image_transport` and a
rosbag would add a lossy re-encode, couple capture rate to topic rate, and
require the class container to be up — all for no benefit. What the pipeline
here *is* good for is being copied: the DepthAI configuration in
`capture_cones.py` is the reference the deployed `cone_perception/yolo_node.py`
should match, so the detector sees at inference the same colors it trained on.

### The camera-ownership problem

Only one process can hold the OAK-D. DonkeyCar's `OakD` part claims it and
hardcodes 640x480 (`donkeycar/parts/oak_d.py:31`), which is both too small and
the wrong aspect ratio for training data.

So DonkeyCar runs as the **drive-by-wire stack only** — it reads the F710 and
drives the VESC — with `CAMERA_TYPE="MOCK"` so it never opens the camera.
`myconfig_capture.py` does that; everything else in it is copied from the live
`myconfig.py` so the car handles the same as always.

Both processes read `/dev/input/js0` at once, which is fine: joydev gives each
open file its own event stream.

### Run

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
   will actually drive — and with the MODE LED off, which swaps the left stick
   and the D-pad.

   **Do not use A or B.** Every process reading `/dev/input/js0` sees the same
   press, and DonkeyCar binds A to emergency_stop and B to its own tub
   recorder. Both tools default to X (2), which DonkeyCar leaves unbound.
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
| `--record-button` | 2 | X. DonkeyCar binds A to E-Stop and B to its tub recorder |
| `--no-joystick --duration N` | — | Record without a gamepad, for smoke tests |
| `--resolution` | `1080p` | Or `4k` |
| `--notes` | — | Free text into `session.json` |

### Output

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

## Lidar: live view and recording

`lidar_view.py` streams the LD06 to Foxglove Studio on the laptop and records
scans on a button press. The live view is always on; the button only gates
recording, because watching the scan is how you decide whether a run is worth
recording at all.

### Why it is not a ROS node either

Same reason as the camera tool, plus one more. A rosbag needs ROS to replay,
and the point of these recordings is the pure-Python replay harness in `sim/` —
Person B develops centerfinding on a laptop with no ROS installed. `ld06.py`
decodes the wire format into a plain `Scan`, which is what
`cone_perception/lidar_cluster.py` consumes; when the nav stack does want the
lidar as a topic, that node is a thin `rclpy` wrapper, not a second decoder.

**Only one process may hold the serial port.** Do not run the container's lidar
driver at the same time as this. Unlike the camera, though, the lidar does not
contend with DonkeyCar, so driving, image capture and lidar capture all run at
once.

### Install

```
~/env/bin/pip install foxglove-sdk        # needs Python 3.10+
```

`pyserial` is already there (donkeycar depends on it). Without `foxglove-sdk`
the tool still records `scans.jsonl` and says so — a failed install at the track
should not cost the data run.

### Preflight

```
python lidar_view.py --selftest
```

Three seconds of link statistics checked against
[`docs/hardware-baseline.md`](../../docs/hardware-baseline.md): ~19 KB/s, ~400
packets/s, 9.9–10.1 Hz, CRC drops near zero. It exits non-zero and names the
likely cause if any of those are off, so it doubles as the lidar half of
re-verification. A spinning motor proves nothing — a charge-only micro-USB cable
powers the LD06 while carrying no data at all.

### Run

```
source ~/env/bin/activate && cd ~/cone_capture_tool
python lidar_view.py --session-label lot-A
```

Then in Foxglove Studio: **Open connection → Foxglove WebSocket →
`ws://robocar:8765`**. Add a 3D panel for `/scan` and a Raw Messages panel for
`/lidar_status`.

| Topic | What |
|---|---|
| `/scan` | The scan you look at, binned to `--bins` equal steps |
| `/scan_raw` | Lossless per-point bearings, so the MCAP is not a downgrade |
| `/tf` | `base_link` → `lidar` from the mount flags |
| `/lidar_status` | Rotation Hz, packets/s, CRC drop rate, live |

### Angle convention — calibrate it, do not reason about it

The one piece of geometry that can be wrong while every number on screen looks
reasonable. Three unknowns feed the bearing:

1. **Native direction.** The LD06's bearing increases clockwise viewed from
   above. Foxglove and ROS (REP-103) are x-forward, y-left, z-up, with angles
   counterclockwise about +z. That is one negation, which the tool applies.
2. **Mount orientation.** Mounting the unit upside down flips its y and z in the
   car frame, so a return the sensor reports at y = +1 lands at y = −1 — the
   same negation again. Inverted plus clockwise-native cancels back out. Two
   independent sign bits, one net sign, which is what `--mirror` flips.
3. **Yaw offset**, if the sensor's zero does not point forward: `--angle-offset`.

Ranges are unaffected by any of this; only bearing is. The failure mode is a
mirrored corridor — a cone on the left appears on the right, every range checks
out, and the centerline steers into the wrong boundary at a junction.

**The check.** One cone at 1.0 m, about 45° to the **left**, nothing else near:

- Front-left (+x, +y) at 1.0 m → correct.
- Front-**right** → the sign is inverted, add `--mirror`.
- Right bearing but rotated → that is the yaw, set `--angle-offset`.

A cone placed *directly ahead* cannot detect the mirror: a point on the x axis
is its own reflection. Keep it off-axis. Then move it to 45° right and confirm
it follows. Record the flags you ended up with here in this file so nobody
re-derives them at a track:

    verified mount flags: (fill in after the first run)

Both flags land in `session.json`, so a session recorded with the sign backwards
is fixable at a desk instead of re-driven.

### Useful flags

| Flag | Default | |
|---|---|---|
| `--session-label` | `session` | Describes the conditions; becomes the directory name |
| `--record-button` | 2 | X — the same button the camera tool uses, so one press records both |
| `--mirror` / `--angle-offset` | — | Set from the cone check above |
| `--mount-x/-y/-z/--mount-yaw` | 0 | Lidar position in `base_link`, metres and degrees |
| `--bins` | 450 | Angular bins for the drawn scan, ~0.8° |
| `--no-joystick --duration N` | — | Record without a gamepad, for smoke tests |
| `--no-live` | — | Record with no Foxglove server |
| `--selftest` | — | Link statistics, then exit |
| `--dump-raw PATH` | — | Raw serial bytes, for building the test fixture |
| `--notes` | — | Free text into `session.json` |

### Output

```
~/lidar_capture/20260826_1432_lot-A/
  scans.mcap     /scan + /scan_raw + /tf + /lidar_status; drag into Foxglove
  scans.jsonl    one revolution per line, no dependencies to read
  session.json   port, angle convention, mount, link health, commit, notes
```

Roughly 2–3 MB/min each at 10 Hz. `scans.jsonl` is deliberately redundant with
`/scan_raw`: it is the zero-dependency path for the replay harness and for a
quick `python -c` on any machine. `--no-jsonl` turns it off.

The first revolution of every session is discarded — the tool joins the stream
mid-rotation, so that one is always partial, and a half scan reads downstream as
a corridor that abruptly ends.

## Tests

`session.py` and `ld06.py` are pure Python — naming, the sampling clock,
manifest assembly, CRC, angle interpolation, the revolution boundary — so the
parts that can be wrong quietly are testable with no car:

```
uv run --with pytest python -m pytest -q
```

`test_ld06.py` synthesizes packets with real CRCs, which only proves the decoder
agrees with itself. Two of its tests are skipped until a recorded fixture exists;
make one on the car and commit it:

```
python lidar_view.py --dump-raw fixtures/ld06_sample.bin --no-joystick --duration 3
```

`joystick.py`, `capture_cones.py` and the serial half of `lidar_view.py` need the
hardware; verify those with `--probe-buttons`, `--preview` and `--selftest` on
the car.
