# Cone capture

Two on-car tools, both gamepad-triggered, both running on the Pi with no ROS
involved:

- **`capture_cones.py`** — image capture from the OAK-D Lite, for building the
  D1 dataset.
- **`lidar_view.py`** — live Foxglove view of the LD06 plus scan recording, for
  developing the centerfinding algorithm against real track data.
- **`depth_view.py`** — live Foxglove view of the OAK-D Lite's stereo depth, as
  a 16-bit depth image, a colorized heat map and a point cloud.

The lidar tool owns the LD06; the two camera tools both want the OAK-D, so those
two are mutually exclusive. Lidar plus either camera tool runs side by side.

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

## Addressing the car

`robocar` is an **ssh alias**, defined in `~/.ssh/config` on the laptop. Only ssh
reads that file — no browser, and no Foxglove, has ever heard of the name. A
Foxglove connection to `ws://robocar:8765` fails with "Connection failed" and no
further explanation, which reads like a dead server rather than a name that does
not exist.

Use the car's real name or its address:

```bash
ssh -G robocar | awk '/^hostname /{print $2}'   # what the alias resolves to
ipconfig getifaddr en0                          # or ask the car: hostname -I
```

On this build that is `ucsdrobocar-148-02.local`. The `.local` name follows the
car across networks; the IP does not, and campus DHCP moves it.

The Foxglove **desktop app** is what these instructions assume. The web app at
app.foxglove.dev is served over HTTPS, and a browser refuses a plain `ws://`
connection from an HTTPS page as mixed content — the same "Connection failed",
for an unrelated reason. `deploy.sh` prints the resolved URL at the end of every
deploy, so it can be pasted rather than reconstructed.

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
`ws://<car-ip>:8765`** — the car's IP or its `.local` name, not the ssh alias
(see [Addressing the car](#addressing-the-car)). Add a 3D panel for `/scan` and
a Raw Messages panel for `/lidar_status`.

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

### Measuring it — `--calibrate`

Do this once per mount, before the first real session, and again any time the
lidar is moved. It takes about a minute.

```
python lidar_view.py --calibrate
```

One cone, two poses. The tool prompts for each, measures ~20 revolutions,
solves for the sign and the yaw together, saves them, and prints the flags:

```
  Place the cone 1.00 m away, 45 deg LEFT of straight ahead.
  Stand clear, then press Enter or button 2:
    sensor bearing  312.94 deg   1.004 m   14.2 pts/scan   +-0.12 deg over 20 scans

  Place the cone 1.00 m away, 45 deg RIGHT of straight ahead.
  Stand clear, then press Enter or button 2:
    sensor bearing  223.01 deg   0.998 m   14.1 pts/scan   +-0.14 deg over 20 scans
--------------------------------------------------------------------
  mirror        True
  angle offset  -8.00 deg
  residual      0.01 deg (the opposite sign: 89.99 deg)
```

**Two poses, not one, and this is the whole reason for the tool.** A single cone
cannot separate a mirrored sign from a yaw offset: any one bearing is fit
exactly by *both* signs, at offsets that differ. Two poses over-determine the
fit by one degree of freedom, which is what makes the residual mean something —
and what makes the losing sign's residual (89.99° above) the evidence that the
question was actually settled. The tool refuses fewer than two poses, and
refuses two that sit within 20° of each other.

Bearings are car bearings: counterclockwise from straight ahead, **left
positive**, the same sense as y-left everywhere else in this repo. Measure them
from the **lidar**, not from the bumper. `--cal-bearings 45,-45,90` adds a third
pose if you want a residual with more to say; `--cal-range` and `--cal-tolerance`
move the range band the cone is looked for in.

The run also reports where the car sees **itself** — the persistent near returns
that are chassis, not obstacle — and checks that arc sits behind the lidar,
which is a free sanity check on the offset it just solved.

### What it writes, and what reads it

`calibration.json`, beside the tool on the car. Every later run of
`lidar_view.py` loads it, prints what it loaded, and records it in
`session.json`; an explicit `--mirror` / `--angle-offset` still overrides it, and
`--no-calibration` ignores it. A run with neither says so, loudly:

```
warning: no calibration found and no --mirror/--angle-offset given.
         The bearing sign is unverified...
```

That is a warning and not an error on purpose — `/scan_raw` and `scans.jsonl`
store the sensor's own bearings untouched, so a session recorded against the
wrong sign is fixable at a desk instead of re-driven. It is still a session
nobody can interpret without going back to the car.

`deploy.sh` excludes `calibration.json` from its `--delete`, so a redeploy does
not wipe it. Copy the printed line here anyway, because the car is not backed up:

    verified mount flags: (fill in after the first --calibrate run)

### `--angle-offset` and `--mount-yaw` are not the same knob

`--angle-offset` rotates the bearings *within* the lidar frame; `--mount-yaw`
rotates the whole `base_link` → `lidar` transform published on `/tf`. Set both
and the scan is rotated twice. The calibrated offset already points the bearings
forward, so leave `--mount-yaw` at 0 — `--mount-x/-y/-z` are the tape-measure
numbers for where the sensor sits, and those you do want. The tool warns if both
rotations are non-zero.

### Useful flags

| Flag | Default | |
|---|---|---|
| `--session-label` | `session` | Describes the conditions; becomes the directory name |
| `--record-button` | 2 | X — the same button the camera tool uses, so one press records both |
| `--calibrate` | — | Measure the two below from cones at known bearings, save, exit |
| `--cal-bearings` | `45,-45` | Car bearings of the poses, left positive. At least two |
| `--cal-range` / `--cal-tolerance` | 1.0 / 0.4 | Metres: where the cone is, and the band searched |
| `--cal-scans` | 20 | Revolutions measured per pose |
| `--mirror` / `--angle-offset` | from `calibration.json` | Override the measured convention |
| `--no-calibration` | — | Ignore `calibration.json` entirely |
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

## Camera: depth

`depth_view.py` streams the OAK-D Lite's stereo depth to Foxglove Studio. It is
the demo tool — it records nothing unless you ask — and it is the quickest proof
that the depth half of the camera works at all.

### It takes the camera from capture_cones.py

Same ownership rule, and this tool is on the *other* side of it: `depth_view.py`
and `capture_cones.py` both open the OAK-D, so only one can run. DonkeyCar still
needs `myconfig_capture.py` (`CAMERA_TYPE="MOCK"`) either way. The second opener
gets `X_LINK_DEVICE_ALREADY_IN_USE`, and the tool says so rather than printing
the raw exception.

The lidar is a separate device, so `lidar_view.py` keeps running throughout —
which is the point of the port split below.

### Install

```
~/env/bin/pip install foxglove-sdk        # needs Python 3.10+
```

Same package `lidar_view.py` needs; `depthai`, `cv2` and `numpy` are already in
`~/env`. Unlike the lidar tool this one has no non-Foxglove fallback, so it exits
with the install line instead of half-running.

### Preflight

```
python depth_view.py --selftest
```

Three seconds of depth against the baseline: USB link speed, frame rate, and
what fraction of pixels got a stereo match. It exits non-zero and names the cause
if any are off, so it is the camera half of re-verification — `SUPER` here is the
`getUsbSpeed()` reading `docs/hardware-baseline.md` says never to infer from
`lsusb`.

A low valid fraction usually is not a fault: stereo needs texture, so a blank
wall or open sky legitimately matches nothing. Point the car at the track.

### Run

```
source ~/env/bin/activate && cd ~/cone_capture_tool
python depth_view.py
```

Then in Foxglove Studio: **Open connection → Foxglove WebSocket →
`ws://<car-ip>:8766`** — the car's IP or its `.local` name, not the ssh alias
(see [Addressing the car](#addressing-the-car)); 8766, not 8765, so the lidar
view keeps its port and both stream at once. Add a **3D panel** for `/depth/points` and an **Image panel** for
`/depth/colorized`.

| Topic | What |
|---|---|
| `/depth/points` | Point cloud in the car frame, colorable by the `range` field |
| `/depth/colorized` | JPEG heat map, near is hot. The one to point at during a demo |
| `/depth/image` | Raw 16-bit millimetres, untouched. The one to measure from |
| `/tf` | `base_link` → `camera` from the mount flags |
| `/depth_status` | USB speed, fps, valid fraction, min/median/max range |

`/depth/colorized` exists because Foxglove's Image panel renders a 16UC1 image
as near-black until you hand-set its value range — a working sensor that looks
broken. Measure from `/depth/image`; demo from `/depth/colorized`.

### Frame convention

Points come out in the car frame — x forward, y left, z up, REP-103 — not the
camera's optical frame, so `/depth/points` and `lidar_view.py`'s `/scan` drop
into one 3D panel and can be compared directly. The conversion happens on the
car; nothing downstream needs to know about optical frames.

Depth is registered to the **right** mono camera by default, which is where its
intrinsics come from. `--align-rgb` registers it to the color camera instead;
that is a narrower FOV and a 4:3 aspect against the mono 16:10, so the point
cloud becomes approximate.

### Range limits

The Lite's stereo baseline is 7.5 cm, which puts the near limit around 35 cm at
400p. `--extended` roughly halves that at the cost of far range; `--subpixel`
trades the other way. They are mutually exclusive on this device and both are
off by default.

### Useful flags

| Flag | Default | |
|---|---|---|
| `--fps` | 10 | Matches the lidar's revolution rate; keeps the stream near 2 MB/s |
| `--max-range` | 10 | Metres. Caps the color ramp and drops farther points |
| `--cloud-step` | 4 | Every Nth pixel into the cloud; 1 is ~16x the data |
| `--mono-resolution` | `400p` | Or `480p`. The Lite's OV7251 does no more |
| `--mount-x/-y/-z` | 0 | Camera position in `base_link`, metres |
| `--mcap PATH` | — | Also record every topic, for replay off-car |
| `--no-cloud` / `--no-colorized` | — | Drop a topic if the link is struggling |
| `--duration N` | — | Stop after N seconds |
| `--selftest` | — | Link and depth statistics, then exit |

### Output

Nothing, unless `--mcap` is given:

```
python depth_view.py --mcap ~/depth_capture/demo.mcap --duration 30
```

Drag that file into Foxglove on the laptop to scrub the run — which is the
easiest way to get a depth figure into the report without the car present.
Existing files are never overwritten.

## Tests

`session.py`, `ld06.py` and `calibrate.py` are pure Python — naming, the
sampling clock, manifest assembly, CRC, angle interpolation, the revolution
boundary, clustering and the bearing solver — so the parts that can be wrong
quietly are testable with no car:

```
uv run --with pytest python -m pytest -q
```

`test_ld06.py` synthesizes packets with real CRCs, which only proves the decoder
agrees with itself. Two of its tests are skipped until a recorded fixture exists;
make one on the car and commit it:

```
python lidar_view.py --dump-raw fixtures/ld06_sample.bin --no-joystick --duration 3
```

`test_calibrate.py` synthesizes scans containing a cone at a bearing chosen from
a known convention and checks the solver recovers it, including the cases where
it must refuse to answer: one pose, two poses too close together, and a cone
dead ahead — a point on the x axis is its own reflection, so an on-axis pair
fits both signs equally and the fit has to say so.

`joystick.py`, `capture_cones.py`, `depth_view.py` and the serial half of
`lidar_view.py` need the hardware; verify those with `--probe-buttons`,
`--preview` and `--selftest` on the car.
