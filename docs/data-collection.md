# Data collection runbook

_How to take the car from a box of parts to a session of recorded cone images and
lidar scans. Follow it in order the first time; after that, [Preflight](#4-preflight)
onward is the whole routine._

Three programs run at once on the car, in three SSH panes:

| Pane | Program | Owns | Produces |
|---|---|---|---|
| 1 | DonkeyCar | VESC, F710 | nothing — it is drive-by-wire only |
| 2 | `capture_cones.py` | OAK-D Lite | `~/cone_capture/<session>/` images |
| 3 | `lidar_view.py` | LD06 | `~/lidar_capture/<session>/` scans |

They coexist because they own different devices. The one hard conflict is the
camera: DonkeyCar must run with `CAMERA_TYPE="MOCK"` or `capture_cones.py`
cannot open the OAK-D. `myconfig_capture.py` handles that.

---

## 0. What to bring

- The car, a charged LiPo, and the battery alarm
- **41 cones** for a full track v1 build, or 31 for the minimum: 18 blue,
  18 yellow, 4 orange, 1 green. Counts and what to cut are in
  [`data/layouts/track_v1.md`](../data/layouts/track_v1.md) — cut from the
  straights, never the forks.
- F710 gamepad **and its USB receiver**, switch set to **X**
- Tape measure and chalk or tape for the survey
- Laptop with Foxglove Studio installed, on the same network as the car

---

## 1. On the laptop, before you leave

### Install Foxglove Studio

Download from [foxglove.dev](https://foxglove.dev/download). You want the
desktop app; the lidar tool serves a WebSocket the app connects to directly.

### Commit, then deploy — in that order

The car has no git clone. `deploy.sh` writes `HEAD` into `VERSION`, and both
tools copy that into every `session.json`. Deploy from a dirty tree, or amend
after deploying, and the commit recorded in your data becomes unreachable.

```bash
git status                      # must be clean, including the track spec
git commit -am "..."            # if it is not
cd model/capture && ./deploy.sh      # defaults to the `robocar` ssh host
```

This rsyncs both tools to `~/cone_capture_tool/` and `myconfig_capture.py` to
`~/mycar/`, then prints the three commands you will run on the car.

> **Rule:** commit → deploy → capture. Every time.

---

## 2. On the car, first time only

SSH in and check the two dependencies the lidar tool needs. Everything the
camera tool needs is already in `~/env`.

```bash
ssh robocar
source ~/env/bin/activate

python -V                       # must be 3.10 or newer for foxglove-sdk
pip install foxglove-sdk
python -c "import serial; print('pyserial ok')"
```

If `python -V` is older than 3.10, `foxglove-sdk` will not install. The lidar
tool still records `scans.jsonl` — run it with `--no-live` and view the data on
the laptop afterwards instead of live.

### Find the car's address for Foxglove

```bash
hostname -I                     # note the IP; Foxglove needs it, not the ssh alias
```

---

## 3. Build and survey the track

Build the layout in [`data/layouts/track_v1.md`](../data/layouts/track_v1.md):
a corridor with two Y-junctions, branches at ±25°, ending at the green goal.

Then **survey it before you drive it.** Fix the convention first and write it on
the sheet: origin at the midpoint of the start line, x forward along Corridor A,
y left, metres, measured to cone **base centres**. Same convention as
`cone_msgs/msg/LabeledCone.msg`, so surveyed truth and perception output are
directly comparable. Record positions in
[`data/layouts/track_v1.csv`](../data/layouts/track_v1.csv).

A track that gets rebuilt between the survey and the run is not ground truth.

---

## 4. Preflight

Bus presence is not function. Each check below covers something that has
actually failed on this car. Full detail and the traps behind each one are in
[`hardware-baseline.md`](hardware-baseline.md).

### 4.1 Everything enumerated

```bash
lsusb -t                        # hub on 5000M, four devices
ls -l /dev/serial/by-id/        # CP2102 (lidar) + ChibiOS (VESC)
ls /dev/input/js0               # gamepad
```

Missing lidar? It is the cable before it is anything else. A charge-only
micro-USB cable powers the LD06, spins the motor and lights both LEDs while
carrying no data at all.

### 4.2 Camera link speed

Idle `lsusb` reports the OAK-D at 480 M and that is correct — it is the ROM
bootloader until DepthAI loads firmware. This is the only valid check:

```bash
~/env/bin/python -c "import depthai as dai; d=dai.Device(); \
print(d.getUsbSpeed().name, d.getConnectedCameras())"

# expect: SUPER [<RGB>, <LEFT>, <RIGHT>]
```

### 4.3 Lidar streaming

```bash
cd ~/cone_capture_tool && python lidar_view.py --selftest
```

Expect roughly:

```
19100 B/s  407.0 packets/s  9.97 Hz  30 scans  450 points/scan
CRC: 1221 ok, 0 bad (0.00%)
  matches docs/hardware-baseline.md.
```

It exits non-zero and names the likely cause if anything is off — a slow motor
points at the 5 V rail (the hub, not the Pi), CRC failures point at the cable.

### 4.4 Gamepad buttons

Two controls on the F710 change the mapping, and both must be set before you
probe or the indices will not match what you get while driving:

- the **X/D slider** — use **X**
- the **MODE button** — its green LED must be **off**, otherwise the left stick
  and the D-pad swap and steering arrives as unmapped D-pad events

```bash
python joystick.py --probe-buttons
```

Both tools default to **X (2)**. Expect `X=2, A=0, B=1, start=7`; if yours
differ, pass `--record-button N` to each tool.

> **Never bind either tool to A or B.** All three processes read
> `/dev/input/js0` and every one of them sees the same press. DonkeyCar binds
> **A to emergency_stop** and **B to toggle_manual_recording**, so a record
> toggle on A E-stops the car mid-run, and one on B writes junk MockCamera tubs
> beside your session. X is unbound in DonkeyCar's map, which is why it is the
> default.

DonkeyCar's own bindings, for reference:

| Action | Button |
|---|---|
| Mode switch | start |
| Emergency stop | A |
| Toggle manual recording | B |
| Erase last N records | Y |

### 4.5 Camera preview — and actually look at it

```bash
python capture_cones.py --preview --out /tmp/preview.jpg
```

Then from the laptop:

```bash
scp robocar:/tmp/preview.jpg . && open preview.jpg
```

Check three things: colours look right, framing is what you expect, and a cone
at 5 m is more than ~20 px tall. A session of badly framed images is worth
nothing and you will not find out until labelling.

---

## 5. Start the three panes

Three SSH sessions to the car. Start them in this order.

### Pane 1 — driving

```bash
source ~/env/bin/activate && cd ~/mycar
python manage.py drive --myconfig=myconfig_capture.py
```

Drive-by-wire only: it reads the F710 and drives the VESC, with a mock camera so
it never claims the OAK-D, and `AUTO_RECORD_ON_THROTTLE = False` so it writes no
tubs.

### Pane 2 — camera capture

```bash
source ~/env/bin/activate && cd ~/cone_capture_tool
python capture_cones.py --session-label lot-sun-A
```

**Point the camera at the track while it settles.** It runs auto exposure, white
balance and focus free for 3 seconds, then pins all three for the whole session.
The four classes *are* colours, so an AWB algorithm re-balancing between sun and
shade shifts cone hue underneath the labels.

Read the banner it prints. If `exposure_us` comes back above ~15000 outdoors,
something is wrong and it says so.

`--session-label` should describe the **conditions** (`lot-sun-A`,
`garage-overcast-B`), because a session is the unit train/val/test are split on.

### Pane 3 — lidar

```bash
source ~/env/bin/activate && cd ~/cone_capture_tool
python lidar_view.py --session-label lot-sun-A
```

Use the same label as the camera pane so the two sessions pair up by name.

### Connect Foxglove

In Foxglove Studio: **Open connection → Foxglove WebSocket →
`ws://<car-ip>:8765`** (the IP from step 2, not the ssh alias).

Add a **3D panel** for `/scan` and a **Raw Messages** panel for `/lidar_status`.

| Topic | What |
|---|---|
| `/scan` | The scan, binned for drawing |
| `/scan_raw` | Lossless per-point bearings |
| `/tf` | `base_link` → `lidar` |
| `/lidar_status` | Rotation Hz, packets/s, CRC drop rate, live |

---

## 6. Calibrate the lidar bearing — once per mount

**Do this before the first real session, and again any time the lidar is
remounted.** It takes about a minute and catches an error that is invisible
afterwards.

Put **one cone at 1.0 m, about 45° to the left**, nothing else nearby. Watch the
3D panel:

| What you see | What it means | Fix |
|---|---|---|
| Front-left, 1.0 m | Correct | nothing |
| Front-**right** | Bearing sign is inverted | add `--mirror` |
| Right side, wrong angle | Sensor zero is not forward | set `--angle-offset N` |

Then move the cone to 45° right and confirm it follows.

**Keep the cone off-axis.** A cone directly ahead cannot detect a mirrored scan —
a point on the x axis is its own reflection. The failure mode is a corridor that
comes out mirrored: every range is correct, nothing looks wrong, and the
centreline steers into the wrong boundary at a junction.

Restart pane 3 with the flags you found, and **write them into
[`model/capture/README.md`](../model/capture/README.md)** so nobody re-derives
them at a track. They are also recorded in every `session.json`, so a session
captured with the sign backwards is fixable at a desk rather than re-driven.

While you are here: note where the ~250 mm self-returns land. Those are the
chassis, not obstacles.

---

## 7. Record a session

Press **X** to start and stop. Both tools default to that same button, so one
press begins and ends the camera and lidar sessions together and the two stay
aligned. Both toggle, and both keep one session directory per run of the script
— the button pauses and resumes within it. That is deliberate: a session means
one set of conditions.

If you want them independent, give one of them a different `--record-button`.

Drive the corridor. Vary approach angles and speeds; cover both branches at both
junctions, including the dead ends.

**What good looks like while it runs:**

```
REC ● 20260826_1432_lot-sun-A  248 frames   124.0s          <- camera
REC ● 20260826_1432_lot-sun-A  1204 scans   120.4s   9.98 Hz  <- lidar
```

Watch for rotation drifting off ~10 Hz or a CRC drop rate climbing above zero in
`/lidar_status` — both mean the data is degrading while you record it.

`Ctrl+C` in each pane finishes the session and writes `session.json`.

Roughly per 4-minute session: ~480 frames / 200 MB of images, ~10 MB of scans.
The card has ~28 GB free.

### Right after the run

```bash
vcgencmd get_throttled          # pass: 0x0
vcgencmd pmic_read_adc | grep EXT5V   # pass: above ~4.8 V
```

Bit 16 latches undervolt since boot. Idle readings prove nothing — this is the
check that only means something after real driving load.

Then **write the session notes down while you still remember them**: conditions,
sun angle, surface, anything unusual. They go in
[`model/dataset/DATASET_CARD.md`](../model/dataset/DATASET_CARD.md).

---

## 8. Pull the data off

### Images — culled on the way in

```bash
cd model/dataset
uv run --with pillow --with numpy python prepare_dataset.py --pull robocar
```

This rsyncs sessions down, drops frames not worth a labeler's time (blurry, or
near-duplicates of the last kept frame), and prints a table to paste into
`DATASET_CARD.md`. Rejected frames are **moved** to `<session>/_rejected/`, never
deleted, with a contact sheet so you can check its judgement and put frames back.

Add `--dry-run` first if you want to see what it would do.

### Lidar scans — a plain copy

There is no culling step for scans; every revolution is worth keeping.

```bash
rsync -av robocar:lidar_capture/ ~/lidar_data/
```

Each session gives you:

| File | For |
|---|---|
| `scans.mcap` | Drag into Foxglove and scrub the run |
| `scans.jsonl` | One revolution per line; reads with no dependencies installed |
| `session.json` | Port, angle convention, mount, link health, commit, notes |

Both are gitignored — keep extracted CSVs and per-run notes in
[`data/trials/`](../data/trials/) instead.

---

## Troubleshooting

| Symptom | Most likely cause |
|---|---|
| Lidar missing from `/dev/serial/by-id/` | Charge-only micro-USB cable. Motor spins, LEDs light, no data. Test the cable on a phone. |
| `capture_cones.py` cannot open the camera | DonkeyCar is holding it — check `myconfig_capture.py` has `CAMERA_TYPE="MOCK"` |
| Camera links at 480 M under load | USB 2.0 cable, or not in a blue port. Must be USB-C to A, 3.0+, ≤ 1 m. |
| Foxglove will not connect | Using the ssh alias instead of the IP, or the tool is running with `--no-live` |
| Corridor is mirrored in Foxglove | Bearing sign — redo [step 6](#6-calibrate-the-lidar-bearing--once-per-mount) with an off-axis cone |
| Rotation below 9.9 Hz | The lidar's 5 V rail — that is the powered hub, not the Pi |
| CRC drops climbing | Cable, before it is anything else |
| Gamepad button does nothing | F710 switch is in D, not X — indices differ. Re-probe. |
| `session.json` commit is unreachable | Amended or rebased after deploying. Re-deploy. |

## The four rules that actually bite

1. **Commit, then deploy, then capture.** Otherwise provenance breaks silently.
2. **One process per device.** DonkeyCar must not hold the camera; the ROS
   container's lidar driver must not hold the serial port.
3. **Camera settings lock at settle time.** Point it at the track for those
   three seconds, because the whole session is pinned to whatever it lands on.
4. **Calibrate the lidar bearing with an off-axis cone.** A mirrored scan looks
   completely normal.
