# Insta360 X2 on v2: what it delivers, measured

_How the v2 camera actually behaves over USB, and how to switch it between modes.
Decisions about the camera (what it's for, where it mounts, which mode v2 uses)
live in [`v2-setup.md`](v2-setup.md) § Sensors; this file holds the measurements
those decisions rest on._

**Measured 2026-10-14**, camera on a MacBook Air over USB-C. Nothing here has
been run on the Pi yet. Camera firmware version not recorded.

## The camera

The unit enumerates as **"Insta360 ONE X2"** (serial `IXSE50FNMCWME9`); the
other v2 docs call it the X2. It is a **USB 2.0** device (480 Mbps) in every
mode, so it needs none of the SuperSpeed cabling v1's OAK-D did.

That limit is the camera's, not the cable's: it declares `bcdUSB` 0x0200 in both
modes, measured on a USB 3.1 host port. A USB 3 device declares 0x0210 or
higher even when it falls back to a 2.0 link. Any USB-C data cable works, and a
USB 3 cable buys nothing. Choose the cable for length, a right-angle end and
strain relief. Since the camera never drives the SuperSpeed pairs, it adds none
of the 2.4 GHz noise USB 3 links are known for, whatever port or cable it's on.

Its USB identity depends on the USB mode set on the camera:

| USB mode | VID:PID | Vendor string | Appears as | Used by |
|---|---|---|---|---|
| Android | `4255:0001` | Arashi Vision | Vendor-specific interface (class `0xFF`, subclass `0xF0`); not a camera to the OS | Insta360 CameraSDK, [`ai4ce/insta360_ros_driver`](https://github.com/ai4ce/insta360_ros_driver), file transfer |
| Webcam | `2e1a:00c1` | Amba | Standard UVC camera | Any V4L2 / AVFoundation reader; no vendor software |

The IDs differ by mode, so a udev rule matches one mode only.

## Webcam mode

| | Measured |
|---|---|
| Modes offered | **640×360, 1280×720, 1920×1080**, all at 30 fps. No 2:1 equirectangular size |
| Delivered rate | **29.7 fps** at 1920×1080 over 8 s; worst frame gap 68 ms |
| Wire format | Compressed: raw 1080p30 would be ~124 MB/s, beyond USB 2.0. MJPEG or H.264 not yet known |
| Frame contents | **The full 360°, stitched and dewarped on the camera**, cut into two 180° halves stacked: top strip 1920×540, bottom strip 1920×540 |

The two strips join end to end: what leaves the right edge of the top strip
reappears at the left edge of the bottom strip. There are no fisheye circles in
the frame and nothing to stitch on the Pi.

Horizontal detail, against what v1's detector had:

| Source | Pixels per degree (horizontal) |
|---|---|
| v1 OAK-D, 416×234 detector input over 69° | ~6.0 |
| X2 webcam, 1920×1080 (1920 px per 180° strip) | ~10.7 |

What it costs is vertical coverage: each strip is 540 px tall, so the view is a
band around the horizon rather than the whole sphere. A camera mounted high looks
steeply down at near cones, and their bases can fall below the band.

## Android mode (CameraSDK)

Not run here; from Insta360's [Desktop CameraSDK docs](https://github.com/Insta360Develop/Desktop-CameraSDK-Cpp/blob/master/README.md)
and the ai4ce driver's README.

- The SDK lists the ONE X2, and Linux AArch64 as a build target (with Jetson
  among the named toolchains; a Pi 5 isn't named). It has to be applied for at
  <https://www.insta360.com/sdk/apply>.
- `StartLiveStreaming` delivers **unstitched dual fisheye**, H.264 (or H.265),
  at 30 fps; the ai4ce driver lists 1920×960, 2304×1152, 2560×1280 and
  3840×1920 and is verified on the X2. Gyro and exposure data arrive on the same
  `StreamDelegate`.
- Stitching is left to Insta360's MediaSDK, which targets x86_64 with a GPU, or
  to your own code.

This is the only route to the raw fisheye frames. Webcam mode makes it
unnecessary unless the webcam band turns out too narrow or the camera's own
stitch is too poor to calibrate against.

## Still to measure

Each of these feeds the [open item](v2-setup.md#open-items) on the X2.

1. **Band extent.** How many degrees each strip spans vertically, and whether
   the camera holds the horizon level when tilted. Test: camera upright at the
   planned mount height, cones at 0.5, 1 and 2 m, one frame.
2. **Where the strips start.** Whether each strip is one lens's hemisphere,
   which puts the stitch seams at the strip edges. If so, the recommended
   lens-forward mount puts straight ahead in the middle of a strip, clear of
   any seam.
3. **Wire format on the Pi.** `v4l2-ctl --list-formats-ext -d <dev>`. The Pi 5
   has no H.264 hardware decoder, so H.264 means CPU decode; MJPEG is cheaper.
4. **USB power.** Whether the X2 charges from the Pi's port while streaming, and
   how much it draws. Until the Pi's USB current limit is raised (v2-setup
   § Power), its ports share 600 mA.
5. **Heat.** Insta360 says these cameras heat faster while charging and stop or
   shut down at a temperature threshold. Run it for a full session on the bench.
6. **Latency.** Unpublished. Measure frame-to-detection time on the Pi.

## Switching the USB mode

The mode is set on the camera, with the cable out.

1. Unplug the USB cable and power the camera on.
2. Swipe **down** from the top of the touchscreen to open the shortcut menu.
3. Tap **Settings** (the gear) → **General** → **USB Mode**.
4. Choose **Webcam** or **Android**.
5. Plug the cable back in.

Confirm it with `lsusb` on the Pi (or `system_profiler SPUSBDataType` on a Mac)
against the IDs above. In webcam mode it also appears in
`v4l2-ctl --list-devices`, or on a Mac in
`ffmpeg -hide_banner -f avfoundation -list_devices true -i ""`.

If **USB Mode** has no Webcam option, update the firmware from the Insta360
app. A firmware update can also reset the mode to Android.
