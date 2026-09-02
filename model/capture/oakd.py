"""The OAK-D preview pipeline, in one place because it must not drift.

capture_cones.py produced the training frames; fusion_view.py and
detect_view.py run the detector against the live camera. If those disagree on
framing or white balance, the detector sees different colours at inference than
it trained on -- and blue-versus-yellow is exactly the discrimination that
costs. model/README.md calls capture_cones.py the reference configuration; this
module is how the inference-time tools stay a copy of it rather than a fork.

Deliberately importing depthai inside the functions, not at module scope: the
frames a laptop replays through detect_view.py are jpgs, and there is no
depthai wheel for every machine that wants to look at them.
"""

import time

from cone_perception import geometry

# 416x234, the 16:9 preview capture_cones.py trains from. Not 640: the OAK-D
# here negotiates USB 2.0 and the Pi runs the model on its CPU, so the larger
# input is a bad trade twice over. See detectors.DEFAULT_IMGSZ.
DEFAULT_PREVIEW = (416, 234)


def build_camera_pipeline(fps, preview=DEFAULT_PREVIEW):
    """ColorCamera -> preview, mirroring capture_cones.py's geometry exactly.

    Same 416x234 16:9 preview with the aspect ratio kept. That is not a
    coincidence to be tidied up later: the detector must see at inference the
    same framing and the same colours it trained on, and capture_cones.py is
    what produced the training frames.
    """
    import depthai as dai

    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setFps(fps)
    cam.setInterleaved(False)
    cam.setPreviewSize(*preview)
    cam.setPreviewKeepAspectRatio(True)

    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("preview")
    # Latest frame wins. A frame queued behind three stale ones is a label
    # attached to where the car was, not where it is.
    xout.input.setBlocking(False)
    xout.input.setQueueSize(1)
    cam.preview.link(xout.input)

    xin = pipeline.create(dai.node.XLinkIn)
    xin.setStreamName("control")
    xin.out.link(cam.inputControl)
    return pipeline, preview


def lock_camera(q_ctrl, settle_s=2.0):
    """Settle auto exposure/WB, then pin them for the run.

    Same rationale as capture_cones.py: a camera left on auto drifts between
    frames, and blue-versus-yellow is exactly the discrimination that drift
    costs. Focus is the one thing capture_cones.py pins and this does not --
    inference does not care about a soft frame the way a training label does.
    """
    import depthai as dai

    time.sleep(settle_s)
    ctrl = dai.CameraControl()
    ctrl.setAutoExposureLock(True)
    ctrl.setAutoWhiteBalanceLock(True)
    q_ctrl.send(ctrl)


def open_camera(fps, preview=DEFAULT_PREVIEW):
    """(device, (width, height)), or exit with the diagnosis that usually applies."""
    import depthai as dai

    pipeline, size = build_camera_pipeline(fps, preview)
    try:
        device = dai.Device(pipeline)
    except RuntimeError as exc:
        message = str(exc)
        hint = ("       Check the USB-C cable and see docs/hardware-baseline.md.")
        if "ALREADY_IN_USE" in message or "already" in message.lower():
            hint = ("       Something else holds the camera. Only one process can:\n"
                    "       stop capture_cones.py, depth_view.py and fusion_view.py,\n"
                    "       and make sure DonkeyCar is on myconfig_capture.py\n"
                    "       (CAMERA_TYPE=\"MOCK\").")
        raise SystemExit(f"error: cannot open the OAK-D\n       {message}\n{hint}")
    return device, size


def camera_intrinsics(device, width, height):
    """fx, fy, cx, cy for the preview, read from the device.

    There is no intrinsics file in this repo and there does not need to be --
    the OAK-D carries its own factory calibration. The preview is a 16:9 crop of
    a 4:3 sensor, so the vertical crop matters for fy (and therefore range_bbox)
    while the horizontal FOV that bearing depends on is untouched.
    """
    import depthai as dai

    matrix = device.readCalibration().getCameraIntrinsics(
        dai.CameraBoardSocket.RGB, width, height)
    return geometry.Intrinsics(fx=matrix[0][0], fy=matrix[1][1],
                               cx=matrix[0][2], cy=matrix[1][2],
                               width=width, height=height)
