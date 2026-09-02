"""OAK-D spatial detection node: runs the custom YOLO blob on-device,
publishes class + XYZ per detection.

Blocked on model/export/, which is empty: there is no .blob and no anchors JSON,
so the OAK-D cannot run the detector on-device at all. See the "Export to the
OAK-D" section of model/dataset/LABELING.md for the tools.luxonis.com round trip.

Two corrections to the plan in this docstring, both settled while building the
fusion layer:

  - Not SPATIAL detection. The Lite's stereo baseline is 7.5 cm, so range error
    grows as z^2 and is metres wide at 5 m, where the LD06 is good to a
    centimetre. Running stereo alongside the NN costs bandwidth on a device that
    negotiates USB 2.0 here, to produce the worst of the three range channels.
    Plain YoloDetectionNetwork; range_stereo stays NaN, which LabeledCone.msg
    explicitly permits.
  - No XYZ per detection. The node publishes boxes; the lidar supplies position.

`model/capture/detectors.py` already defines the interface this node fills --
a `Detection` namedtuple with normalised coordinates and a clip flag -- and
implements the ultralytics and replay backends against it. BlobDetector there is
the seam this node plugs into.
"""


def main():
    raise SystemExit(
        f"{__name__}: not implemented yet -- model/export/ has no .blob. "
        "Use model/capture/fusion_view.py --detector ultralytics.")
