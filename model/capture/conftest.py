"""Make the pure ROS-package modules importable when testing at a desk.

On the car there is no such problem: deploy.sh rsyncs
ros2/src/cone_perception/cone_perception/ and ros2/src/cone_nav/cone_nav/ into
~/cone_capture_tool/ alongside these files, so `import cone_perception.clustering`
resolves from the working directory. In a git checkout they live two levels up
under ros2/src/, so pytest needs to be told.

This is a path fix and nothing more. It deliberately does not add fixtures --
the tests here construct their own scans, which is what makes them readable.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, "..", ".."))

for _pkg in ("cone_perception", "cone_nav"):
    _src = os.path.join(_REPO, "ros2", "src", _pkg)
    if os.path.isdir(_src) and _src not in sys.path:
        sys.path.insert(0, _src)
