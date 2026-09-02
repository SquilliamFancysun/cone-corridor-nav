"""Import paths for testing the pure layers without a built ROS workspace.

The packages under ros2/src/ are ament_python packages, so their importable
root is one level below the package directory. Nothing here builds or installs
them -- the README's promise is that `pytest` works on a laptop with no ROS,
and this is what makes that true.

model/capture is on the path because ld06.py lives there and sim/ generates
Scans with it. On the car that arrangement is the natural one: deploy.sh puts
the tool and the pure packages side by side in the same directory.

analysis/ is on it so the off-car scripts can be tested beside the code they
read. They never run on the car and are not deployed, but they are what the
report is made of, and a plot that quietly mis-transforms a frame is worth
catching at the desk.
"""

import os
import sys

_REPO = os.path.dirname(os.path.abspath(__file__))

for _rel in ("ros2/src/cone_perception", "ros2/src/cone_nav",
             "model/capture", "analysis", "."):
    _path = os.path.normpath(os.path.join(_REPO, _rel))
    if os.path.isdir(_path) and _path not in sys.path:
        sys.path.insert(0, _path)
