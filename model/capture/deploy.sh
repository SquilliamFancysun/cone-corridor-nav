#!/usr/bin/env bash
# Push the capture tool to the car. Usage: ./deploy.sh [ssh-host]
#
# Nothing to install on the far side for capture_cones.py: depthai, cv2 and
# donkeycar already live in ~/env, and joystick.py deliberately avoids evdev
# (which is not installed).
#
# lidar_view.py needs foxglove-sdk for the live view and the MCAP:
#   ~/env/bin/pip install foxglove-sdk      (needs Python 3.10+)
# Without it the tool still records scans.jsonl, so a failed install at the
# track does not cost the run.
set -euo pipefail

HOST="${1:-robocar}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# The car gets the tool by rsync, not by clone, so stamp the commit in by hand.
git -C "$HERE" rev-parse --short HEAD > "$HERE/VERSION" 2>/dev/null || echo "unknown" > "$HERE/VERSION"

rsync -av --delete \
  --exclude='__pycache__' \
  --exclude='.pytest_cache' \
  --exclude='test_*.py' \
  --exclude='fixtures' \
  --exclude='myconfig_capture.py' \
  "$HERE/" "$HOST:cone_capture_tool/"

scp "$HERE/myconfig_capture.py" "$HOST:mycar/"

echo
echo "Deployed to $HOST:~/cone_capture_tool (commit $(cat "$HERE/VERSION"))"
echo
echo "On the car, one pane each — the lidar and the camera do not contend, so"
echo "all three can run at once:"
echo "  1)  source ~/env/bin/activate && cd ~/mycar"
echo "      python manage.py drive --myconfig=myconfig_capture.py"
echo "  2)  source ~/env/bin/activate && cd ~/cone_capture_tool"
echo "      python capture_cones.py --session-label lot-sun-A"
echo "  3)  source ~/env/bin/activate && cd ~/cone_capture_tool"
echo "      python lidar_view.py --session-label lot-sun-A"
echo
echo "Then connect Foxglove Studio to ws://$HOST:8765"
echo
echo "The label describes the conditions and becomes the session directory"
echo "name — replace lot-sun-A with your own. Every line above is literal and"
echo "pastes as-is."
