#!/usr/bin/env bash
# Push the capture tool to the car. Usage: ./deploy.sh [ssh-host]
#
# Nothing to install on the far side: depthai, cv2 and donkeycar already live in
# ~/env, and joystick.py deliberately avoids evdev (which is not installed).
set -euo pipefail

HOST="${1:-robocar}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# The car gets the tool by rsync, not by clone, so stamp the commit in by hand.
git -C "$HERE" rev-parse --short HEAD > "$HERE/VERSION" 2>/dev/null || echo "unknown" > "$HERE/VERSION"

rsync -av --delete \
  --exclude='__pycache__' \
  --exclude='.pytest_cache' \
  --exclude='test_*.py' \
  --exclude='myconfig_capture.py' \
  "$HERE/" "$HOST:cone_capture_tool/"

scp "$HERE/myconfig_capture.py" "$HOST:mycar/"

echo
echo "Deployed to $HOST:~/cone_capture_tool (commit $(cat "$HERE/VERSION"))"
echo
echo "On the car, two panes:"
echo "  1)  source ~/env/bin/activate && cd ~/mycar"
echo "      python manage.py drive --myconfig=myconfig_capture.py"
echo "  2)  source ~/env/bin/activate && cd ~/cone_capture_tool"
echo "      python capture_cones.py --session-label <conditions>"
