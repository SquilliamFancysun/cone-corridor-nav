#!/usr/bin/env bash
# Push the capture tool to the car. Usage: ./deploy.sh [ssh-host]
#
# Nothing to install on the far side for capture_cones.py: depthai, cv2 and
# donkeycar already live in ~/env, and joystick.py deliberately avoids evdev
# (which is not installed).
#
# lidar_view.py and depth_view.py need foxglove-sdk for the live view and MCAP:
#   ~/env/bin/pip install foxglove-sdk      (needs Python 3.10+)
# Without it the tool still records scans.jsonl, so a failed install at the
# track does not cost the run.
set -euo pipefail

HOST="${1:-robocar}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

# The car gets the tool by rsync, not by clone, so stamp the commit in by hand.
git -C "$HERE" rev-parse --short HEAD > "$HERE/VERSION" 2>/dev/null || echo "unknown" > "$HERE/VERSION"

# calibration.json is written on the car by `lidar_view.py --calibrate` and
# describes this car's lidar mount. --delete would take it out on every deploy,
# and the next session would then record an unverified bearing sign without
# anyone noticing, so it is excluded from both halves of the sync.
rsync -av --delete \
  --exclude='__pycache__' \
  --exclude='.pytest_cache' \
  --exclude='.ruff_cache' \
  --exclude='test_*.py' \
  --exclude='conftest.py' \
  --exclude='fixtures' \
  --exclude='calibration.json' \
  --exclude='myconfig_capture.py' \
  --exclude='cone_perception' \
  --exclude='cone_nav' \
  "$HERE/" "$HOST:cone_capture_tool/"

# fusion_view.py imports the pure algorithm modules out of the ROS packages, so
# they have to land beside it. Copied rather than vendored: one source of truth,
# and `import cone_perception.clustering` then resolves the same way on the car
# as it does under pytest at a desk (see conftest.py).
#
# These two run AFTER the --delete above, which is why that rsync excludes them
# --- otherwise every deploy would wipe them and immediately put them back, and
# any interruption in between would leave a tool that cannot start.
#
# No --delete of their own: a stale .pyc is harmless, and a partial transfer
# that had already removed the tree would be worse than one that had not.
for pkg in cone_perception cone_nav; do
  rsync -av \
    --exclude='__pycache__' \
    --exclude='test' \
    "$REPO/ros2/src/$pkg/$pkg/" "$HOST:cone_capture_tool/$pkg/"
done

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
echo "First time on this mount — measure the lidar bearing before pane 3."
echo "One cone, two poses, about a minute; the numbers are then reused"
echo "automatically by every later run:"
echo "      python lidar_view.py --calibrate"
echo
echo "For the depth demo instead of pane 2 — depth_view.py and capture_cones.py"
echo "both open the OAK-D, so they are mutually exclusive:"
echo "      python depth_view.py"
echo
echo "For live fusion + corridor extraction — also instead of pane 2, and it"
echo "needs the lidar too, so instead of pane 3 as well:"
echo "      python fusion_view.py --weights ~/models/best.pt"
echo
# $HOST is an ssh alias, and only ssh can resolve one. Printing ws://$HOST here
# hands over a URL that Foxglove answers with a bare "Connection failed", so ask
# ssh what the alias actually points at.
CAR_HOST="$(ssh -G "$HOST" 2>/dev/null | awk '/^hostname /{print $2; exit}')"
CAR_HOST="${CAR_HOST:-$HOST}"
echo "Then connect the Foxglove desktop app to:"
echo "      ws://$CAR_HOST:8765   (lidar)"
echo "      ws://$CAR_HOST:8766   (depth)"
echo "      ws://$CAR_HOST:8767   (fusion)"
echo
echo "The desktop app, not app.foxglove.dev — a browser blocks plain ws:// from"
echo "an HTTPS page as mixed content, which fails the same opaque way."
echo
echo "The label describes the conditions and becomes the session directory"
echo "name — replace lot-sun-A with your own. Every line above is literal and"
echo "pastes as-is."
