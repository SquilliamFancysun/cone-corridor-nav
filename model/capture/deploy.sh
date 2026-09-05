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

# detect_view.py and fusion_view.py additionally need torch and ultralytics, and
# installing those the obvious way breaks capture_cones.py: ultralytics pulls
# plain opencv-python into an ~/env that already has opencv-contrib-python, and
# the two fight over the same cv2/ directory. See "Install" under "Camera: the
# detector's view" in README.md for the --no-deps recipe that avoids it.
set -euo pipefail

HOST="${1:-robocar}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

# The car gets the tool by rsync, not by clone, so stamp the source revision in
# by hand. This one line is the ONLY link from a running car back to the source
# that produced it, so it carries three fields, not one:
#
#   <full sha> <branch> <tag|->
#
# The short sha alone was not enough, found the hard way. A collaborator
# recovering a car whose VERSION read "34e18cd" had no branch to fetch, and
# `git fetch origin 34e18cd` does not work: GitHub refuses to serve a bare sha
# it has not advertised (uploadpack.allowReachableSHA1InWant is off there), and
# says so in a message that reads as if the commit does not exist. They
# concluded the commit was gone and rebuilt 2,800 files by rsyncing them back
# off the Pi. The branch name is fetchable; the sha is not. Print both.
DEPLOY_SHA="$(git -C "$HERE" rev-parse HEAD 2>/dev/null || echo unknown)"
DEPLOY_BRANCH="$(git -C "$HERE" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
DEPLOY_TAG_NAME="${DEPLOY_TAG_NAME:-deploy/$(date +%Y%m%d-%H%M%S)}"
[ "${DEPLOY_TAG:-1}" = "1" ] || DEPLOY_TAG_NAME="-"
echo "$DEPLOY_SHA $DEPLOY_BRANCH $DEPLOY_TAG_NAME" > "$HERE/VERSION"

# A commit that exists only on this laptop is a commit nobody else can fetch,
# and that is exactly how the recovery above became necessary. Checked against
# the remote-tracking refs, so it costs no network and works at the track --
# which does mean it is only as fresh as the last fetch. Warn, never block: a
# deploy of unpushed work is a normal thing to do at 3pm on a track day.
REMOTE_HAS_HEAD="$(git -C "$HERE" branch -r --contains HEAD 2>/dev/null)"
if [ "$DEPLOY_SHA" != "unknown" ] && [ -z "$REMOTE_HAS_HEAD" ]; then
  echo
  echo "warning:  $DEPLOY_BRANCH ($(git -C "$HERE" rev-parse --short HEAD)) is not on"
  echo "          any remote branch this clone knows about. The car is about to run"
  echo "          code that nobody else can fetch by name. Push it before the run:"
  echo "              git push -u origin $DEPLOY_BRANCH"
  echo
fi

# rsync copies the working tree, not the commit, so any local edit ships while
# VERSION still names HEAD -- a stamp that describes something other than what
# is running. --porcelain rather than `diff --quiet` because a brand new
# untracked file gets deployed too and `diff` cannot see one. Ignored paths
# (VERSION itself, calibration.json) are excluded by --porcelain already.
if [ -n "$(git -C "$HERE" status --porcelain -- "$HERE" 2>/dev/null)" ]; then
  echo "warning:  uncommitted changes under model/capture/ are being deployed."
  echo "          VERSION will name $DEPLOY_BRANCH but the car gets your working tree."
  echo
fi

# calibration.json is written on the car by `lidar_view.py --calibrate` and
# describes this car's lidar mount. --delete would take it out on every deploy,
# and the next session would then record an unverified bearing sign without
# anyone noticing, so it is excluded from both halves of the sync.
#
# The same trap, found the hard way: --delete removes ANY file the car has that
# the source tree does not, and the trial logs the tools write land right here.
# A deploy between a run and reading its log destroyed 22 s of stage-3 data that
# only existed on the car. Excluded now -- run data is the one thing on the car
# that cannot be regenerated from the repo.
#
# routes/ is on that list for the same reason and it is not obvious why: the
# directory does not exist in this source tree at all -- it is filled from
# data/routes/ by the second rsync below -- so --delete would remove it whole
# and then recreate it. Anything the CAR wrote there dies in between, and
# `drive_junction.py --emit-route` writes exactly there: the route the car
# worked out by exploring, which is run data and cannot be regenerated from the
# repo either. The second rsync carries no --delete, so excluding it here loses
# nothing.
rsync -av --delete \
  --exclude='__pycache__' \
  --exclude='*.jsonl' \
  --exclude='routes' \
  --exclude='*.mcap' \
  --exclude='sessions' \
  --exclude='.pytest_cache' \
  --exclude='.ruff_cache' \
  --exclude='test_*.py' \
  --exclude='conftest.py' \
  --exclude='fixtures' \
  --exclude='calibration.json' \
  --exclude='audio' \
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
    "$REPO/src/$pkg/" "$HOST:cone_capture_tool/$pkg/"
done

# drive_junction.py is useless without a route file, and routes live in the
# repo's data/ tree rather than in this directory. Same reasoning as the two
# packages above: copied so there is one source of truth, and landed beside the
# tool so `--route routes/route_v1.txt` works from where the car runs it.
rsync -av "$REPO/data/routes/" "$HOST:cone_capture_tool/routes/"

scp "$HERE/myconfig_capture.py" "$HOST:mycar/"

# The big binaries are deliberately not in git: weights (6 MB) and driving audio
# (7 MB) both change on their own schedule, and re-pushing them with every code
# deploy would be slow for no reason. They live on GitHub Releases instead.
#
# Fetched onto the car ONLY when the car does not already have them, which is
# what keeps a routine code deploy as fast as it was before. All of it runs
# after the code is already across, and none of it can fail the deploy: a car
# with no audio still drives (drive_junction.py warns and carries on), and a car
# with no weights was already going to say so.
#
# The `--exclude='audio'` on the --delete rsync above is what makes fetching
# once rather than every time safe. Without it each deploy would delete an
# audio/ the repo no longer contains -- the same trap that cost 22 s of
# stage-3 trial data.
fetch_to_car() {  # <release tag> <asset name> <directory on the car>
  local tag="$1" asset="$2" dir="$3" tmp=""

  if ssh "$HOST" "test -s '$dir/$asset'" 2>/dev/null; then
    echo "  $asset -- already on the car in $dir/"
    return 0
  fi
  if ! command -v gh >/dev/null 2>&1; then
    echo "  warning:  $dir/$asset is missing on the car and gh is not installed,"
    echo "            so it cannot be fetched. By hand:"
    echo "              gh release download $tag --pattern '$asset'"
    echo "              scp $asset $HOST:$dir/"
    return 0
  fi

  tmp="$(mktemp -d)"
  if GIT_TERMINAL_PROMPT=0 gh release download "$tag" --pattern "$asset" --dir "$tmp" >/dev/null 2>&1; then
    ssh "$HOST" "mkdir -p '$dir'" 2>/dev/null || true
    if scp -q "$tmp/$asset" "$HOST:$dir/" 2>/dev/null; then
      echo "  $asset -- fetched from $tag, copied to $dir/"
    else
      echo "  warning:  downloaded $asset but could not copy it to $HOST:$dir/"
    fi
  else
    echo "  warning:  could not download $asset from release $tag."
    echo "            Check 'gh auth status', then:"
    echo "              gh release download $tag --pattern '$asset'"
    echo "              scp $asset $HOST:$dir/"
  fi
  rm -rf "$tmp"
}

echo
echo "Release assets (fetched only when the car does not have them):"
fetch_to_car audio-v1   DrivingSound.mp3 cone_capture_tool/audio
fetch_to_car audio-v1   EndSound.mp3     cone_capture_tool/audio
fetch_to_car weights-v3 best.pt          models

# Tag the deploy so this exact revision is fetchable BY NAME forever, even if
# the branch moves on. A tag is the thing a bare sha is not: advertised by the
# remote, so `git fetch origin tag deploy/...` just works.
#
# Runs after every transfer above, deliberately. Pushing needs the network and
# the car does not; at a track on a phone hotspot this can hang or fail, and
# none of that may cost the deploy that already succeeded. GIT_TERMINAL_PROMPT=0
# turns a missing credential into an error instead of a prompt that waits
# forever for a keypress nobody is there to give. Set DEPLOY_TAG=0 to skip.
if [ "${DEPLOY_TAG:-1}" = "1" ] && [ "$DEPLOY_SHA" != "unknown" ]; then
  DEPLOY_TAG_MSG="Deployed to $HOST from $DEPLOY_BRANCH"
  if git -C "$HERE" tag -a "$DEPLOY_TAG_NAME" -m "$DEPLOY_TAG_MSG" 2>/dev/null; then
    PUSH="git -C $HERE push --quiet origin $DEPLOY_TAG_NAME"
    if GIT_TERMINAL_PROMPT=0 $PUSH 2>/dev/null; then
      echo "Tagged $DEPLOY_TAG_NAME and pushed it to origin."
    else
      echo "warning:  tagged $DEPLOY_TAG_NAME locally but could not push it."
      echo "          The car's VERSION names a tag origin does not have. Push it"
      echo "          when there is network:  git push origin $DEPLOY_TAG_NAME"
    fi
  else
    echo "warning:  could not create tag $DEPLOY_TAG_NAME (it may already exist)."
  fi
fi

echo
echo "Deployed to $HOST:~/cone_capture_tool"
echo "  VERSION   $(cat "$HERE/VERSION")"
echo "            (full sha, branch, deploy tag -- fetch the car's code by the"
echo "             branch or the tag, never by the sha)"
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
echo "To check the detector on the live camera — also instead of pane 2:"
echo "      python detect_view.py --weights ~/models/best.pt"
echo
echo "For live fusion + corridor extraction — also instead of pane 2, and it"
echo "needs the lidar too, so instead of pane 3 as well:"
echo "      python fusion_view.py --weights ~/models/best.pt"
echo
echo "To DRIVE the corridor. Stop DonkeyCar first — it holds the VESC — and run"
echo "these three in order. Do not skip ahead; the second one is how the"
echo "steering sign gets checked while the car cannot go anywhere."
echo
echo "  a) hand-pushed, nothing actuates, watch matched/ centerline in Foxglove:"
echo "      python drive_corridor.py --weights ~/models/best.pt --dry-run \\"
echo "             --log ~/trials/dry-\$(date +%H%M).jsonl"
echo
echo "  b) ON A STAND, wheels off the ground. Walk a cone across the front and"
echo "     watch which way they turn. Add --invert-steering if it is backwards:"
echo "      python drive_corridor.py --weights ~/models/best.pt --steer-only"
echo
echo "  c) for real. Hold X to arm, release to stop:"
echo "      python drive_corridor.py --weights ~/models/best.pt \\"
echo "             --max-duty 0.05 --log ~/trials/run-\$(date +%H%M).jsonl"
echo
echo "  If the detector is not cooperating, --no-camera drives on the lidar"
echo "  alone. Correct on a plain corridor, WRONG at a fork."
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
echo "      ws://$CAR_HOST:8768   (detections)"
echo "      ws://$CAR_HOST:8769   (driving)"
echo
echo "The desktop app, not app.foxglove.dev — a browser blocks plain ws:// from"
echo "an HTTPS page as mixed content, which fails the same opaque way."
echo
echo "The label describes the conditions and becomes the session directory"
echo "name — replace lot-sun-A with your own. Every line above is literal and"
echo "pastes as-is."
