"""The cone class ids, for code that runs without a built ROS workspace.

`cone_msgs/msg/LabeledCone.msg` is the source of truth. Inside a built
workspace you would import these from `cone_msgs.msg.LabeledCone`; the harness
in model/capture/ has no such workspace, and neither does pytest on a laptop,
so the constants are spelled out here.

They are spelled out rather than parsed because the alternative is worse. The
.msg is not deployed to the car -- deploy.sh pushes this package's Python and
nothing else -- so a parser would find no file there and fall back to a
hardcoded list anyway, which is the same duplication with a silent failure mode
bolted on. Instead the duplication is explicit and `test_cone_classes.py`
asserts it against the .msg, so a class added to the message and not here is a
test failure at the desk rather than a mislabelled cone on the track.

See model/cone_classes.py for the dataset side of the same contract. That one
does parse the .msg, because it only ever runs off-car where the file is
present.
"""

CLASS_BLUE = 0       # left corridor boundary
CLASS_MAGENTA = 1    # goal
CLASS_ORANGE = 2     # dead end
CLASS_RED = 3        # junction gate, always in pairs
CLASS_YELLOW = 4     # right corridor boundary

# Not a detector class, and deliberately not named CLASS_*.
#
# model/cone_classes.py parses every `uint8 CLASS_* = <id>` line out of the
# .msg and requires the ids to form a gapless 0..N-1 run, then checks that run
# against the Roboflow class list. A CLASS_-prefixed constant here -- at 5 or at
# 255 -- would break roboflow_export.py's gate on a message change that has
# nothing to do with the dataset. The name also reads correctly: this is a
# perception state, not a cone colour.
UNLABELED = 255

# Index order is class id order, which is alphabetical because that is what
# Roboflow assigns. See the note in LabeledCone.msg.
CLASS_NAMES = ("blue", "magenta", "orange", "red", "yellow")

# The two boundary colours, which is the only grouping the corridor layer needs.
BOUNDARY_CLASSES = (CLASS_BLUE, CLASS_YELLOW)


def name_of(cone_class):
    """Human-readable class name, for logs and Foxglove labels."""
    if cone_class == UNLABELED:
        return "unlabeled"
    if 0 <= cone_class < len(CLASS_NAMES):
        return CLASS_NAMES[cone_class]
    return f"class_{cone_class}"
