"""The class ids here must equal the ones in LabeledCone.msg.

cone_perception/cone_classes.py spells the ids out because the .msg is not
deployed to the car. That duplication is only safe if something checks it, and
this is that something: a class added to the message and not to the constants
is a failure here, at a desk, instead of a mislabelled cone on the track.

The parser is imported from model/cone_classes.py rather than rewritten, so
this checks the constants against the same reader the dataset pipeline trusts.
"""

import importlib.util
import os

import pytest

from cone_perception import cone_classes

_REPO = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", ".."))
_MSG = os.path.join(_REPO, "ros2", "src", "cone_msgs", "msg", "LabeledCone.msg")


def _model_cone_classes():
    """model/cone_classes.py, loaded by path -- model/ is not a package."""
    path = os.path.join(_REPO, "model", "cone_classes.py")
    spec = importlib.util.spec_from_file_location("_model_cone_classes", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_class_names_match_the_message():
    names = _model_cone_classes().class_names_from_msg(_MSG)
    assert names == cone_classes.CLASS_NAMES


def test_each_constant_has_the_id_the_message_gives_it():
    names = _model_cone_classes().class_names_from_msg(_MSG)
    for expected_id, name in enumerate(names):
        assert getattr(cone_classes, f"CLASS_{name.upper()}") == expected_id


def test_unlabeled_is_not_a_class_constant():
    """The whole reason it is not called CLASS_UNKNOWN.

    model/cone_classes.py requires the CLASS_* ids to be a gapless 0..N-1 run
    matching the Roboflow project. A CLASS_-prefixed sentinel would break
    roboflow_export.py's gate over a message change the dataset knows nothing
    about, so the parser must not see this one.
    """
    names = _model_cone_classes().class_names_from_msg(_MSG)
    assert "unlabeled" not in names
    assert cone_classes.UNLABELED not in range(len(names))


def test_the_message_actually_declares_unlabeled():
    """Guard against the constant existing only in Python."""
    with open(_MSG, encoding="utf-8") as fh:
        assert "uint8 UNLABELED=255" in fh.read()


def test_name_of_covers_every_class_and_the_sentinel():
    for i, name in enumerate(cone_classes.CLASS_NAMES):
        assert cone_classes.name_of(i) == name
    assert cone_classes.name_of(cone_classes.UNLABELED) == "unlabeled"


@pytest.mark.parametrize("attr", ["CLASS_BLUE", "CLASS_YELLOW", "CLASS_RED"])
def test_the_corridor_classes_are_present(attr):
    """Blue, yellow and red are what this plan's corridor layer keys on."""
    assert hasattr(cone_classes, attr)
