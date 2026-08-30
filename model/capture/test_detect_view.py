"""The drawing, which is the only part of detect_view.py that can be wrong quietly.

Everything else in that tool is I/O -- a camera, a window, a websocket -- and
fails loudly when it fails. The box geometry does not: normalised centre-form
in, pixel corners out, and an off-by-half there puts every box beside its cone
rather than on it, which reads as a model that nearly works.

The colour check matters for the same reason the tool exists. A box is drawn in
the colour of the class it claims, so the thing being verified by eye is
exactly `BOX_BGR[cls]` landing on that cone's pixels. If this mapping were
permuted the tool would report a mislabel that is not there, and hide one that
is.
"""

import numpy as np
import pytest

import detect_view
from cone_perception.cone_classes import (CLASS_BLUE, CLASS_MAGENTA, CLASS_NAMES,
                                          CLASS_ORANGE, CLASS_RED, CLASS_YELLOW)
from cone_perception.geometry import Detection

cv2 = pytest.importorskip("cv2")


def det(cls, u=0.5, v=0.5, w=0.2, h=0.3, confidence=0.9, clipped=False):
    return Detection(cls=cls, confidence=confidence, u=u, v=v, w=w, h=h,
                     clipped=clipped)


def blank(width=416, height=234):
    return np.zeros((height, width, 3), dtype=np.uint8)


def test_a_box_lands_where_the_normalised_coordinates_say():
    image = blank()
    detect_view.draw_boxes(cv2, image, [det(CLASS_BLUE, u=0.5, v=0.5, w=0.2, h=0.4)])
    # Centre-form: 0.5 +- 0.1 of 416 is x 166..250, 0.5 +- 0.2 of 234 is y 70..164.
    painted = np.argwhere(image.any(axis=2))
    ys, xs = painted[:, 0], painted[:, 1]
    assert 160 <= xs.min() <= 170
    assert 245 <= xs.max() <= 255
    # The label chip sits above the box, so only the bottom edge pins y.
    assert 158 <= ys.max() <= 170


def test_each_class_is_drawn_in_its_own_colour():
    """The whole premise of looking at this tool rather than reading numbers."""
    for cls in (CLASS_BLUE, CLASS_MAGENTA, CLASS_ORANGE, CLASS_RED, CLASS_YELLOW):
        image = blank()
        detect_view.draw_boxes(cv2, image, [det(cls)])
        colours = {tuple(int(c) for c in px) for px in image.reshape(-1, 3)}
        assert detect_view.BOX_BGR[cls] in colours, f"{cls} not drawn in its colour"


def test_the_five_colours_are_distinct():
    """A duplicate would make two classes indistinguishable on screen."""
    assert len(set(detect_view.BOX_BGR.values())) == len(CLASS_NAMES)
    assert set(detect_view.BOX_BGR) == set(range(len(CLASS_NAMES)))


def test_a_clipped_box_is_marked():
    """Clipped is not cosmetic: fusion's bearing and range both lie about one."""
    plain = blank()
    detect_view.draw_boxes(cv2, plain, [det(CLASS_RED, clipped=False)])
    marked = blank()
    detect_view.draw_boxes(cv2, marked, [det(CLASS_RED, clipped=True)])
    assert not np.array_equal(plain, marked)


def test_a_box_running_off_the_frame_does_not_raise():
    """cv2 clips for us; the point is that the tool keeps drawing the rest."""
    image = blank()
    drawn = detect_view.draw_boxes(cv2, image, [
        det(CLASS_YELLOW, u=0.02, v=0.5, w=0.3, h=0.4, clipped=True),
        det(CLASS_BLUE, u=0.98, v=0.05, w=0.3, h=0.4, clipped=True),
    ])
    assert drawn == 2
    assert image.any()


def test_render_upscales_without_moving_the_boxes():
    """The upscale is for the human. A box must still sit on its cone.

    Measured across the row through the box's middle, which is below the label
    chip -- the chip is drawn at a fixed point size on purpose, so it is a
    smaller share of the frame the more the frame is scaled up, and including
    it here would only measure that.
    """
    detections = [det(CLASS_ORANGE, u=0.25, v=0.5, w=0.1, h=0.2)]
    small = detect_view.render(cv2, blank(), detections, scale=1)
    big = detect_view.render(cv2, blank(), detections, scale=3)
    assert big.shape[0] == small.shape[0] * 3
    assert big.shape[1] == small.shape[1] * 3

    def edges(image):
        row = image[image.shape[0] // 2]
        painted = np.flatnonzero(row.any(axis=1))
        return painted.min() / image.shape[1], painted.max() / image.shape[1]

    assert edges(big) == pytest.approx(edges(small), abs=0.005)
    assert edges(small) == pytest.approx((0.20, 0.30), abs=0.01)


def test_weak_boxes_are_drawn_thinner_but_still_drawn():
    """A cone the model is unsure about is the interesting case, not one to hide."""
    strong = blank()
    detect_view.draw_boxes(cv2, strong, [det(CLASS_RED, confidence=0.9)], conf_floor=0.35)
    weak = blank()
    detect_view.draw_boxes(cv2, weak, [det(CLASS_RED, confidence=0.3)], conf_floor=0.35)
    assert weak.any()
    assert np.count_nonzero(weak.any(axis=2)) < np.count_nonzero(strong.any(axis=2))


def test_tally_counts_per_class_and_notices_a_class_that_never_appears():
    tally = detect_view.Tally()
    tally.add([det(CLASS_BLUE), det(CLASS_BLUE), det(CLASS_YELLOW)])
    tally.add([])
    assert tally.frames == 2
    assert tally.boxes == 3
    assert tally.empty == 1
    assert tally.per_class["blue"] == 2
    assert set(tally.missing()) == {"magenta", "orange", "red"}


def test_status_reports_the_weakest_box_and_the_clipped_count():
    tally = detect_view.Tally()
    detections = [det(CLASS_RED, confidence=0.9),
                  det(CLASS_ORANGE, confidence=0.31, clipped=True)]
    tally.add(detections)
    status = detect_view.status_of(tally, fps=8.0, inference_s=0.12,
                                   detections=detections)
    assert status["boxes"] == 2
    assert status["clipped"] == 1
    assert status["lowest_conf"] == pytest.approx(0.31)
    assert status["inference_ms"] == pytest.approx(120.0)
    assert status["red"] == 1


class FakeUltralytics:
    """Just enough of a detector to exercise the class-order gate."""

    def __init__(self, names):
        self.model = type("Model", (), {"names": dict(enumerate(names))})()


def test_class_order_gate_accepts_the_deployed_order():
    assert detect_view.check_weights_class_order(
        FakeUltralytics(CLASS_NAMES)) == tuple(CLASS_NAMES)


def test_class_order_gate_refuses_a_permutation():
    """The silent failure this repo cares most about, caught before anything is drawn."""
    swapped = ("blue", "magenta", "red", "orange", "yellow")
    with pytest.raises(SystemExit) as excinfo:
        detect_view.check_weights_class_order(FakeUltralytics(swapped))
    assert "cone_classes.py" in str(excinfo.value)


def test_class_order_gate_passes_a_backend_that_carries_no_names():
    """--detector blob has no names to check; that is not a failure."""
    assert detect_view.check_weights_class_order(object()) is None
