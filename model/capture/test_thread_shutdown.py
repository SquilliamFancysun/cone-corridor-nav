"""The two background threads must survive being joined.

Both tools stop their threads inside `finally`, after the run is over, so a
crash here lands on top of the summary of a run that had already succeeded --
and under torch it comes out as a C++ std::terminate and a SIGABRT rather than
a Python traceback, which reads like the car broke rather than the shutdown did.

The bug was `self._stop = threading.Event()`. threading.Thread has a private
`_stop()` METHOD which `join()` calls via `_wait_for_tstate_lock` once the
thread has finished; the attribute shadowed it, and join() raised
`TypeError: 'Event' object is not callable`. Observed on the car on
drive_junction.py, latent in drive_corridor.py and fusion_view.py, all three of
which join their reader.

It survived every test run at a desk because Python 3.13 REMOVED
`Thread._stop`, and the car runs 3.11.2 where it is still there and still
called. That is the real lesson: checking against THIS interpreter's Thread is
not enough, because the interpreter that matters is the car's. So the test
below checks the names against a written-down list of Thread's private methods
across versions, which catches `_stop` on 3.13 too -- where the attribute is
harmless locally and fatal on deploy.
"""

import threading
import time

import pytest

from drive_corridor import ThreadedDetector
from fusion_view import LidarReader


# Private methods CPython's Thread has carried, whether or not THIS version
# still has them. A name here is unsafe on some interpreter we deploy to even
# when it is free on the one running the tests.
RESERVED = frozenset({
    "_stop", "_bootstrap", "_bootstrap_inner", "_wait_for_tstate_lock",
    "_reset_internal_locks", "_set_ident", "_set_native_id", "_set_tstate_lock",
    "_delete", "_after_fork",
})


class QuietHandle(object):
    """A serial handle that yields nothing, slowly enough not to spin a core."""

    def __init__(self):
        self.reads = 0

    def read(self, _size):
        self.reads += 1
        time.sleep(0.002)
        return b""


class EmptyQueue(object):
    def tryGet(self):
        return None


def _run_and_join(thread):
    thread.start()
    for _ in range(200):
        if thread.is_alive():
            break
        time.sleep(0.005)
    thread.stop()
    thread.join(timeout=2.0)
    return thread


# --- the invariant, stated directly -------------------------------------

@pytest.mark.parametrize("make", [
    lambda: LidarReader(QuietHandle()),
    lambda: ThreadedDetector(detector=None, queue=EmptyQueue()),
])
def test_no_attribute_shadows_a_method_of_thread(make):
    """Thread's private methods are called by the stdlib ON OUR INSTANCE, so an
    attribute sharing a name with one replaces machinery we do not own.

    Checked against RESERVED as well as this interpreter, because the
    interpreter that matters is the car's: `_stop` is gone in 3.13 and present
    in 3.11, so a local-only check passes here and the car aborts.
    """
    thread = make()
    ours = set(vars(thread)) - set(vars(threading.Thread()))
    live = {n for n in ours if callable(getattr(threading.Thread, n, None))}
    shadowed = sorted(live | (ours & RESERVED))
    assert not shadowed, (
        f"{type(thread).__name__} shadows Thread method(s) {shadowed}; "
        "the stdlib calls those on our instance and join() will raise")


# --- and end to end, which is how it was found --------------------------

def test_the_lidar_reader_can_be_stopped_and_joined():
    reader = _run_and_join(LidarReader(QuietHandle()))
    assert not reader.is_alive()


def test_the_detector_thread_can_be_stopped_and_joined():
    detector = _run_and_join(ThreadedDetector(detector=None,
                                              queue=EmptyQueue()))
    assert not detector.is_alive()


def test_stopping_twice_is_harmless():
    """`finally` runs stop() on paths that already stopped."""
    reader = _run_and_join(LidarReader(QuietHandle()))
    reader.stop()
    reader.join(timeout=1.0)
    assert not reader.is_alive()
