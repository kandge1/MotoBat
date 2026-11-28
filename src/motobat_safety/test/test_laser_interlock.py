"""The S4b laser-enable interlock, checked exhaustively. (task 3.5b)"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import laser_interlock as LI  # noqa: E402


def test_fault_free_never_enables_unsafely():
    _, bad = LI.explore(None)
    assert bad is None


def test_every_single_stuck_channel_is_survived_and_detected():
    for f in LI.faults():
        _, bad = LI.explore(f)
        assert bad is None, (f, bad)
        assert LI.detected(f), f


def test_single_channel_design_fails_the_same_check():
    bad = [f for f in LI.faults() if f[1] == 0 and LI.explore(f, dual=False)[1]]
    assert bad, 'the check could not fail'
    assert ('lock', 0, True) in bad            # a lying lock monitor: laser on with the door unlocked
