"""Calibration reports what a machine can do; these pin the arithmetic
and the reporting, not the machine -- so they must not time anything
real. Every test here injects a trivial worker and runs serially."""
from __future__ import annotations

from dataclasses import dataclass

import pytest

from common_sim.analysis import calibration


@dataclass
class _Outcome:
    error: object = None


def _ok(_job):
    return _Outcome()


def _boom(_job):
    return _Outcome(error="exploded")


def test_measure_counts_completed_matches_and_reports_a_rate():
    throughput = calibration.measure(_ok, list(range(8)), parallel=False)
    assert throughput.sample_size == 8
    assert throughput.failures == 0
    assert throughput.matches_per_second > 0
    assert throughput.wall_seconds > 0


def test_measure_counts_failures_without_raising():
    """A machine that cannot run the workload should say so, not crash
    whatever progress dialog is waiting on it."""
    throughput = calibration.measure(_boom, list(range(4)), parallel=False)
    assert throughput.failures == 4
    assert throughput.matches_per_second == 0.0
    assert "unreliable" in calibration.report(throughput)


def test_measure_rejects_an_empty_batch():
    with pytest.raises(ValueError):
        calibration.measure(_ok, [], parallel=False)


def test_estimate_scales_linearly_with_batch_size():
    t = calibration.Throughput(matches_per_second=2.0, sample_size=10, wall_seconds=5.0)
    assert calibration.estimate_seconds(100, t) == pytest.approx(50.0)
    assert calibration.estimate_seconds(0, t) == pytest.approx(0.0)
    assert t.matches_per_hour == pytest.approx(7200.0)
    assert t.seconds_per_match == pytest.approx(0.5)


def test_estimate_of_a_dead_machine_is_infinite_not_a_zero_division():
    t = calibration.Throughput(matches_per_second=0.0, sample_size=1, wall_seconds=1.0)
    assert calibration.estimate_seconds(100, t) == float("inf")
    assert calibration.humanize(calibration.estimate_seconds(100, t)) == "unknown"


@pytest.mark.parametrize("seconds,expected", [
    (45, "45 seconds"),
    (600, "10 minutes"),
    (3600 * 5, "5.0 hours"),
    (3600 * 48, "2.0 days"),
])
def test_humanize_uses_the_unit_a_person_would_plan_around(seconds, expected):
    assert calibration.humanize(seconds) == expected


def test_report_flags_only_the_batches_worth_moving_to_another_machine():
    """The suggestion has to be selective to mean anything -- a fast
    machine should get no advice at all."""
    fast = calibration.Throughput(matches_per_second=50.0, sample_size=16, wall_seconds=1.0)
    slow = calibration.Throughput(matches_per_second=0.05, sample_size=16, wall_seconds=320.0)
    assert "faster machine" not in calibration.report(fast)
    assert "faster machine" in calibration.report(slow)
