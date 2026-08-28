"""Rules tests for oracle 03: the things that must never be true.

Oracle 02 proves itself at runtime by wedging the robot against a wall. Oracle
03 mostly cannot: to make real robot code publish a NaN, drive its motors while
disabled, or report a negative piece count, you would have to break it on
purpose, which is oracle 01's trade and it comes out the same way. So these
tests are the proof that the detectors work, and `InvariantMonitor` keeps its
constructor free of any NetworkTables requirement precisely so they can run in
CI, where there is no pyntcore and no JVM.

Every detector is tested twice: once that it fires on the violation, and once
that it stays quiet on a clean snapshot. The second half is the one that
matters for a fuzz campaign -- a detector nobody trusts gets muted, and a muted
detector and a missing one are worth the same.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, replace

import pytest

from bridge.oracles import (
    ERROR,
    NO_COUNT,
    WARNING,
    InvariantMonitor,
    InvariantThresholds,
    Snapshot,
    _OneShot,
)


# ---------------------------------------------------------------------------
# stand-ins
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class FakePose:
    """What the detectors actually need of a pose: three numbers and a metric.

    Deliberately not `robot_state.Pose2d`. That class needs pyntcore to import,
    and the point of this file is that oracle 03's judgement does not.
    """

    x: float
    y: float
    theta: float = 0.0

    def distance_to(self, other) -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def __str__(self) -> str:
        return f"({self.x:.2f}, {self.y:.2f} m, {math.degrees(self.theta):.0f} deg)"


@dataclass(frozen=True)
class FakeSpeeds:
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0

    @property
    def linear(self) -> float:
        return math.hypot(self.vx, self.vy)


@dataclass(frozen=True)
class FakeLimits:
    max_speed_mps: float = 4.0
    max_omega_rad_s: float = 8.0


def clean(**overrides) -> Snapshot:
    """A snapshot of an ordinary robot midway through an ordinary match."""
    base = Snapshot(
        t=1.0,
        enabled=True,
        truth=FakePose(8.0, 4.0, 0.5),
        commanded=FakeSpeeds(2.0, 0.5, 1.0),
        extras=[FakePose(3.0, 1.6), FakePose(3.0, 6.4)],
        held=12,
        drive_current=9.4,
        battery_volts=12.4,
        flywheel_setpoint_rpm=0.0,
    )
    return replace(base, **overrides)


#: The debounced invariants read the monotonic clock, so proving them means
#: really waiting. These tests run the durations five times faster than they
#: ship, which turns twenty seconds of sleeping into four and changes nothing
#: about the logic under test -- the shipped numbers are pinned separately, by
#: `test_the_shipped_durations_are_what_the_readme_says`, so a threshold cannot
#: drift without a test noticing.
FAST = dict(off_field_seconds=0.1, disabled_current_seconds=0.2,
            command_overrange_seconds=0.1)


def monitor(**kwargs) -> InvariantMonitor:
    thresholds = kwargs.pop(
        "thresholds", InvariantThresholds(piece_capacity=40, **FAST)
    )
    kwargs.setdefault("limits", FakeLimits())
    return InvariantMonitor(state=None, thresholds=thresholds, **kwargs)


def kinds(findings) -> list[str]:
    return [f.kind for f in findings]


def hold(mon: InvariantMonitor, snapshot: Snapshot, seconds: float, step: float = 0.05):
    """Feed the same snapshot for `seconds`, returning everything it produced.

    The duration-debounced invariants need real elapsed time, because `_Latch`
    reads the monotonic clock rather than the snapshot's `t`. Sleeping is
    honest here and the durations involved are under two seconds.
    """
    out = []
    deadline = time.monotonic() + seconds
    t = snapshot.t
    while time.monotonic() < deadline:
        t += step
        out.extend(mon.evaluate(replace(snapshot, t=t)))
        time.sleep(step)
    return out


# ---------------------------------------------------------------------------
# a clean run is a quiet one
# ---------------------------------------------------------------------------


def test_an_ordinary_snapshot_produces_nothing():
    mon = monitor()
    assert mon.evaluate(clean()) == []
    assert mon.evaluate(clean(t=1.05)) == []


def test_a_whole_quiet_second_stays_quiet():
    """The debounced detectors must not fire merely because time passed."""
    mon = monitor()
    assert hold(mon, clean(), 0.3) == []


def test_every_invariant_is_active_when_it_is_given_what_it_needs():
    assert monitor().inactive == []


# ---------------------------------------------------------------------------
# 1. not a number
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "field, value",
    [
        ("truth", FakePose(float("nan"), 4.0)),
        ("truth", FakePose(8.0, float("inf"))),
        ("truth", FakePose(8.0, 4.0, float("nan"))),
        ("commanded", FakeSpeeds(float("nan"), 0.0, 0.0)),
        ("commanded", FakeSpeeds(0.0, 0.0, float("nan"))),
        ("flywheel_setpoint_rpm", float("nan")),
        ("battery_volts", float("nan")),
    ],
)
def test_a_non_finite_number_anywhere_is_an_error(field, value):
    found = monitor().evaluate(clean(**{field: value}))
    assert kinds(found) == ["not-a-number"]
    assert found[0].severity == ERROR


def test_a_nan_in_an_extra_robot_is_caught_too():
    """The extras are held to the same invariants, and are named in the finding."""
    found = monitor().evaluate(clean(extras=[FakePose(3.0, 1.6), FakePose(float("nan"), 6.4)]))
    assert kinds(found) == ["not-a-number"]
    assert "extra1" in found[0].message


def test_a_nan_fires_on_the_very_first_sample():
    """No debounce. A NaN is frequently gone by the next read."""
    assert kinds(monitor().evaluate(clean(battery_volts=float("nan")))) == ["not-a-number"]


def test_a_persistent_nan_is_one_finding_not_twenty():
    mon = monitor()
    bad = clean(battery_volts=float("nan"))
    assert len(mon.evaluate(bad)) == 1
    assert mon.evaluate(replace(bad, t=1.05)) == []
    assert mon.evaluate(replace(bad, t=1.10)) == []


def test_a_nan_rearms_after_the_value_recovers():
    mon = monitor()
    bad = clean(battery_volts=float("nan"))
    assert len(mon.evaluate(bad)) == 1
    assert mon.evaluate(clean(t=1.05)) == []
    assert len(mon.evaluate(replace(bad, t=1.10))) == 1


def test_an_impossible_battery_reading_is_not_a_brownout():
    """Oracle 02 owns brownouts. This is the sensor itself being broken."""
    assert kinds(monitor().evaluate(clean(battery_volts=97.0))) == ["not-a-number"]
    assert kinds(monitor().evaluate(clean(battery_volts=-3.0))) == ["not-a-number"]
    # A genuine brownout is a plausible voltage and belongs to the other oracle.
    assert monitor().evaluate(clean(battery_volts=6.4)) == []


# ---------------------------------------------------------------------------
# 2. off the field
# ---------------------------------------------------------------------------


def test_a_robot_past_the_border_is_an_error():
    mon = monitor()
    found = hold(mon, clean(truth=FakePose(-1.5, 4.0)), 0.2)
    assert kinds(found) == ["off-the-field"]
    assert found[0].severity == ERROR


def test_an_extra_robot_off_the_field_is_reported_by_name():
    mon = monitor()
    found = hold(mon, clean(extras=[FakePose(3.0, 1.6), FakePose(3.0, 20.0)]), 0.2)
    assert kinds(found) == ["off-the-field"]
    assert "extra1" in found[0].message


def test_a_bumper_against_the_wall_is_not_off_the_field():
    """The margin exists for contact tolerance, not for driving."""
    mon = monitor()
    assert hold(mon, clean(truth=FakePose(0.1, 0.1)), 0.2) == []


def test_off_the_field_waits_before_it_fires():
    """One sample outside is a torn read; half a second of it is an ejection."""
    mon = monitor()
    assert mon.evaluate(clean(truth=FakePose(-1.5, 4.0))) == []


def test_two_robots_off_the_field_are_two_findings():
    """The latch is per robot, so one ejection does not mask another."""
    mon = monitor()
    found = hold(mon, clean(truth=FakePose(-1.5, 4.0), extras=[FakePose(3.0, 20.0)]), 0.2)
    assert sorted(kinds(found)) == ["off-the-field", "off-the-field"]


# ---------------------------------------------------------------------------
# 3. teleport
# ---------------------------------------------------------------------------


def test_a_pose_jump_no_drivetrain_could_make_is_an_error():
    mon = monitor()
    assert mon.evaluate(clean(t=1.0, truth=FakePose(8.0, 4.0))) == []
    found = mon.evaluate(clean(t=1.05, truth=FakePose(12.0, 4.0)))  # 4 m in 50 ms
    assert kinds(found) == ["teleport"]
    assert found[0].severity == ERROR


def test_driving_fast_is_not_teleporting():
    """5 m/s is a quick robot, and must never be reported as one that jumped."""
    mon = monitor()
    assert mon.evaluate(clean(t=1.00, truth=FakePose(8.00, 4.0))) == []
    assert mon.evaluate(clean(t=1.05, truth=FakePose(8.25, 4.0))) == []
    assert mon.evaluate(clean(t=1.10, truth=FakePose(8.50, 4.0))) == []


def test_a_long_sample_gap_is_not_evidence_of_anything():
    """Dividing a real displacement by a wrong dt is how this invents faults."""
    mon = monitor()
    assert mon.evaluate(clean(t=1.0, truth=FakePose(8.0, 4.0))) == []
    assert mon.evaluate(clean(t=4.0, truth=FakePose(14.0, 4.0))) == []


def test_a_teleport_is_measured_per_robot():
    """Our robot standing still must not mask an extra being flung."""
    mon = monitor()
    assert mon.evaluate(clean(t=1.00, extras=[FakePose(3.0, 1.6)])) == []
    found = mon.evaluate(clean(t=1.05, extras=[FakePose(9.0, 1.6)]))
    assert kinds(found) == ["teleport"]
    assert "extra0" in found[0].message


def test_the_first_sample_of_a_robot_cannot_be_a_teleport():
    """There is nothing to compare it against, and an extra appears mid-run."""
    assert monitor().evaluate(clean(extras=[FakePose(16.0, 7.9)])) == []


def test_an_episode_reset_forgets_where_everyone_was():
    """A phase boundary legitimately rearranges the field."""
    mon = monitor()
    mon.evaluate(clean(t=1.0, truth=FakePose(8.0, 4.0)))
    mon.reset_episode()
    assert mon.evaluate(clean(t=1.05, truth=FakePose(2.0, 1.6))) == []


# ---------------------------------------------------------------------------
# 4. driven while disabled
# ---------------------------------------------------------------------------


def test_motors_drawing_current_while_disabled_is_an_error():
    mon = monitor()
    found = hold(mon, clean(enabled=False, drive_current=22.0), 0.28)
    assert kinds(found) == ["driven-while-disabled"]
    assert found[0].severity == ERROR


def test_a_disabled_robot_drawing_nothing_is_fine():
    mon = monitor()
    assert hold(mon, clean(enabled=False, drive_current=0.0), 0.28) == []


def test_coasting_briefly_after_a_disable_is_not_a_fault():
    """The modules are still spinning for a moment. That is not a command."""
    mon = monitor()
    assert hold(mon, clean(enabled=False, drive_current=22.0), 0.1) == []


def test_an_enabled_robot_drawing_current_is_the_whole_point_of_a_robot():
    mon = monitor()
    assert hold(mon, clean(enabled=True, drive_current=58.0), 0.28) == []


def test_an_unavailable_current_reading_does_not_manufacture_a_fault():
    mon = monitor()
    assert hold(mon, clean(enabled=False, drive_current=None), 0.28) == []


# ---------------------------------------------------------------------------
# 5. command out of range
# ---------------------------------------------------------------------------


def test_commanding_more_than_the_drive_has_is_a_warning():
    mon = monitor()
    found = hold(mon, clean(commanded=FakeSpeeds(9.0, 0.0, 0.0)), 0.18)
    assert kinds(found) == ["command-out-of-range"]
    assert found[0].severity == WARNING  # the calibration could be stale


def test_spinning_faster_than_the_drive_can_counts_too():
    mon = monitor()
    found = hold(mon, clean(commanded=FakeSpeeds(0.0, 0.0, 20.0)), 0.18)
    assert kinds(found) == ["command-out-of-range"]


def test_a_command_just_over_the_measured_limit_is_tolerated():
    """The limits come from a probe, not a constant, so the band is generous."""
    mon = monitor()
    assert hold(mon, clean(commanded=FakeSpeeds(4.6, 0.0, 0.0)), 0.18) == []


def test_without_calibration_the_range_check_stands_down_and_says_so():
    mon = InvariantMonitor(state=None, limits=None,
                           thresholds=InvariantThresholds(piece_capacity=40, **FAST))
    assert hold(mon, clean(commanded=FakeSpeeds(90.0, 0.0, 0.0)), 0.18) == []
    assert any("command-out-of-range" in reason for reason in mon.inactive)


# ---------------------------------------------------------------------------
# 6. possession impossible
# ---------------------------------------------------------------------------


def test_a_negative_piece_count_is_an_error():
    found = monitor().evaluate(clean(held=-1))
    assert kinds(found) == ["possession-impossible"]
    assert found[0].severity == ERROR


def test_holding_more_than_the_hopper_takes_is_an_error():
    assert kinds(monitor().evaluate(clean(held=41))) == ["possession-impossible"]


def test_a_full_hopper_is_not_an_impossible_one():
    assert monitor().evaluate(clean(held=40)) == []
    assert monitor().evaluate(clean(held=0)) == []


def test_nothing_published_is_not_a_negative_count():
    """The sentinel must not read as the violation it stands in front of."""
    assert monitor().evaluate(clean(held=NO_COUNT)) == []


def test_the_sentinel_is_far_from_any_number_a_robot_could_publish():
    """A robot really reporting -1 held is exactly what this oracle is for."""
    assert NO_COUNT < -1
    assert kinds(monitor().evaluate(clean(held=-1))) == ["possession-impossible"]


def test_without_a_capacity_only_the_negative_half_fires_and_it_says_so():
    mon = InvariantMonitor(state=None, limits=FakeLimits(),
                           thresholds=InvariantThresholds(**FAST))
    assert mon.evaluate(clean(held=4000)) == []
    assert kinds(mon.evaluate(clean(held=-1))) == ["possession-impossible"]
    assert any("possession-impossible" in reason for reason in mon.inactive)


# ---------------------------------------------------------------------------
# the primitive, and the shape of the whole
# ---------------------------------------------------------------------------


def test_one_shot_fires_once_and_rearms_only_after_it_clears():
    shot = _OneShot()
    assert shot.update(True) is True
    assert shot.update(True) is False
    assert shot.update(False) is False
    assert shot.update(True) is True


def test_every_kind_the_oracle_advertises_can_actually_be_produced():
    """The list in `KINDS` is what the report and the preflight check against.

    A kind that is advertised but unreachable is the exact failure this file
    exists to prevent: a name in a report that no code path can ever put there.
    """
    mon = monitor()
    produced = set()
    produced.update(kinds(mon.evaluate(clean(battery_volts=float("nan")))))
    mon.reset_episode()
    produced.update(kinds(hold(mon, clean(truth=FakePose(-1.5, 4.0)), 0.2)))
    mon.reset_episode()
    mon.evaluate(clean(t=1.0, truth=FakePose(8.0, 4.0)))
    produced.update(kinds(mon.evaluate(clean(t=1.05, truth=FakePose(14.0, 4.0)))))
    mon.reset_episode()
    produced.update(kinds(hold(mon, clean(enabled=False, drive_current=22.0), 0.28)))
    mon.reset_episode()
    produced.update(kinds(hold(mon, clean(commanded=FakeSpeeds(9.0, 0.0, 0.0)), 0.18)))
    mon.reset_episode()
    produced.update(kinds(mon.evaluate(clean(held=-1))))

    assert produced == set(InvariantMonitor.KINDS)


def test_every_finding_is_filed_under_the_right_oracle():
    found = monitor().evaluate(clean(battery_volts=float("nan"), held=-1))
    assert {f.oracle for f in found} == {"invariants"}


def test_findings_accumulate_across_an_episode_reset():
    """Resetting a phase clears the latches, not the evidence."""
    mon = monitor()
    mon.findings.extend(mon.evaluate(clean(held=-1)))
    mon.reset_episode()
    assert len(mon.findings) == 1


def test_the_shipped_durations_are_what_the_readme_says():
    """The tests above run five times fast, so the real numbers need pinning.

    Not a tautology: it is the one place the shipped debounce is asserted, and
    without it a threshold could be nudged to zero and every test above would
    still pass while the campaign filled with jitter.
    """
    th = InvariantThresholds()
    assert th.off_field_seconds == 0.5
    assert th.disabled_current_seconds == 1.0
    assert th.command_overrange_seconds == 0.5
    assert th.teleport_speed_mps == 12.0
    assert th.disabled_current_amps == 5.0
    assert th.command_overrange_factor == 1.25
    # Every duration is long enough to outlast a sampling hiccup at 20 Hz.
    for seconds in (th.off_field_seconds, th.disabled_current_seconds,
                    th.command_overrange_seconds):
        assert seconds >= 0.5


def test_the_field_defaults_are_the_frc_field():
    """Game-agnostic by default, overridable when a field is not standard."""
    th = InvariantThresholds()
    assert th.field_length_m == pytest.approx(16.541, abs=0.01)
    assert th.field_width_m == pytest.approx(8.052, abs=0.01)


# ---------------------------------------------------------------------------
# the prover the app and the preflight both run
# ---------------------------------------------------------------------------


def prove(**kwargs):
    from bridge.oracles import prove_invariants

    kwargs.setdefault("thresholds", InvariantThresholds(piece_capacity=40, **FAST))
    kwargs.setdefault("limits", FakeLimits())
    return prove_invariants(pose=FakePose, speeds=FakeSpeeds, **kwargs)


def test_the_prover_fires_every_invariant_it_attempts():
    """The claim `run_bridge_oracles.py` and the overnight preflight both make.

    Tested here as well as run live because the prover is the thing that would
    silently stop proving anything -- an injection edited to be just under a
    threshold reports six ok lines and checks nothing.
    """
    from bridge.oracles import unproven_invariants

    proof = prove()
    assert set(proof) == set(InvariantMonitor.KINDS)
    assert unproven_invariants(proof) == []


def test_the_prover_skips_the_range_check_without_limits_rather_than_faking_one():
    proof = prove(limits=None)
    assert "command-out-of-range" not in proof
    assert set(proof) == set(InvariantMonitor.KINDS) - {"command-out-of-range"}


def test_a_detector_that_stopped_working_is_reported_as_unproven():
    """The failure mode the prover exists for, forced.

    A teleport threshold nobody could ever exceed is exactly what a careless
    edit produces, and without this the prover would report `teleport` as
    attempted-and-fine because its own injection would stop reaching it.
    """
    from bridge.oracles import unproven_invariants

    blind = InvariantThresholds(piece_capacity=40, teleport_speed_mps=1e9, **FAST)
    assert unproven_invariants(prove(thresholds=blind)) == ["teleport"]


def test_the_prover_uses_a_fresh_monitor_for_each_invariant():
    """Otherwise one injection's latch could arm or mask the next, and the
    order of the list would quietly become part of what is proved."""
    proof = prove()
    for kind, found in proof.items():
        assert kinds(found) == [kind], f"{kind} injection also tripped {kinds(found)}"


# ---------------------------------------------------------------------------
# what went unchecked
# ---------------------------------------------------------------------------


def test_stood_down_remembers_a_gap_that_inactive_has_forgotten():
    """The harness calibrates during its first match, so `limits` arrives late.

    A monitor asked afterwards reports full coverage for a match that spent
    part of itself checking five invariants out of six. This is the difference.
    """
    mon = monitor(limits=None)
    mon._ever_inactive.update(mon.inactive)  # what poll() does
    mon.limits = FakeLimits()
    assert mon.inactive == []
    assert any("command-out-of-range" in reason for reason in mon.stood_down)


def test_a_monitor_that_never_sampled_claims_nothing():
    mon = monitor()
    assert mon.samples_taken == 0
    assert mon.stood_down == []
