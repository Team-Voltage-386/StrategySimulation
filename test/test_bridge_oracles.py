"""Rules tests for the bridge's fault oracle and its debounce primitive.

Oracle 02 proves itself at runtime -- `apps/run_bridge_oracles.py` deliberately
wedges the robot against a wall every run and fails if the detector stays
quiet. Oracle 01 has no equivalent, because deliberately crashing robot code to
prove a grep works is a poor trade. These tests are its substitute, and they
are the reason `bridge.oracles` is importable without pyntcore: they have to
run in CI, where there is no JVM and no robot project.

The muffle cases are regression guards against lines this actually saw. Each
one is a real line from a real console log that a naive `grep -i error` would
have reported.
"""
from __future__ import annotations

import math

from bridge.oracles import (
    ERROR,
    WARNING,
    FaultOracle,
    Finding,
    Muffle,
    RPM_TO_RAD_S,
    _Latch,
    _wrap,
    summarize,
)


# ---------------------------------------------------------------------------
# oracle 01 -- what counts as a fault
# ---------------------------------------------------------------------------


def test_exception_is_one_finding_carrying_its_stack():
    console = [
        "********** Robot program starting **********",
        "java.lang.NullPointerException: Cannot invoke \"frc.robot.subsystems.intake.IntakeIO.deploy()\"",
        "\tat frc.robot.commands.DeployIntake.initialize(DeployIntake.java:22)",
        "\tat edu.wpi.first.wpilibj2.command.CommandScheduler.schedule(CommandScheduler.java:302)",
        "\tat frc.robot.Robot.teleopInit(Robot.java:142)",
        "Warning at ...: something else entirely",
    ]
    findings = FaultOracle().scan_lines(console)

    exceptions = [f for f in findings if f.kind == "exception"]
    assert len(exceptions) == 1, "each stack frame must not become its own finding"
    assert exceptions[0].severity == ERROR
    assert "NullPointerException" in exceptions[0].message
    assert "DeployIntake.java:22" in exceptions[0].detail
    assert exceptions[0].where == "console:2"


def test_driver_station_reports_split_by_severity():
    console = [
        "Error at frc.robot.Robot.teleopInit(Robot.java:140): intake never deployed",
        "Warning at edu.wpi.first.wpilibj.IterativeRobotBase: something mild",
    ]
    findings = FaultOracle().scan_lines(console)
    by_kind = {f.kind: f for f in findings}

    assert by_kind["ds-error"].severity == ERROR
    assert by_kind["ds-error"].message == "intake never deployed"
    assert "Robot.java:140" in by_kind["ds-error"].detail
    assert by_kind["ds-warning"].severity == WARNING


def test_ds_warnings_can_be_silenced_without_silencing_errors():
    console = [
        "Error at a: real",
        "Warning at b: mild",
    ]
    findings = FaultOracle(report_ds_warnings=False).scan_lines(console)
    assert [f.kind for f in findings] == ["ds-error"]


def test_muffled_lines_are_counted_not_reported():
    # Every one of these is a real line from a bridge run's console.
    console = [
        "> Task :compileJava",
        "D:\\git\\TyRapXXVI_2\\src\\main\\java\\frc\\robot\\Robot.java:142: warning: [removal] schedule() in Command has been deprecated",
        "1 warning",
        "If you receive errors loading the JNI dependencies, make sure you have the latest Visual Studio C++ Redstributable installed.",
        "NT: NT4 socket error: operation canceled",
        "BUILD SUCCESSFUL in 42s",
    ]
    oracle = FaultOracle()
    findings = oracle.scan_lines(console)

    assert findings == [], f"muffled lines leaked through: {findings}"
    assert oracle.muffled_count == len(console)


def test_jni_boilerplate_does_not_read_as_an_error():
    """Regression guard: this line contains 'errors' and reports nothing."""
    line = "If you receive errors loading the JNI dependencies, make sure you have the latest Visual Studio C++ Redstributable installed."
    assert FaultOracle().scan_lines([line]) == []


def test_loop_overruns_only_matter_in_bulk():
    one = ["Warning at ...printLoopOverrunMessage(...): Loop time of 0.02s overrun"]
    oracle = FaultOracle(max_loop_overruns=5)

    assert oracle.scan_lines(one * 3) == []
    assert oracle.loop_overrun_count == 3

    findings = oracle.scan_lines(one * 9)
    assert [f.kind for f in findings] == ["loop-overrun"]
    assert findings[0].severity == WARNING
    assert "9 loop overruns" in findings[0].message


def test_hard_stop_is_reported():
    findings = FaultOracle().scan_lines(["The robot program quit unexpectedly."])
    assert [f.kind for f in findings] == ["robot-stopped"]
    assert findings[0].severity == ERROR


def test_a_clean_console_is_silent():
    console = [
        "********** Robot program starting **********",
        "[AdvantageKit] Logging to \"logs/bridge/akit_26-08-24.wpilog\"",
        "NT: Listening on NT4 port 5810",
    ]
    assert FaultOracle().scan_lines(console) == []


def test_custom_muffle_carries_its_reason():
    muffle = Muffle(r"expected chatter", "the mechanism prints this on every deploy")
    assert muffle.matches("expected chatter from the intake")
    assert muffle.reason  # the whole point of the type


# ---------------------------------------------------------------------------
# the debounce primitive both oracles rely on
# ---------------------------------------------------------------------------


def test_latch_needs_the_condition_held_for_its_full_duration():
    latch = _Latch(2.0)
    assert not latch.update(True, 100.0)  # starts the clock
    assert not latch.update(True, 101.9)  # not yet
    assert latch.update(True, 102.0)  # fires


def test_latch_fires_once_per_episode():
    latch = _Latch(1.0)
    latch.update(True, 0.0)
    assert latch.update(True, 1.0)
    assert not latch.update(True, 2.0), "one wedged mechanism must not be 20 findings a second"
    assert not latch.update(True, 60.0)


def test_latch_rearms_after_the_condition_clears():
    latch = _Latch(1.0)
    latch.update(True, 0.0)
    assert latch.update(True, 1.0)
    latch.update(False, 2.0)  # cleared
    latch.update(True, 3.0)  # a genuinely new episode
    assert latch.update(True, 4.0)


def test_a_flickering_condition_never_fires():
    latch = _Latch(1.0)
    for tick in range(20):
        assert not latch.update(tick % 2 == 0, float(tick))


# ---------------------------------------------------------------------------
# units and helpers
# ---------------------------------------------------------------------------


def test_flywheel_units_line_up_with_what_the_robot_publishes():
    """The setpoint is RPM and the measurement is rad/s.

    Comparing them directly would make a healthy flywheel look like it was
    reaching 10% of its target forever. These are the two numbers observed on
    a real run holding the left bumper.
    """
    assert math.isclose(2200.0 * RPM_TO_RAD_S, 230.3834612632515, rel_tol=1e-12)


def test_wrap_takes_the_short_way_round():
    assert math.isclose(_wrap(math.radians(359)), math.radians(-1), abs_tol=1e-9)
    assert math.isclose(_wrap(math.radians(-359)), math.radians(1), abs_tol=1e-9)
    assert math.isclose(_wrap(0.5), 0.5)


def test_summarize_groups_by_kind():
    findings = [
        Finding("liveness", "frozen-robot", ERROR, "a", "t=1s"),
        Finding("liveness", "frozen-robot", ERROR, "b", "t=9s"),
        Finding("faults", "ds-warning", WARNING, "c", "console:3"),
    ]
    text = summarize(findings)
    assert "2 error(s)" in text and "1 warning(s)" in text
    assert "2 kind(s)" in text
    assert summarize([]) == "no findings"
