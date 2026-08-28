"""Step 2 of the maple-sim bridge: teach the campaign to fail.

Runs a scripted scenario against the real robot code with all three oracles
armed, then reports. The run is in three phases, and *all* have to come out
right:

  EXERCISE  drive, turn, shoot, intake -- ordinary robot operation.
            Expected result: zero findings, from oracle 02 and oracle 03 both.

  PROVOKE   pin the robot against the field wall while still commanding it
            forward. Expected result: oracle 02 reports `frozen-robot`, and
            oracle 03 stays quiet -- a pinned robot is not a teleported one.

  INJECT    push a synthetic violation of each of oracle 03's six invariants
            through detectors built with the thresholds and the drive limits
            this run is actually using. Expected result: all six fire.

The last two phases are the point. An oracle that has never fired is not known
to work, and "0 findings" from a detector that cannot detect looks exactly like
"0 findings" from a clean run. Running a known-bad condition through it every
time is the only thing that keeps the overnight report meaningful -- otherwise
the first silent regression in the detector turns every subsequent night into
a rubber stamp.

PROVOKE and INJECT are not equally strong, and it is worth being plain about
which is which. A wall pin is an honest provocation: a real robot genuinely
wedged, commanded forward, which is precisely the frozen-robot signature and a
situation a real match produces constantly. INJECT is not that -- it is
synthetic snapshots pushed through the detectors by hand, because making real
robot code publish a NaN or drive its motors while disabled would mean breaking
it on purpose. That is oracle 01's trade and it comes out the same way.

    python apps/run_bridge_oracles.py
    python apps/run_bridge_oracles.py --no-provoke   # exercise only
    python apps/run_bridge_oracles.py --attach
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from bridge import drive_model as dm
from bridge import match_view as mv
from bridge import operator as op
from bridge import oracles
from bridge.robot_state import DS_ENABLED, POSE_TRUTH, RobotStateLink
from bridge.sim_process import find_robot_repo, RobotSim

SETTLE = 0.75


def _log(msg: str = "") -> None:
    print(msg, flush=True)


def _step(title: str) -> None:
    _log()
    _log(f"-- {title} " + "-" * max(0, 58 - len(title)))


def _hold(link: op.OperatorLink, seconds: float, **axes: float) -> None:
    """Hold a set of axes for a while, then release just those axes."""
    for name, value in axes.items():
        link.set_axis(getattr(op, f"AXIS_{name.upper()}"), value)
    time.sleep(seconds)
    for name in axes:
        link.set_axis(getattr(op, f"AXIS_{name.upper()}"), 0.0)


def phase_exercise(link: op.OperatorLink, state: RobotStateLink) -> None:
    """Ordinary operation. Nothing here should trip a detector."""
    _log("   drive forward 1.5s")
    _hold(link, 1.5, left_y=-0.5)
    time.sleep(SETTLE)

    _log("   drive back 1.5s")
    _hold(link, 1.5, left_y=0.5)
    time.sleep(SETTLE)

    _log("   turn in place 1.5s")
    _hold(link, 1.5, right_x=-0.6)
    time.sleep(SETTLE)

    # Strafe *away* from the lower wall. The robot starts at y=0.815 m with a
    # 30" bumper, so it is only ~0.43 m off that wall -- strafing toward it
    # would wedge the robot during the phase that is supposed to be clean, and
    # the resulting frozen-robot finding would be the test's own fault.
    _log("   strafe away from the near wall 1.0s")
    _hold(link, 1.0, left_x=-0.5)
    time.sleep(SETTLE)

    _log("   spin up the flywheel 2.5s (leftBumper)")
    link.set_button(op.BTN_LEFT_BUMPER, True)
    time.sleep(2.5)
    link.set_button(op.BTN_LEFT_BUMPER, False)
    time.sleep(SETTLE)

    _log("   run the intake 1.5s (rightTrigger)")
    _hold(link, 1.5, right_trigger=1.0)
    time.sleep(SETTLE)

    _log("   drive and shoot at once 2.0s")
    link.set_button(op.BTN_LEFT_BUMPER, True)
    _hold(link, 2.0, left_y=-0.4)
    link.set_button(op.BTN_LEFT_BUMPER, False)
    time.sleep(SETTLE)


def phase_provoke(link: op.OperatorLink, state: RobotStateLink, seconds: float) -> None:
    """Drive into the field wall and keep commanding. Should trip frozen-robot.

    The arena's lower border runs along y=0 and the robot spends the exercise
    phase within about a metre of it, so pushing -y wedges it in well under a
    second. The rest of the window is drive-still-commanded, pose-no-longer-
    changing: exactly what the detector is looking for.

    The binding is `ySupplier = () -> -kDriveController.getLeftX()`, so a
    positive leftX is -y, toward the wall.
    """
    _log(f"   pushing into the wall for {seconds:.1f}s with the drive still commanded")
    before = state.truth_pose()
    link.set_axis(op.AXIS_LEFT_X, 0.7)
    time.sleep(seconds)
    link.set_axis(op.AXIS_LEFT_X, 0.0)
    after = state.truth_pose()
    _log(f"   {before} -> {after}")


def report(findings: list[oracles.Finding], title: str) -> None:
    _step(title)
    if not findings:
        _log("   no findings")
        return
    for finding in findings:
        _log(f"   {finding}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--repo", type=Path, default=None,
                        help="robot project root (default: the sibling checkout, or $SPARKY_ROBOT_REPO)")
    parser.add_argument("--attach", action="store_true", help="use a sim that is already running")
    parser.add_argument("--no-provoke", action="store_true", help="skip the known-bad phase")
    parser.add_argument("--provoke-seconds", type=float, default=6.0)
    parser.add_argument("--echo", action="store_true", help="stream the robot console")
    parser.add_argument("--boot-timeout", type=float, default=300.0)
    parser.add_argument("--log", type=Path, default=Path("build/bridge/oracles-console.log"))
    args = parser.parse_args(argv)
    # Resolved once, here, so the run prints the project it actually
    # picked and a missing one fails before a JVM is started.
    args.repo = find_robot_repo(args.repo)

    sim: RobotSim | None = None
    exercise_findings: list[oracles.Finding] = []
    provoke_findings: list[oracles.Finding] = []
    fault_findings: list[oracles.Finding] = []
    invariant_findings: list[oracles.Finding] = []
    injected: dict[str, list[oracles.Finding]] = {}
    inactive: list[str] = []  # oracle-03 invariants the live phases could not check
    problems: list[str] = []
    fault_oracle = oracles.FaultOracle()
    thresholds = oracles.InvariantThresholds(piece_capacity=mv.INTAKE_CAPACITY)
    limits = None

    # Which phases actually executed. Tracked separately from their findings
    # because "no findings" and "never ran" are the same empty list, and the
    # first version of this reported a run that aborted during boot as
    # "exercise phase clean". A report that says ok about work it did not do is
    # worse than no report -- it is the exact failure the provoke phase exists
    # to prevent, one level up.
    ran = {"exercise": False, "provoke": False, "inject": False}

    try:
        if args.attach:
            _log("attaching to a sim that is already running")
        else:
            _step("launching robot sim")
            _log(f"   {args.repo}  ->  {args.log}")
            sim = RobotSim(args.repo, args.log, echo=args.echo)
            sim.start()

        with op.OperatorLink() as link, RobotStateLink() as state:
            state.wait_for_connection(timeout=args.boot_timeout)
            state.wait_for_topic(POSE_TRUTH, timeout=args.boot_timeout)

            link.neutral()
            link.teleop_enable(station="blue1")
            time.sleep(1.0)
            if not state.boolean(DS_ENABLED):
                problems.append("robot never reported itself enabled")

            monitor = oracles.LivenessMonitor(state, link)
            invariants = oracles.InvariantMonitor(state, limits=limits, thresholds=thresholds)
            with monitor, invariants:
                _step("phase EXERCISE (expect no findings)")
                start, start_inv = len(monitor.findings), len(invariants.findings)
                phase_exercise(link, state)
                exercise_findings = monitor.findings[start:]
                invariant_findings += invariants.findings[start_inv:]
                ran["exercise"] = True

                if not args.no_provoke:
                    monitor.reset_episode()
                    invariants.reset_episode()
                    _step("phase PROVOKE (expect frozen-robot)")
                    start, start_inv = len(monitor.findings), len(invariants.findings)
                    phase_provoke(link, state, args.provoke_seconds)
                    provoke_findings = monitor.findings[start:]
                    # Kept with the exercise findings rather than reported
                    # apart, because oracle 03's claim about this phase is the
                    # same as its claim about the other one: nothing here is a
                    # violation. A robot pinned against a wall is doing
                    # something legal, and an invariant that cannot tell that
                    # from an ejection would fail every match that touches
                    # anything.
                    invariant_findings += invariants.findings[start_inv:]
                    ran["provoke"] = True

            inactive = invariants.stood_down

            _log()
            if monitor.samples:
                _log(f"   oracle 02 sampled {len(monitor.samples)} times over "
                     f"{monitor.samples[-1].t:.1f}s")
            else:
                problems.append("liveness monitor never took a sample")
            if invariants.samples_taken:
                _log(f"   oracle 03 sampled {invariants.samples_taken} times")
            else:
                problems.append("invariant monitor never took a sample")

            # Measured last, on purpose. INJECT wants real drive limits so it
            # proves the command-range invariant against the number this robot
            # actually produces, but calibration *drives* -- four probes with
            # a reversal that does not perfectly undo them -- and PROVOKE
            # depends on the robot still being near the wall it was placed by.
            # A check that rearranges the field has changed the experiment it
            # was meant to validate, so this one runs when there is nothing
            # left to disturb.
            _step("measuring the drive limits (for INJECT)")
            try:
                limits, _ = dm.calibrate(link, state)
                _log(f"   {limits.max_speed_mps:.2f} m/s, {limits.max_omega_rad_s:.2f} rad/s")
            except Exception as exc:
                _log(f"   could not calibrate ({type(exc).__name__}: {exc})")
                _log("   the command-range invariant will not be injected this run")

            link.neutral()
            link.disable()
            time.sleep(SETTLE)

            fault_findings = fault_oracle.scan_alerts(state)

    except Exception as exc:
        problems.append(f"{type(exc).__name__}: {exc}")
        _log()
        _log(f"ABORTED: {type(exc).__name__}: {exc}")
        if sim is not None:
            for line in sim.tail(25):
                _log(f"   {line}")
    finally:
        if sim is not None:
            _step("stopping robot sim")
            sim.stop()
            _log(f"   stopped (exit {sim.returncode}, expected: the sim is killed, not asked)")

    # Oracle 01 reads the console after the fact, so the whole run is present
    # including anything the JVM printed on the way down.
    if args.log.is_file():
        fault_findings += fault_oracle.scan_file(args.log)

    report(fault_findings, "ORACLE 01 -- hard faults")
    _log(f"   ({fault_oracle.muffled_count} lines muffled as known-benign, "
         f"{fault_oracle.loop_overrun_count} loop overruns seen)")

    report(exercise_findings, "ORACLE 02 -- liveness, EXERCISE phase")
    if not args.no_provoke:
        report(provoke_findings, "ORACLE 02 -- liveness, PROVOKE phase")

    report(invariant_findings, "ORACLE 03 -- invariants, live phases")
    if not ran["exercise"]:
        # An empty `inactive` from a run that never built a monitor reads
        # exactly like full coverage. Say what the live phases would have
        # missed instead of implying they missed nothing.
        inactive = oracles.InvariantMonitor(
            state=None, limits=None, thresholds=thresholds
        ).inactive
    for reason in inactive:
        _log(f"   NOT CHECKED  {reason}")
    if not inactive and ran["exercise"]:
        _log("   all six invariants were active")

    # Deliberately after the sim is stopped. INJECT touches no robot and no
    # NetworkTables, so running it here means it still happens on a run that
    # aborted mid-match -- which is exactly when knowing the detectors work is
    # worth most, because the alternative reading is that they saw nothing.
    _step("phase INJECT (expect all six invariants to fire)")
    try:
        injected = oracles.prove_invariants(thresholds, limits)
        ran["inject"] = True
    except Exception as exc:
        problems.append(f"inject phase failed: {type(exc).__name__}: {exc}")
        _log(f"   ABORTED: {type(exc).__name__}: {exc}")
    for kind in oracles.InvariantMonitor.KINDS:
        if kind not in injected:
            _log(f"   skip  {kind:24} not injected")
        elif any(f.kind == kind for f in injected[kind]):
            _log(f"   ok    {kind:24} fired")
        else:
            _log(f"   FAIL  {kind:24} silent")

    _step("SELF-CHECK")
    if not ran["exercise"]:
        problems.append("exercise phase never ran, so nothing about it was verified")
        _log("   FAIL  exercise phase never ran")
    elif exercise_findings:
        problems.append(
            f"exercise phase produced {len(exercise_findings)} finding(s); ordinary "
            f"operation should be clean, so either the robot has a real problem or a "
            f"threshold is too tight"
        )
        _log("   FAIL  exercise phase was not clean")
    else:
        _log("   ok    exercise phase clean -- no false positives")

    if args.no_provoke:
        _log("   skip  provoke phase disabled; oracle 02 is UNVERIFIED this run")
    elif not ran["provoke"]:
        problems.append("provoke phase never ran, so oracle 02 is unverified")
        _log("   FAIL  provoke phase never ran")
    elif [f for f in provoke_findings if f.kind in oracles.STUCK_KINDS]:
        kind = next(f.kind for f in provoke_findings if f.kind in oracles.STUCK_KINDS)
        _log(f"   ok    provoke phase detected {kind} -- the detector works")
        if kind != "robot-pinned":
            problems.append(
                f"the wall pin was classified {kind}, not robot-pinned; a robot leaning "
                f"on a wall draws ~58 A, so the drive-current threshold looks wrong"
            )
            _log("   FAIL  ...but classified wrong, so the current threshold is off")
    else:
        problems.append(
            "provoke phase reported nothing; oracle 02 cannot be trusted to catch a real one"
        )
        _log("   FAIL  provoke phase went undetected")

    if invariant_findings:
        problems.append(
            f"oracle 03 reported {len(invariant_findings)} finding(s) during ordinary "
            f"operation: {oracles.summarize(invariant_findings)}. Every one of these is "
            f"a thing that must never be true, so either the sim really did it or an "
            f"invariant is wrong"
        )
        _log("   FAIL  oracle 03 was not quiet on the live phases")
    elif ran["exercise"]:
        _log("   ok    oracle 03 quiet on live robot behaviour -- no false positives")

    if not ran["inject"]:
        problems.append("inject phase never ran, so oracle 03 is unverified")
        _log("   FAIL  inject phase never ran")
    else:
        silent = oracles.unproven_invariants(injected)
        skipped = [k for k in oracles.InvariantMonitor.KINDS if k not in injected]
        if silent:
            problems.append(
                f"oracle 03 invariant(s) did not fire on a deliberate violation: "
                f"{', '.join(silent)}"
            )
            _log(f"   FAIL  {len(silent)} invariant(s) went undetected")
        else:
            _log(f"   ok    {len(injected)}/{len(oracles.InvariantMonitor.KINDS)} "
                 f"invariants proven to fire")
        if skipped:
            # Not a failure, because the reason is recorded and legitimate --
            # but it does have to be said out loud, or a run that checked five
            # of six reads identically to one that checked all six.
            _log(f"   note  not injected: {', '.join(skipped)} (see NOT CHECKED above)")

    errors = [f for f in fault_findings if f.severity == oracles.ERROR]
    if errors:
        problems.append(f"oracle 01 found {len(errors)} error-level fault(s) in the console")

    _step("RESULT")
    if problems:
        for problem in problems:
            _log(f"   FAIL  {problem}")
        return 1
    _log("   PASS  all three oracles are armed, quiet on clean operation, and proven to fire")
    _log(f"   {oracles.summarize(fault_findings + exercise_findings + provoke_findings + invariant_findings)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
