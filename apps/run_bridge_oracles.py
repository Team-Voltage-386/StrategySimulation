"""Step 2 of the maple-sim bridge: teach the campaign to fail.

Runs a scripted scenario against the real robot code with both oracles armed,
then reports. The run is in two phases, and *both* have to come out right:

  EXERCISE  drive, turn, shoot, intake -- ordinary robot operation.
            Expected result: zero findings.

  PROVOKE   pin the robot against the field wall while still commanding it
            forward. Expected result: oracle 02 reports `frozen-robot`.

The second phase is the point. An oracle that has never fired is not known to
work, and "0 findings" from a detector that cannot detect looks exactly like
"0 findings" from a clean run. Running a known-bad condition through it every
time is the only thing that keeps the overnight report meaningful -- otherwise
the first silent regression in the detector turns every subsequent night into
a rubber stamp.

A wall pin is an honest provocation, incidentally, not a rigged one: being
commanded forward while wedged is precisely the frozen-robot signature the
detector is for, and it is a situation a real match produces constantly.

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
    problems: list[str] = []
    fault_oracle = oracles.FaultOracle()

    # Which phases actually executed. Tracked separately from their findings
    # because "no findings" and "never ran" are the same empty list, and the
    # first version of this reported a run that aborted during boot as
    # "exercise phase clean". A report that says ok about work it did not do is
    # worse than no report -- it is the exact failure the provoke phase exists
    # to prevent, one level up.
    ran = {"exercise": False, "provoke": False}

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
            with monitor:
                _step("phase EXERCISE (expect no findings)")
                start = len(monitor.findings)
                phase_exercise(link, state)
                exercise_findings = monitor.findings[start:]
                ran["exercise"] = True

                if not args.no_provoke:
                    monitor.reset_episode()
                    _step("phase PROVOKE (expect frozen-robot)")
                    start = len(monitor.findings)
                    phase_provoke(link, state, args.provoke_seconds)
                    provoke_findings = monitor.findings[start:]
                    ran["provoke"] = True

            _log()
            if monitor.samples:
                _log(f"   sampled {len(monitor.samples)} times over {monitor.samples[-1].t:.1f}s")
            else:
                problems.append("liveness monitor never took a sample")

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

    errors = [f for f in fault_findings if f.severity == oracles.ERROR]
    if errors:
        problems.append(f"oracle 01 found {len(errors)} error-level fault(s) in the console")

    _step("RESULT")
    if problems:
        for problem in problems:
            _log(f"   FAIL  {problem}")
        return 1
    _log("   PASS  both oracles are armed, quiet on clean operation, and proven to fire")
    _log(f"   {oracles.summarize(fault_findings + exercise_findings + provoke_findings)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
