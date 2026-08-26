"""Step 1 of the maple-sim bridge: prove the loop closes at all.

Launches the robot project's `simulateJava` headless, connects both channels,
and runs four checks. They are ordered so that a failure localises itself --
each one depends on everything before it and nothing after, so the first
failing check names the layer at fault.

  1. LINK   -- the WebSocket is open and NT is publishing poses. Transport
               only; nothing has been sent yet.
  2. ECHO   -- with the robot still *disabled*, push a distinctive stick
               pattern and read it back off AdvantageKit's own DriverStation
               log. This separates "the wire works" from "the robot code
               reacted", which are different problems with different fixes.
  3. AXIS   -- enable teleop and hold the left stick forward. maple-sim's
               ground-truth pose has to move, and stop when released. The
               input goes through the real binding layer to the drive's
               default command; nothing is bypassed.
  4. BUTTON -- hold the left bumper. Its binding runs `flywheel.shootCommand`,
               so a commanded setpoint appears *and* simulated wheel speed
               follows it, on a subsystem entirely unrelated to the drive.
               Release and both fall back to zero.

No part of this cares which game the robot code implements. The 2026 field is
irrelevant to a test of transport -- what is being checked is that Python can
press a button and the robot's command scheduler notices.

    python apps/run_bridge_smoke.py
    python apps/run_bridge_smoke.py --attach       # against an already-running sim
    python apps/run_bridge_smoke.py --dump-topics  # list everything NT publishes
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from bridge import operator as op
from bridge.robot_state import DS_ENABLED, POSE_ODOMETRY, POSE_TRUTH, Pose2d, RobotStateLink
from bridge.sim_process import find_robot_repo, RobotSim

# Clears DriveCommands.DEADBAND (0.06) with room to spare, and slow enough
# that a 2 s push stays well inside the field.
DRIVE_AXIS = -0.5  # negative: the binding is `() -> -kDriveController.getLeftY()`
DRIVE_SECONDS = 2.0
SETTLE_SECONDS = 0.75

# A pattern with a distinct value per axis, so a transposition or an off-by-one
# in the axis map shows up as a mismatch rather than as coincidentally-equal
# numbers. Axis 2/3 are triggers, which only travel 0..1.
ECHO_AXES = {
    op.AXIS_LEFT_X: -0.25,
    op.AXIS_LEFT_Y: 0.50,
    op.AXIS_LEFT_TRIGGER: 0.75,
    op.AXIS_RIGHT_TRIGGER: 0.30,
    op.AXIS_RIGHT_X: -0.80,
    op.AXIS_RIGHT_Y: 0.10,
}
ECHO_BUTTONS = (op.BTN_A, op.BTN_RIGHT_BUMPER, op.BTN_START)
# The wire carries axes as float32, so an exact compare against a float64
# literal would fail on representation alone.
ECHO_TOLERANCE = 1e-3

MIN_DRIVE_METRES = 0.5
MAX_COAST_METRES = 0.35

FLYWHEEL_SECONDS = 2.0
FLYWHEEL_SETPOINT = "/AdvantageKit/RealOutputs//Shooter/Flywheel/VelocitySetpoint"
FLYWHEEL_SPEED = "/AdvantageKit/Shooter/Flywheel/Inputs/FlywheelSpeed"
MIN_FLYWHEEL_SPEED = 50.0


class CheckFailed(Exception):
    pass


def _log(msg: str = "") -> None:
    print(msg, flush=True)


def _step(title: str) -> None:
    _log()
    _log(f"-- {title} " + "-" * max(0, 58 - len(title)))


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise CheckFailed(message)


def check_link(state: RobotStateLink, link: op.OperatorLink) -> None:
    _step("1. LINK")
    _require(link.connected, f"HALSim WebSocket dropped: {link.closed_reason}")
    _require(state.connected, "NetworkTables client is not connected")

    truth = state.truth_pose()
    _require(truth is not None, f"no value on {POSE_TRUTH}")

    _log(f"   websocket   : open, {link.tx_count} DS packets sent, {link.rx_count} frames received")
    _log(f"   nt4         : connected to the robot's server")
    _log(f"   truth pose  : {truth}")
    _require(link.rx_count > 0, "the robot never sent a single HALSim frame back")
    _log("   PASS -- both channels are live")


def check_echo(state: RobotStateLink, link: op.OperatorLink) -> None:
    _step("2. ECHO -- input reaches the robot (still disabled)")
    _require(
        not state.boolean(DS_ENABLED, True),
        "robot reports itself enabled before we asked it to be",
    )

    for axis, value in ECHO_AXES.items():
        link.set_axis(axis, value)
    for button in ECHO_BUTTONS:
        link.set_button(button, True)
    time.sleep(SETTLE_SECONDS)

    seen_axes = state.joystick_axes(0)
    seen_buttons = state.joystick_buttons(0)
    _log(f"   sent axes   : {[round(ECHO_AXES[i], 2) for i in sorted(ECHO_AXES)]}")
    _log(f"   robot sees  : {[round(v, 3) for v in seen_axes]}")
    _log(f"   sent buttons: {sorted(ECHO_BUTTONS)}")
    _log(f"   robot sees  : {sorted(seen_buttons)}")

    _require(
        len(seen_axes) >= op.AXIS_COUNT,
        f"robot reports {len(seen_axes)} axes, expected at least {op.AXIS_COUNT} -- "
        f"an Xbox binding will read unplugged",
    )
    mismatched = [
        (axis, value, seen_axes[axis])
        for axis, value in ECHO_AXES.items()
        if abs(seen_axes[axis] - value) > ECHO_TOLERANCE
    ]
    _require(
        not mismatched,
        "axis values came back wrong: "
        + ", ".join(f"axis {a} sent {s:+.2f} saw {g:+.3f}" for a, s, g in mismatched),
    )
    _require(
        seen_buttons == set(ECHO_BUTTONS),
        f"button set came back as {sorted(seen_buttons)}, expected {sorted(ECHO_BUTTONS)}",
    )

    link.neutral()
    time.sleep(SETTLE_SECONDS)
    _require(
        not state.joystick_buttons(0),
        "buttons stayed pressed after release -- the robot is latching stale input",
    )
    _log("   PASS -- the robot reads back exactly what was sent, and sees the release")


def check_axis(state: RobotStateLink, link: op.OperatorLink) -> None:
    _step("3. AXIS -> drivetrain")
    start = state.truth_pose()
    _require(start is not None, "lost the truth pose before driving")

    _log(f"   holding leftY = {DRIVE_AXIS:+.2f} for {DRIVE_SECONDS:.1f}s")
    link.set_axis(op.AXIS_LEFT_Y, DRIVE_AXIS)
    time.sleep(DRIVE_SECONDS)
    link.set_axis(op.AXIS_LEFT_Y, 0.0)

    moving = state.truth_pose()
    time.sleep(SETTLE_SECONDS)
    stopped = state.truth_pose()

    travelled = start.distance_to(moving)
    coasted = moving.distance_to(stopped)
    _log(f"   start       : {start}")
    _log(f"   at release  : {moving}   (travelled {travelled:.3f} m)")
    _log(f"   settled     : {stopped}   (coasted {coasted:.3f} m)")

    _require(
        travelled >= MIN_DRIVE_METRES,
        f"stick held for {DRIVE_SECONDS:.1f}s moved the robot only {travelled:.3f} m "
        f"(wanted >= {MIN_DRIVE_METRES} m) -- input is not reaching the drive",
    )
    _require(
        coasted <= MAX_COAST_METRES,
        f"robot kept moving {coasted:.3f} m after the stick was released "
        f"(wanted <= {MAX_COAST_METRES} m) -- releases are not being delivered",
    )
    _log("   PASS -- analog input drives the robot, and releasing it stops the robot")


def check_button(state: RobotStateLink, link: op.OperatorLink) -> None:
    _step("4. BUTTON -> command scheduler")
    idle_setpoint = state.number(FLYWHEEL_SETPOINT)
    idle_speed = state.number(FLYWHEEL_SPEED)
    _log(f"   idle        : setpoint {idle_setpoint:.1f}, measured {idle_speed:.1f}")
    _require(
        abs(idle_setpoint) < 1e-6,
        f"flywheel is already commanded to {idle_setpoint:.1f} before the button is pressed",
    )

    _log(f"   holding leftBumper for {FLYWHEEL_SECONDS:.1f}s (bound to flywheel.shootCommand)")
    link.set_button(op.BTN_LEFT_BUMPER, True)
    time.sleep(FLYWHEEL_SECONDS)
    held_setpoint = state.number(FLYWHEEL_SETPOINT)
    held_speed = state.number(FLYWHEEL_SPEED)
    link.set_button(op.BTN_LEFT_BUMPER, False)
    _log(f"   held        : setpoint {held_setpoint:.1f}, measured {held_speed:.1f}")

    time.sleep(SETTLE_SECONDS)
    released_setpoint = state.number(FLYWHEEL_SETPOINT)
    _log(f"   released    : setpoint {released_setpoint:.1f}")

    _require(
        abs(held_setpoint) > 1e-6,
        "the left bumper never produced a flywheel setpoint -- button edges are not "
        "reaching the command scheduler",
    )
    _require(
        held_speed >= MIN_FLYWHEEL_SPEED,
        f"a setpoint of {held_setpoint:.1f} was commanded but the simulated wheel only "
        f"reached {held_speed:.1f} -- the command ran, the mechanism did not follow",
    )
    _require(
        abs(released_setpoint) < 1e-6,
        f"setpoint stayed at {released_setpoint:.1f} after release -- the falling edge "
        f"was not delivered",
    )
    _log("   PASS -- a button edge scheduled a command on a non-drive subsystem, "
         "and the mechanism responded")


def dump_topics(state: RobotStateLink, prefix: str) -> None:
    info = state.topic_info(prefix, settle=3.0)
    _step(f"NT topics under {prefix!r} ({len(info)})")
    for name in sorted(info):
        _log(f"   {name}   [{info[name]}]")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--repo", type=Path, default=None,
                        help="robot project root (default: the sibling checkout, or $SPARKY_ROBOT_REPO)")
    parser.add_argument("--attach", action="store_true", help="use a sim that is already running")
    parser.add_argument("--keep-alive", action="store_true", help="leave the sim running on exit")
    parser.add_argument("--echo", action="store_true", help="stream the robot console to stdout")
    parser.add_argument("--dump-topics", action="store_true", help="list NT topics before the checks")
    parser.add_argument("--topic-prefix", default="/AdvantageKit")
    parser.add_argument("--boot-timeout", type=float, default=300.0)
    parser.add_argument("--log", type=Path, default=Path("build/bridge/robot-console.log"))
    args = parser.parse_args(argv)
    # Resolved once, here, so the run prints the project it actually
    # picked and a missing one fails before a JVM is started.
    args.repo = find_robot_repo(args.repo)

    sim: RobotSim | None = None
    failures: list[str] = []

    try:
        if args.attach:
            _log("attaching to a sim that is already running")
        else:
            _step("launching robot sim")
            _log(f"   repo : {args.repo}")
            _log(f"   log  : {args.log}")
            _log("   gradlew simulateJava -Pbridge --no-daemon  (first run compiles; be patient)")
            sim = RobotSim(args.repo, args.log, echo=args.echo)
            sim.start()

        with op.OperatorLink() as link, RobotStateLink() as state:
            # `OperatorLink.connect` already waited for the port. NT and the
            # first published pose are the rest of "the robot is actually up".
            state.wait_for_connection(timeout=args.boot_timeout)
            state.wait_for_topic(POSE_TRUTH, timeout=args.boot_timeout)
            state.wait_for_topic(POSE_ODOMETRY, timeout=args.boot_timeout)

            if args.dump_topics:
                dump_topics(state, args.topic_prefix)

            link.neutral()
            time.sleep(SETTLE_SECONDS)

            for check in (check_link, check_echo):
                try:
                    check(state, link)
                except CheckFailed as exc:
                    failures.append(f"{check.__name__}: {exc}")
                    _log(f"   FAIL -- {exc}")

            _step("enabling teleop")
            link.teleop_enable(station="blue1")
            time.sleep(1.0)
            enabled = state.boolean(DS_ENABLED)
            _log(f"   robot reports enabled = {enabled}")
            if not enabled:
                failures.append("enable: robot never reported itself enabled")
                _log("   FAIL -- the DriverStation channel is not being honoured")

            for check in (check_axis, check_button):
                try:
                    check(state, link)
                except CheckFailed as exc:
                    failures.append(f"{check.__name__}: {exc}")
                    _log(f"   FAIL -- {exc}")

            _step("disabling")
            link.neutral()
            link.disable()
            time.sleep(0.5)

    except Exception as exc:
        failures.append(f"{type(exc).__name__}: {exc}")
        _log()
        _log(f"ABORTED: {type(exc).__name__}: {exc}")
        if sim is not None:
            _log()
            _log("last lines of the robot console:")
            for line in sim.tail(30):
                _log(f"   {line}")
    finally:
        if sim is not None and not args.keep_alive:
            _step("stopping robot sim")
            sim.stop()
            # A nonzero code here is the tree-kill, not a robot fault -- gradle
            # has no clean way to be asked to stop a `simulateJava`.
            _log(f"   stopped (exit {sim.returncode}, expected: the sim is killed, not asked)")
            _log(f"   console log kept at {args.log}")

    _step("RESULT")
    if failures:
        for failure in failures:
            _log(f"   FAIL  {failure}")
        _log()
        _log("   the bridge is NOT closed")
        return 1
    _log("   PASS  sparky-sim can launch the robot code, read its state, and command it")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
