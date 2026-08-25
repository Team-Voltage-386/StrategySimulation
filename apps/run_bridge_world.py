"""Step 4, first half: prove the live REBUILT world can be read.

    python apps/run_bridge_world.py

Steps 1-3 gave the bridge a robot it can drive and watch for faults. It
still drives blind: the seeded operator in `bridge/scenario.py` pushes a
script and never asks where anything is. Before the strategy layer can
make a decision, something has to be able to answer "what is on the
field" -- and this is the check that it can.

The point worth being explicit about is that **REBUILT does not need
implementing in `game_specific/` for this to work.** maple-sim implements
REBUILT already, including its scoring, and publishes fuel positions, the
HUB clock and a running score over NetworkTables. What it does not
publish is static geometry, and that is transcribed in `bridge/arena.py`
straight from maple-sim's own source. This app checks both halves:

    GEOMETRY    transcribed constants against the poses the arena publishes
    FIELD       the transcription through sparky-sim's own field validator
    SNAPSHOT    one world read, sanity-checked
    POSSESSION  ball count *changes* when the robot drives through fuel
    CLOCK       the active HUB *flips* while the match runs

The last two matter more than they look. A reader that returns a
plausible constant forever is indistinguishable from a working one, and
possession and the HUB clock are the two live quantities the strategy
layer cannot function without -- collect-versus-score turns on the first,
and where to shoot on the second. So neither is checked by reading it
once; both are checked by making the field change and requiring the
reader to notice. Same discipline as the oracles app, one layer up.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from bridge import arena
from bridge import operator as op
from bridge import robot_state as rs
from bridge import world_state as ws
from bridge.robot_state import RobotStateLink, POSE_TRUTH
from bridge.sim_process import DEFAULT_ROBOT_REPO, RobotSim
from common_sim.field.validation import describe_problems, validate_field

# maple-sim's DriveTrainSimulationConfig in SimContainer: 30 x 30 inch bumpers.
ROBOT_SIZE_IN = 30.0


def _log(msg: str = "") -> None:
    print(msg, flush=True)


def _step(title: str) -> None:
    _log()
    _log(f"-- {title} " + "-" * max(0, 58 - len(title)))


class CheckFailed(Exception):
    pass


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise CheckFailed(message)


# -- 1. geometry --------------------------------------------------------

def check_geometry(link: RobotStateLink) -> None:
    _step("1. GEOMETRY -- transcription vs the running arena")
    checks = ws.check_geometry(link)
    for check in checks:
        _log(f"   {check}")

    unpublished = [c for c in checks if c.published is None]
    _require(
        not unpublished,
        f"the arena published none of {[c.what for c in unpublished]}, so the transcribed "
        "geometry could not be checked against anything. Either maple-sim stopped publishing "
        "its goal poses or the key names moved; see bridge/world_state.py GOALS.",
    )
    bad = [c for c in checks if not c.ok]
    _require(
        not bad,
        f"{len(bad)} transcribed constant(s) disagree with the arena by more than "
        f"{ws.GEOMETRY_TOLERANCE_M * 1000:.0f} mm. bridge/arena.py was copied from a different "
        "version of maple-sim than the one running; re-read Arena2026Rebuilt.java.",
    )
    _log(f"   PASS -- {len(checks)} constants agree with the live arena")


# -- 2. the field itself ------------------------------------------------

def check_field() -> None:
    _step("2. FIELD -- the transcribed arena through the field validator")
    field = arena.build_arena()
    problems = validate_field(field, robot_width=ROBOT_SIZE_IN, robot_length=ROBOT_SIZE_IN)

    _log(f"   {len(field.obstacles)} obstacles, {len(field.scoring_regions)} scoring regions, "
         f"{field.width:.1f} x {field.height:.1f} in")
    for line in (describe_problems(problems) or "no problems").splitlines():
        _log(f"   {line}")

    errors = [p for p in problems if p.severity == "error"]
    _require(
        not errors,
        f"the transcribed field has {len(errors)} error-level problem(s). Navigating a field "
        "the validator rejects produces failures that belong to the field, not the robot code.",
    )

    # Not an assertion -- a note for whoever reads the output. The two
    # HUB colliders are the whole story of this field's navigation, and
    # the width of the gap they leave is the number that explains why
    # matches wedge.
    hub = next(o for o in field.obstacles if o.name == "blue HUB")
    top = min(v[1] for v in hub.vertices)
    _log(f"   the HUB colliders leave {top:.1f} in at each end of the field to get past; "
         f"a {ROBOT_SIZE_IN:.0f} in robot turns in {ROBOT_SIZE_IN * 2 ** 0.5:.1f} in")
    _log("   PASS -- no error-level problems")


# -- 3. one snapshot ----------------------------------------------------

def check_snapshot(reader: ws.WorldStateReader) -> ws.WorldState:
    _step("3. SNAPSHOT -- one read of the live world")
    world = reader.read()

    _log(f"   robot       : {world.robot}")
    _log(f"   fuel loose  : {world.fuel_count}")
    _log(f"   held        : {world.held}")
    _log(f"   match clock : {world.match_clock:.1f} s")
    _log(f"   hub phase   : {world.phase_clock:.1f} s left, active = "
         f"{[side for side, on in world.hub_active.items() if on] or 'neither'}")
    _log(f"   score       : {world.score}")

    _require(world.robot is not None, f"{POSE_TRUTH} published nothing; the robot has no pose")
    _require(
        0 <= world.robot.x <= arena.FIELD_LENGTH_M and 0 <= world.robot.y <= arena.FIELD_WIDTH_M,
        f"the robot is at {world.robot}, which is off a "
        f"{arena.FIELD_LENGTH_M:.2f} x {arena.FIELD_WIDTH_M:.2f} m field -- the pose is being "
        "decoded wrong, or these are not the same field's coordinates",
    )
    _require(
        world.fuel_count > 0,
        "no fuel on the field at all. FieldSimulation/Fuel decoded to an empty array, which "
        "means either the arena never placed pieces or the struct decode is wrong.",
    )
    nearest = world.nearest_fuel()
    _log(f"   nearest fuel: {nearest} "
         f"({((nearest.x - world.robot.x) ** 2 + (nearest.y - world.robot.y) ** 2) ** 0.5:.2f} m away)")
    _log("   PASS -- the world reads as a field with a robot and pieces on it")
    return world


# -- 4. possession, and 5. the clock ------------------------------------

def check_live(link: op.OperatorLink, reader: ws.WorldStateReader, clock_timeout: float) -> None:
    """Both of the checks that need the world to *change*, in one enabled window.

    Sharing the window is not just for speed. The HUB clock only advances
    while the robot is enabled and out of autonomous, so anything waiting
    on it has to be holding teleop anyway -- and driving through the fuel
    pile is a better thing to do with those seconds than idling.
    """
    _step("4. POSSESSION -- does driving through fuel change the ball count?")
    before = reader.read()

    link.neutral()
    link.teleop_enable(station="blue1")
    time.sleep(1.0)

    # Manip Y is DeployIntake (RobotContainer). IntakeIOSim starts the
    # maple-sim IntakeSimulation as soon as the mechanism reads deployed,
    # so nothing else has to be pressed for it to collect.
    stowed = reader.link.number(rs.INTAKE_ARM_ANGLE)
    link.tap(op.BTN_Y, joystick=1, hold=0.3)
    time.sleep(1.0)
    deployed = reader.link.number(rs.INTAKE_ARM_ANGLE)
    _log(f"   intake arm  : {stowed:.0f} deg -> {deployed:.0f} deg  (90 stowed, 0 deployed)")

    # Standing in a pile is not the same as intaking from it, and this is
    # where two attempts went wrong before the geometry got read properly.
    #
    # `IntakeSimulation.OverTheBumperIntake(..., FRONT, ...)` puts the
    # collecting fixture 0.37 to 0.68 m ahead of the robot centre along
    # its *heading*. `joystickDrive` is field-relative, so the robot can
    # translate in any direction while still facing +x -- and it collects
    # only what passes through that forward band. Strafing north through
    # the grid slides fuel down the robot's flank, half a metre from the
    # intake the whole way; reversing west drags the intake over ground
    # the bumper has already pushed clear. Both were tried, and the
    # nearest fuel sat pinned at 0.53 m from the intake for eight seconds.
    #
    # So the robot has to arrive west of the pile and drive *forward*
    # through it. The centre grid spans x 7.36..9.03, y 1.72..5.80, and
    # the robot starts at (8.79, 0.82) facing +x: south of the grid and
    # already past its eastern edge. Three legs -- back out west, step
    # north into the rows, then sweep east with the intake leading.
    link.set_axis(op.AXIS_LEFT_Y, 0.6)  # +leftY is -x: back out west
    time.sleep(2.4)
    link.set_axis(op.AXIS_LEFT_Y, 0.0)
    link.set_axis(op.AXIS_LEFT_X, -0.6)  # -leftX is +y: north into the rows
    time.sleep(1.8)
    link.set_axis(op.AXIS_LEFT_X, 0.0)
    link.set_axis(op.AXIS_LEFT_Y, -0.6)  # forward, intake first
    time.sleep(3.2)
    link.set_axis(op.AXIS_LEFT_Y, 0.0)
    time.sleep(1.0)

    after = reader.read()
    _log(f"   held        : {before.held} -> {after.held}")
    _log(f"   fuel loose  : {before.fuel_count} -> {after.fuel_count}")
    _log(f"   robot       : {before.robot} -> {after.robot}")
    _require(
        after.held != before.held or after.fuel_count != before.fuel_count,
        f"the robot drove through the fuel pile and neither the ball count ({before.held}) nor "
        f"the loose-fuel count ({before.fuel_count}) moved. Possession is the signal "
        "collect-versus-score turns on, and a reader that returns the same number forever "
        f"looks exactly like a working one. The intake arm went {stowed:.0f} -> {deployed:.0f} "
        "degrees, which says whether it never came down or came down and missed.",
    )
    _log("   PASS -- possession and the loose-fuel count are live, not defaults")

    _step("5. CLOCK -- does the active HUB flip while the match runs?")
    start_active = dict(after.hub_active)
    _log(f"   active now  : {[s for s, on in start_active.items() if on] or 'neither'} "
         f"({after.phase_clock:.1f} s left in phase)")

    deadline = time.monotonic() + clock_timeout
    flipped = None
    while time.monotonic() < deadline:
        now = reader.read()
        if now.hub_active != start_active:
            flipped = now
            break
        time.sleep(0.5)

    _require(
        flipped is not None,
        f"the active HUB did not change in {clock_timeout:.0f}s of enabled teleop. "
        f"Arena2026Rebuilt swaps it every {arena.HUB_PHASE_SECONDS:.0f}s, so either the robot "
        "was not really enabled or these booleans are reading their default rather than the "
        "published value.",
    )
    _log(f"   flipped to  : {[s for s, on in flipped.hub_active.items() if on] or 'neither'} "
         f"after {clock_timeout - (deadline - time.monotonic()):.1f} s")
    _log("   PASS -- the HUB clock is live")

    link.neutral()
    link.disable()
    time.sleep(0.5)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--repo", type=Path, default=DEFAULT_ROBOT_REPO)
    parser.add_argument("--attach", action="store_true", help="use a sim that is already running")
    parser.add_argument("--boot-timeout", type=float, default=300.0)
    parser.add_argument("--clock-timeout", type=float, default=40.0,
                        help=f"how long to wait for the {arena.HUB_PHASE_SECONDS:.0f}s HUB swap")
    parser.add_argument("--log", type=Path, default=Path("build/bridge/world-console.log"))
    args = parser.parse_args(argv)

    _log("=" * 64)
    _log("  BRIDGE WORLD-STATE READER")
    _log("=" * 64)

    sim = None
    if not args.attach:
        _step("launching robot sim")
        _log(f"   repo : {args.repo}")
        args.log.parent.mkdir(parents=True, exist_ok=True)
        sim = RobotSim(args.repo, args.log, gradle_args=("simulateJava", "-Pbridge", "--no-daemon"))
        sim.start()

    # Which checks actually executed. "No failures" and "never ran" are the
    # same empty list, and the oracles app already reported a run that died
    # in gradle configuration as a clean pass once.
    ran = {"geometry": False, "field": False, "snapshot": False, "live": False}
    try:
        link = op.OperatorLink(connect_timeout=args.boot_timeout)
        with link, RobotStateLink() as state:
            state.wait_for_connection(timeout=args.boot_timeout)
            state.wait_for_topic(POSE_TRUTH, timeout=args.boot_timeout)
            reader = ws.WorldStateReader(state, alliance="blue")

            check_geometry(state)
            ran["geometry"] = True
            check_field()
            ran["field"] = True
            check_snapshot(reader)
            ran["snapshot"] = True
            check_live(link, reader, args.clock_timeout)
            ran["live"] = True
    except CheckFailed as exc:
        _step("RESULT")
        _log(f"   FAIL -- {exc}")
        return 1
    except Exception as exc:
        _step("RESULT")
        _log(f"   ERROR -- {type(exc).__name__}: {exc}")
        return 2
    finally:
        if sim is not None:
            _step("stopping robot sim")
            sim.stop()
            _log(f"   console log kept at {args.log}")

    _step("RESULT")
    skipped = [name for name, done in ran.items() if not done]
    if skipped:
        _log(f"   INCOMPLETE -- never ran: {', '.join(skipped)}")
        return 1
    _log("   PASS  the live REBUILT world is readable: geometry agrees with the arena,")
    _log("         the field validates, and possession and the HUB clock both move.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
