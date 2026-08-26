"""Step 4, second half: let the strategy layer drive the real robot.

    python apps/run_bridge_strategy.py
    python apps/run_bridge_strategy.py --seconds 60 --shoot-at 12

Up to now the bridge has driven the robot from a seeded script that
never asks where anything is. This closes the loop the other way: a real
`StrategyController`, running real `Collect` and `Score` tactics against
`MapleMatchView`, choosing what to do from the live field and pressing
the buttons itself.

Nothing in `common_sim/control` was changed to make this work, and that
is the result being demonstrated. The tactics do not know they are
driving a JVM.

Four checks, ordered so the first failure names the layer at fault.

    CALIBRATE  the joystick-to-velocity model against the running drive
    CONTRACT   every member the strategy layer reads, answered
    RUN        the controller drives the robot for real
    PROGRESS   it actually got something done

`PROGRESS` is the one that matters. A control loop that ticks without
crashing and commands nothing looks exactly like a working one -- the
same trap as an oracle that has never fired, one layer up again.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from bridge import arena
from bridge import drive_model as dm
from bridge import match_view as mv
from bridge import operator as op
from bridge import oracles
from bridge import robot_state as rs
from bridge import world_state as ws
from bridge.robot_state import POSE_TRUTH, RobotStateLink
from bridge.sim_process import find_robot_repo, RobotSim
from common_sim.control.behavior import BehaviorContext
from common_sim.control.strategy import Rule, Strategy, StrategyController
from common_sim.control.tactics import Collect, Idle, Score
from common_sim.control.triggers import AllOf, PiecesHeld, ScoringAvailable
from common_sim.match.match import Phase

# How far the model may disagree with the drive before the inverse is
# considered wrong. Speed and direction get separate budgets because they
# have different noise floors: the speed comparison is clean, while the
# direction comparison goes through a field-to-robot-frame round trip
# whose two headings are sampled a frame apart. See
# `Calibration.direction_error_deg`. Lumping them would mean a tolerance
# either loose enough to hide a real scaling error or tight enough to
# fail on sampling skew every run.
SPEED_TOLERANCE_MPS = 0.15
OMEGA_TOLERANCE_RAD_S = 0.15
DIRECTION_TOLERANCE_DEG = 8.0

# What counts as "asking to move and not moving", for the stall summary.
# Matched to the liveness oracle's own thresholds so the two agree about
# what a stall is.
STALL_COMMAND_MIN = 1.0  # in/s
STALL_METRES = 0.08
STALL_WORTH_REPORTING = 3.0  # seconds


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


def cycle_fuel(shoot_at: int) -> Strategy:
    """Collect fuel until there is enough of it and a HUB will take it.

    About as simple as a strategy gets, and that is the point -- what is
    being tested is the adapter, not the strategy. `shoot_at` is a
    strategy parameter and not a physical one: the intake really does
    hold 40, and `RobotCharacteristics.piece_capacity` says so, but no
    driver waits for 40 before scoring.

    The `ScoringAvailable` half of the shoot trigger is where REBUILT's
    25-second HUB clock enters. It reads through `world_view` to
    `MapleMatchView.region_blocked`, so a robot with plenty of fuel and
    no live HUB does not go and stand in the goal for twenty seconds --
    it keeps collecting, and shoots when the HUB comes back. Without it
    the shoot rule fires on ball count alone and the robot idles through
    half of every cycle, which is what the first working run did.

    Collect's trigger is "not literally full" rather than the complement
    of the shoot threshold, so it stays the thing to do whenever the
    higher-priority rule is not firing, for whichever of the two reasons.
    """
    return Strategy(
        name="cycle_fuel",
        rules=[
            Rule(
                name="shoot_fuel",
                trigger=AllOf(triggers=(
                    PiecesHeld(piece_type=arena.PIECE_TYPE, min_count=shoot_at),
                    ScoringAvailable(),
                )),
                tactic=Score(action=mv.SHOOT),
                priority=10,
            ),
            Rule(
                name="collect_fuel",
                trigger=PiecesHeld(piece_type=arena.PIECE_TYPE, max_count=mv.INTAKE_CAPACITY - 1),
                tactic=Collect(piece_type=arena.PIECE_TYPE, mode="nearest"),
                priority=5,
            ),
        ],
        fallback=Idle(),
    )


# -- 1. the drive model -------------------------------------------------

def check_calibration(link: op.OperatorLink, state: RobotStateLink) -> dm.DriveLimits:
    _step("1. CALIBRATE -- the joystick model against the running drive")
    link.neutral()
    link.teleop_enable(station="blue1")
    time.sleep(1.0)

    limits, checks = dm.calibrate(link, state)
    _log(f"   measured    : {limits.max_speed_mps:.2f} m/s, {limits.max_omega_rad_s:.2f} rad/s "
         f"(DriveConstants nominal {dm.DriveLimits.NOMINAL_MAX_SPEED_MPS:.1f} m/s)")
    for check in checks:
        _log(f"   {check}")

    _require(
        limits.max_speed_mps > 0.5 and limits.max_omega_rad_s > 0.5,
        f"full stick produced {limits.max_speed_mps:.2f} m/s and "
        f"{limits.max_omega_rad_s:.2f} rad/s. The drive is not responding, so nothing "
        "downstream of this can be trusted.",
    )
    _require(bool(checks), "no calibration probe produced a reading at all")
    bad = [
        f"{c.label} (speed {c.speed_error:.3f} m/s, dir {c.direction_error_deg:.1f}deg, "
        f"omega {c.omega_error:.3f} rad/s)"
        for c in checks
        if c.speed_error > SPEED_TOLERANCE_MPS
        or c.omega_error > OMEGA_TOLERANCE_RAD_S
        or c.direction_error_deg > DIRECTION_TOLERANCE_DEG
    ]
    _require(
        not bad,
        f"the joystick model disagrees with the drive at: {'; '.join(bad)}. "
        "bridge/drive_model.py no longer matches DriveCommands.joystickDrive -- most likely "
        "the deadband, the squaring, or the speed multiplier changed. A wrong model here "
        "does not fail loudly: it produces a robot that drives at the wrong speed and "
        "arrives somewhere the navigator did not expect, which reads as a navigation bug.",
    )
    _log(f"   PASS -- {len(checks)} probes agree within {SPEED_TOLERANCE_MPS:.2f} m/s "
         f"and {DIRECTION_TOLERANCE_DEG:.0f} deg")
    return limits


# -- 2. the contract ----------------------------------------------------

# Every `match.` and `robot.` member reached from common_sim/control.
# Listed rather than discovered, because the failure being guarded against
# is the contract *growing* -- a new tactic reaching for something the
# view does not answer, which surfaces at 3am as an AttributeError in the
# middle of a campaign rather than here.
MATCH_MEMBERS = (
    "field", "robots", "phase", "elapsed", "scoring_rules", "region_full", "region_blocked",
    "deposit_region_for", "station_supply", "protecting_zone", "pin_seconds", "active_pieces",
    "events", "config",
)
ROBOT_MEMBERS = (
    "pose", "characteristics", "held_pieces", "alliance", "footprint", "is_full_for",
    "accepts", "nearby_station", "duration_for", "controller", "deposit_active", "intent",
    "drive_field_relative", "set_intake_active", "set_deposit_active",
)


def check_contract(view: mv.MapleMatchView, robot: mv.MapleRobot) -> None:
    _step("2. CONTRACT -- everything the strategy layer reads is answered")
    missing = [name for name in MATCH_MEMBERS if not hasattr(view, name)]
    missing += [f"robot.{name}" for name in ROBOT_MEMBERS if not hasattr(robot, name)]
    _require(
        not missing,
        f"the view does not answer {missing}. The strategy layer would reach for these "
        "mid-match and fail with an AttributeError somewhere in a tactic.",
    )
    _log(f"   {len(MATCH_MEMBERS)} match members, {len(ROBOT_MEMBERS)} robot members -- all present")
    _log(f"   field       : {len(view.field.obstacles)} obstacles, "
         f"{len(view.field.scoring_regions)} scoring regions, "
         f"{len(view.field.intake_locations)} intake locations")
    _log(f"   characteristics: {robot.characteristics.max_speed:.0f} in/s, "
         f"{robot.characteristics.width:.0f}x{robot.characteristics.length:.0f} in, "
         f"intake reach {robot.characteristics.intake_range:.1f} in past the bumper")
    _log("   PASS")


# -- 3. and 4. the loop, and whether it achieved anything ---------------

def run_strategy(
    link: op.OperatorLink,
    view: mv.MapleMatchView,
    robot: mv.MapleRobot,
    controller: StrategyController,
    seconds: float,
    dt: float,
) -> dict:
    _step(f"3. RUN -- the strategy layer drives, for {seconds:.0f}s")
    _log("      t   tactic          pose (m)        target (m)      held  loose  hub   cmd  io")

    started = time.monotonic()
    start_world = view.sync(0.0, Phase.TELEOP)
    start_pose = start_world.robot
    seen_tactics: set[str] = set()
    peak_held = start_world.held
    peak_command = 0.0
    saturated = False
    next_report = 0.0
    ticks = 0
    shot = 0
    shot_in_zone = 0
    last_held = start_world.held
    stall_from: float | None = None
    stall_pose = None
    longest_stall = 0.0
    stall_where = None

    while (elapsed := time.monotonic() - started) < seconds:
        world = view.sync(elapsed, Phase.TELEOP)
        controller.tick(BehaviorContext(robot=robot, dt=dt, elapsed=elapsed, match=view))
        ticks += 1

        intent = robot.intent
        name = intent.tactic_name if intent is not None else "-"
        seen_tactics.add(name)
        peak_held = max(peak_held, world.held)
        peak_command = max(peak_command, robot.commanded_speed)
        saturated = saturated or robot.saturated

        # Longest stretch of "asking to move and not moving". A run can
        # pass every check above while spending a third of itself wedged
        # in a HUB gap, and a PASS that hides that is the same mistake as
        # a report path that only runs on bad news.
        if robot.commanded_speed > STALL_COMMAND_MIN and world.robot is not None:
            if stall_from is None or world.robot.distance_to(stall_pose) > STALL_METRES:
                stall_from, stall_pose = elapsed, world.robot
            longest_stall = max(longest_stall, elapsed - stall_from)
            if longest_stall == elapsed - stall_from:
                stall_where = world.robot
        else:
            stall_from, stall_pose = None, None
        # Every drop in the ball count is fuel that left the robot. Summed
        # rather than taken start-to-end, because the robot refills
        # between shots and the endpoints hide most of it.
        if world.held < last_held:
            # Where the robot was standing decides whether that fuel was
            # *scored at* or *passed*. `Turret.setTarget` aims at the HUB
            # only from inside the alliance zone; outside it, it throws
            # the fuel back toward a corner instead -- and the two are
            # indistinguishable from the ball count alone.
            leaving = last_held - world.held
            shot += leaving
            if arena.in_alliance_zone(robot.pose.x, robot.alliance):
                shot_in_zone += leaving
        last_held = world.held

        if elapsed >= next_report:
            next_report = elapsed + 2.0
            active = [s for s, on in world.hub_active.items() if on]
            # The `io` column is the diagnostic that separates "the tactic
            # never asked" from "it asked and the mechanism did not
            # move". `arm` is the Mechanism2d angle -- 90 stowed, 0
            # deployed -- and is the only observable deploy state there
            # is, since IntakeIOInputs never reach NetworkTables.
            # "Z" means the robot is inside its own alliance zone, i.e.
            # the turret is aiming at the HUB rather than passing.
            io = ("I" if robot.intake_active else "-") + ("D" if robot.deposit_active else "-") \
                + ("Z" if arena.in_alliance_zone(robot.pose.x, robot.alliance) else "-")
            _log(f"   {elapsed:5.1f}  {name:<14}  ({world.robot.x:5.2f},{world.robot.y:5.2f})"
                 f"  {_target_text(intent):<14}  {world.held:4d}  {world.fuel_count:5d}"
                 f"  {(active or ['-'])[0]:<4}  {robot.commanded_speed:5.0f}"
                 f"  {io} arm{world.intake_arm_deg:3.0f}"
                 f"{'  SATURATED' if robot.saturated else ''}")

        time.sleep(dt)

    end_world = view.sync(elapsed, Phase.TELEOP)
    # Read the amperage while whatever the robot is doing is still being
    # done. After `release_all` a pinned robot and an idle one draw the
    # same nothing, and the whole point of the reading is to tell them
    # apart -- the same trap the step 3 preflight had to be hardened
    # against.
    stalled_amps = view.reader.link.drive_current()
    robot.release_all()
    link.neutral()
    link.disable()
    time.sleep(0.5)

    travelled = (
        math.hypot(end_world.robot.x - start_pose.x, end_world.robot.y - start_pose.y)
        if start_pose and end_world.robot else 0.0
    )
    return {
        "ticks": ticks,
        "rate": ticks / max(elapsed, 1e-6),
        "tactics": seen_tactics,
        "travelled": travelled,
        "peak_command": peak_command,
        "saturated": saturated,
        "stalled_amps": stalled_amps,
        "intake_reasserts": robot.intake_reasserts,
        "aim_toggles": robot.aim_toggles,
        "auto_aim": end_world.auto_aim,
        "shot": shot,
        "shot_in_zone": shot_in_zone,
        "longest_stall": longest_stall,
        "stall_where": stall_where,
        "held_start": start_world.held,
        "held_end": end_world.held,
        "peak_held": peak_held,
        "fuel_start": start_world.fuel_count,
        "fuel_end": end_world.fuel_count,
        "score": end_world.score.get("blue", 0.0),
        "hub_fuel": view.reader.fuel_in_hub("blue"),
        "wasted": view.reader.wasted_fuel("blue"),
    }


def _target_text(intent) -> str:
    """Where the active tactic is trying to go, in metres.

    On the report line because "the robot is not moving" and "the robot
    is not moving *and it wants to be three metres west*" send a reader
    to completely different places. Without it the only way to tell a
    piece-selection problem from a navigation one is to guess.
    """
    if intent is None:
        return "-"
    if intent.target_region is not None:
        return intent.target_region[:14]
    piece = intent.target_piece
    if piece is None:
        return "-"
    return f"({piece.position.x / mv.M_TO_IN:5.2f},{piece.position.y / mv.M_TO_IN:5.2f})"


def _stuck_diagnosis(result: dict, seconds: float) -> str:
    """Why the robot did not move -- three answers, not one.

    Reuses exactly the distinction step 3's oracles had to learn: a robot
    that is not moving is either not being told to, told to and not
    hearing, or told to and held. Those have completely different fixes,
    and reporting them as one line ("it did not move") sends the reader
    to the wrong layer. The drive current is what separates the last two,
    the same signal and the same 5 A floor as `oracles.classify_stuck`.
    """
    where = f"the robot moved {result['travelled']:.2f} m in {seconds:.0f}s"
    amps = result["stalled_amps"]

    if result["peak_command"] < 1.0:
        return (f"{where}, and the strategy layer never commanded a velocity "
                f"(peak {result['peak_command']:.1f} in/s). Its tactics ran but asked for "
                "nothing, so the fault is above the adapter -- in the tactic or its target.")
    if amps is not None and amps < oracles.LivenessThresholds().pinned_current_amps:
        return (f"{where} while commanding up to {result['peak_command']:.0f} in/s, with the "
                f"drive motors drawing {amps:.1f} A -- i.e. nothing. The command is not "
                "reaching the drivetrain: the fault is in MapleRobot.drive_field_relative or "
                "the joystick model, not in the strategy.")
    reading = "no current reading" if amps is None else f"{amps:.0f} A"
    return (f"{where} while commanding up to {result['peak_command']:.0f} in/s and drawing "
            f"{reading}. The robot is being *held* -- it is wedged on field geometry and the "
            "navigator is pushing at it rather than routing around. That is a navigation "
            "result, not a plumbing failure, and the place to look is where it stopped.")


def check_progress(result: dict, seconds: float) -> None:
    _step("4. PROGRESS -- did any of that accomplish anything?")
    _log(f"   ticks       : {result['ticks']} ({result['rate']:.1f} Hz)")
    _log(f"   tactics run : {', '.join(sorted(result['tactics']))}")
    _log(f"   travelled   : {result['travelled']:.2f} m")
    _log(f"   held        : {result['held_start']} -> {result['held_end']} (peak {result['peak_held']})")
    _log(f"   loose fuel  : {result['fuel_start']} -> {result['fuel_end']}")
    _log(f"   blue        : score {result['score']:.0f}, "
         f"{result['hub_fuel']:.0f} fuel in HUB, {result['wasted']:.0f} wasted")

    amps = result["stalled_amps"]
    _log(f"   peak command: {result['peak_command']:.0f} in/s"
         f"{'  (saturated at some point)' if result['saturated'] else ''}")
    _log(f"   drive current: {'not published' if amps is None else f'{amps:.0f} A'}")
    # Reported rather than silently compensated for: a rising count means
    # the transport is losing button edges faster than expected, which is
    # a fact about the link and not about the strategy.
    _log(f"   intake edges re-issued: {result['intake_reasserts']}")
    _log(f"   auto-aim    : {'on' if result['auto_aim'] else 'OFF'} "
         f"after {result['aim_toggles']} toggle(s)")
    _log(f"   fuel shot   : {result['shot']} ({result['shot_in_zone']} from inside the "
         f"alliance zone), of which {result['hub_fuel']:.0f} reached the HUB")
    _log(f"   longest stall: {result['longest_stall']:.1f} s"
         + (f" at {result['stall_where']}" if result["stall_where"] is not None else ""))

    if result["longest_stall"] >= STALL_WORTH_REPORTING:
        _log()
        _log(f"   [!] FINDING: the robot spent {result['longest_stall']:.0f}s asking to move "
             "and not moving,")
        _log(f"       at {result['stall_where']}. Scoring in REBUILT means crossing one of the")
        _log("       two ~50 in HUB gaps twice a cycle -- collect at midfield, thread the gap,")
        _log("       shoot from behind the HUB -- and that is where the navigator wedges.")
        _log("       This is the navigation problem the bridge exists to fuzz.")

    # Not a failure of this app -- it tests the bridge, not the robot's
    # aim -- but the loudest thing in the report, because a zero next to
    # "score" is the easiest number in the world to read past. The two
    # branches point at completely different repos.
    if result["shot"] and result["hub_fuel"] == 0:
        _log()
        if result["shot_in_zone"] == 0:
            _log(f"   [!] FINDING: all {result['shot']} shots were taken from *outside* the "
                 "alliance zone,")
            _log("       so the turret was passing rather than scoring "
                 "(Turret.setTarget retargets a")
            _log("       corner of the zone when isInAllianceArea is false). Not a miss. The")
            _log("       strategy layer is being sent to the wrong place -- check the GOAL")
            _log("       regions in bridge/arena.py against RobotContainer.isInAllianceArea.")
        else:
            _log(f"   [!] FINDING: {result['shot_in_zone']} of {result['shot']} shots were taken "
                 "from inside the alliance")
            _log("       zone, where the turret aims at the HUB, and none reached it. These are")
            _log(f"       real misses -- auto-aim reads "
                 f"{'on' if result['auto_aim'] else 'OFF'} and the robot was in position.")
            _log("       RebuiltHub.checkCollision scores a piece within 0.597 m of the HUB")
            _log("       centre *in 3D*, and that centre is 1.57 m up, so this is a shot")
            _log("       calibration question. Replay the kept WPILOG to see where they went.")

    _require(result["travelled"] > 0.5, _stuck_diagnosis(result, seconds))
    _require(
        result["tactics"] - {"-", "Idle"},
        f"no tactic ever ran -- the controller stayed in its fallback for the whole run "
        f"(saw {sorted(result['tactics'])}). Its triggers are not seeing the world.",
    )
    _require(
        result["peak_held"] > result["held_start"] or result["fuel_end"] < result["fuel_start"],
        "the robot never collected a single piece of fuel. It moved and it chose tactics, so "
        "the failure is between the tactic and the mechanism: either the intake mapping in "
        "MapleRobot.set_intake_active, or an intake reach that parks the robot out of range "
        "(see INTAKE_REACH_IN).",
    )
    _log("   PASS -- the strategy layer moved the robot and worked the field")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--repo", type=Path, default=None,
                        help="robot project root (default: the sibling checkout, or $SPARKY_ROBOT_REPO)")
    parser.add_argument("--attach", action="store_true")
    parser.add_argument("--seconds", type=float, default=45.0)
    parser.add_argument("--rate", type=float, default=20.0, help="strategy ticks per second")
    parser.add_argument("--shoot-at", type=int, default=20,
                        help="fuel held before the robot goes to score")
    parser.add_argument("--boot-timeout", type=float, default=300.0)
    parser.add_argument("--gui", action="store_true", help="keep the Sim GUI up to watch")
    parser.add_argument("--log", type=Path, default=Path("build/bridge/strategy-console.log"))
    args = parser.parse_args(argv)
    # Resolved once, here, so the run prints the project it actually
    # picked and a missing one fails before a JVM is started.
    args.repo = find_robot_repo(args.repo)

    _log("=" * 64)
    _log("  BRIDGE STRATEGY LAYER")
    _log("=" * 64)

    sim = None
    if not args.attach:
        _step("launching robot sim")
        _log(f"   repo : {args.repo}")
        args.log.parent.mkdir(parents=True, exist_ok=True)
        gradle = ["simulateJava", "-Pbridge", "--no-daemon"]
        if args.gui:
            gradle.insert(2, "-PbridgeGui")
        sim = RobotSim(args.repo, args.log, gradle_args=tuple(gradle))
        sim.start()

    ran = {"calibrate": False, "contract": False, "run": False, "progress": False}
    try:
        link = op.OperatorLink(connect_timeout=args.boot_timeout)
        with link, RobotStateLink() as state:
            state.wait_for_connection(timeout=args.boot_timeout)
            state.wait_for_topic(POSE_TRUTH, timeout=args.boot_timeout)

            limits = check_calibration(link, state)
            ran["calibrate"] = True

            reader = ws.WorldStateReader(state, alliance="blue")
            robot = mv.MapleRobot(link, limits, alliance="blue")
            view = mv.MapleMatchView(robot, reader)
            controller = StrategyController(cycle_fuel(args.shoot_at), robot)
            robot.controller = controller

            check_contract(view, robot)
            ran["contract"] = True

            result = run_strategy(link, view, robot, controller, args.seconds, 1.0 / args.rate)
            ran["run"] = True
            check_progress(result, args.seconds)
            ran["progress"] = True
    except CheckFailed as exc:
        _step("RESULT")
        _log(f"   FAIL -- {exc}")
        return 1
    except Exception as exc:
        _step("RESULT")
        _log(f"   ERROR -- {type(exc).__name__}: {exc}")
        import traceback
        traceback.print_exc()
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
    _log("   PASS  sparky-sim's strategy layer drove the real robot code: it read the live")
    _log("         field, chose tactics from it, and worked them with its own hands.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
