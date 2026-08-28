"""Step 6: put five more robots on the field and let sparky-sim drive them.

    python apps/run_bridge_opponents.py
    python apps/run_bridge_opponents.py --seconds 60 --opponents 3 --partners 2
    python apps/run_bridge_opponents.py --opponents 1 --partners 0 --gui

Steps 3 and 4 drove one robot on an empty field. That proves a bridge and
does not make a scenario: a solo cycle only ever exercises the paths where
nothing is in the way, and the failures worth finding in robot code are
the ones where something is.

Five checks, ordered so the first failure names the layer at fault.

    CALIBRATE  the joystick model against the running drive (as step 4)
    CAST       the JVM built the robots we asked for, where we asked
    WIRE       one of them moves when told, and stops when not told
    RUN        every robot on the field decides for itself, at once
    CONTEST    the extra robots actually changed the match

`WIRE` is the one that is easy to skip and should not be. A commanded
robot that does not move and an uncommanded robot that keeps moving are
different bugs with the same symptom -- a field that looks wrong -- and
they live at opposite ends of the wire. Checking the watchdog costs half
a second and is the difference between "the heartbeat works" and "the
heartbeat has never been tested", which is this project's oldest lesson.

`CONTEST` is the one that matters, and it is `PROGRESS` from step 4 one
layer up. Five robots that boot, tick and stand still look exactly like
five robots that are playing: the loop runs, nothing crashes, the report
is green, and the campaign that follows generates precisely the solo
scenarios it was meant to replace.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from bridge import arena
from bridge import match_view as mv
from bridge import operator as op
from bridge import opponents as opp
from bridge import world_state as ws
from bridge.harness import cycle_fuel
from bridge.robot_state import POSE_TRUTH, RobotStateLink
from bridge.sim_process import find_robot_repo, RobotSim
# Step 4's calibration check, imported rather than repeated. The drive
# model is a property of the robot code, not of a particular app, and two
# copies of the tolerance table is two things to update when
# `DriveCommands.joystickDrive` next changes.
from apps.run_bridge_strategy import CheckFailed, check_calibration, _log, _require, _step
from common_sim.control.behavior import BehaviorContext
from common_sim.control.strategy import StrategyController
from common_sim.match.match import Phase

#: How far a robot must land from where it was asked to start before the
#: cast is called wrong. Generous on purpose: dyn4j settles a new body
#: against whatever it is touching, and the check is "did the JVM put
#: them roughly where the roster said" rather than a placement tolerance.
CAST_TOLERANCE_M = 0.5

#: What the WIRE check commands, and for how long.
PROBE_SPEED_MPS = 1.5
PROBE_SECONDS = 1.5

#: How far the probed robot must travel to count as driven. A swerve
#: accelerating to 1.5 m/s covers well over a metre in 1.5 s; half of that
#: is a floor, not a target.
PROBE_MIN_TRAVEL_M = 0.5

#: How far it may travel after the heartbeat stops. `BridgeRobots` zeroes
#: everything after half a second of silence, and a robot at 1.5 m/s
#: coasts through part of that -- so this is the watchdog's latency plus
#: the drivetrain's, not a claim that it stops dead.
WATCHDOG_MAX_DRIFT_M = 1.2

#: How far from its mark the probed robot may be left. Tighter than
#: `CAST_TOLERANCE_M` because this one is under the probe's control, and
#: it exists because getting it wrong is expensive in a way that is very
#: hard to see: the first version of this check probed along x, which on
#: this field drives a robot straight into the side of its own HUB and
#: leaves it wedged there. Everything downstream passed. The robot then
#: spent the whole match pinned, which reads as a defender that would not
#: defend rather than as a probe that parked it in a wall.
PROBE_RETURN_TOLERANCE_M = 0.6

#: What the CONTEST check demands of the extras. Each is a floor chosen so
#: that "the field was populated and inert" cannot pass.
CONTEST_MIN_TRAVEL_M = 2.0
CONTEST_MIN_MOVERS = 2

#: maple-sim publishes its shared battery here, outside AdvantageKit.
BATTERY_VOLTS = "/SmartDashboard/BatterySim/BatteryVoltage (Volts)"
BATTERY_AMPS = "/SmartDashboard/BatterySim/TotalCurrent (Amps)"

#: The rail must stay above this all match. `RoboRioSim.getBrownoutVoltage()`
#: is 6.75 V by default and maple-sim clamps to exactly it, so anything at or
#: below is the clamp having fired. 7.0 leaves room to see it coming.
BROWNOUT_FLOOR_V = 7.0

#: What counts as "asking to move and not moving", for the per-robot stall
#: column. Matched to the liveness oracle's thresholds so that this app and
#: an overnight campaign agree about what a stall is.
STALL_COMMAND_MIN = 1.0  # in/s
STALL_METRES = 0.08


# -- 2. the cast --------------------------------------------------------

def check_cast(link: opp.OpponentLink, roster, timeout: float) -> None:
    _step(f"2. CAST -- ask the simulation for {len(roster)} more robots")
    for entry in roster:
        _log(f"   {entry.name:<7} {entry.alliance:<5} {entry.role:<7} "
             f"({entry.x_in * opp.IN_TO_M:5.2f}, {entry.y_in * opp.IN_TO_M:5.2f}) m")

    link.publish_roster(roster)
    try:
        link.wait_for_cast(len(roster), timeout=timeout)
    except TimeoutError as exc:
        raise CheckFailed(str(exc)) from exc

    # Waited for, not slept on: the poses appear one AdvantageKit cycle
    # after the count does, and reading them in the same breath as the
    # count gets the empty array that was published before the bodies
    # existed.
    deadline = time.monotonic() + 5.0
    poses: list = []
    while time.monotonic() < deadline and len(poses) != len(roster):
        poses = link.poses()
        time.sleep(0.1)

    _require(
        len(poses) == len(roster),
        f"the simulation says it built {link.cast_size()} robots but is publishing "
        f"{len(poses)} poses. FieldSimulation/BridgeRobots is not being written.",
    )
    for entry, pose in zip(roster, poses):
        off = math.hypot(pose.x - entry.x_in * opp.IN_TO_M, pose.y - entry.y_in * opp.IN_TO_M)
        _require(
            off < CAST_TOLERANCE_M,
            f"{entry.name} was asked to start at "
            f"({entry.x_in * opp.IN_TO_M:.2f}, {entry.y_in * opp.IN_TO_M:.2f}) m and is at "
            f"({pose.x:.2f}, {pose.y:.2f}) -- {off:.2f} m away. Either the roster is being "
            f"read in the wrong units or the start pose is inside an obstacle and dyn4j "
            f"pushed the robot out of it.",
        )
    _log(f"   PASS -- {len(poses)} robots exist, each within {CAST_TOLERANCE_M:.1f} m of its mark")


# -- 3. the wire --------------------------------------------------------

def check_wire(link: opp.OpponentLink, count: int) -> None:
    """Drive one extra robot, prove the watchdog stops it, and put it back.

    Deliberately not through a tactic. A robot that fails to move under a
    strategy could be failing anywhere between the trigger and the module
    controllers; a robot that fails to move under a constant 1.5 m/s is
    failing on the wire, and that is a much shorter list of places to look.

    Along y, because the extras start in a lane that is clear of
    everything from wall to wall, while the same probe along x drives them
    into the face of their own HUB. That is not hypothetical: the first
    version of this check did exactly that, every check downstream passed,
    and the wedged robot then spent the whole match pinned -- which reads
    as a defender that would not defend rather than as a probe that parked
    it in a wall.

    And it drives the robot **back to its mark** afterwards, closed-loop
    off the pose rather than by reversing for the same time. Reversing
    does not return it: the acceleration ramp is not symmetric with the
    coast, and the measured overshoot was 0.6 m. A check that leaves the
    field rearranged has changed the experiment it was meant to validate.
    """
    _step("3. WIRE -- one robot moves when told, and stops when not")

    def pose_of(index: int):
        poses = link.poses()
        return poses[index] if index < len(poses) else None

    def command_all(vx: float, vy: float) -> None:
        for index in range(count):
            link.command(index, vx if index == 0 else 0.0, vy if index == 0 else 0.0, 0.0)

    start = pose_of(0)
    _require(start is not None, "no pose is being published for robot 0")

    # Toward the middle of the field, so the probe is open floor either
    # way round and the return leg is not pushing against a wall.
    direction = 1.0 if start.y < arena.FIELD_WIDTH_M / 2 else -1.0

    deadline = time.monotonic() + PROBE_SECONDS
    while time.monotonic() < deadline:
        command_all(0.0, PROBE_SPEED_MPS * direction)
        link.beat()
        time.sleep(0.02)

    moved = pose_of(0)
    travel = math.hypot(moved.x - start.x, moved.y - start.y)
    _log(f"   commanded {PROBE_SPEED_MPS * direction:+.1f} m/s in y for {PROBE_SECONDS:.1f}s "
         f"-> travelled {travel:.2f} m")
    _require(
        travel > PROBE_MIN_TRAVEL_M,
        f"robot 0 was commanded {PROBE_SPEED_MPS:.1f} m/s for {PROBE_SECONDS:.1f}s and moved "
        f"{travel:.2f} m. The command is not reaching the drivetrain -- check that this robot "
        f"build has BridgeRobots.periodic() called from SimContainer, and that the speeds are "
        f"being published to {opp.speeds_key(0)}.",
    )

    # Keep the command on the wire and stop the heartbeat. That is the
    # real test: not "a robot commanded to zero stops", which proves
    # nothing, but "a live command nobody is vouching for stops being
    # obeyed". A crashed harness leaves exactly this state behind.
    command_all(0.0, PROBE_SPEED_MPS * direction)
    time.sleep(1.5)
    settled = pose_of(0)
    drift = math.hypot(settled.x - moved.x, settled.y - moved.y)
    _log(f"   held the command, stopped the heartbeat -> drifted {drift:.2f} m")
    _require(
        drift < WATCHDOG_MAX_DRIFT_M,
        f"robot 0 kept going {drift:.2f} m with a live command and no heartbeat. The "
        f"staleness watchdog in BridgeRobots is not firing, so a crashed harness would "
        f"leave every extra robot driving into a wall for the rest of the session.",
    )

    home = _drive_home(link, count, pose_of, start)
    _log(f"   driven back to its mark          -> {home:.2f} m away")
    _require(
        home < PROBE_RETURN_TOLERANCE_M,
        f"robot 0 finished the probe {home:.2f} m from where it started. The match is about "
        f"to begin from wherever this check left the field, so a probe that does not put the "
        f"robots back is a probe that decides the opening of every run after it.",
    )
    _log("   PASS -- commands arrive, stale ones expire, and the field is as it was")


def _drive_home(link, count, pose_of, target, timeout: float = 6.0) -> float:
    """Walk robot 0 back to `target`, and return how close it got.

    Proportional and capped, with a floor under the speed: a gain alone
    stalls out at a distance where the commanded velocity is smaller than
    what the drivetrain will act on, and the robot then creeps for the
    rest of the timeout.
    """
    deadline = time.monotonic() + timeout
    here = pose_of(0)
    while time.monotonic() < deadline:
        here = pose_of(0)
        dx, dy = target.x - here.x, target.y - here.y
        distance = math.hypot(dx, dy)
        if distance < 0.10:
            break
        speed = min(PROBE_SPEED_MPS, max(0.5, distance * 1.5))
        for index in range(count):
            if index == 0:
                link.command(0, speed * dx / distance, speed * dy / distance, 0.0)
            else:
                link.command(index, 0.0, 0.0, 0.0)
        link.beat()
        time.sleep(0.02)
    link.stand_down(count)
    time.sleep(0.4)
    here = pose_of(0)
    return math.hypot(here.x - target.x, here.y - target.y)


# -- 4. the match -------------------------------------------------------

def run_match(
    link: op.OperatorLink,
    view: mv.MapleMatchView,
    robot: mv.MapleRobot,
    controller: StrategyController,
    cast: opp.OpponentCast,
    seconds: float,
    dt: float,
) -> dict:
    _step(f"4. RUN -- {1 + len(cast.robots)} robots, each deciding for itself, for {seconds:.0f}s")
    header = "      t   ours            " + "".join(f"{r.characteristics.name:<16}" for r in cast.robots)
    _log(header + "loose")

    # Warm every subscription before the clock starts.
    #
    # `RobotStateLink` primes a new subscription by blocking until it
    # carries a value, up to two seconds each, and a first tick creates a
    # dozen of them between the world state and the cast readback. Left
    # inside the loop that cost thirteen seconds of a sixty-second match
    # -- a fifth of the run spent in NT handshakes, with `elapsed` running
    # the whole time. The robots do nothing during it, which is why it
    # showed up as a match that started late rather than as a match that
    # was slow.
    view.sync(0.0, Phase.TELEOP)
    cast.link.poses()
    cast.link.speeds()
    cast.link.held()

    started = time.monotonic()
    start_world = view.sync(0.0, Phase.TELEOP)
    start_poses = [(r.pose.x, r.pose.y) for r in cast.robots]
    # Path length, not displacement. A defender that shadows our robot
    # around the field for a minute can finish where it started, and
    # start-to-end would score that as having never moved.
    travelled = [0.0] * len(cast.robots)
    last_poses = list(start_poses)
    seen_tactics: list[set[str]] = [set() for _ in cast.robots]
    peak_held = [0] * len(cast.robots)
    # Longest stretch of "asking to move and not moving", per robot. The
    # column that separates a defender that has chosen to hold its ground
    # from one that is wedged in a HUB gap -- both of which show up in a
    # travel total as a small number, and only one of which is a finding.
    stall_from: list[float | None] = [None] * len(cast.robots)
    stall_pose: list[tuple | None] = [None] * len(cast.robots)
    longest_stall = [0.0] * len(cast.robots)
    # The shared battery. Watched every tick because it is the one thing
    # the extras change about the robot under test that has no physical
    # counterpart -- see `BridgeRobots.takeOffOurBattery`.
    lowest_volts = 99.0
    peak_amps = 0.0
    ticks = 0
    next_report = 0.0

    while (elapsed := time.monotonic() - started) < seconds:
        world = view.sync(elapsed, Phase.TELEOP)
        context = BehaviorContext(robot=robot, dt=dt, elapsed=elapsed, match=view)
        controller.tick(context)
        # Every extra robot decides against the same instant of the same
        # world our robot just did -- see `OpponentCast.tick`.
        cast.tick(context)
        ticks += 1

        for i, extra in enumerate(cast.robots):
            here = (extra.pose.x, extra.pose.y)
            travelled[i] += math.hypot(here[0] - last_poses[i][0], here[1] - last_poses[i][1])
            last_poses[i] = here
            intent = extra.intent
            seen_tactics[i].add(intent.tactic_name if intent is not None else "-")
            peak_held[i] = max(peak_held[i], len(extra.held_pieces))

            if extra.commanded_speed > STALL_COMMAND_MIN:
                moved_far = (
                    stall_pose[i] is None
                    or math.dist(here, stall_pose[i]) * opp.IN_TO_M > STALL_METRES
                )
                if stall_from[i] is None or moved_far:
                    stall_from[i], stall_pose[i] = elapsed, here
                longest_stall[i] = max(longest_stall[i], elapsed - stall_from[i])
            else:
                stall_from[i], stall_pose[i] = None, None

        volts = view.reader.link.number(BATTERY_VOLTS, 99.0)
        lowest_volts = min(lowest_volts, volts)
        peak_amps = max(peak_amps, view.reader.link.number(BATTERY_AMPS, 0.0))

        if elapsed >= next_report:
            next_report = elapsed + 3.0
            ours = robot.intent.tactic_name if robot.intent is not None else "-"
            cells = "".join(
                f"{(e.intent.tactic_name if e.intent is not None else '-')[:9]:<10}"
                f"{len(e.held_pieces):<6}"
                for e in cast.robots
            )
            _log(f"   {elapsed:5.1f}  {ours[:9]:<10}{world.held:<6}{cells}{world.fuel_count:5d}")

        time.sleep(dt)

    end_world = view.sync(elapsed, Phase.TELEOP)
    robot.release_all()
    cast.stand_down()
    link.neutral()
    link.disable()
    time.sleep(0.5)

    return {
        "ticks": ticks,
        "rate": ticks / max(elapsed, 1e-6),
        "travelled": [t * opp.IN_TO_M for t in travelled],
        "tactics": seen_tactics,
        "peak_held": peak_held,
        "longest_stall": longest_stall,
        "lowest_volts": lowest_volts,
        "peak_amps": peak_amps,
        "our_tactic_count": len(seen_tactics),
        "fuel_start": start_world.fuel_count,
        "fuel_end": end_world.fuel_count,
        "held_end": end_world.held,
        "score": end_world.score.get("blue", 0.0),
        "opponents_seen": _opponents_seen(view, robot),
    }


def _opponents_seen(view, robot) -> int:
    """How many robots our own robot's `world_view` counts as opponents.

    Read through the same helper the tactics use rather than by filtering
    `view.robots` here. The question worth answering is not "are there red
    robots in the list" -- that is obviously true, this app put them there
    -- but "does the strategy layer see them", and those differ if
    anything about the alliance plumbing is wrong.
    """
    from common_sim.control import world_view

    return len(world_view.opponents(view, robot.alliance))


# -- 5. did it matter ---------------------------------------------------

def check_contest(result: dict, cast: opp.OpponentCast, seconds: float) -> None:
    _step("5. CONTEST -- the extra robots actually changed the match")

    for extra, moved, tactics, held, stalled in zip(
        cast.robots, result["travelled"], result["tactics"], result["peak_held"],
        result["longest_stall"],
    ):
        _log(f"   {extra.characteristics.name:<7} {extra.entry.role:<7} "
             f"{moved:6.1f} m  peak held {held:3d}  "
             f"stuck {stalled:4.1f}s  {', '.join(sorted(tactics))}")
    _log(f"   fuel on the floor: {result['fuel_start']} -> {result['fuel_end']}")
    _log(f"   our robot counts {result['opponents_seen']} opponents")
    _log(f"   shared battery: {result['lowest_volts']:.2f} V lowest, "
         f"{result['peak_amps']:.0f} A peak")

    # The extras must not be on our battery. maple-sim's `SimulatedBattery`
    # is static and arena-wide, so without the cancelling appliance in
    # `BridgeRobots` six drivetrains pull the rail to the brownout clamp
    # and throttle the robot under test -- which reads, in every report
    # this project produces, as robot code that browns out under load.
    _require(
        result["lowest_volts"] > BROWNOUT_FLOOR_V,
        f"the shared battery fell to {result['lowest_volts']:.2f} V at "
        f"{result['peak_amps']:.0f} A. maple-sim has one static battery for the whole arena, "
        f"so the extra robots are drawing from the one the robot under test reads as its own "
        f"rail -- and `SimulatedBattery.clamp` is then slowing every motor on the field. "
        f"Check that BridgeRobots.takeOffOurBattery is being called for each new drivetrain.",
    )

    movers = [m for m in result["travelled"] if m >= CONTEST_MIN_TRAVEL_M]
    _require(
        len(movers) >= min(CONTEST_MIN_MOVERS, len(cast.robots)),
        f"only {len(movers)} of {len(cast.robots)} extra robots travelled "
        f"{CONTEST_MIN_TRAVEL_M:.0f} m in {seconds:.0f}s. A populated field that stands still "
        f"is the failure this check exists for: every downstream report would read as a "
        f"contested match and every scenario generated would be a solo one.",
    )

    idle_only = [
        extra.characteristics.name
        for extra, tactics in zip(cast.robots, result["tactics"])
        if tactics <= {"-", "Idle"}
    ]
    _require(
        not idle_only,
        f"{', '.join(idle_only)} never ran a tactic other than Idle. Their strategy's "
        f"triggers are never firing, which usually means the robot's view of the world is "
        f"empty -- check that `MapleMatchView.extra_robots` was populated before the "
        f"controllers were built.",
    )

    expected = sum(1 for e in cast.roster if e.alliance != "blue")
    _require(
        result["opponents_seen"] == expected,
        f"our robot's world_view counts {result['opponents_seen']} opponents and the roster "
        f"has {expected}. The extras are on the field but the strategy layer is not seeing "
        f"them as opposition, so nothing that reads opponents -- Defend, BeingDefended, "
        f"contention, pin pressure -- is being exercised at all.",
    )

    _log(f"   PASS -- {len(movers)} extra robots moved with intent, and our robot sees "
         f"{result['opponents_seen']} of them as opposition")


# -- main ---------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--repo", type=Path, default=None,
                        help="robot project root (default: the sibling checkout, or $SPARKY_ROBOT_REPO)")
    parser.add_argument("--attach", action="store_true")
    parser.add_argument("--seconds", type=float, default=45.0)
    parser.add_argument("--rate", type=float, default=20.0, help="strategy ticks per second")
    parser.add_argument("--shoot-at", type=int, default=8,
                        help="fuel held before a robot goes to score")
    parser.add_argument("--opponents", type=int, default=3)
    parser.add_argument("--partners", type=int, default=2)
    parser.add_argument("--defenders", type=int, default=1,
                        help="how many opponents play defence rather than cycling")
    parser.add_argument("--boot-timeout", type=float, default=300.0)
    parser.add_argument("--gui", action="store_true", help="keep the Sim GUI up to watch")
    parser.add_argument("--log", type=Path, default=Path("build/bridge/opponents-console.log"))
    args = parser.parse_args(argv)
    args.repo = find_robot_repo(args.repo)

    roster = opp.default_roster(
        opponents=args.opponents, partners=args.partners, defenders=args.defenders
    )
    if not roster:
        _log("nothing to do: --opponents 0 --partners 0 is run_bridge_strategy.py")
        return 1

    _log("=" * 64)
    _log("  BRIDGE OPPONENTS")
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

    ran = {"calibrate": False, "cast": False, "wire": False, "run": False, "contest": False}
    try:
        link = op.OperatorLink(connect_timeout=args.boot_timeout)
        with link, RobotStateLink() as state:
            state.wait_for_connection(timeout=args.boot_timeout)
            state.wait_for_topic(POSE_TRUTH, timeout=args.boot_timeout)

            limits = check_calibration(link, state)
            ran["calibrate"] = True

            wire = opp.OpponentLink(state)
            check_cast(wire, roster, timeout=30.0)
            ran["cast"] = True

            check_wire(wire, len(roster))
            ran["wire"] = True

            reader = ws.WorldStateReader(state, alliance="blue")
            robot = mv.MapleRobot(link, limits, alliance="blue")
            view = mv.MapleMatchView(robot, reader)
            controller = StrategyController(cycle_fuel(args.shoot_at), robot)
            robot.controller = controller

            cast = opp.OpponentCast(wire, roster, limits)
            # Attached before the first tick, and that ordering is
            # load-bearing: `Defend` picks its mark from the robots the
            # view knows about, so a controller built against an empty
            # field starts by deciding there is nobody to defend against.
            cast.attach(view, StrategyController)

            link.teleop_enable(station="blue1")
            result = run_match(link, view, robot, controller, cast, args.seconds, 1.0 / args.rate)
            ran["run"] = True

            check_contest(result, cast, args.seconds)
            ran["contest"] = True
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
    _log(f"   PASS  {1 + len(roster)} robots on one field, all of them thinking. The scenarios")
    _log("         this can generate are contested ones now, which is what they were missing.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
