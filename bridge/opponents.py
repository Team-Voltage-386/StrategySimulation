"""Step 6: the other five robots, driven by sparky-sim.

Everything before this drove one robot on an empty field. That is enough
to prove a bridge works and not enough to be worth running overnight: a
solo cycle exercises the paths where nothing is in the way, and the
interesting failures in robot code are the ones where something is.

maple-sim will hold as many drivetrains as it is given, and has no
opinion about what any of them does -- its own answer for opponents is a
PathPlanner replay or a second gamepad, neither of which decides
anything. sparky-sim has five years of decision-making and no way to
touch a real robot. This module is the join: `BridgeRobots.java` owns the
bodies, and everything here says where they should go.

**The extra robots are not props.** Each one is a real `Robot` running a
real `StrategyController` against the same `MapleMatchView` our robot
reads, so it collects the same fuel, navigates around the same obstacles,
and reacts to what our robot is doing. That is the point: an opponent
that follows a recorded path cannot contest anything, because contesting
means changing your mind about a piece that somebody else just took.

Three things this makes reachable that step 4 could not:

* **Fuel that goes away.** `Collect` re-targeting when the piece it was
  driving to is taken, which is the single most common thing that
  happens in a real match and never happened here.
* **Bodies in the way.** `plan_path` around obstacles that move, and the
  wedges, pins and stalls a full field produces. Every stall this project
  has chased was found on a field with one robot on it.
* **A defended goal.** `Defend` is machinery this repo already has and
  has never pointed at real robot code.

Units: sparky-sim's inches everywhere in this file's own API, converted
to the wire's metres in exactly one place -- `OpponentLink`. Same rule as
`match_view.py`, for the same reason.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

import pymunk

from bridge import arena
from bridge import drive_model as dm
from bridge import match_view as mv
from bridge import robot_state as rs
from common_sim.geometry import Pose2d
from common_sim.robot.robot import Robot

M_TO_IN = arena.M_TO_IN
IN_TO_M = 1.0 / M_TO_IN

# -- the wire -----------------------------------------------------------
# Inbound to the JVM is plain NetworkTables under `/Bridge`: these are
# *inputs*, and AdvantageKit's output namespace is for what the robot
# says, not for what it is told. Outbound comes back through AdvantageKit
# so that AdvantageScope and the replay get the extra robots for free.

TABLE = "/Bridge/Robots"
ROSTER = f"{TABLE}/Roster"
TICK = f"{TABLE}/Tick"

POSES = f"{rs.OUTPUTS}/FieldSimulation/BridgeRobots"
SPEEDS = f"{rs.OUTPUTS}/FieldSimulation/BridgeRobotsSpeeds"
HELD = f"{rs.OUTPUTS}/FieldSimulation/BridgeRobotsHeld"
COUNT = f"{rs.OUTPUTS}/Bridge/Robots/Count"

#: Six robots on a field, minus the one that is the robot code. Must
#: agree with `BridgeRobots.MAX_ROBOTS`; the JVM refuses a longer roster
#: outright, so a disagreement shows up as a cast that never appears
#: rather than as a cast that is quietly short.
MAX_ROBOTS = 5


def speeds_key(index: int) -> str:
    return f"{TABLE}/{index}/Speeds"


def intake_key(index: int) -> str:
    return f"{TABLE}/{index}/Intake"


def release_key(index: int) -> str:
    return f"{TABLE}/{index}/Release"


# -- the cast -----------------------------------------------------------


@dataclass(frozen=True)
class RosterEntry:
    """One extra robot: where it starts, whose side it is on, and what it
    is trying to do.

    `alliance` is Python's business alone -- the JVM makes bodies and has
    no idea which way any of them ought to be shooting. Everything that
    follows from it (which HUB is theirs, whether they count as an
    opponent, which pieces they contest) is decided here, by the same
    `world_view` code that decides it in the pure sim.
    """

    x_in: float
    y_in: float
    heading: float
    alliance: str
    role: str = "cycle"
    name: str = ""

    def pose(self) -> Pose2d:
        return Pose2d(self.x_in, self.y_in, self.heading)


#: How far off its own wall an extra robot starts, in metres.
#:
#: 2.30 m puts it clear of its own HUB, which reaches to x = 4.00 m on the
#: blue side and back to 11.34 m on the red. Starting a body inside an
#: obstacle is a dyn4j overlap to resolve, which is to say a robot that
#: begins the match by being flung.
START_STANDOFF_M = 2.30

#: Where along the wall they line up, in metres, low to high.
#:
#: Chosen off the HUB rather than by eye: the HUBs span y = 1.28 .. 6.79 m,
#: and the 50-inch gaps at either end of them are the only ways across the
#: field, so a robot starting at 1.60 or 6.45 begins the match beside the
#: gap it will have to use. The middle lane starts behind its own HUB and
#: has to go round, which is a different and equally worth-having opening.
START_LANES_M = (1.60, 4.03, 6.45)

#: Which lanes get used for a partial alliance: outermost first, so two
#: robots straddle the field rather than crowding one corner, and a lone
#: one starts in the middle where it can go either way.
_LANES_FOR = {0: (), 1: (1,), 2: (0, 2), 3: (0, 1, 2)}


def default_roster(
    *,
    ours: str = "blue",
    opponents: int = 3,
    partners: int = 2,
    defenders: int = 1,
) -> tuple[RosterEntry, ...]:
    """A full field: `partners` more on our side, `opponents` on the other.

    The default is a real match minus us -- two partners, three opponents,
    one of whom plays defence. `defenders` counts from the front of the
    opponent list, so a campaign that wants a quieter field can turn
    opponents down to one and still have that one defending.

    Opponents face us and partners face away, which is only cosmetic --
    every tactic drives field-relative and turns to whatever it needs --
    but it makes a screenshot of the start of a match legible.
    """
    theirs = "red" if ours == "blue" else "blue"
    if opponents + partners > MAX_ROBOTS:
        raise ValueError(
            f"{opponents} opponents and {partners} partners is {opponents + partners} extra "
            f"robots, and BridgeRobots refuses more than {MAX_ROBOTS}"
        )
    if opponents > 3 or partners > 2:
        raise ValueError(
            f"an FRC alliance is three robots and one of ours is the robot code, so at most "
            f"3 opponents and 2 partners -- asked for {opponents} and {partners}"
        )

    entries: list[RosterEntry] = []
    for alliance, wanted in ((theirs, opponents), (ours, partners)):
        # Away from midfield: blue's own wall is at x = 0, red's at the far end.
        wall_m = START_STANDOFF_M if alliance == "blue" else arena.FIELD_LENGTH_M - START_STANDOFF_M
        for slot, lane in enumerate(_LANES_FOR[wanted]):
            entries.append(RosterEntry(
                x_in=wall_m * M_TO_IN,
                y_in=START_LANES_M[lane] * M_TO_IN,
                heading=math.pi if alliance == "red" else 0.0,
                alliance=alliance,
                role="defend" if alliance == theirs and slot < defenders else "cycle",
                # Numbered as a driver station would be, with our own robot
                # holding the 1 on its alliance.
                name=f"{alliance}{slot + (2 if alliance == ours else 1)}",
            ))
    return tuple(entries)


# -- the link -----------------------------------------------------------


class OpponentLink:
    """Python -> the extra robots, and back.

    Rides on the `RobotStateLink` that is already open rather than
    starting a second NT client. One connection, one server address, one
    thing to close -- and the readback comes through the same subscriber
    cache as everything else the robot publishes.
    """

    def __init__(self, state: rs.RobotStateLink):
        self.state = state
        self._tick = 0

    # -- setting the stage ----------------------------------------------

    def publish_roster(self, entries) -> None:
        """Ask for these robots, at these poses.

        Sent once and honoured once: `BridgeRobots` latches the first
        non-empty roster it sees and never looks again. A match's cast
        does not change halfway through, and a re-readable roster would
        make "move a robot" and "add a robot" the same message.
        """
        flat: list[float] = []
        for entry in entries:
            flat += [entry.x_in * IN_TO_M, entry.y_in * IN_TO_M, entry.heading]
        self.state.publish_double_array(ROSTER, flat)
        self.state.flush()

    def cast_size(self) -> int:
        """How many robots the JVM says it made. Zero until it has."""
        return self.state.integer(COUNT, 0)

    def wait_for_cast(self, expected: int, timeout: float = 10.0, poll: float = 0.1) -> None:
        """Block until the JVM confirms it built the whole cast.

        Waited on rather than assumed, because a roster the JVM refuses --
        too long, or not a whole number of triples -- fails by making
        *nothing*, and a match that quietly ran with an empty field would
        report as a clean solo run. That is the same shape of mistake as
        an oracle that never fires.
        """
        import time

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            made = self.cast_size()
            if made == expected:
                return
            time.sleep(poll)
        raise TimeoutError(
            f"asked for {expected} extra robots and the simulation reports {self.cast_size()}. "
            f"Check that this robot build has BridgeRobots wired into SimContainer, and that "
            f"the roster is at most {MAX_ROBOTS} (x, y, theta) triples."
        )

    # -- driving ---------------------------------------------------------

    def command(
        self,
        index: int,
        vx_mps: float,
        vy_mps: float,
        omega: float,
        *,
        intake: bool = False,
        release: bool = False,
    ) -> None:
        """One robot's setpoint for the next tick. Field-relative metres."""
        self.state.publish_double_array(speeds_key(index), (vx_mps, vy_mps, omega))
        self.state.publish_boolean(intake_key(index), intake)
        self.state.publish_boolean(release_key(index), release)

    def beat(self) -> None:
        """Say that the far side is still being driven, and push the batch.

        The heartbeat is what stops a dropped connection from leaving five
        robots executing their last command into a wall for the rest of
        the session: `BridgeRobots` zeroes everything after half a second
        without one. It is a counter and not a timestamp because the two
        processes do not share a clock.

        Flushing here rather than per-command is what makes a tick's worth
        of commands land together. NT4 batches at 100 Hz on its own, so
        without this a 50 Hz loop's commands arrive smeared across the
        next network period, and five robots lurch instead of driving.
        """
        self._tick += 1
        self.state.publish_integer(TICK, self._tick)
        self.state.flush()

    def stand_down(self, count: int) -> None:
        """Stop every extra robot, now, and mean it.

        Called when a match ends rather than left to the watchdog. The
        watchdog is the backstop for a crash; a clean finish should not
        need half a second of coasting to come to rest, because the next
        thing that happens is usually a screenshot or a score reading.
        """
        for index in range(count):
            self.command(index, 0.0, 0.0, 0.0, intake=False, release=False)
        self.beat()

    # -- reading back ----------------------------------------------------

    def poses(self) -> list[rs.Pose2d]:
        """Where the extra robots actually are. Metres, blue-origin."""
        return self.state.pose2d_array(POSES) or []

    def speeds(self) -> list[tuple[float, float, float]]:
        """Field-relative (vx, vy, omega) per robot, metres and radians."""
        flat = self.state.double_array(SPEEDS)
        return [tuple(flat[i:i + 3]) for i in range(0, len(flat) - 2, 3)]

    def held(self) -> list[int]:
        """How many pieces each extra robot is carrying."""
        return self.state.integer_array(HELD)


# -- the robots ---------------------------------------------------------


class OpponentRobot(Robot):
    """A `Robot` whose pose arrives over NT and whose commands leave the
    same way.

    The sibling of `match_view.MapleRobot`, and deliberately much smaller
    than it. `MapleRobot` is complicated because a HALSim joystick is a
    lossy, edge-triggered, debounce-needing way to say "run the intake";
    this one writes a boolean to a field. Everything `MapleRobot` does
    about lost edges, re-asserted presses and toggle reconciliation is
    absent here because none of it is needed -- which is worth saying
    out loud, because the resemblance invites copying that machinery
    across.

    Named after the opponents, used for partners too. A robot on our own
    alliance driven the same way is the same object with a different
    `alliance`, and the strategy layer is what makes the difference mean
    something.
    """

    def __init__(
        self,
        link: OpponentLink,
        index: int,
        entry: RosterEntry,
        limits: dm.DriveLimits,
    ):
        # Its own space, never stepped -- see `MapleRobot.__init__`. The
        # bodies here are somewhere to write a pose that arrives over the
        # wire, and the physics that matters is happening in the JVM.
        self._space = pymunk.Space()
        self.link = link
        self.index = index
        self.entry = entry
        self.limits = limits

        # The real robot's characteristics with a different name on them.
        # Same drivetrain, same reach, same capacity: an opponent that is
        # quicker or longer-armed than the robot under test turns every
        # contested piece into a statement about the difference. The JVM
        # makes the same choice for the same reason -- see
        # `BridgeRobots.CONFIG`.
        super().__init__(
            self._space,
            replace(mv.robot_characteristics(limits), name=entry.name or f"robot{index}"),
            entry.pose(),
            alliance=entry.alliance,
        )

        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0
        self._intake = False
        self._release = False

    # -- reads ------------------------------------------------------------

    def sync(self, pose, speeds, held: int) -> None:
        """Copy one robot's row of the readback into this body."""
        mv.write_measured_pose(self, pose, None)
        if speeds is not None:
            # Already field-relative, unlike our own robot's measured
            # speeds -- `getActualSpeedsFieldRelative` does the rotation
            # in the JVM where the true heading is. So it goes straight
            # into the body rather than through `write_measured_pose`,
            # which exists to rotate a robot-relative reading.
            vx, vy, omega = speeds
            self.chassis.body.velocity = (vx * M_TO_IN, vy * M_TO_IN)
            self.chassis.body.angular_velocity = omega
        mv.reconcile_held(self, self._space, held)

    @property
    def intake_active(self) -> bool:
        """Mirrors `MapleRobot.intake_active`: what a tactic asked for."""
        return self._intake

    # -- writes -----------------------------------------------------------

    def drive_field_relative(self, dt: float, vx: float, vy: float, omega: float) -> None:
        """Field-relative inches/s in, metres/s onto the wire.

        The inherited call still runs, and it matters: it updates
        `commanded_velocity` without touching the body, which is what
        `commanded_speed` reads -- and `commanded_speed` is how the
        liveness oracle and several tactics tell a robot that is waiting
        from one that is being held. An opponent pinned against a wall is
        exactly the state this bridge exists to produce, so it must not be
        the state whose detection is quietly disarmed.

        Clamped to the same drivetrain the real robot has. The JVM's
        swerve sim would clamp it anyway, at the module level and
        silently; doing it here means `commanded_velocity` describes a
        velocity that can actually happen.
        """
        super().drive_field_relative(dt, vx, vy, omega)
        vx_mps, vy_mps = vx * IN_TO_M, vy * IN_TO_M
        speed = math.hypot(vx_mps, vy_mps)
        if speed > self.limits.max_speed_mps and speed > 0.0:
            scale = self.limits.max_speed_mps / speed
            vx_mps, vy_mps = vx_mps * scale, vy_mps * scale
        self._vx, self._vy = vx_mps, vy_mps
        self._omega = max(-self.limits.max_omega_rad_s, min(self.limits.max_omega_rad_s, omega))

    def set_intake_active(self, active: bool) -> None:
        self._intake = bool(active)

    def set_deposit_active(self, active: bool, action: str | None = None) -> None:
        """Deposit means *dump*, for an extra robot.

        This is the one place the extra robots are honestly less than the
        real one, and it is worth being plain about rather than hiding
        behind the word "score". `BridgeRobots` gives an opponent a
        drivetrain and an intake and no shooter, so pressing deposit puts
        its fuel back on the floor behind it instead of into a HUB.

        Building an opponent shooter would mean transcribing this game's
        ballistics into the opponent's side of the arena -- REBUILT work,
        on the game this bridge was only ever proved against. What the
        extra robots are for is bodies that move with intent and pieces
        that get taken; a dump contests both, and it also keeps fuel
        circulating instead of letting three opponents hoover the field
        into their hoppers and stop.

        The visible consequence is that maple-sim's score for the other
        alliance stays at zero. Nothing reads it, and a differential
        scoring oracle must not start.
        """
        super().set_deposit_active(active, action)
        self._release = bool(active)

    def publish(self) -> None:
        """Send this tick's commands. Called once per tick, after the
        controller has run and before the heartbeat."""
        self.link.command(
            self.index, self._vx, self._vy, self._omega,
            intake=self._intake, release=self._release,
        )

    def release_all(self) -> None:
        """Drop everything. `Robot`'s contract for a disabled robot."""
        super().release_all()
        self._release = True


# -- the whole field ----------------------------------------------------


class OpponentCast:
    """Every extra robot, and the loop that drives them.

    Owns the link, the robots and one `StrategyController` each, and
    exposes the two calls a match loop needs: `deploy` before the match
    and `tick` during it.

    Attached to a `MapleMatchView` rather than owned by it, because the
    view is a *view* -- it reads a running simulation and decides nothing.
    Putting five controllers inside it would make `sync` a thing that
    drives robots, which is precisely the shape of coupling that made
    `match.robots` worth answering honestly in the first place.
    """

    def __init__(
        self,
        link: OpponentLink,
        roster,
        limits: dm.DriveLimits,
        *,
        strategy_for=None,
    ):
        self.link = link
        self.roster = tuple(roster)
        self.robots = [
            OpponentRobot(link, index, entry, limits)
            for index, entry in enumerate(self.roster)
        ]
        self.controllers: list = []
        self._strategy_for = strategy_for or default_strategy_for

    def deploy(self, timeout: float = 10.0) -> None:
        """Publish the roster and wait for the bodies to exist."""
        self.link.publish_roster(self.roster)
        self.link.wait_for_cast(len(self.roster), timeout=timeout)

    def attach(self, view, controller_factory) -> None:
        """Put the extra robots into the view, and give each one a brain.

        `controller_factory` is passed in rather than imported so that
        this module stays free of `common_sim.control` at import time --
        the same reason `harness.cycle_fuel` does its imports inside the
        function. The scripted driver and the rules tests import this
        file's constants and have neither pymunk's strategy stack nor a
        reason to.
        """
        view.extra_robots = list(self.robots)
        self.controllers = [
            controller_factory(self._strategy_for(robot.entry), robot)
            for robot in self.robots
        ]
        for robot, controller in zip(self.robots, self.controllers):
            robot.controller = controller

    def tick(self, context) -> None:
        """Read where everyone is, decide, and command. Once per loop.

        Takes a ready-made `BehaviorContext` for our own robot and swaps
        the robot in for each extra one, so every robot on the field is
        deciding against the same instant of the same world. Building a
        fresh context per robot would let `elapsed` drift between them
        within a tick, which is the sort of difference that shows up much
        later as two robots disagreeing about who got somewhere first.
        """
        poses = self.link.poses()
        speeds = self.link.speeds()
        held = self.link.held()
        for index, robot in enumerate(self.robots):
            robot.sync(
                poses[index] if index < len(poses) else None,
                speeds[index] if index < len(speeds) else None,
                held[index] if index < len(held) else 0,
            )
        for robot, controller in zip(self.robots, self.controllers):
            controller.tick(_with_robot(context, robot))
            robot.publish()
        self.link.beat()

    def stand_down(self) -> None:
        self.link.stand_down(len(self.robots))


def _with_robot(context, robot):
    """The same context, pointed at a different robot.

    `dataclasses.replace` would be the obvious thing and does not work:
    `BehaviorContext` is a plain class in some versions of this codebase
    and a dataclass in others, and a tactic reads `context.match` far more
    often than anything else. Copying the instance keeps whatever fields
    it happens to carry.
    """
    import copy

    swapped = copy.copy(context)
    swapped.robot = robot
    return swapped


def default_strategy_for(entry: RosterEntry):
    """What an extra robot plays, from its roster role.

    Two roles, and the split is deliberate. `cycle` is the same strategy
    our own robot runs -- the point of a partner or a second scorer is
    another body wanting the same fuel, and the cheapest honest way to get
    one is to run the strategy that is already known to work. `defend`
    is the interesting one and the reason the field is worth populating at
    all: `Defend` has existed in this repo for a long time and has never
    once been pointed at real robot code.

    A `cycle` opponent that fills up will drive to its own HUB and press
    deposit, which for an extra robot means dumping on the floor there --
    see `OpponentRobot.set_deposit_active`. That is not scoring and is not
    pretending to be; it is what keeps fuel moving instead of ending the
    match with three full hoppers parked in a corner.
    """
    from bridge.harness import cycle_fuel
    from common_sim.control.strategy import Rule, Strategy
    from common_sim.control.tactics import Defend, Idle
    from common_sim.control.triggers import Always

    if entry.role == "defend":
        return Strategy(
            name="defend",
            rules=[
                Rule(
                    name="deny",
                    trigger=Always(),
                    # "shadow" and not "block": this field has no protected
                    # zones (`MapleMatchView.protecting_zone` returns None
                    # because maple-sim adjudicates no fouls), and with
                    # nothing forbidden, man coverage is what actually
                    # follows a robot that has not committed yet. Block
                    # goalkeeps a region, which on a field whose only
                    # scoring region is a wide arc means standing in the
                    # middle of it and being driven around.
                    tactic=Defend(mode="shadow", deny="any"),
                    priority=10,
                ),
            ],
            fallback=Idle(),
        )
    # Same threshold our own robot uses. A different one would make every
    # contested piece a statement about the difference.
    return cycle_fuel(shoot_at=8)
