"""Step 4, second half: present the live world in the shape tactics read.

`world_state.py` answers "what is on the field". This turns that answer
into the two objects the strategy layer already knows how to read -- a
`match` and a `Robot` -- so `Collect`, `Score`, `NavigateTo` and every
trigger built on `world_view` run against real robot code without being
told they are.

**A view, not a second Match.** `MapleMatchView` duck-types the contract
rather than subclassing `common_sim.match.Match`, and the reason is the
same one that decided step 4's first half: a real `Match` brings its own
pymunk world, its own scoring and its own piece bookkeeping, all of which
would drift from the simulation that actually decides what happens. Two
models that disagree report failures that are really disagreements. So
everything here either reads through to NetworkTables or is honestly
absent.

The contract, surveyed by grepping every `match.` and `robot.` access in
`common_sim/control/`, is 14 members on the match and 12 on the robot,
of which exactly three are writes. Each is answered below with a note on
what it means once "the match" is a running JVM.

`MapleRobot` *does* subclass `Robot`, for the opposite reason: its
geometry helpers -- `footprint`, `side_reach_points`, `side_engages_polygon`,
`is_full_for`, `accepts` -- are pure functions of a pose and a set of
characteristics, with no simulation state behind them. Reimplementing
them would be copying, and a copy of a geometry routine is a copy that
disagrees at the edges. Its pymunk body is never stepped; it is a place
to write the pose that arrives over the wire.

Units change here, once, and only here: NetworkTables is metres and
radians, sparky-sim is inches and radians.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import pymunk

from bridge import arena
from bridge import drive_model as dm
from bridge import operator as op
from bridge import world_state as ws
from common_sim.field.game_piece import GamePiece, piece_spec
from common_sim.geometry import Pose2d
from common_sim.field.field_config import point_in_polygon
from common_sim.match.events import EventLog
from common_sim.match.match import Phase
from common_sim.robot.characteristics import RobotCharacteristics, SideManipulators
from common_sim.robot.robot import Robot

M_TO_IN = arena.M_TO_IN
IN_TO_M = 1.0 / M_TO_IN

#: The one scoring action REBUILT offers a robot: put fuel in the HUB.
SHOOT = "shoot"


# -- the robot's characteristics ----------------------------------------

# maple-sim's IntakeSimulation.getIntakeRectangle for an over-the-bumper
# FRONT intake places the collecting fixture from `bumperLengthX/2` out to
# `bumperLengthX/2 + lengthExtended - 0.01`. With 30-inch bumpers and a
# 12-inch extension that is 14.6 to 26.6 inches from the chassis centre,
# so it reaches about 11.6 inches past the bumper.
#
# This number is load-bearing and not cosmetic. `Collect._piece_aim`
# parks the robot so the target sits *mid-wedge* -- at
# `half_length + intake_range/2` from the piece -- so getting the reach
# wrong parks the robot somewhere the intake cannot get to, and the
# symptom is a robot that drives beautifully to a piece and never picks
# it up. That exact failure cost two runs before the fixture geometry got
# read properly; see bridge/README.md.
INTAKE_REACH_IN = 11.6

BUMPER_IN = 30.0

#: IntakeIOSim: `IntakeSimulation.OverTheBumperIntake(..., 40)`.
INTAKE_CAPACITY = 40


def robot_characteristics(limits: dm.DriveLimits) -> RobotCharacteristics:
    """What the real robot can do, in sparky-sim's terms.

    The speeds come from `limits`, which is *measured* off the running
    robot rather than transcribed -- see `drive_model.calibrate`.

    Scoring is declared on the front side and that is a polite fiction:
    the robot has a turret and shoots from wherever it happens to be
    facing. It is here because `world_view._robot_can_score` needs some
    side to say yes, and because `deposit_region_for` is overridden below
    to test position rather than which way a bumper points. If a future
    tactic starts reasoning about the *scoring side* specifically, this
    is the line that will mislead it.
    """
    return RobotCharacteristics(
        name="TyRapXXVI",
        max_speed=limits.max_speed_mps * M_TO_IN,
        max_angular_speed=limits.max_omega_rad_s,
        # Acceleration is not published and not measured here. These are
        # the sim's defaults, and they only affect sparky-sim's own
        # velocity ramp -- which this adapter bypasses, because the real
        # drivetrain does its own ramping. Nothing downstream reads them.
        max_accel=250.0,
        max_angular_accel=20.0,
        width=BUMPER_IN,
        length=BUMPER_IN,
        piece_capacity=INTAKE_CAPACITY,
        intake_range=INTAKE_REACH_IN,
        accepted_piece_types=frozenset({arena.PIECE_TYPE}),
        side_manipulators={
            "front": SideManipulators(
                intake_piece_types=frozenset({arena.PIECE_TYPE}),
                score_piece_types=frozenset({arena.PIECE_TYPE}),
                # "field" and not "both": REBUILT has no station a robot
                # loads from on demand. Everything, including what the
                # OUTPOSTs throw back in, is loose on the floor.
                intake_source="field",
            ),
        },
    )


# -- scoring ------------------------------------------------------------


class ShootOnly:
    """`match.scoring_rules`, reduced to the one thing it is asked.

    Only `points_for(action, phase)` is ever called from the control
    layer, by `utility.score_options` and one trigger. REBUILT's real
    point values live in maple-sim and come back as `TotalScore`; there
    is deliberately no copy of them here.

    The constant is not a guess that might be wrong -- with a single
    scoring action the value cancels out of every comparison
    `utility.py` makes, so the ranking is by travel and deposit time,
    which is the ordering wanted anyway. The moment REBUILT grows a
    second action worth different points, this stops being harmless and
    has to be measured.
    """

    def points_for(self, action: str, phase: str) -> float:
        return 1.0 if action == SHOOT else 0.0


class PieceTracker:
    """Keeps one `GamePiece` object attached to one physical piece of fuel.

    Needed because **the fuel array's order is not stable**.
    `SimulatedArena.gamePieces` is a `HashSet`, so iteration order can
    change whenever anything is added or removed, and
    `getGamePiecesPosesByType` walks it directly. Handing out pooled
    pieces by array index therefore gives a caller an object whose
    coordinates jump to some other piece, somewhere else on the field,
    between one tick and the next.

    That is not a cosmetic problem. `Collect` holds a reference to its
    target across replans; if the target teleports, the robot chases a
    ghost at full stick and never arrives, which is what it does. The
    first live run of the strategy layer drove three metres, wedged
    against the red HUB, and commanded maximum speed at it for the
    remaining forty seconds.

    Matching is greedy nearest-neighbour through a coarse spatial hash,
    with each previous piece consumable once. Fuel in a REBUILT grid sits
    about six inches apart, so two adjacent pieces *can* swap identities
    when one is disturbed -- harmless, since they are six inches apart
    and every tactic replans anyway. What this rules out is identity
    jumping across the field, which is the failure that matters.
    """

    #: Spatial-hash cell, in inches. Comfortably larger than the match
    #: radius so a candidate is never more than one cell away.
    CELL_IN = 24.0

    #: How far a piece may move in one tick and still be recognised. At
    #: 20 Hz a piece shoved by a robot at full speed travels about nine
    #: inches, so this has margin -- but it is deliberately not huge: a
    #: generous radius buys nothing and makes neighbour swaps likelier.
    MATCH_RADIUS_IN = 12.0

    def __init__(self, space: pymunk.Space):
        self._space = space
        self._live: list[GamePiece] = []

    def update(self, positions: list[tuple[float, float]]) -> list[GamePiece]:
        buckets: dict[tuple[int, int], list[GamePiece]] = {}
        for piece in self._live:
            buckets.setdefault(self._cell(piece.body.position), []).append(piece)

        taken: set[int] = set()
        matched = [self._take_nearest(buckets, taken, position) for position in positions]

        # Anything left over was collected, scored, or shot away. Retired
        # rather than pooled for reuse, which is the tempting thing to do
        # and would reintroduce the exact bug this class exists to fix: a
        # tactic holding a reference to the piece it just collected would
        # find that object silently reissued for a *new* piece somewhere
        # else on the field. Retirement costs one allocation per piece
        # that leaves the field -- a few hundred over a whole match, not
        # a few hundred per tick, which is the cost that mattered.
        #
        # `scored` is what makes a stale reference inert rather than
        # merely wrong: `world_view.collectable_pieces` filters on it, so
        # a tactic still holding a retired piece stops seeing it as an
        # option instead of driving to where it used to be.
        for piece in self._live:
            if id(piece) not in taken:
                piece.scored = True
                piece.remove_from_space()

        result = []
        for position, piece in zip(positions, matched):
            if piece is None:
                piece = self._new()
            piece.body.position = position
            result.append(piece)
        self._live = result
        return result

    def _cell(self, position) -> tuple[int, int]:
        return (int(position[0] // self.CELL_IN), int(position[1] // self.CELL_IN))

    def _take_nearest(self, buckets, taken: set[int], position) -> GamePiece | None:
        cx, cy = self._cell(position)
        best, best_distance = None, self.MATCH_RADIUS_IN ** 2
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for piece in buckets.get((cx + dx, cy + dy), ()):
                    if id(piece) in taken:
                        continue
                    px, py = piece.body.position
                    distance = (px - position[0]) ** 2 + (py - position[1]) ** 2
                    if distance < best_distance:
                        best, best_distance = piece, distance
        if best is not None:
            taken.add(id(best))
        return best

    def _new(self) -> GamePiece:
        spec = piece_spec(arena.PIECE_TYPE)
        return GamePiece(
            self._space, arena.PIECE_TYPE, (0.0, 0.0),
            radius=spec.radius, mass=spec.mass, color=spec.color, source="field",
        )


@dataclass(frozen=True)
class ViewConfig:
    """`match.config`, which is read for exactly one thing: how long the
    match lasts, so a trigger can ask how much is left."""

    total_duration: float = 150.0


# -- the robot ----------------------------------------------------------


class MapleRobot(Robot):
    """A `Robot` whose pose arrives over NT and whose commands leave over
    the HALSim link.

    The three writes the strategy layer makes -- `drive_field_relative`,
    `set_intake_active`, `set_deposit_active` -- are overridden to press
    buttons and push sticks. Everything else is inherited, because
    everything else is geometry.
    """

    def __init__(
        self,
        link: op.OperatorLink,
        limits: dm.DriveLimits,
        *,
        alliance: str = "blue",
        characteristics: RobotCharacteristics | None = None,
    ):
        # A space of its own that is never stepped. `Robot.__init__`
        # wants somewhere to put a chassis body and its intake sensor
        # shapes; nothing here integrates them, and `sync` overwrites the
        # body's state from the wire every tick. Sharing the strategy
        # sim's space would be worse than useless -- it would invite
        # something to step it.
        self._space = pymunk.Space()
        self.limits = limits
        self.link = link
        super().__init__(
            self._space,
            characteristics or robot_characteristics(limits),
            Pose2d(0.0, 0.0, 0.0),
            alliance=alliance,
        )

        self._piece_pool: list[GamePiece] = []
        self._deposit_commanded = False
        self._deposit_want = False
        self._deposit_want_since = 0.0
        self.saturated = False  # last drive command exceeded the drivetrain

        # Intake command state. `_want` is what the tactic asked for,
        # `_commanded` is what the buttons currently say; they differ
        # while a too-brief "off" is being debounced away.
        self._intake_want: bool | None = None
        self._intake_want_since = 0.0
        self._intake_commanded: bool | None = None
        self._intake_changed_at = 0.0
        self._reasserting = False
        self._now = 0.0
        #: How many times a lost button edge had to be re-issued. Zero on
        #: a healthy run; a rising count means the transport is dropping
        #: edges faster than expected and is worth reporting rather than
        #: silently compensating for.
        self.intake_reasserts = 0

        #: Whether the turret should track the HUB by itself. True is
        #: what a driver sets at the start of a match and leaves.
        self.want_auto_aim = True
        self._aim_pressed_at: float | None = None
        #: How many times the auto-aim toggle had to be pressed. One on a
        #: healthy run -- more means something else is turning it off.
        self.aim_toggles = 0

    @property
    def intake_active(self) -> bool:
        """Whether a tactic currently wants the intake, and never None.

        The mirror of the inherited `deposit_active`, which `Robot`
        provides and this does not -- readable so a report can separate
        "the tactic never asked for the intake" from "it asked and the
        mechanism did not move", which are failures in different repos.
        """
        return bool(self._intake_want)

    # -- reads: the wire fills these in ---------------------------------

    def sync(
        self,
        world: ws.WorldState,
        measured_robot_relative: tuple[float, float, float] | None,
        now: float = 0.0,
    ) -> None:
        """Copy one `WorldState` into the inherited `Robot` state.

        Writes straight into the pymunk body rather than through any
        `drive_*` call: the real drivetrain has already done the
        integrating, and running a second integrator over the answer
        would produce a pose that lags the robot by however long the
        strategy loop takes.
        """
        if world.robot is not None:
            self.chassis.body.position = (world.robot.x * M_TO_IN, world.robot.y * M_TO_IN)
            self.chassis.body.angle = world.robot.theta

        if measured_robot_relative is not None:
            # `SwerveChassisSpeeds/Measured` is robot-relative (it comes
            # from the kinematics' view of the module states), while
            # `Robot.speed` and everything reading it mean field-relative.
            vx, vy, omega = measured_robot_relative
            heading = self.chassis.body.angle
            self.chassis.body.velocity = (
                (vx * math.cos(heading) - vy * math.sin(heading)) * M_TO_IN,
                (vx * math.sin(heading) + vy * math.cos(heading)) * M_TO_IN,
            )
            self.chassis.body.angular_velocity = omega

        self._now = now
        self._sync_held(world.held)
        self._reconcile_intake(world.intake_arm_deg)
        self._reconcile_aim(world.auto_aim)
        # Runs every tick, not only on a command change, so the feed
        # debounce can expire without waiting for the next call.
        self._apply_deposit()

    def _sync_held(self, count: int) -> None:
        """Make `held_pieces` as long as the ball count says it is.

        **A hopper is a queue, and the order is load-bearing.** Fuel
        collected goes on the back; fuel shot leaves from the front.
        Which physical ball leaves is unknowable and would not matter --
        they are interchangeable -- except that
        `behavior.RunManipulator` names `held_pieces[0]` as the piece it
        is depositing and returns SUCCESS when that object is no longer
        held. Truncating the list from the tail, which is the obvious
        thing to do, means the named piece never leaves: `Score` runs
        until its timeout instead of finishing, every single time, and
        nothing about that looks like a bug in the adapter.

        Pieces are otherwise stable across ticks, so a tactic can hold a
        reference to one for as long as the robot holds it.
        """
        current = list(self.held_pieces)
        for piece in current[:max(0, len(current) - count)]:
            # Gone -- shot, or ejected. Flagged so a stale reference
            # reads as "not held any more" rather than as a live piece.
            piece.held_by = None
        current = current[max(0, len(current) - count):]

        spec = piece_spec(arena.PIECE_TYPE)
        while len(current) < count:
            piece = GamePiece(
                self._space, arena.PIECE_TYPE, (0.0, 0.0),
                radius=spec.radius, mass=spec.mass, color=spec.color, source="field",
            )
            piece.held_by = self
            piece.last_holder_alliance = self.alliance
            current.append(piece)
        self.held_pieces = current

    # -- writes: these leave over the HALSim link ------------------------

    def drive_field_relative(self, dt: float, vx: float, vy: float, omega: float) -> None:
        """Field-relative velocity in inches/s and rad/s, onto the sticks.

        Also passed to the inherited implementation, which updates
        `commanded_velocity` without touching the body. That keeps
        `commanded_speed` honest, and `commanded_speed` is what the
        liveness oracle and several tactics use to tell a robot that is
        waiting from one that is being held -- so leaving it at zero
        would quietly disarm the stall detection this project spent a
        campaign getting right.
        """
        super().drive_field_relative(dt, vx, vy, omega)
        request = dm.axes_for(
            vx * IN_TO_M, vy * IN_TO_M, omega, self.limits, shooting=self._deposit_commanded
        )
        self.saturated = request.saturated
        request.apply(self.link)

    def set_intake_active(self, active: bool) -> None:
        """Record what the tactic wants. The buttons are pressed in `sync`.

        Deliberately not written straight through, for two reasons that
        both showed up live.

        **The command chatters.** `behavior.RunIntake` sets the intake
        off the instant a piece is captured and on again on the next
        tick, so a robot collecting a stream of fuel toggles it once per
        ball. A driver does not stow the intake between pieces, and this
        one physically cannot: the arm takes about a tenth of a second
        each way, so obeying every toggle means an arm that is somewhere
        in the middle whenever it matters.

        **Edges can be swallowed.** Manip Y and B are bound `onTrue`, so
        the mapping depends on the robot *seeing* a transition -- but the
        operator link transmits at 50 Hz, and a press and release inside
        one 20 ms window is never on the wire at all. The result is a
        command that reads as active while the mechanism sits stowed,
        which looks exactly like a broken intake mapping.

        So the desired state is recorded here and reconciled against the
        arm's *observed* angle in `_reconcile_intake`.
        """
        super().set_intake_active(active)
        if active != self._intake_want:
            self._intake_want = active
            self._intake_want_since = self._now

    #: How long the intake must be commanded off before it is actually
    #: stowed. Longer than the gap between two pieces in a collect loop
    #: and far shorter than any real decision to stop intaking.
    STOW_DEBOUNCE = 0.5

    #: How long the arm may disagree with the command before the edge is
    #: assumed lost and re-issued.
    REASSERT_AFTER = 0.6

    #: Arm angles: `IntakeConstants.retractedAngle` is 90 degrees and
    #: `extendedAngle` is 0, and `IntakeIOSim` counts anything within 5
    #: degrees of extended as deployed.
    DEPLOYED_BELOW_DEG = 20.0

    def _reconcile_intake(self, arm_deg: float) -> None:
        """Make the buttons agree with what the tactic asked for, using
        the arm as the source of truth rather than the last thing sent.

        This is a controller, not a translator, and it is a controller
        because the thing it drives is edge-triggered over a lossy-by-
        design transport. `set_intake_active` explains why.
        """
        if self._intake_want is None:
            # Nobody has asked for anything yet. A robot that has never
            # been told either way should not be issuing retract commands.
            return

        want = self._intake_want
        # The debounce applies only to *stopping* an intake that is
        # already running -- "do not stow between two pieces". Applying
        # it to the initial state instead would deploy the intake before
        # any tactic had asked for it.
        if not want and self._intake_commanded and self._now - self._intake_want_since < self.STOW_DEBOUNCE:
            want = True

        if self._reasserting:
            # One tick with Y released, so the next press is a genuine
            # rising edge rather than a no-op on an already-held button.
            self._reasserting = False
            self._press_intake(want)
            return

        if want != self._intake_commanded:
            self._press_intake(want)
            return

        deployed = arm_deg <= self.DEPLOYED_BELOW_DEG
        if want != deployed and self._now - self._intake_changed_at > self.REASSERT_AFTER:
            self.link.set_button(op.BTN_Y, False, joystick=1)
            self.link.set_button(op.BTN_B, False, joystick=1)
            self._reasserting = True
            self.intake_reasserts += 1

    def _press_intake(self, active: bool) -> None:
        self._intake_commanded = active
        self._intake_changed_at = self._now
        self.link.set_button(op.BTN_Y, active, joystick=1)
        self.link.set_button(op.BTN_B, not active, joystick=1)
        # Drive right trigger is `intake.takeInCommand` on rising edge and
        # `stopMotorCommand` on falling, so this one really is a hold.
        self.link.set_axis(op.AXIS_RIGHT_TRIGGER, 1.0 if active else 0.0, joystick=0)

    def _reconcile_aim(self, enabled: bool) -> None:
        """Keep the turret's auto-aim on.

        A driver turns this on once and leaves it; without it the turret
        points wherever it was left and every shot lands on the floor --
        which is what the first working strategy runs did. Fuel left the
        robot, the loose count went up by the same amount, and the score
        stayed at zero.

        Manip Start is a *toggle* (`turret.toggleAutoAimCommand`), so
        pressing it blind is as likely to turn auto-aim off as on. The
        robot publishes `autoAimEnabled`, so the press is conditioned on
        the published state -- the same shape as the intake
        reconciliation above, and for the same reason: drive an
        edge-triggered mechanism from what it says it is doing, not from
        what was last sent to it.
        """
        if enabled or not self.want_auto_aim:
            self.link.set_button(op.BTN_START, False, joystick=1)
            self._aim_pressed_at = None
            return
        if self._aim_pressed_at is not None and self._now - self._aim_pressed_at < self.AIM_TOGGLE_SETTLE:
            return  # already asked; give the command a chance to run
        self.link.set_button(op.BTN_START, True, joystick=1)
        self._aim_pressed_at = self._now
        self.aim_toggles += 1

    #: How long to wait after pressing the auto-aim toggle before
    #: concluding it did not take. Long enough for the command to be
    #: scheduled and the output to be published back.
    AIM_TOGGLE_SETTLE = 1.0

    #: How long the feeder is held on after the deposit command drops.
    #: `Score` runs one `RunManipulator` per piece and cycles the command
    #: between them, so without this the feeder is switched off and on
    #: once per shot -- and the manip right trigger is `onTrue`/`onFalse`,
    #: so a cycle inside one 50 Hz transmit window never reaches the wire
    #: and the feeder simply stays off. Observed live: a burst that
    #: emptied twenty balls and then sat with the deposit commanded and
    #: nothing leaving for fourteen seconds.
    FEED_DEBOUNCE = 0.5

    def set_deposit_active(self, active: bool, action: str | None = None) -> None:
        """Spin the flywheel and run the feeder.

        Left bumper on the drive controller is `flywheel.shootCommand`,
        bound `whileTrue`, so it is held for as long as the shot lasts.
        The manip right trigger turns the feeder on, which is what
        actually sends fuel out -- and which also halves the drivetrain,
        because `joystickDrive` takes `spindexer::isFeederOn` as its
        speed multiplier. That is why the flag is recorded rather than
        just written: the next `drive_field_relative` has to invert a
        different mapping.

        Debounced like the intake, and for the same reason. Unlike the
        intake there is **no observable to close the loop on** -- nothing
        publishes the feeder's state, so a lost edge here cannot be
        detected, only avoided. If the robot code ever logs
        `spindexer.isFeederOn`, this should become a reconciliation like
        `_reconcile_intake`.
        """
        super().set_deposit_active(active, action)
        if active != self._deposit_want:
            self._deposit_want = active
            self._deposit_want_since = self._now
        self._apply_deposit()

    def _apply_deposit(self) -> None:
        want = self._deposit_want
        if not want and self._deposit_commanded and self._now - self._deposit_want_since < self.FEED_DEBOUNCE:
            want = True  # between two shots of the same burst
        if want == self._deposit_commanded:
            return
        self._deposit_commanded = want
        self.link.set_button(op.BTN_LEFT_BUMPER, want, joystick=0)
        self.link.set_axis(op.AXIS_RIGHT_TRIGGER, 1.0 if want else 0.0, joystick=1)

    def release_all(self) -> None:
        """Let go of everything. For the end of a match, or a handover
        back to the scripted operator.

        Bypasses the intake debounce on purpose: this is not a tactic
        changing its mind, it is the run ending, and nothing should be
        left held afterwards.
        """
        self.set_intake_active(False)
        self._press_intake(False)
        self.set_deposit_active(False)
        self._deposit_want_since = -math.inf  # skip the feed debounce; the run is over
        self._apply_deposit()
        self.link.set_button(op.BTN_B, False, joystick=1)
        self.want_auto_aim = False
        self.link.set_button(op.BTN_START, False, joystick=1)
        self.link.set_axis(op.AXIS_LEFT_X, 0.0)
        self.link.set_axis(op.AXIS_LEFT_Y, 0.0)
        self.link.set_axis(op.AXIS_RIGHT_X, 0.0)


# -- the match ----------------------------------------------------------


class MapleMatchView:
    """The read-only `match` the strategy layer sees.

    Every member is one of three things: read through to the live
    simulation, derived from the transcribed arena, or honestly absent
    because REBUILT-under-maple-sim does not have it. Nothing is
    simulated a second time.
    """

    def __init__(self, robot: MapleRobot, reader: ws.WorldStateReader, *, config: ViewConfig | None = None):
        self.field = arena.build_arena()
        self.robot = robot
        self.reader = reader
        self.config = config or ViewConfig()
        self.scoring_rules = ShootOnly()
        self.events = EventLog()

        # No station in REBUILT dispenses on demand, so `station_options`
        # comes back empty and no tactic ever asks. Present because
        # `world_view.station_has_supply` reads it unconditionally.
        self.station_supply: dict = {}

        self.phase = Phase.AUTO
        self.elapsed = 0.0
        self.world = ws.WorldState(
            robot=None, fuel=(), held=0,
            match_clock=0.0, phase_clock=0.0, hub_active={}, score={},
        )
        self._space = pymunk.Space()
        self._tracker = PieceTracker(self._space)
        self.active_pieces: list[GamePiece] = []

    # -- the tick -------------------------------------------------------

    def sync(self, elapsed: float, phase: Phase) -> ws.WorldState:
        """Pull one world, and make every derived view agree with it."""
        self.elapsed = elapsed
        self.phase = phase
        self.world = self.reader.read()
        self._sync_pieces(self.world)
        self.robot.sync(self.world, self.reader.measured_chassis_speeds(), now=elapsed)
        return self.world

    def _sync_pieces(self, world: ws.WorldState) -> None:
        """Mirror the loose fuel into `active_pieces`.

        Through `PieceTracker`, which matches by position rather than by
        array index -- see its docstring for why an index is not an
        identity here.
        """
        self.active_pieces = self._tracker.update(
            [(pose.x * M_TO_IN, pose.y * M_TO_IN) for pose in world.fuel]
        )

    # -- the duck-typed contract ----------------------------------------

    @property
    def robots(self) -> list[MapleRobot]:
        """Just ours, for now.

        maple-sim can hold more drivetrains, and step 6 puts sparky-sim
        opponents in them. Until then every `world_view` query about
        opponents, partners or defenders correctly returns nothing, and
        the tactics that read them do nothing -- which is the right
        answer for a field with one robot on it, not a stub.
        """
        return [self.robot]

    def region_full(self, region, action: str) -> bool:
        """Never. The HUB takes fuel until the match ends; `TotalFuelInHub`
        has no ceiling."""
        return False

    def region_blocked(self, region, action: str) -> bool:
        """True while this HUB is not the one accepting.

        This is the 25-second clock, and it is the most REBUILT-specific
        behaviour the bridge can express. `Arena2026Rebuilt` makes
        exactly one alliance's HUB active at a time outside autonomous
        and swaps every 25 seconds; fuel shot into the other one comes
        back as `WastedFuel`.

        Routed through `region_blocked` rather than `region_full` on
        purpose, and the distinction matters to a caller:
        `world_view.scoring_slots_for_type` drops a blocked slot from the
        options, so a robot holding fuel with no live HUB stops
        presenting a scoring option and its strategy falls through to
        whatever it does otherwise -- collect more, reposition. "Full"
        would say the same thing and mean something permanent.

        The docstring on `world_view.scoring_slots_for_type` describes
        this hook as "obstructed by an uncollected piece", which is
        REEFSCAPE's use of it. The general contract is "this (region,
        action) is not available right now", which is exactly this.
        """
        alliance = getattr(region, "alliance", None)
        if alliance is None:
            return False
        return not self.world.hub_active.get(alliance, True)

    def deposit_region_for(self, robot, piece=None, action: str | None = None):
        """Where a shot taken right now would land, or None.

        Deliberately *not* `Match.deposit_region_for`, which asks whether
        a bumper side is engaging the region polygon. That is the right
        question for a robot that reaches into a structure and the wrong
        one for a robot with a turret: this one shoots from wherever it
        is standing, at whatever the turret is pointed at, so position is
        the whole test.

        Still gated on the HUB being live, so a robot cannot read itself
        as ready to score into a HUB that is not accepting.
        """
        if action is None:
            action = robot.deposit_action
        if action is None or not robot.held_pieces:
            return None
        for region in self.field.scoring_regions:
            if region.alliance is not None and region.alliance != robot.alliance:
                continue
            if action not in region.actions:
                continue
            if self.region_blocked(region, action):
                continue
            if point_in_polygon((robot.pose.x, robot.pose.y), region.vertices):
                return region
        return None

    def protecting_zone(self, robot):
        """None, always. maple-sim adjudicates no fouls, so there is
        nowhere an opponent may not follow -- and telling the strategy
        layer otherwise would have it rely on protection that does not
        exist in the simulation it is being tested in."""
        return None

    def pin_seconds(self, offender, victim) -> float:
        """Zero. Present for completeness only: `world_view.pin_pressure`
        checks `field.pin_rule` first and this field has none, so nothing
        reaches here."""
        return 0.0
