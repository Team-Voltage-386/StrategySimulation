"""Tests for the drive model and the strategy-layer adapter.

Two things are being guarded, and they fail differently.

The **drive model** is an inverse. `DriveCommands.joystickDrive` deadbands
the stick, squares it and halves the drivetrain while the feeder runs, and
`axes_for` undoes all three. An inverse either round-trips or it does not,
so these tests are exact rather than approximate -- and the round-trip is
the whole test, because a model that is merely close produces a robot that
drives at nearly the right speed in nearly the right direction and arrives
somewhere the navigator did not plan for. That reads as a navigation bug,
which is the wrong place to go looking.

The **adapter** is checked against a stub reader rather than a JVM. Every
member of the duck-typed contract is exercised, because the way this
breaks in practice is not a wrong answer -- it is a tactic reaching for a
member nobody thought to provide, which surfaces as an AttributeError
several hours into an overnight campaign.

`bridge.match_view` needs pymunk (it subclasses `Robot`) but not pyntcore
or a robot project, which is what lets all of this run in CI.
"""
from __future__ import annotations

import math

import pytest

from bridge import arena
from bridge import drive_model as dm
from bridge import match_view as mv
from bridge import operator as op
from bridge import world_state as ws
from bridge.robot_state import Pose2d, Pose3d
from common_sim.control.behavior import BehaviorContext
from common_sim.control.strategy import StrategyController
from common_sim.control.behavior import Status
from common_sim.control.tactics import Collect, Pass, Score
from common_sim.field.field_config import point_in_polygon
from common_sim.control.world_view import scoring_slots_for_type
from common_sim.match.match import Phase

LIMITS = dm.DriveLimits(max_speed_mps=4.0, max_omega_rad_s=8.0)


class FakeLink:
    """Records what would have gone out over the HALSim WebSocket."""

    def __init__(self):
        self.axes: dict[tuple[int, int], float] = {}
        self.buttons: dict[tuple[int, int], bool] = {}

    def set_axis(self, axis: int, value: float, *, joystick: int = 0) -> None:
        self.axes[(joystick, axis)] = value

    def set_button(self, button: int, pressed: bool, *, joystick: int = 0) -> None:
        self.buttons[(joystick, button)] = pressed


class FakeReader:
    """A `WorldStateReader` that answers from memory."""

    def __init__(self, *, held: int = 0, fuel=(), hub_active=None, robot=None):
        self.held = held
        self.fuel = tuple(fuel)
        self.hub_active = hub_active or {"blue": True, "red": False}
        self.robot = robot if robot is not None else Pose2d(8.0, 2.0, 0.0)
        self.link = None
        self.speeds = (0.0, 0.0, 0.0)
        self.arm = 90.0  # stowed; IntakeConstants.retractedAngle
        self.auto_aim = False
        self.feeder_on = False

    def read(self) -> ws.WorldState:
        return ws.WorldState(
            robot=self.robot, fuel=self.fuel, held=self.held,
            match_clock=10.0, phase_clock=20.0,
            hub_active=dict(self.hub_active), score={"blue": 0.0, "red": 0.0},
            intake_arm_deg=self.arm, auto_aim=self.auto_aim,
            feeder_on=self.feeder_on,
        )

    def measured_chassis_speeds(self):
        return self.speeds


def _view(**reader_kwargs):
    link = FakeLink()
    robot = mv.MapleRobot(link, LIMITS)
    reader = FakeReader(**reader_kwargs)
    view = mv.MapleMatchView(robot, reader)
    view.sync(0.0, Phase.TELEOP)
    return link, robot, view, reader


# ---------------------------------------------------------------------------
# the drive model
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("vx,vy,omega", [
    (1.0, 0.0, 0.0),
    (0.0, -2.0, 0.0),
    (1.5, 1.5, 0.4),
    (0.0, 0.0, -3.0),
    (0.05, 0.0, 0.0),  # slower than the deadband would pass unscaled
    (-1.2, 0.7, -1.1),
])
def test_the_joystick_model_round_trips_exactly(vx, vy, omega):
    request = dm.axes_for(vx, vy, omega, LIMITS)
    got = dm.forward(request.axes, limits=LIMITS)
    assert got == pytest.approx((vx, vy, omega), abs=1e-9)
    assert not request.saturated


def test_the_deadband_is_undone_not_merely_offset():
    """`MathUtil.applyDeadband` rescales the surviving range back out to
    [0, 1]. An inverse that only re-adds the offset is wrong everywhere
    except at full stick, which is exactly where a casual check would
    look."""
    for fraction in (0.1, 0.5, 0.9, 1.0):
        assert dm.apply_deadband(dm.undo_deadband(fraction)) == pytest.approx(fraction)


def test_a_request_to_stop_is_a_real_stop():
    """Zero must map to zero, not to the deadband edge -- otherwise the
    robot creeps whenever the strategy layer asks it to hold still, and
    a creeping robot is a robot the stall detector never fires on."""
    request = dm.axes_for(0.0, 0.0, 0.0, LIMITS)
    assert all(value == 0.0 for value in request.axes.values())


def test_squaring_is_modelled_and_not_just_a_scale_factor():
    """Half stick is a quarter speed, not half. A model that got this
    wrong would still look right at full stick."""
    half = dm.forward({op.AXIS_LEFT_Y: -dm.undo_deadband(0.5)})
    assert half[0] == pytest.approx(0.25)


def test_the_binding_layer_is_part_of_the_model():
    """`RobotContainer` passes `-getLeftY()` as the forward axis and
    `-getLeftX()` as the leftward one. A sign error here sends the robot
    the wrong way while every unit test on velocities still passes."""
    forward_request = dm.axes_for(1.0, 0.0, 0.0, LIMITS)
    assert forward_request.axes[op.AXIS_LEFT_Y] < 0
    left_request = dm.axes_for(0.0, 1.0, 0.0, LIMITS)
    assert left_request.axes[op.AXIS_LEFT_X] < 0


def test_shooting_halves_the_drivetrain_and_the_inverse_knows():
    """`joystickDrive` takes `spindexer::isFeederOn` as a speed
    multiplier, so the same stick means different velocities depending on
    whether a shot is being fed. Asking for 2 m/s while shooting has to
    push the stick further."""
    normal = dm.axes_for(2.0, 0.0, 0.0, LIMITS)
    shooting = dm.axes_for(2.0, 0.0, 0.0, LIMITS, shooting=True)
    assert abs(shooting.axes[op.AXIS_LEFT_Y]) > abs(normal.axes[op.AXIS_LEFT_Y])
    assert dm.forward(shooting.axes, shooting=True, limits=LIMITS)[0] == pytest.approx(2.0)


def test_an_impossible_request_keeps_its_direction_and_says_so():
    """Clipping the heading as well would be the worse failure: a robot
    that cannot go as fast as asked still goes the right way, whereas one
    whose direction quietly bends looks like a navigation bug."""
    request = dm.axes_for(100.0, 100.0, 0.0, LIMITS)
    assert request.saturated
    vx, vy, _ = dm.forward(request.axes, limits=LIMITS)
    assert math.atan2(vy, vx) == pytest.approx(math.radians(45))
    assert math.hypot(vx, vy) == pytest.approx(LIMITS.max_speed_mps)


# ---------------------------------------------------------------------------
# the adapter's reads
# ---------------------------------------------------------------------------


def test_the_pose_arrives_in_inches():
    """The one unit boundary in the bridge. Getting it wrong by 39x
    would put the robot off the field rather than in the wrong place,
    which at least fails loudly -- but only if something checks."""
    _, robot, _, _ = _view(robot=Pose2d(8.0, 2.0, 0.5))
    assert robot.pose.x == pytest.approx(8.0 * arena.M_TO_IN)
    assert robot.pose.y == pytest.approx(2.0 * arena.M_TO_IN)
    assert robot.pose.heading == pytest.approx(0.5)


def test_measured_speed_is_rotated_into_the_field_frame():
    """`SwerveChassisSpeeds/Measured` is robot-relative and `Robot.speed`
    means field-relative. A robot facing +y driving 'forward' at 1 m/s is
    moving in +y, and anything reading `speed` to decide whether the
    robot is stuck has to see that."""
    link = FakeLink()
    robot = mv.MapleRobot(link, LIMITS)
    reader = FakeReader(robot=Pose2d(8.0, 2.0, math.pi / 2))
    reader.speeds = (1.0, 0.0, 0.0)
    view = mv.MapleMatchView(robot, reader)
    view.sync(0.0, Phase.TELEOP)

    vx, vy = robot.chassis.body.velocity
    assert vx == pytest.approx(0.0, abs=1e-6)
    assert vy == pytest.approx(1.0 * arena.M_TO_IN)


def test_possession_comes_from_the_ball_count():
    _, robot, view, reader = _view(held=3)
    assert len(robot.held_pieces) == 3
    assert all(p.piece_type == arena.PIECE_TYPE for p in robot.held_pieces)
    assert all(p.held_by is robot for p in robot.held_pieces)

    reader.held = 1
    view.sync(0.1, Phase.TELEOP)
    assert len(robot.held_pieces) == 1


def test_the_hopper_empties_from_the_front():
    """`behavior.RunManipulator` names `held_pieces[0]` as the piece it
    is depositing and returns SUCCESS when that object is no longer held.
    Truncating the list from the tail -- the obvious implementation -- means
    the named piece never leaves, so `Score` runs to its timeout every
    single time instead of finishing, and nothing about that looks like an
    adapter bug.

    Fuel is interchangeable, so which ball leaves is unknowable; a queue
    is both the faithful model of a hopper and the one the tactic needs.
    """
    _, robot, view, reader = _view(held=3)
    first, second, third = robot.held_pieces

    reader.held = 1
    view.sync(0.1, Phase.TELEOP)

    assert robot.held_pieces == [third], "the oldest fuel should have been shot"
    assert first not in robot.held_pieces
    assert first.held_by is None, "a piece that left must not still read as held"


def test_held_pieces_are_pooled_across_ticks():
    """A couple of hundred pymunk bodies rebuilt at loop rate would cost
    more than the planning does, and nothing reads a held piece's
    identity -- fuel is interchangeable."""
    _, robot, view, reader = _view(held=2)
    first = list(robot.held_pieces)
    reader.held = 4
    view.sync(0.1, Phase.TELEOP)
    assert robot.held_pieces[:2] == first


def test_outpost_render_poses_are_not_collectable_fuel():
    """`Arena2026Rebuilt.getGamePiecesPosesByType` appends 24 Pose3ds per
    OUTPOST that are a *drawing* of what a human player is holding,
    stacked outside the field walls. They look exactly like collectable
    fuel to anything reading the array, and a `Collect` tactic that picks
    one drives at a spot behind the alliance wall and leans on the wall
    for the rest of the match.

    The real numbers: 192 poses on a field carrying 144 pieces.
    """
    field_piece = Pose3d(8.0, 4.0, 0.075)
    blue_render = Pose3d(-0.12, 0.325, 0.845)  # RebuiltOutpost.blueRenderPose
    red_render = Pose3d(16.64, 7.7, 0.845)  # .redRenderPose

    kept = ws.on_field([field_piece, blue_render, red_render])
    assert kept == [field_piece]


def test_loose_fuel_becomes_active_pieces_in_inches():
    _, _, view, _ = _view(fuel=[Pose3d(7.5, 2.6, 0.075), Pose3d(9.0, 3.0, 0.075)])
    assert len(view.active_pieces) == 2
    assert view.active_pieces[0].position.x == pytest.approx(7.5 * arena.M_TO_IN)


def test_a_piece_keeps_its_identity_when_the_array_order_shuffles():
    """The regression this whole tracker exists for.

    `SimulatedArena.gamePieces` is a `HashSet`, so the fuel array's order
    can change whenever anything is added or removed. Pooling by index
    hands `Collect` a target object whose coordinates jump to a different
    piece somewhere else on the field, and the robot chases a ghost at
    full stick forever. That is not hypothetical: the first live run
    drove three metres, wedged on the red HUB, and commanded maximum
    speed at it for the remaining forty seconds.
    """
    import pymunk as _pymunk

    tracker = mv.PieceTracker(_pymunk.Space())
    far_apart = [(100.0, 100.0), (400.0, 200.0), (600.0, 60.0)]
    first = tracker.update(far_apart)
    identity = {id(p): p.position for p in first}

    shuffled = [far_apart[2], far_apart[0], far_apart[1]]
    second = tracker.update(shuffled)

    for piece, position in zip(second, shuffled):
        assert identity[id(piece)] == pytest.approx((position[0], position[1])), (
            "a pooled piece was handed out for a different physical piece"
        )


def test_a_piece_that_drifts_a_little_is_still_the_same_piece():
    """Fuel gets shoved. A tracker that only matched exact positions
    would drop a target the moment the robot nudged it."""
    import pymunk as _pymunk

    tracker = mv.PieceTracker(_pymunk.Space())
    original = tracker.update([(100.0, 100.0)])[0]
    moved = tracker.update([(104.0, 103.0)])[0]
    assert moved is original


def test_a_piece_that_jumps_across_the_field_is_a_different_piece():
    """The bound that makes the previous test safe: beyond the match
    radius, nothing is claimed and a fresh identity is issued."""
    import pymunk as _pymunk

    tracker = mv.PieceTracker(_pymunk.Space())
    original = tracker.update([(100.0, 100.0)])[0]
    elsewhere = tracker.update([(500.0, 250.0)])[0]
    assert elsewhere is not original


def test_a_collected_piece_is_retired_and_never_reissued():
    """Pooling collected pieces for reuse is the tempting optimisation
    and would reintroduce the bug above one level down: a tactic holding
    the piece it just collected would find that object silently reissued
    for a new piece somewhere else.

    `scored` is what makes the stale reference inert rather than merely
    wrong -- `world_view.collectable_pieces` filters on it, so a tactic
    still holding it stops seeing an option instead of driving to where
    the piece used to be.
    """
    import pymunk as _pymunk

    tracker = mv.PieceTracker(_pymunk.Space())
    first = tracker.update([(100.0, 100.0), (400.0, 200.0)])
    collected = first[1]

    tracker.update([(100.0, 100.0)])
    assert collected.scored

    again = tracker.update([(100.0, 100.0), (600.0, 60.0)])
    assert again[0] is first[0], "the piece that stayed keeps its identity"
    assert again[1] is not collected, "a retired body must not come back as a new piece"


def test_collected_fuel_leaves_the_field():
    _, _, view, reader = _view(fuel=[Pose3d(7.5, 2.6, 0.0), Pose3d(9.0, 3.0, 0.0)])
    reader.fuel = (Pose3d(9.0, 3.0, 0.0),)
    view.sync(0.1, Phase.TELEOP)
    assert len(view.active_pieces) == 1


# ---------------------------------------------------------------------------
# the adapter's writes
# ---------------------------------------------------------------------------


def test_driving_pushes_sticks_and_keeps_the_commanded_velocity_honest():
    """`commanded_speed` has to stay truthful even though the body is
    never integrated here: it is what the liveness oracle and several
    tactics use to tell a robot that is waiting from one that is held."""
    link, robot, _, _ = _view()
    robot.drive_field_relative(0.05, 40.0, 0.0, 0.0)  # in/s

    assert link.axes[(0, op.AXIS_LEFT_Y)] < 0
    assert robot.commanded_speed == pytest.approx(40.0)


def _tick(view, robot, reader, at: float, *, arm: float | None = None,
          feeder: bool | None = None):
    """One `sync` at time `at`, optionally with the arm reading `arm` and
    the feeder reading `feeder`."""
    if arm is not None:
        reader.arm = arm
    if feeder is not None:
        reader.feeder_on = feeder
    view.sync(at, Phase.TELEOP)


def test_the_intake_mapping_produces_edges_not_a_held_button():
    """Manip Y and B are bound `onTrue`, so they need rising edges. The
    mapping is 'hold Y while intaking, hold B while not', which gives
    exactly one edge per transition and none while held."""
    link, robot, view, reader = _view()
    assert (1, op.BTN_Y) not in link.buttons, "nothing should be pressed before the first command"

    robot.set_intake_active(True)
    _tick(view, robot, reader, 1.0)
    assert link.buttons[(1, op.BTN_Y)] is True
    assert link.buttons[(1, op.BTN_B)] is False
    assert link.axes[(0, op.AXIS_RIGHT_TRIGGER)] == 1.0

    robot.set_intake_active(False)
    _tick(view, robot, reader, 2.0, arm=0.0)  # debounce has expired
    assert link.buttons[(1, op.BTN_Y)] is False
    assert link.buttons[(1, op.BTN_B)] is True
    assert link.axes[(0, op.AXIS_RIGHT_TRIGGER)] == 0.0


def test_a_momentary_stop_does_not_stow_the_intake():
    """`behavior.RunIntake` turns the intake off the instant a piece is
    captured and on again next tick, so a robot collecting a stream of
    fuel toggles it once per ball. The arm takes about a tenth of a
    second each way -- obeying every toggle means an arm that is
    somewhere in the middle whenever it matters, and a driver does not
    stow the intake between pieces anyway."""
    link, robot, view, reader = _view()
    robot.set_intake_active(True)
    _tick(view, robot, reader, 1.0, arm=0.0)

    robot.set_intake_active(False)
    _tick(view, robot, reader, 1.05, arm=0.0)
    assert link.buttons[(1, op.BTN_Y)] is True, "a 50 ms gap is not a decision to stop"

    robot.set_intake_active(True)
    _tick(view, robot, reader, 1.10, arm=0.0)
    assert link.buttons[(1, op.BTN_Y)] is True


def test_a_swallowed_edge_is_noticed_and_re_issued():
    """The buttons are `onTrue`, and the operator link transmits at
    50 Hz -- a press and release inside one 20 ms window never reaches
    the wire at all. The symptom is a command that reads as active while
    the mechanism sits stowed, which looks exactly like a broken mapping.

    So the arm angle, not the last thing sent, is the source of truth.
    """
    link, robot, view, reader = _view()
    robot.set_intake_active(True)
    _tick(view, robot, reader, 1.0, arm=90.0)
    assert link.buttons[(1, op.BTN_Y)] is True
    assert robot.intake_reasserts == 0

    # The arm stays stowed despite the command. Past the reassert window,
    # Y is released for one tick so the next press is a real edge.
    _tick(view, robot, reader, 2.0, arm=90.0)
    assert link.buttons[(1, op.BTN_Y)] is False
    assert robot.intake_reasserts == 1

    _tick(view, robot, reader, 2.05, arm=90.0)
    assert link.buttons[(1, op.BTN_Y)] is True, "and then pressed again"


def test_an_arm_that_obeys_is_left_alone():
    """The counterpart: no re-issue when the mechanism agrees, so a
    rising `intake_reasserts` really does mean edges are being lost."""
    link, robot, view, reader = _view()
    robot.set_intake_active(True)
    for t in (1.0, 2.0, 3.0, 4.0):
        _tick(view, robot, reader, t, arm=0.0)
    assert robot.intake_reasserts == 0
    assert link.buttons[(1, op.BTN_Y)] is True


def test_a_feeder_that_never_starts_gets_its_edge_re_issued():
    """The manip right trigger is `onTrue`/`onFalse`, so a press and
    release inside one 50 Hz transmit window never reaches the wire and
    the feeder simply stays off. Observed live before there was anything
    to check against: a burst emptied twenty balls and then sat with the
    deposit commanded and nothing leaving for fourteen seconds.

    `Spindexer/FeederOn` is what makes that detectable rather than merely
    avoidable, so this is the intake reconciliation's exact shape.
    """
    link, robot, view, reader = _view()
    robot.set_deposit_active(True)
    _tick(view, robot, reader, 0.1, feeder=False)
    assert link.axes[(1, op.AXIS_RIGHT_TRIGGER)] == 1.0
    assert robot.feeder_reasserts == 0, "not yet -- give the command time to run"

    # Commanded, and the feeder still reads off. Past the reassert window
    # the trigger is released for one tick so the next press is an edge.
    _tick(view, robot, reader, 2.0, feeder=False)
    assert link.axes[(1, op.AXIS_RIGHT_TRIGGER)] == 0.0
    assert robot.feeder_reasserts == 1

    _tick(view, robot, reader, 2.05, feeder=False)
    assert link.axes[(1, op.AXIS_RIGHT_TRIGGER)] == 1.0, "and then pressed again"


def test_a_feeder_that_obeys_is_left_alone():
    """The counterpart, so a rising `feeder_reasserts` really does mean
    edges are being lost rather than that the check is trigger-happy."""
    link, robot, view, reader = _view()
    robot.set_deposit_active(True)
    for t in (1.0, 2.0, 3.0, 4.0):
        _tick(view, robot, reader, t, feeder=True)
    assert robot.feeder_reasserts == 0
    assert link.axes[(1, op.AXIS_RIGHT_TRIGGER)] == 1.0


def test_the_drive_mapping_follows_the_observed_feeder_not_the_command():
    """`joystickDrive` halves the whole drivetrain from
    `spindexer::isFeederOn`, so which mapping to invert is a question
    about the feeder's *actual* state. Inferring it from the last button
    sent makes every commanded velocity out by a factor of two whenever
    an edge is lost -- silently, and looking like a navigation bug.
    """
    link, robot, view, reader = _view()
    robot.set_deposit_active(True, action=mv.SHOOT)
    _tick(view, robot, reader, 0.1, feeder=False)   # commanded, not yet running
    robot.drive_field_relative(0.05, 60.0, 0.0, 0.0)
    commanded_only = link.axes[(0, op.AXIS_LEFT_Y)]

    _tick(view, robot, reader, 0.2, feeder=True)    # now it really is running
    robot.drive_field_relative(0.05, 60.0, 0.0, 0.0)

    assert abs(link.axes[(0, op.AXIS_LEFT_Y)]) > abs(commanded_only), (
        "the same velocity needs more stick once the feeder has actually halved the drivetrain"
    )


def test_auto_aim_is_toggled_on_only_when_it_reads_off():
    """Manip Start is a toggle, so pressing it blind is as likely to turn
    auto-aim off as on. Without auto-aim the turret points wherever it
    was left and every shot lands on the floor -- which is what the first
    working strategy runs did: fuel left the robot, the loose count went
    up by the same amount, and the score stayed at zero."""
    link, robot, view, reader = _view()
    assert link.buttons.get((1, op.BTN_START)) is True, "it starts off, so ask for it"
    assert robot.aim_toggles == 1

    reader.auto_aim = True
    _tick(view, robot, reader, 2.0)
    assert link.buttons[(1, op.BTN_START)] is False, "released once it is on"
    assert robot.aim_toggles == 1, "and not pressed again"


def test_the_auto_aim_toggle_is_not_spammed_while_it_settles():
    """A toggle pressed once per tick while the command schedules would
    turn it on and off again several times a second."""
    link, robot, view, reader = _view()
    for t in (0.1, 0.2, 0.5):
        _tick(view, robot, reader, t)
    assert robot.aim_toggles == 1


def test_depositing_holds_the_flywheel_and_runs_the_feeder():
    link, robot, _, _ = _view()
    robot.set_deposit_active(True, action=mv.SHOOT)
    assert link.buttons[(0, op.BTN_LEFT_BUMPER)] is True
    assert link.axes[(1, op.AXIS_RIGHT_TRIGGER)] == 1.0
    assert robot.deposit_action == mv.SHOOT


def test_the_feeder_stays_on_between_two_shots_of_a_burst():
    """`Score` runs one `RunManipulator` per piece and cycles the deposit
    command between them. The manip right trigger is `onTrue`/`onFalse`,
    so a cycle inside one 50 Hz transmit window never reaches the wire and
    the feeder simply stays off.

    Observed live before this: a burst emptied twenty balls, then sat with
    the deposit commanded and nothing leaving for fourteen seconds.

    The feeder reads *on* throughout, because that is what a burst is --
    which is what keeps this a test of the debounce rather than of the
    reassert that now backs it up.
    """
    link, robot, view, reader = _view()
    robot.set_deposit_active(True, action=mv.SHOOT)
    _tick(view, robot, reader, 1.0, feeder=True)
    assert link.axes[(1, op.AXIS_RIGHT_TRIGGER)] == 1.0

    robot.set_deposit_active(False)
    _tick(view, robot, reader, 1.05, feeder=True)
    assert link.axes[(1, op.AXIS_RIGHT_TRIGGER)] == 1.0, "still the same burst"

    robot.set_deposit_active(True, action=mv.SHOOT)
    _tick(view, robot, reader, 1.10, feeder=True)
    assert link.axes[(1, op.AXIS_RIGHT_TRIGGER)] == 1.0
    assert robot.feeder_reasserts == 0, "the feeder was running; nothing to re-issue"


def test_a_real_stop_does_stop_the_feeder():
    """The counterpart -- the debounce must not make the feeder
    unstoppable, or a robot that has finished scoring keeps flinging fuel
    and keeps its drivetrain halved."""
    link, robot, view, reader = _view()
    robot.set_deposit_active(True, action=mv.SHOOT)
    _tick(view, robot, reader, 1.0, feeder=True)

    robot.set_deposit_active(False)
    _tick(view, robot, reader, 2.0, feeder=False)
    assert link.axes[(1, op.AXIS_RIGHT_TRIGGER)] == 0.0
    assert link.buttons[(0, op.BTN_LEFT_BUMPER)] is False


def test_a_shot_in_progress_changes_how_the_next_drive_is_inverted():
    """The feeder halves the drivetrain, so the same velocity needs a
    different stick.

    Keyed to the feeder being *observed* running, not to the deposit
    having been commanded. Those differ for as long as an edge takes to
    land, and for ever if it is lost.
    """
    link, robot, view, reader = _view()
    robot.drive_field_relative(0.05, 60.0, 0.0, 0.0)
    idle_stick = link.axes[(0, op.AXIS_LEFT_Y)]

    robot.set_deposit_active(True, action=mv.SHOOT)
    _tick(view, robot, reader, 0.1, feeder=True)
    robot.drive_field_relative(0.05, 60.0, 0.0, 0.0)
    assert abs(link.axes[(0, op.AXIS_LEFT_Y)]) > abs(idle_stick)


def test_release_all_lets_go_of_everything():
    link, robot, _, _ = _view()
    robot.set_intake_active(True)
    robot.set_deposit_active(True, action=mv.SHOOT)
    robot.drive_field_relative(0.05, 60.0, 0.0, 0.0)

    robot.release_all()
    assert not any(link.buttons.values())
    assert link.axes[(0, op.AXIS_LEFT_X)] == 0.0
    assert link.axes[(0, op.AXIS_LEFT_Y)] == 0.0
    assert link.axes[(0, op.AXIS_RIGHT_X)] == 0.0


# ---------------------------------------------------------------------------
# the match view's own answers
# ---------------------------------------------------------------------------


def test_an_inactive_hub_offers_no_scoring_slots():
    """The 25-second clock, expressed where the strategy layer already
    looks. Fuel shot into the HUB that is not accepting comes back as
    WastedFuel, so a robot holding fuel with no live HUB should stop
    seeing a scoring option and fall through to doing something else."""
    _, robot, view, reader = _view(held=5)
    assert scoring_slots_for_type(view, robot, arena.PIECE_TYPE)

    reader.hub_active = {"blue": False, "red": True}
    view.sync(0.1, Phase.TELEOP)
    assert scoring_slots_for_type(view, robot, arena.PIECE_TYPE) == []


def test_deposit_readiness_is_position_not_which_way_a_bumper_points():
    """A turret robot shoots from wherever it is standing. `Match`'s own
    version asks whether a bumper side engages the region, which is the
    right question for a robot that reaches into a structure and the
    wrong one here."""
    region = next(r for r in arena.build_arena().scoring_regions if r.name == "blue GOAL")
    inside = (sum(v[0] for v in region.vertices) / 4, sum(v[1] for v in region.vertices) / 4)

    _, robot, view, _ = _view(held=1, robot=Pose2d(inside[0] / arena.M_TO_IN, inside[1] / arena.M_TO_IN, 0.0))
    assert view.deposit_region_for(robot, action=mv.SHOOT).name == "blue GOAL"

    # Facing directly away from the HUB changes nothing -- the turret moves.
    robot.chassis.body.angle = math.pi
    assert view.deposit_region_for(robot, action=mv.SHOOT).name == "blue GOAL"


def test_nothing_is_ready_to_score_into_a_dead_hub():
    region = next(r for r in arena.build_arena().scoring_regions if r.name == "blue GOAL")
    inside = (sum(v[0] for v in region.vertices) / 4, sum(v[1] for v in region.vertices) / 4)
    _, robot, view, reader = _view(
        held=1, robot=Pose2d(inside[0] / arena.M_TO_IN, inside[1] / arena.M_TO_IN, 0.0),
        hub_active={"blue": False, "red": True},
    )
    assert view.deposit_region_for(robot, action=mv.SHOOT) is None


def test_a_robot_scores_from_the_zone_not_from_the_polygon():
    """The fix. The region polygon is a navigation aid and covers less
    ground than the rule does; while readiness was point-in-polygon on
    it, `Score` refused to shoot from thousands of square inches of
    perfectly good scoring floor and drove on to reach the polygon --
    on this field, through the 50-inch pinch beside the HUB ramp.

    The pose below is the one the campaign's `robot-pinned` finding
    reported, verbatim, in every long match: (3.720, 7.518) metres. It
    is inside the blue alliance zone and outside the blue GOAL polygon,
    which is precisely the gap this closes."""
    region = next(r for r in arena.build_arena().scoring_regions if r.name == "blue GOAL")
    pinned = (3.720, 7.518)
    assert not point_in_polygon(
        (pinned[0] * arena.M_TO_IN, pinned[1] * arena.M_TO_IN), region.vertices
    ), "pick a pose outside the polygon or this test proves nothing"

    _, robot, view, _ = _view(held=1, robot=Pose2d(pinned[0], pinned[1], 0.0))
    assert view.deposit_region_for(robot, action=mv.SHOOT).name == "blue GOAL"


def test_the_goal_mouth_is_still_not_a_scoring_pose():
    """The other direction, and the one that cost sixty-five shots. The
    mouths open toward midfield, which is *outside* the alliance zone, so
    a robot at the mouth is passing however well it aims."""
    mouth_x = arena.goal_face_x("blue") / arena.M_TO_IN
    hub_y = arena.hub_centre("blue")[1] / arena.M_TO_IN
    _, robot, view, _ = _view(held=1, robot=Pose2d(mouth_x + 0.3, hub_y, 0.0))
    assert view.deposit_region_for(robot, action=mv.SHOOT) is None


def test_readiness_does_not_depend_on_y():
    """`isInAllianceArea` is a half-plane in x. A robot in the far corner
    of its own zone is as ready as one beside the HUB, and the old
    polygon -- 47 inches tall, sized to the goal *mouth* -- said
    otherwise about almost the whole field width."""
    ready = []
    for y in (0.5, 2.0, 4.0, 6.0, 7.5):
        _, robot, view, _ = _view(held=1, robot=Pose2d(2.0, y, 0.0))
        ready.append(view.deposit_region_for(robot, action=mv.SHOOT) is not None)
    assert all(ready), ready


def test_pass_refuses_from_inside_the_alliance_zone():
    """The block this lifts, and the reason `Pass` shipped unwired.

    `Pass`'s guard is "I could not score from here". While readiness was
    point-in-polygon on the GOAL region, that was true across most of the
    alliance zone -- so a `Pass` rule would have fired from inside the
    area where `Turret.setTarget` aims at the HUB, making the "pass" a
    deliberately bad shot."""
    _, robot, view, _ = _view(held=3, robot=Pose2d(2.0, 4.0, math.pi))
    ctx = BehaviorContext(robot=robot, dt=0.02, match=view)
    assert Pass().tick(ctx) is Status.FAILURE
    assert robot.deposit_active is False


def test_pass_lets_go_from_outside_the_alliance_zone():
    """Where the same press really is a pass: outside the zone the turret
    retargets a corner of the robot's own end and lobs the fuel back. So
    the bridge needs no new intent for `Pass` -- a pass presses exactly
    what a score presses, and the whole difference is where the robot is
    standing.

    Pre-aimed, because a `MapleRobot`'s pose comes from NetworkTables and
    does not move in response to a drive command. The turning half of
    `Pass` is exercised against real physics in
    test/common_sim/test_tactics.py; what belongs here is the guard."""
    _, robot, view, _ = _view(held=3, robot=Pose2d(8.0, 4.0, math.pi))
    ctx = BehaviorContext(robot=robot, dt=0.02, match=view)
    assert Pass().tick(ctx) is Status.RUNNING
    assert robot.deposit_active, "already lined up on its own zone, so it lets go"


def test_an_empty_handed_robot_is_never_ready_to_score():
    _, robot, view, _ = _view(held=0)
    assert view.deposit_region_for(robot, action=mv.SHOOT) is None


def test_there_are_no_intake_locations_and_that_is_the_answer():
    """REBUILT has nothing a robot loads from on demand -- every piece,
    including what the OUTPOSTs throw back in, is loose on the floor. An
    empty list is the correct model, not a gap."""
    _, robot, view, _ = _view()
    assert view.field.intake_locations == ()
    assert view.station_supply == {}
    assert robot.nearby_station() is None


def test_nothing_is_protected_because_nothing_adjudicates_fouls():
    """Telling the strategy layer otherwise would have it rely on
    protection that does not exist in the simulation it is tested in."""
    from common_sim.control.world_view import is_protected

    _, robot, view, _ = _view()
    assert not is_protected(view, robot)


def test_the_scoring_constant_cancels_out_of_every_comparison():
    """With one action the value is arbitrary, and utility.py ranks by
    points-per-second -- so the ordering is by travel and deposit time.
    The moment a second action appears this stops being harmless."""
    rules = mv.ShootOnly()
    assert rules.points_for(mv.SHOOT, "teleop") > 0
    assert rules.points_for("nonexistent", "teleop") == 0
    actions = {a for r in arena.build_arena().scoring_regions for a in r.actions}
    assert actions == {mv.SHOOT}, "a second action means the constant has to be measured"


# ---------------------------------------------------------------------------
# end to end, against the stub
# ---------------------------------------------------------------------------


def _drive_strategy(shoot_at: int, *, held: int, fuel, hub_active=None, ticks: int = 4):
    from apps.run_bridge_strategy import cycle_fuel

    link, robot, view, reader = _view(held=held, fuel=fuel, hub_active=hub_active)
    controller = StrategyController(cycle_fuel(shoot_at), robot)
    robot.controller = controller
    for i in range(ticks):
        view.sync(i * 0.05, Phase.TELEOP)
        controller.tick(BehaviorContext(robot=robot, dt=0.05, elapsed=i * 0.05, match=view))
    return link, robot, view


def test_an_empty_robot_collects():
    link, robot, _ = _drive_strategy(
        20, held=0, fuel=[Pose3d(7.5, 2.6, 0.0), Pose3d(9.0, 3.0, 0.0)]
    )
    assert robot.intent.tactic_name == Collect.__name__
    assert link.buttons[(1, op.BTN_Y)] is True, "Collect must have deployed the intake"


def test_a_full_robot_goes_to_score():
    link, robot, _ = _drive_strategy(20, held=25, fuel=[Pose3d(7.5, 2.6, 0.0)])
    assert robot.intent.tactic_name == Score.__name__
    assert robot.intent.target_region == "blue GOAL"


def test_a_full_robot_with_no_live_hub_does_not_stand_there():
    """The fallthrough that makes the 25-second clock a strategy input
    rather than a wasted shot."""
    link, robot, _ = _drive_strategy(
        20, held=25, fuel=[Pose3d(7.5, 2.6, 0.0)], hub_active={"blue": False, "red": True}
    )
    assert robot.intent is None or robot.intent.target_region != "blue GOAL"
