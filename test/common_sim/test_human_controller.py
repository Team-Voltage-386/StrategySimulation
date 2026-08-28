"""
HumanController -- the seam that lets Match.step drive a human-piloted
robot through the same `robot.controller.tick(ctx)` path a StrategyController
uses, instead of the app special-casing "no controller -> drive it here".
"""
from common_sim.control.behavior import BehaviorContext
from common_sim.control.human import HumanController
from common_sim.control.input_sources import DriveCommand, OperatorCommand
from common_sim.field.field_config import FieldConfig, IntakeLocation, ScoringRegion
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.match.scoring import TableScoringRules
from common_sim.robot.characteristics import RobotCharacteristics

WIDGET = "widget"
GADGET = "gadget"


def make_field(intake_locations=()) -> FieldConfig:
    region = ScoringRegion(
        name="goal", vertices=((220, 40), (280, 40), (280, 160), (220, 160)),
        actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}),
    )
    return FieldConfig(width=300, height=200, scoring_regions=(region,), intake_locations=intake_locations)


def make_characteristics(**overrides) -> RobotCharacteristics:
    defaults = dict(
        name="test-bot", max_speed=150.0, max_accel=400.0, max_angular_speed=6.0,
        max_angular_accel=20.0, width=28.0, length=28.0, piece_capacity=1,
        intake_time=0.1, deposit_time=0.1, intake_range=6.0,
        accepted_piece_types=frozenset({WIDGET}),
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)


def make_match(intake_locations=()):
    rules = TableScoringRules({("score_widget", "auto"): 3.0})
    return Match(make_field(intake_locations), rules, MatchConfig(auto_duration=1000, teleop_duration=1000))


def test_intent_is_none_not_missing():
    """The one AttributeError trap: Robot.intent reads
    self.controller.intent unconditionally once a controller is assigned."""
    controller = HumanController(command_provider=lambda: (DriveCommand(), OperatorCommand()))
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    robot.controller = controller

    assert robot.intent is None  # must not raise


def test_command_provider_is_read_not_polled_by_the_controller():
    """HumanController must not own an InputSource poll() call -- it
    only ever reads whatever the caller already polled this frame, since
    OperatorCommand's edge-triggered fields would otherwise race a second
    poller. Verified here by counting provider calls: exactly one per tick,
    never more."""
    calls = []

    def provider():
        calls.append(1)
        return DriveCommand(vx=1.0), OperatorCommand()

    controller = HumanController(command_provider=provider)
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    robot.controller = controller

    for _ in range(5):
        match.step(1.0 / 60.0)

    assert len(calls) == 5


def test_drive_command_scales_by_robot_max_speed():
    controller = HumanController(command_provider=lambda: (DriveCommand(vx=1.0, vy=0.0, omega=0.0), OperatorCommand()))
    match = make_match()
    robot = match.add_robot(make_characteristics(max_speed=100.0), Pose2d(20, 100, 0))
    robot.controller = controller

    for _ in range(120):
        match.step(1.0 / 60.0)

    assert robot.pose.x > 20.0  # actually moved
    assert robot.chassis.commanded_velocity.length <= 100.0 + 1e-6


def test_deposit_action_provider_supplies_the_scoring_action():
    seen_actions = []

    def deposit_action():
        seen_actions.append("l4")
        return "l4"

    controller = HumanController(
        command_provider=lambda: (DriveCommand(), OperatorCommand(deposit_active=True)),
        deposit_action_provider=deposit_action,
    )
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    robot.controller = controller

    match.step(1.0 / 60.0)

    assert seen_actions
    assert robot.deposit_action == "l4"


def test_human_driven_robot_can_score_end_to_end():
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(240, 100, 3.14159265), alliance="blue")
    piece = match.spawn_piece(WIDGET, (240, 100))
    piece.held_by = robot
    piece.shape.sensor = True
    piece.last_holder_alliance = robot.alliance
    robot.held_pieces.append(piece)

    robot.controller = HumanController(
        command_provider=lambda: (DriveCommand(), OperatorCommand(deposit_active=True)),
        deposit_action_provider=lambda: "score_widget",
    )

    for _ in range(120):
        match.step(1.0 / 60.0)
        if piece.scored:
            break

    assert piece.scored
    assert match.scores.get("blue", 0.0) > 0


def test_synthesized_intent_targets_scoring_region_while_holding_a_piece():
    """A human carrying a piece should read to a Defend(opponent_intent)
    exactly like Score's own published intent would -- see
    world_view.likely_scoring_region and human.py's _synthesize_intent."""
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="blue")
    piece = match.spawn_piece(WIDGET, (20, 100))
    piece.held_by = robot
    piece.last_holder_alliance = robot.alliance
    robot.held_pieces.append(piece)
    robot.controller = HumanController(command_provider=lambda: (DriveCommand(), OperatorCommand()))

    match.step(1.0 / 60.0)

    assert robot.intent.tactic_name == "Human"
    assert robot.intent.target_region == "goal"
    assert robot.intent.target_piece is piece


def test_synthesized_intent_only_targets_a_region_for_a_held_piece_type():
    gadget_goal = ScoringRegion(
        name="gadget_goal", vertices=((30, 40), (70, 40), (70, 160), (30, 160)),
        actions=frozenset({"score_gadget"}), piece_types=frozenset({GADGET}),
    )
    widget_goal = ScoringRegion(
        name="widget_goal", vertices=((220, 40), (280, 40), (280, 160), (220, 160)),
        actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}),
    )
    field = FieldConfig(width=300, height=200, scoring_regions=(gadget_goal, widget_goal))
    match = Match(
        field, TableScoringRules({("score_widget", "auto"): 3.0}),
        MatchConfig(auto_duration=1000, teleop_duration=1000),
    )
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="blue")
    piece = match.spawn_piece(WIDGET, (20, 100))
    piece.held_by = robot
    piece.last_holder_alliance = robot.alliance
    robot.held_pieces.append(piece)
    robot.controller = HumanController(command_provider=lambda: (DriveCommand(), OperatorCommand()))

    match.step(1.0 / 60.0)

    assert robot.intent.target_region == "widget_goal"
    assert robot.intent.target_piece is piece


def test_synthesized_intent_targets_nearest_collectable_piece_when_empty_handed():
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="blue")
    near = match.spawn_piece(WIDGET, (40, 100))
    far = match.spawn_piece(WIDGET, (280, 190))
    robot.controller = HumanController(command_provider=lambda: (DriveCommand(), OperatorCommand()))

    match.step(1.0 / 60.0)

    assert robot.intent.target_piece is near
    assert robot.intent.target_piece is not far
    assert robot.intent.target_region is None


def test_synthesized_intent_targets_station_over_a_farther_piece():
    station = IntakeLocation(name="feeder", vertices=((0, 0), (20, 0), (20, 20), (0, 20)), piece_type=WIDGET)
    match = make_match(intake_locations=(station,))
    robot = match.add_robot(make_characteristics(), Pose2d(15, 15, 0), alliance="blue")
    match.spawn_piece(WIDGET, (280, 190))  # far piece, should lose to the near station
    robot.controller = HumanController(command_provider=lambda: (DriveCommand(), OperatorCommand()))

    match.step(1.0 / 60.0)

    assert robot.intent.target_region == "feeder"
    assert robot.intent.target_piece is None


def test_synthesized_intent_is_none_of_everything_with_nothing_on_the_field():
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="blue")
    robot.controller = HumanController(command_provider=lambda: (DriveCommand(), OperatorCommand()))

    match.step(1.0 / 60.0)

    assert robot.intent.tactic_name == "Human"
    assert robot.intent.target_region is None
    assert robot.intent.target_piece is None
