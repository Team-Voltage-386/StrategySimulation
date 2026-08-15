from common_sim.control.planning import GreedyRatePlanner
from common_sim.field.field_config import FieldConfig, ScoringRegion
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.match.scoring import TableScoringRules
from common_sim.robot.characteristics import RobotCharacteristics

WIDGET = "widget"


def make_characteristics(**overrides) -> RobotCharacteristics:
    defaults = dict(
        name="test-bot", max_speed=150.0, max_accel=400.0, max_angular_speed=6.0,
        max_angular_accel=20.0, width=28.0, length=28.0, piece_capacity=2,
        intake_time=0.1, deposit_time=0.5, intake_range=6.0,
        accepted_piece_types=frozenset({WIDGET}),
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)


def test_greedy_planner_orders_by_value_rate():
    near = ScoringRegion(
        name="near", vertices=((20, -10), (40, -10), (40, 10), (20, 10)),
        actions=frozenset({"score"}), piece_types=frozenset({WIDGET}),
    )
    far = ScoringRegion(
        name="far", vertices=((280, -10), (300, -10), (300, 10), (280, 10)),
        actions=frozenset({"score"}), piece_types=frozenset({WIDGET}),
    )
    field = FieldConfig(width=300, height=200, scoring_regions=(near, far))
    # Same points, but "far" costs much more travel time -- value_rate
    # should favor "near" every time this rate matters.
    rules = TableScoringRules({("score", "auto"): 10.0})
    match = Match(field, rules, MatchConfig(auto_duration=1000, teleop_duration=1000))
    robot = match.add_robot(make_characteristics(piece_capacity=1), Pose2d(0, 0, 0))
    piece = match.spawn_piece(WIDGET, (0, 0))
    piece.held_by = robot
    robot.held_pieces.append(piece)

    plan = GreedyRatePlanner().plan(match, robot)
    assert len(plan) == 1
    assert plan[0].region.name == "near"
    assert plan[0].points == 10.0


def test_greedy_planner_prefers_higher_points_when_travel_time_equal():
    region = ScoringRegion(
        name="reef", vertices=((80, -60), (250, -60), (250, 60), (80, 60)),
        actions=frozenset({"l1", "l4"}), piece_types=frozenset({WIDGET}),
    )
    field = FieldConfig(width=300, height=200, scoring_regions=(region,))
    rules = TableScoringRules({("l1", "auto"): 2.0, ("l4", "auto"): 7.0})
    match = Match(field, rules, MatchConfig(auto_duration=1000, teleop_duration=1000))
    characteristics = make_characteristics(piece_capacity=1, deposit_time_by_action={"l1": 0.1, "l4": 0.1})
    robot = match.add_robot(characteristics, Pose2d(150, 0, 0))
    piece = match.spawn_piece(WIDGET, (0, 0))
    piece.held_by = robot
    robot.held_pieces.append(piece)

    plan = GreedyRatePlanner().plan(match, robot)
    assert len(plan) == 1
    assert plan[0].action == "l4"  # same travel time, l4 pays far more


def test_greedy_planner_chains_multiple_held_pieces():
    region = ScoringRegion(
        name="goal", vertices=((80, -60), (250, -60), (250, 60), (80, 60)),
        actions=frozenset({"score"}), piece_types=frozenset({WIDGET}),
    )
    field = FieldConfig(width=300, height=200, scoring_regions=(region,))
    rules = TableScoringRules({("score", "auto"): 5.0})
    match = Match(field, rules, MatchConfig(auto_duration=1000, teleop_duration=1000))
    characteristics = make_characteristics(piece_capacity=2)
    robot = match.add_robot(characteristics, Pose2d(150, 0, 0))
    for pos in ((0, 0), (0, 10)):
        piece = match.spawn_piece(WIDGET, pos)
        piece.held_by = robot
        robot.held_pieces.append(piece)

    plan = GreedyRatePlanner().plan(match, robot)
    assert len(plan) == 2
    assert {opt.piece for opt in plan} == set(robot.held_pieces)


def test_greedy_planner_returns_empty_when_nothing_held():
    field = FieldConfig(width=300, height=200)
    match = Match(field, TableScoringRules({}), MatchConfig())
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))
    assert GreedyRatePlanner().plan(match, robot) == []
