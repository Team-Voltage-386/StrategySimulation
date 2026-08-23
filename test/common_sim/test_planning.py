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


def _two_action_setup(**characteristics_overrides):
    """One region, two ways to use it: `rich` pays more per second on
    gross points, and less once the misses are counted. The REEFSCAPE
    L3-vs-L4 shape, which is where the distinction actually bites."""
    goal = ScoringRegion(
        name="goal", vertices=((380, -60), (420, -60), (420, 60), (380, 60)),
        actions=frozenset({"cheap", "rich"}), piece_types=frozenset({WIDGET}),
    )
    field = FieldConfig(width=500, height=200, scoring_regions=(goal,))
    rules = TableScoringRules({("cheap", "auto"): 4.0, ("rich", "auto"): 5.0})
    match = Match(field, rules, MatchConfig(auto_duration=1000, teleop_duration=1000))
    characteristics = make_characteristics(
        piece_capacity=1, deposit_time_by_action={"cheap": 1.0, "rich": 1.8},
        **characteristics_overrides,
    )
    robot = match.add_robot(characteristics, Pose2d(0, 0, 0))
    piece = match.spawn_piece(WIDGET, (0, 0))
    piece.held_by = robot
    robot.held_pieces.append(piece)
    return match, robot


def test_greedy_planner_ranks_on_expected_points():
    """A plan made in gross points is a plan the robot will not be paid
    for. `rich` wins on points per second and loses on points per second
    actually landed, and the planner has to follow the second one --
    otherwise reliability can only reach the decision by somebody
    pinning an action from outside, which is what Pursue tried and what
    cost 51 points a match."""
    match, robot = _two_action_setup(
        scoring_reliability_by_action={"cheap": 0.9, "rich": 0.82},
    )
    plan = GreedyRatePlanner().plan(match, robot)
    assert [o.action for o in plan] == ["cheap"]


def test_greedy_planner_still_takes_the_richer_target_when_it_lands():
    """The control for the test above: identical geometry and points, a
    robot that never misses. If this one ever fails, the fixture's
    travel time has drifted out of the window where the two rankings
    disagree at all and the test above has stopped testing anything."""
    match, robot = _two_action_setup()
    plan = GreedyRatePlanner().plan(match, robot)
    assert [o.action for o in plan] == ["rich"]
