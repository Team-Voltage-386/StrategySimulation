"""
End-to-end: Match.step ticks a robot's controller before physics, so a
full Collect -> Score strategy can run a robot to completion driven
purely by match.step(dt) calls -- no external tactic/behavior ticking
required, unlike a hand-driven behavior loop.
"""
from common_sim.control.strategy import Rule, Strategy, StrategyController
from common_sim.control.tactics import Collect, Score
from common_sim.control.triggers import AtCapacity, PiecesHeld
from common_sim.field.field_config import FieldConfig, ScoringRegion
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.match.scoring import TableScoringRules
from common_sim.robot.characteristics import RobotCharacteristics

WIDGET = "widget"


def make_field() -> FieldConfig:
    region = ScoringRegion(
        name="goal", vertices=((220, 40), (280, 40), (280, 160), (220, 160)),
        actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}),
    )
    return FieldConfig(width=300, height=200, scoring_regions=(region,))


def make_characteristics(**overrides) -> RobotCharacteristics:
    defaults = dict(
        name="test-bot", max_speed=150.0, max_accel=400.0, max_angular_speed=6.0,
        max_angular_accel=20.0, width=28.0, length=28.0, piece_capacity=1,
        intake_time=0.1, deposit_time=0.1, intake_range=6.0,
        accepted_piece_types=frozenset({WIDGET}),
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)


def test_match_step_drives_controller_end_to_end():
    field = make_field()
    rules = TableScoringRules({("score_widget", "auto"): 3.0})
    match = Match(field, rules, MatchConfig(auto_duration=1000, teleop_duration=1000))
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    piece = match.spawn_piece(WIDGET, (40, 100))

    strategy = Strategy(name="cycle", rules=[
        Rule(name="score", trigger=AtCapacity(piece_type=WIDGET), tactic=Score(), priority=1),
        Rule(name="collect", trigger=PiecesHeld(piece_type=WIDGET, max_count=0), tactic=Collect(piece_type=WIDGET), priority=0),
    ])
    robot.controller = StrategyController(strategy, robot)

    for _ in range(3000):
        match.step(1.0 / 60.0)
        if piece.scored:
            break

    assert piece.scored
    assert match.scores.get("blue", 0.0) > 0
    assert robot.intent is not None
