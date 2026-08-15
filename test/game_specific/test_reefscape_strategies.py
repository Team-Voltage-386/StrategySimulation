"""
Loads every example strategy file under
game_specific/reefscape/strategies/ and runs a short headless match with
it attached to a robot on the real REEFSCAPE field -- these files double
as the strategy format's documentation and as fixtures proving the
whole world_view -> triggers -> tactics -> strategy -> strategy_io stack
actually drives a robot on a real (not synthetic) game.
"""
from pathlib import Path

import pytest

from common_sim.control import strategy_io
from common_sim.control.strategy import StrategyController
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.robot.characteristics import RobotCharacteristics, SideManipulators
from game_specific.reefscape.field import build_field, reef_center
from game_specific.reefscape.game_pieces import CORAL_TYPE, spawn_coral
from game_specific.reefscape.scoring import REEFSCAPE_SCORING_RULES

STRATEGIES_DIR = Path(__file__).resolve().parents[2] / "game_specific" / "reefscape" / "strategies"
STRATEGY_FILES = sorted(STRATEGIES_DIR.glob("*.json"))


def make_characteristics() -> RobotCharacteristics:
    return RobotCharacteristics(
        name="strategy-bot", max_speed=150.0, max_accel=400.0, max_angular_speed=6.0,
        max_angular_accel=20.0, width=28.0, length=28.0, piece_capacity=1,
        intake_time=0.1, deposit_time=0.1, intake_range=6.0,
        deposit_time_by_action={"l1": 0.2, "l2": 0.3, "l3": 0.4, "l4": 0.5, "processor": 0.3},
        side_manipulators={
            "front": SideManipulators(
                intake_piece_types=frozenset({"coral", "algae"}),
                score_piece_types=frozenset({"coral", "algae"}),
            ),
        },
    )


@pytest.mark.parametrize("path", STRATEGY_FILES, ids=lambda p: p.stem)
def test_strategy_file_loads_and_runs(path):
    strategy = strategy_io.load_strategy(path)
    assert strategy.rules, f"{path.name} declares no rules"

    field = build_field()
    match = Match(field, REEFSCAPE_SCORING_RULES, MatchConfig(auto_duration=15.0, teleop_duration=1000.0))
    start = reef_center("blue")
    robot = match.add_robot(make_characteristics(), Pose2d(start[0] - 100, start[1], 0), alliance="blue")
    robot.controller = StrategyController(strategy, robot)
    spawn_coral(match, (start[0] - 150, start[1]))

    for _ in range(600):  # 10s headless -- just proving it runs without error, not full-cycle completion
        match.step(1.0 / 60.0)

    assert robot.intent is not None
