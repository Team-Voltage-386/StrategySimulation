"""
Headless strategy-comparison sweep -- demonstrates that "strategy" is
just another swept parameter to common_sim/analysis/monte_carlo.py:
ParameterSweep already takes arbitrary named params, and
analysis/results.to_dataframe already carries every param (including
"strategy") through into the results DataFrame, so no changes to
monte_carlo.py itself were needed to add this.

trial_fn is module-level (not a closure) because run_monte_carlo's
parallel=True path sends it to worker processes via multiprocessing,
which requires it to be picklable.

Run: `python -m apps.run_strategy_sweep`
"""
from __future__ import annotations

from pathlib import Path

from common_sim.analysis.monte_carlo import ParameterSweep, run_match_to_completion, run_monte_carlo
from common_sim.analysis.results import summarize, to_dataframe
from common_sim.control import strategy_io
from common_sim.control.strategy import StrategyController
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from game_specific.reefscape.field import build_field, coral_station_positions, reef_center
from game_specific.reefscape.game_pieces import ALGAE_TYPE, CORAL_TYPE, spawn_algae, spawn_coral
from game_specific.reefscape.scoring import REEFSCAPE_SCORING_RULES
from common_sim.robot.characteristics import RobotCharacteristics

STRATEGIES_DIR = Path(__file__).resolve().parent.parent / "game_specific" / "reefscape" / "strategies"

DEFAULT_PIECE_CAPACITY = {CORAL_TYPE: 1, ALGAE_TYPE: 1}
DEFAULT_INTAKE_TIMES = {CORAL_TYPE: 0.4, ALGAE_TYPE: 0.4}
DEFAULT_DEPOSIT_TIMES = {"l1": 0.3, "l2": 0.6, "l3": 1.0, "l4": 1.8, "processor": 0.4, "net": 1.2}


def build_characteristics(max_speed: float) -> RobotCharacteristics:
    return RobotCharacteristics(
        name="sweep-bot", max_speed=max_speed, max_accel=400.0, max_angular_speed=6.0, max_angular_accel=20.0,
        width=28.0, length=28.0,
        piece_capacity_by_type=dict(DEFAULT_PIECE_CAPACITY),
        intake_time_by_type=dict(DEFAULT_INTAKE_TIMES), station_intake_time=0.6, intake_range=6.0,
        deposit_time=0.5, deposit_time_by_action=dict(DEFAULT_DEPOSIT_TIMES),
        accepted_piece_types=frozenset({CORAL_TYPE, ALGAE_TYPE}),
    )


def build_match(alliance: str) -> Match:
    """Same demo field-scatter apps/run_reefscape.py's build_demo_match
    uses -- kept as its own small copy here rather than imported, so
    this headless script doesn't pull in run_reefscape's Qt/GUI imports
    just to build a match."""
    field = build_field()
    match = Match(field, REEFSCAPE_SCORING_RULES, MatchConfig(auto_duration=15, teleop_duration=135))
    center = reef_center(alliance)
    toward_wall = -1.0 if alliance == "blue" else 1.0
    for i in range(6):
        spawn_coral(match, (center[0] + toward_wall * 60, center[1] + (i - 2.5) * 12))
    for i in range(3):
        spawn_algae(match, (center[0] + toward_wall * 40, center[1] - 60 + 40 * i))
    return match


def trial_fn(params: dict) -> Match:
    alliance = params.get("alliance", "blue")
    match = build_match(alliance)

    station = coral_station_positions(alliance)[0]
    facing = 0.0 if alliance == "blue" else 3.14159265
    start_pose = Pose2d(station[0] + (30.0 if alliance == "blue" else -30.0), station[1], facing)
    characteristics = build_characteristics(max_speed=params.get("max_speed", 150.0))
    robot = match.add_robot(characteristics, start_pose, alliance=alliance)

    strategy = strategy_io.load_strategy(STRATEGIES_DIR / f"{params['strategy']}.json")
    robot.controller = StrategyController(strategy, robot)

    return run_match_to_completion(match)


def main() -> None:
    sweep = ParameterSweep({
        "strategy": ["cycle_coral", "algae_processor", "endgame_defense", "auto_then_cycle"],
        "max_speed": [130.0, 150.0, 170.0],
    })
    results = run_monte_carlo(trial_fn, sweep, repetitions=3)
    df = to_dataframe(results)

    print(df.to_string(index=False))
    print()
    print(summarize(df, ["strategy"], metric="total_score"))


if __name__ == "__main__":
    main()
