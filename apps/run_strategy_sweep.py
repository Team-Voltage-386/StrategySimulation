"""
Headless strategy-comparison sweep -- a scriptable proof that the SWEEP
tab's engine (common_sim/analysis/sweep_spec.py + runner.py +
game_specific/reefscape/sweep_trial.py) works with no Qt anywhere in the
import graph. Demonstrates that "strategy" is just another sweepable
field: SweepVariable doesn't care whether its path names a numeric
RobotCharacteristics field or "strategy" -- expand_jobs treats both the
same way, and results.to_dataframe carries every swept column (strategy
included) straight through into the DataFrame.

Run: `python -m apps.run_strategy_sweep`
"""
from __future__ import annotations

from common_sim.analysis.results import summarize, to_dataframe
from common_sim.analysis.runner import run_all
from common_sim.analysis.sweep_spec import MatchSpec, RobotSpec, SweepVariable, characteristics_to_spec, expand_jobs
from common_sim.analysis.variability import VariabilityModel
from common_sim.robot.characteristics import RobotCharacteristics
from game_specific.reefscape.game_pieces import ALGAE_TYPE, CORAL_TYPE
from game_specific.reefscape.sweep_trial import STRATEGIES_DIR, SWEEP_DT, run_trial

DEFAULT_PIECE_CAPACITY = {CORAL_TYPE: 1, ALGAE_TYPE: 1}
DEFAULT_INTAKE_TIMES = {CORAL_TYPE: 0.4, ALGAE_TYPE: 0.4}
DEFAULT_DEPOSIT_TIMES = {"l1": 0.3, "l2": 0.6, "l3": 1.0, "l4": 1.8, "processor": 0.4, "net": 1.2}


def build_characteristics() -> RobotCharacteristics:
    return RobotCharacteristics(
        name="sweep-bot", max_speed=150.0, max_accel=400.0, max_angular_speed=6.0, max_angular_accel=20.0,
        width=28.0, length=28.0,
        piece_capacity_by_type=dict(DEFAULT_PIECE_CAPACITY),
        intake_time_by_type=dict(DEFAULT_INTAKE_TIMES), station_intake_time=0.6, intake_range=6.0,
        deposit_time=0.5, deposit_time_by_action=dict(DEFAULT_DEPOSIT_TIMES),
        accepted_piece_types=frozenset({CORAL_TYPE, ALGAE_TYPE}),
    )


def main() -> None:
    char_spec = characteristics_to_spec(build_characteristics())
    robot = RobotSpec(label="PRIMARY", alliance="blue", roster_index=-1, characteristics=char_spec, strategy="cycle_coral")
    match = MatchSpec(auto_duration=15.0, teleop_duration=135.0)

    variables = [
        SweepVariable(
            target="PRIMARY", path="strategy",
            values=("cycle_coral", "algae_processor", "endgame_defense", "auto_then_cycle"),
        ),
        SweepVariable(target="PRIMARY", path="max_speed", values=(130.0, 150.0, 170.0)),
    ]

    jobs = expand_jobs([robot], match, VariabilityModel(), variables, repetitions=3, strategies_dir=STRATEGIES_DIR, dt=SWEEP_DT)
    outcomes = run_all(run_trial, jobs, parallel=True)

    errors = [o for o in outcomes if o.error is not None]
    if errors:
        print(f"{len(errors)} trial(s) failed, e.g.:\n{errors[0].error}")
    outcomes = [o for o in outcomes if o.error is None]

    df = to_dataframe(outcomes)
    print(df.to_string(index=False))
    print()
    print(summarize(df, ["PRIMARY.strategy"], metric="total_score"))


if __name__ == "__main__":
    main()
