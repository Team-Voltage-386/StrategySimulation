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
from game_specific.reefscape.robot import build_characteristics
from game_specific.reefscape.sweep_trial import STRATEGIES_DIR, SWEEP_DT, run_trial


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
