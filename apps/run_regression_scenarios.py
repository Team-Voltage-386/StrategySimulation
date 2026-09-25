"""Run the short, fixed-seed REEFSCAPE release-regression scenarios.

    python -m apps.run_regression_scenarios
    python -m apps.run_regression_scenarios --scenario cycler_vs_defense

Use this before a demo/release when a full pytest run would hide the useful
answer in hundreds of unit tests.  A failure prints the scenario name and the
metric that moved; replay its job in the MATCH tab before accepting a new
golden value.
"""
from __future__ import annotations

import argparse

from game_specific.reefscape.regression_scenarios import SCENARIOS
from game_specific.reefscape.sweep_trial import run_trial


def _problems(scenario, metrics) -> list[str]:
    problems = []
    if metrics.final_scores != scenario.expected_scores:
        problems.append(f"scores expected {scenario.expected_scores}, got {metrics.final_scores}")
    if metrics.pieces_scored != scenario.expected_pieces_scored:
        problems.append(f"pieces scored expected {scenario.expected_pieces_scored}, got {metrics.pieces_scored}")
    if metrics.misses != scenario.expected_misses:
        problems.append(f"misses expected {scenario.expected_misses}, got {metrics.misses}")
    return problems


def main() -> int:
    parser = argparse.ArgumentParser(description="Run fixed-seed REEFSCAPE release regressions.")
    parser.add_argument("--scenario", action="append", choices=[scenario.key for scenario in SCENARIOS],
                        help="run only this scenario (repeatable); default is all")
    args = parser.parse_args()
    wanted = set(args.scenario or [scenario.key for scenario in SCENARIOS])

    failed = False
    for scenario in SCENARIOS:
        if scenario.key not in wanted:
            continue
        outcome = run_trial(scenario.job())
        if outcome.error is not None:
            print(f"FAIL {scenario.key}: trial error\n{outcome.error}")
            failed = True
            continue
        problems = _problems(scenario, outcome.metrics)
        if problems:
            print(f"FAIL {scenario.key}: {'; '.join(problems)}")
            failed = True
        else:
            print(f"PASS {scenario.key}: {scenario.purpose}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
