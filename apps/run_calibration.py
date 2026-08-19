"""What can this computer actually run?

Sweeps and searches are sized in matches, but a match costs very
different wall time on different hardware, and this project gets run by
whoever on the team has a machine free. Rather than quote one reference
machine's numbers in the docs and let everyone else be surprised, this
measures the machine it is run on and prints what batches are realistic
on it.

Run: `python -m apps.run_calibration [--matches N] [--dt 1/30] [--serial]`
"""
from __future__ import annotations

import argparse
from fractions import Fraction

from common_sim.analysis.calibration import measure, report
from common_sim.analysis.runner import default_worker_count
from common_sim.analysis.sweep_spec import MatchSpec, RobotSpec, characteristics_to_spec, expand_jobs
from common_sim.analysis.variability import VariabilityModel
from game_specific.reefscape.sweep_trial import STRATEGIES_DIR, SWEEP_DT, run_trial
from apps.run_strategy_sweep import build_characteristics

# A deliberately ordinary workload: a full-length 3-robot match with one
# robot cycling, one partner and one opponent. Not the cheapest possible
# match and not the most expensive -- the point is that the number
# transfers to what a real sweep will cost, so it has to look like one.
def reference_jobs(n: int, dt: float):
    char = characteristics_to_spec(build_characteristics())
    robots = [
        RobotSpec(label="PRIMARY", alliance="blue", roster_index=-1,
                  characteristics=dict(char), strategy="cycle_coral"),
        RobotSpec(label="PARTNER", alliance="blue", roster_index=0,
                  characteristics=dict(char), strategy="cycle_coral"),
        RobotSpec(label="OPPONENT", alliance="red", roster_index=0,
                  characteristics=dict(char), strategy="cycle_coral"),
    ]
    return expand_jobs(
        robots, MatchSpec(auto_duration=15.0, teleop_duration=135.0),
        VariabilityModel(), [], repetitions=n, strategies_dir=STRATEGIES_DIR, dt=dt,
    )


def _parse_dt(text: str) -> float:
    """Accept either "1/30" or "0.0333" -- the first is how everyone
    talks about it and the second is what a script would pass."""
    return float(Fraction(text))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matches", type=int, default=16,
                        help="reference matches to run (default 16)")
    parser.add_argument("--dt", type=_parse_dt, default=SWEEP_DT,
                        help=f"control timestep, e.g. 1/30 (default {SWEEP_DT:.5f} = SWEEP_DT)")
    parser.add_argument("--serial", action="store_true",
                        help="measure single-core throughput instead of parallel")
    args = parser.parse_args()

    workers = 1 if args.serial else default_worker_count()
    print(f"Timing {args.matches} reference matches at dt=1/{1/args.dt:.0f} "
          f"on {workers} worker{'s' if workers != 1 else ''}...")
    jobs = reference_jobs(args.matches, args.dt)
    throughput = measure(run_trial, jobs, parallel=not args.serial)
    print()
    print(report(throughput))
    print()
    print("Timings scale with the control timestep: a coarser --dt runs "
          "proportionally more matches per hour.")


if __name__ == "__main__":
    main()
