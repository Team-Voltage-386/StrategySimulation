"""
Headless batch trial runner: sweep RobotCharacteristics (or any other
per-trial config) over a basic design-of-experiments grid, run each
trial's Match to completion with no wall-clock throttling, and collect
metrics -- this is the "faster than real time" half of the tool.

A trial is defined by the caller as `trial_fn(params: dict) -> Match`,
already stepped to completion (typically via run_match_to_completion
below) -- monte_carlo.py knows nothing about how a Match gets built or
what a game's pieces/scoring look like. For `parallel=True`,
`trial_fn` is sent to worker processes via multiprocessing, so it must
be a module-level function (picklable), not a lambda or closure --
the same constraint Python's multiprocessing always has.
"""
from __future__ import annotations

import itertools
from dataclasses import dataclass
from typing import Callable, Optional

from common_sim.analysis.metrics import MatchMetrics, extract_metrics
from common_sim.analysis.runner import run_all
from common_sim.match.match import Match


def run_match_to_completion(match: Match, dt: float = 1.0 / 20.0, max_ticks: int = 1_000_000) -> Match:
    """Step `match` with no throttling until it ends (or `max_ticks` is
    hit, as a runaway-config guard rather than a real limit)."""
    ticks = 0
    while not match.ended and ticks < max_ticks:
        match.step(dt)
        ticks += 1
    return match


@dataclass(frozen=True)
class ParameterSweep:
    """Full-factorial grid over named parameters: `values` maps a
    parameter name to the list of values to try, and every combination
    becomes one trial config. A different generator with the same
    `.configs()` shape (e.g. random or Latin-hypercube sampling) can
    stand in for this without changing run_monte_carlo."""
    values: dict

    def configs(self) -> list[dict]:
        keys = list(self.values.keys())
        if not keys:
            return [{}]
        combos = itertools.product(*(self.values[k] for k in keys))
        return [dict(zip(keys, combo)) for combo in combos]


@dataclass(frozen=True)
class TrialResult:
    params: dict
    metrics: MatchMetrics


def _run_one(trial_fn: Callable[[dict], Match], params: dict) -> TrialResult:
    match = trial_fn(params)
    return TrialResult(params=params, metrics=extract_metrics(match))


def _run_one_pair(pair: tuple) -> TrialResult:
    """Module-level (not a closure) so it stays picklable for
    run_all's parallel=True / multiprocessing path."""
    trial_fn, params = pair
    return _run_one(trial_fn, params)


def run_monte_carlo(
    trial_fn: Callable[[dict], Match],
    sweep: ParameterSweep,
    repetitions: int = 1,
    parallel: bool = False,
    max_workers: Optional[int] = None,
) -> list[TrialResult]:
    """Run every (config, repetition) combination and return one
    TrialResult per run. Repeating each config multiple times is what
    makes Monte Carlo actually Monte Carlo when trial_fn has any
    randomness in it (e.g. a vision noise model, a randomized behavior);
    for a fully deterministic trial_fn, repetitions>1 just wastes time.

    Expressed on top of common_sim/analysis/runner.run_all -- the same
    bounded-submission, cancellable executor the SWEEP tab uses."""
    configs = [cfg for cfg in sweep.configs() for _ in range(repetitions)]
    items = [(trial_fn, cfg) for cfg in configs]
    return run_all(_run_one_pair, items, parallel=parallel, max_workers=max_workers)
