"""
Phase 2 of the development path: CMA-ES over the continuous fields of an
existing `Rule[]`, holding the rule structure fixed.

What this is *for* is worth stating before what it does, because the
obvious reading is wrong. This is not a research toy bolted onto the
sweep -- it is a correction to the sweep's numbers. `expand_jobs` +
`run_trial` compute `score(design, strategy)`. What a build team needs is
`max over strategies of score(design, ...)` -- the best that design can
do. The strategy x speed grid approximates that maximum over six
hand-written strategies, all authored with roughly one robot in mind, so
it is a *biased* lower bound, and the bias is worst exactly where the
interesting design trades live: a high-capacity robot is being judged by
strategies nobody wrote for a high-capacity robot. Running the parameter
loop once per design point replaces that with a best response per design.

Three pieces, kept separate because they fail differently:

* `search_parameters` -- the loop. Normalizes the box, drives `CMAES`,
  and knows nothing about matches.
* `AllianceScoreEvaluator` -- turns candidate strategies into TrialJobs
  and their outcomes into one number each. Game-agnostic: the caller
  passes the worker (`run_trial`) in.
* the entry point (`apps/run_param_search.py`) -- which robots, which
  strategy, which game.

Two properties of the evaluator matter more than anything in the
optimizer, because both are ways to get a plausible-looking answer that
means nothing:

**Common random numbers.** Every candidate in every generation is
evaluated on the *same* seed set. The quantity CMA-ES ranks on is then a
paired difference, and the seed-to-seed variance -- which is several
times larger than the effect being chased -- mostly cancels. Give each
candidate fresh seeds instead and the search spends its budget ranking
luck.

**Variability has to be on.** With `VariabilityModel(enabled=False)` and
no scoring-reliability configured, nothing in a trial consumes the seed:
`perturb_characteristics` returns its input, `scatter_offset` returns
zero, and `Match._rng` is never drawn from. An N-seed evaluation is then
N identical matches, and the search reads a single sample's noise as
signal. That is silent, so this module refuses it (see
`AllianceScoreEvaluator`).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np

from common_sim.analysis.cmaes import CMAES
from common_sim.analysis.runner import run_all
from common_sim.analysis.sweep_spec import apply_variable, expand_jobs
from common_sim.control import strategy_params

# The fitness of a candidate no trial could evaluate. Not a large finite
# penalty: a finite one is a number CMA-ES will happily interpolate
# toward, and "this strategy crashed" is not a point on the landscape.
FAILED = float("-inf")


def default_population(n: int) -> int:
    """CMA-ES's default population for an `n`-dimensional problem.

    Duplicated from `CMAES` so a run can be *sized* before the optimizer
    exists -- both the CLI's `--estimate-only` and the GUI's budget
    readout need the number while the user is still choosing settings.
    `CMAES` remains the authority; `test_cmaes` pins the two together.
    """
    return 4 + int(3 * math.log(n))


def matches_required(generations: int, population: int, seeds: int, confirm_seeds: int) -> int:
    """Total matches one search costs, including its confirmation.

    `+1` is the baseline evaluation, which is a real match set and not a
    remembered number (see `search_parameters`); the confirmation
    re-scores two strategies, so it costs `2 * confirm_seeds`.
    """
    return (generations * population + 1) * seeds + 2 * confirm_seeds


@dataclass
class Generation:
    """One generation's record, for a progress callback and the log."""
    index: int
    best: float
    mean: float
    best_so_far: float
    sigma: float
    failures: int
    seconds: float


@dataclass
class SearchResult:
    payload: dict                  # the winning strategy, ready for strategy_io.from_dict
    fitness: float                 # its mean score over the seed set
    baseline_payload: dict         # what the search started from
    baseline_fitness: float        # and what that scored, on the same seeds
    refs: tuple                    # the ParamRefs, so a report can name the axes
    vector: tuple                  # winning values, in refs order
    generations: tuple = ()
    matches: int = 0

    @property
    def improvement(self) -> float:
        return self.fitness - self.baseline_fitness

    def summary(self) -> str:
        lines = [
            f"baseline {self.baseline_fitness:.1f} -> best {self.fitness:.1f} "
            f"({self.improvement:+.1f} points) over {len(self.generations)} generations, "
            f"{self.matches:,} matches",
            strategy_params.describe(self.refs, self.vector),
        ]
        return "\n".join(lines)


class AllianceScoreEvaluator:
    """Mean final score of one alliance, over a fixed seed set, for each
    of a batch of candidate strategies.

    The whole batch becomes one `run_all` call rather than one per
    candidate: a generation is ~10 candidates x ~8 seeds, and submitting
    all 80 at once is what keeps every worker busy through the tail of
    each candidate instead of draining the pool ten times.

    `expand_jobs` and `run_all` are used exactly as the SWEEP tab uses
    them -- a candidate reaches the worker as `RobotSpec.strategy` holding
    a `strategy_io` payload dict, which is the same channel an unsaved GUI
    edit already travels on.
    """

    def __init__(
        self, run_fn, *, robots, match, variability, strategies_dir, dt: float,
        target_label: str, alliance: str, seeds: int = 8, base_seed: int = 0,
        parallel: bool = True, max_workers: int | None = None,
    ):
        if seeds > 1 and not getattr(variability, "enabled", False):
            raise ValueError(
                f"seeds={seeds} with a disabled VariabilityModel: every seed would run the "
                "identical match, so the search would be reading one sample's noise as signal. "
                "Pass an enabled VariabilityModel, or seeds=1 if that is genuinely what you want."
            )
        if not any(r.label == target_label for r in robots):
            raise ValueError(
                f"no robot labelled {target_label!r} in the roster "
                f"({', '.join(r.label for r in robots)}) -- nothing to attach a candidate strategy to"
            )
        self.run_fn = run_fn
        self.robots = list(robots)
        self.match = match
        self.variability = variability
        self.strategies_dir = str(strategies_dir)
        self.dt = float(dt)
        self.target_label = target_label
        self.alliance = alliance
        self.seeds = int(seeds)
        self.base_seed = int(base_seed)
        self.parallel = parallel
        self.max_workers = max_workers
        self.matches_run = 0
        self.failures = 0

    def jobs_for(self, payload: dict) -> list:
        """The seed set for one candidate. Every candidate gets
        `base_seed .. base_seed + seeds - 1` -- identical across
        candidates, which is the whole point (see the module docstring).
        `TrialJob.index` therefore repeats across candidates; results are
        matched back positionally, which is what `run_all` guarantees."""
        robots = [
            apply_variable(r, "strategy", payload) if r.label == self.target_label else r
            for r in self.robots
        ]
        return expand_jobs(
            robots, self.match, self.variability, [], repetitions=self.seeds,
            base_seed=self.base_seed, strategies_dir=self.strategies_dir, dt=self.dt,
        )

    def __call__(self, payloads) -> list:
        payloads = list(payloads)
        jobs = []
        for payload in payloads:
            jobs.extend(self.jobs_for(payload))
        outcomes = run_all(self.run_fn, jobs, parallel=self.parallel, max_workers=self.max_workers)
        self.matches_run += len(outcomes)

        fitnesses = []
        for i in range(len(payloads)):
            chunk = outcomes[i * self.seeds: (i + 1) * self.seeds]
            scores = [
                o.metrics.final_scores.get(self.alliance, 0.0)
                for o in chunk if o.error is None and o.metrics is not None
            ]
            self.failures += len(chunk) - len(scores)
            # A candidate that failed on *some* seeds is still scored on
            # the ones that ran: dropping it entirely would let a single
            # flaky trial discard a good strategy, and the seed set is
            # small enough that losing one is a widening of the error bar,
            # not a change of answer.
            fitnesses.append(sum(scores) / len(scores) if scores else FAILED)
        return fitnesses


@dataclass(frozen=True)
class Confirmation:
    """The winner's score on seeds the search never saw."""
    baseline: float
    tuned: float
    seeds: int

    @property
    def improvement(self) -> float:
        return self.tuned - self.baseline

    def summary(self) -> str:
        return (f"held out {self.seeds} fresh seeds: baseline {self.baseline:.1f} -> "
                f"tuned {self.tuned:.1f} ({self.improvement:+.1f} points)")


def confirm(result: SearchResult, evaluate) -> Confirmation:
    """Re-score the baseline and the winner on a *different* seed set.

    This is not a formality, and skipping it is the easiest way to
    overstate what the search found. `SearchResult.fitness` is a maximum
    over every candidate ever evaluated, all of them on the same handful
    of seeds -- so it carries the selection bias of a best-of-N, and part
    of what it measures is how well the winner fits those particular
    piece scatters and start poses rather than how good the strategy is.
    The gap between the two numbers is the honest size of that bias, and
    on a small seed set it can be most of the reported gain.

    `evaluate` must be an evaluator built over seeds disjoint from the
    search's (a different `base_seed`); nothing here can check that, so
    it is the caller's job -- `apps/run_param_search.py` offsets by the
    seed count it used.
    """
    baseline, tuned = evaluate([result.baseline_payload, result.payload])
    return Confirmation(baseline=baseline, tuned=tuned, seeds=getattr(evaluate, "seeds", 0))


def search_parameters(
    payload: dict, evaluate, *, generations: int = 20, sigma: float = 0.25,
    population_size: int | None = None, seed: int = 0, overrides: dict | None = None,
    progress=None,
) -> SearchResult:
    """Improve `payload`'s continuous parameters, leaving its structure
    alone.

    `evaluate(payloads) -> list[float]` is *maximized* -- a mean score,
    the way the rest of the project reports one. CMA-ES minimizes, so the
    sign is flipped here rather than making every caller remember to.

    The search runs in a normalized [0, 1] box, one axis per `ParamRef`,
    so a single `sigma` is meaningful across parameters measured in
    seconds and parameters measured in inches. In raw units it would not
    be: a step size sensible for a `cooldown` is invisible on an
    `engage_range`, and CMA-ES would spend its first several generations
    just learning the scales.
    """
    refs = strategy_params.continuous_params(payload, overrides=overrides)
    if not refs:
        raise ValueError(
            "this strategy has no searchable continuous parameters -- every candidate would be "
            "identical. Check that its timings and ranges are numbers rather than null; an "
            "unset optional (max_range: null) is structure, not a parameter (see strategy_params)."
        )

    lower = np.array([r.lower for r in refs], dtype=float)
    span = np.array([r.span for r in refs], dtype=float)
    seed_vector = np.array(strategy_params.to_vector(refs), dtype=float)

    def denormalize(z):
        return lower + np.asarray(z, dtype=float) * span

    def payload_for(z):
        return strategy_params.with_vector(payload, refs, denormalize(z))

    z0 = (seed_vector - lower) / span
    optimizer = CMAES(z0, sigma, lower=0.0, upper=1.0, population_size=population_size, seed=seed)

    # The baseline is evaluated on the same seeds as every candidate, so
    # "+7 points" is a paired comparison rather than two numbers from two
    # different sets of matches.
    baseline_fitness = evaluate([payload])[0]
    best_z, best_fitness = z0.copy(), baseline_fitness

    log: list = []
    for index in range(1, generations + 1):
        started = time.perf_counter()
        candidates = optimizer.ask()
        fitnesses = evaluate([payload_for(z) for z in candidates])

        finite = [f for f in fitnesses if f != FAILED]
        if not finite:
            raise RuntimeError(
                f"generation {index}: every candidate failed to evaluate. The search cannot "
                "distinguish a bad strategy from a broken one -- run a single trial by hand "
                "and read its traceback (TrialOutcome.error)."
            )

        # CMA-ES minimizes; a failed candidate becomes +inf, which sorts
        # last and so is never recombined into the mean.
        optimizer.tell(candidates, [-f if f != FAILED else float("inf") for f in fitnesses])

        generation_best = max(finite)
        if generation_best > best_fitness:
            best_fitness = generation_best
            best_z = candidates[fitnesses.index(generation_best)].copy()

        record = Generation(
            index=index, best=generation_best, mean=sum(finite) / len(finite),
            best_so_far=best_fitness, sigma=optimizer.sigma,
            failures=len(fitnesses) - len(finite), seconds=time.perf_counter() - started,
        )
        log.append(record)
        if progress is not None:
            progress(record)

        stop = optimizer.should_stop()
        if stop is not None:
            if progress is not None:
                progress(f"stopping after generation {index}: {stop}")
            break

    best_values = denormalize(best_z)
    return SearchResult(
        payload=strategy_params.with_vector(payload, refs, best_values),
        fitness=best_fitness,
        baseline_payload=payload,
        baseline_fitness=baseline_fitness,
        refs=refs,
        vector=tuple(float(v) for v in best_values),
        generations=tuple(log),
        matches=getattr(evaluate, "matches_run", 0),
    )
