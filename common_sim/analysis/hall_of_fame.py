"""
Phase 3 of the development path: grade a candidate against an archive of
strategies instead of one fixed opponent, and report exploitability
alongside score.

Why this exists, stated once so callers do not have to re-derive it: a
strategy graded against a fixed opponent set discovers *counters to that
set*, and the score cannot tell you that is what happened. Phase 2's
`AllianceScoreEvaluator` grades a candidate against one hand-written
opponent (`full_defense`, say) -- exactly this trap, just not yet
expensive enough to notice. `apps/run_defense_bench.py` already works
around a version of the same problem with a red x blue payoff grid; this
module generalizes that into a payoff matrix that grows as the search
finds new strategies, plus the one number that grid was really for: how
much the single best counter in the field beats a candidate by.

The artifact this implements is explicit that this has to land *before*
the structure search (Phase 5) -- searching rule structure against a
fixed opponent optimizes against a fiction, and a structure search costs
enough that discovering that after the fact is expensive.

Two pieces:

* `Archive` -- plain, immutable, JSON-round-trippable. Holds the
  strongest strategies found so far, keyed by name. `sample()` draws a
  subset to keep evaluation cost bounded (5-10 opponents per candidate,
  per the artifact, not the whole archive).
* `HallOfFameEvaluator` -- same shape and contract as
  `param_search.AllianceScoreEvaluator` (`__call__(payloads) ->
  list[float]`, so it drops into `search_parameters` unchanged), except
  the opponent robot's strategy varies across a sampled field instead of
  staying fixed. Reports a payoff matrix and exploitability per candidate
  as a side channel (`last_payoffs`), since CMA-ES only wants the float.
"""
from __future__ import annotations

import json
import random
from dataclasses import dataclass
from pathlib import Path

from common_sim.analysis.param_search import FAILED
from common_sim.analysis.runner import run_all
from common_sim.analysis.sweep_spec import TrialJob, apply_variable


@dataclass(frozen=True)
class ArchiveEntry:
    name: str
    payload: dict
    fitness: float = 0.0


@dataclass(frozen=True)
class Archive:
    """Immutable so a `search_parameters` run can be handed a snapshot
    without worrying about it changing mid-search; the caller re-assigns
    the name after `add`, the way a frozen dataclass's `replace` does."""

    entries: tuple = ()

    @staticmethod
    def load(path) -> "Archive":
        p = Path(path)
        if not p.exists():
            return Archive()
        data = json.loads(p.read_text(encoding="utf-8"))
        return Archive(tuple(ArchiveEntry(**e) for e in data))

    def save(self, path) -> None:
        payload = [
            {"name": e.name, "payload": e.payload, "fitness": e.fitness}
            for e in self.entries
        ]
        Path(path).write_text(json.dumps(payload, indent=2), encoding="utf-8")

    def add(self, name: str, payload: dict, fitness: float, *, max_size: int | None = None) -> "Archive":
        """Keep the strongest, not the newest -- once `max_size` entries
        exist, adding a candidate that scored worse than everything
        already archived is a no-op in all but name; adding a strong one
        evicts the current weakest instead of the oldest."""
        entries = self.entries + (ArchiveEntry(name, payload, fitness),)
        if max_size is not None and len(entries) > max_size:
            entries = tuple(sorted(entries, key=lambda e: e.fitness, reverse=True)[:max_size])
        return Archive(entries)

    def sample(self, k: int, rng: random.Random) -> tuple:
        if k >= len(self.entries):
            return self.entries
        return tuple(rng.sample(self.entries, k))

    def __len__(self) -> int:
        return len(self.entries)


@dataclass(frozen=True)
class Payoff:
    """One row of the payoff matrix: one opponent, this candidate's mean
    score and the opponent's mean score against it, over the seed set."""

    opponent: str
    candidate_score: float
    opponent_score: float

    @property
    def margin(self) -> float:
        """Positive: the opponent outscored the candidate by this many
        points. This is the honest number, not the friendly one -- a
        candidate that wins every matchup in the sample has every margin
        negative, which is what "unexploited, in this field" looks like."""
        return self.opponent_score - self.candidate_score


def exploitability(payoffs) -> float:
    """How much the single best counter in the field beats a strategy
    by. Floored at 0: a strategy that wins every sampled matchup is not
    "negatively exploitable", it is unexploited by this field -- 0 is the
    honest report, not "the field couldn't find anything", which a raw
    negative number would misleadingly claim."""
    if not payoffs:
        return 0.0
    return max(0.0, max(p.margin for p in payoffs))


class HallOfFameEvaluator:
    """Mean score of a candidate against a sampled field -- some of the
    hall of fame, plus a fixed hand-written set -- instead of one
    opponent. `__call__` returns `list[float]` like
    `param_search.AllianceScoreEvaluator`, so it is a drop-in `evaluate`
    for `search_parameters`; the payoff matrix and exploitability for the
    most recent call live in `last_payoffs` / `last_exploitability` for a
    caller that wants to report them (CMA-ES itself only wants the float).

    The field is re-sampled once per `__call__`, not once per candidate:
    every candidate in a generation is judged against the same opponents,
    which is `param_search`'s common-random-numbers argument applied to
    the opponent axis instead of the seed axis. It reshuffles across
    generations, so the search cannot overfit to one lucky sample of the
    archive the way it could overfit to one lucky seed set.
    """

    def __init__(
        self, run_fn, *, robots, match, variability, strategies_dir, dt: float,
        target_label: str, opponent_label: str, alliance: str, opponent_alliance: str,
        archive: Archive, hand_written: dict, sample_size: int = 4,
        seeds: int = 4, base_seed: int = 0, rng_seed: int = 0,
        parallel: bool = True, max_workers: int | None = None,
    ):
        if seeds > 1 and not getattr(variability, "enabled", False):
            raise ValueError(
                f"seeds={seeds} with a disabled VariabilityModel: every seed would run the "
                "identical match, so the search would be reading one sample's noise as signal. "
                "Pass an enabled VariabilityModel, or seeds=1 if that is genuinely what you want."
            )
        labels = {r.label for r in robots}
        if target_label not in labels:
            raise ValueError(f"no robot labelled {target_label!r} in the roster ({', '.join(labels)})")
        if opponent_label not in labels:
            raise ValueError(f"no robot labelled {opponent_label!r} in the roster ({', '.join(labels)})")
        if not archive and not hand_written:
            raise ValueError(
                "empty archive and no hand-written opponents -- nothing to grade a candidate against"
            )
        self.run_fn = run_fn
        self.robots = list(robots)
        self.match = match
        self.variability = variability
        self.strategies_dir = str(strategies_dir)
        self.dt = float(dt)
        self.target_label = target_label
        self.opponent_label = opponent_label
        self.alliance = alliance
        self.opponent_alliance = opponent_alliance
        self.archive = archive
        self.hand_written = dict(hand_written)
        self.sample_size = int(sample_size)
        self.seeds = int(seeds)
        self.base_seed = int(base_seed)
        self._rng = random.Random(rng_seed)
        self.parallel = parallel
        self.max_workers = max_workers
        self.matches_run = 0
        self.failures = 0
        self.last_payoffs: list[tuple] = []
        self.last_exploitability: list[float] = []

    def _field(self) -> list:
        """Archive sample plus every hand-written opponent, named."""
        sampled = [(e.name, e.payload) for e in self.archive.sample(self.sample_size, self._rng)]
        return sampled + list(self.hand_written.items())

    def _jobs_for(self, payload: dict, field: list) -> list:
        jobs = []
        index = 0
        for name, opponent_payload in field:
            robots = tuple(
                apply_variable(r, "strategy", payload) if r.label == self.target_label
                else apply_variable(r, "strategy", opponent_payload) if r.label == self.opponent_label
                else r
                for r in self.robots
            )
            for _ in range(self.seeds):
                jobs.append(TrialJob(
                    index=index, seed=self.base_seed + index, params={"opponent": name},
                    robots=robots, match=self.match, variability=self.variability,
                    strategies_dir=self.strategies_dir, dt=self.dt,
                ))
                index += 1
        return jobs

    def __call__(self, payloads) -> list:
        payloads = list(payloads)
        field = self._field()
        if not field:
            raise ValueError("empty archive and no hand-written opponents -- nothing to grade against")

        jobs_by_payload = [self._jobs_for(payload, field) for payload in payloads]
        jobs = [job for group in jobs_by_payload for job in group]
        outcomes = run_all(self.run_fn, jobs, parallel=self.parallel, max_workers=self.max_workers)
        self.matches_run += len(outcomes)

        fitnesses = []
        self.last_payoffs = []
        self.last_exploitability = []
        offset = 0
        for group in jobs_by_payload:
            chunk = outcomes[offset: offset + len(group)]
            offset += len(group)
            payoffs = self._reduce(chunk, field)
            self.last_payoffs.append(payoffs)
            if payoffs:
                fitnesses.append(sum(p.candidate_score for p in payoffs) / len(payoffs))
                self.last_exploitability.append(exploitability(payoffs))
            else:
                fitnesses.append(FAILED)
                self.last_exploitability.append(FAILED)
        return fitnesses

    def _reduce(self, chunk, field) -> tuple:
        by_opponent: dict = {name: [] for name, _ in field}
        for outcome in chunk:
            if outcome.error is not None or outcome.metrics is None:
                self.failures += 1
                continue
            by_opponent[outcome.params["opponent"]].append(outcome.metrics)

        payoffs = []
        for name, metrics in by_opponent.items():
            if not metrics:
                # every seed against this opponent failed -- drop the row
                # rather than the whole candidate, same reasoning as
                # AllianceScoreEvaluator's partial-failure handling.
                continue
            candidate = sum(m.final_scores.get(self.alliance, 0.0) for m in metrics) / len(metrics)
            opponent = sum(m.final_scores.get(self.opponent_alliance, 0.0) for m in metrics) / len(metrics)
            payoffs.append(Payoff(opponent=name, candidate_score=candidate, opponent_score=opponent))
        return tuple(payoffs)


def describe_payoffs(payoffs) -> str:
    """One line per opponent, worst matchup last -- the payoff matrix a
    report actually wants to read, not a dict dump."""
    ordered = sorted(payoffs, key=lambda p: p.margin)
    lines = [
        f"  {p.opponent:<24} candidate {p.candidate_score:7.1f}  opponent {p.opponent_score:7.1f}  "
        f"margin {p.margin:+7.1f}"
        for p in ordered
    ]
    lines.append(f"  exploitability: {exploitability(payoffs):.1f}")
    return "\n".join(lines)
