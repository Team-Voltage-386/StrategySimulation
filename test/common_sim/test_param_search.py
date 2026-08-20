"""The search loop and its evaluator, with a fake worker.

Nothing here runs a match. A real trial costs seconds and the point of
these tests is the plumbing that decides *what* gets run and how its
numbers come back -- especially the two ways to get an answer that looks
fine and means nothing: unpaired seeds, and a variability model that
makes every seed the same match.
"""
from __future__ import annotations

from dataclasses import dataclass

import pytest

from common_sim.analysis import param_search
from common_sim.analysis.param_search import AllianceScoreEvaluator, search_parameters
from common_sim.analysis.sweep_spec import MatchSpec, RobotSpec, TrialOutcome
from common_sim.analysis.variability import VariabilityModel
from common_sim.control import strategy_io, strategy_params
from common_sim.control.strategy import Rule, Strategy
from common_sim.control.tactics import Collect
from common_sim.control.triggers import PiecesHeld

MATCH = MatchSpec(auto_duration=15.0, teleop_duration=135.0)
ON = VariabilityModel(enabled=True, max_speed_pct=0.05)


@dataclass(frozen=True)
class _Metrics:
    final_scores: dict


def _payload() -> dict:
    return strategy_io.to_dict(Strategy(name="cycle", rules=[
        Rule(name="collect", trigger=PiecesHeld(piece_type="coral", max_count=0),
             tactic=Collect(piece_type="coral", cluster_radius=24.0),
             priority=5, min_duration=0.0, cooldown=0.0),
    ]))


def _roster():
    char = {"max_speed": 150.0}
    return [
        RobotSpec(label="PRIMARY", alliance="blue", roster_index=-1, characteristics=dict(char), strategy="cycle"),
        RobotSpec(label="OPPONENT", alliance="red", roster_index=0, characteristics=dict(char), strategy="cycle"),
    ]


def _evaluator(run_fn, **kw):
    kw.setdefault("seeds", 4)
    return AllianceScoreEvaluator(
        run_fn, robots=_roster(), match=MATCH, variability=ON, strategies_dir="/strategies",
        dt=1 / 30, target_label="PRIMARY", alliance="blue", parallel=False, **kw,
    )


def _outcome(job, blue):
    return TrialOutcome(index=job.index, seed=job.seed, params=job.params,
                        metrics=_Metrics(final_scores={"blue": blue, "red": 10.0}))


# -- the evaluator ------------------------------------------------------

def test_every_candidate_is_judged_on_the_same_seeds():
    """Common random numbers. Give each candidate fresh seeds and the
    search spends its budget ranking luck instead of strategies."""
    seen = []

    def run_fn(job):
        seen.append(job.seed)
        return _outcome(job, 100.0)

    evaluate = _evaluator(run_fn)
    evaluate([_payload(), _payload(), _payload()])

    assert len(seen) == 12
    assert seen[0:4] == seen[4:8] == seen[8:12]


def test_the_candidate_reaches_the_worker_as_the_target_robots_strategy():
    payloads = []

    def run_fn(job):
        by_label = {r.label: r for r in job.robots}
        payloads.append(by_label["PRIMARY"].strategy)
        assert by_label["OPPONENT"].strategy == "cycle", "the opponent must not be retuned too"
        return _outcome(job, 100.0)

    candidate = strategy_params.with_vector(
        _payload(), strategy_params.continuous_params(_payload()), [1.5, 2.5, 30.0])
    _evaluator(run_fn)([candidate])

    assert all(p == candidate for p in payloads)
    assert payloads[0]["rules"][0]["tactic"]["cluster_radius"] == 30.0


def test_fitness_is_the_mean_score_of_the_named_alliance():
    scores = iter([10.0, 20.0, 30.0, 40.0])
    evaluate = _evaluator(lambda job: _outcome(job, next(scores)))
    assert evaluate([_payload()]) == [pytest.approx(25.0)]


def test_the_search_timestep_is_carried_onto_every_job():
    dts = []
    _evaluator(lambda job: (dts.append(job.dt), _outcome(job, 1.0))[1])([_payload()])
    assert set(dts) == {1 / 30}


def test_a_partly_failed_candidate_is_scored_on_the_seeds_that_ran():
    results = iter([None, 30.0, 50.0, None])

    def run_fn(job):
        value = next(results)
        if value is None:
            return TrialOutcome(index=job.index, seed=job.seed, params=job.params,
                                metrics=None, error="boom")
        return _outcome(job, value)

    evaluate = _evaluator(run_fn)
    assert evaluate([_payload()]) == [pytest.approx(40.0)]
    assert evaluate.failures == 2


def test_a_wholly_failed_candidate_is_not_a_finite_score():
    """A large finite penalty is a number CMA-ES will interpolate toward.
    "This strategy crashed" is not a point on the landscape."""
    def run_fn(job):
        return TrialOutcome(index=job.index, seed=job.seed, params=job.params,
                            metrics=None, error="boom")

    assert _evaluator(run_fn)([_payload()]) == [param_search.FAILED]


def test_multiple_seeds_with_variability_off_is_refused():
    """With a disabled model nothing in a trial consumes the seed, so N
    seeds is N identical matches -- and the search would read one sample's
    noise as signal. Silent, so it has to be an error."""
    with pytest.raises(ValueError, match="identical match"):
        AllianceScoreEvaluator(
            lambda job: None, robots=_roster(), match=MATCH, variability=VariabilityModel(),
            strategies_dir="/s", dt=1 / 30, target_label="PRIMARY", alliance="blue", seeds=8)


def test_one_seed_with_variability_off_is_allowed():
    AllianceScoreEvaluator(
        lambda job: None, robots=_roster(), match=MATCH, variability=VariabilityModel(),
        strategies_dir="/s", dt=1 / 30, target_label="PRIMARY", alliance="blue", seeds=1)


def test_a_missing_target_robot_is_caught_before_anything_runs():
    with pytest.raises(ValueError, match="no robot labelled"):
        AllianceScoreEvaluator(
            lambda job: None, robots=_roster(), match=MATCH, variability=ON,
            strategies_dir="/s", dt=1 / 30, target_label="TYPO", alliance="blue", seeds=4)


# -- the search loop ----------------------------------------------------

def _synthetic_fitness(ideal):
    """A cheap stand-in for a match: score peaks when the parameters hit
    `ideal` and falls off quadratically. Lets the loop be tested for
    convergence without simulating anything."""
    def evaluate(payloads):
        out = []
        for payload in payloads:
            refs = strategy_params.continuous_params(payload)
            values = strategy_params.to_vector(refs)
            error = sum(((v - t) / max(r.span, 1e-9)) ** 2 for v, t, r in zip(values, ideal, refs))
            out.append(100.0 - 100.0 * error)
        return out
    return evaluate


def test_the_search_improves_on_the_hand_written_values():
    payload = _payload()
    refs = strategy_params.continuous_params(payload)
    ideal = [2.0, 1.0, 40.0][: len(refs)]

    result = search_parameters(payload, _synthetic_fitness(ideal), generations=25, seed=3)

    assert result.fitness > result.baseline_fitness
    assert result.improvement > 0
    # Per-axis, as a fraction of that axis's range -- the search works in
    # a normalized box, so an absolute tolerance would be 100x stricter on
    # `cluster_radius` (0..360 in) than on `min_duration` (0..20 s).
    for value, target, ref in zip(result.vector, ideal, refs):
        assert abs(value - target) < 0.05 * ref.span, ref.path


def test_the_result_is_a_loadable_strategy_with_the_same_structure():
    payload = _payload()
    result = search_parameters(payload, _synthetic_fitness([2.0, 1.0, 40.0]), generations=5, seed=3)

    tuned = strategy_io.from_dict(result.payload)
    assert [r.name for r in tuned.rules] == ["collect"]
    assert [r.priority for r in tuned.rules] == [5]
    assert type(tuned.rules[0].tactic).__name__ == "Collect"


def test_the_baseline_is_measured_not_assumed():
    """The reported gain is a paired comparison, so the hand-written
    strategy has to be evaluated by the same evaluator on the same seeds
    rather than carried in from a previous sweep."""
    seen = []

    def evaluate(payloads):
        seen.append(len(payloads))
        return _synthetic_fitness([2.0, 1.0, 40.0])(payloads)

    result = search_parameters(_payload(), evaluate, generations=2, seed=3)
    assert seen[0] == 1
    assert result.baseline_fitness == pytest.approx(
        _synthetic_fitness([2.0, 1.0, 40.0])([_payload()])[0])


def test_a_strategy_with_nothing_to_search_is_an_error_not_a_no_op():
    payload = strategy_io.to_dict(Strategy(name="empty", rules=[]))
    with pytest.raises(ValueError, match="no searchable continuous parameters"):
        search_parameters(payload, _synthetic_fitness([]), generations=1)


def test_a_generation_that_wholly_fails_stops_the_search():
    with pytest.raises(RuntimeError, match="every candidate failed"):
        search_parameters(_payload(), lambda payloads: [param_search.FAILED] * len(payloads),
                          generations=3)


def test_progress_is_reported_once_per_generation():
    records = []
    search_parameters(_payload(), _synthetic_fitness([2.0, 1.0, 40.0]),
                      generations=4, seed=3, progress=records.append)
    generations = [r for r in records if not isinstance(r, str)]
    assert [r.index for r in generations] == [1, 2, 3, 4]
    assert all(r.best_so_far >= r.best - 1e-9 for r in generations)


def test_the_search_is_reproducible_for_a_fixed_seed():
    fitness = _synthetic_fitness([2.0, 1.0, 40.0])
    a = search_parameters(_payload(), fitness, generations=6, seed=17)
    b = search_parameters(_payload(), fitness, generations=6, seed=17)
    assert a.vector == pytest.approx(b.vector)


def test_confirmation_rescores_both_strategies_on_the_holdout_evaluator():
    """`SearchResult.fitness` is a maximum over every candidate ever
    evaluated, all on the same seeds, so it carries a best-of-N selection
    bias. `confirm` is what separates the real gain from the fit."""
    result = search_parameters(_payload(), _synthetic_fitness([2.0, 1.0, 40.0]),
                               generations=4, seed=3)
    seen = []

    def holdout(payloads):
        seen.append(list(payloads))
        return [10.0, 17.0]

    holdout.seeds = 16
    checked = param_search.confirm(result, holdout)

    assert seen == [[result.baseline_payload, result.payload]]
    assert (checked.baseline, checked.tuned, checked.seeds) == (10.0, 17.0, 16)
    assert checked.improvement == pytest.approx(7.0)
    assert "16 fresh seeds" in checked.summary()


def test_a_confirmation_run_uses_seeds_the_search_never_saw():
    """The offset is the caller's job -- this pins that the evaluator can
    express it, since a confirmation on the search's own seeds would
    measure nothing."""
    seen = []
    search = _evaluator(lambda job: (seen.append(job.seed), _outcome(job, 1.0))[1], seeds=4)
    holdout = _evaluator(lambda job: (seen.append(job.seed), _outcome(job, 1.0))[1],
                         seeds=4, base_seed=4)
    search([_payload()])
    search_seeds = set(seen)
    seen.clear()
    holdout([_payload()])
    assert search_seeds.isdisjoint(seen)


def test_the_summary_names_the_parameters_it_moved():
    result = search_parameters(_payload(), _synthetic_fitness([2.0, 1.0, 40.0]),
                               generations=3, seed=3)
    summary = result.summary()
    assert "cluster_radius" in summary
    assert "baseline" in summary
