"""The hall-of-fame archive and evaluator, with a fake worker.

Same spirit as test_param_search.py: nothing here runs a match. The
point is the plumbing that decides which opponents a candidate faces,
how the payoff matrix is built, and where the archive's persistence and
capping earn their keep.
"""
from __future__ import annotations

import json
from dataclasses import dataclass

import pytest

from common_sim.analysis import param_search
from common_sim.analysis.hall_of_fame import (
    Archive, HallOfFameEvaluator, Payoff, describe_payoffs, exploitability,
)
from common_sim.analysis.sweep_spec import MatchSpec, RobotSpec, TrialOutcome
from common_sim.analysis.variability import VariabilityModel

MATCH = MatchSpec(auto_duration=15.0, teleop_duration=135.0)
ON = VariabilityModel(enabled=True, max_speed_pct=0.05)


@dataclass(frozen=True)
class _Metrics:
    final_scores: dict


def _roster():
    char = {"max_speed": 150.0}
    return [
        RobotSpec(label="PRIMARY", alliance="blue", roster_index=-1, characteristics=dict(char), strategy="cycle"),
        RobotSpec(label="OPPONENT", alliance="red", roster_index=0, characteristics=dict(char), strategy="cycle"),
    ]


def _outcome(job, blue, red):
    return TrialOutcome(index=job.index, seed=job.seed, params=job.params,
                        metrics=_Metrics(final_scores={"blue": blue, "red": red}))


def _archive(*names) -> Archive:
    a = Archive()
    for i, name in enumerate(names):
        a = a.add(name, {"name": name}, fitness=float(i))
    return a


def _evaluator(run_fn, **kw):
    kw.setdefault("seeds", 2)
    kw.setdefault("sample_size", 1)
    kw.setdefault("hand_written", {"full_defense": {"name": "full_defense"}})
    kw.setdefault("archive", _archive("gen3_winner", "gen5_winner"))
    return HallOfFameEvaluator(
        run_fn, robots=_roster(), match=MATCH, variability=ON, strategies_dir="/strategies",
        dt=1 / 30, target_label="PRIMARY", opponent_label="OPPONENT",
        alliance="blue", opponent_alliance="red", parallel=False, **kw,
    )


# -- Archive --------------------------------------------------------------

def test_add_keeps_strongest_when_capped():
    a = Archive()
    a = a.add("weak", {}, fitness=1.0, max_size=2)
    a = a.add("strong", {}, fitness=9.0, max_size=2)
    a = a.add("medium", {}, fitness=5.0, max_size=2)
    assert {e.name for e in a.entries} == {"strong", "medium"}


def test_add_without_a_cap_keeps_everything():
    a = Archive().add("a", {}, 1.0).add("b", {}, 2.0)
    assert len(a) == 2


def test_sample_returns_everything_when_k_exceeds_size():
    a = _archive("x", "y")
    import random
    assert set(e.name for e in a.sample(10, random.Random(0))) == {"x", "y"}


def test_sample_draws_k_when_archive_is_larger():
    a = _archive("a", "b", "c", "d")
    import random
    sampled = a.sample(2, random.Random(0))
    assert len(sampled) == 2
    assert set(e.name for e in sampled) <= {"a", "b", "c", "d"}


def test_save_load_roundtrip(tmp_path):
    a = _archive("winner").add("second", {"rules": []}, fitness=42.5)
    path = tmp_path / "archive.json"
    a.save(path)
    loaded = Archive.load(path)
    assert loaded.entries == a.entries


def test_load_of_a_missing_file_is_an_empty_archive(tmp_path):
    assert Archive.load(tmp_path / "nope.json") == Archive()


# -- exploitability ---------------------------------------------------------

def test_exploitability_is_the_worst_margin():
    payoffs = [
        Payoff("a", candidate_score=50.0, opponent_score=40.0),   # candidate wins, margin -10
        Payoff("b", candidate_score=30.0, opponent_score=45.0),   # opponent wins, margin +15
    ]
    assert exploitability(payoffs) == pytest.approx(15.0)


def test_exploitability_is_floored_at_zero_when_candidate_wins_everything():
    payoffs = [Payoff("a", candidate_score=50.0, opponent_score=10.0)]
    assert exploitability(payoffs) == 0.0


def test_exploitability_of_an_empty_field_is_zero():
    assert exploitability([]) == 0.0


def test_describe_payoffs_names_every_opponent_and_the_exploitability():
    payoffs = [
        Payoff("full_defense", candidate_score=50.0, opponent_score=40.0),
        Payoff("gen3_winner", candidate_score=30.0, opponent_score=45.0),
    ]
    text = describe_payoffs(payoffs)
    assert "full_defense" in text and "gen3_winner" in text
    assert "exploitability: 15.0" in text


# -- HallOfFameEvaluator ----------------------------------------------------

def test_field_is_the_archive_sample_plus_every_hand_written_opponent():
    seen = set()

    def run_fn(job):
        seen.add(job.params["opponent"])
        return _outcome(job, 100.0, 50.0)

    _evaluator(run_fn, sample_size=2)([{"name": "candidate"}])

    assert "full_defense" in seen
    assert seen <= {"full_defense", "gen3_winner", "gen5_winner"}
    assert len(seen) == 3  # sample_size=2 archive entries + 1 hand-written, archive has exactly 2


def test_the_field_is_common_across_every_candidate_in_one_call():
    """Same argument as param_search's common random numbers, applied to
    the opponent axis: every candidate in a generation must face the
    same field, or the ranking partly measures who got the easier draw."""
    seen_by_payload: dict = {}

    def run_fn(job):
        seen_by_payload.setdefault(job.robots[0].strategy["name"], set()).add(job.params["opponent"])
        return _outcome(job, 100.0, 50.0)

    evaluator = _evaluator(run_fn, sample_size=1)
    evaluator([{"name": "a"}, {"name": "b"}])

    fields = list(seen_by_payload.values())
    assert fields[0] == fields[1]


def test_candidate_score_is_the_mean_across_opponents_not_across_seeds():
    def run_fn(job):
        # full_defense is generous, the archive winner is not
        blue = 80.0 if job.params["opponent"] == "full_defense" else 20.0
        return _outcome(job, blue, 10.0)

    evaluator = _evaluator(run_fn, sample_size=1, archive=_archive("hard_opponent"))
    fitness = evaluator([{"name": "candidate"}])[0]
    assert fitness == pytest.approx(50.0)  # mean of 80 and 20, not weighted by seed count


def test_last_payoffs_records_one_row_per_opponent():
    def run_fn(job):
        blue = 80.0 if job.params["opponent"] == "full_defense" else 20.0
        return _outcome(job, blue, 10.0)

    evaluator = _evaluator(run_fn, sample_size=1, archive=_archive("hard_opponent"))
    evaluator([{"name": "candidate"}])

    payoffs = evaluator.last_payoffs[0]
    assert {p.opponent for p in payoffs} == {"full_defense", "hard_opponent"}
    assert evaluator.last_exploitability[0] == pytest.approx(0.0)  # candidate always outscores 10


def test_a_wholly_failed_opponent_row_is_dropped_not_the_whole_candidate():
    def run_fn(job):
        if job.params["opponent"] == "full_defense":
            return TrialOutcome(index=job.index, seed=job.seed, params=job.params,
                                metrics=None, error="boom")
        return _outcome(job, 50.0, 10.0)

    evaluator = _evaluator(run_fn, sample_size=1, archive=_archive("ok_opponent"))
    fitness = evaluator([{"name": "candidate"}])[0]
    assert fitness == pytest.approx(50.0)
    assert {p.opponent for p in evaluator.last_payoffs[0]} == {"ok_opponent"}


def test_a_wholly_failed_candidate_is_not_a_finite_score():
    def run_fn(job):
        return TrialOutcome(index=job.index, seed=job.seed, params=job.params,
                            metrics=None, error="boom")

    assert _evaluator(run_fn)([{"name": "candidate"}]) == [param_search.FAILED]


def test_multiple_seeds_with_variability_off_is_refused():
    with pytest.raises(ValueError, match="identical match"):
        HallOfFameEvaluator(
            lambda job: None, robots=_roster(), match=MATCH, variability=VariabilityModel(),
            strategies_dir="/s", dt=1 / 30, target_label="PRIMARY", opponent_label="OPPONENT",
            alliance="blue", opponent_alliance="red", archive=_archive("a"), hand_written={}, seeds=4)


def test_a_missing_target_robot_is_caught_before_anything_runs():
    with pytest.raises(ValueError, match="no robot labelled 'TYPO'"):
        HallOfFameEvaluator(
            lambda job: None, robots=_roster(), match=MATCH, variability=ON,
            strategies_dir="/s", dt=1 / 30, target_label="TYPO", opponent_label="OPPONENT",
            alliance="blue", opponent_alliance="red", archive=_archive("a"), hand_written={}, seeds=1)


def test_a_missing_opponent_robot_is_caught_before_anything_runs():
    with pytest.raises(ValueError, match="no robot labelled 'TYPO'"):
        HallOfFameEvaluator(
            lambda job: None, robots=_roster(), match=MATCH, variability=ON,
            strategies_dir="/s", dt=1 / 30, target_label="PRIMARY", opponent_label="TYPO",
            alliance="blue", opponent_alliance="red", archive=_archive("a"), hand_written={}, seeds=1)


def test_an_empty_archive_and_no_hand_written_opponents_is_an_error():
    with pytest.raises(ValueError, match="nothing to grade"):
        HallOfFameEvaluator(
            lambda job: None, robots=_roster(), match=MATCH, variability=ON,
            strategies_dir="/s", dt=1 / 30, target_label="PRIMARY", opponent_label="OPPONENT",
            alliance="blue", opponent_alliance="red", archive=Archive(), hand_written={}, seeds=1)


def test_the_opponent_strategy_travels_to_the_worker_not_the_candidate():
    def run_fn(job):
        by_label = {r.label: r for r in job.robots}
        assert by_label["PRIMARY"].strategy == {"name": "candidate"}
        assert by_label["OPPONENT"].strategy != {"name": "candidate"}
        return _outcome(job, 50.0, 10.0)

    _evaluator(run_fn, sample_size=1)([{"name": "candidate"}])
