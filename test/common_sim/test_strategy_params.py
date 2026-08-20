"""The genome view onto a strategy. What these mostly pin is what is
*not* searchable -- the exclusions are the whole safety argument for
letting an optimizer edit a strategy file, so each one is a test rather
than a comment."""
from __future__ import annotations

import pytest

from common_sim.control import strategy_io, strategy_params
from common_sim.control.strategy import Rule, Strategy
from common_sim.control.tactics import Collect, Defend, Idle, Score
from common_sim.control.triggers import AllOf, Always, AtCapacity, BeingDefended, PiecesHeld


def _payload(strategy: Strategy) -> dict:
    return strategy_io.to_dict(strategy)


def _cycle_strategy() -> Strategy:
    return Strategy(name="cycle", rules=[
        Rule(name="score", trigger=AtCapacity(piece_type="coral"), tactic=Score(),
             priority=10, min_duration=0.5, cooldown=0.0),
        Rule(name="collect", trigger=PiecesHeld(piece_type="coral", max_count=0),
             tactic=Collect(piece_type="coral", cluster_radius=24.0),
             priority=5, min_duration=0.0, cooldown=0.0),
    ])


def _paths(refs) -> list:
    return [r.path for r in refs]


def test_finds_rule_timings_and_float_tactic_params():
    refs = strategy_params.continuous_params(_payload(_cycle_strategy()))
    assert _paths(refs) == [
        "rules[0].min_duration", "rules[0].cooldown",
        "rules[1].min_duration", "rules[1].cooldown",
        "rules[1].tactic.cluster_radius",
    ]
    assert strategy_params.to_vector(refs) == [0.5, 0.0, 0.0, 0.0, 24.0]


def test_an_unset_optional_stays_unset():
    """`max_range: null` means "no range limit". Giving it a number adds
    a constraint the author did not write -- that is structure, and it
    belongs to a structure search."""
    refs = strategy_params.continuous_params(_payload(_cycle_strategy()))
    assert not any("max_range" in p for p in _paths(refs))


def test_priority_and_once_are_not_searchable():
    """`once` is a bool, and a bool is an int in Python -- if it leaked
    into the vector an optimizer would set it to 0.4."""
    refs = strategy_params.continuous_params(_payload(_cycle_strategy()))
    assert not any(p.endswith(".priority") or p.endswith(".once") for p in _paths(refs))


def test_integer_counts_are_not_searchable():
    refs = strategy_params.continuous_params(_payload(_cycle_strategy()))
    assert not any("max_count" in p or "min_count" in p for p in _paths(refs))


def test_finds_the_float_params_of_a_defend_tactic():
    strategy = Strategy(name="defense", rules=[
        Rule(name="defend", trigger=Always(), tactic=Defend(standoff=24.0, engage_range=200.0),
             priority=100, min_duration=1.0),
    ])
    refs = strategy_params.continuous_params(_payload(strategy))
    by_path = {r.path: r for r in refs}
    assert by_path["rules[0].tactic.standoff"].value == 24.0
    assert by_path["rules[0].tactic.engage_range"].value == 200.0
    # engage_range's seed sits above the generic " in" default upper, and
    # a box that excludes its own starting point would silently rewrite
    # the strategy on generation zero.
    assert by_path["rules[0].tactic.engage_range"].upper >= 200.0


def test_an_outermost_for_duration_is_searchable():
    strategy = Strategy(name="reactive", rules=[
        Rule(name="evade", trigger=BeingDefended(within=48.0, for_duration=1.5),
             tactic=Idle(), priority=1),
    ])
    refs = strategy_params.continuous_params(_payload(strategy))
    by_path = {r.path: r for r in refs}
    assert by_path["rules[0].trigger.for_duration"].value == 1.5
    assert by_path["rules[0].trigger.within"].value == 48.0


def test_a_nested_for_duration_is_left_alone():
    """A `for_duration` inside an AllOf is never applied, and
    `strategy._reject_nested_for_duration` refuses to build a controller
    for a non-zero one. Raising a nested 0.0 would turn a working
    strategy into one that fails every trial."""
    strategy = Strategy(name="nested", rules=[
        Rule(name="evade", tactic=Idle(), priority=1, trigger=AllOf(triggers=(
            BeingDefended(within=48.0), Always(),
        ))),
    ])
    payload = _payload(strategy)
    payload["rules"][0]["trigger"]["triggers"][0]["for_duration"] = 0.0

    refs = strategy_params.continuous_params(payload)
    assert "rules[0].trigger.triggers[0].within" in _paths(refs)
    assert not any("for_duration" in p for p in _paths(refs))


def test_nested_trigger_params_are_still_searchable():
    strategy = Strategy(name="nested", rules=[
        Rule(name="evade", tactic=Idle(), priority=1, trigger=AllOf(triggers=(
            BeingDefended(within=48.0), AtCapacity(piece_type="coral"),
        ))),
    ])
    refs = strategy_params.continuous_params(_payload(strategy))
    assert "rules[0].trigger.triggers[0].within" in _paths(refs)


def test_with_vector_writes_values_without_touching_structure():
    payload = _payload(_cycle_strategy())
    refs = strategy_params.continuous_params(payload)
    updated = strategy_params.with_vector(payload, refs, [1.25, 2.0, 0.0, 3.0, 30.0])

    assert updated["rules"][0]["min_duration"] == 1.25
    assert updated["rules"][1]["tactic"]["cluster_radius"] == 30.0
    # Structure identical: same rules, order, types, priorities.
    assert [r["name"] for r in updated["rules"]] == [r["name"] for r in payload["rules"]]
    assert [r["priority"] for r in updated["rules"]] == [r["priority"] for r in payload["rules"]]
    assert [r["tactic"]["type"] for r in updated["rules"]] == [r["tactic"]["type"] for r in payload["rules"]]
    assert updated["rules"][1]["tactic"]["max_range"] is None


def test_with_vector_does_not_mutate_the_seed_payload():
    """A search builds a whole population from one seed payload every
    generation; an in-place write would make each generation a
    perturbation of the previous one's last candidate."""
    payload = _payload(_cycle_strategy())
    refs = strategy_params.continuous_params(payload)
    strategy_params.with_vector(payload, refs, [9.0] * len(refs))
    assert payload["rules"][0]["min_duration"] == 0.5


def test_the_result_still_loads_as_a_strategy():
    """The output has to be an ordinary strategy file -- that is what
    makes the search's answer readable by a drive team rather than a model
    that needs distilling."""
    payload = _payload(_cycle_strategy())
    refs = strategy_params.continuous_params(payload)
    tuned = strategy_params.with_vector(payload, refs, [1.0, 2.0, 0.5, 1.5, 36.0])

    strategy = strategy_io.from_dict(tuned)
    assert isinstance(strategy, Strategy)
    assert [r.name for r in strategy.rules] == ["score", "collect"]
    assert strategy.rules[1].tactic.cluster_radius == 36.0
    assert strategy.rules[0].min_duration == 1.0


def test_with_vector_rejects_a_wrong_length_vector():
    payload = _payload(_cycle_strategy())
    refs = strategy_params.continuous_params(payload)
    with pytest.raises(ValueError):
        strategy_params.with_vector(payload, refs, [1.0])


def test_bounds_can_be_overridden_per_path():
    payload = _payload(_cycle_strategy())
    refs = strategy_params.continuous_params(
        payload, overrides={"rules[1].tactic.cluster_radius": (6.0, 60.0)})
    ref = next(r for r in refs if r.path == "rules[1].tactic.cluster_radius")
    assert (ref.lower, ref.upper) == (6.0, 60.0)


def test_a_payload_that_is_not_a_strategy_is_rejected():
    with pytest.raises(ValueError):
        strategy_params.continuous_params({"type": "Collect", "cluster_radius": 24.0})


def test_every_shipped_strategy_exposes_a_searchable_space():
    """A regression guard on the whole pipeline: if a schema change makes
    one of these unsearchable, the search silently has nothing to do."""
    from pathlib import Path
    strategies_dir = (Path(__file__).resolve().parents[2]
                      / "game_specific" / "reefscape" / "strategies")
    for path in sorted(strategies_dir.glob("*.json")):
        payload = strategy_io.to_dict(strategy_io.load_strategy(path))
        refs = strategy_params.continuous_params(payload)
        assert refs, f"{path.name} has no searchable parameters"
        # Every ref's seed value must sit inside its own box.
        for ref in refs:
            assert ref.lower <= ref.value <= ref.upper, f"{path.name}: {ref.path} starts out of bounds"
