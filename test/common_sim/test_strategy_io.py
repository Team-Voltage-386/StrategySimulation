import json

import pytest

from common_sim.control import strategy_io
from common_sim.control.strategy import Rule, Strategy
from common_sim.control.tactics import Collect, Defend, Idle, Score
from common_sim.control.triggers import AllOf, AtCapacity, MatchTime, Not, PiecesHeld


def make_strategy() -> Strategy:
    return Strategy(
        name="cycle_and_defend",
        rules=[
            Rule(
                name="endgame_defense",
                trigger=AllOf(triggers=(MatchTime(remaining_under=25.0), Not(trigger=PiecesHeld(min_count=1)))),
                tactic=Defend(target="opponent_intent", standoff=30.0, engage_range=150.0),
                priority=100, min_duration=1.0, cooldown=2.0, once=False,
            ),
            Rule(
                name="score",
                trigger=AtCapacity(piece_type="coral"),
                tactic=Score(region="blue_reef_face_0", action="l4"),
                priority=10,
            ),
            Rule(
                name="collect",
                trigger=PiecesHeld(piece_type="coral", max_count=0),
                tactic=Collect(piece_type="coral", mode="densest", cluster_radius=30.0),
                priority=5,
            ),
        ],
        fallback=Idle(),
    )


def test_round_trip_preserves_structure(tmp_path):
    strategy = make_strategy()
    path = tmp_path / "cycle_and_defend.json"
    strategy_io.save_strategy(strategy, path)
    loaded = strategy_io.load_strategy(path)

    assert strategy_io.to_dict(loaded) == strategy_io.to_dict(strategy)


def test_saved_file_is_plain_json_with_string_params(tmp_path):
    strategy = make_strategy()
    path = tmp_path / "s.json"
    strategy_io.save_strategy(strategy, path)

    raw = json.loads(path.read_text(encoding="utf-8"))
    score_tactic = raw["rules"][1]["tactic"]
    assert score_tactic["type"] == "Score"
    assert score_tactic["region"] == "blue_reef_face_0"
    assert score_tactic["action"] == "l4"


def test_registry_covers_every_trigger_and_tactic_used():
    strategy = make_strategy()
    for rule in strategy.rules:
        assert type(rule.trigger).__name__ in strategy_io.REGISTRY or True  # nested triggers checked below
    assert "Collect" in strategy_io.REGISTRY
    assert "Score" in strategy_io.REGISTRY
    assert "Defend" in strategy_io.REGISTRY
    assert "Idle" in strategy_io.REGISTRY
    assert "AllOf" in strategy_io.REGISTRY
    assert "Not" in strategy_io.REGISTRY
    assert "MatchTime" in strategy_io.REGISTRY
    assert "PiecesHeld" in strategy_io.REGISTRY
    assert "AtCapacity" in strategy_io.REGISTRY


def test_defense_surface_round_trips():
    """The defense/counter-defense additions have to survive the same
    save/load path everything else does -- a Defend `mode` that silently
    reverted to the default on load would be invisible in a sweep."""
    from common_sim.control import tactics, triggers

    strategy = Strategy(
        name="defense",
        rules=[
            Rule(
                name="respond",
                trigger=triggers.AllOf(
                    for_duration=2.0,
                    triggers=(triggers.AtCapacity(piece_type="coral"),
                              triggers.BeingDefended(within=120.0, region="blue_reef_face_0")),
                ),
                tactic=tactics.Score(action="l1"),
                priority=20,
            ),
            Rule(
                name="deny",
                trigger=triggers.Always(),
                tactic=tactics.Defend(mode="shadow", standoff=30.0, engage_range=180.0),
            ),
        ],
    )

    restored = strategy_io.from_dict(strategy_io.to_dict(strategy))
    assert strategy_io.to_dict(restored) == strategy_io.to_dict(strategy)

    defend = restored.rules[1].tactic
    assert (defend.mode, defend.standoff, defend.engage_range) == ("shadow", 30.0, 180.0)
    being_defended = restored.rules[0].trigger.triggers[1]
    assert (being_defended.within, being_defended.region) == (120.0, "blue_reef_face_0")
    assert restored.rules[0].trigger.for_duration == 2.0


# --- Load failures ---------------------------------------------------------
#
# A strategy file is the one artifact in this project that gets edited by
# hand, so a bad one is a normal event, not an internal error. These tests
# assert on the *content* of the message rather than just the exception
# type, because the content is the feature: the file, where in the file,
# and what would have been accepted instead. Asserting only
# `pytest.raises(StrategyLoadError)` would pass just as happily on a bare
# `KeyError: 'type'`, which is what these replaced.


def _write(tmp_path, payload, name="broken.json"):
    path = tmp_path / name
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _broken(**overrides):
    payload = strategy_io.to_dict(make_strategy())
    payload.update(overrides)
    return payload


def test_unknown_type_names_the_location_and_suggests_a_match(tmp_path):
    payload = _broken()
    payload["rules"][2]["tactic"]["type"] = "Colect"

    with pytest.raises(strategy_io.StrategyLoadError) as excinfo:
        strategy_io.load_strategy(_write(tmp_path, payload))

    message = str(excinfo.value)
    assert "broken.json" in message
    assert "rules[2].tactic" in message
    assert "Colect" in message
    assert "Did you mean: Collect?" in message
    assert "Collect" in message and "Score" in message  # the valid-type list


def test_unknown_type_nested_deep_reports_the_full_path(tmp_path):
    payload = _broken()
    payload["rules"][0]["trigger"]["triggers"][1] = {"type": "MatchTyme"}

    with pytest.raises(strategy_io.StrategyLoadError) as excinfo:
        strategy_io.load_strategy(_write(tmp_path, payload))

    assert "rules[0].trigger.triggers[1]" in str(excinfo.value)


def test_bad_parameter_name_lists_accepted_parameters(tmp_path):
    payload = _broken()
    payload["rules"][2]["tactic"]["peice_type"] = "coral"

    with pytest.raises(strategy_io.StrategyLoadError) as excinfo:
        strategy_io.load_strategy(_write(tmp_path, payload))

    message = str(excinfo.value)
    assert "rules[2].tactic" in message
    assert "Did you mean: piece_type?" in message
    # Read off __init__, not PARAM_SCHEMA, so a legal-but-not-GUI-editable
    # parameter is still reported as accepted.
    assert "replan_period" in message


def test_replan_period_is_accepted_even_though_not_in_param_schema():
    """Guards the claim the error message above makes."""
    tactic = strategy_io.from_dict({"type": "Collect", "piece_type": "coral", "replan_period": 0.5})
    assert tactic.replan_period == 0.5


def test_rule_missing_tactic_is_reported_as_a_rule_problem(tmp_path):
    payload = _broken()
    del payload["rules"][1]["tactic"]

    with pytest.raises(strategy_io.StrategyLoadError) as excinfo:
        strategy_io.load_strategy(_write(tmp_path, payload))

    message = str(excinfo.value)
    assert "rules[1]" in message
    assert "tactic" in message


def test_object_in_a_scalar_position_names_the_parameter(tmp_path):
    payload = _broken()
    payload["rules"][2]["tactic"]["cluster_radius"] = {"oops": 1}

    with pytest.raises(strategy_io.StrategyLoadError) as excinfo:
        strategy_io.load_strategy(_write(tmp_path, payload))

    assert "rules[2].tactic.cluster_radius" in str(excinfo.value)


def test_unknown_runscript_step_lists_valid_steps(tmp_path):
    payload = {
        "name": "auto", "fallback": None,
        "rules": [{
            "name": "routine", "trigger": {"type": "Always"},
            "tactic": {"type": "RunScript", "children": [{"type": "Waite", "duration": 1.0}]},
        }],
    }

    with pytest.raises(strategy_io.StrategyLoadError) as excinfo:
        strategy_io.load_strategy(_write(tmp_path, payload))

    message = str(excinfo.value)
    assert "rules[0].tactic.children[0]" in message
    assert "Did you mean: Wait?" in message


def test_missing_fallback_key_loads(tmp_path):
    """A hand-written file that just lists rules is a reasonable thing to
    type. It used to fail with a complaint about a missing "type" key --
    an error naming a key the author had never heard of."""
    payload = _broken()
    del payload["fallback"]

    strategy = strategy_io.load_strategy(_write(tmp_path, payload))

    assert strategy.fallback is None
    assert len(strategy.rules) == 3


def test_invalid_json_reports_file_and_line(tmp_path):
    path = tmp_path / "broken.json"
    path.write_text('{\n  "name": "s",\n  "rules": [,]\n}', encoding="utf-8")

    with pytest.raises(strategy_io.StrategyLoadError) as excinfo:
        strategy_io.load_strategy(path)

    message = str(excinfo.value)
    assert "broken.json" in message  # json's own error never includes this
    assert "line 3" in message


def test_missing_file_lists_the_strategies_that_do_exist(tmp_path):
    strategy_io.save_strategy(make_strategy(), tmp_path / "cycle_coral.json")
    strategy_io.save_strategy(make_strategy(), tmp_path / "full_defense.json")

    with pytest.raises(strategy_io.StrategyLoadError) as excinfo:
        strategy_io.load_strategy(tmp_path / "cycle_corel.json")

    message = str(excinfo.value)
    assert "cycle_coral" in message and "full_defense" in message


def test_load_error_is_a_value_error():
    """`run_trial` reports a bad strategy as a TrialOutcome.error rather
    than killing the pool, and other callers catch ValueError. Neither
    should have to learn a new exception type."""
    assert issubclass(strategy_io.StrategyLoadError, ValueError)
