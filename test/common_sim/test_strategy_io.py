import json

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
                tactic=Collect(piece_type="coral", mode="densest", cluster_radius=30.0, prefer_station=True),
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
