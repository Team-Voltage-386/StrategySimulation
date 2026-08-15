"""
Strategy <-> JSON. A saved strategy file is the same thing a GUI's
strategy editor would build and what a Monte Carlo sweep loads by name
-- keeping this the single serialization path is what stops those three
consumers from drifting apart. Region/action/piece-type params are
already plain strings on every Trigger/Tactic, so a strategy file reads
as ordinary, game-portable JSON with no custom encoding.
"""
from __future__ import annotations

import dataclasses
import json
from pathlib import Path

from common_sim.control import tactics, triggers
from common_sim.control.behavior import Behavior, DriveToPose, RunIntake, RunManipulator, Wait
from common_sim.control.strategy import Rule, Strategy
from common_sim.control.triggers import Trigger
from common_sim.geometry import Pose2d

REGISTRY: dict[str, type] = {
    cls.__name__: cls
    for cls in (
        triggers.Always, triggers.PiecesAvailable, triggers.MatchTime, triggers.PiecesHeld,
        triggers.AtCapacity, triggers.ScoringAvailable, triggers.OpponentNear,
        triggers.AllOf, triggers.AnyOf, triggers.Not,
        tactics.Collect, tactics.Score, tactics.Defend, tactics.RunScript, tactics.Idle,
    )
}


# RunScript wraps a plain Sequence of behavior.py primitives, which are
# NOT Tactics/Triggers (no PARAM_SCHEMA, and behavior.py is never
# modified -- see ARCHITECTURE.md's "nothing in behavior.py changes").
# This is a small, local encoding for just the primitives an auto
# routine actually uses, so a RunScript rule can still round-trip
# through a strategy file without touching behavior.py itself.
def _serialize_primitive(obj) -> dict:
    if isinstance(obj, Wait):
        return {"type": "Wait", "duration": obj.duration}
    if isinstance(obj, RunIntake):
        return {"type": "RunIntake", "timeout": obj.timeout}
    if isinstance(obj, RunManipulator):
        return {"type": "RunManipulator", "action": obj.action, "timeout": obj.timeout}
    if isinstance(obj, DriveToPose):
        return {
            "type": "DriveToPose", "target": [obj.target.x, obj.target.y, obj.target.heading],
            "position_tolerance": obj.position_tolerance, "heading_tolerance": obj.heading_tolerance,
            "speed_gain": obj.speed_gain, "heading_gain": obj.heading_gain,
        }
    raise TypeError(f"strategy_io cannot serialize RunScript primitive {obj!r}")


def _deserialize_primitive(d: dict):
    kind = d["type"]
    if kind == "Wait":
        return Wait(d["duration"])
    if kind == "RunIntake":
        return RunIntake(timeout=d.get("timeout"))
    if kind == "RunManipulator":
        return RunManipulator(d["action"], timeout=d.get("timeout"))
    if kind == "DriveToPose":
        x, y, heading = d["target"]
        return DriveToPose(
            Pose2d(x, y, heading), position_tolerance=d.get("position_tolerance", 2.0),
            heading_tolerance=d.get("heading_tolerance", 0.05),
            speed_gain=d.get("speed_gain", 3.0), heading_gain=d.get("heading_gain", 4.0),
        )
    raise ValueError(f"unknown RunScript primitive type {kind!r}")


def _serialize_value(value):
    if isinstance(value, (Trigger, Behavior)):
        return to_dict(value)
    if isinstance(value, (tuple, list)):
        return [_serialize_value(v) for v in value]
    return value


def _deserialize_value(value):
    if isinstance(value, dict):
        return from_dict(value)
    if isinstance(value, list):
        return tuple(_deserialize_value(v) for v in value)
    return value


def to_dict(obj) -> dict | None:
    if obj is None:
        return None
    if isinstance(obj, Strategy):
        return {"name": obj.name, "rules": [to_dict(r) for r in obj.rules], "fallback": to_dict(obj.fallback)}
    if isinstance(obj, Rule):
        return {
            "name": obj.name, "trigger": to_dict(obj.trigger), "tactic": to_dict(obj.tactic),
            "priority": obj.priority, "min_duration": obj.min_duration, "cooldown": obj.cooldown, "once": obj.once,
        }
    if isinstance(obj, Trigger):
        d = {"type": type(obj).__name__}
        for f in dataclasses.fields(obj):
            d[f.name] = _serialize_value(getattr(obj, f.name))
        return d
    if isinstance(obj, tactics.RunScript):
        return {"type": "RunScript", "children": [_serialize_primitive(c) for c in obj.children]}
    if isinstance(obj, Behavior):
        d = {"type": type(obj).__name__}
        for param in getattr(obj, "PARAM_SCHEMA", ()):
            d[param.name] = _serialize_value(getattr(obj, param.name))
        return d
    raise TypeError(f"strategy_io cannot serialize {obj!r}")


def from_dict(d: dict | None):
    if d is None:
        return None
    if "rules" in d and "fallback" in d:
        return Strategy(name=d["name"], rules=[from_dict(r) for r in d["rules"]], fallback=from_dict(d["fallback"]))
    if "trigger" in d and "tactic" in d:
        return Rule(
            name=d["name"], trigger=from_dict(d["trigger"]), tactic=from_dict(d["tactic"]),
            priority=d.get("priority", 0), min_duration=d.get("min_duration", 0.0),
            cooldown=d.get("cooldown", 0.0), once=d.get("once", False),
        )
    if d["type"] == "RunScript":
        return tactics.RunScript([_deserialize_primitive(c) for c in d["children"]])
    cls = REGISTRY[d["type"]]
    kwargs = {k: _deserialize_value(v) for k, v in d.items() if k != "type"}
    return cls(**kwargs)


def save_strategy(strategy: Strategy, path: str | Path) -> None:
    Path(path).write_text(json.dumps(to_dict(strategy), indent=2), encoding="utf-8")


def load_strategy(path: str | Path) -> Strategy:
    return from_dict(json.loads(Path(path).read_text(encoding="utf-8")))
