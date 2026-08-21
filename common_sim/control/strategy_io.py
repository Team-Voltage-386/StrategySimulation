"""
Strategy <-> JSON. A saved strategy file is the same thing a GUI's
strategy editor would build and what a Monte Carlo sweep loads by name
-- keeping this the single serialization path is what stops those three
consumers from drifting apart. Region/action/piece-type params are
already plain strings on every Trigger/Tactic, so a strategy file reads
as ordinary, game-portable JSON with no custom encoding.

Because a strategy file is ordinary JSON, it is also the one part of the
sim a person edits by hand -- which makes *load failure* a normal
occurrence rather than an internal error, and the message it produces
part of the interface. Every failure below reports the file, the JSON
location inside it (`rules[2].tactic`), and the vocabulary that would
have been accepted, because the reader is someone who typo'd a tactic
name, not someone debugging this module.
"""
from __future__ import annotations

import dataclasses
import difflib
import inspect
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
        triggers.AtCapacity, triggers.ScoringAvailable, triggers.OpponentNear, triggers.BeingDefended,
        triggers.AllOf, triggers.AnyOf, triggers.Not,
        tactics.Collect, tactics.Score, tactics.Pursue, tactics.Defend, tactics.RunScript, tactics.Idle,
    )
}

# The RunScript primitives, kept as a name -> class map for the same
# reason REGISTRY exists: an unknown name should be able to report what
# the valid names were.
_PRIMITIVES: dict[str, type] = {cls.__name__: cls for cls in (Wait, RunIntake, RunManipulator, DriveToPose)}


class StrategyLoadError(ValueError):
    """A strategy file or payload could not be turned into a Strategy.

    Deliberately a `ValueError` subclass. `run_trial` wraps a whole trial
    in `except Exception` and turns it into a `TrialOutcome.error`
    string, and the GUI catches broadly too, so raising a new top-level
    exception type would change nothing for existing handlers -- but a
    caller that today catches `ValueError` around a load keeps working.
    """


def _location(where: str, source: str | Path | None) -> str:
    """Prefix identifying *which* JSON and *where inside it*.

    `where` is a JSON-ish path built up as the recursion descends
    (`rules[2].tactic.triggers[0]`) rather than a Python object repr,
    since what the reader has open is the file, and the thing they need
    is somewhere to put their cursor.
    """
    parts = []
    if source is not None:
        parts.append(str(source))
    parts.append(where or "(top level)")
    return ": ".join(parts)


def _fail(problem: str, where: str, source: str | Path | None, *hints: str):
    message = f"{_location(where, source)}: {problem}"
    for hint in hints:
        if hint:
            message += f"\n  {hint}"
    raise StrategyLoadError(message)


def _suggest(name: str, valid) -> str:
    """"Did you mean" line, or empty when nothing is close.

    A typo is the overwhelmingly likely cause of an unknown name, and
    the full valid list is 16 entries -- long enough that the near-match
    is worth pulling out separately rather than leaving someone to
    eyeball the list for the one character they got wrong.
    """
    close = difflib.get_close_matches(name, sorted(valid), n=3, cutoff=0.6)
    return f"Did you mean: {', '.join(close)}?" if close else ""


def _accepted_params(cls) -> list[str]:
    """Constructor parameter names, for reporting an unexpected key.

    Read off `__init__` rather than `PARAM_SCHEMA` because the two
    genuinely differ: `PARAM_SCHEMA` is the subset the GUI offers as
    editable, while `Collect.__init__` also accepts `replan_period`.
    A hand-written file that sets `replan_period` is legal, so the error
    message must not claim otherwise.
    """
    try:
        return [p for p in inspect.signature(cls.__init__).parameters if p not in ("self", "args", "kwargs")]
    except (TypeError, ValueError):  # a C-level or otherwise uninspectable __init__
        return [f.name for f in dataclasses.fields(cls)] if dataclasses.is_dataclass(cls) else []


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


def _deserialize_primitive(d, where: str, source):
    if not isinstance(d, dict):
        _fail(f"expected a RunScript step object, got {type(d).__name__}", where, source)
    kind = d.get("type")
    if kind is None:
        _fail("RunScript step has no \"type\"", where, source,
              f"Valid step types: {', '.join(sorted(_PRIMITIVES))}")
    if kind not in _PRIMITIVES:
        _fail(f"unknown RunScript step type {kind!r}", where, source,
              _suggest(kind, _PRIMITIVES), f"Valid step types: {', '.join(sorted(_PRIMITIVES))}")

    try:
        if kind == "Wait":
            return Wait(d["duration"])
        if kind == "RunIntake":
            return RunIntake(timeout=d.get("timeout"))
        if kind == "RunManipulator":
            return RunManipulator(d["action"], timeout=d.get("timeout"))
        x, y, heading = d["target"]
        return DriveToPose(
            Pose2d(x, y, heading), position_tolerance=d.get("position_tolerance", 2.0),
            heading_tolerance=d.get("heading_tolerance", 0.05),
            speed_gain=d.get("speed_gain", 3.0), heading_gain=d.get("heading_gain", 4.0),
        )
    except KeyError as exc:
        _fail(f"{kind} is missing required key {exc.args[0]!r}", where, source,
              f"{kind} accepts: {', '.join(_accepted_params(_PRIMITIVES[kind]))}")
    except (TypeError, ValueError) as exc:
        _fail(f"{kind} could not be built: {exc}", where, source)


def _serialize_value(value):
    if isinstance(value, (Trigger, Behavior)):
        return to_dict(value)
    if isinstance(value, (tuple, list)):
        return [_serialize_value(v) for v in value]
    return value


def _deserialize_value(value, where: str, source):
    if isinstance(value, dict):
        # In a *parameter* position the only legal object is a nested
        # trigger or tactic, which always carries a "type". Checking here
        # rather than deferring to `_from_dict` is what lets the error name
        # the parameter: no trigger or tactic takes a plain dict, so
        # `cluster_radius: {...}` is a mistake, and left to the constructor
        # it would be accepted silently and only misbehave mid-match.
        if "type" not in value:
            _fail(
                f"expected a value here, but found a JSON object with keys "
                f"{', '.join(sorted(map(str, value))) or '(none)'}",
                where, source,
                "Only a nested trigger or tactic can be an object, and it needs a \"type\".",
            )
        return _from_dict(value, where, source)
    if isinstance(value, list):
        return tuple(_deserialize_value(v, f"{where}[{i}]", source) for i, v in enumerate(value))
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


def _strategy_from_dict(d: dict, where: str, source) -> Strategy:
    rules = d.get("rules")
    if not isinstance(rules, list):
        _fail(f"\"rules\" must be a list, got {type(rules).__name__}", where, source)
    prefix = f"{where}." if where else ""
    return Strategy(
        name=d.get("name", ""),
        # `_rule_from_dict` directly, not `_from_dict`: an entry in "rules"
        # is a rule by definition, so there is nothing to sniff, and going
        # through the general path meant a rule that had lost its "tactic"
        # key was reported as an unidentifiable object instead of as a rule
        # missing a tactic.
        rules=[_rule_from_dict(r, f"{prefix}rules[{i}]", source) for i, r in enumerate(rules)],
        # `fallback` is optional on the way in even though `to_dict`
        # always writes it. A hand-written file that just lists rules is
        # a reasonable thing to type, and before this it failed as
        # "no 'type' key" -- an error about a key the author never knew
        # existed, pointing at a file that looked complete.
        fallback=_from_dict(d.get("fallback"), f"{prefix}fallback", source),
    )


def _rule_from_dict(d, where: str, source) -> Rule:
    if not isinstance(d, dict):
        _fail(f"expected a rule object, got {type(d).__name__}", where, source)
    prefix = f"{where}." if where else ""
    for required in ("trigger", "tactic"):
        if d.get(required) is None:
            _fail(f"rule has no {required!r}", where, source,
                  "A rule needs both a trigger (when it applies) and a tactic (what to do).")
    try:
        return Rule(
            name=d.get("name", ""),
            trigger=_from_dict(d["trigger"], f"{prefix}trigger", source),
            tactic=_from_dict(d["tactic"], f"{prefix}tactic", source),
            priority=d.get("priority", 0), min_duration=d.get("min_duration", 0.0),
            cooldown=d.get("cooldown", 0.0), once=d.get("once", False),
        )
    except TypeError as exc:  # a bad scalar type on priority/min_duration/...
        _fail(f"rule could not be built: {exc}", where, source)


def _typed_from_dict(d: dict, where: str, source):
    name = d["type"]
    if name == "RunScript":
        children = d.get("children", [])
        if not isinstance(children, list):
            _fail(f"RunScript \"children\" must be a list, got {type(children).__name__}", where, source)
        prefix = f"{where}." if where else ""
        return tactics.RunScript([
            _deserialize_primitive(c, f"{prefix}children[{i}]", source) for i, c in enumerate(children)
        ])

    cls = REGISTRY.get(name)
    if cls is None:
        _fail(f"unknown trigger/tactic type {name!r}", where, source,
              _suggest(name, REGISTRY), f"Valid types: {', '.join(sorted(REGISTRY))}")

    prefix = f"{where}." if where else ""
    kwargs = {k: _deserialize_value(v, f"{prefix}{k}", source) for k, v in d.items() if k != "type"}
    try:
        return cls(**kwargs)
    except TypeError as exc:
        # The common cause is a misspelled or invented parameter name, so
        # lead with the accepted set. `unexpected keyword argument` text
        # from CPython names only the offending key, not the alternatives.
        accepted = _accepted_params(cls)
        unexpected = sorted(set(kwargs) - set(accepted))
        hints = [f"{name} accepts: {', '.join(accepted) or '(no parameters)'}"]
        if unexpected:
            hints.insert(0, _suggest(unexpected[0], accepted))
        _fail(f"{name} could not be built: {exc}", where, source, *hints)


def _from_dict(d, where: str, source):
    if d is None:
        return None
    if not isinstance(d, dict):
        _fail(f"expected a JSON object, got {type(d).__name__}", where, source)

    # Dispatch on "type" first. Shape-sniffing has to come second because
    # the shapes genuinely overlap: `Not` has a field named `trigger`, so
    # "has a trigger key" cannot mean "is a rule". Only Strategy and Rule
    # lack a "type", and no registered class has a "rules" or "tactic"
    # field, so these three tests are mutually exclusive.
    if "type" in d:
        return _typed_from_dict(d, where, source)
    if "rules" in d:
        return _strategy_from_dict(d, where, source)
    if "tactic" in d:
        return _rule_from_dict(d, where, source)

    _fail(
        f"cannot tell what this object is -- it has no \"type\", \"rules\", or \"tactic\" key "
        f"(keys present: {', '.join(sorted(map(str, d))) or 'none'})",
        where, source,
        "A strategy has \"rules\"; a rule has \"trigger\" and \"tactic\"; "
        "a trigger or tactic has \"type\".",
    )


def from_dict(d: dict | None, *, source: str | Path | None = None):
    """Rebuild a Strategy/Rule/Trigger/Tactic from its dict form.

    `source` is cosmetic -- the file or origin name to put in front of
    any error. `load_strategy` fills it in; a caller deserializing an
    in-memory payload (an unsaved GUI edit crossing a process boundary,
    see sweep_trial._resolve_strategy) can pass its own label or leave it
    off.
    """
    return _from_dict(d, "", source)


def save_strategy(strategy: Strategy, path: str | Path) -> None:
    Path(path).write_text(json.dumps(to_dict(strategy), indent=2), encoding="utf-8")


def load_strategy(path: str | Path) -> Strategy:
    path = Path(path)
    try:
        text = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        # Naming the siblings turns the single most common sweep failure
        # -- a strategy referenced by a name that was never saved under
        # it -- from a bare path into a list to pick from.
        available = sorted(p.stem for p in path.parent.glob("*.json")) if path.parent.is_dir() else []
        raise StrategyLoadError(
            f"{path}: no such strategy file."
            + (f"\n  Strategies in {path.parent}: {', '.join(available)}" if available
               else f"\n  No strategy files found in {path.parent}.")
        ) from None
    except OSError as exc:
        raise StrategyLoadError(f"{path}: could not be read: {exc}") from None

    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        # json's own message carries line/column but never the filename,
        # which is the half that matters when a sweep loads six files.
        raise StrategyLoadError(f"{path}: line {exc.lineno}, column {exc.colno}: invalid JSON: {exc.msg}") from None

    strategy = from_dict(payload, source=path.name)
    if not isinstance(strategy, Strategy):
        _fail(
            f"file describes a {type(strategy).__name__}, not a whole strategy",
            "", path.name,
            "A strategy file's top-level object needs a \"rules\" list.",
        )
    return strategy
