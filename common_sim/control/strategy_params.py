"""
The continuous half of a strategy, as a flat vector -- the view a
numerical optimizer needs onto the JSON a person edits.

A `Rule[]` mixes two kinds of decision that want completely different
search methods, and the seam between them is already in the data model:

* **Structure** -- which rules exist, their trigger and tactic *types*,
  their `priority` ordering, `once`. Discrete, and fitness is
  piecewise-constant in it: nothing changes until an ordering flips, then
  everything does.
* **Parameters** -- `for_duration`, `min_duration`, `cooldown`,
  `cluster_radius`, `max_range`, `standoff`, `engage_range`. Continuous,
  and where a gradient-free optimizer has something to work with.

This module exposes the second set and nothing else, so a parameter
search provably cannot alter what the strategy *is*: same rules, same
order, same tactics, only their numbers move. That is what keeps the
output readable by a drive team -- what gets optimized is literally the
`Rule[]` handed over, with no distillation step in between.

Operating on the `strategy_io.to_dict()` payload rather than on live
`Strategy` objects is deliberate. The payload is what already crosses the
process boundary into a sweep worker (`RobotSpec.strategy` accepts a
dict), it round-trips through `strategy_io` for free, and it means a
candidate can be written to disk as an ordinary strategy file the moment
the search likes it.

Three exclusions worth stating, because each one is a judgement rather
than an oversight:

* **`null` stays `null`.** An optional float that is currently unset
  (`max_range: null` means "no range limit") is *structure*: giving it a
  number adds a constraint the author did not write. Turning it on
  belongs to a structure search, not here.
* **`int` params are not searched.** `min_count`/`max_count` are counts
  of pieces; the landscape in them is a staircase with two or three
  steps, which CMA-ES handles badly and a plain enumeration handles
  perfectly.
* **`priority` is not searched**, for the same reason, and because its
  effect is entirely relative -- only the induced ordering matters.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass

from common_sim.control.strategy_io import REGISTRY

# Fields of a Rule itself (not of its trigger or tactic) that are
# continuous timings. `once` is boolean and `priority` is ordinal, so
# neither belongs here -- see the module docstring.
_RULE_TIMING_FIELDS = ("min_duration", "cooldown")

# Declared on Trigger once rather than repeated in every subclass's
# PARAM_SCHEMA, so it has to be picked up by name.
_TRIGGER_TIMING_FIELD = "for_duration"

# Where an upper bound comes from when the Param does not declare a
# `max`, keyed by the unit the Param already carries in its `suffix`.
# Deriving one from the seed value instead (say, four times whatever the
# author wrote) would make the search space depend on the starting point,
# so two strategies that differ only in a seed value would be searched
# over different boxes and their results would not be comparable.
#
# These are generous on purpose -- a bound the optimum sits against is a
# bound that decided the answer. `param_space(overrides=...)` is the way
# to say something game-specific, e.g. that a range wants the field's
# diagonal.
DEFAULT_UPPER_BY_SUFFIX = {
    " s": 20.0,     # a fifth of a teleop period; longer is a rule that fires once
    " in": 360.0,   # a bit over half an FRC field's long axis
    " pts": 50.0,
}
DEFAULT_UPPER = 100.0

# Timings measured in seconds that live on the Rule rather than on a
# Param, so they have no `suffix` to look up.
_RULE_TIMING_UPPER = DEFAULT_UPPER_BY_SUFFIX[" s"]


@dataclass(frozen=True)
class ParamRef:
    """One searchable number: where it lives, what it is now, and the box
    it may move in.

    `location` is the sequence of dict keys and list indices to walk from
    the strategy payload down to the value. `path` is the same thing
    spelled the way `strategy_io`'s errors spell it
    (`rules[1].tactic.cluster_radius`), for reports a person reads.
    """
    path: str
    location: tuple
    value: float
    lower: float
    upper: float

    @property
    def span(self) -> float:
        return self.upper - self.lower


def _bounds_for(param, value: float, upper_default: float) -> tuple:
    lower = 0.0 if param is None or param.min is None else float(param.min)
    upper = upper_default if param is None or param.max is None else float(param.max)
    # The seed value always has to be inside its own box, or generation 0
    # starts by silently rewriting the strategy it was asked to improve.
    return min(lower, float(value)), max(upper, float(value))


def _float_params(type_name: str) -> dict:
    """PARAM_SCHEMA entries of kind "float", by name, for a registered
    trigger or tactic. An unregistered type (RunScript, whose children
    are behavior.py primitives with no schema) contributes nothing."""
    cls = REGISTRY.get(type_name)
    return {p.name: p for p in getattr(cls, "PARAM_SCHEMA", ()) if p.kind == "float"}


def _walk_node(node, location: tuple, path: str, out: list, overrides: dict, *, nested: bool) -> None:
    """Collect the searchable numbers in one trigger or tactic, then
    recurse into any nested triggers (AllOf/AnyOf hold a `triggers` list,
    Not holds a single `trigger`)."""
    if not isinstance(node, dict):
        return
    type_name = node.get("type")
    if not isinstance(type_name, str):
        return

    schema = _float_params(type_name)
    # `for_duration` is not in any PARAM_SCHEMA (Trigger declares it once
    # for every subclass), so it is handled by name.
    #
    # Only on an *outermost* trigger, though. A `for_duration` nested
    # inside an AllOf/AnyOf/Not is never applied -- the parent evaluates
    # its children with plain `evaluate()` and never consults the
    # hysteresis clock -- and `strategy._reject_nested_for_duration`
    # refuses to build a controller for one rather than run a strategy
    # that doesn't mean what it says. A search that raised a nested 0.0
    # to 2.5 would turn a working strategy into one that fails every
    # trial, so nested duration fields are left alone.
    names = list(schema)
    if not nested and _TRIGGER_TIMING_FIELD in node:
        names.append(_TRIGGER_TIMING_FIELD)

    for name in names:
        value = node.get(name)
        if not _is_searchable_number(value):
            continue
        param = schema.get(name)
        suffix = " s" if param is None else param.suffix
        default_upper = DEFAULT_UPPER_BY_SUFFIX.get(suffix, DEFAULT_UPPER)
        _append(out, f"{path}.{name}", location + (name,), value, param, default_upper, overrides)

    if isinstance(node.get("trigger"), dict):  # Not
        _walk_node(node["trigger"], location + ("trigger",), f"{path}.trigger", out, overrides, nested=True)
    children = node.get("triggers")  # AllOf / AnyOf
    if isinstance(children, list):
        for i, child in enumerate(children):
            _walk_node(child, location + ("triggers", i), f"{path}.triggers[{i}]", out, overrides, nested=True)


def _is_searchable_number(value) -> bool:
    # `bool` is an `int` in Python, and `once: true` must never be read as
    # the number 1 and handed to a continuous optimizer.
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _append(out: list, path: str, location: tuple, value, param, default_upper: float, overrides: dict) -> None:
    lower, upper = _bounds_for(param, value, default_upper)
    if path in overrides:
        lower, upper = (float(b) for b in overrides[path])
    if upper <= lower:
        # A zero-width axis contributes nothing but a division by zero
        # downstream, so it is dropped rather than searched.
        return
    out.append(ParamRef(path=path, location=location, value=float(value), lower=lower, upper=upper))


def continuous_params(payload: dict, *, overrides: dict | None = None) -> tuple:
    """Every searchable number in a `strategy_io.to_dict()` payload, in a
    stable order (rule order, then rule timings, then trigger, then
    tactic).

    Order matters more than it looks: it fixes the meaning of every
    position in the search vector, so a resumed or re-run search over the
    same strategy explores the same space. `overrides` maps a `ParamRef.path`
    to an explicit `(lower, upper)`, for the cases where the generic
    bounds are wrong for a particular game.
    """
    overrides = overrides or {}
    rules = payload.get("rules")
    if not isinstance(rules, list):
        raise ValueError('strategy payload has no "rules" list -- expected strategy_io.to_dict() output')

    out: list = []
    for i, rule in enumerate(rules):
        if not isinstance(rule, dict):
            continue
        base = f"rules[{i}]"
        for name in _RULE_TIMING_FIELDS:
            value = rule.get(name)
            if _is_searchable_number(value):
                _append(out, f"{base}.{name}", ("rules", i, name),
                        value, None, _RULE_TIMING_UPPER, overrides)
        if isinstance(rule.get("trigger"), dict):
            _walk_node(rule["trigger"], ("rules", i, "trigger"), f"{base}.trigger", out, overrides, nested=False)
        if isinstance(rule.get("tactic"), dict):
            _walk_node(rule["tactic"], ("rules", i, "tactic"), f"{base}.tactic", out, overrides, nested=False)
    return tuple(out)


def to_vector(refs) -> list:
    """The strategy's current values, in `refs` order -- the seed a search
    starts from."""
    return [ref.value for ref in refs]


def with_vector(payload: dict, refs, vector) -> dict:
    """A deep copy of `payload` with each `refs[i]` set to `vector[i]`.

    A copy, never a mutation: a search holds one seed payload and builds
    a population from it every generation, and an in-place write would
    make generation N+1 a perturbation of generation N's last candidate.
    """
    values = list(vector)
    if len(values) != len(refs):
        raise ValueError(f"expected {len(refs)} values, got {len(values)}")
    result = copy.deepcopy(payload)
    for ref, value in zip(refs, values):
        node = result
        for key in ref.location[:-1]:
            node = node[key]
        node[ref.location[-1]] = float(value)
    return result


def describe(refs, vector=None) -> str:
    """A table of path, value and box -- what a search prints so its
    result can be checked against the strategy file by eye."""
    values = to_vector(refs) if vector is None else list(vector)
    width = max((len(r.path) for r in refs), default=4)
    lines = [f"  {'parameter'.ljust(width)}  {'value':>9}   bounds"]
    for ref, value in zip(refs, values):
        lines.append(f"  {ref.path.ljust(width)}  {value:9.3f}   [{ref.lower:g}, {ref.upper:g}]")
    return "\n".join(lines)
