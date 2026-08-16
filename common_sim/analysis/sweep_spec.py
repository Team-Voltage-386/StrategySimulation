"""
Plain-data description of a Monte Carlo sweep trial: which robots are in
the match, which of their fields vary and how, and the fully-resolved
per-run job that gets sent across the process boundary. Nothing here
imports Qt or game_specific -- see ARCHITECTURE.md's import contract.

A trial is a selectable 1..N robot roster (blue + red), not a single
robot. `RobotSpec.characteristics` ships as a plain dict (not the
`RobotCharacteristics` dataclass) so a swept field is addressed by a
dotted path (`deposit_time_by_action.l4`, trivial to do on a dict),
stays JSON-able for future persistence, and doesn't tie the worker to
the caller's exact dataclass definition.

Jobs are fully resolved *before* dispatch: `expand_jobs` applies every
grid combination to copies of the base `RobotSpec`s, so the worker
(`game_specific/reefscape/sweep_trial.py`) never interprets a variable
-- `TrialJob.params` exists only to label the results row.
"""
from __future__ import annotations

import dataclasses
import itertools
from dataclasses import dataclass, field
from typing import Optional

from common_sim.analysis.monte_carlo import ParameterSweep
from common_sim.robot.characteristics import RobotCharacteristics, SideManipulators


# -- RobotCharacteristics <-> plain dict -------------------------------

def _encode_value(value):
    if isinstance(value, frozenset):
        return sorted(value)
    if isinstance(value, SideManipulators):
        return {
            "intake_piece_types": sorted(value.intake_piece_types),
            "score_piece_types": sorted(value.score_piece_types),
            "intake_source": value.intake_source,
        }
    if isinstance(value, dict):
        return {k: _encode_value(v) for k, v in value.items()}
    return value


def characteristics_to_spec(c: RobotCharacteristics) -> dict:
    """RobotCharacteristics -> plain dict: every frozenset becomes a
    sorted list, every SideManipulators becomes a dict, so the result is
    JSON-able and addressable by dotted path."""
    return {f.name: _encode_value(getattr(c, f.name)) for f in dataclasses.fields(c)}


def characteristics_from_spec(spec: dict) -> RobotCharacteristics:
    """Inverse of characteristics_to_spec."""
    kwargs = dict(spec)
    if "accepted_piece_types" in kwargs and kwargs["accepted_piece_types"] is not None:
        kwargs["accepted_piece_types"] = frozenset(kwargs["accepted_piece_types"])
    side_manipulators = kwargs.get("side_manipulators")
    if side_manipulators:
        kwargs["side_manipulators"] = {
            side: SideManipulators(
                intake_piece_types=frozenset(sm.get("intake_piece_types", ())),
                score_piece_types=frozenset(sm.get("score_piece_types", ())),
                intake_source=sm.get("intake_source", "both"),
            )
            for side, sm in side_manipulators.items()
        }
    return RobotCharacteristics(**kwargs)


# -- sweepable field discovery ------------------------------------------

@dataclass(frozen=True)
class FieldDescriptor:
    path: str        # "max_speed" | "deposit_time_by_action.l4" | "strategy"
    kind: str         # "float" | "int" | "categorical"
    default: object
    choices: tuple = ()
    minimum: Optional[float] = None
    maximum: Optional[float] = None
    display_scale: float = 1.0
    suffix: str = ""
    label: str = ""   # human-readable name for the variable-picker UI; falls back to `path` if unset


def numeric_characteristic_fields() -> tuple:
    """(path, kind) for every top-level scalar numeric
    RobotCharacteristics field, auto-derived from the dataclass.

    Trap: characteristics.py uses `from __future__ import annotations`,
    so dataclasses.fields(...)[i].type is the *string* "float"/"int",
    not the type object -- compared as strings here."""
    result = []
    for f in dataclasses.fields(RobotCharacteristics):
        if f.type in ("float", "int"):
            result.append((f.name, f.type))
    return tuple(result)


def _numeric_dict_fields() -> tuple:
    """(field_name, element_kind) for every RobotCharacteristics field
    that is a str-keyed dict of a numeric type (piece_capacity_by_type,
    intake_time_by_type, deposit_time_by_action, ...) -- excludes
    side_manipulators (dict[str, SideManipulators])."""
    result = []
    for f in dataclasses.fields(RobotCharacteristics):
        if f.type == "dict[str, int]":
            result.append((f.name, "int"))
        elif f.type == "dict[str, float]":
            result.append((f.name, "float"))
    return tuple(result)


def _default_label(path: str) -> str:
    return path.replace("_", " ").title()


def sweepable_fields(
    char_spec: dict, *, unit_hints: Optional[dict] = None, label_hints: Optional[dict] = None, extra: tuple = (),
) -> tuple:
    """Every FieldDescriptor a sweep can vary on this robot: every
    top-level scalar numeric field, plus one dotted-path descriptor per
    key actually present in `char_spec`'s own *_by_* dicts (so a robot
    that never set a "net" deposit time doesn't get a phantom column),
    plus whatever the caller appends via `extra` (e.g. a "strategy"
    categorical). `unit_hints` maps a path (or a whole dict field name,
    as a fallback for its per-key entries) to (display_scale, suffix).

    `label_hints` maps a path to the human-readable name shown in the
    variable-picker UI (falls back to a title-cased `path`). For a
    dict-field entry it may instead key on the whole field name with a
    "{type}" placeholder, e.g. {"intake_time_by_type": "Field Intake
    Time ({type})"} -- lets game-specific code disambiguate names that
    collide once stripped of their dict-field prefix, like
    "intake_time_by_type.coral" (field pickup) vs "station_intake_time"
    (human-player station), without this module needing to know that
    distinction itself."""
    unit_hints = unit_hints or {}
    label_hints = label_hints or {}
    descriptors = []

    for path, kind in numeric_characteristic_fields():
        scale, suffix = unit_hints.get(path, (1.0, ""))
        label = label_hints.get(path, _default_label(path))
        descriptors.append(FieldDescriptor(
            path=path, kind=kind, default=char_spec.get(path, 0), display_scale=scale, suffix=suffix, label=label,
        ))

    for field_name, kind in _numeric_dict_fields():
        sub = char_spec.get(field_name) or {}
        for key in sorted(sub):
            path = f"{field_name}.{key}"
            scale, suffix = unit_hints.get(path, unit_hints.get(field_name, (1.0, "")))
            if path in label_hints:
                label = label_hints[path]
            elif field_name in label_hints:
                label = label_hints[field_name].format(type=key.title())
            else:
                label = f"{_default_label(field_name)} ({key.title()})"
            descriptors.append(FieldDescriptor(
                path=path, kind=kind, default=sub[key], display_scale=scale, suffix=suffix, label=label,
            ))

    descriptors.extend(extra)
    return tuple(descriptors)


# -- sampling / variables -------------------------------------------------

@dataclass(frozen=True)
class NumericSampling:
    minimum: float
    maximum: float
    count: int

    def values(self) -> tuple:
        if self.count <= 1:
            return (self.minimum,)
        step = (self.maximum - self.minimum) / (self.count - 1)
        return tuple(self.minimum + step * i for i in range(self.count))


@dataclass(frozen=True)
class SweepVariable:
    target: str       # robot label, e.g. "PRIMARY" | "BLUE 0"
    path: str         # "max_speed" | "deposit_time_by_action.l4" | "strategy"
    values: tuple

    @property
    def column(self) -> str:
        return f"{self.target}.{self.path}"


def apply_variable(robot_spec: "RobotSpec", path: str, value) -> "RobotSpec":
    if path == "strategy":
        return dataclasses.replace(robot_spec, strategy=value)
    if "." in path:
        field_name, key = path.split(".", 1)
        if field_name not in robot_spec.characteristics:
            raise ValueError(f"unknown sweep path {path!r}")
        chars = dict(robot_spec.characteristics)
        sub = dict(chars[field_name] or {})
        sub[key] = value
        chars[field_name] = sub
        return dataclasses.replace(robot_spec, characteristics=chars)
    if path not in robot_spec.characteristics:
        raise ValueError(f"unknown sweep path {path!r}")
    chars = dict(robot_spec.characteristics)
    chars[path] = value
    return dataclasses.replace(robot_spec, characteristics=chars)


# -- jobs -----------------------------------------------------------------

@dataclass(frozen=True)
class RobotSpec:
    label: str                    # "PRIMARY" | "BLUE 0" | "RED 0" | ...
    alliance: str                 # "blue" | "red"
    roster_index: int             # -1 for PRIMARY, else the roster slot index
    characteristics: dict
    strategy: object = None       # strategy name (str), strategy_io payload (dict), or None


@dataclass(frozen=True)
class MatchSpec:
    auto_duration: float
    teleop_duration: float
    disable_friendly_collisions: bool = False


@dataclass(frozen=True)
class TrialJob:
    index: int
    seed: int
    params: dict
    robots: tuple
    match: MatchSpec
    variability: object           # VariabilityModel
    strategies_dir: str
    dt: float


@dataclass(frozen=True)
class TrialOutcome:
    index: int
    seed: int
    params: dict
    metrics: object = None        # MatchMetrics | None
    error: Optional[str] = None
    duration_s: float = 0.0


def expand_jobs(
    robots: list, match: MatchSpec, variability, variables: list,
    *, repetitions: int = 1, base_seed: int = 0, strategies_dir, dt: float,
) -> list:
    """Full-factorial expansion (via monte_carlo.ParameterSweep, keyed by
    SweepVariable.column) x repetitions. Reusing ParameterSweep keeps
    full-factorial expansion in one place -- a Latin-hypercube generator
    with the same `.configs()` shape drops in here later with no other
    change needed."""
    sweep = ParameterSweep({v.column: v.values for v in variables})
    jobs = []
    index = 0
    for config in sweep.configs():
        for _ in range(max(1, repetitions)):
            robots_for_job = list(robots)
            for var in variables:
                value = config[var.column]
                target_i = next(i for i, r in enumerate(robots_for_job) if r.label == var.target)
                robots_for_job[target_i] = apply_variable(robots_for_job[target_i], var.path, value)
            jobs.append(TrialJob(
                index=index, seed=base_seed + index,
                params={var.column: config[var.column] for var in variables},
                robots=tuple(robots_for_job), match=match, variability=variability,
                strategies_dir=str(strategies_dir), dt=dt,
            ))
            index += 1
    return jobs


def total_run_count(variables: list, repetitions: int) -> int:
    """Cheap: must not build the config list."""
    count = 1
    for var in variables:
        count *= max(1, len(var.values))
    return count * max(1, repetitions)
