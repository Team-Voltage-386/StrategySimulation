"""
Declarative parameter description shared by Trigger and Tactic
PARAM_SCHEMA tuples. A GUI (gui_utils/strategy_editor.py) builds a
property-inspector widget straight from these -- no per-trigger or
per-tactic GUI code required when a new one is added to common_sim.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Param:
    name: str
    # "piece_type" | "region_name" | "action" | "choice" | "float" | "int"
    # | "bool" | "str" | "trigger" | "trigger_list"
    kind: str
    default: object = None
    optional: bool = False
    choices: tuple = ()
    min: float | None = None
    max: float | None = None
    suffix: str = ""
