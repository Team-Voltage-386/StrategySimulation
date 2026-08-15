"""
Scoring-table abstraction. A game_specific package supplies a concrete
ScoringRules with its own point values; common_sim's Match only ever
calls points_for(action, phase) and never hardcodes a point value.
"""
from __future__ import annotations

from abc import ABC, abstractmethod


class ScoringRules(ABC):
    @abstractmethod
    def points_for(self, action: str, phase: str) -> float:
        """Points awarded for `action` (a ScoringRegion.action string)
        during `phase` ("auto" or "teleop"). Return 0 for an unrecognized
        action rather than raising, so a Match never crashes mid-run on a
        field/scoring mismatch -- that's a config bug to catch in tests,
        not a runtime fault."""
        raise NotImplementedError


class TableScoringRules(ScoringRules):
    """Straightforward points_for backed by a {(action, phase): points}
    dict -- covers the common case without every game needing its own
    ScoringRules subclass."""

    def __init__(self, table: dict[tuple[str, str], float]):
        self.table = table

    def points_for(self, action: str, phase: str) -> float:
        return self.table.get((action, phase), 0.0)
