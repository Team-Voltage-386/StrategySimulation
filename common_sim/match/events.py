"""
Timestamped match event log. Feeds both live telemetry display and
analysis.metrics -- kept as plain dataclasses/dicts rather than a
game-specific schema so metrics extraction can filter by event `kind`
without common_sim needing to know what any of them mean.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class MatchEvent:
    timestamp: float
    kind: str            # "intake" | "deposit" | "score" | "phase_change" | ...
    data: dict = field(default_factory=dict)


class EventLog:
    def __init__(self):
        self._events: list[MatchEvent] = []

    def log(self, timestamp: float, kind: str, data: dict | None = None) -> MatchEvent:
        event = MatchEvent(timestamp, kind, data or {})
        self._events.append(event)
        return event

    def of_kind(self, kind: str) -> list[MatchEvent]:
        return [e for e in self._events if e.kind == kind]

    def __iter__(self):
        return iter(self._events)

    def __len__(self) -> int:
        return len(self._events)
