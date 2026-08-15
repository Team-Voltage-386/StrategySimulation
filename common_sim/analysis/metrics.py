"""
Per-match metric extraction from a completed Match's event log. Reads
only the generic event kinds Match itself logs ("score"/"intake"/
"deposit") -- game-specific metrics belong in a game_specific package,
built on top of this or read directly from match.events.
"""
from __future__ import annotations

from dataclasses import dataclass

from common_sim.match.match import Match


@dataclass(frozen=True)
class MatchMetrics:
    final_scores: dict
    pieces_scored: int
    pieces_intaked: int
    pieces_deposited: int
    misses: int              # deposits that did not result in a score
    cycle_times: list        # seconds between consecutive scores
    mean_cycle_time: float | None


def extract_metrics(match: Match) -> MatchMetrics:
    score_events = match.events.of_kind("score")
    intake_events = match.events.of_kind("intake")
    deposit_events = match.events.of_kind("deposit")

    times = [e.timestamp for e in score_events]
    cycle_times = [b - a for a, b in zip(times, times[1:])]
    mean_cycle_time = sum(cycle_times) / len(cycle_times) if cycle_times else None

    return MatchMetrics(
        final_scores=dict(match.scores),
        pieces_scored=len(score_events),
        pieces_intaked=len(intake_events),
        pieces_deposited=len(deposit_events),
        misses=len(deposit_events) - len(score_events),
        cycle_times=cycle_times,
        mean_cycle_time=mean_cycle_time,
    )
