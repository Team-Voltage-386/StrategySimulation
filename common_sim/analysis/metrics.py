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
    # Same two figures split by alliance. Whole-match totals answer
    # "how productive was this configuration"; only the split answers
    # "what did one alliance do to the other", which is the entire
    # question when anything on the field is playing defense -- a
    # defender's effect shows up as the *opponent's* rate falling, and
    # is invisible in a number that adds both alliances together.
    pieces_scored_by_alliance: dict
    mean_cycle_time_by_alliance: dict
    # alliance -> protected-zone contact violations its robots committed
    # (see field_config.ProtectedZone). A defense tuning that only reads
    # the opponent's score can't tell denial from fouling, since both
    # look like a defender standing on top of somebody.
    protection_fouls_by_alliance: dict
    # alliance -> pin violations its robots committed (see
    # field_config.PinRule). Separate from the above because they measure
    # opposite failures of the same defender: a protection foul is
    # touching somebody where touching is forbidden, a pin foul is
    # touching somebody too *long* where touching is fine.
    pin_fouls_by_alliance: dict


def extract_metrics(match: Match) -> MatchMetrics:
    score_events = match.events.of_kind("score")
    intake_events = match.events.of_kind("intake")
    deposit_events = match.events.of_kind("deposit")

    times = [e.timestamp for e in score_events]
    cycle_times = [b - a for a, b in zip(times, times[1:])]
    mean_cycle_time = sum(cycle_times) / len(cycle_times) if cycle_times else None

    by_alliance: dict[str, list[float]] = {}
    for event in score_events:
        by_alliance.setdefault(event.data.get("alliance", "unknown"), []).append(event.timestamp)

    return MatchMetrics(
        final_scores=dict(match.scores),
        pieces_scored=len(score_events),
        pieces_intaked=len(intake_events),
        pieces_deposited=len(deposit_events),
        misses=len(deposit_events) - len(score_events),
        cycle_times=cycle_times,
        mean_cycle_time=mean_cycle_time,
        pieces_scored_by_alliance={a: len(t) for a, t in by_alliance.items()},
        mean_cycle_time_by_alliance={
            alliance: (sum(b - a for a, b in zip(stamps, stamps[1:])) / (len(stamps) - 1))
            for alliance, stamps in by_alliance.items() if len(stamps) > 1
        },
        protection_fouls_by_alliance=dict(match.protection_fouls),
        pin_fouls_by_alliance=dict(match.pin_fouls),
    )
