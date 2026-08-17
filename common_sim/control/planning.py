"""
The "what's worth the most" model: turns world_view's legal scoring
options into ranked, valued ones, and lets a robot holding several
pieces plan a chain across them. Kept separate from world_view because
computing value needs scoring_rules + characteristics + navigation, none
of which world_view depends on.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

from common_sim.control import navigation, world_view
from common_sim.field.field_config import ScoringRegion, polygon_centroid
from common_sim.field.game_piece import GamePiece

if TYPE_CHECKING:  # pragma: no cover
    from common_sim.robot.robot import Robot


@dataclass(frozen=True)
class ScoringOption:
    region: ScoringRegion
    action: str
    piece: GamePiece
    points: float          # scoring_rules.points_for(action, phase)
    deposit_time: float    # characteristics.deposit_duration(action)
    travel_time: float     # navigation.estimate_travel_time(...)

    @property
    def value_rate(self) -> float:
        return self.points / max(1e-6, self.travel_time + self.deposit_time)


class ScorePlanner(ABC):
    @abstractmethod
    def plan(self, match, robot: "Robot", exclude: set | None = None) -> list[ScoringOption]:
        """Ordered scoring options for every piece `robot` currently
        holds -- the tactic executes the head and replans after each
        deposit, so only the order (not that every option remains valid
        forever) matters.

        `exclude` is a set of (region name, action) pairs the caller has
        reason not to pick again right now -- Score's failed-target
        cooldown. It is a *preference*, not a filter: a piece whose every
        option is excluded still gets its best one, because a robot
        standing still holding a piece it refuses to place scores less
        than one contesting a spot it probably can't have."""
        raise NotImplementedError


def build_option(match, robot: "Robot", legal: world_view.LegalScoringOption, from_pos: tuple[float, float]) -> ScoringOption:
    """Turn a world_view.LegalScoringOption into a valued ScoringOption
    from `from_pos`. Public (not just GreedyRatePlanner-internal) so a
    caller pinning a specific region/action (Score) can value just that
    one candidate without going through a planner's own choice logic."""
    points = match.scoring_rules.points_for(legal.action, match.phase.value)
    deposit_time = robot.characteristics.deposit_duration(legal.action)
    goal = polygon_centroid(legal.region.vertices)
    travel_time = navigation.estimate_travel_time(match.field, from_pos, goal, robot.characteristics)
    return ScoringOption(
        region=legal.region, action=legal.action, piece=legal.piece,
        points=points, deposit_time=deposit_time, travel_time=travel_time,
    )


class GreedyRatePlanner(ScorePlanner):
    """Sorts legal options by value_rate (points per second). For a
    robot holding several pieces, chains greedily: pick the best first
    option, advance a virtual pose/clock to that region, then re-pick
    the best option for the *next* held piece from there -- so the
    order accounts for how many pieces the robot carries, their value,
    and how long it'll take to score all of them, without needing a
    combinatorial search.

    Route congestion/blockage is deliberately excluded here -- it would
    enter as an extra term in travel_time once navigation can report a
    blocked path, but that isn't modeled yet."""

    def plan(self, match, robot: "Robot", exclude: set | None = None) -> list[ScoringOption]:
        remaining_pieces = list(robot.held_pieces)
        pos = (robot.pose.x, robot.pose.y)
        ordered: list[ScoringOption] = []

        while remaining_pieces:
            candidates = []
            for legal in world_view.scoring_options(match, robot):
                if legal.piece not in remaining_pieces:
                    continue
                candidates.append(build_option(match, robot, legal, pos))
            if not candidates:
                break
            # Applied per piece, not to the finished plan: a robot
            # holding one piece has exactly one candidate per region, so
            # filtering afterward would always hit the fallback and the
            # exclusion would silently do nothing.
            wanted = [o for o in candidates if (o.region.name, o.action) not in exclude] if exclude else []
            best = max(wanted or candidates, key=lambda o: o.value_rate)
            ordered.append(best)
            remaining_pieces.remove(best.piece)
            pos = polygon_centroid(best.region.vertices)

        return ordered


class LookaheadPlanner(ScorePlanner):
    """Stub for real multi-step planning (e.g. weighing whether to
    detour for a denser pickup before scoring) -- same interface as
    GreedyRatePlanner, so plugging it in later never touches Score."""

    def plan(self, match, robot: "Robot", exclude: set | None = None) -> list[ScoringOption]:
        raise NotImplementedError("LookaheadPlanner is a seam for future work, not implemented yet")
