"""
Ordering a robot's held pieces into a scoring plan.

The *valuing* half of this job now lives in utility.py, which prices any
candidate action -- a deposit or a pickup -- in the same points-per-second
currency. What is left here is the part that is specifically about a plan:
given several pieces in hand, which one goes where, and in what order.

`ScoringOption` and `build_option` are re-exported rather than moved out
of sight, because they were this module's public surface long before
utility.py existed and Score still imports them from here.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

from common_sim.control import utility
from common_sim.control.utility import ScoringOption, build_option
from common_sim.field.field_config import polygon_centroid

if TYPE_CHECKING:  # pragma: no cover
    from common_sim.robot.robot import Robot

__all__ = ["ScoringOption", "build_option", "ScorePlanner", "GreedyRatePlanner", "LookaheadPlanner"]


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


class GreedyRatePlanner(ScorePlanner):
    """Sorts candidates by value_rate (points per second). For a robot
    holding several pieces, chains greedily: pick the best first option,
    advance a virtual pose/clock to that region, then re-pick the best
    option for the *next* held piece from there -- so the order accounts
    for how many pieces the robot carries, their value, and how long
    it'll take to score all of them, without needing a combinatorial
    search.

    Ranks on `value_rate` alone. `Outcome` also carries a success
    probability and, for pickups, a lookahead; neither is consulted here.
    Weighing those is a change to what the robot does, and belongs to a
    caller that can be measured against this one.

    Route congestion/blockage is deliberately excluded here -- it would
    enter as an extra term in travel_time once navigation can report a
    blocked path, but that isn't modeled yet."""

    def plan(self, match, robot: "Robot", exclude: set | None = None) -> list[ScoringOption]:
        remaining_pieces = list(robot.held_pieces)
        pos = (robot.pose.x, robot.pose.y)
        ordered: list[ScoringOption] = []

        while remaining_pieces:
            candidates = [
                outcome.payload
                for outcome in utility.score_outcomes(match, robot, pos, pieces=remaining_pieces)
            ]
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
    GreedyRatePlanner, so plugging it in later never touches Score.

    utility.collect_outcomes is the piece this was waiting on: it prices
    a detour in the same units as a deposit. What is still missing is the
    policy for spending that information."""

    def plan(self, match, robot: "Robot", exclude: set | None = None) -> list[ScoringOption]:
        raise NotImplementedError("LookaheadPlanner is a seam for future work, not implemented yet")
