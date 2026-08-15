"""
Read-only query surface over a live Match, for triggers/tactics/planning
to answer "what's out there" without hand-rolling field-state scans
themselves. Every function is duck-typed on `match` (TYPE_CHECKING-only
import of Match) so this module -- and everything built on it -- stays
decoupled from match.py and unit-testable against a stub object exposing
just the attributes it reads.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from common_sim.field.field_config import IntakeLocation, ScoringRegion, polygon_centroid
from common_sim.field.game_piece import GamePiece
from common_sim.robot.characteristics import SIDES
from common_sim.robot.robot import Robot

if TYPE_CHECKING:  # pragma: no cover
    from common_sim.match.match import Match


def collectable_pieces(
    match,
    *,
    piece_type: str | None = None,
    alliance: str | None = None,
    exclude_held: bool = True,
) -> list[GamePiece]:
    """Un-scored pieces currently on the field, optionally filtered by
    type and by which alliance last held them (a piece dropped without
    scoring keeps `last_holder_alliance`; a never-held field/station
    piece has it as None and only matches `alliance=None`)."""
    pieces = []
    for piece in match.active_pieces:
        if piece.scored:
            continue
        if exclude_held and piece.held_by is not None:
            continue
        if piece_type is not None and piece.piece_type != piece_type:
            continue
        if alliance is not None and piece.last_holder_alliance != alliance:
            continue
        pieces.append(piece)
    return pieces


@dataclass(frozen=True)
class Cluster:
    centroid: tuple[float, float]
    pieces: tuple[GamePiece, ...]
    count: int


def piece_clusters(match, pieces: list[GamePiece], radius: float) -> list[Cluster]:
    """Greedy radius clustering: repeatedly pull out the piece with the
    most as-yet-unclustered neighbors within `radius` (inclusive) and
    group it with them, until every piece has been assigned to exactly
    one cluster. Favors dense groups first, which is what "densest"
    collection mode wants -- this is not a general density-based
    clustering algorithm (no notion of transitively chained neighbors)."""
    remaining = list(pieces)
    clusters: list[Cluster] = []

    while remaining:
        best_idx = 0
        best_neighbors: list[int] = []
        for i, anchor in enumerate(remaining):
            neighbors = [
                j for j, other in enumerate(remaining)
                if j != i and _distance(anchor.position, other.position) <= radius
            ]
            if len(neighbors) > len(best_neighbors):
                best_idx, best_neighbors = i, neighbors

        member_indices = sorted({best_idx, *best_neighbors})
        members = tuple(remaining[i] for i in member_indices)
        xs = [p.position.x for p in members]
        ys = [p.position.y for p in members]
        centroid = (sum(xs) / len(members), sum(ys) / len(members))
        clusters.append(Cluster(centroid=centroid, pieces=members, count=len(members)))

        remaining = [p for i, p in enumerate(remaining) if i not in member_indices]

    return clusters


def _distance(a, b) -> float:
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5


def station_options(match, robot: Robot) -> list[IntakeLocation]:
    """Intake locations with remaining supply whose piece type `robot`
    has a side configured to intake, and capacity left for."""
    options = []
    for location in match.field.intake_locations:
        remaining = match.station_supply.get(location, 1)
        if remaining <= 0:
            continue
        if not any(robot.characteristics.side_intake_accepts(side, location.piece_type) for side in SIDES):
            continue
        held_of_type = sum(1 for p in robot.held_pieces if p.piece_type == location.piece_type)
        if held_of_type >= robot.characteristics.capacity_for(location.piece_type):
            continue
        options.append(location)
    return options


@dataclass(frozen=True)
class LegalScoringOption:
    """A (region, action, piece) triple a robot is legally able to
    execute right now -- membership only, no value judgement. Kept
    separate from planning.ScoringOption (which adds points/travel_time/
    deposit_time) because computing those needs navigation +
    scoring_rules + characteristics, which is planning.py's job, not
    world_view's."""
    region: ScoringRegion
    action: str
    piece: GamePiece


def scoring_options(match, robot: Robot) -> list[LegalScoringOption]:
    """Every (region, action) pair legal for a piece `robot` holds:
    region accepts the type, action is in region.actions, the
    region/action isn't full (via `match.region_full`, if the Match
    implementation provides it -- treated as never-full otherwise), and
    robot has a scoring side configured for that type."""
    region_full = getattr(match, "region_full", None)
    options = []
    for piece in robot.held_pieces:
        if not _robot_can_score(robot, piece.piece_type):
            continue
        for region in match.field.scoring_regions:
            if region.piece_types and piece.piece_type not in region.piece_types:
                continue
            for action in region.actions:
                if region_full is not None and region_full(region, action):
                    continue
                options.append(LegalScoringOption(region=region, action=action, piece=piece))
    return options


def _robot_can_score(robot: Robot, piece_type: str) -> bool:
    return any(robot.characteristics.side_score_accepts(side, piece_type) for side in SIDES)


def opponents(match, alliance: str) -> list[Robot]:
    return [r for r in match.robots if r.alliance != alliance]


def partners(match, alliance: str) -> list[Robot]:
    return [r for r in match.robots if r.alliance == alliance]


def region_by_name(match, name: str) -> ScoringRegion | None:
    for region in match.field.scoring_regions:
        if region.name == name:
            return region
    return None


def region_centroid(region: ScoringRegion) -> tuple[float, float]:
    return polygon_centroid(region.vertices)
