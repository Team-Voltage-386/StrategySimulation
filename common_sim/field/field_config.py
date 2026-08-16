"""
Game-agnostic field description. A game_specific package builds one
concrete FieldConfig instance describing that year's field; nothing in
common_sim ever hardcodes a particular game's layout.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Obstacle:
    """A static field obstacle (reef, stage truss, charge station, ...).
    Rendered and collided with as a solid polygon."""
    name: str
    vertices: tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class ScoringRegion:
    """A sensor zone that scores an un-held piece released into it, for
    whichever scoring action the robot targeted the deposit at.

    A single physical zone can support several distinct scoring actions
    (e.g. one reef face offering "l1_coral".."l4_coral") -- `actions`
    lists every action name valid at this location. Each action's point
    value (via ScoringRules, see match/scoring.py) and how long a robot
    takes to perform it (via RobotCharacteristics.deposit_time_by_action)
    are looked up by that name wherever it's used, so "l4_coral" means
    the same points/timing at every reef face that offers it.

    `piece_types`, if non-empty, restricts which piece types this region
    accepts; empty means "any piece type".

    `passive_scoring` distinguishes two real scoring mechanics that look
    identical to a point-in-polygon check but aren't: most FIRST scoring
    locations (REEF branches, PROCESSOR) require a robot to actually be
    in position performing the scoring action -- a piece can never just
    roll or bounce its way to points there. A few (like REEFSCAPE's NET)
    are scored by launching a piece from a distance; the piece only needs
    to land in the zone, with no robot presence required at landing time.
    False (default) means only an explicit robot deposit while engaged
    with this region (Match.deposit_region_for finds it) can score a
    piece here. True additionally allows a piece already carrying a
    matching target_action to score by drifting/rolling/flying into the
    region afterward, with no robot in position -- see Match._try_score."""
    name: str
    vertices: tuple[tuple[float, float], ...]
    actions: frozenset[str]
    piece_types: frozenset[str] = frozenset()
    passive_scoring: bool = False
    # Per-action cap on how many pieces this region can ever accept for
    # that action, e.g. {"l4_coral": 1} for a REEF branch that physically
    # only holds one CORAL. An action missing from this mapping (or a
    # None mapping entirely, the default) is unlimited -- so a game that
    # never sets this keeps scoring as many pieces as land there, exactly
    # like before this field existed. Enforced by Match, see
    # Match.region_full / Match._try_score.
    capacity_by_action: dict[str, int] | None = None
    # Which alliance owns this region ("red"/"blue"), None if it's
    # neutral/shared. world_view.scoring_options and Match.deposit_region_for
    # both filter on this against Robot.alliance, so a robot never plans to
    # (or actually does) score on the opposing alliance's regions.
    alliance: str | None = None


@dataclass(frozen=True)
class IntakeLocation:
    """A zone with an unlimited, continuously-replenished supply of one
    piece type -- e.g. a human-player feeder/loading station. Unlike a
    PieceSpawnRegion, no physical GamePiece needs to exist in the zone
    beforehand: a robot that sits in the zone with intake commanded
    active for `dispense_time` seconds gets a new piece handed directly
    into its held pieces (see Match._register_collision_handlers /
    Robot.update_station_intake), the same way a real feeder keeps
    handing off pieces as fast as a robot can take them.

    `starting_pieces`, if set, caps the total number of pieces this
    location can ever dispense over a match (Match tracks the remaining
    count and stops dispensing at 0); None means unlimited.

    `piece_color`, if set, is passed through to a dispensed piece's
    GamePiece.color -- without it, a station-dispensed piece gets no
    color (GamePiece.color defaults to None) and a GUI falls back to its
    own placeholder color, which can visibly mismatch same-type pieces
    spawned elsewhere (e.g. field piles) with an explicit color."""
    name: str
    vertices: tuple[tuple[float, float], ...]
    piece_type: str
    dispense_time: float
    starting_pieces: int | None = None
    piece_color: str | None = None
    # Which alliance this station belongs to ("red"/"blue"), None if
    # shared. world_view.station_options filters on this against
    # Robot.alliance, mirroring ScoringRegion.alliance.
    alliance: str | None = None


@dataclass(frozen=True)
class PieceSpawnRegion:
    """Where game pieces originate. `is_station` marks a human-player
    station (unlimited supply, uses a robot's station_intake_time) as
    opposed to a field pile (finite pieces, uses intake_time)."""
    name: str
    vertices: tuple[tuple[float, float], ...]
    piece_type: str
    is_station: bool = False
    count: int | None = None  # None = unlimited (typical for a station)


@dataclass(frozen=True)
class FieldConfig:
    width: float
    height: float
    obstacles: tuple[Obstacle, ...] = field(default_factory=tuple)
    scoring_regions: tuple[ScoringRegion, ...] = field(default_factory=tuple)
    spawn_regions: tuple[PieceSpawnRegion, ...] = field(default_factory=tuple)
    intake_locations: tuple[IntakeLocation, ...] = field(default_factory=tuple)


def polygon_centroid(vertices: tuple[tuple[float, float], ...]) -> tuple[float, float]:
    """Simple average-of-vertices centroid -- good enough for placing
    spawned pieces within a region; not an area-weighted centroid."""
    xs = [v[0] for v in vertices]
    ys = [v[1] for v in vertices]
    return (sum(xs) / len(xs), sum(ys) / len(ys))


def polygon_area(vertices: tuple[tuple[float, float], ...]) -> float:
    """Shoelace area, always positive -- callers describe regions in
    whichever winding order reads naturally, and none of them care about
    orientation."""
    total = 0.0
    x1, y1 = vertices[-1]
    for x2, y2 in vertices:
        total += x1 * y2 - x2 * y1
        x1, y1 = x2, y2
    return abs(total) / 2.0


def point_in_polygon(point: tuple[float, float], vertices: tuple[tuple[float, float], ...]) -> bool:
    """Standard ray-casting point-in-polygon test."""
    x, y = point
    inside = False
    n = len(vertices)
    x1, y1 = vertices[-1]
    for i in range(n):
        x2, y2 = vertices[i]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
        x1, y1 = x2, y2
    return inside
