"""
Read-only query surface over a live Match, for triggers/tactics/planning
to answer "what's out there" without hand-rolling field-state scans
themselves. Every function is duck-typed on `match` (TYPE_CHECKING-only
import of Match) so this module -- and everything built on it -- stays
decoupled from match.py and unit-testable against a stub object exposing
just the attributes it reads.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

from common_sim.field.field_config import (
    IntakeLocation,
    ScoringRegion,
    point_in_polygon,
    polygon_area,
    polygon_centroid,
)
from common_sim.control.navigation import polygon_distance
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
    robot: Robot | None = None,
) -> list[GamePiece]:
    """Un-scored pieces currently on the field, optionally filtered by
    type and by which alliance last held them (a piece dropped without
    scoring keeps `last_holder_alliance`; a never-held field/station
    piece has it as None and only matches `alliance=None`). `robot`, when
    given, additionally drops pieces `robot` has no side configured to
    intake from the field (a side wired for "station" only doesn't make
    a loose field piece collectable) -- otherwise a Collect tactic would
    target and drive at a piece it physically can never pick up."""
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
        if robot is not None and not any(
            robot.characteristics.side_intake_accepts(side, piece.piece_type, source="field") for side in SIDES
        ):
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


def station_has_supply(match, station: IntakeLocation) -> bool:
    """Whether `station` still has anything to dispense.

    Defaults to True for a station the match does not track at all -- an
    unmetered human player, or a duck-typed match in a test -- because
    "no entry in the table" means unlimited here, not empty.

    Split out of `station_options` so a tactic already committed to a
    station can re-ask the one part of that legality that can go stale
    underneath it. It can and does: a REEFSCAPE REEF ALGAE position
    holds exactly one, and a robot that arrives to find its target
    already taken is standing on furniture.

    The sim has always known this -- `Match.step` gates
    `Robot.update_station_intake` on the same counter and simply
    declines to dispense. What was missing was any way for the control
    layer to ask, so a committed tactic ran an intake that could never
    complete against a patience clock that had stopped."""
    return match.station_supply.get(station, 1) > 0


def station_options(match, robot: Robot) -> list[IntakeLocation]:
    """Intake locations with remaining supply whose piece type `robot`
    has a side configured to intake, and capacity left for."""
    options = []
    for location in match.field.intake_locations:
        if location.alliance is not None and location.alliance != robot.alliance:
            continue
        if not station_has_supply(match, location):
            continue
        if not any(
            robot.characteristics.side_intake_accepts(side, location.piece_type, source="station") for side in SIDES
        ):
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
    separate from utility.ScoringOption (which adds points/travel_time/
    deposit_time) because computing those needs navigation +
    scoring_rules + characteristics, which is utility.py's job, not
    world_view's.

    `piece` is None when the option was enumerated for a piece *type*
    rather than a physical piece -- a caller asking what a piece it has
    not collected yet would be worth (see `scoring_slots_for_type`)."""
    region: ScoringRegion
    action: str
    piece: GamePiece | None


def scoring_slots_for_type(match, robot: Robot, piece_type: str) -> list[tuple[ScoringRegion, str]]:
    """Every (region, action) pair a piece of `piece_type` could legally
    be scored in by `robot`: region accepts the type, action is in
    region.actions, the region/action isn't full (via
    `match.region_full`, if the Match implementation provides it --
    treated as never-full otherwise), and robot has a scoring side
    configured for that type.

    Asked by type rather than by piece so a caller can price a piece it
    does not have yet -- "is that station worth driving to" is answered
    by where the thing it dispenses could go. `scoring_options` is this
    same rule applied to each held piece, so the two cannot drift."""
    if not _robot_can_score(robot, piece_type):
        return []
    region_full = getattr(match, "region_full", None)
    slots = []
    for region in match.field.scoring_regions:
        if region.alliance is not None and region.alliance != robot.alliance:
            continue
        if region.piece_types and piece_type not in region.piece_types:
            continue
        for action in region.actions:
            if region_full is not None and region_full(region, action):
                continue
            slots.append((region, action))
    return slots


def scoring_options(match, robot: Robot) -> list[LegalScoringOption]:
    """Every (region, action) pair legal for a piece `robot` holds.

    Emitted piece-major (each held piece's full slate of regions and
    actions, in field order, before the next piece's) -- callers rank
    these with `max`, and `max` keeps the first of equal keys, so the
    order is part of the behavior."""
    return [
        LegalScoringOption(region=region, action=action, piece=piece)
        for piece in robot.held_pieces
        for region, action in scoring_slots_for_type(match, robot, piece.piece_type)
    ]


def _robot_can_score(robot: Robot, piece_type: str) -> bool:
    return any(robot.characteristics.side_score_accepts(side, piece_type) for side in SIDES)


def own_side_test(match, alliance: str, *, margin_frac: float = 0.0):
    """Builds a `(x, y) -> bool` predicate answering "is this point on
    `alliance`'s own half of the field?".

    `margin_frac` pushes the dividing line that fraction of the distance
    between the two alliances' centroids into the *opponents'* half, so
    a caller can ask the stricter "is it well over the line?" instead of
    "is it past it at all" -- what you want when the answer flips a
    decision that's expensive to change your mind about.

    The halves are inferred from the field itself rather than declared,
    so this stays game-agnostic: take the centroid of every feature each
    alliance owns (its scoring regions and intake locations), split on
    whichever axis those two centroids are furthest apart along, and put
    the dividing line halfway between them. For REEFSCAPE that lands the
    line on the long axis at midfield, with each alliance's REEF, CORAL
    STATIONS and PROCESSOR on its own side -- which is the split a robot
    deciding whether a piece is worth crossing the field for cares about.

    Returned as a closure because the split is a property of the field,
    not of the point: a caller ranking a dozen candidate pieces computes
    it once instead of per piece.

    Degenerate fields -- one alliance owning nothing, or both alliances'
    features sharing a centroid -- have no discernible split, and every
    point is reported as own-side so that a caller gating on this simply
    does nothing rather than something arbitrary."""
    owned: dict[str, list[tuple[float, float]]] = {}
    for feature in (*match.field.scoring_regions, *match.field.intake_locations):
        if feature.alliance is not None:
            owned.setdefault(feature.alliance, []).append(polygon_centroid(feature.vertices))
    centroids = {
        side: (sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points))
        for side, points in owned.items()
    }

    ours = centroids.get(alliance)
    theirs = next((c for side, c in centroids.items() if side != alliance), None)
    if ours is None or theirs is None:
        return lambda x, y: True

    axis = 0 if abs(ours[0] - theirs[0]) >= abs(ours[1] - theirs[1]) else 1
    if abs(ours[axis] - theirs[axis]) < 1e-6:
        return lambda x, y: True

    midline = (ours[axis] + theirs[axis]) / 2.0 + margin_frac * (theirs[axis] - ours[axis])
    ours_is_low = ours[axis] < midline
    return lambda x, y: ((x, y)[axis] < midline) == ours_is_low


def opponents(match, alliance: str) -> list[Robot]:
    return [r for r in match.robots if r.alliance != alliance]


def partners(match, alliance: str) -> list[Robot]:
    return [r for r in match.robots if r.alliance == alliance]


def defenders(match, alliance: str) -> list[Robot]:
    """Opposing robots currently publishing a *defensive* intent (see
    strategy.Intent.defending) -- i.e. ones whose declared target is
    something they intend to take away rather than something they intend
    to produce with. A robot that merely happens to be standing in the
    way isn't one: it will move on when its own job is done, and
    treating it as a defender would have the whole alliance re-routing
    around ordinary traffic."""
    return [
        r for r in opponents(match, alliance)
        if getattr(getattr(r, "intent", None), "defending", False)
    ]


def defenders_against(match, robot: Robot) -> list[Robot]:
    """The subset of `defenders` aimed at `robot` specifically -- either
    marking it by name, or (for a defender that hasn't resolved a robot
    to mark) not aimed at anyone in particular and therefore potentially
    aimed at us. A defender explicitly marking a *teammate* is excluded:
    it's committed elsewhere, and reacting to it would have an entire
    alliance evade a defender that only ever had the bandwidth to deny
    one of them."""
    result = []
    for other in defenders(match, robot.alliance):
        marking = getattr(other.intent, "marking", None)
        if marking is robot or marking is None:
            result.append(other)
    return result


def is_protected(match, robot: Robot) -> bool:
    """Whether `robot` is standing somewhere opponents may not touch it
    (see field_config.ProtectedZone). Offense reads this to know it can
    finish an alignment unmolested; defense reads it to know that
    pressing any harder only donates points."""
    return match.protecting_zone(robot) is not None


def protection_distance(match, robot: Robot) -> float:
    """How far `robot` is from the nearest zone that would protect it:
    0.0 once any part of it is inside, and infinity on a field with no
    such zone. Lets a defender disengage *before* its mark crosses the
    line rather than at the moment it does -- a robot being leaned on
    carries the leaner in with it, and by then the foul has happened."""
    best = math.inf
    footprint = robot.footprint()
    for zone in getattr(match.field, "protected_zones", ()):
        if zone.alliance is not None and zone.alliance != robot.alliance:
            continue
        best = min(best, min(polygon_distance(p, zone.vertices) for p in footprint))
    return best


def pin_pressure(match, defender: Robot, mark: Robot) -> float:
    """How close `defender` is to fouling for pinning `mark`, as a
    fraction of the rule's limit: 0.0 when nothing is accumulated, 1.0 at
    the point a foul is charged. 0.0 on a field with no pin rule, so a
    caller can read it unconditionally.

    Expressed as a fraction rather than as seconds so a tactic can hold
    one release threshold across games whose limits differ."""
    rule = getattr(match.field, "pin_rule", None)
    if rule is None or rule.max_seconds <= 0:
        return 0.0
    return match.pin_seconds(defender, mark) / rule.max_seconds


def protection_keepout(defender: Robot, protected: Robot, margin: float = 2.0) -> float:
    """How far a defender's *center* must stay from a protected robot's
    center to be sure of not touching it. Half-diagonals sum, so it holds
    at every relative heading rather than only the square-on one -- a
    conservative circle around a rectangle, which is the right way to be
    wrong about a rule that costs points to break."""
    def half_diagonal(robot: Robot) -> float:
        return math.hypot(robot.characteristics.width, robot.characteristics.length) / 2.0

    return half_diagonal(defender) + half_diagonal(protected) + margin


def region_denied_by(match, region_name: str, alliance: str) -> list[Robot]:
    """Opposing defenders declaring `region_name` as the thing they're
    denying. Distinct from `region_occupants`, which counts anybody
    claiming the region for any reason -- a caller deciding whether to
    *wait* wants occupants, one deciding whether to *give up on the
    region entirely* wants this."""
    return [
        r for r in defenders(match, alliance)
        if getattr(r.intent, "target_region", None) == region_name
    ]


def alliance_scoring_regions(match, alliance: str) -> list[ScoringRegion]:
    """Scoring regions `alliance` owns. A region owned by nobody
    (`alliance is None`) belongs to both and is included for either."""
    return [
        r for r in match.field.scoring_regions
        if r.alliance is None or r.alliance == alliance
    ]


def alliance_intake_locations(match, alliance: str) -> list[IntakeLocation]:
    """Intake locations `alliance` may use, by ownership alone -- the
    mirror of `alliance_scoring_regions`. Unlike `station_options` this
    asks nothing about a particular robot's intakes or the remaining
    supply, because the caller is usually asking about an *opponent's*
    station, whose configuration is not ours to reason about."""
    return [
        s for s in match.field.intake_locations
        if s.alliance is None or s.alliance == alliance
    ]


def denial_target_by_name(match, name: str):
    """The named field feature a defender could deny -- a scoring region
    or an intake location, whichever carries the name.

    Both are just polygons a robot has to reach to do its job, and
    everything defense does with one (centroid, block segment, occupancy
    claim) is duck-typed on `.name`/`.vertices`, so there is no reason
    for the *lookup* to be the thing that decides which half of a cycle
    can be attacked. Games differ on which half is worth taking away:
    where scoring is protected but the feeder is not (REEFSCAPE), supply
    is the softer target, and a defender that can only name scoring
    regions is locked out of the only contact it is allowed to make."""
    region = region_by_name(match, name)
    if region is not None:
        return region
    for location in match.field.intake_locations:
        if location.name == name:
            return location
    return None


def denial_target_kind(match, feature) -> str:
    """"scoring" or "supply", for a caller that has a feature and needs
    to know which half of the opponent's cycle it belongs to."""
    return "supply" if feature in match.field.intake_locations else "scoring"


def likely_denial_target(match, robot: Robot, kinds: tuple[str, ...] = ("scoring", "supply")):
    """Where `robot` is most plausibly headed next, restricted to the
    `kinds` of feature the asking defender is willing to deny.

    The published intent when there is one and it is of a wanted kind;
    otherwise a guess from what the robot is carrying -- holding a piece
    means it is going to score, empty-handed means it is going to load.
    That guess is the whole point of the function: a defender that waits
    for a declaration always arrives second, and an empty-handed
    opponent guessed to be heading for a *scoring* region gets marked at
    a place it has no reason to visit."""
    intent = getattr(robot, "intent", None)
    named = getattr(intent, "target_region", None) if intent is not None else None
    if named is not None:
        feature = denial_target_by_name(match, named)
        if feature is not None and denial_target_kind(match, feature) in kinds:
            return feature

    if not robot.held_pieces and "supply" in kinds:
        guess = _supply_guess(match, robot)
        if guess is not None:
            return guess
    if "scoring" in kinds:
        return likely_scoring_region(match, robot)
    if "supply" in kinds:
        return _supply_guess(match, robot)
    return None


def _supply_guess(match, robot: Robot):
    """Which supply point `robot` will next go to, for a defender that
    intends to be there first.

    Nearest is the wrong answer on its own, and expensively so. A field
    carries supply points of very different kinds -- REEFSCAPE has two
    corner CORAL STATIONS a robot cycles all match, and six one-ALGAE
    nooks tucked into the REEF itself -- and "nearest to where the mark
    is standing right now" picks a nook every time, because the mark is
    standing at the REEF. A defender then camps a pickup its mark has no
    intention of using.

    What it is carrying is the tell: a robot cycling CORAL is going back
    for CORAL, not detouring to the nearest thing that happens to
    dispense something. So prefer supply of a type the mark already
    holds, and fall back to what it could take right now for an
    empty-handed mark (which almost always has a declared target anyway,
    and rarely reaches this code).

    Note the base list is every station the mark's alliance owns, not
    `station_options` -- that query answers "what can this robot load
    from *now*", and a robot holding a CORAL is by definition full of
    CORAL, so the very stations it is cycling are the ones missing from
    it. Predicting where a robot goes next is not the same question as
    what it can do this instant."""
    stations = alliance_intake_locations(match, robot.alliance)
    if not stations:
        return None
    held = {p.piece_type for p in robot.held_pieces}
    wanted = (
        [s for s in stations if s.piece_type in held] if held
        else station_options(match, robot)
    )
    return _nearest_by_centroid(wanted or stations, robot)


def _nearest_by_centroid(features, robot: Robot):
    origin = (robot.pose.x, robot.pose.y)
    return min(
        features,
        key=lambda f: math.hypot(
            polygon_centroid(f.vertices)[0] - origin[0], polygon_centroid(f.vertices)[1] - origin[1]
        ),
    )


def likely_scoring_region(match, robot: Robot) -> ScoringRegion | None:
    """Where `robot` would most plausibly go to score next, for a caller
    that has to guess -- a defender deciding which side of its mark to
    sit on before that mark has declared anything.

    Its own published intent when it has one, otherwise the nearest
    region its alliance can score in. Deliberately a guess and not a
    prediction: getting it roughly right is enough to be on the correct
    side of an opponent, and nothing here needs more than that."""
    intent = getattr(robot, "intent", None)
    named = getattr(intent, "target_region", None) if intent is not None else None
    if named is not None:
        region = region_by_name(match, named)
        if region is not None:
            return region

    regions = alliance_scoring_regions(match, robot.alliance)
    if not regions:
        return None
    origin = (robot.pose.x, robot.pose.y)
    return min(
        regions,
        key=lambda r: math.hypot(
            polygon_centroid(r.vertices)[0] - origin[0], polygon_centroid(r.vertices)[1] - origin[1]
        ),
    )


def region_by_name(match, name: str) -> ScoringRegion | None:
    for region in match.field.scoring_regions:
        if region.name == name:
            return region
    return None


def region_centroid(region: ScoringRegion) -> tuple[float, float]:
    return polygon_centroid(region.vertices)


def region_robot_capacity(region: ScoringRegion, robot: Robot) -> int:
    """How many robots can plausibly work `region` at the same time, from
    its area against `robot`'s own footprint. A REEFSCAPE REEF face's
    scoring zone is barely wider than one robot (capacity 1 -- a second
    robot heading there has nowhere to stand that isn't where the first
    one already is); a zone like the NET spans most of the field and fits
    several side by side.

    Area-over-footprint is deliberately crude: it ignores region shape (a
    long thin corridor packs worse than a square of equal area) and the
    fact that robots score from a standoff rather than parking inside.
    All it has to separate is "one robot at a time" from "plenty of
    room", which is the only distinction that decides whether a second
    robot should go find a different region. Never returns less than 1 --
    a region too small for even one robot to stand in is still a region
    exactly one robot scores at."""
    characteristics = robot.characteristics
    footprint = max(1e-6, characteristics.width * characteristics.length)
    return max(1, int(polygon_area(region.vertices) // footprint))


def region_occupants(match, region: ScoringRegion, *, exclude: Robot | None = None) -> list[Robot]:
    """Robots currently working `region`: standing in it, or publishing an
    `intent.target_region` naming it (on their way there to score, or
    parked on it to defend it). Intent counts as much as position --
    two robots that both pick the same region commit to it long before
    either arrives, and by the time they're both physically in it they're
    already nose to nose, which is exactly what a caller checking this is
    trying to avoid. Not filtered by alliance: an opponent sitting in a
    region blocks it just as thoroughly as a partner does."""
    occupants = []
    for other in match.robots:
        if other is exclude:
            continue
        intent = getattr(other, "intent", None)
        claimed = getattr(intent, "target_region", None) if intent is not None else None
        if claimed == region.name or point_in_polygon((other.pose.x, other.pose.y), region.vertices):
            occupants.append(other)
    return occupants


def region_has_room(match, region: ScoringRegion, robot: Robot) -> bool:
    """Whether `robot` can join `region` without crowding whoever is
    already working it."""
    return len(region_occupants(match, region, exclude=robot)) < region_robot_capacity(region, robot)


# Grid resolution for region_approach_point's search over a region's
# interior. 7x7 is enough to find a well-separated spot in a big zone
# without the cost mattering (this runs at most once per replan period).
_APPROACH_SAMPLES = 7


def region_approach_point(region: ScoringRegion, robot: Robot, occupants: list[Robot]) -> tuple[float, float]:
    """Where inside `region` `robot` should drive to. The centroid when
    it has the region to itself; otherwise the interior point that clears
    the other occupants, preferring the closest such point to `robot`
    rather than the single farthest one (which would send it to a far
    corner of a big zone for no benefit). Without this, two robots
    sharing a region big enough for both would still aim at the identical
    centroid and shove each other over it.

    A teammate and an opponent are wanted at different distances, so the
    two are weighed differently. A teammate only has to be clear of:
    past a footprint diagonal there is nothing more to gain, and the
    nearest adequate point saves the drive. An opponent should be as far
    away as the region allows, uncapped, because the whole value of a
    large scoring region is that it cannot be denied at one spot -- and
    a defender parked in the middle of it denies the middle only.

    This was tried once before and measured as a catastrophic loss (1v1
    against a full-time blocker, ALGAE production fell 19.0 -> 6.0), and
    the reading at the time was that a defender follows, so the far
    corner isn't open by the time you get there. That reading was wrong:
    the robot was not being followed to the far corner, it was arriving
    and failing to score, because `Score` drove on past its legal pose
    and the facing test measured against the region's centroid. On the
    fixed code the same change is worth 48.9 -> 71.0 on that bench, and
    71.0 -> 95.4 alongside Score's failed-target cooldown."""
    centroid = polygon_centroid(region.vertices)
    if not occupants:
        return centroid

    friendly = [(r.pose.x, r.pose.y) for r in occupants if r.alliance == robot.alliance]
    hostile = [(r.pose.x, r.pose.y) for r in occupants if r.alliance != robot.alliance]

    characteristics = robot.characteristics
    enough = math.hypot(characteristics.width, characteristics.length)
    origin = (robot.pose.x, robot.pose.y)

    xs = [v[0] for v in region.vertices]
    ys = [v[1] for v in region.vertices]
    min_x, max_x, min_y, max_y = min(xs), max(xs), min(ys), max(ys)

    best, best_key = centroid, _spread_key(centroid, friendly, hostile, origin, enough)
    for i in range(_APPROACH_SAMPLES):
        for j in range(_APPROACH_SAMPLES):
            point = (
                min_x + (i + 0.5) / _APPROACH_SAMPLES * (max_x - min_x),
                min_y + (j + 0.5) / _APPROACH_SAMPLES * (max_y - min_y),
            )
            if not point_in_polygon(point, region.vertices):
                continue
            key = _spread_key(point, friendly, hostile, origin, enough)
            if key > best_key:
                best, best_key = point, key
    return best


def _spread_key(point, friendly, hostile, origin, enough: float):
    """Sort key for a candidate approach point, larger is better:
    adequate clearance from teammates first, then distance from the
    nearest opponent, then closeness to the robot. With no opponent in
    the region the second term is constant and the tiebreak is the
    shorter drive, which is the un-contested behavior."""
    friendly_clear = min(
        (math.hypot(point[0] - o[0], point[1] - o[1]) for o in friendly), default=enough,
    )
    hostile_clear = min(
        (math.hypot(point[0] - o[0], point[1] - o[1]) for o in hostile), default=math.inf,
    )
    if hostile:
        return (min(friendly_clear, enough), hostile_clear)
    return (min(friendly_clear, enough), -math.hypot(point[0] - origin[0], point[1] - origin[1]))
