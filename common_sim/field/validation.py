"""
Static checks on a `FieldConfig`, before anybody runs a match on it.

Every problem this module reports is one that otherwise produces a
*plausible-looking wrong match* rather than an error: a scoring region
with no legal approach, a piece type nothing knows the size of, an
action worth nothing, an emitter linked to a station that isn't there.
The SALVAGE dry run lost an evening to exactly one of those and it
presented as "a strategy is losing" -- see DRY_RUN_LOG.md, F7. On
game-reveal day, when the field is new and the strategies are new and
both are suspect at once, a check that separates "the field is wrong"
from "the plan is wrong" in milliseconds is worth more than it costs.

**This deliberately imports `control.navigation`**, inverting the usual
field-below-control direction. It has to: the question "can a robot park
somewhere that scores here" is only meaningful if it is asked with the
same geometry the robot will actually use. A validator with its own
private idea of standoffs and inflation is a second opinion that can
disagree with the simulator, which is worse than no opinion at all.

Nothing here is called automatically. A game package calls it from its
own tests (see `test/game_specific/`), and a tool can call it before a
long run.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from common_sim.control.navigation import (
    convex_overlap, footprint_polygon, plan_path, polygon_distance,
)
from common_sim.field.field_config import (
    FieldConfig, polygon_area, polygon_centroid, point_in_polygon, polygons_intersect,
)
from common_sim.field.game_piece import has_piece_spec

ERROR = "error"
WARNING = "warning"
NOTE = "note"

_SEVERITY_ORDER = {ERROR: 0, WARNING: 1, NOTE: 2}

# Bearings tried when asking whether a feature has *any* legal approach.
# Deliberately finer than `navigation._STANDOFF_SAMPLES` (24): this runs
# once, offline, and the question here is existence rather than choice --
# a feature that only one narrow approach angle reaches is reachable, and
# reporting it as unreachable because the coarse grid missed that angle
# would be a false alarm in the one place false alarms are expensive.
_APPROACH_BEARINGS = 72


@dataclass(frozen=True)
class FieldProblem:
    """One thing wrong (or suspicious) about a field.

    `severity` is `ERROR` for something that makes a match meaningless,
    `WARNING` for something almost certainly a mistake but survivable,
    and `NOTE` for geometry worth a second look that may well be
    intended."""
    severity: str
    where: str
    problem: str
    hint: str = ""

    def __str__(self) -> str:
        line = f"{self.severity}: {self.where}: {self.problem}"
        return f"{line}\n  {self.hint}" if self.hint else line


def describe_problems(problems: list[FieldProblem]) -> str:
    """Problems as one readable block, worst first. Empty string when
    there are none, so a caller can `print(describe_problems(...) or
    "field OK")`."""
    ordered = sorted(problems, key=lambda p: (_SEVERITY_ORDER.get(p.severity, 9), p.where))
    return "\n".join(str(p) for p in ordered)


def validate_field(
    field: FieldConfig,
    *,
    robot_width: float,
    robot_length: float,
    scoring_rules=None,
    reference_point: tuple[float, float] | None = None,
) -> list[FieldProblem]:
    """Every problem found on `field` for a robot of this size.

    `scoring_rules`, when given, additionally checks that every action
    some region offers is worth points in at least one phase -- the
    single most common way a new game's field and its point table drift
    apart, and one that shows up as robots ignoring a whole structure
    for no visible reason.

    `reference_point` is where the reachability check plans from,
    defaulting to the field centre. One origin is enough: the check is
    for a feature walled off from the field, not for a feature awkward
    from one particular corner."""
    problems: list[FieldProblem] = []
    problems += _check_names(field)
    problems += _check_polygons(field)
    problems += _check_piece_types(field)
    problems += _check_regions(field, scoring_rules)
    problems += _check_gates(field)
    problems += _check_emitters(field)
    problems += _check_secondary_awards(field, scoring_rules)
    problems += _check_reachability(field, robot_width, robot_length, reference_point)
    problems += _check_pinch_points(field, robot_width, robot_length)
    return problems


# -- naming ------------------------------------------------------------

def _named_features(field: FieldConfig):
    """(kind, feature) for everything on the field that carries a name.
    Strategies, benches and `Match` all address features by name, so a
    duplicate is not a cosmetic problem."""
    for o in field.obstacles:
        yield "obstacle", o
    for r in field.scoring_regions:
        yield "scoring region", r
    for s in field.intake_locations:
        yield "intake location", s
    for e in field.emitter_regions:
        yield "emitter", e
    for z in field.protected_zones:
        yield "protected zone", z
    for s in field.spawn_regions:
        yield "spawn region", s


def _check_names(field: FieldConfig) -> list[FieldProblem]:
    problems = []
    seen: dict[str, str] = {}
    for kind, feature in _named_features(field):
        if not feature.name:
            problems.append(FieldProblem(ERROR, f"<unnamed {kind}>", "has an empty name"))
            continue
        previous = seen.get(feature.name)
        if previous is not None:
            problems.append(FieldProblem(
                ERROR, feature.name,
                f"name is used by both a {previous} and a {kind}",
                "Strategies, Match.deposit_region_for and every bench address features by "
                "name; a duplicate resolves to whichever was declared first.",
            ))
        else:
            seen[feature.name] = kind
    return problems


# -- geometry ----------------------------------------------------------

def _check_polygons(field: FieldConfig) -> list[FieldProblem]:
    problems = []
    for kind, feature in _named_features(field):
        vertices = feature.vertices
        if len(vertices) < 3:
            problems.append(FieldProblem(
                ERROR, feature.name, f"{kind} has {len(vertices)} vertices, needs at least 3"))
            continue
        if polygon_area(vertices) <= 0.0:
            problems.append(FieldProblem(
                ERROR, feature.name, f"{kind} encloses no area"))
        outside = [v for v in vertices if not (0.0 <= v[0] <= field.width and 0.0 <= v[1] <= field.height)]
        if outside:
            problems.append(FieldProblem(
                WARNING, feature.name,
                f"{kind} has {len(outside)} vertex/vertices outside the {field.width:g}x{field.height:g} field",
                f"e.g. {outside[0]}. Nothing clips these -- the part outside simply never "
                "gets used, which makes the feature quietly smaller than it reads.",
            ))
    # A zone that clips a structure is usually fine and often deliberate
    # -- a REEFSCAPE REEF face's scoring zone sits against the REEF wall
    # and its corners cut into the hex. It is a NOTE, not a fault: the
    # part of the zone outside the structure is the part that gets used,
    # and if that part is too small to work with, the approach check
    # below says so outright. Reported only so that a region which turns
    # out to be hard to score has somewhere to start looking.
    for kind, features in (("scoring region", field.scoring_regions),
                           ("intake location", field.intake_locations)):
        for feature in features:
            for obstacle in field.obstacles:
                if polygons_intersect(feature.vertices, obstacle.vertices):
                    problems.append(FieldProblem(
                        NOTE, feature.name,
                        f"{kind} overlaps obstacle {obstacle.name!r}",
                        "Only the part outside the structure is usable, since a robot "
                        "cannot stand in one.",
                    ))
    return problems


# -- pieces and points -------------------------------------------------

def _referenced_piece_types(field: FieldConfig):
    for r in field.scoring_regions:
        for t in r.piece_types:
            yield t, r.name
    for s in field.intake_locations:
        yield s.piece_type, s.name
    for e in field.emitter_regions:
        yield e.piece_type, e.name
    for s in field.spawn_regions:
        yield s.piece_type, s.name


def _check_piece_types(field: FieldConfig) -> list[FieldProblem]:
    problems = []
    reported = set()
    for piece_type, where in _referenced_piece_types(field):
        if piece_type in reported or has_piece_spec(piece_type):
            continue
        reported.add(piece_type)
        problems.append(FieldProblem(
            ERROR, where,
            f"piece type {piece_type!r} has no registered GamePieceSpec",
            "Call game_piece.register_piece_spec for it (usually in the game's "
            "game_pieces.py) -- without one every spawn path has to invent a radius.",
        ))
    return problems


def _check_gates(field: FieldConfig) -> list[FieldProblem]:
    """ScoringRegion.blocked_until_collected sanity. Every failure here
    is one a match would otherwise express as a scoring action that
    quietly never becomes available -- which reads on a results table as
    a strategy choosing not to use it, not as a broken field."""
    problems = []
    locations_by_name = {s.name: s for s in field.intake_locations}
    for region in field.scoring_regions:
        for action, location_name in sorted((region.blocked_until_collected or {}).items()):
            if action not in region.actions:
                problems.append(FieldProblem(
                    ERROR, region.name,
                    f"blocked_until_collected gates {action!r}, which this region does not offer",
                    f"Its actions are {sorted(region.actions)}. The gate is dead: "
                    "Match.region_blocked only ever asks about an action it accepted.",
                ))
            location = locations_by_name.get(location_name)
            if location is None:
                problems.append(FieldProblem(
                    ERROR, region.name,
                    f"blocked_until_collected points {action!r} at unknown intake location {location_name!r}",
                    "Match resolves these by name at construction and raises on a miss, "
                    "so this field cannot start a match at all.",
                ))
            elif location.starting_pieces is None:
                problems.append(FieldProblem(
                    ERROR, region.name,
                    f"{action!r} is gated on {location_name!r}, which has unlimited supply",
                    "The gate opens when the location runs dry, and an unlimited location "
                    "never does -- so this action is unscoreable for the whole match. Give "
                    "the location a finite starting_pieces count.",
                ))
    return problems


def _check_regions(field: FieldConfig, scoring_rules) -> list[FieldProblem]:
    problems = []
    for region in field.scoring_regions:
        if not region.actions:
            problems.append(FieldProblem(
                ERROR, region.name, "scoring region offers no actions"))
        for action in sorted((region.capacity_by_action or {})):
            if action not in region.actions:
                problems.append(FieldProblem(
                    ERROR, region.name,
                    f"capacity_by_action caps {action!r}, which this region does not offer",
                    f"Its actions are {sorted(region.actions)}. The cap is dead: "
                    "Match.region_full only ever asks about an action it accepted.",
                ))
        if scoring_rules is not None:
            for action in sorted(region.actions):
                if all(scoring_rules.points_for(action, phase) == 0.0 for phase in ("auto", "teleop")):
                    problems.append(FieldProblem(
                        WARNING, region.name,
                        f"action {action!r} is worth 0 points in every phase",
                        "ScoringRules.points_for returns 0 for an unknown action rather than "
                        "raising, so a typo on either side looks exactly like this. A robot "
                        "will never choose it, and the whole region may go unused.",
                    ))
    for alliance_holder, kind in (
        [(r, "scoring region") for r in field.scoring_regions]
        + [(s, "intake location") for s in field.intake_locations]
        + [(z, "protected zone") for z in field.protected_zones]
    ):
        alliance = getattr(alliance_holder, "alliance", None)
        if alliance is not None and alliance not in ("red", "blue"):
            problems.append(FieldProblem(
                WARNING, alliance_holder.name,
                f"{kind} alliance is {alliance!r}, not 'red'/'blue'/None",
                "Ownership is compared against Robot.alliance by string equality, so this "
                "feature belongs to nobody and is filtered out for everyone.",
            ))
    return problems


def _check_emitters(field: FieldConfig) -> list[FieldProblem]:
    problems = []
    station_names = {s.name for s in field.intake_locations}
    region_names = {r.name for r in field.scoring_regions}
    for emitter in field.emitter_regions:
        linked = emitter.linked_collection_region
        if linked is not None and linked not in station_names:
            problems.append(FieldProblem(
                ERROR, emitter.name,
                f"linked_collection_region {linked!r} is not an intake location",
                f"Known intake locations: {sorted(station_names)}",
            ))
        if linked is not None and emitter.initial_capacity is not None:
            problems.append(FieldProblem(
                ERROR, emitter.name,
                "sets both linked_collection_region and initial_capacity",
                "They are mutually exclusive: a linked emitter draws down the station's "
                "own supply, so its own capacity would double-count the same pieces.",
            ))
        scored = emitter.linked_scoring_region
        if scored is not None and scored not in region_names:
            problems.append(FieldProblem(
                ERROR, emitter.name,
                f"linked_scoring_region {scored!r} is not a scoring region"))
        for start, end in emitter.active_times:
            if end <= start:
                problems.append(FieldProblem(
                    WARNING, emitter.name,
                    f"active window ({start:g}, {end:g}) never opens"))
    return problems


def _check_secondary_awards(field: FieldConfig, scoring_rules) -> list[FieldProblem]:
    actions = {action for r in field.scoring_regions for action in r.actions}
    problems = []
    for award in field.secondary_awards:
        where = f"secondary award on {award.action!r}"
        if award.action not in actions:
            problems.append(FieldProblem(
                WARNING, where,
                f"no scoring region offers action {award.action!r}",
                "This award can never trigger -- Match._try_score only rolls it from an "
                "actual scoring event on that action name.",
            ))
        if scoring_rules is not None and all(
            scoring_rules.points_for(award.award_action, phase) == 0.0 for phase in ("auto", "teleop")
        ):
            problems.append(FieldProblem(
                WARNING, where,
                f"award_action {award.award_action!r} is worth 0 points in every phase",
                "Match._try_score looks the award's value up via points_for(award_action, "
                "phase), so this award silently pays out nothing whenever it lands.",
            ))
        if award.alliance_of not in ("scoring", "opponent"):
            problems.append(FieldProblem(
                ERROR, where,
                f"alliance_of is {award.alliance_of!r}, not 'scoring'/'opponent'",
                "Match._try_score only checks for 'scoring'; anything else silently pays "
                "out to the opponent, same as a real typo would.",
            ))
        if not (0.0 <= award.probability <= 1.0):
            problems.append(FieldProblem(
                ERROR, where, f"probability is {award.probability!r}, must be in [0, 1]"))
        if award.delay < 0.0:
            problems.append(FieldProblem(
                ERROR, where, f"delay is {award.delay!r}, must be >= 0"))
    return problems


# -- reachability ------------------------------------------------------

def _has_legal_approach(field: FieldConfig, aim, robot_width: float, robot_length: float):
    """Whether some pose exists from which a robot can work `aim`.

    One question for scoring regions and intake locations alike, because
    the simulator asks one question of both: a deposit needs the scoring
    side's reach inside the zone (`Robot.side_engages_polygon`) and a
    station handoff needs the intake side's sensor shape overlapping it
    (`Match`'s `_INTAKE_TYPE`/`_STATION_TYPE` handler). Neither needs the
    chassis *centre* in the zone -- which matters, because a corner
    loading station is routinely half outside the field and no robot
    could ever put its centre on that centroid.

    Returns the first pose found, or None."""
    obstacles = [o.vertices for o in field.obstacles]
    distance = robot_length / 2.0
    for i in range(_APPROACH_BEARINGS):
        angle = 2.0 * math.pi * i / _APPROACH_BEARINGS
        center = (aim[0] + math.cos(angle) * distance, aim[1] + math.sin(angle) * distance)
        heading = angle + math.pi
        chassis = footprint_polygon(center, heading, robot_width, robot_length)
        if not all(0.0 <= x <= field.width and 0.0 <= y <= field.height for x, y in chassis):
            continue
        if any(convex_overlap(chassis, vertices) for vertices in obstacles):
            continue
        return (center, heading)
    return None


# How finely a planned route is sampled when checking it does not pass
# through a structure. One midpoint per segment is not enough -- a
# straight-line fallback across a corner clips it without its midpoint
# landing inside.
_ROUTE_SAMPLES = 24


def _route_is_real(field: FieldConfig, start, goal, robot_radius: float) -> bool:
    """Whether `plan_path` found an actual obstacle-free route rather
    than falling back to the straight line.

    The fallback is deliberate and right (a tactic with no waypoints is
    worse than a bad one), but it means an unreachable goal is reported
    by the navigator as a perfectly ordinary path. Offline, that is
    exactly the thing worth knowing, so check the route the way a test
    would: no point along it may be inside a structure."""
    path = plan_path(field, start, goal, robot_radius)
    for a, b in zip(path, path[1:]):
        for k in range(1, _ROUTE_SAMPLES):
            t = k / _ROUTE_SAMPLES
            point = (a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t)
            for obstacle in field.obstacles:
                if point_in_polygon(point, obstacle.vertices):
                    return False
    return True


# Grid used to find a route origin in open field. Coarse on purpose: it
# only has to land somewhere a robot could stand, and every field this
# runs on has far more open space than structure.
_ORIGIN_SAMPLES = 9


def open_reference_point(field: FieldConfig, robot_width: float, robot_length: float):
    """Somewhere in open field to plan routes from.

    The reachability check needs an origin, and the two obvious choices
    are both wrong. The field centre is wrong on any game that puts a
    structure there -- SALVAGE puts its depot on the exact centre, which
    made every feature report as unroutable the first time this ran,
    since the route started inside an obstacle and so never left one.
    The nearest free point to the centre is wrong too: it can land in a
    sealed pocket just big enough to stand in, from which nothing is
    reachable and every feature reports a false alarm.

    So pick the free point with the most room around it, breaking ties
    toward the middle. The most open spot on a field is the one place a
    robot is certainly not walled in."""
    obstacles = [o.vertices for o in field.obstacles]
    center = (field.width / 2.0, field.height / 2.0)
    best, best_key = None, None
    for i in range(_ORIGIN_SAMPLES):
        for j in range(_ORIGIN_SAMPLES):
            point = (
                field.width * (i + 0.5) / _ORIGIN_SAMPLES,
                field.height * (j + 0.5) / _ORIGIN_SAMPLES,
            )
            chassis = footprint_polygon(point, 0.0, robot_width, robot_length)
            if not all(0.0 <= x <= field.width and 0.0 <= y <= field.height for x, y in chassis):
                continue
            if any(convex_overlap(chassis, vertices) for vertices in obstacles):
                continue
            room = min(
                [point[0], field.width - point[0], point[1], field.height - point[1]]
                + [polygon_distance(point, vertices) for vertices in obstacles]
            )
            key = (room, -math.hypot(point[0] - center[0], point[1] - center[1]))
            if best_key is None or key > best_key:
                best, best_key = point, key
    return best


def _check_reachability(field, robot_width, robot_length, reference_point) -> list[FieldProblem]:
    problems = []
    robot_radius = math.hypot(robot_length / 2.0, robot_width / 2.0)
    origin = reference_point or open_reference_point(field, robot_width, robot_length)
    if origin is None:
        return [FieldProblem(
            ERROR, "<field>",
            f"no open space anywhere for a {robot_width:g}x{robot_length:g} robot",
            "Obstacles (or the field dimensions) leave nowhere to stand, so nothing "
            "downstream of this is worth checking.",
        )]

    targets = (
        [("scoring region", r) for r in field.scoring_regions]
        + [("intake location", s) for s in field.intake_locations]
    )
    for kind, feature in targets:
        aim = polygon_centroid(feature.vertices)
        approach = _has_legal_approach(field, aim, robot_width, robot_length)
        if approach is None:
            problems.append(FieldProblem(
                ERROR, feature.name,
                f"no pose exists from which a {robot_width:g}x{robot_length:g} robot can work this {kind}",
                "Every bearing around its centroid puts the chassis outside the field or "
                "inside a structure. Robots will drive at it, be held by whatever is in the "
                "way, and never arrive -- which reads as a strategy failing, not a field "
                "error.",
            ))
            continue
        if not _route_is_real(field, origin, approach[0], robot_radius):
            problems.append(FieldProblem(
                ERROR, feature.name,
                f"{kind} has an approach pose but no route to it from open field",
                f"Planned from {origin[0]:.0f},{origin[1]:.0f}. plan_path falls back to a "
                "straight line when no obstacle-free route exists, so this is silent at "
                "runtime: the robot drives into a structure and pushes.",
            ))
    return problems


# -- pinch points ------------------------------------------------------

def _polygon_gap(a, b) -> float:
    """Smallest distance between two polygons, 0.0 if they touch or
    overlap. Vertex-to-edge both ways, which is exact for the convex
    shapes a field is built from."""
    if polygons_intersect(a, b):
        return 0.0
    return min(
        min(polygon_distance(v, b) for v in a),
        min(polygon_distance(v, a) for v in b),
    )


def _wall_gaps(field: FieldConfig, vertices):
    xs = [v[0] for v in vertices]
    ys = [v[1] for v in vertices]
    return (
        ("the -x wall", min(xs)),
        ("the +x wall", field.width - max(xs)),
        ("the -y wall", min(ys)),
        ("the +y wall", field.height - max(ys)),
    )


def _check_pinch_points(field, robot_width, robot_length) -> list[FieldProblem]:
    """Gaps a robot cannot fit through.

    Not an error -- a game is entitled to a wall a robot cannot squeeze
    behind, and REEFSCAPE has several. It is a `NOTE` because the *shape*
    of the mistake it catches is expensive: a pocket that looks open on a
    drawing, is closed to a real chassis, and has something a robot wants
    on the far side of it. Read alongside the reachability errors above
    -- a pinch point next to a feature is the usual cause of one."""
    problems = []
    diagonal = math.hypot(robot_width, robot_length)
    obstacles = list(field.obstacles)
    for i, first in enumerate(obstacles):
        for label, gap in _wall_gaps(field, first.vertices):
            if 0.0 < gap < diagonal:
                problems.append(FieldProblem(
                    NOTE, first.name,
                    f"gap to {label} is {gap:.1f}in, less than a {diagonal:.1f}in robot diagonal",
                    "Nothing gets through there at any heading.",
                ))
        for second in obstacles[i + 1:]:
            gap = _polygon_gap(first.vertices, second.vertices)
            if 0.0 < gap < diagonal:
                problems.append(FieldProblem(
                    NOTE, f"{first.name} / {second.name}",
                    f"gap between them is {gap:.1f}in, less than a {diagonal:.1f}in robot diagonal",
                    "Nothing gets through there at any heading.",
                ))
    return problems
