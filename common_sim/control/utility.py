"""
One currency for "what could this robot do next, and is it worth it".

`ScoringOption` already answers that question well -- value over cost, in
points per second -- but only ever for a deposit, and only for a piece
already in hand. That makes the interesting comparison unaskable: *score
the ALGAE I'm holding* against *drive to a CORAL STATION first* lives
across two tactics, and the choice between them is a hand-written integer
`priority` on a Rule. An integer cannot know that the PROCESSOR is four
feet away and the station is across the field.

`Outcome` is the same points-per-second idea widened to cover a pickup.
A pickup scores nothing by itself, so its whole value is the deposit it
sets up: `enables` carries that one-step lookahead, valued *from the
pickup location*, which is what puts fetching and scoring into the same
units.

This module deliberately stops there. It generates and prices candidates;
it does not choose between them, weigh a lookahead against an immediate
score, or modulate anything by match time, score margin, or defensive
pressure. Those are decisions, they change behavior, and they belong to
the tactic that consumes this where they can be measured -- not to the
generator, where they would silently perturb scoring behavior that is
already tuned.

Layering: this sits above world_view (legality) and navigation (cost),
below planning and tactics. `ScoringOption`/`build_option` live here
rather than in planning.py so that planning can depend on this module and
not the other way round; planning re-exports both for its callers.
"""
from __future__ import annotations

from dataclasses import dataclass, field as dataclass_field
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
    piece: GamePiece | None    # None for a hypothetical piece not collected yet
    points: float              # scoring_rules.points_for(action, phase)
    deposit_time: float        # characteristics.deposit_duration(action)
    travel_time: float         # navigation.estimate_travel_time(...)

    @property
    def value_rate(self) -> float:
        return self.points / max(1e-6, self.travel_time + self.deposit_time)


@dataclass(frozen=True)
class Outcome:
    """One thing a robot could do next, priced in points and seconds.

    `points` is immediate game points and is 0.0 for a collect -- read
    `enables` for what a pickup is actually worth. `rp_progress` maps a
    ranking-point criterion to the fractional progress this outcome would
    make toward it; it is always empty until ranking points are modeled,
    and exists now so that adding them is not a change to this shape.

    `payload` is the kind-specific object an executing tactic needs: a
    ScoringOption for "score", an IntakeLocation or a GamePiece for
    "collect".
    """
    kind: str                   # "score" | "collect"
    label: str                  # human-readable, for a decision log
    points: float
    duration: float             # travel + action, seconds
    success_probability: float
    payload: object
    enables: "Outcome | None" = None
    rp_progress: dict = dataclass_field(default_factory=dict)

    @property
    def value_rate(self) -> float:
        """Immediate points per second. Deliberately ignores
        `success_probability` and `enables`: this is the exact quantity
        `ScoringOption.value_rate` has always been, including the 1e-6
        floor, so ranking on it is unchanged. Folding in the other terms
        is a decision, and decisions live in the consumer."""
        return self.points / max(1e-6, self.duration)


def build_option(match, robot: "Robot", legal, from_pos: tuple[float, float]) -> ScoringOption:
    """Turn a world_view.LegalScoringOption into a valued ScoringOption
    from `from_pos`. Public (not just planner-internal) so a caller
    pinning a specific region/action (Score) can value just that one
    candidate without going through a planner's own choice logic."""
    points = match.scoring_rules.points_for(legal.action, match.phase.value)
    deposit_time = robot.characteristics.deposit_duration(legal.action)
    goal = polygon_centroid(legal.region.vertices)
    travel_time = navigation.estimate_travel_time(match.field, from_pos, goal, robot.characteristics)
    return ScoringOption(
        region=legal.region, action=legal.action, piece=legal.piece,
        points=points, deposit_time=deposit_time, travel_time=travel_time,
    )


def _origin(robot: "Robot", from_pos) -> tuple[float, float]:
    return (robot.pose.x, robot.pose.y) if from_pos is None else from_pos


def _score_outcome(option: ScoringOption, robot: "Robot", piece_type: str) -> Outcome:
    return Outcome(
        kind="score",
        label=f"score {option.action} @ {option.region.name}",
        points=option.points,
        duration=option.travel_time + option.deposit_time,
        success_probability=robot.characteristics.reliability_for(piece_type),
        payload=option,
    )


def score_outcomes(match, robot: "Robot", from_pos=None, pieces=None) -> list[Outcome]:
    """Every legal deposit for a piece `robot` is holding, priced from
    `from_pos` (its own pose by default; a planner chaining across
    several held pieces passes a virtual one).

    `pieces`, when given, restricts the result to options for those
    pieces -- the chaining case, where a piece already placed earlier in
    the plan must not be re-offered. Filtering here rather than in the
    caller keeps the travel-time estimate off options that were going to
    be discarded anyway.

    Emission order is world_view.scoring_options' order (piece, then
    region, then action), which is what decides ties under `max`."""
    origin = _origin(robot, from_pos)
    outcomes = []
    for legal in world_view.scoring_options(match, robot):
        if pieces is not None and legal.piece not in pieces:
            continue
        option = build_option(match, robot, legal, origin)
        outcomes.append(_score_outcome(option, robot, legal.piece.piece_type))
    return outcomes


def best_score_for_type(match, robot: "Robot", piece_type: str, from_pos) -> "Outcome | None":
    """The best deposit `robot` could make with a piece of `piece_type`
    it does not have yet, valued from `from_pos`.

    This is the payoff half of a pickup, and it is why the lookahead is
    worth computing at all: a station is only as good as the nearest
    place the thing it dispenses can be put. The returned Outcome's
    ScoringOption carries `piece=None` -- the piece genuinely does not
    exist yet, and inventing one would make the option look executable.

    None when there is nowhere legal to put that type, in which case
    collecting it is worth nothing at all right now."""
    best: Outcome | None = None
    for region, action in world_view.scoring_slots_for_type(match, robot, piece_type):
        legal = world_view.LegalScoringOption(region=region, action=action, piece=None)
        candidate = _score_outcome(build_option(match, robot, legal, from_pos), robot, piece_type)
        if best is None or candidate.value_rate > best.value_rate:
            best = candidate
    return best


def collect_outcomes(match, robot: "Robot", from_pos=None) -> list[Outcome]:
    """Every pickup available to `robot` right now -- human-player
    stations first, then loose pieces on the field -- priced from
    `from_pos`.

    `points` is 0.0 for all of them, by definition: a pickup scores
    nothing. Their value is in `enables`, the best deposit for what would
    be picked up, valued from the pickup point rather than from here --
    so a station on the far side of the field is charged for the drive
    out *and* correctly credited for landing next to a REEF.

    Duration uses the robot's own intake timings, which already
    distinguish a station handoff from scooping a piece off the floor
    (`station_intake_time` vs `Robot.duration_for`); a caller must not
    re-derive that split."""
    origin = _origin(robot, from_pos)
    characteristics = robot.characteristics
    outcomes = []

    for station in world_view.station_options(match, robot):
        goal = polygon_centroid(station.vertices)
        travel = navigation.estimate_travel_time(match.field, origin, goal, characteristics)
        outcomes.append(Outcome(
            kind="collect",
            label=f"collect {station.piece_type} @ {station.name}",
            points=0.0,
            duration=travel + characteristics.station_intake_time,
            success_probability=1.0,
            payload=station,
            enables=best_score_for_type(match, robot, station.piece_type, goal),
        ))

    for piece in world_view.collectable_pieces(match, robot=robot):
        if not _has_room_for(robot, piece.piece_type):
            continue
        goal = (piece.position.x, piece.position.y)
        travel = navigation.estimate_travel_time(match.field, origin, goal, characteristics)
        outcomes.append(Outcome(
            kind="collect",
            label=f"collect {piece.piece_type} @ ({goal[0]:.0f}, {goal[1]:.0f})",
            points=0.0,
            duration=travel + robot.duration_for(piece),
            success_probability=1.0,
            payload=piece,
            enables=best_score_for_type(match, robot, piece.piece_type, goal),
        ))

    return outcomes


def _has_room_for(robot: "Robot", piece_type: str) -> bool:
    """Whether `robot` could actually hold another piece of this type.

    `world_view.station_options` already applies this, but
    `collectable_pieces` does not -- it answers "what is lying around
    that this robot's intakes accept", which is a different question and
    the right one for its other callers. Offering a pickup the robot has
    no room for would put an outcome in the list that can never
    complete."""
    held = sum(1 for p in robot.held_pieces if p.piece_type == piece_type)
    return held < robot.characteristics.capacity_for(piece_type)
