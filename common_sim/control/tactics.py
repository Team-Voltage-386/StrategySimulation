"""
High-level tactics: Behaviors that decide their own target each tick
instead of being handed a Pose2d. Each owns a target + a NavigateTo +
primitive children internally, re-evaluated on `replan_period`.

Each tactic exposes `PARAM_SCHEMA` (see strategy_editor's Param) so a
GUI can build a property inspector for it with zero per-tactic GUI code.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

from common_sim.control import world_view
from common_sim.control.behavior import Behavior, BehaviorContext, Status
from common_sim.control.navigation import NavigateTo, clear_standoff, estimate_travel_time
from common_sim.control.param import Param
from common_sim.control.planning import GreedyRatePlanner, ScorePlanner, build_option
from common_sim.field.field_config import IntakeLocation, point_in_polygon, polygon_centroid
from common_sim.geometry import Pose2d, wrap_angle
from common_sim.robot.characteristics import SIDE_OUTWARD


def _intercept_time(origin: tuple[float, float], position: tuple[float, float], velocity, speed: float) -> float:
    """Time for something moving at `speed` from `origin` to reach a
    target now at `position` and moving at constant `velocity` --
    i.e. treats a rolling piece as continuing in a straight line rather
    than freezing it where it is right now. Solves the standard pursuit
    quadratic |position + velocity*t - origin| = speed*t for its
    smallest positive root; falls back to plain distance/speed when the
    target isn't moving (the overwhelmingly common case), and returns
    infinity when the target is outrunning us and is never caught."""
    dx, dy = position[0] - origin[0], position[1] - origin[1]
    vx, vy = velocity[0], velocity[1]
    if vx == 0.0 and vy == 0.0:
        return math.hypot(dx, dy) / speed

    a = vx * vx + vy * vy - speed * speed
    b = 2.0 * (dx * vx + dy * vy)
    c = dx * dx + dy * dy

    if abs(a) < 1e-9:
        if abs(b) < 1e-9:
            return math.inf
        t = -c / b
        return t if t > 1e-9 else math.inf

    disc = b * b - 4.0 * a * c
    if disc < 0:
        return math.inf
    sqrt_disc = math.sqrt(disc)
    roots = [(-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a)]
    positive = [t for t in roots if t > 1e-9]
    return min(positive) if positive else math.inf


def _predicted_piece_position(origin: tuple[float, float], piece, speed: float) -> tuple[float, float]:
    """Where a rolling piece will plausibly be by the time it's caught,
    for handing to the real (obstacle-aware) path planner as its goal.
    `plan_path` has no notion of a moving goal, so this does a one-shot
    estimate via the empty-field intercept time and projects the piece
    that far along its current velocity -- good enough since a piece
    decelerates under friction and is usually caught well short of
    that point anyway, and any error left over just becomes noise the
    next replan corrects."""
    naive = _intercept_time(origin, (piece.position.x, piece.position.y), piece.velocity, speed)
    if naive == math.inf or naive <= 0.0:
        return (piece.position.x, piece.position.y)
    horizon = min(naive, 1.0)
    return (piece.position.x + piece.velocity.x * horizon, piece.position.y + piece.velocity.y * horizon)


# Margin required for a freshly re-evaluated target to displace one
# already committed to -- whichever is larger of a flat floor or a
# fraction of the current ETA. Without this, two options that come out
# within noise of each other tick to tick (a piece rolling slightly, a
# rival's published intent shifting) would have a robot swapping
# targets every replan instead of ever actually driving to one.
_RETARGET_MARGIN_MIN = 0.3
_RETARGET_MARGIN_RATIO = 0.15

# How far past the halfway line a committed piece has to have rolled --
# as a fraction of the distance between the two alliances' ends -- before
# `opposing_side="last_resort"` gives up on chasing it (see
# `Collect._rolled_to_opposing_side`).
_OPPOSING_SIDE_ROLL_MARGIN = 0.10


def _last_resort(tier: tuple[int, int, int]) -> bool:
    """Whether a `_piece_rank` tier is one a robot would rather not take
    at all, as opposed to merely a worse bet -- which is what decides
    whether a station gets to win outright or only on ETA.

    Two of the three demotions mean the piece isn't really ours to go
    for: a teammate is about to reach it (`_piece_rank` reason 1), or
    it's parked on the opponents' half (reason 3). Faced with either, a
    station is the better job however far off it is. Being out-run by an
    *opponent* (reason 2) is different -- contesting is still real play,
    and whether it beats a trip to the station is exactly the ETA
    question `_pick_target` already asks."""
    return bool(tier[0] or tier[2])


@dataclass(frozen=True)
class _Contention:
    """The best (soonest) ETA any *other* robot currently declaring a
    given piece would reach it in, kept split by whose robot it is. The
    two are not interchangeable: a teammate that beats us there has
    already covered that piece for the alliance, so following it in is
    pure waste, whereas out-racing us to a piece is exactly what an
    opponent is trying to do and contesting it can still be the right
    call."""
    teammate: float = math.inf
    opponent: float = math.inf


def _robot_engaged_with_station(robot, station) -> bool:
    """Whether `robot` is physically working `station` right now --
    parked on the feed, or close enough that the sim counts it as
    engaged -- as opposed to just declaring it as a target. Engagement
    (`nearby_station`) is the same test the sim uses to decide whether
    to dispense, and it is checked before polygon containment because a
    robot parked at its intake standoff can sit a couple of inches
    outside a station's zone while still being served by it."""
    if robot.nearby_station() is station:
        return True
    return point_in_polygon((robot.pose.x, robot.pose.y), station.vertices)


def _tiebreak_bias(match, robot) -> float:
    """A deterministic, vanishingly small (~1us) per-robot offset used
    only to break an exact ETA tie between two robots. Obstacle-routed
    ETA often lands on the *same* value for two robots racing for the
    same thing -- two teammates that are mirror images of each other on
    a symmetric field take mirror-image routes of identical length, and
    a robot compared against itself is always exactly tied -- and
    without a tiebreaker each side of a `<=` comparison reads itself as
    at least as fast as the other, so both claim the same target (or,
    for a capacity-1 station, both defer to the other's claim) forever.
    `match.robots` is stable for the life of a match, so this is
    consistent no matter which robot's perspective is asking."""
    try:
        return match.robots.index(robot) * 1e-6
    except ValueError:
        return 0.0


class Tactic(Behavior):
    """Marker base -- a Tactic *is* a Behavior, so it composes with
    Sequence/Parallel/Repeat and existing routines unchanged."""

    PARAM_SCHEMA: tuple[Param, ...] = ()


class Idle(Tactic):
    """Explicit do-nothing, so "no rule fired" is a visible state rather
    than a silent stall. Never terminates on its own."""

    PARAM_SCHEMA = ()

    def tick(self, ctx: BehaviorContext) -> Status:
        ctx.robot.drive_field_relative(ctx.dt, 0.0, 0.0, 0.0)
        return Status.RUNNING

    def reset(self) -> None:
        pass


class RunScript(Tactic):
    """Wraps a plain child Behavior (typically a Sequence of existing
    primitives) so a hand-scripted routine is a first-class Rule
    alongside reactive tactics."""

    PARAM_SCHEMA = ()

    def __init__(self, children: list[Behavior]):
        from common_sim.control.behavior import Sequence
        self.children = children
        self._sequence = Sequence(children)

    def tick(self, ctx: BehaviorContext) -> Status:
        return self._sequence.tick(ctx)

    def reset(self) -> None:
        self._sequence.reset()


class Collect(Tactic):
    """Picks a target piece/station from world_view, navigates so an
    intaking side faces it, and holds intake active. SUCCESS when held
    count increases or capacity is reached; FAILURE when nothing is
    collectable. Re-targets every tick (not just on replan) if the
    targeted piece is taken by someone else first."""

    PARAM_SCHEMA = (
        Param("piece_type", kind="piece_type", default=None, optional=True),
        Param("mode", kind="choice", choices=("nearest", "densest"), default="nearest"),
        Param("cluster_radius", kind="float", default=24.0, min=0, suffix=" in"),
        Param("max_range", kind="float", default=None, optional=True, min=0, suffix=" in"),
        Param("opposing_side", kind="choice", choices=("last_resort", "allow"), default="last_resort"),
    )

    def __init__(
        self,
        piece_type: str | None = None,
        mode: Literal["nearest", "densest"] = "nearest",
        cluster_radius: float = 24.0,
        max_range: float | None = None,
        opposing_side: Literal["last_resort", "allow"] = "last_resort",
        replan_period: float = 0.1,
    ):
        self.piece_type = piece_type
        self.mode = mode
        self.cluster_radius = cluster_radius
        self.max_range = max_range
        # Whether a piece sitting on the opponents' half of the field is
        # a normal option ("allow") or one only taken when this half has
        # nothing left ("last_resort"). A game gate, not a tuning knob:
        # in REEFSCAPE a cross-field trip costs most of a cycle and the
        # CORAL STATION back home never runs dry, so wandering over is
        # almost always a loss -- but a game whose pieces all live in a
        # contested middle, or one with no meaningful side split at all,
        # wants "allow". Fields where world_view can't tell the halves
        # apart ignore this setting either way.
        self.opposing_side = opposing_side
        self.replan_period = replan_period

        self._target_piece = None
        self._target_station = None
        self._start_held_count = None
        self._reconsider_timer = 0.0
        self._nav = NavigateTo(self._provide_target, heading_mode="face_target", replan_period=replan_period)

    def reset(self) -> None:
        self._target_piece = None
        self._target_station = None
        self._start_held_count = None
        self._reconsider_timer = 0.0
        self._nav.reset()

    def _provide_target(self, ctx: BehaviorContext) -> Pose2d:
        robot = ctx.robot
        characteristics = robot.characteristics
        if self._target_station is not None:
            cx, cy = self._station_aim(ctx)
            piece_type = self._target_station.piece_type
        else:
            assert self._target_piece is not None
            cx, cy = self._target_piece.position.x, self._target_piece.position.y
            piece_type = self._target_piece.piece_type

        # Rotate so the configured intake side faces the target, the same
        # way Score._provide_target presents the scoring side -- otherwise
        # a robot whose intake only accepts a piece type on a non-front
        # side (e.g. algae on "right") drives straight at the target
        # nose-first and never actually gets it in intake range.
        side = characteristics.intake_side_for(piece_type)
        outward = SIDE_OUTWARD[side]
        side_local_angle = math.atan2(outward[1], outward[0])

        # Aimed at with the *bumper*, parked so the target sits mid-wedge
        # in the intake's reach rather than under the chassis center.
        # This matters just as much for a station as for a loose piece:
        # driving the chassis *center* onto a station's centroid parks
        # the bumper `half_extent` further out along the approach
        # bearing, and for a corner station (small footprint, tight
        # against two field walls) that overshoot lands the intake wedge
        # (bumper -MANIPULATOR_INSET..+intake_range) mostly outside the
        # station polygon -- the robot settles nearby but never actually
        # captures a piece. Standing off so the centroid falls mid-wedge
        # fixes both that and (for a loose piece) REEFSCAPE's ALGAE
        # spawning ~7in off the REEF wall, where a center-on-centroid
        # pose buries half the chassis in the REEF and caps the
        # clearance the planner is allowed to keep on the way over (see
        # _clearance_for_goal), clipping the REEF's corner getting there
        # too.
        half_extent = (characteristics.length if side in ("front", "back") else characteristics.width) / 2.0
        x, y, heading = clear_standoff(
            ctx.match.field, (cx, cy), (robot.pose.x, robot.pose.y),
            half_extent + characteristics.intake_range / 2.0,
            width=characteristics.width, length=characteristics.length,
            side_local_angle=side_local_angle,
        )
        return Pose2d(x, y, heading)

    def _station_aim(self, ctx: BehaviorContext) -> tuple[float, float]:
        """Normally the station's centroid -- but held one footprint back
        from it while someone else is already working the station, so a
        robot that arrives second queues up outside instead of driving
        into the occupant and shoving it off the feed before it has its
        piece. Re-evaluated every replan, so the wait ends by itself the
        moment the station frees up.

        The robot already on the feed is exempt (see `_holds_station`):
        whoever got there first keeps it."""
        station = self._target_station
        robot = ctx.robot
        aim = polygon_centroid(station.vertices)
        if self._station_has_room_for(ctx, station):
            return aim

        characteristics = robot.characteristics
        dx, dy = robot.pose.x - aim[0], robot.pose.y - aim[1]
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            return aim
        # Backed off along the bearing from the station rather than from
        # the occupant: the occupant shuffles as it collects, and an aim
        # point chasing it would have the waiting robot orbiting.
        back = math.hypot(characteristics.width, characteristics.length)
        return (aim[0] + dx / distance * back, aim[1] + dy / distance * back)

    def _holds_station(self, robot) -> bool:
        """Whether `robot` is the one already being served by its target
        station, rather than a robot on its way to it. Both a robot
        queueing outside and the robot on the feed publish a claim on the
        station, so each reads the other as an occupant; without this the
        incumbent would treat itself as crowded out and leave, and the
        two would trade places forever without either collecting."""
        return _robot_engaged_with_station(robot, self._target_station)

    def _better_station_exists(self, ctx: BehaviorContext) -> bool:
        """Whether the committed station is full and some other one this
        robot could use isn't."""
        match, robot = ctx.match, ctx.robot
        if self._station_has_room_for(ctx, self._target_station):
            return False
        return any(
            station is not self._target_station
            and (self.piece_type is None or station.piece_type == self.piece_type)
            and self._station_has_room_for(ctx, station)
            for station in world_view.station_options(match, robot)
        )

    def _station_has_room_for(self, ctx: BehaviorContext, station: IntakeLocation) -> bool:
        """Whether `robot` should treat `station` as having a free slot
        right now. Physically-engaged robots (parked on the feed) always
        take up real capacity; whatever's left goes to whichever
        *intent-only* claimants -- robots declaring the station as their
        target but not physically there yet -- would actually arrive
        soonest.

        Racing the intent-only claimants matters because without it, two
        robots that both declare the same capacity-1 station at the same
        moment each read the *other's* mere intent as filling the one
        slot and both back off to queue outside -- and since neither is
        ever physically there to break the tie (`_holds_station` only
        protects an incumbent once it has arrived), they defer to each
        other forever instead of the faster one actually going in."""
        match, robot = ctx.match, ctx.robot
        if _robot_engaged_with_station(robot, station):
            return True

        capacity = world_view.region_robot_capacity(station, robot)
        engaged = [r for r in match.robots if r is not robot and _robot_engaged_with_station(r, station)]
        remaining = capacity - len(engaged)
        if remaining <= 0:
            return False

        claimants = [
            r for r in match.robots
            if r is not robot and r not in engaged
            and getattr(getattr(r, "intent", None), "target_region", None) == station.name
        ]
        if not claimants:
            return True

        centroid = polygon_centroid(station.vertices)
        our_eta = estimate_travel_time(match.field, (robot.pose.x, robot.pose.y), centroid, robot.characteristics)
        our_eta += _tiebreak_bias(match, robot)
        faster = sum(
            1 for r in claimants
            if estimate_travel_time(match.field, (r.pose.x, r.pose.y), centroid, r.characteristics)
            + _tiebreak_bias(match, r) < our_eta
        )
        return faster < remaining

    def _best_station(self, ctx: BehaviorContext) -> tuple[IntakeLocation | None, float]:
        match, robot = ctx.match, ctx.robot
        origin = robot.pose.translation
        characteristics = robot.characteristics
        stations = world_view.station_options(match, robot)
        if self.piece_type is not None:
            stations = [s for s in stations if s.piece_type == self.piece_type]
        if not stations:
            return None, math.inf

        # Same rule Score._pick_option applies to scoring regions: take
        # one nobody is working before contesting one. Two robots both
        # picking "nearest" otherwise converge on the same corner, and a
        # REEFSCAPE CORAL STATION is 36x36 against a 28x28 robot --
        # capacity 1, no room to share. Falling back to the crowded
        # nearest is fine because _station_aim then queues outside it
        # rather than barging in; with every station busy, waiting for
        # the nearest is exactly the right thing to do.
        roomy = [s for s in stations if world_view.region_has_room(match, s, robot)]
        candidates = roomy or stations

        # Obstacle-routed travel time, not straight-line distance -- a
        # station is stationary, but the *path* to it (around the REEF,
        # around a charge station) is not, and the same time unit as
        # _best_piece's ETA is what lets _pick_target compare the two
        # directly.
        def eta(s):
            return estimate_travel_time(match.field, (origin.x, origin.y), polygon_centroid(s.vertices), characteristics)

        best = min(candidates, key=eta)
        return best, eta(best)

    def _piece_contenders(self, ctx: BehaviorContext) -> dict[object, _Contention]:
        """Map of {piece: _Contention} from every other robot on the field
        currently declaring that piece as its intent, with each rival's
        ETA filed under whether it is a teammate's or an opponent's.
        Obstacle-routed, like our own ETA, so a rival on the far side of
        the REEF from a piece isn't mistaken for the faster claimant just
        because it looks closer as the crow flies."""
        match, robot = ctx.match, ctx.robot
        contenders: dict[object, _Contention] = {}
        for other in match.robots:
            if other is robot:
                continue
            intent = other.intent
            target = getattr(intent, "target_piece", None) if intent is not None else None
            if target is None:
                continue
            characteristics = other.characteristics
            speed = max(characteristics.max_speed, 1e-6)
            origin = other.pose.translation
            predicted = _predicted_piece_position((origin.x, origin.y), target, speed)
            eta = estimate_travel_time(match.field, (origin.x, origin.y), predicted, characteristics) + _tiebreak_bias(match, other)
            current = contenders.get(target, _Contention())
            if other.alliance == robot.alliance:
                contenders[target] = _Contention(min(current.teammate, eta), current.opponent)
            else:
                contenders[target] = _Contention(current.teammate, min(current.opponent, eta))
        return contenders

    def _piece_rank(self, piece, our_eta: float, contention: _Contention, on_own_side) -> tuple[int, int, int, float]:
        """Sort key deciding which of several reachable pieces to go for:
        the three flags pick a *tier*, and ETA only breaks ties inside
        it. Every flag clear is the tier a robot actually wants; each one
        set is a reason this piece is only worth taking because nothing
        better exists.

        Worst first, because that's the order the reasons trump each
        other:

        1. A teammate gets there first -- the alliance already has that
           piece covered, so following it in scores nothing at all. This
           is the tier that has to lose to everything else, including a
           trip across the field, and the reason two robots stop
           converging on one piece: the loser of the race is left with a
           strictly better option (usually the station) and takes it.
        2. An opponent gets there first -- likely a wasted drive too, but
           contesting at least denies them if they fumble it, so this
           only loses to options we'd actually win.
        3. It's on the opposing half and `opposing_side` says to treat
           that as a last resort -- slow, but we do come home with a
           piece, so this is the mildest demotion of the three."""
        return (
            int(contention.teammate < our_eta),
            int(contention.opponent < our_eta),
            int(not on_own_side(piece.position.x, piece.position.y)),
            our_eta,
        )

    def _piece_eta(self, ctx: BehaviorContext, origin, piece, characteristics) -> float:
        """Obstacle-routed time to reach a piece that may still be
        rolling: `_predicted_piece_position` locates roughly where it'll
        be by the time it's caught, and the real path planner is asked
        for the route there rather than assuming a straight line -- a
        piece sitting just past the REEF from us is not actually
        "nearest" just because it's close by air distance."""
        speed = max(characteristics.max_speed, 1e-6)
        predicted = _predicted_piece_position((origin.x, origin.y), piece, speed)
        return estimate_travel_time(ctx.match.field, (origin.x, origin.y), predicted, characteristics)

    def _piece_tiers(self, ctx: BehaviorContext) -> dict[object, tuple[int, int, int, float]]:
        """`_piece_rank` for every piece this robot could collect right
        now, keyed by piece. Built in one pass because both the rival
        ETAs and the own-half split are shared across all the candidates
        and neither is cheap enough to recompute per piece."""
        match, robot = ctx.match, ctx.robot
        origin = robot.pose.translation
        characteristics = robot.characteristics
        pieces = world_view.collectable_pieces(match, piece_type=self.piece_type, robot=robot)
        if self.max_range is not None:
            pieces = [p for p in pieces if origin.get_distance(p.position) <= self.max_range]
        if not pieces:
            return {}

        contenders = self._piece_contenders(ctx)
        our_bias = _tiebreak_bias(match, robot)
        if self.opposing_side == "last_resort":
            on_own_side = world_view.own_side_test(match, robot.alliance)
        else:
            on_own_side = lambda x, y: True  # noqa: E731 -- gate off: every piece counts as near

        # "Nearest" means quickest to actually reach, not closest right
        # now -- a piece rolling away, or one tucked past an obstacle,
        # can take longer to reach than one that's farther off by air
        # distance but has a clear, direct route. The tiebreak bias goes
        # in here so the comparison against a rival's (equally biased)
        # ETA can't read as a tie from both sides at once.
        return {
            piece: self._piece_rank(
                piece,
                self._piece_eta(ctx, origin, piece, characteristics) + our_bias,
                contenders.get(piece, _Contention()),
                on_own_side,
            )
            for piece in pieces
        }

    def _best_piece(self, ctx: BehaviorContext) -> tuple[object | None, float, bool]:
        """The piece to go for, its ETA, and whether the tier it came
        from is one this robot would rather not take at all -- see
        `_last_resort`."""
        tiers = self._piece_tiers(ctx)
        if not tiers:
            return None, math.inf, False

        # Consider only the best tier present, so a reason to avoid a
        # piece is never traded away for a shorter drive: two robots
        # both picking "nearest" otherwise converge on the identical
        # piece and idle nose-to-nose at it instead of splitting up.
        # Dropping to a worse tier when that's all there is (rather than
        # standing still) is the same "contest rather than sit idle"
        # rule stations use once every station is crowded.
        best_tier = min(rank[:3] for rank in tiers.values())
        candidates = [piece for piece, rank in tiers.items() if rank[:3] == best_tier]

        if self.mode == "densest":
            clusters = world_view.piece_clusters(ctx.match, candidates, self.cluster_radius)
            best = max(clusters, key=lambda c: c.count)
            target = min(best.pieces, key=lambda p: tiers[p][3])
        else:
            target = min(candidates, key=lambda p: tiers[p][3])

        return target, tiers[target][3], _last_resort(best_tier)

    def _rolled_to_opposing_side(self, ctx: BehaviorContext) -> bool:
        """Whether the committed piece has since rolled well onto the
        opponents' half. Chasing it over is the same cross-field trip
        `opposing_side` refuses to *start*, and a piece knocked loose
        toward the opponents' end is exactly how a robot ends up making
        it anyway -- so give it up and re-pick rather than follow.

        Judged against a line pushed `_OPPOSING_SIDE_ROLL_MARGIN` past
        the real one, so a piece drifting along midfield doesn't flip
        this every replan; nothing else moves the boundary, so a plain
        threshold would be a coin toss right where pieces linger."""
        if self.opposing_side != "last_resort":
            return False
        piece = self._target_piece
        on_own_side = world_view.own_side_test(
            ctx.match, ctx.robot.alliance, margin_frac=_OPPOSING_SIDE_ROLL_MARGIN,
        )
        return not on_own_side(piece.position.x, piece.position.y)

    def _losing_piece_race(self, ctx: BehaviorContext) -> bool:
        """Whether some other robot would now reach the committed piece
        first. Re-checked periodically (see `_reconsider_now`), not just
        at pick time, because intents change after commitment: a piece
        that was uncontested when picked can still get out-run by
        whoever declares it next."""
        robot = ctx.robot
        piece = self._target_piece
        if piece is None:
            return False
        our_eta = self._piece_eta(ctx, robot.pose.translation, piece, robot.characteristics) + _tiebreak_bias(ctx.match, robot)
        contention = self._piece_contenders(ctx).get(piece, _Contention())
        return min(contention.teammate, contention.opponent) < our_eta

    def _better_piece_exists(self, ctx: BehaviorContext) -> bool:
        """Whether, now that real obstacle-routed travel time is known
        rather than assumed at pick time, something else -- another
        piece, or the station -- is meaningfully quicker to reach than
        the piece committed to. Catches both a piece that turns out to
        need a long detour around field structure once actually pathed,
        and a piece that's since spawned closer on our own side -- without
        this a robot that picked a piece across the field early stays
        committed to it, chasing a slow option for the rest of the match,
        even once plenty of quicker ones exist. `_RETARGET_MARGIN_*` keeps
        two closely-matched options from swapping back and forth every
        reconsideration instead of ever actually being driven to."""
        piece = self._target_piece
        if piece is None:
            return False
        robot = ctx.robot
        current_eta = self._piece_eta(ctx, robot.pose.translation, piece, robot.characteristics)
        if current_eta == math.inf:
            return True
        _, station_eta = self._best_station(ctx)
        _, piece_eta, piece_last_resort = self._best_piece(ctx)
        # A last-resort piece never displaces a committed one on ETA
        # alone -- "closer" is not a reason to go take one a teammate is
        # already about to reach, or to head across the field.
        best_eta = min(station_eta, math.inf if piece_last_resort else piece_eta)
        if best_eta == math.inf:
            return False
        margin = max(_RETARGET_MARGIN_MIN, current_eta * _RETARGET_MARGIN_RATIO)
        return best_eta + margin < current_eta

    def _reconsider_now(self, ctx: BehaviorContext) -> bool:
        """Throttle for the expensive re-evaluation checks (obstacle-
        routed ETAs, one per candidate) to `replan_period`, same as
        `_nav`'s own replanning cadence -- the cheap held/scored checks
        in `tick` still run every physics tick regardless."""
        self._reconsider_timer += ctx.dt
        if self._reconsider_timer < self.replan_period:
            return False
        self._reconsider_timer = 0.0
        return True

    def _pick_target(self, ctx: BehaviorContext) -> bool:
        # Field and station are both just supply the robot's intakes can
        # use (world_view already filters each to what the configured
        # sides accept) -- so "nearest" means quickest to reach of
        # either, not station-always-first. Compared in ETA, not raw
        # distance, so a piece rolling away doesn't out-ruler a station
        # that's slightly farther off but standing still. Ties go to the
        # station since it never runs out mid-cycle.
        #
        # A station also wins outright, however far off it is, over a
        # piece only a last resort would take (see `_last_resort`). This
        # is what actually settles a two-robot race for one piece: the
        # robot that would lose it has somewhere strictly better to be,
        # so it goes there instead of trailing its own teammate in.
        station, station_eta = self._best_station(ctx)
        piece, piece_eta, piece_last_resort = self._best_piece(ctx)

        if station is not None and (piece_last_resort or station_eta <= piece_eta):
            self._target_station, self._target_piece = station, None
            return True
        if piece is not None:
            self._target_piece, self._target_station = piece, None
            return True

        self._target_piece = None
        self._target_station = None
        return False

    def tick(self, ctx: BehaviorContext) -> Status:
        robot = ctx.robot
        if self._start_held_count is None:
            self._start_held_count = len(robot.held_pieces)

        if len(robot.held_pieces) > self._start_held_count:
            robot.set_intake_active(False)
            return Status.SUCCESS
        # Full for what this node is actually here to collect -- not for
        # everything it happens to be carrying. `self.piece_type` is the
        # declared intent, and it is checked before a target is picked,
        # so it is the only type known at this point.
        if robot.held_pieces and robot.is_full_for(self.piece_type):
            robot.set_intake_active(False)
            return Status.SUCCESS

        # The expensive re-evaluation checks below (obstacle-routed ETAs)
        # only actually run once per replan_period; `reconsider` is False
        # on every other tick in between.
        reconsider = self._reconsider_now(ctx)
        target_lost = self._target_piece is not None and (
            self._target_piece.held_by is not None
            or self._target_piece.scored
            or (reconsider and (
                self._rolled_to_opposing_side(ctx)
                or self._losing_piece_race(ctx)
                or self._better_piece_exists(ctx)
            ))
        )
        # A station is committed to once and then queued for, so nothing
        # would ever pull a robot off one an opponent has parked in --
        # except a free station elsewhere, which is strictly better than
        # waiting. Guarded on another station actually having room so
        # this can't cycle between two equally crowded ones.
        if self._target_station is not None and reconsider and self._better_station_exists(ctx):
            target_lost = True
        need_target = self._target_piece is None and self._target_station is None
        if need_target or target_lost:
            if not self._pick_target(ctx):
                robot.set_intake_active(False)
                return Status.FAILURE

        robot.set_intake_active(True)
        self._nav.tick(ctx)
        return Status.RUNNING


class Score(Tactic):
    """Plans and executes scoring for whatever the robot holds. `region`
    / `action` pin a specific choice ("always L4 on the far face");
    left None, the planner chooses. Navigates with the scoring side
    presented, gates the deposit on `match.deposit_region_for` so the
    sim's own readiness check is the single source of truth. Loops
    until empty-handed."""

    PARAM_SCHEMA = (
        Param("region", kind="region_name", default=None, optional=True),
        Param("action", kind="action", default=None, optional=True),
    )

    def __init__(self, planner: ScorePlanner | None = None, region: str | None = None, action: str | None = None, replan_period: float = 0.1):
        self.planner = planner or GreedyRatePlanner()
        self.region = region
        self.action = action
        self.replan_period = replan_period

        self._current = None  # planning.ScoringOption
        self._nav = NavigateTo(self._provide_target, heading_mode="face_target", replan_period=replan_period)

    def reset(self) -> None:
        self._current = None
        self._nav.reset()

    def _provide_target(self, ctx: BehaviorContext) -> Pose2d:
        assert self._current is not None
        robot = ctx.robot
        region = self._current.region
        # Aim off-centroid when someone else is already working this
        # region -- only ever happens on a region big enough that
        # _pick_option was willing to share it, and without it both
        # robots would drive at the identical centroid.
        occupants = world_view.region_occupants(ctx.match, region, exclude=robot)
        aim = world_view.region_approach_point(region, robot, occupants)
        side = robot.characteristics.score_side_for(self._current.piece.piece_type)
        outward = SIDE_OUTWARD[side]
        side_local_angle = math.atan2(outward[1], outward[0])

        # Park with the scoring side's bumper edge on `aim`, not the
        # chassis *center* on it. A scoring zone is sized to the entry
        # face of the structure it belongs to -- a REEFSCAPE REEF face's
        # is 10in deep, starting at the REEF wall -- so a robot aiming
        # its center there is asking to put 14in of its own chassis
        # inside solid structure. It never arrives: it grinds along the
        # REEF, ends up parked askew, and its manipulator never squares
        # up. Scoring only ever needed the side's reach points inside
        # the zone (Robot.side_engages_polygon), which a bumper-on-the-
        # zone pose satisfies with the whole chassis in free space.
        characteristics = robot.characteristics
        half_extent = (characteristics.length if side in ("front", "back") else characteristics.width) / 2.0
        x, y, heading = clear_standoff(
            ctx.match.field, aim, (robot.pose.x, robot.pose.y), half_extent,
            width=characteristics.width, length=characteristics.length,
            side_local_angle=side_local_angle,
        )
        return Pose2d(x, y, heading)

    def _pick_option(self, ctx: BehaviorContext) -> bool:
        match, robot = ctx.match, ctx.robot
        if self.region is not None or self.action is not None:
            # A pinned region/action narrows *which* candidates are legal
            # at all -- go straight to world_view rather than through
            # the planner's own single "best overall" choice, which
            # could easily not be the pinned one.
            legal = [o for o in world_view.scoring_options(match, robot) if o.piece in robot.held_pieces]
            if self.region is not None:
                legal = [o for o in legal if o.region.name == self.region]
            if self.action is not None:
                legal = [o for o in legal if o.action == self.action]
            if not legal:
                self._current = None
                return False
            self._current = self._best_uncrowded(ctx, legal) or self._best_valued(ctx, legal)
            return True

        options = self.planner.plan(match, robot)
        if not options:
            self._current = None
            return False
        best = options[0]
        if world_view.region_has_room(match, best.region, robot):
            self._current = best
            return True

        # The planner ranks on value alone, so with identical regions to
        # choose from every robot on the alliance picks the same one and
        # they converge on it together. When its choice is a region
        # someone else is already working that's too small to share (a
        # single REEF face, say), re-pick among the regions that do have
        # room -- for the same piece, since the planner already decided
        # which piece to score first. Falls back to the crowded choice
        # when nothing has room: contesting a spot still eventually
        # scores, standing around holding the piece never does.
        legal = [o for o in world_view.scoring_options(match, robot) if o.piece is best.piece]
        self._current = self._best_uncrowded(ctx, legal) or best
        return True

    def _best_uncrowded(self, ctx: BehaviorContext, legal) -> object | None:
        roomy = [o for o in legal if world_view.region_has_room(ctx.match, o.region, ctx.robot)]
        return self._best_valued(ctx, roomy)

    def _best_valued(self, ctx: BehaviorContext, legal) -> object | None:
        if not legal:
            return None
        pos = (ctx.robot.pose.x, ctx.robot.pose.y)
        built = [build_option(ctx.match, ctx.robot, o, pos) for o in legal]
        return max(built, key=lambda o: o.value_rate)

    def tick(self, ctx: BehaviorContext) -> Status:
        robot = ctx.robot
        if not robot.held_pieces:
            robot.set_deposit_active(False)
            return Status.SUCCESS

        if self._current is None or self._current.piece not in robot.held_pieces:
            if not self._pick_option(ctx):
                return Status.RUNNING  # holding something with nowhere legal to put it (yet) -- keep waiting, not a failure

        self._nav.tick(ctx)

        ready_region = ctx.match.deposit_region_for(robot, self._current.piece)
        if ready_region is not None and ready_region.name == self._current.region.name:
            robot.set_deposit_active(True, action=self._current.action)
        else:
            robot.set_deposit_active(False, action=self._current.action)

        return Status.RUNNING


class Defend(Tactic):
    """Resolves a region to deny -- an explicit name, or (when
    `target == "opponent_intent"`) the nearest opponent's own published
    `intent.target_region`. Holds the blocking pose on the segment
    between that opponent and the region centroid, at `standoff` from
    the region, facing the opponent. Never terminates on its own --
    day-one denial is purely positional (parking in the way); there's
    no pushing-power/contact-penalty model yet."""

    PARAM_SCHEMA = (
        Param("target", kind="str", default="opponent_intent"),
        Param("standoff", kind="float", default=24.0, min=0, suffix=" in"),
        Param("engage_range", kind="float", default=200.0, min=0, suffix=" in"),
    )

    def __init__(self, target: str = "opponent_intent", standoff: float = 24.0, engage_range: float = 200.0, replan_period: float = 0.1):
        self.target = target
        self.standoff = standoff
        self.engage_range = engage_range
        self.replan_period = replan_period
        self.target_region_name: str | None = None
        self._nav = NavigateTo(
            self._provide_target, heading_mode="face_target", replan_period=replan_period, avoid_robots=False
        )

    def reset(self) -> None:
        self.target_region_name = None
        self._nav.reset()

    def _resolve_region_name(self, ctx: BehaviorContext, opponent) -> str | None:
        if self.target != "opponent_intent":
            return self.target
        intent = getattr(opponent, "intent", None)
        return getattr(intent, "target_region", None) if intent is not None else None

    def _pick_opponent(self, ctx: BehaviorContext):
        opponents = world_view.opponents(ctx.match, ctx.robot.alliance)
        opponents = [o for o in opponents if self._resolve_region_name(ctx, o) is not None]
        if not opponents:
            return None
        origin = ctx.robot.pose.translation
        return min(opponents, key=lambda o: origin.get_distance(o.pose.translation))

    def _provide_target(self, ctx: BehaviorContext) -> Pose2d:
        robot = ctx.robot
        opponent = self._pick_opponent(ctx)
        if opponent is None:
            self.target_region_name = None
            return robot.pose

        region_name = self._resolve_region_name(ctx, opponent)
        region = world_view.region_by_name(ctx.match, region_name) if region_name else None
        if region is None:
            self.target_region_name = None
            return robot.pose
        self.target_region_name = region.name

        rx, ry = world_view.region_centroid(region)
        ox, oy = opponent.pose.x, opponent.pose.y
        dx, dy = rx - ox, ry - oy  # opponent -> region
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            block_x, block_y = rx, ry
        else:
            t = min(1.0, self.standoff / dist)  # fraction of the segment back from the region
            block_x, block_y = rx - dx * t, ry - dy * t

        heading = math.atan2(oy - block_y, ox - block_x)
        return Pose2d(block_x, block_y, heading)

    def tick(self, ctx: BehaviorContext) -> Status:
        self._nav.tick(ctx)
        return Status.RUNNING
