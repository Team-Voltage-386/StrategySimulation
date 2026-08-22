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
from typing import Literal, NamedTuple, NamedTuple

from common_sim.control import utility, world_view
from common_sim.control.behavior import Behavior, BehaviorContext, Status
from common_sim.control.navigation import (
    NavigateTo, clear_standoff, estimate_travel_time, polygon_distance,
)
from common_sim.control.param import Param
from common_sim.control.planning import GreedyRatePlanner, ScorePlanner
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

# How long a robot sticks with a target it has just switched to before
# it is willing to switch again. A defender re-reads our published intent
# and gives chase, so without this the two would trade moves every replan
# -- we move, it follows, we move -- and neither the defender's denial
# nor our scoring would ever complete. One commitment window is long
# enough to actually arrive and deposit at a target the defender was not
# already sitting on.
_EVADE_COMMIT_PERIOD = 3.0

# How long a robot keeps at a scoring attempt that isn't completing
# before re-opening the question of what to score and where: this
# multiple of the time the attempt *should* have taken (travel +
# deposit, priced when the target was picked), floored at
# `_STALL_PATIENCE_MIN` so a target already within arm's reach still
# gets a fair few seconds.
#
# This is not a defense mechanism -- it's the escape hatch for a Score
# that has committed to something it cannot finish, from any cause. The
# worst case is a robot that scoops up a piece its strategy has no plan
# for (a stray ALGAE picked up in passing by a CORAL cycler): the
# planner ranks that piece's one region top, the region never comes
# free, and with no re-pick the robot holds it and stops cycling for the
# rest of the match. Measured on a 2v2: 140s of a 150s match spent
# holding one un-scoreable piece.
#
# It is also, in practice, the whole of the counter-defense. Comparing
# arrival times ("could a defender get there before me?") looks like the
# principled test and turns out to be useless for the case that matters:
# a defender already parked on the spot we're parked at wins that race by
# roughly nothing, because our remaining travel time -- the thing it has
# to beat -- has gone to zero. Which is precisely what being denied *is*:
# both robots stationary, one of them not scoring. Time spent failing is
# the only signal that separates it from a spot merely busy for a moment.
_STALL_PATIENCE_RATIO = 1.6
_STALL_PATIENCE_MIN = 2.5

# How long a target a robot gave up on stays out of the running, so that
# giving up on it also means not immediately re-picking it (see
# Score._allowed). Without this, a robot denied at its best option
# ping-pongs between its top two: it gives up on A while standing at A
# (where B now ranks best), drives to B, gives up on B at B, and drives
# back, forever. Measured 1v1 against a blocker: 14 switches between the
# PROCESSOR and the NET, 55s holding one ALGAE, no attempt completed.
#
# Worth reading as a cautionary tale about measurement, not just a
# constant. This was implemented, measured *worse* than the ping-pong
# (15.5 against 19.0), and deleted -- because it was benchmarked on a
# Score that could not convert even when it did get somewhere free. On
# the fixed code the same change is worth 48.9 -> 54.6 alone and
# 71.0 -> 95.4 alongside alliance-aware aiming
# (world_view.region_approach_point), and costs nothing undefended.
# A behavior downstream of a broken one cannot be measured at all.
_FAILED_TARGET_COOLDOWN = 8.0

# The same escape hatch as `_STALL_PATIENCE_*` above, for a robot on its
# way to a collection station rather than to a scoring region. Priced the
# same way (this multiple of what the trip should have taken, floored),
# and it exists because commitment to a station is otherwise permanent by
# construction: `_better_station_exists` only fires when the station is
# *full*, and full means an opponent physically standing on the feed or
# out-racing us to it. A defender denying a station does neither. It
# parks in the approach -- outside the polygon, so it never reads as
# engaged -- and then simply leans on whoever arrives. Time spent failing
# is the only signal that separates that from a station busy for a
# moment, exactly as it is for Score.
#
# The floor is the whole mechanism in REEFSCAPE and the ratio never binds
# here: a station trip costs ~1.6s and the priced ETA overestimates by 2x
# (actual/priced measured at 0.50 median, 1.56 worst *under denial*), so
# `ratio * eta` is always under the floor -- every firing observed was at
# exactly 8.0/8.0. The ratio stays because it is the part that generalises:
# on a longer field, or a game whose supply is a drive away rather than a
# corner away, it is the term that keeps the budget proportionate.
#
# The floor has to be set high, which took measuring at two field sizes to
# see. A low one fires constantly, on short trips, where the delay is a
# queue rather than a defender: at 2v2 a 3s floor was worth -3.5 against
# block/supply, -7.1 against shadow/supply and +9.2 against shadow/any,
# which is noise in three directions. At 8s it is bit-identical to no
# escape at all on 5 of the 7 defense plans, because 2v2 commitments are
# short (median 0.7-2.0s) and never reach the budget.
#
# At 3v3 the long commitments appear -- 25% of blue's station time in
# commitments of 10-17s with a defender parked ~23in off the station, a
# 10x overrun on the same 1.6s trip -- and a floor high enough to be
# silent at 2v2 still catches them.
#
# Be honest about what this bought: essentially nothing on the mean
# (totals across the whole defense grid, 2v2 215.3 -> 215.5, 3v3 260.8 ->
# 261.2). What it moved was the *spread*, on the one plan where those long
# commitments live: 3v3 block/any went 269.8 +-25.9 to 276.5 +-12.4. That
# is the shape of a failure mode being removed rather than an average
# being nudged, and it is the reason this ships despite a flat mean.
_STATION_PATIENCE_RATIO = 1.6
_STATION_PATIENCE_MIN = 8.0

# How long a station a robot gave up on stays out of the running. Without
# it the robot re-picks the station it just abandoned the instant it
# steps away from it (from a few feet back, the denied station is nearest
# again), and the give-up does nothing but reset the clock.
#
# It has to be comfortably longer than the patience above, which is not
# obvious and cost a measurable regression to learn: at 8s each -- the
# two equal -- a robot abandoned feeder A, drove to feeder B, spent
# exactly the patience there, and found A's cooldown had just expired.
# One 3v3 seed spent 96 of 150s alternating 0,1,0,1,0,1 and lost 9
# pieces. The give-up has to outlast the trip it sends you on.
#
# Kept on the tactic across `reset()` on purpose: Collect is re-entered
# from scratch every cycle, so a cooldown cleared on reset would be wiped
# before it was ever read -- the same trap that made Score's cooldown a
# silent no-op the first time it was written.
_FAILED_STATION_COOLDOWN = 20.0

# The same escape hatch again, for a committed loose PIECE rather than a
# station or a scoring region. This is the third instance of one idea, and
# the reason it needed a third is that a piece is the one target whose
# commitment had no time-based release at all: `_losing_piece_race` and
# `_better_piece_exists` are the only ways off one, and both ask "is
# somebody else getting there first, or is something else closer?" -- not
# "am I getting there?".
#
# Neither can see a blocked piece, for the same reason `_better_station_exists`
# cannot see a blocked station. A defender denying a piece does not declare
# it (it declares the *robot* it is marking), so there is no contention to
# lose a race to; and `estimate_travel_time` routes around field geometry
# only, never robots, so the ETA to a piece one foot behind a parked
# opponent reads as one foot. Being close and stopped is exactly what
# `_better_piece_exists` reads as success.
#
# The case it was measured on: a CORAL or ALGAE at rest inside a corner
# CORAL STATION's polygon, with a `deny=supply` defender parked on that
# station's mouth and backed against the two field walls. The piece is
# behind it, unreachable, and 20in away. Over 8 seeds of
# blue={algae_processor, cycle_coral} vs red={full_defense, cycle_coral},
# eight commitments ran past 8s and totalled 132s of alliance time -- the
# worst seed spent 88.6s of 150 on three of them and scored 210 against a
# 228 mean. Every one was a corner piece with the nearest opponent 16-22in
# from it.
#
# Priced like the station budget (that multiple of what the trip should
# have cost, floored) and floored at the same value, for the same reason:
# a low floor fires on trips that are merely slow rather than denied. A
# piece differs from a station in being cheap to give up -- there are
# usually several, and the station is always there as the fallback -- but
# it is also cheap to give up *wrongly*, so the floor stays where the
# station measurements put it rather than being tuned down on the argument
# alone.
_PIECE_PATIENCE_RATIO = 1.6
_PIECE_PATIENCE_MIN = 8.0

# How much closer to the piece the robot has to get for the trip to count
# as still going, which resets the patience clock: the budget above is
# spent on time making *no progress*, not on elapsed time.
#
# The station and region escapes do not need this and this one does,
# because the two failure modes are not the same shape. A station is a
# fixed spot and a station trip is short and stereotyped, so overrunning
# its budget really does mean denial. A loose piece is wherever it came to
# rest -- tucked under the REEF, out at a wall, behind field structure --
# and reaching one legitimately takes anywhere from half a second to most
# of a cycle, so plain elapsed time cannot tell "denied" from "far away
# and awkward".
#
# Measured: with elapsed time alone at the same 8s floor, the mixed-blue
# defense grid lost 14.9 points across the seven ALGAE-cycler rows and,
# tellingly, 1.2 of them on the *undefended* control -- where nothing is
# blocking anything, so every firing there was a legitimate trip thrown
# away. Ratcheting on the closest approach so far fixes that by
# construction: a trip that is still closing never spends budget, however
# long it takes, and a robot held at 20in for 8s spends all of it.
#
# Compared against the *best* distance so far, not the last one, so a
# detour that temporarily increases the distance (routing around the REEF)
# is not mistaken for progress on the way back in. Sized just above the
# noise a stationary robot shows against a piece that is still settling,
# and well under the intake reach it is trying to close.
_PIECE_PROGRESS_EPSILON = 3.0

# The same ratchet, for a committed *station* (see `_station_stalled`).
#
# The piece escape's docstring used to say the station escape did not need
# a progress test. That was wrong, and the case it misses is not subtle: a
# robot routing around the REEF to an ALGAE staging position on the far
# face can wedge its bumper against the hex and command ~107 in/s into it
# for the rest of the match without moving an inch. Every existing release
# is blind to that -- the station is not full, not empty, and has no
# opponent on it, so the elapsed clock runs past its patience and then
# stops at `_better_station_exists`, which wants somewhere else of the
# same type to go. Once the alliance's other five ALGAE positions are
# consumed there is nowhere, and the commitment becomes permanent.
#
# Measured on blue={pursue_tuned} vs block/supply, 24 seeds: 3 of them
# collapsed to 55-64 points against a 212 median, one robot frozen for
# 120s of a 150s match. Distance to the aim point, not to the centroid,
# so queueing a footprint back (see `_station_aim`) is not read as a
# stall.
_STATION_PROGRESS_EPSILON = 3.0

# How long a piece given up on stays out of the running -- longer than the
# patience above, for the reason `_FAILED_STATION_COOLDOWN` spells out at
# length: a give-up has to outlast the trip it sends you on, or the robot
# arrives at its new target just as the old one becomes pickable again and
# alternates between the two. Kept across `reset()` for the same reason
# the station cooldowns are.
_FAILED_PIECE_COOLDOWN = 20.0

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


class _Throttle:
    """Fires at most once per `period` of accumulated `dt`, for the
    expensive re-evaluation a tactic wants on its replan cadence rather
    than every physics tick (obstacle-routed ETAs, one per candidate).
    The cheap per-tick checks around it still run every tick."""

    __slots__ = ("period", "_elapsed")

    def __init__(self, period: float):
        self.period = period
        self._elapsed = 0.0

    def ready(self, dt: float) -> bool:
        self._elapsed += dt
        if self._elapsed < self.period:
            return False
        self._elapsed = 0.0
        return True

    def reset(self) -> None:
        self._elapsed = 0.0


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
    targeted piece is taken by someone else first.

    A committed *station* is given up on three ways: it fills up and
    another has room (`_better_station_exists`), an opponent is holding
    it and some other feeder is only busy with a teammate
    (`_held_by_opponent`), or the trip has simply overrun its budget
    (`_station_stalled`). The last of those is the only one that can see
    a defender denying a feeder from the approach, since standing in the
    way makes a station neither full nor occupied."""

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
        self._reconsider = _Throttle(replan_period)
        self._nav = NavigateTo(self._provide_target, heading_mode="face_target", replan_period=replan_period)

        # Stall escape for a committed station (see
        # `_STATION_PATIENCE_*`). `_station_cooldowns` deliberately
        # survives `reset()` -- it is the only piece of state here that
        # has to outlive one collect cycle to mean anything.
        self._station_elapsed = 0.0
        self._station_patience = _STATION_PATIENCE_MIN
        self._station_closest = math.inf
        self._station_stuck = 0.0
        self._station_cooldowns: dict[str, float] = {}

        # The same escape for a committed loose piece (see
        # `_PIECE_PATIENCE_*`). `_piece_cooldowns` is keyed by the piece
        # object, not an id() -- an id is reused once a piece is garbage
        # collected, which would silently put a fresh piece on the cooldown
        # of a dead one -- and it survives `reset()` like the station one.
        self._piece_elapsed = 0.0
        self._piece_patience = _PIECE_PATIENCE_MIN
        self._piece_closest = math.inf
        self._piece_cooldowns: dict[object, float] = {}

    def reset(self) -> None:
        self._target_piece = None
        self._target_station = None
        self._start_held_count = None
        self._station_elapsed = 0.0
        self._station_closest = math.inf
        self._station_stuck = 0.0
        self._piece_elapsed = 0.0
        self._piece_closest = math.inf
        self._reconsider.reset()
        self._nav.reset()

    @property
    def has_target(self) -> bool:
        """Whether a trip is under way right now -- for an arbiter that
        needs to know before changing the job out from under it."""
        return self._target_piece is not None or self._target_station is not None

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

    def _served_by_teammate(self, ctx: BehaviorContext, station: IntakeLocation) -> bool:
        """Whether one of ours is on `station`'s feed right now -- so the
        wait we are in is a queue rather than a denial."""
        return any(other is not ctx.robot and other.alliance == ctx.robot.alliance
                   and _robot_engaged_with_station(other, station)
                   for other in ctx.match.robots)

    def _held_by_opponent(self, ctx: BehaviorContext, station: IntakeLocation) -> bool:
        """Whether an opponent is physically on `station`. Engagement
        only, not a declared claim: a claim is a race we might win (see
        `_station_has_room_for`), and treating one as possession hands a
        defender both feeders for the price of announcing them."""
        return any(other.alliance != ctx.robot.alliance
                   and _robot_engaged_with_station(other, station)
                   for other in ctx.match.robots)

    def _best_station(self, ctx: BehaviorContext) -> tuple[IntakeLocation | None, float]:
        match, robot = ctx.match, ctx.robot
        origin = robot.pose.translation
        characteristics = robot.characteristics
        stations = world_view.station_options(match, robot)
        if self.piece_type is not None:
            stations = [s for s in stations if s.piece_type == self.piece_type]
        if not stations:
            return None, math.inf

        # Stations recently given up on drop out -- unless that leaves
        # nothing, in which case every feeder is being denied at once and
        # going back to the nearest one and waiting is the best available
        # play.
        #
        # That fallback stops at a station an opponent is *physically*
        # parked on, and without that exception the give-up above is a
        # no-op: the robot abandons the station, re-picks the only one of
        # its type on the very next tick, and the cooldown does nothing
        # but reset a clock. Waiting behind a teammate is a queue and
        # resolves itself; waiting behind an opponent is the thing the
        # opponent came to do. Engagement only, matching
        # `_held_by_opponent` -- a mere claim is a race we might still
        # win, and conceding to one hands a defender every feeder for the
        # price of announcing it.
        #
        # Returning nothing here is the useful answer rather than a gap,
        # and it is the same answer the piece side already gives: with no
        # station left, `_pick_target` weighs loose pieces, and failing
        # that Collect reports FAILURE -- which is what lets the *caller*
        # switch jobs. Pursue re-arbitrates to the other piece type; a
        # rule-based strategy falls through to its next rule. No tactic
        # pinned to one piece type can make that call from inside its own
        # scope. Measured on shadow/supply seed 1004: both blue robots
        # queued 78s past an 8s patience at the one REEF ALGAE position a
        # red defender was sitting on, with two CORAL STATIONS free and
        # offered, and the match ended 51-234.
        available = [s for s in stations if s.name not in self._station_cooldowns]
        if not available:
            available = [s for s in stations if not self._held_by_opponent(ctx, s)]
        if not available:
            return None, math.inf
        stations = available

        # Same rule Score._pick_option applies to scoring regions: take
        # one nobody is working before contesting one. Two robots both
        # picking "nearest" otherwise converge on the same corner, and a
        # REEFSCAPE CORAL STATION is 36x36 against a 28x28 robot --
        # capacity 1, no room to share. Falling back to the crowded
        # nearest is fine because _station_aim then queues outside it
        # rather than barging in; with every station busy, waiting for
        # the nearest is exactly the right thing to do.
        #
        # `_station_has_room_for` rather than the plain occupancy count,
        # so a claim only takes the station away from us if its owner
        # would actually get there first. The two differ exactly when a
        # rival has *declared* a station it has not reached, which is the
        # normal state of an opponent moving to deny one: on a bare
        # occupancy count a defender takes a station off the list by
        # announcing it, from anywhere on the field, and offense concedes
        # a race it was winning. Conceding is still right once the
        # defender is physically there -- that reads as engaged, not as
        # a claim, and no ETA beats it.
        roomy = [s for s in stations if self._station_has_room_for(ctx, s)]

        # With nothing roomy, *who* has the room matters and the plain
        # "wait at the nearest" fallback ignores it. Queueing behind a
        # teammate costs the couple of seconds it takes them to load;
        # queueing behind an opponent costs however long they feel like
        # standing there, because being in the way is the entire reason
        # they are at a feeder they cannot use. So a station an opponent
        # is holding is a last resort among last resorts -- go stand
        # behind the teammate, even if it is farther.
        #
        # The weaker of the two changes here, and worth saying so: it
        # fires on ~2% of station picks, and it measured +8.3 on 2v2
        # block/supply (spread 15.7 -> 12.4) but -2.7 on the *same plan*
        # at 3v3. Opposite signs on one plan is not signal. It ships on
        # the argument rather than the number, so treat it as untested
        # rather than established, and re-measure it before building
        # anything on top of it.
        candidates = roomy or [s for s in stations if not self._held_by_opponent(ctx, s)] or stations

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
        # Pieces there is no room for. `collectable_pieces` answers "what
        # is lying around that this robot's intakes accept" -- physical
        # capability, which is the right question for its other callers
        # -- and deliberately says nothing about how full the robot is
        # right now. `world_view.station_options` applies this same check
        # itself, so a station never had the hole; a loose piece did.
        #
        # Nothing hit it until a caller left `piece_type` unset, because
        # a typed Collect is protected by its own SUCCESS test
        # (`is_full_for(self.piece_type)`) and by the trigger that fired
        # it. Untyped, both go away: `is_full_for(None)` means "nothing
        # can be taken at all", which is false for a robot holding one
        # CORAL with a free ALGAE slot. Measured on a Pursue robot that
        # picked a CORAL 19in off while already holding one -- it parked
        # in intake range and stayed there for 130 of 150s, scoring
        # nothing. Even the stall escape could not save it:
        # `_piece_stalled` stops its clock while `robot.accepts(piece)`
        # is true, on the principle that an intake under way should get
        # to finish, and this intake was under way forever.
        pieces = [p for p in pieces if not robot.is_full_for(p.piece_type)]
        if self.max_range is not None:
            pieces = [p for p in pieces if origin.get_distance(p.position) <= self.max_range]

        # Pieces recently given up on drop out, with no "unless that
        # leaves nothing" fallback -- and that is the one place this
        # deliberately does NOT copy `_best_station`. A station given up on
        # is still the only source of its piece type and regenerates
        # supply, so going back and waiting on it beats touring the field.
        # A piece is not: an unreachable piece is unreachable, and waiting
        # on the last one on the field is precisely the 22s stall this
        # cooldown exists to end. Leaving nothing here is the useful
        # answer, because `_pick_target` then fails and Collect reports
        # FAILURE, which is what lets the *strategy* switch jobs -- an
        # algae cycler with no gettable algae going to cycle CORAL
        # instead. No tactic can make that call from inside its own scope.
        pieces = [p for p in pieces if p not in self._piece_cooldowns]
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
            self._commit_station(ctx, station, station_eta)
            return True
        if piece is not None:
            self._commit_piece(ctx, piece, piece_eta)
            return True

        self._target_piece = None
        self._target_station = None
        return False

    def _commit_station(self, ctx: BehaviorContext, station: IntakeLocation, eta: float) -> None:
        """Take `station` as the target and restart its patience clock,
        budgeted off what the trip should cost from here -- drive plus one
        feed -- so a station across the field is given the time to reach
        it and one already underfoot is not. Re-committing to the station
        already held restarts nothing: the clock is there to measure how
        long this attempt has been going, and a re-pick that lands where
        it started is the same attempt continuing."""
        if station is not self._target_station:
            self._station_elapsed = 0.0
            self._station_closest = math.inf
            self._station_stuck = 0.0
            self._station_patience = max(
                _STATION_PATIENCE_MIN,
                _STATION_PATIENCE_RATIO * (eta + ctx.robot.characteristics.station_intake_time),
            )
        self._target_station, self._target_piece = station, None

    def _commit_piece(self, ctx: BehaviorContext, piece, eta: float) -> None:
        """`_commit_station`'s counterpart for a loose piece: take it and
        restart its patience clock, budgeted off drive plus one intake from
        here. Re-committing to the piece already held restarts nothing --
        a re-pick that lands where it started is the same attempt
        continuing, and restarting there would make the clock unable to
        ever expire, since `_pick_target` runs again the moment anything
        else changes."""
        if piece is not self._target_piece:
            self._piece_elapsed = 0.0
            self._piece_closest = math.inf
            self._piece_patience = max(
                _PIECE_PATIENCE_MIN,
                _PIECE_PATIENCE_RATIO * (eta + ctx.robot.characteristics.intake_duration(piece.piece_type)),
            )
        self._target_piece, self._target_station = piece, None

    def _piece_stalled(self, ctx: BehaviorContext) -> bool:
        """Whether the committed piece has gone long enough without the
        robot getting any closer to it that it is worth going somewhere
        else.

        What the budget is spent on is time making no progress, not elapsed
        time -- see `_PIECE_PROGRESS_EPSILON` for why. `_station_stalled`
        carries the same ratchet for the same reason; it did not
        originally, and `_STATION_PROGRESS_EPSILON` records what that
        cost. Every fresh closest approach restarts
        the clock, so a slow trip that is still closing is never given up
        on, however far the piece is or however awkwardly it is placed.

        The clock also stops once the piece is in intake range and acceptable
        (`robot.accepts`, the same test the physics uses to feed the intake
        timer): an intake already under way always gets to finish, exactly
        as `_station_stalled` exempts a robot already on the feed. What is
        being timed is the part a defender can deny -- getting there.

        There is no teammate exemption to match the station's, and that
        asymmetry is the point. A teammate ahead of us at a feeder is a
        queue that clears; a teammate ahead of us at a *piece* is not
        queueing, it is taking it, and that is already `_losing_piece_race`
        and `_piece_rank`'s first tier. The only thing left for a clock to
        catch here is a trip that is not happening.

        Unlike the station escape this does *not* require somewhere fresh
        to give up to first, and that was the whole finding: on the
        measured case there was nowhere, because the blocked piece was the
        last ALGAE on the field and blue's REEF was empty. Requiring an
        alternative makes the escape silent in exactly the situation that
        hurts most -- one unreachable piece, and the robot's whole job
        depending on it. Giving up with nothing in scope to give up to is
        not a dead end, it is a FAILURE the strategy can act on."""
        piece = self._target_piece
        robot = ctx.robot
        if piece is None or robot.accepts(piece):
            return False

        distance = math.hypot(piece.position.x - robot.pose.x, piece.position.y - robot.pose.y)
        if distance < self._piece_closest - _PIECE_PROGRESS_EPSILON:
            self._piece_closest = distance
            self._piece_elapsed = 0.0
            return False

        self._piece_elapsed += ctx.dt
        return self._piece_elapsed >= self._piece_patience

    def _wedged(self, ctx: BehaviorContext) -> bool:
        """Whether the chassis is up against a static obstacle.

        The one signal that separates "a defender is holding me off the
        feed" from "my bumper is on the REEF", which are otherwise the
        same observation -- a robot that is not moving and not arriving.
        Obstacles are field geometry, so unlike a defender they are never
        going to clear, and a commitment that depends on one moving is a
        commitment for the rest of the match.

        Deliberately a footprint test, not a contact test: the physics
        contact set is not exposed here, and a chassis whose half-extent
        overlaps an obstacle boundary is wedged closely enough for the
        purpose whether or not a solver pair happens to be live this
        tick."""
        field = getattr(ctx.match, "field", None)
        obstacles = getattr(field, "obstacles", ())
        if not obstacles:
            return False
        characteristics = ctx.robot.characteristics
        half_extent = math.hypot(characteristics.width, characteristics.length) / 2.0
        point = (ctx.robot.pose.x, ctx.robot.pose.y)
        return any(polygon_distance(point, o.vertices) < half_extent for o in obstacles)

    def _station_stalled(self, ctx: BehaviorContext) -> bool:
        """Whether the committed station has taken long enough without
        delivering that it is worth trying the other one.

        The clock stops once the robot is actually on the feed
        (`_holds_station`): an intake already under way always gets to
        finish, the same way Score never shops for a new region while the
        deposit it has is legal. What it is timing is the part a defender
        can actually deny -- getting there.

        It also stops while a *teammate* is on the feed, which is the
        distinction the whole mechanism turns on. A teammate ahead of us
        is a queue that is moving: it will load and leave, and the wait
        is priced in the trip we already chose. An opponent in the way is
        not a queue at all. Without this the escape cannot tell them
        apart, and at 3v3 -- three robots to two feeders -- it is almost
        always the harmless one. Measured: one seed abandoned the same
        feeder 19 times and gave up 9 pieces.

        Giving up also requires somewhere to give up *to*. With every
        other feeder already on cooldown, this trip is not going badly
        relative to the alternatives -- it is the alternative -- and the
        original fallback is right that waiting at the nearest crowded
        station beats touring the field. The clock keeps running while it
        waits, so the moment a cooldown lapses and a fresh option
        reappears, the escape fires on the next tick."""
        if self._target_station is None or self._holds_station(ctx.robot):
            return False
        if self._served_by_teammate(ctx, self._target_station):
            return False
        # The progress ratchet, exactly `_piece_stalled`'s: every fresh
        # closest approach to the aim point restarts the stuck clock, so a
        # slow trip that is still closing never spends any of it, and a
        # robot that has stopped closing spends all of it. Measured
        # against the *best* approach so far rather than the last one, so
        # routing around the REEF (which increases the distance before it
        # decreases) is not mistaken for a stall.
        aim_x, aim_y = self._station_aim(ctx)
        distance = math.hypot(aim_x - ctx.robot.pose.x, aim_y - ctx.robot.pose.y)
        if distance < self._station_closest - _STATION_PROGRESS_EPSILON:
            self._station_closest = distance
            self._station_stuck = 0.0
        else:
            self._station_stuck += ctx.dt

        self._station_elapsed += ctx.dt
        if self._station_elapsed < self._station_patience:
            return False
        # A trip that has stopped closing *while the chassis is against
        # static geometry* is not a slow trip, it is a trip that is not
        # happening: the bumper is on the REEF and the drive is commanding
        # full speed into it. Released with no alternative required, for
        # the same reason `_piece_stalled` needs none -- requiring one
        # makes the escape silent in precisely the case that costs the
        # most, the last station of a type the arbiter above has pointed
        # us at. See `_STATION_PROGRESS_EPSILON` for the measurement.
        #
        # The obstacle test is what keeps this from swallowing the
        # deliberate decision beside it (`test_collect_keeps_the_only_
        # station_however_long_it_takes`): a robot held off a feeder by a
        # *defender* is stationary too, and from inside this tactic the
        # two are otherwise identical. Waiting out a defender is right --
        # it moves eventually, and touring the field instead measured as a
        # loss. Waiting out the REEF is not; it will still be there at the
        # buzzer.
        if self._station_stuck >= self._station_patience and self._wedged(ctx):
            return True
        # An opponent physically on the feed is its own reason to leave,
        # with no alternative required. The "somewhere to give up to"
        # rule below is about preferring a better trip to a worse one,
        # which presumes this trip will eventually complete -- and behind
        # a parked defender it will not. Requiring an alternative *of the
        # same piece type* made that presumption unfalsifiable at the
        # last feeder of a type: a robot Pursue had pointed at ALGAE sat
        # out 70 seconds past its patience at the only ALGAE position on
        # the field because no second ALGAE position existed to leave
        # for, while the CORAL it could have fetched instead went
        # uncollected. Releasing hands the choice up to whoever picked
        # the type, which is the only layer that can change it.
        if self._held_by_opponent(ctx, self._target_station):
            return True
        return any(
            station is not self._target_station
            and station.name not in self._station_cooldowns
            and (self.piece_type is None or station.piece_type == self.piece_type)
            for station in world_view.station_options(ctx.match, ctx.robot)
        )

    def tick(self, ctx: BehaviorContext) -> Status:
        robot = ctx.robot
        for name in list(self._station_cooldowns):
            self._station_cooldowns[name] -= ctx.dt
            if self._station_cooldowns[name] <= 0.0:
                del self._station_cooldowns[name]
        for piece in list(self._piece_cooldowns):
            self._piece_cooldowns[piece] -= ctx.dt
            if self._piece_cooldowns[piece] <= 0.0:
                del self._piece_cooldowns[piece]

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
        reconsider = self._reconsider.ready(ctx.dt)
        target_lost = self._target_piece is not None and (
            self._target_piece.held_by is not None
            or self._target_piece.scored
            or (reconsider and (
                self._rolled_to_opposing_side(ctx)
                or self._losing_piece_race(ctx)
                or self._better_piece_exists(ctx)
            ))
        )
        # The piece counterpart of the station escape below, and the only
        # release a committed piece has that does not depend on somebody
        # else wanting it: the trip has taken too long. A piece sitting
        # behind a parked defender -- typically one at rest in a corner
        # station's polygon, with the defender denying that station backed
        # up against it -- is uncontested (a defender declares the robot it
        # marks, not the piece) and reads as a foot away, so every check
        # above is satisfied while the robot goes nowhere. Guarded on
        # nothing else having released the piece already, so a piece
        # somebody else just took is not also put on cooldown.
        if self._target_piece is not None and not target_lost and self._piece_stalled(ctx):
            self._piece_cooldowns[self._target_piece] = _FAILED_PIECE_COOLDOWN
            self._piece_elapsed = 0.0
            target_lost = True
        # The station counterpart of "somebody else took the piece": the
        # supply ran out. Unconditional, like its piece twin -- no
        # patience clock, no reconsideration cadence, and no requirement
        # that somewhere better exist -- because this is not a trip going
        # badly, it is a target that has stopped being a target at all.
        #
        # Every other release below is guarded on something that an empty
        # station does not satisfy, so without this the commitment is
        # permanent: `_station_stalled` stops its clock while
        # `_holds_station` is true, on the reasonable theory that an
        # intake under way should be allowed to finish -- but nothing is
        # under way, the station has nothing to give -- and
        # `_better_station_exists` wants the committed station to be
        # *full*, which an empty one conspicuously is not. Measured on
        # blue={pursue, pursue} vs block/supply: a robot reached a REEF
        # ALGAE position at t=24, found it emptied, and sat on it with an
        # intake it could not fill for the remaining 126 seconds.
        if self._target_station is not None and not world_view.station_has_supply(ctx.match, self._target_station):
            target_lost = True
        # Two ways off a committed station. The cheap one: it filled up
        # and another has room, which is strictly better than waiting.
        # Guarded on the other station actually having room so this can't
        # cycle between two equally crowded ones.
        elif self._target_station is not None and reconsider and self._better_station_exists(ctx):
            target_lost = True
        # The one that catches a defender: the trip has simply taken too
        # long. A station being denied never reads as full -- the defender
        # is in the approach, not on the feed -- so without this the
        # commitment is permanent and both robots queue behind a trip that
        # is not happening. Elapsed time is the only signal that sees it.
        elif self._station_stalled(ctx):
            self._station_cooldowns[self._target_station.name] = _FAILED_STATION_COOLDOWN
            self._station_elapsed = 0.0
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

    def __init__(
        self,
        planner: ScorePlanner | None = None,
        region: str | None = None,
        action: str | None = None,
        replan_period: float = 0.1,
    ):
        self.planner = planner or GreedyRatePlanner()
        self.region = region
        self.action = action
        self.replan_period = replan_period

        self._current = None  # planning.ScoringOption
        self._committed_elapsed = 0.0
        self._patience = _STALL_PATIENCE_MIN
        self._evade_hold = 0.0
        # (region name, action) -> seconds left before it may be chosen
        # again, for targets this robot gave up on. See _allowed.
        self._cooldowns: dict[tuple[str, str], float] = {}
        self._reconsider = _Throttle(replan_period)
        self._nav = NavigateTo(self._provide_target, heading_mode="face_target", replan_period=replan_period)

    def reset(self) -> None:
        # `_cooldowns` deliberately survives: "this region would not take
        # my piece a moment ago" is knowledge about the field, not state
        # belonging to one activation of this tactic. A strategy that
        # alternates Collect and Score resets Score once per cycle, so
        # clearing here would wipe every cooldown before it was ever
        # consulted -- measured, that reduced the whole mechanism to a
        # no-op (48.9 -> 48.9 against a blocker, against 54.6 with it).
        self._commit(None)
        self._evade_hold = 0.0
        self._reconsider.reset()
        self._nav.reset()

    def _allowed(self, options: list) -> list:
        """`options` minus the ones this robot recently gave up on --
        unless that empties the list, in which case the whole list comes
        back.

        Without this a robot denied at its best option ping-pongs
        between its top two: it gives up on A while standing at A (where
        B now ranks best), drives to B, gives up on B at B, and drives
        back, forever. Measured 1v1 against a blocker: 14 switches
        between the PROCESSOR and the NET, 55s holding one ALGAE, no
        attempt completed. Falling back to the unfiltered list when
        everything is cooling is what keeps the cure from being worse:
        contesting a spot still scores eventually, standing still
        holding the piece never does."""
        if not self._cooldowns:
            return options
        return [o for o in options if (o.region.name, o.action) not in self._cooldowns] or options

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
                self._commit(None)
                return False
            legal = self._allowed(legal)
            self._commit(self._best_uncrowded(ctx, legal) or self._best_valued(ctx, legal))
            return True

        options = self.planner.plan(match, robot, exclude=set(self._cooldowns))
        if not options:
            self._commit(None)
            return False
        best = options[0]
        if world_view.region_has_room(match, best.region, robot):
            self._commit(best)
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
        legal = self._allowed([o for o in world_view.scoring_options(match, robot) if o.piece is best.piece])
        self._commit(self._best_uncrowded(ctx, legal) or best)
        return True

    def _commit(self, option) -> None:
        """Take `option` as the target and restart the patience clock. The
        patience budget is priced off the option itself -- how long the
        attempt *should* take -- so a far region is given the time to
        drive to it and a region already underfoot is not."""
        self._current = option
        self._committed_elapsed = 0.0
        self._patience = _STALL_PATIENCE_MIN if option is None else max(
            _STALL_PATIENCE_MIN, _STALL_PATIENCE_RATIO * (option.travel_time + option.deposit_time),
        )

    def _best_uncrowded(self, ctx: BehaviorContext, legal) -> object | None:
        roomy = [o for o in legal if world_view.region_has_room(ctx.match, o.region, ctx.robot)]
        return self._best_valued(ctx, roomy)

    def _best_valued(self, ctx: BehaviorContext, legal) -> object | None:
        """The best of `legal` by expected points per second.

        The same ranking `GreedyRatePlanner` uses, and it has to be: this
        is the re-pick path -- a pinned region/action, or the planner's
        own choice turning out to be crowded -- so a re-pick ranking on
        a different quantity than the plan would quietly undo it, and
        the robot would price one target and drive to another."""
        if not legal:
            return None
        pos = (ctx.robot.pose.x, ctx.robot.pose.y)
        travel = utility.TravelCache(ctx.match.field, ctx.robot.characteristics)
        built = [utility.score_outcome(ctx.match, ctx.robot, o, pos, travel) for o in legal]
        return max(built, key=lambda o: o.expected_rate).payload

    def _reconsider_target(self, ctx: BehaviorContext) -> None:
        """Re-open the choice of what to score and where once the current
        attempt has run past its patience budget without completing (see
        `_STALL_PATIENCE_*`) -- whether it's a defender in the way, a
        region that filled up, or a piece nothing on the field will
        accept.

        Locked out for `_EVADE_COMMIT_PERIOD` after each actual change of
        target. A defender re-reads our published intent and follows, so
        an unthrottled robot would abandon each new target the moment the
        chaser closed on it and never arrive anywhere to deposit."""
        if self._current is None:
            return
        self._committed_elapsed += ctx.dt
        self._evade_hold = max(0.0, self._evade_hold - ctx.dt)
        if self._evade_hold > 0.0 or not self._reconsider.ready(ctx.dt):
            return
        if self._committed_elapsed < self._patience:
            return

        previous = self._current
        if not self._pick_option(ctx) or self._current is None:
            # Nothing legal to switch to -- keep the target we had rather
            # than dropping to no target at all, which would leave
            # `_provide_target` with nothing to aim at this tick.
            self._commit(previous)
            return
        if self._current.region.name != previous.region.name or self._current.action != previous.action:
            # Only a target actually walked away from goes on cooldown --
            # a re-pick that lands on the same place is the robot still
            # trying, not giving up.
            self._cooldowns[(previous.region.name, previous.action)] = _FAILED_TARGET_COOLDOWN
        if self._current.region.name != previous.region.name or self._current.piece is not previous.piece:
            self._evade_hold = _EVADE_COMMIT_PERIOD
            self._nav.reset()

    def tick(self, ctx: BehaviorContext) -> Status:
        robot = ctx.robot
        for key in list(self._cooldowns):
            self._cooldowns[key] -= ctx.dt
            if self._cooldowns[key] <= 0.0:
                del self._cooldowns[key]

        if not robot.held_pieces:
            robot.set_deposit_active(False)
            return Status.SUCCESS

        if self._current is None or self._current.piece not in robot.held_pieces:
            if not self._pick_option(ctx):
                return Status.RUNNING  # holding something with nowhere legal to put it (yet) -- keep waiting, not a failure
        elif not self._deposit_ready(ctx):
            # Don't shop for a different target while this one is already
            # scorable from where we stand -- the patience clock stops
            # too, so a deposit that has started always gets to finish.
            self._reconsider_target(ctx)

        if self._deposit_ready(ctx):
            # Stop. `_provide_target` aims at a *nominal* point in the
            # region, but any pose that satisfies `deposit_region_for` is
            # already a scoring pose, and the deposit timer only survives
            # while the pose stays legal. Driving on toward the aim point
            # from a legal pose therefore cancels the very deposit it was
            # trying to set up -- invisible on a region whose legal area
            # is barely bigger than the aim tolerance, fatal on a large
            # one. REEFSCAPE's NET is 80x285in: a robot entered it,
            # latched the deposit, drove ~40in further to the aim point
            # and left the far side still holding the ALGAE, over and
            # over. Nearest-legal-pose-wins is also what makes a large
            # region hard to defend, which is the point of a large
            # region: the defender has to deny the whole area, not one
            # spot in it.
            robot.drive_field_relative(ctx.dt, 0.0, 0.0, 0.0)
            robot.set_deposit_active(True, action=self._current.action)
            return Status.RUNNING

        self._nav.tick(ctx)
        robot.set_deposit_active(False, action=self._current.action)
        return Status.RUNNING

    def _deposit_ready(self, ctx: BehaviorContext) -> bool:
        """Whether a deposit finishing on this tick would score in the
        committed region. Publishes the action first because
        `Match.deposit_region_for` resolves against the action the robot
        has latched, so asking before setting it answers for the
        *previous* target's action."""
        if self._current is None:
            return False
        robot = ctx.robot
        robot.set_deposit_active(robot.deposit_active, action=self._current.action)
        region = ctx.match.deposit_region_for(robot, self._current.piece)
        return region is not None and region.name == self._current.region.name


# How much of a job's value a robot already on the field feature it needs
# takes away, per robot (see `Pursue._pressure`).
#
# An opposing defender declaring your target halves the job. A teammate
# already working it costs far less, because a teammate is a queue that
# is moving -- it will load or score and leave -- while an opponent is
# not a queue at all. That is the same distinction Collect draws in
# `_station_stalled`, for the same reason.
#
# Measured on the 2v2 defense bench, 8 seeds, against the same tactic
# with both at zero: +23.0 points on block/supply and +8.5 on
# shadow/supply -- the two rows where a defender camps the feeder, which
# is what the term is for -- and roughly a wash elsewhere, except
# block/any at -16.5 which is not explained. Summed across the grid it
# is +23.8, which at ~5-9 points of standard error per row is a weak
# result carried by one strong row. Kept because the mechanism is
# principled and the row it targets moves a lot; both are flat float
# Params, so Phase F's search tunes them without new code.
CONTEST_PENALTY = 0.5
CLAIM_PENALTY = 0.2


# How much room a job has to leave on the clock to be worth its full
# value to Pursue (see `Pursue._time_fit`). A job that will finish with
# this many seconds to spare is priced at face value; one that exactly
# fills the time remaining is priced at zero, and the ramp between the
# two is where an ETA that is a little optimistic gets caught.
#
# One CORAL cycle on the REEFSCAPE bench is 8-12s, so this is roughly
# "half a cycle of slack": long enough to cover the error in an
# obstacle-routed estimate, short enough that it does not start refusing
# work with twenty seconds left.
TIME_FIT_SLACK = 4.0


# How much of a deposit's miss rate Pursue charges it for (see
# `Pursue._reliability`). 1.0 is full expected value, which makes this
# tactic's numerator identical to the `Outcome.expected_rate` the
# planner beneath it already ranks targets by -- so the job Pursue
# prices is the job its Score child performs.
#
# The default is that coherence rather than a measured gain: on the 2v2
# defense bench, 8 seeds, 1.0 against 0.0 is -1.5 / +13.8 / -14.3 / +1.9
# / -9.3 / +1.0 / +3.4 across the seven red plans, summing to -5.0 --
# inside the noise of a seven-row sum. What the measurement settles is
# the older number: the same weight lost 48.9 summed while the planner
# still ranked on gross points, so nearly all of that was the two layers
# disagreeing rather than expected value being wrong.
RELIABILITY_WEIGHT = 1.0


class _Ranking(NamedTuple):
    """One arbitration's answer: what each job is worth per second, and
    which flavour of it was priced.

    The piece type is handed down to Collect, because *what kind of
    thing is worth fetching* is the value judgement Pursue just made and
    Collect ranks supply on ETA alone -- it cannot re-derive one from
    the other.

    There is deliberately no matching action for Score. Handing one down
    was tried and measured: it cost 51 points a match under
    `block/scoring`, because `_score_rate` ranks a single next deposit
    while Score's planner ranks a *sequence* over everything held, and a
    REEF branch's value depends on what is already on it
    (`match.region_full`). Piece type is a genuine either/or -- fetch
    CORAL or fetch ALGAE, not both. Which action to score is not: the
    robot will put down everything it carries eventually, so that is a
    sequencing question, and sequencing is what the planner is for.
    Pinned to the one-step argmax, a robot committed to PROCESSOR on 68
    of 93 re-picks and scored 32 pieces where the planner scored 43."""

    score_rate: float
    collect_rate: float
    collect_type: "str | None"


class _Prospects:
    """The context one arbitration pass values its outcomes in: how much
    match is left, and how contested each field feature is.

    Both are per-pass rather than per-outcome because neither can change
    while a pass runs and both are asked over and over inside one.
    `region_occupants` in particular walks every robot on the field and
    runs a point-in-polygon per one, while an arbitration prices a
    deposit per (region, action) pair -- REEFSCAPE's four levels across
    six REEF faces bring the same six polygons round four times each.
    The same redundancy `utility.TravelCache` exists for, and the same
    fix.

    Duck-typed on `match` throughout, as world_view is: a stub match
    with no `config` reports an unknown clock rather than an expired
    one, because "the match is over" zeroes every job on the board and
    is an expensive thing to say by accident.
    """

    __slots__ = ("match", "robot", "remaining", "_contention")

    def __init__(self, match, robot):
        self.match = match
        self.robot = robot
        total = getattr(getattr(match, "config", None), "total_duration", None)
        self.remaining = None if total is None else total - match.elapsed
        self._contention: dict[str, tuple[int, int]] = {}

    def contention(self, feature) -> tuple[int, int]:
        """(opposing defenders denying `feature`, everybody else working
        it), for a scoring region or an intake location -- which are the
        same `.name` + `.vertices` pair to everything downstream. A
        ScoringOption is unwrapped to the region it names, so a caller
        can hand over an Outcome's payload without knowing which kind it
        got.

        The two counts are disjoint on purpose. A denier declares the
        feature as its `intent.target_region`, which is exactly what
        makes `region_occupants` count it too, so charging both raw
        would price one defender as two robots and leave the two
        penalties impossible to weigh against each other.

        (0, 0) for anything that is neither: a loose piece on the floor
        has no name for a defender to declare and no polygon to stand
        in, so nobody can be denying it."""
        feature = getattr(feature, "region", feature)
        name = getattr(feature, "name", None)
        if name is None or not getattr(feature, "vertices", None):
            return (0, 0)
        counts = self._contention.get(name)
        if counts is None:
            deniers = len(world_view.region_denied_by(self.match, name, self.robot.alliance))
            occupants = len(world_view.region_occupants(self.match, feature, exclude=self.robot))
            counts = self._contention[name] = (deniers, max(0, occupants - deniers))
        return counts


class Pursue(Tactic):
    """Chooses between *fetching* and *scoring* by what each is worth per
    second, then hands the job to Collect or Score to actually do.

    This is the one decision the rule layer could not express. A Rule
    picks a tactic by a hand-written integer `priority`, so "score the
    ALGAE I'm holding" versus "go get CORAL first" is settled once, at
    authoring time, by a number that cannot know the PROCESSOR is four
    feet away and the station is across the field. Both jobs are already
    priced in the same currency by utility.py; all that was missing was
    something to compare them.

    The comparison is a rate, in points per second of the time the job
    would occupy:

    * scoring is `Outcome.value_rate` -- the points, over travel plus
      deposit.
    * fetching is the *whole cycle* it buys: the deposit's points over
      (drive to the pickup + intake + drive to the region + deposit).
      A pickup on its own scores nothing, so pricing it any other way
      would rank every collect at zero.

    Both denominators are "seconds this robot is committed for", which is
    what makes them comparable at all, and neither needs a weight to be
    on the same scale. `lookahead_weight` exists anyway, at a default of
    1.0 that changes nothing, because the fetch half is a *prediction*
    and the score half is in hand: the region may fill, a defender may
    arrive, another robot may take the piece. How much to discount that
    is a question for measurement (and for a parameter search -- it is a
    flat float Param, so `strategy_params` picks it up for free), not for
    a number invented here.

    **It arbitrates the job, not the target.** Once fetching wins, Collect
    picks *which* piece or station by its own ETA tiers, contention races
    and patience clocks; Score likewise picks its own region. That is
    deliberate: those mechanisms are what stop two robots converging on
    one feeder and what gets a robot off a target a defender is denying,
    and none of it is re-derivable from a rate. The consequence to know is
    that the rate Pursue compares is the *best* pickup's, while Collect
    may set off for a different one -- so the fetch side is an optimistic
    bound, not a promise.

    Commitment is the other half. Without it a robot re-decides ten times
    a second and drives the midpoint of the two jobs: `min_commit` is how
    long a job runs before the question is re-opened at all, and
    `switch_margin` is how much better the other one has to look before
    the answer changes. Both are floors under dithering, not tuning for
    quality -- the same shape as Defend's `_MARK_DWELL`/`_MARK_SWAP_MARGIN`
    and for the same reason.

    Never reports SUCCESS. It is a standing job rather than a task that
    completes, and an `Always`-triggered rule whose tactic reports SUCCESS
    is re-selected by the arbiter on the very next tick -- logging a
    behavior change every tick forever. When neither job has anything to
    do it reports FAILURE, which is the arbiter's channel for "let another
    rule have the robot" and comes with the suppression window that keeps
    the retry from being every tick (see strategy._FAILED_RULE_SUPPRESSION).
    """

    PARAM_SCHEMA = (
        Param("switch_margin", kind="float", default=0.25, min=0.0, max=2.0),
        Param("min_commit", kind="float", default=2.0, min=0.0, suffix=" s"),
        Param("lookahead_weight", kind="float", default=1.0, min=0.0, max=2.0),
        Param("time_fit_slack", kind="float", default=TIME_FIT_SLACK, min=0.0, suffix=" s"),
        Param("reliability_weight", kind="float", default=RELIABILITY_WEIGHT, min=0.0, max=1.0),
        Param("contest_penalty", kind="float", default=CONTEST_PENALTY, min=0.0, max=1.0),
        Param("claim_penalty", kind="float", default=CLAIM_PENALTY, min=0.0, max=1.0),
    )

    def __init__(
        self,
        switch_margin: float = 0.25,
        min_commit: float = 2.0,
        lookahead_weight: float = 1.0,
        time_fit_slack: float = TIME_FIT_SLACK,
        reliability_weight: float = RELIABILITY_WEIGHT,
        contest_penalty: float = CONTEST_PENALTY,
        claim_penalty: float = CLAIM_PENALTY,
        replan_period: float = 0.5,
    ):
        self.switch_margin = switch_margin
        self.min_commit = min_commit
        self.lookahead_weight = lookahead_weight
        self.time_fit_slack = time_fit_slack
        self.reliability_weight = reliability_weight
        self.contest_penalty = contest_penalty
        self.claim_penalty = claim_penalty
        # Five times Score's and Collect's, and not a copy of their
        # cadence by oversight. Those replan a *target* -- which face,
        # which feeder -- and want to react to a defender arriving. This
        # replans a *job*, and a job that flips faster than `min_commit`
        # allows is arithmetic thrown away. It is also the expensive one:
        # a full arbitration prices every pickup on the field plus a
        # lookahead for each, ~5ms against the ~3ms Score's planner
        # spends, so running it at 0.1s would cost more than the rest of
        # the control stack put together.
        self.replan_period = replan_period

        self._score = Score()
        self._collect = Collect()
        self._active: Tactic | None = None
        self._commit_elapsed = 0.0
        self._reconsider = _Throttle(replan_period)

    @property
    def active_tactic(self) -> Tactic | None:
        """The child actually driving the robot, for
        `strategy._update_intent` to read the published intent off (see
        `strategy._delegate`). Everyone else's coordination runs through
        that intent -- station claim races, piece contention, a defender
        reading what its mark is doing -- so a Pursue robot that
        published only "Pursue" would be invisible to all of it."""
        return self._active

    def reset(self) -> None:
        # The children's own `reset()` deliberately preserves their
        # cooldown dicts ("this region would not take my piece a moment
        # ago" is knowledge about the field, not about one activation).
        # Nothing here may reach past that and clear them: doing so is
        # the exact trap Score.reset and _FAILED_STATION_COOLDOWN
        # document, where a cooldown is wiped every cycle and the whole
        # mechanism silently becomes a no-op.
        self._score.reset()
        self._collect.reset()
        self._active = None
        self._commit_elapsed = 0.0
        self._reconsider.reset()

    def _time_fit(self, prospects: "_Prospects", duration: float) -> float:
        """How much of a job's value survives the clock: all of it with
        `time_fit_slack` seconds to spare, none once the job no longer
        fits in what is left, a straight line between.

        This half is arithmetic rather than taste. A fetch is priced on
        the deposit it enables, and a deposit that happens after the
        buzzer scores nothing -- so without it a robot holding a piece
        it could put down right now still sets off across the field at
        t=147 for a cycle it cannot finish, and the fetch that "wins"
        the arbitration returns exactly zero points.

        The taste is only in how sharp the edge is. Zero slack makes it
        a cliff, which is wrong for the reason every other estimate here
        carries a margin: `duration` is an ETA over an obstacle-routed
        path, and a job worth everything on one tick and nothing on the
        next flips back and forth across the boundary."""
        remaining = prospects.remaining
        if remaining is None:
            return 1.0
        if self.time_fit_slack <= 0.0:
            return 1.0 if duration < remaining else 0.0
        return min(1.0, max(0.0, (remaining - duration) / self.time_fit_slack))

    def _reliability(self, probability: float) -> float:
        """The fraction of a deposit's points to expect, given how often
        it actually lands.

        Defaults to the full discount (weight 1.0), which makes this
        factor exactly `Outcome.expected_rate`'s -- and that is the
        point of the default rather than a measured gain. `Pursue`
        decides whether a job is worth doing; the planner underneath it
        decides which target to do it at, and ranks on expected points.
        At any weight below 1.0 the two disagree: this tactic prices
        scoring at L4's five points and its own Score child then goes
        and performs L3 for four, so the fetch-versus-score comparison
        is made in a currency nobody spends.

        Measured, that coherence is worth roughly nothing on the defense
        bench -- 5 points summed across seven rows, well inside a sum's
        noise -- and it is kept anyway, because two options that measure
        the same are not equally good if one of them is internally
        inconsistent. What the measurement *does* settle is the older
        result: at weight 1.0 against a planner still ranking on gross
        points this lost 48.9 summed, and nearly all of that was the
        mismatch rather than expected value itself.

        The weight survives as a weight because how far to trust a
        reliability estimate is a real question -- these numbers are
        illustrative, not fitted to event data -- and because a flat
        float Param is tunable by `strategy_params` for free."""
        return 1.0 - self.reliability_weight * (1.0 - probability)

    def _pressure(self, prospects: "_Prospects", *features) -> float:
        """What is left of a job's value after the robots already on the
        field features it needs.

        Job-level, and that is the whole reason it lives here rather
        than one layer down. Score already re-picks among regions that
        have room and Collect already races teammates for a feeder --
        but neither can conclude that *this kind of job* has stopped
        being worth doing. A robot whose every scoring face is being
        denied should go fetch; one whose supply is camped should put
        down what it is already holding. Only something holding both
        jobs at once can say that, and saying it is what this tactic is
        for.

        Compounds per robot rather than saturating, so a second defender
        on the same feature costs as much again as the first."""
        factor = 1.0
        for feature in features:
            deniers, crowd = prospects.contention(feature)
            factor *= (1.0 - self.contest_penalty) ** deniers
            factor *= (1.0 - self.claim_penalty) ** crowd
        return factor

    def _expected_rate(self, prospects, points, duration, probability, features) -> float:
        """Points per second of commitment, after everything the raw
        Outcome does not know: whether the job fits inside the match,
        how often the deposit lands, and who else is on the field
        features it needs.

        Every term is a multiplier on the points, never on the seconds.
        A contested region does not take longer to score in -- it scores
        less often -- and keeping the denominator as the honest ETA is
        what stops these weights from quietly re-pricing the commitment
        they are supposed to be judging."""
        value = points * self._reliability(probability) * self._time_fit(prospects, duration)
        return value * self._pressure(prospects, *features) / max(1e-6, duration)

    def _score_rate(self, prospects: "_Prospects", outcome) -> float:
        """A deposit's rate: `Outcome.value_rate` with the context terms
        applied."""
        return self._expected_rate(
            prospects, outcome.points, outcome.duration,
            outcome.success_probability, (outcome.payload,),
        )

    def _cycle_rate(self, prospects: "_Prospects", outcome) -> float:
        """Points per second for a pickup: the deposit it enables, over
        the whole trip out and back. Zero when there is nowhere legal to
        put the thing -- collecting it then genuinely buys nothing.

        Both ends are charged for contention: a camped feeder spoils the
        trip out, a denied REEF face spoils the trip back, and a fetch
        has to survive both to be worth making."""
        payoff = outcome.enables
        if payoff is None:
            return 0.0
        return self.lookahead_weight * self._expected_rate(
            prospects, payoff.points, outcome.duration + payoff.duration,
            payoff.success_probability, (outcome.payload, payoff.payload),
        )

    def _rank_jobs(self, ctx: BehaviorContext) -> _Ranking:
        """Price both jobs from where the robot stands right now.

        A rate of 0.0 means that job has nothing worth doing -- either no
        candidate at all, or only candidates worth nothing. The two are
        the same decision here, so they are not distinguished.

        Ties within a job go to the first candidate, which is the order
        `utility` emits them in (piece, then region, then action) -- the
        same tie-break `max` gave before, kept deliberately so the
        emission order stays part of the behavior rather than becoming
        accidental."""
        match, robot = ctx.match, ctx.robot
        travel = utility.TravelCache(match.field, robot.characteristics)
        prospects = _Prospects(match, robot)

        score_rate = max(
            (self._score_rate(prospects, outcome)
             for outcome in utility.score_outcomes(match, robot, travel=travel)),
            default=0.0,
        )

        collect_rate, collect_type = 0.0, None
        for outcome in utility.collect_outcomes(match, robot, travel=travel):
            rate = self._cycle_rate(prospects, outcome)
            if rate > collect_rate:
                collect_rate, collect_type = rate, getattr(outcome.payload, "piece_type", None)
        return _Ranking(score_rate, collect_rate, collect_type)

    def _arbitrate(self, ctx: BehaviorContext) -> None:
        ranking = self._rank_jobs(ctx)
        if ranking.score_rate <= 0.0 and ranking.collect_rate <= 0.0:
            self._select(ctx, None)
            return

        # Ties go to scoring: the points are in hand and the fetch side's
        # rate is a prediction of a trip not yet made.
        best = self._collect if ranking.collect_rate > ranking.score_rate else self._score

        if self._active is not None and self._active is not best:
            # A swap. It has to clear both gates or the incumbent keeps
            # the robot.
            if self._commit_elapsed < self.min_commit:
                return
            incumbent = ranking.score_rate if self._active is self._score else ranking.collect_rate
            challenger = ranking.collect_rate if best is self._collect else ranking.score_rate
            if challenger <= incumbent * (1.0 + self.switch_margin):
                return

        if best is self._collect:
            self._aim_collect_at(ctx, ranking.collect_type)
        self._select(ctx, best)

    def _aim_collect_at(self, ctx: BehaviorContext, piece_type: "str | None") -> None:
        """Point the Collect child at the type the arbitration priced.

        Only ever between trips. Two piece types whose rates sit close
        together would otherwise trade the argmax on every arbitration,
        and since a change of type invalidates the committed target,
        patience clock and approach that belong to the old one, each flip
        would restart the trip: the robot drives half a second toward one
        and half a second toward the other, and arrives at neither. It is
        the same ping-pong `_FAILED_TARGET_COOLDOWN` and
        `_RETARGET_MARGIN_*` exist to stop one level down.

        Nothing is lost by waiting. A trip that has stopped being worth
        making is already Collect's own business, through its patience
        clocks and cooldowns; abandoning fetching altogether is still the
        `switch_margin` decision above, which is free to fire mid-trip."""
        if piece_type is None or piece_type == self._collect.piece_type:
            return
        if self._active is self._collect and self._collect.has_target:
            return
        self._collect.piece_type = piece_type
        self._collect.reset()
        ctx.robot.set_intake_active(False)

    def _select(self, ctx: BehaviorContext, child: "Tactic | None") -> None:
        if child is self._active:
            return
        if self._active is not None:
            self._active.reset()
            # The same cleanup StrategyController does when it preempts a
            # tactic, and for the same reason: Collect commands the intake
            # on every tick it runs and only turns it off again in its own
            # SUCCESS/FAILURE branch. Swapping to Score mid-collect without
            # this leaves the intake latched on for the rest of the match,
            # and the robot hoovers up whatever compatible piece it drives
            # past.
            ctx.robot.set_intake_active(False)
            ctx.robot.set_deposit_active(False)
        self._active = child
        self._commit_elapsed = 0.0
        # A fresh window before the new job is second-guessed, so
        # `min_commit` measures time on the job rather than time since
        # the last time the throttle happened to fire.
        self._reconsider.reset()

    def tick(self, ctx: BehaviorContext) -> Status:
        self._commit_elapsed += ctx.dt
        if self._active is None or self._reconsider.ready(ctx.dt):
            self._arbitrate(ctx)
        if self._active is None:
            return self._stand_down(ctx, Status.FAILURE)

        status = self._active.tick(ctx)
        if status is Status.RUNNING:
            return Status.RUNNING

        # The job is over -- finished (Collect captured something, Score
        # emptied the robot) or impossible (Collect found nothing it can
        # get). Either way the arbitration that chose it is stale, so
        # re-open it now instead of waiting out the replan period:
        # `min_commit` exists to stop dithering between two workable jobs,
        # not to hold a robot on one that has just declared itself done.
        self._select(ctx, None)
        self._arbitrate(ctx)
        if self._active is None:
            return self._stand_down(ctx, Status.FAILURE)
        return Status.RUNNING

    def _stand_down(self, ctx: BehaviorContext, status: Status) -> Status:
        robot = ctx.robot
        robot.set_intake_active(False)
        robot.set_deposit_active(False)
        robot.drive_field_relative(ctx.dt, 0.0, 0.0, 0.0)
        return status


# How long a defender stays on the opponent it has marked before it will
# consider swapping, and how much better (in the `_threat` key's own ETA
# units) another opponent has to look to be worth swapping to.
#
# A defender picking the nearest opponent fresh every tick trades marks
# every time two of them cross, and ends up escorting the midpoint of the
# pair rather than denying either. Both robots then cycle almost freely
# while the defender looks busy. Committing to one mark is most of what
# makes a defender worth a robot at all.
_MARK_DWELL = 2.0
_MARK_SWAP_MARGIN = 1.5

# How close a mark may get to a zone that would protect it before a
# defender stops pressing and holds a no-contact standoff instead (see
# Defend._respect_protection). Roughly a robot length: the distance a
# leaned-on robot carries its leaner over the line before either can
# react.
_PROTECTION_RELEASE_MARGIN = 12.0

# What fraction of field_config.PinRule.max_seconds a defender lets run
# before it releases a mark it is holding still (see
# Defend._respect_pin_limit). Short of 1.0 because the retreat takes
# time, and a foul charged mid-retreat costs full price.
#
# Swept 0.5/0.7/0.85/0.95 against 1.0 (never release) on the 2v2 bench:
# the value does not matter, the release does. Every fraction in that
# range measured the same to within noise, while 1.0 differed sharply and
# in opposite directions per plan -- block/supply gave away 2.8 pins a
# match (a TECH FOUL each) for 12 points of extra denial, shadow/any gave
# away 0.5 for 13. So obeying the rule is a clear win where the geometry
# pins constantly and a clear loss where it barely does, and the sim
# cannot see the difference that decides it: it prices a single foul, not
# the card that follows a defender committing three a match.
_PIN_RELEASE_FRACTION = 0.7


class Defend(Tactic):
    """Denies an opponent the thing it is trying to do, positionally.
    Never terminates on its own. Denial is mostly about parking in the
    way, but contact is real: the drivetrain is traction-limited (see
    physics/swerve.py), so a defender square on an equally powered mark
    genuinely stops it, and the pin rule then limits how long it may
    (see `_respect_pin_limit`).

    Which opponent: the most threatening one within `engage_range` (see
    `_threat`), held for `_MARK_DWELL` so the mark doesn't change every
    time two opponents cross. With nobody in range, the defender falls
    back to lurking on the opponents' scoring end rather than standing
    where it happens to be -- an idle defender parked at our own wall
    denies nothing and is a long way from where it will next be needed.

    Which spot: `target` names a region or intake location to deny
    outright, or "opponent_intent" reads it from the mark's own published
    intent, or -- when the mark hasn't declared one -- guesses from what
    the mark is carrying (see `world_view.likely_denial_target`).
    Guessing beats waiting: a defender that only engages once its mark
    has committed to a region arrives after it does, every time.

    `deny` limits which half of the opponent's cycle is fair game:
    "scoring" for the regions it puts pieces into, "supply" for the
    stations it takes them from, "any" for whichever it is currently
    headed to. Which one is worth taking away is a property of the game,
    not a preference: a game that protects its scoring locations from
    contact (see field_config.ProtectedZone) but leaves its feeders open
    makes supply the only place a defender is allowed to do anything at
    all.

    "any" is the default because following the mark is the answer that
    needs to know nothing about the game. Measured 2v2 against a
    full-time defender, blue's points (which include fouls won) and red's
    protection fouls per match, against 234.5 undefended:

        block   scoring 234.8 / 15.5   supply 209.5 / 0.8   any 217.9 / 11.9
        shadow  scoring 191.5 /  8.1   supply 232.0 / 9.9   any 186.9 /  7.6

    `deny` and `mode` are not independent, which is why this is a
    per-tactic param rather than a defender-wide switch. Block denies
    supply well and scoring not at all -- goalkeeping a REEF face means
    standing where contact is forbidden, so it fouls away everything it
    denies. Shadow is the reverse: it positions relative to the *mark*,
    so pointing it at a station the mark is not currently driving to
    just parks it on the wrong side of the robot.

    `mode` picks what to do with that spot. "block" takes the segment
    between mark and region and sits `standoff` back from the region --
    goalkeeping, best when the region is the scarce thing. "shadow" sits
    `standoff` off the *mark* on the side facing the region -- man
    coverage, which stays with an opponent that hasn't committed yet and
    denies whatever it eventually picks."""

    PARAM_SCHEMA = (
        Param("target", kind="str", default="opponent_intent"),
        Param("mode", kind="choice", choices=("block", "shadow"), default="block"),
        Param("deny", kind="choice", choices=("scoring", "supply", "any"), default="any"),
        Param("standoff", kind="float", default=24.0, min=0, suffix=" in"),
        Param("engage_range", kind="float", default=200.0, min=0, suffix=" in"),
    )

    def __init__(
        self,
        target: str = "opponent_intent",
        mode: Literal["block", "shadow"] = "block",
        deny: Literal["scoring", "supply", "any"] = "any",
        standoff: float = 24.0,
        engage_range: float = 200.0,
        replan_period: float = 0.1,
    ):
        self.target = target
        self.mode = mode
        self.deny = deny
        self.standoff = standoff
        self.engage_range = engage_range
        self.replan_period = replan_period
        self.target_region_name: str | None = None
        # Published on the robot's Intent so the robot being denied can
        # tell it's the one being denied -- see world_view.defenders_against.
        self.marked_robot = None
        self._mark_elapsed = 0.0
        self._repick = _Throttle(replan_period)
        self._nav = NavigateTo(
            self._provide_target, heading_mode="face_target", replan_period=replan_period, avoid_robots=False
        )

    def reset(self) -> None:
        self.target_region_name = None
        self.marked_robot = None
        self._mark_elapsed = 0.0
        self._repick.reset()
        self._nav.reset()

    @property
    def _kinds(self) -> tuple[str, ...]:
        return ("scoring", "supply") if self.deny == "any" else (self.deny,)

    def _region_for(self, ctx: BehaviorContext, opponent):
        """The field feature this defender is denying `opponent` -- a
        scoring region or an intake location, both of which are just a
        `.name` and a `.vertices` to everything downstream. An explicit
        `target` name wins; otherwise the opponent's own declaration,
        falling back to a guess at where it will go."""
        if self.target != "opponent_intent":
            return world_view.denial_target_by_name(ctx.match, self.target)
        return world_view.likely_denial_target(ctx.match, opponent, self._kinds)

    def _threat(self, ctx: BehaviorContext, opponent) -> tuple[int, int, float]:
        """How much `opponent` is worth marking, smallest first: one
        already holding a piece can score the moment it's left alone and
        outranks one still hunting for something to carry, and among
        equals the one we can actually get to soonest wins. Obstacle-
        routed, like every other ETA a tactic compares, so a mark on the
        far side of the REEF isn't mistaken for the reachable one.

        An opponent already inside a protected zone sorts behind every
        opponent that isn't, whatever it's carrying. There is nothing
        left to deny it -- it has arrived somewhere we may not touch it
        -- so marking it spends the one defender we have on the one
        opponent we can no longer affect."""
        eta = estimate_travel_time(
            ctx.match.field, (ctx.robot.pose.x, ctx.robot.pose.y),
            (opponent.pose.x, opponent.pose.y), ctx.robot.characteristics,
        )
        protected = 1 if world_view.is_protected(ctx.match, opponent) else 0
        return (protected, 0 if opponent.held_pieces else 1, eta)

    def _in_range(self, ctx: BehaviorContext, opponent) -> bool:
        origin = ctx.robot.pose.translation
        return origin.get_distance(opponent.pose.translation) <= self.engage_range

    def _pick_opponent(self, ctx: BehaviorContext):
        """The opponent to mark, sticky for `_MARK_DWELL` and only handed
        over to a rival that is better by `_MARK_SWAP_MARGIN`."""
        candidates = [
            o for o in world_view.opponents(ctx.match, ctx.robot.alliance)
            if self._in_range(ctx, o) and self._region_for(ctx, o) is not None
        ]
        if not candidates:
            return None

        best = min(candidates, key=lambda o: self._threat(ctx, o))
        incumbent = self.marked_robot
        if incumbent is None or incumbent not in candidates:
            return best
        if self._mark_elapsed < _MARK_DWELL:
            return incumbent

        best_key, incumbent_key = self._threat(ctx, best), self._threat(ctx, incumbent)
        # Any tier improvement (the mark became untouchable, or a rival
        # picked a piece up) hands the mark over immediately; a merely
        # closer opponent has to beat the incumbent by _MARK_SWAP_MARGIN.
        if best_key[:-1] < incumbent_key[:-1] or best_key[-1] + _MARK_SWAP_MARGIN < incumbent_key[-1]:
            return best
        return incumbent

    def _lurk_pose(self, ctx: BehaviorContext) -> Pose2d:
        """Where to wait with no opponent in range: the nearest of the
        features this defender is willing to deny. That is where the
        opponents have to come back to, so waiting there is both the
        shortest trip to the next engagement and, in itself, a spot
        they'd rather we weren't. Filtered by `deny` for the same reason
        the mark's target is -- a defender that only ever attacks the
        feeder should be idling at the feeder, not at a REEF it will
        never engage anyone at."""
        robot = ctx.robot
        opposing = next(
            (o.alliance for o in world_view.opponents(ctx.match, robot.alliance)), None,
        )
        regions = []
        if opposing:
            if "scoring" in self._kinds:
                regions += world_view.alliance_scoring_regions(ctx.match, opposing)
            if "supply" in self._kinds:
                regions += world_view.alliance_intake_locations(ctx.match, opposing)
        if not regions:
            return robot.pose
        origin = (robot.pose.x, robot.pose.y)
        region = min(
            regions,
            key=lambda r: math.hypot(
                world_view.region_centroid(r)[0] - origin[0], world_view.region_centroid(r)[1] - origin[1]
            ),
        )
        self.target_region_name = region.name
        cx, cy = world_view.region_centroid(region)
        return Pose2d(cx, cy, math.atan2(cy - origin[1], cx - origin[0]))

    def _update_mark(self, ctx: BehaviorContext) -> None:
        """Re-run the (obstacle-routed, so not free) mark selection and
        restart the dwell clock if it came back with someone new. Driven
        from `tick` on the replan cadence rather than from
        `_provide_target`, which runs every physics tick."""
        opponent = self._pick_opponent(ctx)
        if opponent is not self.marked_robot:
            self._mark_elapsed = 0.0
            self.marked_robot = opponent

    def _provide_target(self, ctx: BehaviorContext) -> Pose2d:
        robot = ctx.robot
        opponent = self.marked_robot
        if opponent is None:
            self.target_region_name = None
            return self._lurk_pose(ctx)

        region = self._region_for(ctx, opponent)
        if region is None:
            self.target_region_name = None
            return self._lurk_pose(ctx)
        self.target_region_name = region.name

        rx, ry = world_view.region_centroid(region)
        ox, oy = opponent.pose.x, opponent.pose.y
        dx, dy = rx - ox, ry - oy  # opponent -> region
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            block_x, block_y = rx, ry
        elif self.mode == "shadow":
            # `standoff` out from the mark toward the region: stays with
            # an opponent wherever it goes, and is already on the correct
            # side of it when it finally commits.
            t = min(1.0, self.standoff / dist)
            block_x, block_y = ox + dx * t, oy + dy * t
        else:
            t = min(1.0, self.standoff / dist)  # fraction of the segment back from the region
            block_x, block_y = rx - dx * t, ry - dy * t

        block_x, block_y = self._respect_protection(ctx, opponent, block_x, block_y)
        block_x, block_y = self._respect_pin_limit(ctx, opponent, block_x, block_y)
        heading = math.atan2(oy - block_y, ox - block_x)
        return Pose2d(block_x, block_y, heading)

    def _respect_pin_limit(self, ctx: BehaviorContext, opponent, x: float, y: float) -> tuple[float, float]:
        """Let a mark go before the pin clock runs out (see
        field_config.PinRule). Same retreat as `_respect_protection` --
        out along the closing line to a no-contact standoff -- for the
        same reason: the defender keeps the side and the angle it earned,
        and is still the nearest robot to the mark when it re-engages.

        Released at `_PIN_RELEASE_FRACTION` of the limit rather than at
        it, because backing off is not instantaneous: the defender has to
        cover the standoff distance, and a foul charged while it was on
        its way out costs exactly as much as one charged standing still.

        The clock is only running at all when the mark is *held* --
        pressing a robot that keeps driving where it wants accumulates
        nothing, and neither does parking across a place it wanted to
        reach. So this releases precisely the defense that was working,
        which is the rule's whole intent."""
        if world_view.pin_pressure(ctx.match, ctx.robot, opponent) < _PIN_RELEASE_FRACTION:
            return (x, y)
        return self._back_off_from(ctx, opponent, x, y)

    def _respect_protection(self, ctx: BehaviorContext, opponent, x: float, y: float) -> tuple[float, float]:
        """Back the block point off a mark standing in a zone where we
        may not touch it (see field_config.ProtectedZone), pushing
        straight out along the line we were closing on so the defender
        still shows the same face from the same side -- it holds the
        approach, one robot-length further out, instead of pressing into
        a contact that would hand the opponent points.

        Released a robot-length early (`_PROTECTION_RELEASE_MARGIN`)
        rather than at the boundary, because a defender leaning on a mark
        is carried across the line by it: measured on the 2v2 bench,
        11 of 13 fouls in a match were contact that predated the
        victim's protection by at least a tick. Waiting for the mark to
        actually be inside means always being late, and late is the foul.

        Backing off is not the same as giving up -- protection is
        positional, so the mark loses it the moment it leaves, and the
        defender is waiting right there when it does."""
        if world_view.protection_distance(ctx.match, opponent) > _PROTECTION_RELEASE_MARGIN:
            return (x, y)
        return self._back_off_from(ctx, opponent, x, y)

    def _back_off_from(self, ctx: BehaviorContext, opponent, x: float, y: float) -> tuple[float, float]:
        """Push a block point out to a distance where the two chassis
        cannot touch at any relative heading, along the line it already
        sat on. Shared by the two rules that require a defender to stop
        making contact without giving up the mark."""
        keepout = world_view.protection_keepout(ctx.robot, opponent)
        dx, dy = x - opponent.pose.x, y - opponent.pose.y
        distance = math.hypot(dx, dy)
        if distance >= keepout:
            return (x, y)
        if distance < 1e-6:
            # Standing on top of the mark: no line to back off along, so
            # retreat toward where we came from.
            dx, dy = ctx.robot.pose.x - opponent.pose.x, ctx.robot.pose.y - opponent.pose.y
            distance = math.hypot(dx, dy)
            if distance < 1e-6:
                return (x, y)
        scale = keepout / distance
        return (opponent.pose.x + dx * scale, opponent.pose.y + dy * scale)

    def tick(self, ctx: BehaviorContext) -> Status:
        self._mark_elapsed += ctx.dt
        if self.marked_robot is None or self._repick.ready(ctx.dt):
            self._update_mark(ctx)
        self._nav.tick(ctx)
        return Status.RUNNING
