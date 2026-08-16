"""
High-level tactics: Behaviors that decide their own target each tick
instead of being handed a Pose2d. Each owns a target + a NavigateTo +
primitive children internally, re-evaluated on `replan_period`.

Each tactic exposes `PARAM_SCHEMA` (see strategy_editor's Param) so a
GUI can build a property inspector for it with zero per-tactic GUI code.
"""
from __future__ import annotations

import math
from typing import Literal

from common_sim.control import world_view
from common_sim.control.behavior import Behavior, BehaviorContext, Status
from common_sim.control.navigation import NavigateTo, clear_standoff
from common_sim.control.param import Param
from common_sim.control.planning import GreedyRatePlanner, ScorePlanner, build_option
from common_sim.field.field_config import point_in_polygon, polygon_centroid
from common_sim.geometry import Pose2d, wrap_angle
from common_sim.robot.characteristics import SIDE_OUTWARD


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
        Param("prefer_station", kind="bool", default=False),
        Param("max_range", kind="float", default=None, optional=True, min=0, suffix=" in"),
    )

    def __init__(
        self,
        piece_type: str | None = None,
        mode: Literal["nearest", "densest"] = "nearest",
        cluster_radius: float = 24.0,
        prefer_station: bool = False,
        max_range: float | None = None,
        replan_period: float = 0.1,
    ):
        self.piece_type = piece_type
        self.mode = mode
        self.cluster_radius = cluster_radius
        self.prefer_station = prefer_station
        self.max_range = max_range
        self.replan_period = replan_period

        self._target_piece = None
        self._target_station = None
        self._start_held_count = None
        self._nav = NavigateTo(self._provide_target, heading_mode="face_target", replan_period=replan_period)

    def reset(self) -> None:
        self._target_piece = None
        self._target_station = None
        self._start_held_count = None
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
        if world_view.region_has_room(ctx.match, station, robot) or self._holds_station(robot):
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
        two would trade places forever without either collecting.

        Engagement (`nearby_station`) is the same test the sim uses to
        decide whether to dispense, and it is checked before polygon
        containment because a robot parked at its intake standoff can sit
        a couple of inches outside a station's zone while being served
        by it."""
        station = self._target_station
        if robot.nearby_station() is station:
            return True
        return point_in_polygon((robot.pose.x, robot.pose.y), station.vertices)

    def _better_station_exists(self, ctx: BehaviorContext) -> bool:
        """Whether the committed station is full and some other one this
        robot could use isn't."""
        match, robot = ctx.match, ctx.robot
        if self._holds_station(robot) or world_view.region_has_room(match, self._target_station, robot):
            return False
        return any(
            station is not self._target_station
            and (self.piece_type is None or station.piece_type == self.piece_type)
            and world_view.region_has_room(match, station, robot)
            for station in world_view.station_options(match, robot)
        )

    def _pick_target(self, ctx: BehaviorContext) -> bool:
        match, robot = ctx.match, ctx.robot
        origin = robot.pose.translation

        if self.prefer_station:
            stations = world_view.station_options(match, robot)
            if self.piece_type is not None:
                stations = [s for s in stations if s.piece_type == self.piece_type]
            if stations:
                # Same rule Score._pick_option applies to scoring regions:
                # take one nobody is working before contesting one. Two
                # robots both picking "nearest" otherwise converge on the
                # same corner, and a REEFSCAPE CORAL STATION is 36x36
                # against a 28x28 robot -- capacity 1, no room to share.
                # Falling back to the crowded nearest is fine because
                # _station_aim then queues outside it rather than barging
                # in; with every station busy, waiting for the nearest is
                # exactly the right thing to do.
                roomy = [s for s in stations if world_view.region_has_room(match, s, robot)]
                self._target_station, self._target_piece = min(
                    roomy or stations, key=lambda s: origin.get_distance(polygon_centroid(s.vertices))
                ), None
                return True

        pieces = world_view.collectable_pieces(match, piece_type=self.piece_type, robot=robot)
        if self.max_range is not None:
            pieces = [p for p in pieces if origin.get_distance(p.position) <= self.max_range]
        if not pieces:
            self._target_piece = None
            self._target_station = None
            return False

        # Prefer a piece no teammate is already heading for -- two robots
        # independently picking "nearest" converge on the same piece when
        # it's the obvious choice for both, driving them nose-to-nose at
        # its position instead of splitting up. Only when every remaining
        # piece is already claimed do we fall back to contesting one
        # (better to double up than to sit idle).
        claimed = {
            partner.intent.target_piece
            for partner in world_view.partners(match, robot.alliance)
            if partner is not robot and partner.intent is not None
        }
        unclaimed = [p for p in pieces if p not in claimed]
        if unclaimed:
            pieces = unclaimed

        if self.mode == "densest":
            clusters = world_view.piece_clusters(match, pieces, self.cluster_radius)
            best = max(clusters, key=lambda c: c.count)
            target = min(best.pieces, key=lambda p: origin.get_distance(p.position))
        else:
            target = min(pieces, key=lambda p: origin.get_distance(p.position))

        self._target_piece = target
        self._target_station = None
        return True

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

        target_lost = self._target_piece is not None and (
            self._target_piece.held_by is not None or self._target_piece.scored
        )
        # A station is committed to once and then queued for, so nothing
        # would ever pull a robot off one an opponent has parked in --
        # except a free station elsewhere, which is strictly better than
        # waiting. Guarded on another station actually having room so
        # this can't cycle between two equally crowded ones.
        if self._target_station is not None and self._better_station_exists(ctx):
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
