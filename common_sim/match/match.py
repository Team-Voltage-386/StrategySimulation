"""
Match orchestrator. Owns the SimEngine, builds field geometry from a
FieldConfig, wires the collision handlers that connect intake sensors
and scoring regions to Robot/piece state, and advances match phase
(auto -> teleop) on a clock. This is the one place in common_sim that
ties physics + field + robots + scoring together; everything it calls
into (Robot, ScoringRules, FieldConfig) stays game-agnostic on its own.
"""
from __future__ import annotations

import random
from enum import Enum

import pymunk

from common_sim.control.behavior import BehaviorContext
from common_sim.field.field_config import (
    FieldConfig, ProtectedZone, point_in_polygon, polygon_centroid, polygons_intersect,
)
from common_sim.field.game_piece import GamePiece, piece_spec
from common_sim.match.events import EventLog
from common_sim.match.scoring import ScoringRules
from common_sim.physics.engine import DEFAULT_SUBSTEP, SimEngine
from common_sim.robot.characteristics import RobotCharacteristics
from common_sim.robot.robot import Robot
from common_sim.geometry import Pose2d


# How close two bumpers count as touching (in). Not zero: the solver
# leaves a small gap between resting bodies, and a rule about contact
# should not hinge on a sub-millimeter numerical artifact.
_CONTACT_TOLERANCE = 0.25


class Phase(Enum):
    AUTO = "auto"
    TELEOP = "teleop"


class MatchConfig:
    def __init__(
        self,
        auto_duration: float = 15.0,
        teleop_duration: float = 135.0,
        disable_friendly_collisions: bool = False,
        emit_coral_to_field: bool = False,
    ):
        self.auto_duration = auto_duration
        self.teleop_duration = teleop_duration
        # When True, robots on the same alliance pass through each other
        # (pymunk ShapeFilter group per alliance) while collisions against
        # opposing-alliance robots are unaffected.
        self.disable_friendly_collisions = disable_friendly_collisions
        # Master on/off switch for Match._step_emitters -- False (default)
        # means FieldConfig.emitter_regions are declared but never actually
        # spawn/consume/return pieces, so a game can describe its emitters
        # once and let this toggle gate them at match-setup time instead of
        # needing two different field configs. Named for REEFSCAPE's one
        # current use (a coral emitter per alliance zone); a future game
        # with a differently-themed emitter would still reuse this same
        # flag rather than get its own, unless that ever needs to be
        # independently toggleable.
        self.emit_coral_to_field = emit_coral_to_field

    @property
    def total_duration(self) -> float:
        return self.auto_duration + self.teleop_duration


# Collision-type IDs used internally by Match. Game/GUI code should not
# need to know these -- Robot/GamePiece are constructed by Match, which
# assigns them consistently.
_CHASSIS_TYPE = 1
_INTAKE_TYPE = 2
_PIECE_TYPE = 3
_SCORING_TYPE = 4
_STATION_TYPE = 5

# pymunk ShapeFilter groups used for MatchConfig.disable_friendly_collisions
# -- shapes sharing a nonzero group never collide with each other, but a
# robot's group still differs from the opposing alliance's, so cross-alliance
# contact is untouched. Extend this if a game ever has more than two alliances.
_ALLIANCE_COLLISION_GROUPS = {"red": 1, "blue": 2}


class Match:
    def __init__(
        self,
        field_config: FieldConfig,
        scoring_rules: ScoringRules,
        match_config: MatchConfig | None = None,
        substep: float = DEFAULT_SUBSTEP,
        rng: random.Random | None = None,
    ):
        self.field = field_config
        self.scoring_rules = scoring_rules
        self.config = match_config or MatchConfig()
        self.engine = SimEngine(substep=substep)
        # Source of randomness for scoring-reliability rolls (see
        # RobotCharacteristics.scoring_reliability_by_type). A caller that
        # needs reproducible rolls (the Monte Carlo sweep) passes a seeded
        # substream; unseeded (the default) is fine for interactive play,
        # and a robot with no reliability configured never draws from this
        # at all, so most callers can ignore it entirely.
        self._rng = rng if rng is not None else random.Random()

        self.robots: list[Robot] = []
        self.active_pieces: list[GamePiece] = []
        self.scores: dict[str, float] = {}
        # region name -> action -> count of pieces scored there, e.g. for a
        # GUI to show per-location/per-level piece counts.
        self.region_scores: dict[str, dict[str, int]] = {}
        # alliance -> how many protected-zone contact violations its
        # robots committed (see _step_protection). Points from those
        # violations land in `scores` under the *other* alliance.
        self.protection_fouls: dict[str, int] = {}
        # (offender id, protected id) -> seconds left before a contact
        # that never breaks counts as a second foul.
        self._foul_cooldown: dict[tuple[int, int], float] = {}
        self.events = EventLog()

        self.elapsed = 0.0
        self.phase = Phase.AUTO
        self.ended = False

        # Remaining dispensable pieces per IntakeLocation, for locations with
        # a finite starting_pieces count -- a location with starting_pieces
        # None (unlimited) never appears here, so a .get(location) miss
        # elsewhere in Match always means "unlimited supply".
        self.station_supply: dict = {
            location: location.starting_pieces
            for location in field_config.intake_locations
            if location.starting_pieces is not None
        }

        intake_locations_by_name = {location.name: location for location in field_config.intake_locations}
        # Resolved IntakeLocation for each emitter that shares a station's
        # pool (EmitterRegion.linked_collection_region), None for an
        # emitter with its own independent capacity.
        self._emitter_linked_stations: dict = {
            emitter: intake_locations_by_name[emitter.linked_collection_region]
            for emitter in field_config.emitter_regions
            if emitter.linked_collection_region is not None
        }
        # Remaining emit count for emitters with their own finite capacity
        # (initial_capacity set, not station-linked). An emitter missing
        # here either shares a station's self.station_supply count instead,
        # or has unlimited capacity.
        self.emitter_supply: dict = {
            emitter: emitter.initial_capacity
            for emitter in field_config.emitter_regions
            if emitter.linked_collection_region is None and emitter.initial_capacity is not None
        }
        # Seconds until each emitter's next emit, counted down in
        # _step_emitters; only while the emitter is within an active_times
        # window (elsewhere the timer just holds).
        self._emitter_cooldowns: dict = {
            emitter: 1.0 / emitter.emit_rate_hz for emitter in field_config.emitter_regions
        }
        # (ready_time, emitter) pairs for pieces scored in a
        # linked_scoring_region, waiting out their return_delay before
        # going back into the emitter's pool -- see _try_score / _step_emitters.
        self._pending_emitter_returns: list = []
        self._emitters_by_scoring_region: dict = {}
        for emitter in field_config.emitter_regions:
            if emitter.linked_scoring_region is not None:
                self._emitters_by_scoring_region.setdefault(emitter.linked_scoring_region, []).append(emitter)

        self._build_obstacles()
        self._register_collision_handlers()

    # -- setup ---------------------------------------------------------

    def _build_obstacles(self) -> None:
        self._build_perimeter_walls()

        for obstacle in self.field.obstacles:
            shape = pymunk.Poly(self.engine.space.static_body, list(obstacle.vertices))
            shape.elasticity = 0.3
            shape.friction = 0.5
            self.engine.space.add(shape)

        for region in self.field.scoring_regions:
            shape = pymunk.Poly(self.engine.space.static_body, list(region.vertices))
            shape.sensor = True
            shape.collision_type = _SCORING_TYPE
            shape.scoring_region = region
            self.engine.space.add(shape)

        for location in self.field.intake_locations:
            shape = pymunk.Poly(self.engine.space.static_body, list(location.vertices))
            shape.sensor = True
            shape.collision_type = _STATION_TYPE
            shape.intake_location = location
            self.engine.space.add(shape)

    def _build_perimeter_walls(self) -> None:
        """Every field has an outer boundary robots and pieces can't cross,
        independent of whatever game-specific obstacles it also declares."""
        w, h = self.field.width, self.field.height
        corners = [(0, 0), (w, 0), (w, h), (0, h)]
        body = self.engine.space.static_body
        for i in range(4):
            a, b = corners[i], corners[(i + 1) % 4]
            wall = pymunk.Segment(body, a, b, 1.0)
            wall.elasticity = 0.3
            wall.friction = 0.5
            self.engine.space.add(wall)

    def _register_collision_handlers(self) -> None:
        space = self.engine.space

        def intake_begin(arbiter, space, data):
            intake_shape, piece_shape = _order_by_type(arbiter.shapes, _INTAKE_TYPE)
            intake_shape.owner_robot.piece_entered_intake_range(piece_shape.game_piece, intake_shape.intake_side)
            return True

        def intake_separate(arbiter, space, data):
            intake_shape, piece_shape = _order_by_type(arbiter.shapes, _INTAKE_TYPE)
            intake_shape.owner_robot.piece_left_intake_range(piece_shape.game_piece, intake_shape.intake_side)

        space.on_collision(_INTAKE_TYPE, _PIECE_TYPE, begin=intake_begin, separate=intake_separate)

        def station_begin(arbiter, space, data):
            intake_shape, station_shape = _order_by_type(arbiter.shapes, _INTAKE_TYPE)
            intake_shape.owner_robot.station_entered(station_shape.intake_location, intake_shape.intake_side)
            return True

        def station_separate(arbiter, space, data):
            intake_shape, station_shape = _order_by_type(arbiter.shapes, _INTAKE_TYPE)
            intake_shape.owner_robot.station_left(station_shape.intake_location, intake_shape.intake_side)

        space.on_collision(_INTAKE_TYPE, _STATION_TYPE, begin=station_begin, separate=station_separate)

        def scoring_begin(arbiter, space, data):
            # A physics contact, not an explicit robot deposit -- only
            # scores a region that allows passive (roll/fly-in) scoring;
            # see ScoringRegion.passive_scoring.
            region_shape, piece_shape = _order_by_type(arbiter.shapes, _SCORING_TYPE)
            self._try_score(piece_shape.game_piece, region_shape.scoring_region, explicit=False)
            return False  # sensor: never a physical response

        space.on_collision(_SCORING_TYPE, _PIECE_TYPE, begin=scoring_begin)

    # -- entity creation -------------------------------------------------

    def add_robot(
        self, characteristics: RobotCharacteristics, start_pose: Pose2d, alliance: str = "blue", controller=None,
    ) -> Robot:
        """`controller`, if given, is assigned to `robot.controller` --
        typically not passed here (a StrategyController needs the live
        Robot instance to construct, which doesn't exist until this
        returns), so the usual pattern is:

            robot = match.add_robot(characteristics, pose)
            robot.controller = StrategyController(strategy, robot)

        The parameter exists for a controller built some other way that
        doesn't need `robot` up front (e.g. a duck-typed test double)."""
        chassis_shape_filter = None
        if self.config.disable_friendly_collisions:
            group = _ALLIANCE_COLLISION_GROUPS.get(alliance)
            if group is not None:
                chassis_shape_filter = pymunk.ShapeFilter(group=group)
        robot = Robot(
            self.engine.space,
            characteristics,
            start_pose,
            alliance=alliance,
            chassis_collision_type=_CHASSIS_TYPE,
            intake_collision_type=_INTAKE_TYPE,
            chassis_shape_filter=chassis_shape_filter,
        )
        if controller is not None:
            robot.controller = controller
        preload_type = characteristics.preload_piece_type or _default_piece_type(characteristics)
        for _ in range(characteristics.starting_piece_count):
            piece = self.spawn_piece(preload_type, start_pose.as_tuple()[:2], source="preload")
            piece.held_by = robot
            piece.shape.sensor = True
            piece.last_holder_alliance = alliance
            robot.held_pieces.append(piece)
        self.robots.append(robot)
        return robot

    def spawn_piece(self, piece_type: str, position: tuple[float, float], source: str = "field", **kwargs) -> GamePiece:
        """radius/mass/color default to piece_type's registered
        GamePieceSpec (see common_sim.field.game_piece) -- pass any of
        them explicitly to override for this one piece."""
        spec = piece_spec(piece_type)
        kwargs.setdefault("radius", spec.radius)
        kwargs.setdefault("mass", spec.mass)
        kwargs.setdefault("color", spec.color)
        piece = GamePiece(self.engine.space, piece_type, position, collision_type=_PIECE_TYPE, source=source, **kwargs)
        self.active_pieces.append(piece)
        return piece

    # -- scoring ---------------------------------------------------------

    def _check_region_scoring(self, piece: GamePiece) -> None:
        """Explicit point-in-polygon check against every scoring region,
        used right after a robot releases a piece that wasn't released
        from within an engaged deposit (Match.step's `ready_region` was
        None). Since the robot wasn't confirmed in position, this can
        only score a region that allows passive scoring -- see
        ScoringRegion.passive_scoring -- the same as a later physics
        contact would (scoring_begin below); a non-passive region always
        needs the explicit, robot-engaged path in Match.step."""
        if piece.held_by is not None or piece.scored:
            return
        for region in self.field.scoring_regions:
            if point_in_polygon((piece.position.x, piece.position.y), region.vertices):
                self._try_score(piece, region, explicit=False)
                if piece.scored:
                    return

    def region_full(self, region, action: str) -> bool:
        """Whether `region` has already accepted as many pieces for
        `action` as `region.capacity_by_action` allows -- None (default,
        or the action missing from the mapping) means unlimited."""
        if not region.capacity_by_action:
            return False
        cap = region.capacity_by_action.get(action)
        if cap is None:
            return False
        scored_count = self.region_scores.get(region.name, {}).get(action, 0)
        return scored_count >= cap

    def _roll_scoring_success(self, robot: Robot, piece: GamePiece) -> bool:
        """Whether a deliberate scoring attempt by `robot` lands, per its
        RobotCharacteristics.scoring_reliability_for(piece.piece_type).
        Only ever consulted for the explicit deposit-into-a-ready-region
        path in step() -- a piece that merely drifts/bounces into a region
        later (passive scoring) isn't a robot "attempt" and always keeps
        the old deterministic behavior. Short-circuits at reliability 1.0
        (the default for any unconfigured type) so the common case never
        draws from self._rng, keeping existing sims' RNG draw sequence
        untouched."""
        reliability = robot.characteristics.reliability_for(piece.piece_type)
        if reliability >= 1.0:
            return True
        return self._rng.random() < reliability

    def _try_score(self, piece: GamePiece, region, *, explicit: bool) -> None:
        """`explicit` says whether the robot was actually confirmed in
        position performing the scoring action right now (Match.step's
        `ready_region` path) as opposed to a piece merely touching or
        sitting inside the region's sensor with no robot present (a
        physics contact, or a release that landed in the zone without the
        robot being properly engaged with it). A region with
        passive_scoring=False (most REEFSCAPE scoring locations) can only
        ever be scored via the explicit path -- it must actually require
        a robot to be there doing it, matching how a real REEF/PROCESSOR
        can't be scored by a piece that merely rolls or bounces in."""
        if piece.held_by is not None or piece.scored:
            return
        if not explicit and not region.passive_scoring:
            return
        if region.piece_types and piece.piece_type not in region.piece_types:
            return
        action = piece.target_action
        if action is None or action not in region.actions:
            return  # e.g. a piece deposited/launched without a matching target -- a miss, not a crash
        if self.region_full(region, action):
            return  # e.g. a REEF branch that already holds a CORAL -- a miss, not a crash
        points = self.scoring_rules.points_for(action, self.phase.value)
        alliance = piece.last_holder_alliance or "unknown"
        self.scores[alliance] = self.scores.get(alliance, 0.0) + points
        region_counts = self.region_scores.setdefault(region.name, {})
        region_counts[action] = region_counts.get(action, 0) + 1
        piece.scored = True
        self.events.log(self.elapsed, "score", {
            "alliance": alliance, "action": action, "points": points, "piece_type": piece.piece_type,
        })

        if self.config.emit_coral_to_field:
            for emitter in self._emitters_by_scoring_region.get(region.name, ()):
                if emitter.piece_type != piece.piece_type:
                    continue
                ready_time = self.elapsed + (emitter.return_delay or 0.0)
                self._pending_emitter_returns.append((ready_time, emitter))

    def deposit_piece_for(self, robot: Robot) -> GamePiece | None:
        """Which of `robot`'s held pieces its currently-commanded deposit
        action targets, or None if it isn't holding anything. Needed once
        a robot can hold more than one piece type at once (e.g. coral +
        algae) -- the action string alone ("l4", "processor") doesn't say
        which type it applies to, so this disambiguates via which scoring
        regions declare that action for which piece_types. Falls back to
        the FIFO-first held piece when the action doesn't uniquely resolve
        one (a single-type robot, or an action with no matching region),
        matching the pre-multi-type behavior."""
        if not robot.held_pieces:
            return None
        action = robot.deposit_action
        if action is None:
            return robot.held_pieces[0]
        eligible_types = {
            piece_type
            for region in self.field.scoring_regions
            if action in region.actions
            for piece_type in (region.piece_types or {p.piece_type for p in robot.held_pieces})
        }
        for piece in robot.held_pieces:
            if piece.piece_type in eligible_types:
                return piece
        return robot.held_pieces[0]

    def deposit_region_for(self, robot: Robot, piece: GamePiece | None = None):
        """The scoring region `robot`'s currently-commanded deposit would
        score in if it completed right now, or None. `piece` -- which
        held piece the deposit applies to -- defaults to
        `deposit_piece_for(robot)` if not given. Public because it is
        the single source of truth for "is this robot in position to
        score": Match.step uses it right after a deposit completes to
        decide whether to award points for the released piece, and a GUI
        reads it to decide whether to show the robot as ready -- so an
        indicator can never disagree with whether scoring will actually
        happen. NOT a gate on whether a deposit can be *commanded* --
        Robot.update_manipulator's timer runs regardless of region, so a
        piece can be dropped anywhere; this only governs whether that
        drop counts as a score.

        Evaluated fresh each call against the live pose via
        Robot.side_reach_points -- that memoizes per side/pose but never
        returns a stale value from an earlier pose."""
        if piece is None:
            piece = self.deposit_piece_for(robot)
        if piece is None:
            return None
        action = robot.deposit_action
        if action is None:
            return None
        side = robot.scoring_side(piece.piece_type)
        if side is None:
            return None
        piece_type = piece.piece_type
        for region in self.field.scoring_regions:
            if region.alliance is not None and region.alliance != robot.alliance:
                continue
            if region.piece_types and piece_type not in region.piece_types:
                continue
            if action not in region.actions:
                continue
            if robot.side_engages_polygon(side, region.vertices):
                return region
        return None

    # -- protected zones ---------------------------------------------------

    def protecting_zone(self, robot: Robot) -> ProtectedZone | None:
        """The ProtectedZone currently shielding `robot` from opponent
        contact, or None. Any overlap counts -- "BUMPERS in the zone"
        means partially in, so a robot straddling the boundary is as
        protected as one parked in the middle."""
        for zone in self.field.protected_zones:
            if zone.alliance is not None and zone.alliance != robot.alliance:
                continue
            if polygons_intersect(robot.footprint(), zone.vertices):
                return zone
        return None

    def robots_in_contact(self, a: Robot, b: Robot) -> bool:
        """Whether two robots' bumpers are touching. One outline is grown
        by _CONTACT_TOLERANCE before the overlap test, so the moment of
        contact counts: two rigid bodies at rest against each other touch
        without overlapping, and a strict area test would call that no
        contact at all."""
        return polygons_intersect(a.footprint(_CONTACT_TOLERANCE), b.footprint())

    def _step_protection(self, dt: float) -> None:
        """Charge a foul for each opponent touching a protected robot.

        The cooldown is per offender/victim pair and lives across ticks,
        so an offender that parks against a protected robot is charged
        once per zone.foul_period rather than once per physics tick --
        a referee calls a foul, then keeps watching, and a longer
        violation costs more than a brief one without costing 60x more
        per second. Pairs are directional: two robots shoving each other
        inside overlapping zones each owe the other."""
        for key in list(self._foul_cooldown):
            self._foul_cooldown[key] -= dt
            if self._foul_cooldown[key] <= 0.0:
                del self._foul_cooldown[key]

        if not self.field.protected_zones:
            return

        protected = [(r, self.protecting_zone(r)) for r in self.robots]
        for victim, zone in protected:
            if zone is None:
                continue
            for offender in self.robots:
                if offender is victim or offender.alliance == victim.alliance:
                    continue
                key = (id(offender), id(victim))
                if key in self._foul_cooldown or not self.robots_in_contact(offender, victim):
                    continue
                self._foul_cooldown[key] = zone.foul_period
                self.protection_fouls[offender.alliance] = self.protection_fouls.get(offender.alliance, 0) + 1
                if zone.foul_points:
                    self.scores[victim.alliance] = self.scores.get(victim.alliance, 0.0) + zone.foul_points
                self.events.log(self.elapsed, "protection_foul", {
                    "zone": zone.name, "alliance": offender.alliance,
                    "against": victim.alliance, "points": zone.foul_points,
                })

    # -- emitters ----------------------------------------------------------

    def emitter_capacity_remaining(self, emitter) -> int | None:
        """None means unlimited. A station-linked emitter reads the same
        counter robots dispense from (self.station_supply), so it never
        double-counts a station's stock; an unlinked emitter reads its own
        self.emitter_supply, which is absent entirely (i.e. unlimited) when
        initial_capacity was None."""
        station = self._emitter_linked_stations.get(emitter)
        if station is not None:
            return self.station_supply.get(station)
        return self.emitter_supply.get(emitter)

    def _emitter_consume(self, emitter) -> None:
        station = self._emitter_linked_stations.get(emitter)
        if station is not None:
            if station in self.station_supply:
                self.station_supply[station] -= 1
        elif emitter in self.emitter_supply:
            self.emitter_supply[emitter] -= 1

    def _emitter_return(self, emitter) -> None:
        station = self._emitter_linked_stations.get(emitter)
        if station is not None:
            if station in self.station_supply:
                self.station_supply[station] += 1
        elif emitter in self.emitter_supply:
            self.emitter_supply[emitter] += 1

    def _emitter_active_now(self, emitter) -> bool:
        if not emitter.active_times:
            return True
        return any(start <= self.elapsed < end for start, end in emitter.active_times)

    def _step_emitters(self, dt: float) -> None:
        ready_returns = [pending for pending in self._pending_emitter_returns if pending[0] <= self.elapsed]
        for pending in ready_returns:
            self._pending_emitter_returns.remove(pending)
            self._emitter_return(pending[1])

        for emitter in self.field.emitter_regions:
            if not self._emitter_active_now(emitter):
                continue
            cooldown = self._emitter_cooldowns[emitter] - dt
            while cooldown <= 0.0:
                remaining = self.emitter_capacity_remaining(emitter)
                if remaining is not None and remaining <= 0:
                    cooldown = 0.0
                    break
                position = polygon_centroid(emitter.vertices)
                self.spawn_piece(emitter.piece_type, position, source="emitter")
                self._emitter_consume(emitter)
                self.events.log(self.elapsed, "emit", {"emitter": emitter.name, "piece_type": emitter.piece_type})
                cooldown += 1.0 / emitter.emit_rate_hz
            self._emitter_cooldowns[emitter] = cooldown

    # -- stepping ----------------------------------------------------------

    def step(self, dt: float) -> None:
        if self.ended:
            return

        self.elapsed += dt
        if self.phase == Phase.AUTO and self.elapsed >= self.config.auto_duration:
            self.phase = Phase.TELEOP
            self.events.log(self.elapsed, "phase_change", {"phase": self.phase.value})
        if self.elapsed >= self.config.total_duration:
            self.ended = True
            self.events.log(self.elapsed, "match_end", {})

        for robot in self.robots:
            if robot.controller is not None:
                ctx = BehaviorContext(robot=robot, dt=dt, elapsed=self.elapsed, match=self)
                robot.controller.tick(ctx)

            captured = robot.update_intake(dt)
            if captured is not None:
                captured.last_holder_alliance = robot.alliance
                self.events.log(self.elapsed, "intake", {"alliance": robot.alliance, "piece_type": captured.piece_type})

            target_station = robot.nearby_station()
            station_has_supply = self.station_supply.get(target_station, 1) > 0 if target_station is not None else True
            dispensed_at = robot.update_station_intake(dt, station_has_supply)
            if dispensed_at is not None:
                if dispensed_at in self.station_supply:
                    self.station_supply[dispensed_at] -= 1
                color_override = {"color": dispensed_at.piece_color} if dispensed_at.piece_color is not None else {}
                piece = self.spawn_piece(
                    dispensed_at.piece_type, robot.pose.as_tuple()[:2], source="station", **color_override,
                )
                piece.held_by = robot
                piece.shape.sensor = True
                piece.last_holder_alliance = robot.alliance
                robot.held_pieces.append(piece)
                self.events.log(self.elapsed, "intake", {"alliance": robot.alliance, "piece_type": piece.piece_type})

            target_piece = self.deposit_piece_for(robot)
            ready_region = self.deposit_region_for(robot, target_piece)
            released = robot.update_manipulator(dt, target_piece, scoring_ready=ready_region is not None)
            if released is not None:
                self.events.log(self.elapsed, "deposit", {"alliance": robot.alliance, "piece_type": released.piece_type})
                if ready_region is not None:
                    if self._roll_scoring_success(robot, released):
                        self._try_score(released, ready_region, explicit=True)
                    else:
                        self.events.log(self.elapsed, "score_miss", {
                            "alliance": robot.alliance, "piece_type": released.piece_type,
                        })
                else:
                    self._check_region_scoring(released)

            robot.sync_held_piece_positions()

        if self.config.emit_coral_to_field:
            self._step_emitters(dt)

        self.engine.step(dt)
        # After the solver, so contact is judged on where the robots
        # actually ended up this tick rather than where they were aiming.
        self._step_protection(dt)

        if any(p.scored for p in self.active_pieces):
            still_active, scored = [], []
            for p in self.active_pieces:
                (scored if p.scored else still_active).append(p)
            for p in scored:
                p.remove_from_space()
            self.active_pieces = still_active


def _default_piece_type(characteristics: RobotCharacteristics) -> str:
    accepted = characteristics.accepted_piece_types
    return next(iter(accepted)) if accepted else "piece"


def _order_by_type(shapes: tuple[pymunk.Shape, pymunk.Shape], first_type: int) -> tuple[pymunk.Shape, pymunk.Shape]:
    """`arbiter.shapes` is not reliably ordered to match the (type_a,
    type_b) order a handler was registered with -- pin the shape whose
    collision_type is `first_type` to the front explicitly rather than
    assuming positional order."""
    a, b = shapes
    return (a, b) if a.collision_type == first_type else (b, a)
