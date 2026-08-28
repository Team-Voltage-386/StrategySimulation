"""
Generic robot: a SwerveChassis plus an Intake/Manipulator pair and a
held-piece list. Game-agnostic -- no notion of scoring points or which
game piece types exist beyond the plain strings on GamePiece.piece_type.

Nearby-piece and scoring-overlap tracking is driven by pymunk collision
callbacks that Match registers globally (once per collision-type pair,
across all robots/pieces) and dispatches to the owning Robot/ScoringRegion
via shape attributes -- see match/match.py. Robot itself just exposes the
entry points those callbacks call.
"""
from __future__ import annotations

import math

import pymunk

from common_sim.field.field_config import IntakeLocation, point_in_polygon, polygon_centroid
from common_sim.field.game_piece import GamePiece
from common_sim.geometry import Pose2d
from common_sim.physics.swerve import SwerveChassis, SwerveLimits
from common_sim.robot.characteristics import SIDE_OUTWARD, RobotCharacteristics
from common_sim.robot.mechanisms import Intake, Manipulator, StationIntake


def _side_intake_poly(half_l: float, half_w: float, reach: float, side: str) -> list[tuple[float, float]]:
    if side == "front":
        return [(half_l, -half_w), (half_l + reach, -half_w), (half_l + reach, half_w), (half_l, half_w)]
    if side == "back":
        return [(-half_l, -half_w), (-half_l - reach, -half_w), (-half_l - reach, half_w), (-half_l, half_w)]
    if side == "left":
        return [(-half_l, half_w), (-half_l, half_w + reach), (half_l, half_w + reach), (half_l, half_w)]
    if side == "right":
        return [(-half_l, -half_w), (-half_l, -half_w - reach), (half_l, -half_w - reach), (half_l, -half_w)]
    raise ValueError(f"unknown side {side!r}")


def _closest_boundary_point(point: tuple[float, float], vertices) -> tuple[float, float]:
    """The point on the polygon's boundary nearest `point` -- the part of
    a scoring zone a robot outside it is actually up against."""
    px, py = point
    best, best_d2 = vertices[0], float("inf")
    for i in range(len(vertices)):
        ax, ay = vertices[i]
        bx, by = vertices[(i + 1) % len(vertices)]
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        t = 0.0 if length2 < 1e-12 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
        qx, qy = ax + t * dx, ay + t * dy
        d2 = (px - qx) ** 2 + (py - qy) ** 2
        if d2 < best_d2:
            best, best_d2 = (qx, qy), d2
    return best


class Robot:
    # Forward push applied to a piece on deposit release, on top of chassis
    # velocity -- separates it from the chassis footprint so it can reach an
    # adjacent scoring region rather than sitting pinned at the robot's
    # center.
    #
    # The value actually used is `characteristics.eject_speed`, which
    # defaults to this: how hard a robot throws is a property of the robot,
    # not of the class, and a robot built to pass throws much harder. Kept
    # here as the default's documented source.
    DEFAULT_EJECT_SPEED = 24.0  # in/s

    # How far *back* inside its own bumper edge a side's manipulator still
    # counts as reaching, in inches. Two solid bodies pressed together
    # settle with a small overlap in pymunk (collision slop), so a test
    # point placed exactly at the bumper edge reads as slightly *past* a
    # structure's face whenever the robot is flush against it -- which
    # would make scoring work only while backed a little off the very
    # structure being scored on. A real mechanism also occupies some depth
    # rather than being a knife-edge at the bumper line.
    MANIPULATOR_INSET = 3.0  # in

    # How finely side_reach_points samples its span; small enough that no
    # plausible scoring zone fits between two consecutive samples.
    REACH_SAMPLE_SPACING = 1.5  # in

    # Minimum cos(angle) between a side's outward normal and the bearing
    # from the chassis center to a target, for that side to count as the
    # one presented toward it -- 0.5 allows a 60-degree approach cone.
    # Reach alone is not enough: a scoring zone sits in the space between
    # a structure's face and the robot approaching it, so a side pointing
    # some *other* way can still clip a corner of the zone its own
    # chassis is parked in.
    SIDE_FACING_TOLERANCE = 0.5

    # Lateral spacing between multiple simultaneously-held pieces, in
    # inches -- keeps e.g. a held coral and algae from being drawn stacked
    # exactly on top of each other at the chassis center. Physics-wise the
    # pieces are sensors pinned kinematically (see sync_held_piece_positions),
    # so this offset is purely cosmetic/visual, not a mechanism model.
    HELD_PIECE_SPACING = 7.0  # in

    def __init__(
        self,
        space: pymunk.Space,
        characteristics: RobotCharacteristics,
        start_pose: Pose2d,
        *,
        alliance: str = "blue",
        chassis_collision_type: int = 0,
        intake_collision_type: int = 0,
        chassis_shape_filter: pymunk.ShapeFilter | None = None,
    ):
        self.characteristics = characteristics
        self.alliance = alliance

        limits = SwerveLimits(
            max_speed=characteristics.max_speed,
            max_accel=characteristics.max_accel,
            max_angular_speed=characteristics.max_angular_speed,
            max_angular_accel=characteristics.max_angular_accel,
        )
        self.chassis = SwerveChassis(
            space,
            limits,
            width=characteristics.width,
            length=characteristics.length,
            mass=characteristics.mass,
            start_pose=start_pose,
            collision_type=chassis_collision_type,
            shape_filter=chassis_shape_filter,
        )

        # Intake sensors: one wedge per side with intake capability,
        # extending outward from that side's bumper by intake_range. A
        # robot with no side configuration gets the legacy single
        # forward-facing wedge (RobotCharacteristics.active_intake_sides()).
        half_l = characteristics.length / 2.0
        half_w = characteristics.width / 2.0
        reach = characteristics.intake_range
        self.intake_shapes: dict[str, pymunk.Shape] = {}
        for side in characteristics.active_intake_sides():
            shape = pymunk.Poly(self.chassis.body, _side_intake_poly(half_l, half_w, reach, side))
            shape.sensor = True
            shape.collision_type = intake_collision_type
            shape.owner_robot = self
            shape.intake_side = side
            space.add(shape)
            self.intake_shapes[side] = shape

        self.intake = Intake()
        self.station_intake = StationIntake()
        self.manipulator = Manipulator()
        self.held_pieces: list[GamePiece] = []

        # piece -> set of sides it's currently overlapping (a piece can sit
        # in more than one side's wedge at once near a corner).
        self._nearby_pieces: dict[GamePiece, set[str]] = {}
        # location -> set of sides currently overlapping it (mirrors
        # _nearby_pieces) -- insertion order of the dict is entry order.
        self._nearby_stations: dict[IntakeLocation, set[str]] = {}
        self._commanded_intake = False
        self._commanded_deposit = False
        self._deposit_action: str | None = None
        # Whether the deposit attempt currently in progress is spending
        # the real deposit timer vs. dropping instantly -- latched once
        # per commanded-deposit press (see update_manipulator) rather
        # than re-read every tick, so a sub-inch chassis nudge mid-drop
        # (e.g. recoil from the previous piece's eject) can't flip an
        # already-timing deposit into an instant one partway through.
        self._deposit_was_active = False
        self._deposit_ready_latched = True

        # side -> (pose_key, points) memo for side_reach_points -- deposit
        # region checks call it repeatedly (once per candidate scoring
        # region) against the same live pose within a tick; keyed on pose
        # so it's still recomputed the moment the robot actually moves.
        self._reach_cache: dict[str, tuple[tuple[float, float, float], list[tuple[float, float]]]] = {}

        # Set by Match.add_robot when a controller= is passed; Match.step
        # ticks it each frame before mechanism/physics updates. None for
        # a human-driven robot (an InputSource-driven loop drives it
        # directly instead) -- unaffected either way.
        self.controller = None

    @property
    def intent(self):
        """Passthrough to `self.controller.intent` (see strategy.py's
        StrategyController) -- what a Defend tactic on an opposing robot
        reads, and what a GUI overlay draws. None for a controller-less
        (human-driven or uncontrolled) robot."""
        return self.controller.intent if self.controller is not None else None

    @property
    def pose(self) -> Pose2d:
        return self.chassis.pose

    @property
    def commanded_speed(self) -> float:
        """How fast this robot is *asking* to go, in/s.

        Translation only -- a robot asking for rotation and nothing else
        reads 0.0 here. Anything deciding whether a motionless robot is
        waiting or being held has to read `commanded_angular_speed` too;
        see DRY_RUN_LOG.md (F3) for the 110-second stall that was
        invisible to this number on its own."""
        return self.chassis.commanded_velocity.length

    @property
    def commanded_angular_speed(self) -> float:
        """How fast this robot is *asking* to rotate, rad/s, unsigned."""
        return abs(self.chassis.commanded_omega)

    @property
    def speed(self) -> float:
        """How fast it is actually going, in/s."""
        vx, vy = self.chassis.body.velocity
        return math.hypot(vx, vy)

    @property
    def deposit_action(self) -> str | None:
        return self._deposit_action

    @property
    def deposit_active(self) -> bool:
        """Whether a deposit is currently commanded. Readable so a caller
        can re-publish the action (see set_deposit_active) without having
        to know, or accidentally change, whether the trigger is held."""
        return self._commanded_deposit

    def drive_field_relative(self, dt: float, vx: float, vy: float, omega: float) -> None:
        self.chassis.drive_field_relative(dt, vx, vy, omega)

    def drive_robot_relative(self, dt: float, vx: float, vy: float, omega: float) -> None:
        self.chassis.drive_robot_relative(dt, vx, vy, omega)

    def set_intake_active(self, active: bool) -> None:
        self._commanded_intake = active

    def set_deposit_active(self, active: bool, action: str | None = None) -> None:
        """`action` selects which scoring action (e.g. "l1_coral" vs
        "l4_coral") the manipulator is attempting -- it determines both
        how long the deposit takes (RobotCharacteristics.deposit_time_by_action)
        and which ScoringRegion.actions the released piece can score
        against. Commanding active=True with action=None is a deliberate
        no-op (the timer does not run) rather than an error, since a
        driver UI may briefly have "deposit held" true before a target
        level is chosen."""
        self._commanded_deposit = active
        if action is not None:
            self._deposit_action = action

    # -- collision-callback entry points (called by Match) ----------------

    def piece_entered_intake_range(self, piece: GamePiece, side: str) -> None:
        if piece.held_by is None:
            self._nearby_pieces.setdefault(piece, set()).add(side)

    def piece_left_intake_range(self, piece: GamePiece, side: str) -> None:
        sides = self._nearby_pieces.get(piece)
        if sides is None:
            return
        sides.discard(side)
        if not sides:
            self._nearby_pieces.pop(piece, None)

    def station_entered(self, location: IntakeLocation, side: str) -> None:
        self._nearby_stations.setdefault(location, set()).add(side)

    def station_left(self, location: IntakeLocation, side: str) -> None:
        sides = self._nearby_stations.get(location)
        if sides is None:
            return
        sides.discard(side)
        if not sides:
            self._nearby_stations.pop(location, None)

    # -- per-tick updates ---------------------------------------------------

    def duration_for(self, piece: GamePiece) -> float:
        return (
            self.characteristics.station_intake_time
            if piece.source == "station"
            else self.characteristics.intake_duration(piece.piece_type)
        )

    def accepts(self, piece: GamePiece) -> bool:
        sides = self._nearby_pieces.get(piece)
        if not sides:
            return False
        source = "station" if piece.source == "station" else "field"
        return any(self.characteristics.side_intake_accepts(s, piece.piece_type, source=source) for s in sides)

    def _capacity_available_for(self, piece_type: str) -> bool:
        held_of_type = sum(1 for p in self.held_pieces if p.piece_type == piece_type)
        return held_of_type < self.characteristics.capacity_for(piece_type)

    def has_capacity_for(self, piece_type: str) -> bool:
        """Whether one more `piece_type` would fit -- the same rule the
        intake enforces, exposed so behaviors decide to keep collecting
        on the same answer the physics will give them."""
        return self._capacity_available_for(piece_type)

    def is_full_for(self, piece_type: str | None = None) -> bool:
        """Whether there is any point collecting `piece_type` (None =
        anything at all).

        Under a per-type capacity, a robot full of one type is not full:
        a coral cycler that scooped an ALGAE it has no rule to score
        still has its coral slot free. Counting all held pieces against
        one shared limit called that robot full, so it stopped
        collecting, stopped driving, and sat out the rest of the match
        holding one algae -- while its intake, which has always been
        per-type, would happily have taken a coral.

        Without `piece_capacity_by_type` the capacity really is a single
        shared pool, and any piece fills it; that legacy meaning is kept
        exactly."""
        characteristics = self.characteristics
        if not characteristics.piece_capacity_by_type:
            return len(self.held_pieces) >= characteristics.piece_capacity
        if piece_type is not None:
            return not self._capacity_available_for(piece_type)
        # "Full" with no type named means nothing can be taken at all.
        return not any(
            self._capacity_available_for(t) for t in characteristics.piece_capacity_by_type
        )

    def update_intake(self, dt: float) -> GamePiece | None:
        """Advance the intake timer and capture a piece if it completes.
        Returns the captured GamePiece, or None."""
        nearby = [
            p for p in self._nearby_pieces
            if p.held_by is None and self.accepts(p) and self._capacity_available_for(p.piece_type)
        ]
        captured = self.intake.update(dt, self._commanded_intake, nearby, bool(nearby), self.duration_for)
        if captured is not None:
            captured.held_by = self
            captured.shape.sensor = True  # pinned to the chassis now; no physical collision while held
            self._nearby_pieces.pop(captured, None)
            self.held_pieces.append(captured)
        return captured

    def nearby_station(self) -> IntakeLocation | None:
        """The IntakeLocation this robot would target on its next
        update_station_intake() call -- exposed so Match can check that
        location's remaining supply before dt advances the timer. Only a
        location currently touched by a side actually configured to
        intake its piece_type counts -- e.g. a side wired for algae only
        can sit inside a CORAL station's zone without triggering it."""
        for location, sides in self._nearby_stations.items():
            if any(self.characteristics.side_intake_accepts(s, location.piece_type, source="station") for s in sides):
                return location
        return None

    def update_station_intake(self, dt: float, station_has_supply: bool = True) -> IntakeLocation | None:
        """Advance the station-dispense timer. Returns the IntakeLocation
        once it completes, or None -- the caller (Match) is responsible
        for actually materializing the dispensed piece, since Robot has
        no access to piece-spawning/collision-type bookkeeping.
        `station_has_supply` lets Match veto dispensing (and reset the
        timer) once a location with a finite starting_pieces count has
        run dry."""
        location = self.nearby_station()
        capacity_available = station_has_supply and (
            location is None or self._capacity_available_for(location.piece_type)
        )
        return self.station_intake.update(
            dt, self._commanded_intake, location, capacity_available, self.characteristics.station_intake_time,
        )

    def footprint(self, margin: float = 0.0) -> tuple[tuple[float, float], ...]:
        """World-frame bumper corners at the current pose -- the outline
        rules reason about, since a robot is "in" a zone when any part of
        its BUMPERS is. Read off the physics shape rather than rebuilt
        from width/length so it can never disagree with what actually
        collides.

        `margin` grows the outline outward by that much on every side,
        for a caller asking "near enough to count as touching" rather
        than "strictly overlapping". Each corner moves away from the
        chassis center along both body axes, which is the correct offset
        for the centered rectangle a bumper shape always is."""
        body = self.chassis.body
        points = []
        for v in self.chassis.bumper_shape.get_vertices():
            if margin:
                v = pymunk.Vec2d(v.x + math.copysign(margin, v.x), v.y + math.copysign(margin, v.y))
            world = body.local_to_world(v)
            points.append((world.x, world.y))
        return tuple(points)

    def side_bumper_point(self, side: str) -> tuple[float, float]:
        """World-frame location of `side`'s bumper-edge center, given the
        robot's current pose."""
        half_l, half_w = self.characteristics.length / 2.0, self.characteristics.width / 2.0
        local = {
            "front": (half_l, 0.0), "back": (-half_l, 0.0), "left": (0.0, half_w), "right": (0.0, -half_w),
        }[side]
        point = self.chassis.body.position + pymunk.Vec2d(*local).rotated(self.chassis.body.angle)
        return (point.x, point.y)

    def side_reach_points(self, side: str) -> list[tuple[float, float]]:
        """Sample points along `side`'s outward *centerline*, spanning from
        MANIPULATOR_INSET behind its bumper edge out to intake_range
        beyond it -- the depth a manipulator mounted on that side can
        physically act over, recomputed fresh from the live pose (and so
        rotating with the robot's heading).

        A scoring region is typically sized to cover only the true entry
        face (e.g. a REEF face or PROCESSOR opening), not the robot's
        whole footprint, so the chassis *center* -- half_l/half_w behind
        whichever bumper is actually presented to the structure -- can sit
        well outside it even while the correct side is squarely in
        position. These are the points that actually need to land inside.

        Sampled along the centerline rather than swept across the side's
        full width so a robot sitting near the corner between two adjacent
        targets (e.g. two REEF faces meeting at a hex vertex) doesn't
        register against a neighbor it isn't actually squared up to.

        Memoized per side against the live pose (position + heading) --
        callers within the same tick and same pose (e.g.
        Match.deposit_region_for scanning several candidate regions) reuse
        the cached points; any pose change invalidates it."""
        body = self.chassis.body
        pose_key = (body.position.x, body.position.y, body.angle)
        cached = self._reach_cache.get(side)
        if cached is not None and cached[0] == pose_key:
            return cached[1]
        origin = pymunk.Vec2d(*self.side_bumper_point(side))
        outward = pymunk.Vec2d(*SIDE_OUTWARD[side]).rotated(self.chassis.body.angle)
        near, far = -self.MANIPULATOR_INSET, self.characteristics.intake_range
        steps = max(1, int(round((far - near) / self.REACH_SAMPLE_SPACING)))
        points = []
        for i in range(steps + 1):
            offset = near + (far - near) * i / steps
            p = origin + outward * offset
            points.append((p.x, p.y))
        self._reach_cache[side] = (pose_key, points)
        return points

    def side_engages_polygon(self, side: str, vertices) -> bool:
        """Whether `side`'s manipulator is in position to act on the given
        polygon: it must both physically *reach* into it and be the side
        actually *presented* toward it. Both halves are needed --

        reach alone accepts a robot parked in a zone with some other side
        pointing at it (a scoring zone occupies the space between a
        structure's face and the approaching robot, so the robot's own
        body sits in/next to the zone no matter how it's turned, and a
        side pointing elsewhere can still clip a corner of it);

        facing alone accepts a robot squared up but out of range, or one
        whose mechanism is nowhere near the zone."""
        points = self.side_reach_points(side)
        # The reach points are collinear along one centerline, so the
        # first and last bound all of them: if that box misses the
        # polygon's box, no point is inside and there is nothing to ray
        # cast. Worth the four comparisons because the hot callers
        # (Match.deposit_region_for, Match.pickup_region_for) walk every
        # region the field declares looking for the one a robot is at, and
        # the answer is "not this one" for nearly all of them -- which
        # otherwise costs a full ray cast per reach point per region.
        (x1, y1), (x2, y2) = points[0], points[-1]
        min_x, max_x = (x1, x2) if x1 <= x2 else (x2, x1)
        min_y, max_y = (y1, y2) if y1 <= y2 else (y2, y1)
        if (max(v[0] for v in vertices) < min_x or min(v[0] for v in vertices) > max_x
                or max(v[1] for v in vertices) < min_y or min(v[1] for v in vertices) > max_y):
            return False
        if not any(point_in_polygon(p, vertices) for p in points):
            return False
        return self._side_faces_polygon(side, vertices)

    def _side_faces_polygon(self, side: str, vertices) -> bool:
        """Whether `side`'s outward normal points from the chassis center
        toward the polygon, within SIDE_FACING_TOLERANCE. Measured from
        the chassis center (not the bumper edge) because the target
        usually lies *behind* the bumper line -- between the structure
        and the robot -- so a bumper-relative bearing doesn't
        discriminate, while a center-relative one does.

        The bearing is taken to the nearest point of the zone, not to its
        centroid. For a zone sized to a structure's face the two are the
        same direction, but a centroid is only a stand-in for "where the
        zone is" while the zone is small compared to the robot. Against
        REEFSCAPE's 80x285in NET it was badly wrong: a robot squared up
        to the near edge with its manipulator inside the zone read as not
        facing it, because the centroid was 77in off its nose. It parked
        in a legal scoring pose and never scored. A robot whose center is
        inside the zone faces it from any heading -- there is no bearing
        to speak of, and a zone that large models an area you score from
        rather than a face you square up to."""
        center = self.chassis.body.position
        if point_in_polygon((center.x, center.y), vertices):
            return True
        cx, cy = _closest_boundary_point((center.x, center.y), vertices)
        dx, dy = cx - center.x, cy - center.y
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            return True
        outward = pymunk.Vec2d(*SIDE_OUTWARD[side]).rotated(self.chassis.body.angle)
        return (outward.x * dx + outward.y * dy) / distance >= self.SIDE_FACING_TOLERANCE

    def scoring_side(self, piece_type: str | None = None) -> str | None:
        """The side that would release a deposit of `piece_type`, or None
        if the robot isn't holding anything. `piece_type` defaults to the
        FIFO-first held piece's type, which is only ever correct while a
        robot holds a single type at once -- a robot holding two types
        simultaneously (e.g. coral + algae) needs the caller to say which
        one it means (see Match.deposit_piece_for), since the two can use
        different sides. This is the single definition of "which side is
        doing the scoring" that both Match's deposit gate and a GUI's
        readiness indicator read, so the two can't disagree about it."""
        if not self.held_pieces:
            return None
        if piece_type is None:
            piece_type = self.held_pieces[0].piece_type
        return self.characteristics.score_side_for(piece_type)

    def update_manipulator(
        self, dt: float, target: GamePiece | None = None, scoring_ready: bool = True,
    ) -> GamePiece | None:
        """Advance the deposit timer and, once it completes, release
        `target` (or, if not given, the FIFO-first held piece -- correct
        only while a robot holds a single type at once). `target` lets a
        caller holding multiple piece types (e.g. coral + algae) say which
        one the current commanded action actually applies to, since the
        action string alone doesn't say (see Match.deposit_piece_for).
        No-ops (timer does not run) if no target action has ever been
        selected via set_deposit_active(..., action=...). NOT gated on
        being positioned inside a valid scoring region -- a robot can drop
        a held piece anywhere on the field; Match only decides afterward,
        from the release position, whether it actually scored (see
        Match._try_score / _check_region_scoring). `scoring_ready` (Match
        passes whether deposit_region_for currently resolves) only affects
        *how long that takes*: a real scoring deposit spends the
        action's deposit_duration, but a drop with nothing to score
        against happens on the very next tick -- there's no manipulator
        motion to model when it's just discarding the piece onto the
        field, and waiting out the same timer made an obviously-missed
        drop look like it was still trying to score. Only sampled on the
        rising edge of the commanded press (see _deposit_ready_latched)
        -- re-reading it every tick would let a mid-drop chassis wobble
        that's already timing toward a real score flip it to instant
        partway through."""
        if target is None and self.held_pieces:
            target = self.held_pieces[0]
        has_piece = target is not None
        action = self._deposit_action
        active = self._commanded_deposit and action is not None
        if active and not self._deposit_was_active:
            self._deposit_ready_latched = scoring_ready
        self._deposit_was_active = active
        duration = (
            self.characteristics.deposit_duration(action) if (action is not None and self._deposit_ready_latched) else 0.0
        )
        completed = self.manipulator.update(dt, active, has_piece, duration)
        if completed:
            piece = target
            self.held_pieces.remove(piece)
            piece.held_by = None
            piece.shape.sensor = False
            piece.target_action = action
            eject_side = self.characteristics.score_side_for(piece.piece_type)
            outward = pymunk.Vec2d(*SIDE_OUTWARD[eject_side]).rotated(self.chassis.body.angle)
            self.release_piece(piece, velocity=pymunk.Vec2d(*self.chassis.body.velocity) + outward * self.characteristics.eject_speed)
            return piece
        return None

    def intake_progress_fraction(self) -> float | None:
        """0-1 progress toward capturing the currently targeted piece (or,
        while sitting in an IntakeLocation's zone, toward the next
        dispensed piece), or None if no intake is in progress -- for a
        GUI countdown display."""
        target = self.intake.target
        if target is not None and self.intake.progress > 0.0:
            duration = self.duration_for(target)
            return min(1.0, self.intake.progress / duration) if duration > 0 else None

        location = self.station_intake.target
        if location is not None and self.station_intake.progress > 0.0:
            duration = self.characteristics.station_intake_time
            return min(1.0, self.station_intake.progress / duration) if duration > 0 else None

        return None

    def deposit_progress_fraction(self) -> float | None:
        """0-1 progress toward completing the currently commanded deposit,
        or None if no deposit is in progress -- for a GUI countdown display."""
        if not self._commanded_deposit or self._deposit_action is None or not self.held_pieces:
            return None
        if self.manipulator.progress <= 0.0:
            return None
        duration = self.characteristics.deposit_duration(self._deposit_action)
        return min(1.0, self.manipulator.progress / duration) if duration > 0 else None

    def sync_held_piece_positions(self) -> None:
        """Held pieces are kinematically pinned to the chassis center each
        tick rather than left to physics -- this sim isn't trying to model
        in-mechanism piece dynamics. Multiple simultaneously-held pieces
        (e.g. a coral and an algae) are fanned out side-by-side along the
        chassis's local lateral axis (see HELD_PIECE_SPACING) purely so a
        GUI can show them as visibly distinct dots instead of one hiding
        behind the other."""
        count = len(self.held_pieces)
        for i, piece in enumerate(self.held_pieces):
            local_y = (i - (count - 1) / 2.0) * self.HELD_PIECE_SPACING
            offset = pymunk.Vec2d(0.0, local_y).rotated(self.chassis.body.angle)
            piece.body.position = self.chassis.body.position + offset
            piece.body.velocity = self.chassis.body.velocity

    def release_piece(self, piece: GamePiece, velocity: tuple[float, float] | None = None) -> None:
        """Used by a Manipulator that launches (vs. gently deposits) a
        piece -- sets an explicit exit velocity instead of inheriting the
        chassis's."""
        if velocity is not None:
            piece.body.velocity = velocity
