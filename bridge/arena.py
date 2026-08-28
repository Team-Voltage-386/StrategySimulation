"""The REBUILT 2026 arena, as maple-sim actually builds it.

**This is geometry, not a game.** There are no scoring rules here, no
piece lifecycle, and no point table, and this field is not playable in
`common_sim.match`. It exists so the strategy layer can *navigate* the
arena the robot code is really driving in -- where the walls are, where
the HUBs stand, which gaps a robot fits through. Everything dynamic --
fuel positions, which HUB is currently active, the score -- is published
live by maple-sim over NetworkTables and read from there, not modelled
here. See `bridge/world_state.py`.

That division is the whole reason step 4 does not need REBUILT
implemented in `game_specific/`. maple-sim already implements REBUILT,
including its scoring; a second implementation on this side would be a
model to keep in sync with the one that actually decides what happens,
and the two would disagree on the night it mattered.

Every constant below is transcribed from maple-sim 0.4.0-beta:

    org/ironmaple/simulation/seasonspecific/rebuilt2026/Arena2026Rebuilt.java
    org/ironmaple/simulation/seasonspecific/rebuilt2026/RebuiltHub.java
    org/ironmaple/simulation/seasonspecific/rebuilt2026/RebuiltOutpost.java

Source values are kept in metres with their Java names attached, so this
file can be diffed against that source when maple-sim is upgraded. The
conversion to sparky-sim's inches happens in exactly one place (`_in`).
Do not "tidy" the metre constants into round inches -- the point is that
they are copies.

Origin and axes agree between the two simulators: (0, 0) is the blue
alliance wall's right-hand corner looking downfield, +x runs toward the
red wall, +y across the field. Only the unit differs.
"""
from __future__ import annotations

import math

from common_sim.field.field_config import FieldConfig, Obstacle, ScoringRegion
from common_sim.field.game_piece import GamePieceSpec, register_piece_spec

M_TO_IN = 39.37007874015748


def _in(metres: float) -> float:
    return metres * M_TO_IN


# -- the field itself ---------------------------------------------------
# Arena2026Rebuilt.RebuiltFieldObstaclesMap, the four addBorderLine calls.
FIELD_LENGTH_M = 16.540988
FIELD_WIDTH_M = 8.052

FIELD_LENGTH = _in(FIELD_LENGTH_M)  # 651.22 in
FIELD_WIDTH = _in(FIELD_WIDTH_M)  # 317.01 in


# -- the HUBs -----------------------------------------------------------
# RebuiltHub.blueHubPose / .redHubPose. The z component (1.5748) is the
# goal height and is dropped: this is a floor-plan.
HUB_CENTRE_M = {
    "blue": (4.5974, 4.034536),
    "red": (11.938, 4.034536),
}

# RebuiltHub.GoalRadius -- the radius of the goal mouth, and also the
# +x offset of the four shoot poses from the hub centre, which is what
# puts the goal face on the midfield side of each HUB.
#
# It is also the *scoring* test, and this is the number that decides how
# hard REBUILT is to score in. `RebuiltHub.checkCollision` counts a piece
# as scored when it is within GoalRadius of the hub position **in three
# dimensions** -- and that position is 1.5748 m up. So the target is a
# 60 cm sphere floating at chest height, not a mouth on a wall, and a
# shot that is well aimed in plan view can still miss it entirely.
#
# `addPoints` then returns the fuel to the field from one of the four
# shoot poses at 2 m/s, so scored fuel comes back. That is the chute,
# not a bug in the piece accounting.
GOAL_RADIUS_M = 0.5969
GOAL_HEIGHT_M = 1.5748

# Arena2026Rebuilt.RebuiltFieldObstaclesMap, the `AddRampCollider` branch.
# The no-argument `Arena2026Rebuilt()` constructor passes true, and
# `SimulatedArena.getInstance()` -- which is what SimContainer calls --
# uses that constructor. So the arena the robot code is driving in has
# the *ramps* included in the HUB collider, and each HUB is a 47 x 217
# inch wall, not a 47-inch box.
#
# That 217 inches is the single most consequential number in this file.
# Two of them, spanning y = 50.3 .. 267.3 on a 317-inch-wide field, leave
# a gap of about 50 inches at each end for a 30-inch robot to get through
# -- and those two gaps are the only ways past. It is why matches wedge:
# see the false-positive analysis in bridge/README.md, which spent a
# campaign discovering empirically what this constant says outright.
HUB_SIZE_IN = (47.0, 217.0)
HUB_SIZE_WITHOUT_RAMPS_IN = (47.0, 47.0)


# -- the trench walls ---------------------------------------------------
# Four 53 x 12 inch blocks, placed at (8.27, 4.035) +/- these offsets.
TRENCH_CENTRE_M = (8.27, 4.035)
TRENCH_OFFSET_X_IN = 120.0 + 47.0 / 2  # 143.5
TRENCH_OFFSET_Y_IN = 73.0 + 47.0 / 2 + 6.0  # 102.5
TRENCH_SIZE_IN = (53.0, 12.0)


# -- the tower poles ----------------------------------------------------
# Already written in inches in the Java source, and not symmetric: the
# two poles sit at different y (159 and 170), and the second is placed
# from 651 rather than from the field length of 651.22.
TOWER_POLE_SIZE_IN = (2.0, 47.0)
TOWER_POLES_IN = ((42.0, 159.0), (651.0 - 42.0, 170.0))


# -- where fuel starts, and where more of it arrives --------------------
# Not modelled as field features: every one of these is a pile of loose
# fuel on the floor, and loose fuel is published live in
# `FieldSimulation/Fuel`. Reading the real positions beats predicting
# them, and the OUTPOSTs add to those piles at times only maple-sim
# knows. Kept as constants because an operator that wants to go *find*
# fuel benefits from knowing where it tends to be.
#
# Arena2026Rebuilt.placeGamePiecesOnField: a 12 x 30 grid at 5.991 x 5.95
# inch spacing, thinned to every third row unless efficiency mode is
# turned off.
CENTRE_FUEL_CORNER_M = (7.35737, 1.724406)
FUEL_SPACING_IN = (5.991, 5.95)
CENTRE_FUEL_GRID = (12, 30)

# The two 4 x 6 corner piles. Note the names are maple-sim's own and read
# backwards against position: `blueDepot` is at the *red* wall. Only one
# of the two is spawned in efficiency mode, chosen by the driver
# station's alliance.
DEPOT_CORNERS_M = {
    "blueDepot": (16.0274, 1.646936),
    "redDepot": (0.02, 5.53),
}
DEPOT_GRID = (4, 6)

# RebuiltOutpost -- the human player positions that feed fuel back in.
# The red one is at x = 16.621, i.e. outside the 16.541 field wall: these
# are people, not floor. The dump poses are where fuel lands.
OUTPOST_DUMP_M = {
    "blue": (0.2, 0.665988),
    "red": (16.421, 7.2),
}


# -- FUEL ---------------------------------------------------------------
# RebuiltFuelOnField.REBUILT_FUEL_INFO: a dyn4j Circle takes a *radius*,
# so 7.5 cm is the radius and the ball is 15 cm across -- which is why
# the spawn grid steps by 5.99 inches. The piles start touching.
PIECE_TYPE = "Fuel"
FUEL_RADIUS_M = 0.075
FUEL_MASS_KG = 0.5 * 0.45359237  # Pounds.of(0.5)

# Registered at import, following the pattern in each game's
# game_pieces.py. Nothing in the bridge spawns a piece -- maple-sim owns
# the piece physics and we only read positions -- but the field validator
# refuses to check a region whose piece type has no registered size, and
# a field that cannot be validated is a field nobody checked.
register_piece_spec(
    PIECE_TYPE,
    GamePieceSpec(radius=_in(FUEL_RADIUS_M), mass=FUEL_MASS_KG, color="orange"),
)


# -- the alliance zones -------------------------------------------------
# `RobotContainer.isInAllianceArea`, and the single most consequential
# rule in this file after the HUB collider:
#
#     blue: pose.x < 4.625594      red: pose.x > 11.915394
#
# **A robot outside its own zone cannot score.** `Turret.setTarget`
# branches on exactly this: inside, it aims at the HUB; outside, it aims
# at a corner of its own zone and *passes* the fuel back. The shot still
# happens, the ball still leaves, the ball still lands on the field --
# and nothing about that looks different from a missed shot.
#
# This was worth a whole run to learn. The strategy layer shot 65 pieces
# from midfield with auto-aim confirmed on and scored nothing, which read
# as "the robot's aim is broken" when it was doing precisely what it was
# told. The scoring regions were in the wrong place.
#
# Note the boundaries are the trench-wall line to within a millimetre
# (TRENCH_CENTRE_M[0] -/+ TRENCH_OFFSET_X_IN), which is a useful check
# that both transcriptions read the same field.
ALLIANCE_ZONE_BOUND_M = {"blue": 4.625594, "red": 11.915394}

# Where a robot outside its zone throws fuel instead: Constants
# blue/red Left/RightCorner, chosen by `verticalHalfOfField` (which side
# of y = 4.034536 the robot is on).
PASS_TARGETS_M = {
    "blue": ((2.5, 2.0), (2.5, 6.069326)),
    "red": ((14.0, 6.069326), (14.0, 2.0)),
}


def in_alliance_zone(x_in: float, alliance: str) -> bool:
    """Whether a robot at this x (inches) can score rather than pass.

    Computed here rather than read off NetworkTables because
    `Turret.isScoring` is not logged -- but `isInAllianceArea` is a pure
    function of the pose, so the Python side can evaluate the same rule
    exactly rather than observe its consequences.
    """
    bound = _in(ALLIANCE_ZONE_BOUND_M[alliance])
    return x_in < bound if alliance == "blue" else x_in > bound


# -- the HUB clock ------------------------------------------------------
# Arena2026Rebuilt.simulationSubTick: outside autonomous, exactly one
# alliance's HUB is active at a time and they swap every 25 seconds.
# Which one is live is read from NetworkTables, not tracked here.
HUB_PHASE_SECONDS = 25.0


def _rect(centre: tuple[float, float], width: float, depth: float) -> tuple[tuple[float, float], ...]:
    """A rectangle in the same sense dyn4j's `Geometry.createRectangle`
    means it: `width` along x, `depth` along y, centred on `centre`."""
    hw, hd = width / 2.0, depth / 2.0
    cx, cy = centre
    return ((cx - hw, cy - hd), (cx + hw, cy - hd), (cx + hw, cy + hd), (cx - hw, cy + hd))


def hub_centre(alliance: str) -> tuple[float, float]:
    x, y = HUB_CENTRE_M[alliance]
    return (_in(x), _in(y))


def goal_face_x(alliance: str) -> float:
    """The x of the HUB's goal mouth.

    RebuiltHub puts the blue shoot poses at +GoalRadius in x from the
    blue hub centre, and mirrors them for red, so both mouths open toward
    midfield.

    Which is *not* where a robot shoots from. The mouths face midfield
    and the alliance zone is behind the HUB, so a robot scores from its
    own side and the fuel goes over the structure -- see
    `build_scoring_regions`. This function describes the goal, not the
    approach.
    """
    x = _in(HUB_CENTRE_M[alliance][0])
    return x + _in(GOAL_RADIUS_M) if alliance == "blue" else x - _in(GOAL_RADIUS_M)


def shooting_face_x(alliance: str) -> float:
    """The x of the HUB face a robot in its own zone actually stands at:
    the *back* of the structure, on the alliance's own side."""
    x = _in(HUB_CENTRE_M[alliance][0])
    return x - _in(GOAL_RADIUS_M) if alliance == "blue" else x + _in(GOAL_RADIUS_M)


def trench_wall_centres(*, faithful: bool = True) -> tuple[tuple[float, float], ...]:
    """The trench wall positions.

    `faithful=True` reproduces what maple-sim 0.4.0-beta actually places,
    which is **three** walls, not four: the fourth `addRectangularObstacle`
    call in `RebuiltFieldObstaclesMap` repeats the (-x, -y) corner
    verbatim, so the (+x, +y) corner has no wall. That is a bug in
    maple-sim, and it is reproduced here on purpose -- this file describes
    the arena the robot is driving in, and a navigator that steers around
    an obstacle the physics does not contain is wrong in the direction
    that is hardest to notice.

    `faithful=False` gives the symmetric four the source clearly intended,
    for the day it is fixed upstream. Check the source before flipping it:
    a wall appearing under a navigator that was tuned without it is worth
    knowing about deliberately rather than by surprise.
    """
    cx, cy = _in(TRENCH_CENTRE_M[0]), _in(TRENCH_CENTRE_M[1])
    dx, dy = TRENCH_OFFSET_X_IN, TRENCH_OFFSET_Y_IN
    corners = ((-1, -1), (+1, -1), (-1, +1)) if faithful else ((-1, -1), (+1, -1), (-1, +1), (+1, +1))
    return tuple((cx + sx * dx, cy + sy * dy) for sx, sy in corners)


def build_obstacles(*, ramps: bool = True, faithful_trenches: bool = True) -> tuple[Obstacle, ...]:
    """Everything solid on the field, in the order maple-sim adds it.

    `ramps` follows `Arena2026Rebuilt(AddRampCollider)`; the default
    matches `SimulatedArena.getInstance()`, which is what the robot
    project uses.
    """
    hub_w, hub_d = HUB_SIZE_IN if ramps else HUB_SIZE_WITHOUT_RAMPS_IN
    obstacles = [
        Obstacle(name=f"{alliance} HUB", vertices=_rect(hub_centre(alliance), hub_w, hub_d))
        for alliance in ("blue", "red")
    ]
    obstacles += [
        Obstacle(name=f"TRENCH WALL {i + 1}", vertices=_rect(centre, *TRENCH_SIZE_IN))
        for i, centre in enumerate(trench_wall_centres(faithful=faithful_trenches))
    ]
    obstacles += [
        Obstacle(name=f"TOWER POLE {i + 1}", vertices=_rect(centre, *TOWER_POLE_SIZE_IN))
        for i, centre in enumerate(TOWER_POLES_IN)
    ]
    return tuple(obstacles)


# Scoring.java's declared range, in metres from the turret to the target.
# `ShotCalculation` compares the lookahead distance against these and
# publishes the answer as `ShotCalculation/isValid` -- and **nothing
# consumes it**. `Turret.periodic` gates the spindexer on turret yaw
# alone, so the robot shoots outside this range too, and the
# `InterpolatingDoubleTreeMap`s clamp to their end entries when it does.
#
# Transcribed for the record, and **not** the range this file enforces.
# What actually scores is much narrower and is not derivable from this
# table; see `SCORING_RANGE_M`.
SHOT_MIN_DISTANCE_M = 1.5
SHOT_MAX_DISTANCE_M = 6.7

# How far from the HUB centre a shot actually lands in the goal, in
# metres. **Measured, not derived, and three separate forward models got
# it wrong before anyone measured it.**
#
# The reason models fail here is that the scoring geometry is not what
# the source appears to say. Two independent things happen to a FUEL
# projectile near the HUB:
#
# * `Goal.simulationSubTick` scores it -- using `checkValidity` ->
#   `positionChecker`, which the `Goal` constructor set to
#   `box(xyBox, position.getZ(), position.getZ() + height)`. RebuiltHub
#   passes 47 x 47 inches and a height of 10 inches, so the target is a
#   flat pad: +/-0.597 m in x and y, and z from 1.5748 to 1.8288.
#   `RebuiltHub.checkCollision`, the 3D sphere test that reads like the
#   scoring rule and that every model here was built on, is overridden
#   and **never called by anything**.
# * `GamePieceProjectile.launch()` walks the arc on a 0.02 s grid and
#   latches `calculatedHitTargetTime` at the first step within the
#   +/-0.5 m `withTargetTolerance` box. From then on
#   `updateGamePieceProjectiles` *removes the piece* and runs
#   `hitTargetCallBack` -- which `TurretIOSim` never sets. The ball is
#   deleted whether it scored or not.
#
# So a ball has to be inside a 254 mm-tall pad during the 97 mm of travel
# between entering the goal box and entering the delete box. That is thin
# enough that the outcome turns on details a closed-form model does not
# have, which is why this is a measurement.
#
# The measurement, on 60-second live runs: **22 of 24 at 2.04 m**, and
# 2 of 42, 6 of 44 and 6 of 42 at 2.50-2.55 m. Somewhere between those
# two the shot stops arriving, and nobody has swept it to find out where.
#
# So this is deliberately conservative rather than a survey. The near
# edge is the old pocket's, which is the robot's own bumpers against the
# HUB. The far edge sits just past the only distance measured to work and
# well short of the nearest one measured to fail -- which matters more
# than it looks, because the region's centroid is the pose `Score` and
# `Stage` drive to, so **the far edge sets the distance the robot
# actually shoots from**. Widening it is not free: it moves the aim point
# outward, and 2.53 m is where scoring collapsed.
#
# Confirmed live at the resulting aim point, 1.80 m: **42 of 48**, on the
# same test that gave 2 of 42 with no range term -- and with no
# `robot-pinned` finding, because the y freedom the region keeps is what
# stops the robot driving into the pinch. At match length the same holds:
# a campaign on the two seeds that had reported `robot-pinned` 4 matches
# out of 4 came back with none.
#
# **Not swept, and deliberately not going to be.** REBUILT is the game
# this bridge was built *against*, not the game it is for: the point of
# the exercise is to have the machinery working before the 2027 reveal,
# and a precise scoring band for a hypothetical season buys nothing that
# survives it. What has to be right is that the strategy layer can score
# at all, so the rest of the tool can be exercised -- and 42 of 48 is
# that. If a future run needs the shot to be sharper, sweep it then; the
# two data points above and this comment are what a sweep would start
# from.
SCORING_RANGE_M = (1.05, 2.30)

# Half a 30-inch frame plus bumpers. A scoring region is tested against
# the robot's *centre* (`world_view.region_occupants` does a
# point-in-polygon on the pose), so a region that reaches a wall or the
# HUB describes robot positions half inside solid structure.
ROBOT_STANDOFF_IN = 18.0


def can_score_from(x_in: float, y_in: float, alliance: str) -> bool:
    """Whether a shot taken from here would go in.

    Two terms, and they answer different questions.

    **The alliance zone** decides whether the press is a score or a
    *pass*. `Turret.setTarget` branches on
    `RobotContainer.isInAllianceArea` and on nothing else -- inside, the
    turret aims at the HUB; outside, it aims at a corner of your own zone
    and throws the fuel back. This is the term `Pass` reads.

    **The range** decides whether the shot that is aimed at the HUB
    arrives in the goal. It is a radius from the HUB centre and not a
    slab in x, because the turret rotates: standing off the HUB's axis in
    y costs nothing, and the region this replaced said otherwise about
    almost the whole field width. See `SCORING_RANGE_M` for why the
    numbers are measured rather than computed.
    """
    if not in_alliance_zone(x_in, alliance):
        return False
    near, far = SCORING_RANGE_M
    distance = math.dist((x_in, y_in), hub_centre(alliance))
    return _in(near) <= distance <= _in(far)


def build_scoring_regions() -> tuple[ScoringRegion, ...]:
    """One region per HUB: the arc of floor inside the alliance zone from
    which a shot lands in the goal.

    A lens, not a pocket, and the difference is the y axis. This started
    as an 82 x 47 inch rectangle behind the HUB, and the 47 inches came
    from the goal mouth's *width* -- which says nothing about where a
    robot stands, because the turret rotates. Being off the HUB's axis in
    y costs nothing at all, and a region sized to the mouth threw away
    almost the whole field width.

    The cost of that was not cosmetic. `Score` navigates at a region and
    stops the moment `deposit_region_for` says the pose is legal; a
    region far smaller than the legal area is a robot that drives past
    perfectly good scoring positions to reach a nominated one -- here,
    into the 50-inch pinch between the HUB ramp and the field wall, which
    is where the campaign's recurring `robot-pinned` finding lives.

    The *depth* is a different story and is kept: it is the one part of
    the old pocket that measurement supports. Widening the region to the
    whole alliance zone was tried, and scoring fell from 22-of-24 to
    2-of-42 because the robot started shooting from half a metre further
    out. See `SCORING_RANGE_M`.

    The polygon is a navigation aid -- what `Score` and `Stage` aim at,
    and what `region_occupants` shares between robots. The *rule* is
    `can_score_from`. Keeping the two apart is what lets the polygon be
    the conservative inset below without lying: a robot in the 18-inch
    band beside the HUB is outside the polygon and still, correctly,
    ready to score.

    Named GOAL rather than HUB because the obstacle is already called
    HUB, and the field validator is right that a name shared by a
    structure and a scoring region is ambiguous -- `deposit_region_for`
    and every bench resolve features by name and would take whichever
    was declared first.

    `capacity_by_action` is left unset: the HUB has no capacity, and
    whether it is *accepting* right now is the 25-second clock, which
    changes during the match and is read live rather than declared here.
    """
    regions = []
    near_m, far_m = SCORING_RANGE_M
    for alliance in ("blue", "red"):
        # Away from midfield: blue's own side is -x, red's is +x.
        outward = -1.0 if alliance == "blue" else 1.0
        hub_x, hub_y = hub_centre(alliance)
        far = _in(far_m)
        # The near edge is not the shot's minimum range but the robot's
        # own bumpers: the HUB is 47 inches across, so a robot cannot get
        # closer than this whatever the flywheel could manage. The
        # minimum range is left to `can_score_from`, where a robot that
        # has somehow got closer still reads correctly.
        chord = HUB_SIZE_IN[0] / 2.0 + ROBOT_STANDOFF_IN
        assert chord < far, f"the {alliance} HUB is wider than the shot's reach"

        # A circular segment: the arc at the far edge of the scoring
        # range, closed by the chord along the face of the HUB. Every
        # point on the arc is the same distance from the goal, which is
        # the whole reason to describe the region as a radius -- a
        # rectangle would have to shrink to the arc's narrowest point and
        # hand back most of the y freedom this exists to win.
        limit = math.acos(chord / far)
        steps = 12
        vertices = [
            (hub_x + outward * far * math.cos(limit * (-1.0 + 2.0 * i / steps)),
             hub_y + far * math.sin(limit * (-1.0 + 2.0 * i / steps)))
            for i in range(steps + 1)
        ]

        # Clip to floor a robot can stand on: inside the field walls, and
        # inside the alliance zone the shot has to be taken from.
        bound = _in(ALLIANCE_ZONE_BOUND_M[alliance])
        clipped = []
        for x, y in vertices:
            x = min(x, bound) if alliance == "blue" else max(x, bound)
            x = min(max(x, ROBOT_STANDOFF_IN), FIELD_LENGTH - ROBOT_STANDOFF_IN)
            y = min(max(y, ROBOT_STANDOFF_IN), FIELD_WIDTH - ROBOT_STANDOFF_IN)
            if not clipped or (x, y) != clipped[-1]:
                clipped.append((x, y))

        region = ScoringRegion(
            name=f"{alliance} GOAL",
            vertices=tuple(clipped),
            actions=frozenset({"shoot"}),
            piece_types=frozenset({PIECE_TYPE}),
            alliance=alliance,
        )
        assert all(in_alliance_zone(x, alliance) for x, _ in region.vertices), (
            f"the {alliance} GOAL region reaches outside the {alliance} alliance zone, so a "
            "robot sent there would pass the fuel instead of scoring it"
        )
        regions.append(region)
    return tuple(regions)


def build_arena(*, ramps: bool = True, faithful_trenches: bool = True) -> FieldConfig:
    """The arena as a `FieldConfig`, for the navigator and the field validator.

    No `intake_locations`: REBUILT has nothing a robot loads from on
    demand. Every piece of fuel is loose on the floor, including what the
    OUTPOSTs throw back in, so possession comes from driving over it and
    the live `FieldSimulation/Fuel` array says where it is.

    No `pin_rule` and no `protected_zones`: maple-sim adjudicates no
    fouls, so declaring rules here would have the strategy layer avoid
    contact that carries no penalty in the simulation it is driving.
    """
    return FieldConfig(
        width=FIELD_LENGTH,
        height=FIELD_WIDTH,
        obstacles=build_obstacles(ramps=ramps, faithful_trenches=faithful_trenches),
        scoring_regions=build_scoring_regions(),
    )
