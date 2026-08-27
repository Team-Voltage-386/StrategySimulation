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
# Transcribed anyway, because they are the robot code's own statement of
# where it believes it can score, and because the check at the end of
# `build_scoring_regions` uses the upper one to establish that the
# scoring rule below is entitled to leave the range term out.
SHOT_MIN_DISTANCE_M = 1.5
SHOT_MAX_DISTANCE_M = 6.7

# Half a 30-inch frame plus bumpers. A scoring region is tested against
# the robot's *centre* (`world_view.region_occupants` does a
# point-in-polygon on the pose), so a region that reaches a wall or the
# HUB describes robot positions half inside solid structure.
ROBOT_STANDOFF_IN = 18.0


def can_score_from(x_in: float, y_in: float, alliance: str) -> bool:
    """Whether a shot taken from here would be *aimed at the HUB*.

    This is the whole scoring rule, and it is one term: are you inside
    your own alliance zone. `Turret.setTarget` branches on
    `RobotContainer.isInAllianceArea` and on nothing else -- inside, the
    turret aims at the HUB and the shot is a scoring attempt; outside, it
    aims at a corner of your own zone and the identical press is a
    *pass*.

    `y_in` is unused and is in the signature anyway, because the omission
    is the interesting part rather than an oversight: the rule is a
    half-plane in x, so a robot pressed against its own alliance wall in
    the far corner is as much in scoring position as one parked beside
    the HUB.

    **What is deliberately not here is a range term.** The shot table
    declares 1.5-6.7 m, and leaving it out is a measurement rather than
    an assumption:

    * The far corner of an alliance zone is 6.12 m from its HUB, so the
      upper bound cannot bind anywhere a robot could score from at all.
      `build_scoring_regions` checks precisely that, so this paragraph
      cannot quietly go stale.
    * The lower bound can be crossed -- a robot with its bumpers on the
      HUB's back face is 1.05 m from the centre -- but nothing enforces
      it, and solving the maple-sim ballistics against `RebuiltHub`'s
      goal sphere puts the >=95% hit band at 0.8-7.0 m once
      `TurretIOSim`'s launch speed is right. The declared minimum is
      conservative against what the simulated shot actually does.

    That second point is a fact about *this simulation*, whose shooter is
    forgiving -- no air drag, and a couple of degrees of aim noise. It is
    not a claim about the real robot, and anything that starts judging
    the *shot* rather than the navigation should read the shot caveat in
    bridge/README.md first.
    """
    del y_in  # in the signature on purpose; see above
    return in_alliance_zone(x_in, alliance)


def build_scoring_regions() -> tuple[ScoringRegion, ...]:
    """One region per HUB: **the alliance zone**, less the floor a robot
    cannot stand on.

    The zone, not a pocket beside the goal. This started as an 82 x 47
    inch rectangle behind the HUB, sized to the goal mouth and a guessed
    shooting standoff, and that was two mistakes stacked:

    * The goal mouth's *width* has nothing to say about where a robot
      stands. The turret rotates, so being off the HUB's axis in y costs
      nothing whatever, and a 47-inch-tall region claimed otherwise.
    * The standoff in x was a guess at flywheel range, and measurement
      later showed range was never the binding constraint at all (see
      `can_score_from`).

    So the region was a small arbitrary subset of the true one, and the
    cost of that was not cosmetic. `Score` navigates at a region and
    stops the moment `deposit_region_for` says the pose is legal; a
    region a quarter the size of the legal area is a robot that drives
    past perfectly good scoring positions to reach a nominated one --
    here, straight into the 50-inch pinch between the HUB ramp and the
    field wall, which is where the campaign's recurring `robot-pinned`
    finding lives.

    The polygon is a *navigation aid*: it is what `Score` and `Stage` aim
    at, and what `region_occupants` shares out between robots. The
    *rule* is `can_score_from`, which `MapleMatchView.deposit_region_for`
    applies directly and which has no polygon in it. Keeping those two
    jobs apart is what lets the polygon be the conservative inset below
    without lying -- a robot in the 18-inch band beside the HUB is
    outside the polygon and still, correctly, ready to score.

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
    for alliance in ("blue", "red"):
        # Away from midfield: blue's own side is -x, red's is +x.
        outward = -1.0 if alliance == "blue" else 1.0
        hub_x = _in(HUB_CENTRE_M[alliance][0])
        wall_x = 0.0 if alliance == "blue" else FIELD_LENGTH

        # The zone boundary itself would be the near edge, except that
        # the HUB straddles it -- blue's bound (182.1 in) lies 1.1 inches
        # *past* the blue hub centre, so the last two feet of the zone
        # are inside the structure. Stop at the face a robot can reach.
        near = hub_x + outward * (HUB_SIZE_IN[0] / 2.0 + ROBOT_STANDOFF_IN)
        far = wall_x - outward * ROBOT_STANDOFF_IN
        lo_x, hi_x = min(near, far), max(near, far)
        lo_y, hi_y = ROBOT_STANDOFF_IN, FIELD_WIDTH - ROBOT_STANDOFF_IN

        region = ScoringRegion(
            name=f"{alliance} GOAL",
            vertices=((lo_x, lo_y), (hi_x, lo_y), (hi_x, hi_y), (lo_x, hi_y)),
            actions=frozenset({"shoot"}),
            piece_types=frozenset({PIECE_TYPE}),
            alliance=alliance,
        )
        assert all(in_alliance_zone(x, alliance) for x, _ in region.vertices), (
            f"the {alliance} GOAL region reaches outside the {alliance} alliance zone, so a "
            "robot sent there would pass the fuel instead of scoring it"
        )
        regions.append(region)

    # The range term `can_score_from` leaves out, checked rather than
    # argued in prose. Every corner of an alliance *zone* -- not of the
    # inset polygon above, which is only a navigation aid -- has to lie
    # inside the shot table's declared reach, or "you are in the zone"
    # stops being the whole scoring rule and this file owes its callers a
    # distance test.
    for alliance in ("blue", "red"):
        hub = HUB_CENTRE_M[alliance]
        wall_m = 0.0 if alliance == "blue" else FIELD_LENGTH_M
        reach = max(
            math.dist(hub, (x, y))
            for x in (wall_m, ALLIANCE_ZONE_BOUND_M[alliance])
            for y in (0.0, FIELD_WIDTH_M)
        )
        assert reach <= SHOT_MAX_DISTANCE_M, (
            f"the {alliance} zone reaches {reach:.2f} m from its HUB, past the shot table's "
            f"{SHOT_MAX_DISTANCE_M} m -- `can_score_from` now needs a range term"
        )
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
