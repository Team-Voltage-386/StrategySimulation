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
GOAL_RADIUS_M = 0.5969

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
    """The x of the HUB's goal mouth -- the side a robot shoots at.

    Both HUBs open toward midfield: RebuiltHub puts the blue shoot poses
    at +GoalRadius in x from the blue hub centre, and mirrors them for
    red. Approaching from behind is approaching a wall.
    """
    x = _in(HUB_CENTRE_M[alliance][0])
    return x + _in(GOAL_RADIUS_M) if alliance == "blue" else x - _in(GOAL_RADIUS_M)


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


# How far out from the goal mouth the shooting zone reaches. This is the
# one number in this file that maple-sim does not decide, because it is
# not a property of the field: it is how far away the *robot* can score
# from, which depends on its flywheel and hood. 100 inches is a guess
# chosen to be a plausible standoff rather than a measured range, and it
# only has to be roughly right -- it decides where the operator drives to
# shoot, and being wrong makes for a worse shot, not a wrong field.
#
# Measure it and replace it. Until then, treat any conclusion that turns
# on this number as untested.
SHOOTING_STANDOFF_IN = 100.0

# The near edge. A scoring region is tested against the robot's *centre*
# (`world_view.region_occupants` does a point-in-polygon on the pose), so
# a region that reaches the goal face describes robot positions half
# inside the HUB. Half of a 30-inch frame plus bumpers.
GOAL_MIN_STANDOFF_IN = 18.0


def build_scoring_regions() -> tuple[ScoringRegion, ...]:
    """One region per HUB: the band of floor in front of its goal mouth.

    Named GOAL rather than HUB because the obstacle is already called
    HUB, and the field validator is right that a name shared by a
    structure and a scoring region is ambiguous -- `deposit_region_for`
    and every bench resolve features by name and would take whichever
    was declared first.

    Deliberately narrow in y -- the mouth is about 47 inches across while
    the HUB collider is 217 -- because a robot parked beside the ramp is
    not looking at the goal, and a region spanning the whole structure
    would send it there.

    `capacity_by_action` is left unset: the HUB has no capacity, and
    whether it is *accepting* right now is the 25-second clock, which
    changes during the match and is read live rather than declared here.
    """
    regions = []
    for alliance in ("blue", "red"):
        sign = 1.0 if alliance == "blue" else -1.0
        near = goal_face_x(alliance) + sign * GOAL_MIN_STANDOFF_IN
        far = goal_face_x(alliance) + sign * SHOOTING_STANDOFF_IN
        cy = _in(HUB_CENTRE_M[alliance][1])
        half_mouth = _in(GOAL_RADIUS_M)
        regions.append(ScoringRegion(
            name=f"{alliance} GOAL",
            vertices=(
                (near, cy - half_mouth), (far, cy - half_mouth),
                (far, cy + half_mouth), (near, cy + half_mouth),
            ),
            actions=frozenset({"shoot"}),
            piece_types=frozenset({PIECE_TYPE}),
            alliance=alliance,
        ))
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
