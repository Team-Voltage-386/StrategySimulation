"""
SALVAGE 2027 -- an *invented* FRC-shaped game, written as a dry run for
the two-day new-game turnaround that `game_specific/` exists to support.

There is no such competition. The point is that the framework has only
ever run one game, so every "game-agnostic" claim in ARCHITECTURE.md is
currently untested against a second one. This field is therefore chosen
to be awkward in the specific ways REEFSCAPE is not:

  * **Seven obstacles, not two.** The visibility-graph navigator (and
    all of the perf work built on it) has only ever seen a field with
    two convex hexagons in the middle. Here there are two long hull
    barriers splitting the field into three lanes, a hex depot shell at
    the centre, and four smaller structures scattered asymmetrically.
  * **Three piece types, sourced three different ways.** See
    game_pieces.py -- one from your own stations, one *only* from a
    finite neutral depot, one that only ever arrives on the floor.
  * **A finite, contested supply.** The SALVAGE DEPOT holds
    `DEPOT_CELLS_PER_SIDE` CELLs per side and belongs to nobody. It runs
    out, and the alliance that took them is the reason. REEFSCAPE's
    CORAL stations never run dry, which is why Pursue's scarcity
    reasoning has never been exercised.
  * **Neutral scoring capacity.** The BEACON accepts CRATEs from either
    alliance into one shared, capped set of slots. Filling it denies it.
  * **Value that moves with the phase, and not monotonically.** See
    scoring.py.
  * **Scoring targets that are in different places.** REEFSCAPE's L1-L4
    are the same physical spot, so choosing a level costs no travel.
    Here the cheap hold is by the wall and the valuable one is deep
    downfield, which is the case points-per-second was written for.
  * **A half that is not mirror-symmetric with itself.** Each alliance's
    two CARGO BAYs sit at different distances from its own structures.
    The field as a whole is 180-degree rotationally symmetric about its
    centre -- like a real FRC field, and unlike a left-right mirror --
    so a head-to-head is still fair.

Dimensions are in inches; origin is the blue ALLIANCE WALL's bottom-left
corner, +x toward the red wall.
"""
from __future__ import annotations

import math

from common_sim.field.field_config import (
    EmitterRegion, FieldConfig, IntakeLocation, Obstacle, PinRule, ProtectedZone, ScoringRegion,
)
from game_specific.salvage.game_pieces import CELL_COLOR, CELL_TYPE, CRATE_COLOR, CRATE_TYPE, SCRAP_TYPE

FIELD_LENGTH = 600.0
FIELD_WIDTH = 320.0

AUTO_DURATION = 15.0
TELEOP_DURATION = 135.0

# CELLs per depot mouth. Two mouths, so ten on the field for the whole
# match, shared between every robot in it -- small enough that taking
# one is taking it from somebody.
DEPOT_CELLS_PER_SIDE = 5

# CRATEs each CARGO BAY can hand out. Large enough not to be the
# bottleneck (that is the DEPOT's job), finite so the GUI has a number.
CARGO_BAY_CRATES = 40

# SCRAP arrives on the clock: one piece per alliance zone every 8s of
# TELEOP, up to this many.
SCRAP_EMIT_RATE_HZ = 1.0 / 8.0
SCRAP_EMIT_CAPACITY = 8

# G-rule analogues. Deliberately different numbers from REEFSCAPE's, so
# a test that passes on both is not passing on a hardcoded 3.0/6.0.
PIN_MAX_SECONDS = 4.0
PIN_RELEASE_SECONDS = 1.0
PIN_FOUL_POINTS = 5.0
REACTOR_ZONE_MARGIN = 14.0
REACTOR_ZONE_FOUL_POINTS = 2.0

# How many CRATEs the shared BEACON holds, total, across both alliances.
BEACON_CAPACITY = 6
# Per-alliance cap on the deep hold.
HOLD_HIGH_CAPACITY = 6

DEPOT_APOTHEM = 22.0
DEPOT_CENTER = (FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0)


def _rect(center: tuple[float, float], width: float, depth: float) -> tuple[tuple[float, float], ...]:
    hw, hd = width / 2.0, depth / 2.0
    cx, cy = center
    return ((cx - hw, cy - hd), (cx + hw, cy - hd), (cx + hw, cy + hd), (cx - hw, cy + hd))


def _hex(center: tuple[float, float], apothem: float) -> tuple[tuple[float, float], ...]:
    radius = apothem / math.cos(math.radians(30))
    return tuple(
        (center[0] + radius * math.cos(math.radians(30 + 60 * i)),
         center[1] + radius * math.sin(math.radians(30 + 60 * i)))
        for i in range(6)
    )


def rotate(point: tuple[float, float]) -> tuple[float, float]:
    """A point's 180-degree rotation about the field centre -- how red's
    copy of any blue feature is placed. Rotational, not mirrored: a
    mirrored field would let a bug that swaps x for FIELD_LENGTH-x pass
    while leaving y alone, which is exactly the bug a rotationally
    symmetric field catches."""
    return (FIELD_LENGTH - point[0], FIELD_WIDTH - point[1])


def _rotate_all(vertices):
    return tuple(rotate(v) for v in vertices)


def other_alliance(alliance: str) -> str:
    return "red" if alliance == "blue" else "blue"


# -- blue-side feature centres (red's are these, rotated) ---------------

BLUE_HOLD_LOW = (60.0, 160.0)
BLUE_HOLD_HIGH = (220.0, 295.0)
BLUE_REACTOR = (150.0, 160.0)
BLUE_AIRLOCK = (20.0, 160.0)
BLUE_CARGO_BAYS = ((35.0, 35.0), (95.0, 300.0))
BLUE_SCRAP_DROP = (150.0, 230.0)
BLUE_START_LINE_X = 45.0

HOLD_LOW_SIZE = (40.0, 90.0)
HOLD_HIGH_SIZE = (56.0, 36.0)
REACTOR_SIZE = (44.0, 44.0)
AIRLOCK_SIZE = (40.0, 200.0)
CARGO_BAY_SIZE = (40.0, 40.0)
DEPOT_MOUTH_SIZE = (34.0, 34.0)
BEACON_SIZE = (80.0, 36.0)
SCRAP_DROP_SIZE = (24.0, 24.0)

BEACON_CENTER = (300.0, 300.0)
DEPOT_MOUTHS = ((258.0, 160.0), (342.0, 160.0))


def _side_point(alliance: str, point: tuple[float, float]) -> tuple[float, float]:
    return point if alliance == "blue" else rotate(point)


def hold_low_position(alliance: str) -> tuple[float, float]:
    return _side_point(alliance, BLUE_HOLD_LOW)


def hold_high_position(alliance: str) -> tuple[float, float]:
    return _side_point(alliance, BLUE_HOLD_HIGH)


def reactor_position(alliance: str) -> tuple[float, float]:
    return _side_point(alliance, BLUE_REACTOR)


def cargo_bay_positions(alliance: str) -> tuple[tuple[float, float], ...]:
    """This alliance's two CARGO BAY loading zones. They are *not* a
    mirror pair -- one is tucked in the near corner and the other is
    well downfield along the far guardrail, so "the other station" is
    never the same trip as "this station"."""
    return tuple(_side_point(alliance, p) for p in BLUE_CARGO_BAYS)


def start_pose_positions(alliance: str, count: int) -> tuple[tuple[float, float], ...]:
    """`count` starting spots along this alliance's wall, spread across
    the field width."""
    x = BLUE_START_LINE_X if alliance == "blue" else FIELD_LENGTH - BLUE_START_LINE_X
    return tuple(
        (x, FIELD_WIDTH * (i + 1) / (count + 1))
        for i in range(count)
    )


def scrap_staging_positions(alliance: str) -> tuple[tuple[float, float], ...]:
    """Pre-match SCRAP on the floor in this alliance's zone. Spread
    around the zone rather than clustered, so a collect has to choose."""
    base = ((120.0, 130.0), (185.0, 190.0), (110.0, 205.0), (205.0, 120.0))
    return tuple(_side_point(alliance, p) for p in base)


def crate_staging_positions(alliance: str) -> tuple[tuple[float, float], ...]:
    """Two loose CRATEs per alliance, staged pre-match -- the only
    CRATEs that start on the floor rather than in a CARGO BAY."""
    base = ((85.0, 95.0), (165.0, 215.0))
    return tuple(_side_point(alliance, p) for p in base)


def build_field() -> FieldConfig:
    # -- obstacles ------------------------------------------------------
    # The two hulls split the field into three lanes: a north lane above
    # y=258, a middle lane between the hulls (with the depot shell in the
    # centre of it, so crossing the middle means going round something),
    # and a south lane below y=62. The pylons and ramps then break up
    # each alliance's own zone, so even a same-side trip is not a
    # straight line.
    blue_pylon = _rect((170.0, 265.0), 34.0, 34.0)
    blue_ramp = _rect((140.0, 55.0), 50.0, 28.0)
    obstacles = (
        Obstacle(name="hull_north", vertices=_rect((300.0, 244.0), 140.0, 28.0)),
        Obstacle(name="hull_south", vertices=_rect((300.0, 76.0), 140.0, 28.0)),
        Obstacle(name="depot_shell", vertices=_hex(DEPOT_CENTER, DEPOT_APOTHEM)),
        Obstacle(name="blue_pylon", vertices=blue_pylon),
        Obstacle(name="red_pylon", vertices=_rotate_all(blue_pylon)),
        Obstacle(name="blue_ramp", vertices=blue_ramp),
        Obstacle(name="red_ramp", vertices=_rotate_all(blue_ramp)),
    )

    # -- scoring --------------------------------------------------------
    scoring_regions = tuple(
        region
        for alliance in ("blue", "red")
        for region in (
            ScoringRegion(
                name=f"{alliance}_hold_low",
                vertices=_rect(hold_low_position(alliance), *HOLD_LOW_SIZE),
                actions=frozenset({"hold_low"}), piece_types=frozenset({CRATE_TYPE}), alliance=alliance,
            ),
            ScoringRegion(
                name=f"{alliance}_hold_high",
                vertices=_rect(hold_high_position(alliance), *HOLD_HIGH_SIZE),
                actions=frozenset({"hold_high"}), piece_types=frozenset({CRATE_TYPE}),
                capacity_by_action={"hold_high": HOLD_HIGH_CAPACITY}, alliance=alliance,
            ),
            ScoringRegion(
                name=f"{alliance}_reactor",
                vertices=_rect(reactor_position(alliance), *REACTOR_SIZE),
                actions=frozenset({"reactor"}), piece_types=frozenset({CELL_TYPE}), alliance=alliance,
            ),
            # The lob. A SCRAP that lands here scores with nobody in
            # position, which is what passive_scoring means.
            ScoringRegion(
                name=f"{alliance}_airlock",
                vertices=_rect(_side_point(alliance, BLUE_AIRLOCK), *AIRLOCK_SIZE),
                actions=frozenset({"airlock"}), piece_types=frozenset({SCRAP_TYPE}),
                passive_scoring=True, alliance=alliance,
            ),
        )
    ) + (
        # Neutral, and capped across BOTH alliances -- the only shared
        # scoring capacity on the field. Filling it is simultaneously
        # scoring and denying.
        ScoringRegion(
            name="beacon", vertices=_rect(BEACON_CENTER, *BEACON_SIZE),
            actions=frozenset({"beacon"}), piece_types=frozenset({CRATE_TYPE}),
            capacity_by_action={"beacon": BEACON_CAPACITY}, alliance=None,
        ),
    )

    # -- collection -----------------------------------------------------
    intake_locations = tuple(
        IntakeLocation(
            name=f"{alliance}_cargo_bay_{i}",
            vertices=_rect(pos, *CARGO_BAY_SIZE),
            piece_type=CRATE_TYPE, starting_pieces=CARGO_BAY_CRATES,
            piece_color=CRATE_COLOR, alliance=alliance,
        )
        for alliance in ("blue", "red")
        for i, pos in enumerate(cargo_bay_positions(alliance))
    ) + tuple(
        # Neutral and finite. Nobody owns these; whoever gets there first
        # takes the CELL, and when the count hits zero the REACTOR is
        # simply closed for the rest of the match.
        IntakeLocation(
            name=f"salvage_depot_{side}",
            vertices=_rect(pos, *DEPOT_MOUTH_SIZE),
            piece_type=CELL_TYPE, starting_pieces=DEPOT_CELLS_PER_SIDE,
            piece_color=CELL_COLOR, alliance=None,
        )
        for side, pos in zip(("west", "east"), DEPOT_MOUTHS)
    )

    # -- emitters -------------------------------------------------------
    emitter_regions = tuple(
        EmitterRegion(
            name=f"{alliance}_scrap_drop",
            vertices=_rect(_side_point(alliance, BLUE_SCRAP_DROP), *SCRAP_DROP_SIZE),
            piece_type=SCRAP_TYPE,
            active_times=((AUTO_DURATION, AUTO_DURATION + TELEOP_DURATION),),
            emit_rate_hz=SCRAP_EMIT_RATE_HZ,
            initial_capacity=SCRAP_EMIT_CAPACITY,
            alliance=alliance,
        )
        for alliance in ("blue", "red")
    )

    # -- protection -----------------------------------------------------
    protected_zones = tuple(
        ProtectedZone(
            name=f"{alliance}_reactor_zone",
            vertices=_rect(
                reactor_position(alliance),
                REACTOR_SIZE[0] + 2 * REACTOR_ZONE_MARGIN,
                REACTOR_SIZE[1] + 2 * REACTOR_ZONE_MARGIN,
            ),
            alliance=alliance,
            foul_points=REACTOR_ZONE_FOUL_POINTS,
        )
        for alliance in ("blue", "red")
    )

    return FieldConfig(
        width=FIELD_LENGTH, height=FIELD_WIDTH,
        obstacles=obstacles, scoring_regions=scoring_regions, intake_locations=intake_locations,
        emitter_regions=emitter_regions, protected_zones=protected_zones,
        pin_rule=PinRule(
            max_seconds=PIN_MAX_SECONDS,
            release_seconds=PIN_RELEASE_SECONDS,
            foul_points=PIN_FOUL_POINTS,
        ),
    )
