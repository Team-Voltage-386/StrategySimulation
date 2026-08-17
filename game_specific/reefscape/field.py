"""
2025 FIRST Robotics Competition (REEFSCAPE) field layout, built from the
official Game Manual (V13), Section 5 ARENA. Dimensions are the
manual's nominal values, in inches; field origin is the blue ALLIANCE
WALL's bottom-left corner, +x toward the red ALLIANCE WALL, +y along
the wall.

Not modeled in this first pass (noted here rather than silently
missing): the BARGE/CAGE climb endgame mechanic and the AUTO LEAVE
mobility bonus -- both are position/time-based bonuses rather than
piece-deposit scoring, which is a different mechanic than
common_sim's current ScoringRegion model covers. CORAL (REEF L1-L4)
and ALGAE (PROCESSOR/NET) scoring, which is the core scoring loop and
the point of this dry run, are fully modeled.
"""
from __future__ import annotations

import math

from common_sim.field.field_config import (
    EmitterRegion, FieldConfig, IntakeLocation, Obstacle, PinRule, ProtectedZone, ScoringRegion,
)
from game_specific.reefscape.game_pieces import ALGAE_RADIUS, ALGAE_TYPE, CORAL_TYPE

# Game Manual (V13) doesn't cap CORAL station supply -- 30 is a
# dry-run placeholder (a generous multiple of what one match could plausibly
# consume) so the GUI's remaining-count label has a finite number to show
# and count down, rather than every station reading unlimited forever.
CORAL_STATION_STARTING_PIECES = 30

# Standard match structure this field assumes for phase-gated regions (e.g.
# the coral emitter's active_times below) -- matches the MatchConfig values
# apps/reefscape_widgets.py's build_demo_match constructs by default.
AUTO_DURATION = 15.0
TELEOP_DURATION = 135.0

# One CORAL emitted every 10s, while linked station stock lasts -- see
# build_field's emitter_regions and MatchConfig.emit_coral_to_field.
CORAL_EMITTER_RATE_HZ = 1.0 / 10.0
# Zone size the emitter drops pieces within -- arbitrary but small, since
# it's just a spawn point, not a physical structure a robot interacts with.
CORAL_EMITTER_ZONE_SIZE = 24.0

# -- manual dimensions (inches), Section 5 ARENA -----------------------

FIELD_LENGTH = 57 * 12 + 6 + 7 / 8       # 57 ft 6-7/8 in = 690.875 in (long axis, wall to wall)
FIELD_WIDTH = 26 * 12 + 5                # 26 ft 5 in = 317 in (short axis, guardrail to guardrail)

REEF_DISTANCE_FROM_WALL = 12 * 12        # REEF centered 12 ft from its ALLIANCE WALL
REEF_HEX_WIDTH = 5 * 12 + 5.5            # REEF structure, face-to-face: 5 ft 5-1/2 in
REEF_HEX_APOTHEM = REEF_HEX_WIDTH / 2.0

CORAL_STATION_WIDTH = 5 * 12 + 10 + 7 / 8   # 5 ft 10-7/8 in
CORAL_STATION_DEPTH = 13 * 12 + 8 + 3 / 8   # 13 ft 8-3/8 in

# How far a CORAL STATION loading zone's center sits from the true field
# corner along each wall -- keeps the zone (and an approaching robot)
# clear of the perimeter walls while still reading as "at the corner".
CORAL_STATION_CORNER_MARGIN = 10.0

PROCESSOR_WIDTH = 3 * 12 + 7 + 3 / 8      # 3 ft 7-3/8 in
PROCESSOR_DEPTH = 1 * 12 + 6              # 1 ft 6 in

# 6.3.4 SCORING ELEMENTS: FIELD STAFF pre-stage 1 CORAL (with 1 ALGAE on
# top of it) on each of 3 CORAL MARKs per alliance, within that
# alliance's ALLIANCE AREA -- see coral_mark_positions. Figure 6-2 shows
# exact placement as a diagram only (no tabulated coordinates), so these
# are a dry-run approximation: evenly spread across the field width at a
# fixed distance from the ALLIANCE WALL, comfortably inside the
# ALLIANCE AREA (which runs the full 13 ft 8-3/8 in depth of the CORAL
# STATIONs).
CORAL_MARK_COUNT = 3
CORAL_MARK_WALL_OFFSET = 60.0
# Lateral offset between a CORAL MARK's staged CORAL and its ALGAE --
# the manual stages the ALGAE resting on top of the CORAL, which this
# 2D top-down sim has no "on top of" for; offsetting them sideways
# instead keeps both pieces distinct on spawn rather than overlapping
# and immediately shoving each other via physics.
CORAL_MARK_ALGAE_OFFSET = 12.0

# -- REEF scoring-face geometry -----------------------------------------

REEF_LEVELS = frozenset({"l1", "l2", "l3", "l4"})
# How far a REEF face's scoring zone extends outward from the structure.
# common_sim's generic deposit model ejects a piece forward from the
# chassis center, not from an arm/elevator reaching over the bumper --
# so this needs enough depth for a robot to stop clear of the solid
# REEF (bumper not touching it) while its center still lands inside the
# zone once it does. 24in comfortably clears a robot with the ~14in
# chassis half-length used elsewhere in this sim plus a few inches of
# standoff; a real robot's actual reach mechanism doesn't affect this,
# since scoring is timing/points-based here, not mechanism geometry.
REEF_FACE_SCORING_DEPTH = 10.0
REEF_FACE_SCORING_WIDTH = 22.0   # matches roughly one REEF face's width

# REEF ZONE: the area within REEF_ZONE_MARGIN of the REEF, inside which
# an opponent may not contact a robot of the REEF's own alliance. Since
# the REEF is a regular hexagon, the zone is the same hexagon grown by
# that margin on its apothem -- every face offset outward by exactly
# REEF_ZONE_MARGIN, which is what "12 inches from the REEF on all sides"
# describes.
#
# REEF_ZONE_FOUL_POINTS is one FOUL, the 2025 value. It is the price of
# a single call, not of the whole shove: a defender that leans on a
# protected robot is charged again every ProtectedZone.foul_period.
REEF_ZONE_MARGIN = 12.0
REEF_ZONE_FOUL_POINTS = 2.0

# G211 in 2025: a robot may not pin an opponent for more than three
# seconds, and must move away before the count restarts. The 2025 answer
# is a TECH FOUL, worth 6 points to the pinned robot's alliance.
PIN_MAX_SECONDS = 3.0
PIN_RELEASE_SECONDS = 1.0
PIN_FOUL_POINTS = 6.0

# L2-L4 are individual branches -- one CORAL each, physically. L1 is a
# trough that holds several; the Game Manual (V13) doesn't give an exact
# count, so this is a dry-run placeholder (generous enough that L1 rarely
# becomes the strategy bottleneck, matching how it plays in practice).
REEF_BRANCH_CAPACITY = 1
# Each REEF face has 2 branches (A/B) at every one of L2-L4 -- a single
# ScoringRegion here models the whole face (not one branch, since both
# branches sit in the same physical zone), so its per-level capacity
# needs to be both branches' worth, not one. Getting this wrong makes a
# face's L2/L3/L4 register "full" after only 1 CORAL each, well before
# the other physical branch is actually occupied.
REEF_BRANCHES_PER_LEVEL = 2
REEF_TROUGH_CAPACITY = 6
REEF_LEVEL_CAPACITY = {
    "l1": REEF_TROUGH_CAPACITY,
    "l2": REEF_BRANCHES_PER_LEVEL * REEF_BRANCH_CAPACITY,
    "l3": REEF_BRANCHES_PER_LEVEL * REEF_BRANCH_CAPACITY,
    "l4": REEF_BRANCHES_PER_LEVEL * REEF_BRANCH_CAPACITY,
}

# 6.3.4.2.A: 1 ALGAE is staged lightly on the REEF at each of its 6
# faces before the MATCH (12 total field-wide, alternating L2/L3 branch
# height per Figure 6-3 -- a height distinction this sim doesn't model,
# since REEF face scoring is already flattened to one zone per face, see
# _reef_scoring_regions). Modeled as a per-face collection region (an
# IntakeLocation with starting_pieces=1) rather than a physical loose
# piece resting against the hex obstacle, matching how a CORAL STATION
# is modeled -- a robot dwells in the zone to pick the ALGAE up. The
# staging point sits just outside the REEF's solid hex face so it
# doesn't spawn/collide inside the obstacle.
REEF_ALGAE_STAGING_CLEARANCE = ALGAE_RADIUS + 4.0


def _hex_vertices(center: tuple[float, float], apothem: float) -> tuple[tuple[float, float], ...]:
    # Vertices at 30/90/.../330 deg (not 0/60/.../300) so that faces --
    # bisecting consecutive vertices -- land at 0/60/.../300 deg, giving
    # a face that points due -x/+x/etc. rather than a vertex. A vertex
    # sitting at exactly 180 deg (due -x) would put a sharp point, not a
    # flat scoring face, in the path of a robot approaching straight on.
    radius = apothem / math.cos(math.radians(30))
    return tuple(
        (center[0] + radius * math.cos(math.radians(30 + 60 * i)), center[1] + radius * math.sin(math.radians(30 + 60 * i)))
        for i in range(6)
    )


def _hex_face_centers_and_normals(center: tuple[float, float], apothem: float):
    """6 (face_midpoint, outward_normal) pairs, bisecting the vertices
    _hex_vertices produces (which sit at 30/90/.../330 deg) -- so face
    midpoints land at 0/60/.../300 deg, i.e. one face's normal is
    exactly (-1, 0) (due -x) and its opposite is exactly (1, 0)."""
    for i in range(6):
        angle = math.radians(60 * i)
        normal = (math.cos(angle), math.sin(angle))
        midpoint = (center[0] + normal[0] * apothem, center[1] + normal[1] * apothem)
        yield midpoint, normal


def _reef_scoring_regions(name_prefix: str, center: tuple[float, float], alliance: str) -> tuple[ScoringRegion, ...]:
    """One ScoringRegion per REEF face (6 total), each offering all 4
    CORAL levels -- L1-L4 differ in point value and (via a robot's
    RobotCharacteristics.deposit_time_by_action) how long a robot takes
    to reach that level, not in field position, matching how the real
    REEF has all 4 levels stacked on every face."""
    regions = []
    for i, (face_center, normal) in enumerate(_hex_face_centers_and_normals(center, REEF_HEX_APOTHEM)):
        tangent = (-normal[1], normal[0])
        half_w = REEF_FACE_SCORING_WIDTH / 2.0
        inner = face_center
        outer = (face_center[0] + normal[0] * REEF_FACE_SCORING_DEPTH, face_center[1] + normal[1] * REEF_FACE_SCORING_DEPTH)
        vertices = (
            (inner[0] + tangent[0] * half_w, inner[1] + tangent[1] * half_w),
            (inner[0] - tangent[0] * half_w, inner[1] - tangent[1] * half_w),
            (outer[0] - tangent[0] * half_w, outer[1] - tangent[1] * half_w),
            (outer[0] + tangent[0] * half_w, outer[1] + tangent[1] * half_w),
        )
        regions.append(ScoringRegion(
            name=f"{name_prefix}_face_{i}", vertices=vertices, actions=REEF_LEVELS, piece_types=frozenset({CORAL_TYPE}),
            capacity_by_action=dict(REEF_LEVEL_CAPACITY), alliance=alliance,
        ))
    return tuple(regions)


def reef_center(alliance: str) -> tuple[float, float]:
    x = REEF_DISTANCE_FROM_WALL if alliance == "blue" else FIELD_LENGTH - REEF_DISTANCE_FROM_WALL
    return (x, FIELD_WIDTH / 2.0)


def coral_station_positions(alliance: str) -> tuple[tuple[float, float], tuple[float, float]]:
    """Center of each of the 2 CORAL STATION loading zones for this
    alliance -- one at each of its two field corners (the real CORAL
    STATION openings are cut into the field's corners, not set back
    along the ALLIANCE WALL). Blue owns the x=0 corners, red the
    x=FIELD_LENGTH corners; CORAL_STATION_CORNER_MARGIN keeps each zone's
    center a bit clear of the perimeter walls rather than flush with
    them."""
    x = CORAL_STATION_CORNER_MARGIN if alliance == "blue" else FIELD_LENGTH - CORAL_STATION_CORNER_MARGIN
    return ((x, CORAL_STATION_CORNER_MARGIN), (x, FIELD_WIDTH - CORAL_STATION_CORNER_MARGIN))


def coral_mark_positions(alliance: str) -> tuple[tuple[float, float], ...]:
    """This alliance's 3 pre-match CORAL MARK positions (see
    CORAL_MARK_COUNT), evenly spread across the field width at a fixed
    distance from the ALLIANCE WALL."""
    x = CORAL_MARK_WALL_OFFSET if alliance == "blue" else FIELD_LENGTH - CORAL_MARK_WALL_OFFSET
    return tuple(
        (x, FIELD_WIDTH * (i + 1) / (CORAL_MARK_COUNT + 1))
        for i in range(CORAL_MARK_COUNT)
    )


def reef_algae_staging_positions(alliance: str) -> tuple[tuple[float, float], ...]:
    """This alliance's 6 pre-match REEF ALGAE staging points, one per
    REEF face, just outside the hex structure (see
    REEF_ALGAE_STAGING_CLEARANCE)."""
    center = reef_center(alliance)
    return tuple(
        (mid[0] + normal[0] * REEF_ALGAE_STAGING_CLEARANCE, mid[1] + normal[1] * REEF_ALGAE_STAGING_CLEARANCE)
        for mid, normal in _hex_face_centers_and_normals(center, REEF_HEX_APOTHEM)
    )


def alliance_zone_center(alliance: str) -> tuple[float, float]:
    """Midpoint between this alliance's ALLIANCE WALL and its REEF --
    roughly the middle of the open zone the alliance operates in, where
    the CORAL emitter (build_field's emitter_regions) sits so a human
    player lobbing pieces in doesn't need to reach either the wall or the
    REEF specifically."""
    reef_x, _ = reef_center(alliance)
    wall_x = 0.0 if alliance == "blue" else FIELD_LENGTH
    return ((wall_x + reef_x) / 2.0, FIELD_WIDTH / 2.0)


def processor_position(alliance: str) -> tuple[float, float]:
    """Approximate PROCESSOR opening center, integrated into the
    guardrail near this alliance's REEF."""
    reef_x, _ = reef_center(alliance)
    return (reef_x, PROCESSOR_DEPTH / 2.0)


def other_alliance(alliance: str) -> str:
    return "red" if alliance == "blue" else "blue"


def build_field() -> FieldConfig:
    blue_reef_center = reef_center("blue")
    red_reef_center = reef_center("red")

    obstacles = (
        Obstacle(name="blue_reef", vertices=_hex_vertices(blue_reef_center, REEF_HEX_APOTHEM)),
        Obstacle(name="red_reef", vertices=_hex_vertices(red_reef_center, REEF_HEX_APOTHEM)),
    )

    # Each alliance's REEF ZONE protects *that* alliance's robots: blue
    # may be blocked on its way to the blue REEF, but not bumped once it
    # gets there. Note the zone swallows the REEF's own scoring regions
    # (10in deep, well inside the 12in margin), which is the point --
    # arriving at a face is arriving somewhere safe.
    protected_zones = tuple(
        ProtectedZone(
            name=f"{alliance}_reef_zone",
            vertices=_hex_vertices(reef_center(alliance), REEF_HEX_APOTHEM + REEF_ZONE_MARGIN),
            alliance=alliance,
            foul_points=REEF_ZONE_FOUL_POINTS,
        )
        for alliance in ("blue", "red")
    )

    scoring_regions = (
        _reef_scoring_regions("blue_reef", blue_reef_center, "blue")
        + _reef_scoring_regions("red_reef", red_reef_center, "red")
        + (
            # PROCESSOR (and REEF above) require a robot to actually place
            # the piece -- passive_scoring defaults False, so a piece that
            # merely rolls/bounces in without a robot engaged there can't
            # score. NET below is the one REEFSCAPE exception: ALGAE is
            # scored by launching it from a distance, so it only needs to
            # land in the zone, with no robot present at landing time.
            ScoringRegion(
                name="blue_processor",
                vertices=_rect(processor_position("blue"), PROCESSOR_WIDTH, PROCESSOR_DEPTH),
                actions=frozenset({"processor"}), piece_types=frozenset({ALGAE_TYPE}), alliance="blue",
            ),
            ScoringRegion(
                name="red_processor",
                vertices=_rect(processor_position("red"), PROCESSOR_WIDTH, PROCESSOR_DEPTH),
                actions=frozenset({"processor"}), piece_types=frozenset({ALGAE_TYPE}), alliance="red",
            ),
            ScoringRegion(
                name="blue_net",
                vertices=_rect((FIELD_LENGTH / 2.0 - 60, FIELD_WIDTH / 2.0), 80, FIELD_WIDTH * 0.9),
                actions=frozenset({"net"}), piece_types=frozenset({ALGAE_TYPE}), passive_scoring=True, alliance="blue",
            ),
            ScoringRegion(
                name="red_net",
                vertices=_rect((FIELD_LENGTH / 2.0 + 60, FIELD_WIDTH / 2.0), 80, FIELD_WIDTH * 0.9),
                actions=frozenset({"net"}), piece_types=frozenset({ALGAE_TYPE}), passive_scoring=True, alliance="red",
            ),
        )
    )

    intake_locations = tuple(
        IntakeLocation(
            name=f"{alliance}_coral_station_{i}",
            vertices=_rect(pos, 36.0, 36.0),
            piece_type=CORAL_TYPE,
            starting_pieces=CORAL_STATION_STARTING_PIECES,
            piece_color="white",  # matches spawn_coral's field-pile color
            alliance=alliance,
        )
        for alliance in ("blue", "red")
        for i, pos in enumerate(coral_station_positions(alliance))
    ) + tuple(
        IntakeLocation(
            name=f"{alliance}_reef_algae_{i}",
            vertices=_rect(pos, 20.0, 20.0),
            piece_type=ALGAE_TYPE,
            starting_pieces=1,
            piece_color="green",  # matches spawn_algae's field-pile color
            alliance=alliance,
        )
        for alliance in ("blue", "red")
        for i, pos in enumerate(reef_algae_staging_positions(alliance))
    )

    # One CORAL emitter per alliance, sharing its pool with that alliance's
    # "top" (higher-y) CORAL STATION -- linked_collection_region means
    # emitting draws down the same station_supply count a robot loading
    # from that station would, rather than a separate stock. Only actually
    # spawns pieces when MatchConfig.emit_coral_to_field is True.
    emitter_regions = tuple(
        EmitterRegion(
            name=f"{alliance}_coral_emitter",
            vertices=_rect(alliance_zone_center(alliance), CORAL_EMITTER_ZONE_SIZE, CORAL_EMITTER_ZONE_SIZE),
            piece_type=CORAL_TYPE,
            active_times=((AUTO_DURATION, AUTO_DURATION + TELEOP_DURATION),),
            emit_rate_hz=CORAL_EMITTER_RATE_HZ,
            linked_collection_region=f"{alliance}_coral_station_1",
            alliance=alliance,
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


def _rect(center: tuple[float, float], width: float, depth: float) -> tuple[tuple[float, float], ...]:
    hw, hd = width / 2.0, depth / 2.0
    cx, cy = center
    return ((cx - hw, cy - hd), (cx + hw, cy - hd), (cx + hw, cy + hd), (cx - hw, cy + hd))
