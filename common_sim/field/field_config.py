"""
Game-agnostic field description. A game_specific package builds one
concrete FieldConfig instance describing that year's field; nothing in
common_sim ever hardcodes a particular game's layout.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Obstacle:
    """A static field obstacle (reef, stage truss, charge station, ...).
    Rendered and collided with as a solid polygon.

    `height`, in inches, is visual-only -- collision is always the 2D
    footprint regardless of this value (this sim has no z-axis physics).
    0.0 (the default) draws the flat footprint everywhere, matching every
    game_specific package that predates this field; a game_specific
    package that sets it gets an extruded prism instead, but only under
    a driver-station camera (see gui_utils/field_canvas.py) -- the
    top-down view ignores it entirely, so setting this can never change
    what the ordinary strategy view looks like."""
    name: str
    vertices: tuple[tuple[float, float], ...]
    height: float = 0.0


@dataclass(frozen=True)
class ScoringRegion:
    """A sensor zone that scores an un-held piece released into it, for
    whichever scoring action the robot targeted the deposit at.

    A single physical zone can support several distinct scoring actions
    (e.g. one reef face offering "l1_coral".."l4_coral") -- `actions`
    lists every action name valid at this location. Each action's point
    value (via ScoringRules, see match/scoring.py) and how long a robot
    takes to perform it (via RobotCharacteristics.deposit_time_by_action)
    are looked up by that name wherever it's used, so "l4_coral" means
    the same points/timing at every reef face that offers it.

    `piece_types`, if non-empty, restricts which piece types this region
    accepts; empty means "any piece type".

    `passive_scoring` distinguishes two real scoring mechanics that look
    identical to a point-in-polygon check but aren't: most FIRST scoring
    locations (REEF branches, PROCESSOR) require a robot to actually be
    in position performing the scoring action -- a piece can never just
    roll or bounce its way to points there. A few (like REEFSCAPE's NET)
    are scored by launching a piece from a distance; the piece only needs
    to land in the zone, with no robot presence required at landing time.
    False (default) means only an explicit robot deposit while engaged
    with this region (Match.deposit_region_for finds it) can score a
    piece here. True additionally allows a piece already carrying a
    matching target_action to score by drifting/rolling/flying into the
    region afterward, with no robot in position -- see Match._try_score."""
    name: str
    vertices: tuple[tuple[float, float], ...]
    actions: frozenset[str]
    piece_types: frozenset[str] = frozenset()
    passive_scoring: bool = False
    # Per-action cap on how many pieces this region can ever accept for
    # that action, e.g. {"l4_coral": 1} for a REEF branch that physically
    # only holds one CORAL. An action missing from this mapping (or a
    # None mapping entirely, the default) is unlimited -- so a game that
    # never sets this keeps scoring as many pieces as land there, exactly
    # like before this field existed. Enforced by Match, see
    # Match.region_full / Match._try_score.
    capacity_by_action: dict[str, int] | None = None
    # Which alliance owns this region ("red"/"blue"), None if it's
    # neutral/shared. world_view.scoring_options and Match.deposit_region_for
    # both filter on this against Robot.alliance, so a robot never plans to
    # (or actually does) score on the opposing alliance's regions.
    alliance: str | None = None
    # Visual-only elevation, in inches, of this zone above the carpet --
    # see Obstacle.height for the same contract (ignored by every scoring
    # check, ignored by the top-down view, only drawn by a driver-station
    # camera). 0.0 (the default, and every region this sim currently
    # builds) means "on the ground", which is honest for REEFSCAPE: a
    # face's ScoringRegion already represents all of L1-L4 flattened into
    # one zone (see game_specific/reefscape/field.py), so there is no
    # single real elevation to assign it without inventing one.
    height: float = 0.0


@dataclass(frozen=True)
class ProtectedZone:
    """An area where a robot is safe from opponent contact -- REEFSCAPE's
    REEF ZONE, DEEP CAGE zones, older games' LOADING ZONE / protected
    scoring areas. Nearly every FIRST game has at least one, because a
    game needs somewhere a robot can complete a delicate alignment
    without being shoved off it, so this is a generic primitive rather
    than a REEFSCAPE detail.

    The protection is on the *robot*, not the area: a robot with any part
    of its footprint inside the zone may not be contacted by an opponent,
    anywhere. An opponent may stand in the zone, drive through it, and
    deny the approach to it -- it just may not touch the robot being
    protected once that robot has arrived. That asymmetry is the whole
    tactical point: defense can keep you out, but cannot dislodge you.

    `alliance` names whose robots the zone protects ("red"/"blue"); None
    protects whichever robot is inside it from every robot that isn't on
    its alliance, which is how a neutral safe area works.

    `foul_points` is what a single violation is worth to the protected
    robot's alliance -- games price these differently (a FOUL vs. a TECH
    FOUL vs. a warning), so the number lives with the zone rather than
    being hardcoded. 0.0 (the default) makes the zone advisory: the
    contact is still detected and logged, but costs nothing, which is
    what a game wants when the real penalty is a card rather than points.

    `foul_period` is how long one continuous contact goes before it
    counts again. A referee calls a foul, not a foul per 1/60th of a
    second, so without this a two-second shove would be scored 120 times.
    """
    name: str
    vertices: tuple[tuple[float, float], ...]
    alliance: str | None = None
    foul_points: float = 0.0
    foul_period: float = 1.0


@dataclass(frozen=True)
class PinRule:
    """A limit on how long one robot may prevent another from moving.
    REEFSCAPE's is three seconds; the rule is near-universal across FIRST
    games because without it the strongest defense in any game is simply
    to sit on the best opponent for the whole match.

    What counts is *motion*, not access. A defender parked across a
    feeder mouth, or squatting on the spot a robot wants to score from,
    commits no pin however long it stays -- its victim can drive
    anywhere it likes, it just cannot get to the one place it wanted.
    Only sustained contact that leaves the victim unable to go anywhere
    starts the clock, which in practice means shoving: `Match._step_pins`
    asks whether a robot in contact with an opponent is commanding
    motion and not achieving it. Under a traction-limited drivetrain
    (see physics/swerve.py) that question has a real answer -- a victim
    with anywhere to go gets there, and a victim held square by an
    equally powered opponent does not.

    `max_seconds` is how long a single unbroken pin may last.
    `release_seconds` is how long the offender must let the victim move
    before the clock resets: a defender that backs off for an instant
    and immediately re-pins has not released, and real rules say so.
    `foul_points` and the accounting behave exactly like ProtectedZone's
    -- 0.0 makes the rule advisory (counted and logged, costing nothing),
    which is what a game wants when the real answer is a card.

    `stopped_speed` is the speed below which a robot counts as not
    moving, and also the commanded speed above which it counts as
    trying to. It is one number rather than two because the question is
    symmetric: the victim asked for more than this and got less."""
    max_seconds: float = 3.0
    release_seconds: float = 1.0
    foul_points: float = 0.0
    stopped_speed: float = 6.0  # in/s


@dataclass(frozen=True)
class IntakeLocation:
    """A zone with an unlimited, continuously-replenished supply of one
    piece type -- e.g. a human-player feeder/loading station. Unlike a
    PieceSpawnRegion, no physical GamePiece needs to exist in the zone
    beforehand: a robot that sits in the zone with intake commanded
    active for its own RobotCharacteristics.station_intake_time seconds
    gets a new piece handed directly into its held pieces (see
    Match._register_collision_handlers / Robot.update_station_intake),
    the same way a real feeder keeps handing off pieces as fast as a
    robot can take them. The dispense timing is per-robot, not per-
    location, so two robots with different station_intake_time values
    cycle the same station at different rates.

    `starting_pieces`, if set, caps the total number of pieces this
    location can ever dispense over a match (Match tracks the remaining
    count and stops dispensing at 0); None means unlimited.

    `piece_color`, if set, is passed through to a dispensed piece's
    GamePiece.color -- without it, a station-dispensed piece gets no
    color (GamePiece.color defaults to None) and a GUI falls back to its
    own placeholder color, which can visibly mismatch same-type pieces
    spawned elsewhere (e.g. field piles) with an explicit color."""
    name: str
    vertices: tuple[tuple[float, float], ...]
    piece_type: str
    starting_pieces: int | None = None
    piece_color: str | None = None
    # Which alliance this station belongs to ("red"/"blue"), None if
    # shared. world_view.station_options filters on this against
    # Robot.alliance, mirroring ScoringRegion.alliance.
    alliance: str | None = None


@dataclass(frozen=True)
class PieceSpawnRegion:
    """Where game pieces originate. `is_station` marks a human-player
    station (unlimited supply, uses a robot's station_intake_time) as
    opposed to a field pile (finite pieces, uses intake_time)."""
    name: str
    vertices: tuple[tuple[float, float], ...]
    piece_type: str
    is_station: bool = False
    count: int | None = None  # None = unlimited (typical for a station)


@dataclass(frozen=True)
class EmitterRegion:
    """A zone that spawns loose game pieces onto the field over time, the
    way a human player tosses pieces in rather than a robot collecting
    them from a station -- see Match._step_emitters.

    `active_times` is a list of (start, end) match-elapsed-second windows
    during which the emitter runs; empty means "active for the whole
    match". Outside its windows the emitter just idles -- capacity isn't
    consumed and the emit-rate timer doesn't advance.

    `emit_rate_hz` is how often (in Hz, e.g. 0.1 for one piece every 10s)
    the emitter drops a new piece while it has capacity and is active.

    `initial_capacity` caps how many pieces this emitter can ever emit,
    None meaning unlimited. Ignored (must be None) when
    `linked_collection_region` is set -- see below.

    `linked_collection_region`, if set, names an IntakeLocation this
    emitter shares its piece pool with: emitting draws down that
    location's own remaining-supply counter (Match.station_supply)
    instead of a separate count of the emitter's own, so a human-player
    station and its matching on-field emitter never double-count the
    same physical stock of pieces. Mutually exclusive with
    `initial_capacity`.

    `linked_scoring_region` + `return_delay` model a piece cycling back
    onto the field after being scored (e.g. an algae that gets pulled
    back off the NET and reused) -- naming a ScoringRegion here means a
    piece of this emitter's `piece_type` scored there gets returned to
    this emitter's pool (station-shared or its own) `return_delay`
    seconds later, to be emitted again in its own turn at emit_rate_hz
    rather than reappearing on the field immediately. None/0.0 delay
    means the very next tick. Only meaningful alongside
    `linked_scoring_region`.

    An emitted piece's radius/mass/color come from `piece_type`'s
    registered GamePieceSpec (see common_sim.field.game_piece,
    Match.spawn_piece) -- an EmitterRegion doesn't redeclare them."""
    name: str
    vertices: tuple[tuple[float, float], ...]
    piece_type: str
    active_times: tuple[tuple[float, float], ...] = ()
    emit_rate_hz: float = 1.0
    initial_capacity: int | None = None
    linked_collection_region: str | None = None
    linked_scoring_region: str | None = None
    return_delay: float | None = None
    alliance: str | None = None


@dataclass(frozen=True)
class FieldConfig:
    width: float
    height: float
    obstacles: tuple[Obstacle, ...] = field(default_factory=tuple)
    scoring_regions: tuple[ScoringRegion, ...] = field(default_factory=tuple)
    spawn_regions: tuple[PieceSpawnRegion, ...] = field(default_factory=tuple)
    intake_locations: tuple[IntakeLocation, ...] = field(default_factory=tuple)
    emitter_regions: tuple[EmitterRegion, ...] = field(default_factory=tuple)
    protected_zones: tuple[ProtectedZone, ...] = field(default_factory=tuple)
    # None (the default) means the game has no pin rule and Match never
    # runs the check at all -- pinning is then simply legal.
    pin_rule: PinRule | None = None


def polygon_centroid(vertices: tuple[tuple[float, float], ...]) -> tuple[float, float]:
    """Simple average-of-vertices centroid -- good enough for placing
    spawned pieces within a region; not an area-weighted centroid."""
    xs = [v[0] for v in vertices]
    ys = [v[1] for v in vertices]
    return (sum(xs) / len(xs), sum(ys) / len(ys))


def polygon_area(vertices: tuple[tuple[float, float], ...]) -> float:
    """Shoelace area, always positive -- callers describe regions in
    whichever winding order reads naturally, and none of them care about
    orientation."""
    total = 0.0
    x1, y1 = vertices[-1]
    for x2, y2 in vertices:
        total += x1 * y2 - x2 * y1
        x1, y1 = x2, y2
    return abs(total) / 2.0


def _segments_cross(p1, p2, p3, p4) -> bool:
    """Whether segment p1-p2 crosses segment p3-p4. Collinear overlap
    reads as no crossing, which is fine here: the callers below have
    already tested both polygons' vertices for containment, and two
    polygons whose edges merely lie along each other have a vertex of
    one inside (or on) the other."""
    def side(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    d1, d2 = side(p3, p4, p1), side(p3, p4, p2)
    d3, d4 = side(p1, p2, p3), side(p1, p2, p4)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def polygons_intersect(a: tuple[tuple[float, float], ...], b: tuple[tuple[float, float], ...]) -> bool:
    """Whether two polygons share any area. Vertex-containment both ways
    catches one polygon fully inside the other; the edge-crossing pass
    catches the case neither containment test sees, two polygons
    overlapping in a band with every vertex outside the other (a plus
    sign). Works on concave polygons, unlike a separating-axis test --
    a game is free to describe an L-shaped safe zone.

    Bounding boxes are compared first because the honest answer is almost
    always "no": the hot callers are `Match.protecting_zone` (one robot
    against every zone on the field) and `Match.robots_in_contact` (every
    robot against every other), and on a 690x317in field a given pair is
    nowhere near each other on the overwhelming majority of ticks. Two
    boxes that don't overlap cannot share area, so the reject is exact,
    and it costs four comparisons against a full pass that is O(n*m) in
    edges plus n+m ray casts."""
    ax = [v[0] for v in a]
    bx = [v[0] for v in b]
    if max(ax) < min(bx) or min(ax) > max(bx):
        return False
    ay = [v[1] for v in a]
    by = [v[1] for v in b]
    if max(ay) < min(by) or min(ay) > max(by):
        return False
    if any(point_in_polygon(v, b) for v in a) or any(point_in_polygon(v, a) for v in b):
        return True
    for i in range(len(a)):
        a1, a2 = a[i], a[(i + 1) % len(a)]
        for j in range(len(b)):
            if _segments_cross(a1, a2, b[j], b[(j + 1) % len(b)]):
                return True
    return False


def point_in_polygon(point: tuple[float, float], vertices: tuple[tuple[float, float], ...]) -> bool:
    """Standard ray-casting point-in-polygon test."""
    x, y = point
    inside = False
    n = len(vertices)
    x1, y1 = vertices[-1]
    for i in range(n):
        x2, y2 = vertices[i]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
        x1, y1 = x2, y2
    return inside
