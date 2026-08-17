"""
world_view tests against a synthetic field/match, mirroring
test_match_synthetic.py's trivial made-up game.
"""
import math

from common_sim.control import world_view
from common_sim.field.field_config import FieldConfig, IntakeLocation, ScoringRegion, point_in_polygon
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.match.scoring import TableScoringRules
from common_sim.robot.characteristics import RobotCharacteristics

WIDGET = "widget"
GADGET = "gadget"


def make_field() -> FieldConfig:
    region = ScoringRegion(
        name="goal",
        vertices=((80, -60), (250, -60), (250, 60), (80, 60)),
        actions=frozenset({"score_widget"}),
        piece_types=frozenset({WIDGET}),
    )
    station = IntakeLocation(
        name="feeder", vertices=((0, -20), (20, -20), (20, 20), (0, 20)),
        piece_type=WIDGET, starting_pieces=2,
    )
    return FieldConfig(width=300, height=200, scoring_regions=(region,), intake_locations=(station,))


def make_characteristics(**overrides) -> RobotCharacteristics:
    defaults = dict(
        name="test-bot", max_speed=150.0, max_accel=400.0, max_angular_speed=6.0,
        max_angular_accel=20.0, width=28.0, length=28.0, piece_capacity=1,
        intake_time=0.1, deposit_time=0.1, intake_range=6.0,
        accepted_piece_types=frozenset({WIDGET}),
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)


def make_match(**config_overrides) -> Match:
    return Match(make_field(), TableScoringRules({("score_widget", "auto"): 3.0}), MatchConfig(**config_overrides))


def test_collectable_pieces_excludes_held_and_scored():
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))
    free = match.spawn_piece(WIDGET, (100, 100))
    held = match.spawn_piece(WIDGET, (0, 0))
    held.held_by = robot
    robot.held_pieces.append(held)
    scored = match.spawn_piece(WIDGET, (150, 0))
    scored.scored = True

    result = world_view.collectable_pieces(match)
    assert result == [free]


def test_collectable_pieces_filters_by_type():
    match = make_match()
    widget = match.spawn_piece(WIDGET, (10, 10))
    gadget = match.spawn_piece(GADGET, (10, 10))

    assert world_view.collectable_pieces(match, piece_type=WIDGET) == [widget]
    assert world_view.collectable_pieces(match, piece_type=GADGET) == [gadget]


def test_piece_clusters_groups_dense_group_first():
    match = make_match()
    a = match.spawn_piece(WIDGET, (0, 0))
    b = match.spawn_piece(WIDGET, (5, 0))
    c = match.spawn_piece(WIDGET, (5, 5))
    far = match.spawn_piece(WIDGET, (200, 200))

    clusters = world_view.piece_clusters(match, [a, b, c, far], radius=10)

    assert len(clusters) == 2
    dense = max(clusters, key=lambda c: c.count)
    assert dense.count == 3
    assert set(dense.pieces) == {a, b, c}
    lone = min(clusters, key=lambda c: c.count)
    assert lone.pieces == (far,)


def test_station_options_respects_supply_and_type():
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))
    assert [loc.name for loc in world_view.station_options(match, robot)] == ["feeder"]

    station = match.field.intake_locations[0]
    match.station_supply[station] = 0
    assert world_view.station_options(match, robot) == []


def test_station_options_excludes_when_capacity_full():
    match = make_match()
    robot = match.add_robot(make_characteristics(piece_capacity=1), Pose2d(0, 0, 0))
    piece = match.spawn_piece(WIDGET, (0, 0))
    piece.held_by = robot
    robot.held_pieces.append(piece)

    assert world_view.station_options(match, robot) == []


def test_scoring_options_lists_legal_region_action_pairs():
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(150, 0, 0))
    piece = match.spawn_piece(WIDGET, (0, 0))
    piece.held_by = robot
    robot.held_pieces.append(piece)

    options = world_view.scoring_options(match, robot)
    assert len(options) == 1
    assert options[0].region.name == "goal"
    assert options[0].action == "score_widget"
    assert options[0].piece is piece


def test_scoring_options_empty_when_robot_holds_nothing():
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(150, 0, 0))
    assert world_view.scoring_options(match, robot) == []


def test_scoring_options_respects_region_full_hook():
    match = make_match()
    match.region_full = lambda region, action: True
    robot = match.add_robot(make_characteristics(), Pose2d(150, 0, 0))
    piece = match.spawn_piece(WIDGET, (0, 0))
    piece.held_by = robot
    robot.held_pieces.append(piece)

    assert world_view.scoring_options(match, robot) == []


def test_opponents_and_partners():
    match = make_match()
    blue1 = match.add_robot(make_characteristics(), Pose2d(0, 0, 0), alliance="blue")
    blue2 = match.add_robot(make_characteristics(), Pose2d(10, 0, 0), alliance="blue")
    red1 = match.add_robot(make_characteristics(), Pose2d(20, 0, 0), alliance="red")

    assert world_view.opponents(match, "blue") == [red1]
    assert world_view.partners(match, "blue") == [blue1, blue2]


def test_region_by_name():
    match = make_match()
    assert world_view.region_by_name(match, "goal") is not None
    assert world_view.region_by_name(match, "nope") is None


TIGHT_REGION = ScoringRegion(
    name="tight", vertices=((100, -10), (120, -10), (120, 10), (100, 10)),
    actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}),
)


class _FakeIntent:
    def __init__(self, target_region):
        self.target_region = target_region


class _FakeController:
    def __init__(self, target_region):
        self.intent = _FakeIntent(target_region)

    def tick(self, ctx):
        pass


def test_region_robot_capacity_scales_with_region_size():
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))
    # 20x20 -- smaller than the robot's own 28x28 footprint.
    assert world_view.region_robot_capacity(TIGHT_REGION, robot) == 1
    # 170x120 "goal" -- room for plenty.
    assert world_view.region_robot_capacity(match.field.scoring_regions[0], robot) > 1


def test_region_occupants_counts_both_position_and_intent():
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))
    standing_in = match.add_robot(make_characteristics(), Pose2d(150, 0, 0))
    heading_for = match.add_robot(make_characteristics(), Pose2d(0, 100, 0))
    heading_for.controller = _FakeController(target_region="goal")
    elsewhere = match.add_robot(make_characteristics(), Pose2d(0, -100, 0))
    elsewhere.controller = _FakeController(target_region="somewhere-else")

    goal = match.field.scoring_regions[0]
    occupants = world_view.region_occupants(match, goal, exclude=robot)
    assert set(occupants) == {standing_in, heading_for}
    assert elsewhere not in occupants
    # `exclude` keeps a robot from counting itself as its own crowd.
    assert robot not in world_view.region_occupants(match, goal, exclude=robot)


def test_region_has_room_only_until_capacity():
    field = FieldConfig(width=300, height=200, scoring_regions=(TIGHT_REGION,))
    match = Match(field, TableScoringRules({}), MatchConfig())
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))
    assert world_view.region_has_room(match, TIGHT_REGION, robot)

    claimer = match.add_robot(make_characteristics(), Pose2d(0, 50, 0))
    claimer.controller = _FakeController(target_region="tight")
    assert not world_view.region_has_room(match, TIGHT_REGION, robot)


def test_region_approach_point_is_centroid_when_region_is_empty():
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))
    goal = match.field.scoring_regions[0]
    assert world_view.region_approach_point(goal, robot, []) == world_view.region_centroid(goal)


def test_region_approach_point_clears_another_occupant():
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))
    goal = match.field.scoring_regions[0]
    centroid = world_view.region_centroid(goal)
    occupant = match.add_robot(make_characteristics(), Pose2d(*centroid, 0))

    point = world_view.region_approach_point(goal, robot, [occupant])
    assert point != centroid
    # Far enough that the two chassis aren't touching, and still inside
    # the region (so the deposit still counts).
    assert math.hypot(point[0] - centroid[0], point[1] - centroid[1]) >= 28.0
    assert point_in_polygon(point, goal.vertices)


def test_region_approach_point_weighs_a_teammate_and_an_opponent_differently():
    """A teammate only has to be out of the way -- past a footprint
    diagonal there is nothing more to gain, so the nearest adequate point
    wins and the robot stops on the near side of the region.

    An opponent is not something to merely clear. The whole value of a
    large scoring region is that it cannot be denied at one spot, which
    only pays if the robot actually uses the far end of it, so the same
    occupant in the same place is worth driving much further from."""
    match = make_match()
    goal = match.field.scoring_regions[0]  # x 80..250, y -60..60
    # Approaching from off the region's near (low-x) edge, with the
    # occupant sitting just inside that same edge.
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0), alliance="blue")
    occupied = Pose2d(100, 0, 0)
    partner = match.add_robot(make_characteristics(), occupied, alliance="blue")
    defender = match.add_robot(make_characteristics(), occupied, alliance="red")

    friendly_point = world_view.region_approach_point(goal, robot, [partner])
    hostile_point = world_view.region_approach_point(goal, robot, [defender])

    def clearance(point):
        return math.hypot(point[0] - occupied.x, point[1] - occupied.y)

    for point in (friendly_point, hostile_point):
        assert point_in_polygon(point, goal.vertices)
        assert clearance(point) >= 28.0  # both at least get out of the way

    assert clearance(hostile_point) > 2 * clearance(friendly_point)
    assert friendly_point[0] < 130.0   # stays on the near side it came from
    assert hostile_point[0] > 200.0    # crosses to the far end instead


def _two_sided_match() -> Match:
    """Owned features at each end of the long axis, mirrored -- blue low,
    red high, so the halves come out split at x=150."""
    field = FieldConfig(
        width=300, height=200,
        scoring_regions=(
            ScoringRegion(
                name="blue_goal", vertices=((10, 80), (50, 80), (50, 120), (10, 120)),
                actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}), alliance="blue",
            ),
            ScoringRegion(
                name="red_goal", vertices=((250, 80), (290, 80), (290, 120), (250, 120)),
                actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}), alliance="red",
            ),
        ),
        intake_locations=(
            IntakeLocation(
                name="blue_feeder", vertices=((0, 0), (20, 0), (20, 20), (0, 20)),
                piece_type=WIDGET, starting_pieces=2, alliance="blue",
            ),
            IntakeLocation(
                name="red_feeder", vertices=((280, 0), (300, 0), (300, 20), (280, 20)),
                piece_type=WIDGET, starting_pieces=2, alliance="red",
            ),
        ),
    )
    return Match(field, TableScoringRules({}), MatchConfig())


def test_own_side_test_splits_on_the_axis_the_alliances_are_spread_along():
    match = _two_sided_match()
    blue, red = world_view.own_side_test(match, "blue"), world_view.own_side_test(match, "red")

    for x in (0, 100, 149):
        assert blue(x, 100) and not red(x, 100)
    for x in (151, 200, 300):
        assert red(x, 100) and not blue(x, 100)
    # The split is on x, so y doesn't move a point across it.
    assert blue(50, 0) and blue(50, 199)


def test_own_side_test_passes_everything_when_the_field_has_no_split():
    # A field where only one alliance owns anything has no discernible
    # halves -- rather than invent a dividing line, every point counts as
    # own-side so a caller gating on this simply does nothing.
    match = make_match()   # unowned goal + feeder
    test = world_view.own_side_test(match, "blue")
    assert test(0, 0) and test(300, 200)


class _DefenseIntent:
    """Just the attributes world_view reads off a live strategy.Intent."""

    def __init__(self, *, defending=False, marking=None, target_region=None):
        self.defending = defending
        self.marking = marking
        self.target_region = target_region
        self.target_piece = None


class _DefenseController:
    def __init__(self, intent):
        self.intent = intent

    def tick(self, ctx):
        pass


def _declare(robot, **intent_kwargs):
    robot.controller = _DefenseController(_DefenseIntent(**intent_kwargs))
    return robot


def test_defenders_only_counts_declared_defensive_intent():
    match = make_match()
    ours = match.add_robot(make_characteristics(), Pose2d(0, 0, 0), alliance="blue")
    defender = _declare(
        match.add_robot(make_characteristics(), Pose2d(100, 0, 0), alliance="red"),
        defending=True, marking=ours, target_region="goal",
    )
    # An opponent going about its own business is not a defender, however
    # squarely it happens to be standing in the way.
    _declare(match.add_robot(make_characteristics(), Pose2d(50, 0, 0), alliance="red"), target_region="goal")

    assert world_view.defenders(match, "blue") == [defender]
    assert world_view.defenders(match, "red") == []


def test_defenders_against_excludes_one_marking_a_teammate():
    match = make_match()
    ours = match.add_robot(make_characteristics(), Pose2d(0, 0, 0), alliance="blue")
    teammate = match.add_robot(make_characteristics(), Pose2d(0, 40, 0), alliance="blue")

    on_us = _declare(
        match.add_robot(make_characteristics(), Pose2d(100, 0, 0), alliance="red"),
        defending=True, marking=ours,
    )
    unassigned = _declare(
        match.add_robot(make_characteristics(), Pose2d(120, 0, 0), alliance="red"), defending=True, marking=None,
    )
    _declare(
        match.add_robot(make_characteristics(), Pose2d(140, 0, 0), alliance="red"),
        defending=True, marking=teammate,
    )

    assert world_view.defenders_against(match, ours) == [on_us, unassigned]


def test_region_denied_by_matches_declared_region_only():
    match = make_match()
    match.add_robot(make_characteristics(), Pose2d(0, 0, 0), alliance="blue")
    on_goal = _declare(
        match.add_robot(make_characteristics(), Pose2d(100, 0, 0), alliance="red"),
        defending=True, target_region="goal",
    )
    _declare(
        match.add_robot(make_characteristics(), Pose2d(120, 0, 0), alliance="red"),
        defending=True, target_region="elsewhere",
    )

    assert world_view.region_denied_by(match, "goal", "blue") == [on_goal]
    assert world_view.region_denied_by(match, "elsewhere", "red") == []


def test_likely_scoring_region_prefers_declared_then_falls_back_to_nearest():
    match = make_match()
    robot = _declare(match.add_robot(make_characteristics(), Pose2d(0, 0, 0)), target_region="goal")
    assert world_view.likely_scoring_region(match, robot).name == "goal"

    # No declaration -- a defender still has to guess somewhere, and the
    # only region on this field is the one it should guess.
    robot.controller = None
    assert world_view.likely_scoring_region(match, robot).name == "goal"


def test_denial_target_by_name_finds_stations_as_well_as_regions():
    match = make_match()
    assert world_view.denial_target_by_name(match, "goal").name == "goal"
    assert world_view.denial_target_by_name(match, "feeder").name == "feeder"
    assert world_view.denial_target_by_name(match, "nope") is None
    assert world_view.denial_target_kind(match, world_view.denial_target_by_name(match, "feeder")) == "supply"


def test_likely_denial_target_guesses_supply_for_an_empty_handed_mark():
    """The old guess was always a scoring region, which put a defender in
    front of a place its empty-handed mark had no reason to visit."""
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))

    assert world_view.likely_denial_target(match, robot).name == "feeder"

    robot.held_pieces.append(match.spawn_piece(WIDGET, (0, 0)))
    assert world_view.likely_denial_target(match, robot).name == "goal"


def test_likely_denial_target_guesses_supply_for_the_type_the_mark_is_cycling():
    """A field mixes supply points of very different kinds. Guessing
    "nearest" alone sends a defender to whichever one happens to be
    underfoot -- on REEFSCAPE, a one-ALGAE nook tucked into the REEF the
    mark is currently scoring at, rather than the CORAL STATION it will
    actually drive back to. Measured, that mistake was worth 68 points a
    match, in the defender's favour and entirely by accident."""
    nook = IntakeLocation(
        name="nook", vertices=((150, -10), (170, -10), (170, 10), (150, 10)),
        piece_type=GADGET, starting_pieces=1,
    )
    field = make_field()
    field = FieldConfig(
        width=field.width, height=field.height, scoring_regions=field.scoring_regions,
        intake_locations=(*field.intake_locations, nook),
    )
    match = Match(field, TableScoringRules({("score_widget", "auto"): 3.0}), MatchConfig())
    robot = match.add_robot(
        make_characteristics(accepted_piece_types=frozenset({WIDGET, GADGET})), Pose2d(160, 0, 0),
    )
    robot.held_pieces.append(match.spawn_piece(WIDGET, (160, 0)))

    # The nook is right under it and the feeder is 160in away, but it is
    # carrying a widget, so the widget feeder is where it is headed.
    assert world_view.likely_denial_target(match, robot, ("supply",)).name == "feeder"


def test_likely_denial_target_honors_the_kinds_it_is_allowed_to_deny():
    match = make_match()
    robot = _declare(match.add_robot(make_characteristics(), Pose2d(0, 0, 0)), target_region="feeder")

    assert world_view.likely_denial_target(match, robot, ("scoring", "supply")).name == "feeder"
    # Told to attack scoring only, a declared feeder is not an answer --
    # falling through to the scoring guess beats marking nothing at all.
    assert world_view.likely_denial_target(match, robot, ("scoring",)).name == "goal"


def test_alliance_intake_locations_includes_unowned_stations():
    match = make_match()
    assert [s.name for s in world_view.alliance_intake_locations(match, "blue")] == ["feeder"]


def test_alliance_scoring_regions_includes_unowned_regions():
    match = make_match()
    names = [r.name for r in world_view.alliance_scoring_regions(match, "blue")]
    assert names == ["goal"]  # the synthetic field's region has alliance=None
