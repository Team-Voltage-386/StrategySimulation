"""
world_view tests against a synthetic field/match, mirroring
test_match_synthetic.py's trivial made-up game.
"""
from common_sim.control import world_view
from common_sim.field.field_config import FieldConfig, IntakeLocation, ScoringRegion
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
        piece_type=WIDGET, dispense_time=0.5, starting_pieces=2,
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
