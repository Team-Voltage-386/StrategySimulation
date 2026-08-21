"""
utility tests against a synthetic field/match, mirroring
test_world_view.py's and test_planning.py's trivial made-up game.

The recurring theme is that a pickup's value is not in the pickup: most
of these assert something about `enables`, because that is the term that
makes fetching and scoring comparable at all.
"""
from common_sim.control import navigation, utility, world_view
from common_sim.field.field_config import FieldConfig, IntakeLocation, ScoringRegion
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.match.scoring import TableScoringRules
from common_sim.robot.characteristics import RobotCharacteristics

WIDGET = "widget"
GADGET = "gadget"


def make_field(**overrides) -> FieldConfig:
    goal = ScoringRegion(
        name="goal", vertices=((80, -60), (250, -60), (250, 60), (80, 60)),
        actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}),
    )
    feeder = IntakeLocation(
        name="feeder", vertices=((0, -20), (20, -20), (20, 20), (0, 20)),
        piece_type=WIDGET,
    )
    defaults = dict(width=300, height=200, scoring_regions=(goal,), intake_locations=(feeder,))
    defaults.update(overrides)
    return FieldConfig(**defaults)


def make_characteristics(**overrides) -> RobotCharacteristics:
    defaults = dict(
        name="test-bot", max_speed=150.0, max_accel=400.0, max_angular_speed=6.0,
        max_angular_accel=20.0, width=28.0, length=28.0, piece_capacity=1,
        intake_time=0.1, station_intake_time=0.6, deposit_time=0.1, intake_range=6.0,
        accepted_piece_types=frozenset({WIDGET}),
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)


def make_match(field=None) -> Match:
    return Match(
        field or make_field(),
        TableScoringRules({("score_widget", "auto"): 3.0}),
        MatchConfig(auto_duration=1000, teleop_duration=1000),
    )


def give(match, robot, piece_type=WIDGET, position=(0, 0)):
    piece = match.spawn_piece(piece_type, position)
    piece.held_by = robot
    robot.held_pieces.append(piece)
    return piece


def travel_to(match, robot, goal):
    return navigation.estimate_travel_time(
        match.field, (robot.pose.x, robot.pose.y), goal, robot.characteristics,
    )


# --- Outcome ---------------------------------------------------------


def test_value_rate_matches_scoring_option_and_floors_zero_duration():
    option = utility.ScoringOption(
        region=None, action="score_widget", piece=None,
        points=6.0, deposit_time=0.5, travel_time=1.5,
    )
    outcome = utility.Outcome(
        kind="score", label="x", points=option.points,
        duration=option.travel_time + option.deposit_time,
        success_probability=1.0, payload=option,
    )
    assert outcome.value_rate == option.value_rate == 3.0

    # The 1e-6 floor is load-bearing: a zero-cost option must be very
    # good, not a ZeroDivisionError.
    free = utility.Outcome(
        kind="score", label="x", points=1.0, duration=0.0,
        success_probability=1.0, payload=None,
    )
    assert free.value_rate == 1.0 / 1e-6


def test_outcomes_do_not_share_a_mutable_rp_progress():
    a = utility.Outcome(kind="score", label="a", points=1.0, duration=1.0,
                        success_probability=1.0, payload=None)
    b = utility.Outcome(kind="score", label="b", points=1.0, duration=1.0,
                        success_probability=1.0, payload=None)
    a.rp_progress["coral"] = 0.5
    assert b.rp_progress == {}


# --- score_outcomes --------------------------------------------------


def test_score_outcomes_mirror_world_view_options_in_order():
    match = make_match()
    robot = match.add_robot(make_characteristics(piece_capacity=2), Pose2d(0, 0, 0))
    give(match, robot)
    give(match, robot, position=(0, 10))

    legal = world_view.scoring_options(match, robot)
    outcomes = utility.score_outcomes(match, robot)

    assert len(outcomes) == len(legal) == 2
    for outcome, expected in zip(outcomes, legal):
        option = outcome.payload
        assert (option.region, option.action, option.piece) == (expected.region, expected.action, expected.piece)
        assert outcome.points == option.points == 3.0
        assert outcome.duration == option.travel_time + option.deposit_time
        assert outcome.kind == "score"
        assert outcome.label == "score score_widget @ goal"


def test_score_outcomes_price_from_a_virtual_pose():
    """The chaining case: a planner that has already placed one piece
    asks for the next one's options from where that placement left it,
    not from where the robot is standing now."""
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))
    give(match, robot)

    near = utility.score_outcomes(match, robot, from_pos=(200, 0))[0]
    far = utility.score_outcomes(match, robot, from_pos=(0, 0))[0]
    assert near.duration < far.duration
    assert far.duration == utility.score_outcomes(match, robot)[0].duration  # default is the robot's pose


def test_score_outcomes_pieces_filter_restricts_to_named_pieces():
    match = make_match()
    robot = match.add_robot(make_characteristics(piece_capacity=2), Pose2d(0, 0, 0))
    first = give(match, robot)
    second = give(match, robot, position=(0, 10))

    only_second = utility.score_outcomes(match, robot, pieces=[second])
    assert [o.payload.piece for o in only_second] == [second]
    assert utility.score_outcomes(match, robot, pieces=[]) == []
    assert len(utility.score_outcomes(match, robot, pieces=[first, second])) == 2


def test_score_outcomes_carry_per_type_reliability():
    match = make_match()
    characteristics = make_characteristics(scoring_reliability_by_type={WIDGET: 0.75})
    robot = match.add_robot(characteristics, Pose2d(0, 0, 0))
    give(match, robot)
    assert utility.score_outcomes(match, robot)[0].success_probability == 0.75


def test_score_outcomes_empty_when_nothing_held():
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))
    assert utility.score_outcomes(match, robot) == []


# --- scoring_slots_for_type ------------------------------------------


def test_scoring_slots_for_type_answers_about_a_piece_not_held():
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))
    assert not robot.held_pieces

    slots = world_view.scoring_slots_for_type(match, robot, WIDGET)
    assert [(region.name, action) for region, action in slots] == [("goal", "score_widget")]
    # The region only accepts WIDGETs, and the robot only accepts WIDGETs.
    assert world_view.scoring_slots_for_type(match, robot, GADGET) == []


def test_scoring_slots_for_type_respects_region_full():
    goal = ScoringRegion(
        name="goal", vertices=((80, -60), (250, -60), (250, 60), (80, 60)),
        actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}),
        capacity_by_action={"score_widget": 1},
    )
    match = make_match(make_field(scoring_regions=(goal,)))
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))
    assert world_view.scoring_slots_for_type(match, robot, WIDGET)

    match.region_scores.setdefault("goal", {})["score_widget"] = 1
    assert world_view.scoring_slots_for_type(match, robot, WIDGET) == []


def test_scoring_slots_for_type_respects_region_alliance():
    goal = ScoringRegion(
        name="goal", vertices=((80, -60), (250, -60), (250, 60), (80, 60)),
        actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}),
        alliance="red",
    )
    match = make_match(make_field(scoring_regions=(goal,)))
    ours = match.add_robot(make_characteristics(), Pose2d(0, 0, 0), alliance="red")
    theirs = match.add_robot(make_characteristics(), Pose2d(0, 40, 0), alliance="blue")

    assert world_view.scoring_slots_for_type(match, ours, WIDGET)
    assert world_view.scoring_slots_for_type(match, theirs, WIDGET) == []


# --- collect_outcomes ------------------------------------------------


def test_collect_outcomes_cover_stations_and_loose_pieces():
    match = make_match()
    robot = match.add_robot(make_characteristics(piece_capacity=2), Pose2d(0, 0, 0))
    loose = match.spawn_piece(WIDGET, (100, 100))

    outcomes = utility.collect_outcomes(match, robot)
    assert [o.kind for o in outcomes] == ["collect", "collect"]
    # Stations first, then loose pieces -- both are priced, neither wins
    # by being enumerated first.
    station, field_piece = outcomes
    assert station.payload is match.field.intake_locations[0]
    assert field_piece.payload is loose
    assert station.points == field_piece.points == 0.0


def test_collect_duration_uses_the_right_intake_timing():
    """A station handoff and scooping a piece off the floor are timed by
    different characteristics fields; collect_outcomes must not collapse
    them into one."""
    match = make_match()
    characteristics = make_characteristics(station_intake_time=5.0, intake_time=0.25)
    robot = match.add_robot(characteristics, Pose2d(0, 0, 0))
    loose = match.spawn_piece(WIDGET, (100, 100))

    station, field_piece = utility.collect_outcomes(match, robot)
    assert station.duration - travel_to(match, robot, (10.0, 0.0)) == 5.0
    assert field_piece.duration - travel_to(match, robot, (100, 100)) == robot.duration_for(loose) == 0.25


def test_collect_enables_the_best_deposit_valued_from_the_pickup():
    """The lookahead is priced from where the piece would be picked up,
    not from where the robot stands -- that is the whole point of it."""
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))
    station = utility.collect_outcomes(match, robot)[0]

    assert station.enables is not None
    assert station.enables.kind == "score"
    assert station.enables.payload.region.name == "goal"
    # Priced from the feeder's centroid, not the robot's pose. Same
    # place here, so pin the stronger claim: it equals what asking
    # directly from that centroid gives.
    direct = utility.best_score_for_type(match, robot, WIDGET, (10.0, 0.0))
    assert station.enables.duration == direct.duration


def test_collect_enables_carries_no_physical_piece():
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))
    station = utility.collect_outcomes(match, robot)[0]
    # The piece has not been collected -- inventing one would make the
    # option look executable to a tactic that received it.
    assert station.enables.payload.piece is None


def test_collect_enables_is_none_when_there_is_nowhere_to_put_it():
    match = make_match(make_field(scoring_regions=()))
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))
    station = utility.collect_outcomes(match, robot)[0]
    assert station.enables is None


def test_collect_prefers_the_deposit_worth_more_per_second():
    cheap = ScoringRegion(
        name="cheap", vertices=((80, -60), (120, -60), (120, 60), (80, 60)),
        actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}),
    )
    rich = ScoringRegion(
        name="rich", vertices=((80, 80), (120, 80), (120, 120), (80, 120)),
        actions=frozenset({"jackpot"}), piece_types=frozenset({WIDGET}),
    )
    match = Match(
        make_field(scoring_regions=(cheap, rich)),
        TableScoringRules({("score_widget", "auto"): 3.0, ("jackpot", "auto"): 30.0}),
        MatchConfig(auto_duration=1000, teleop_duration=1000),
    )
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))
    station = utility.collect_outcomes(match, robot)[0]
    assert station.enables.payload.region.name == "rich"


def test_collect_skips_pieces_there_is_no_room_for():
    match = make_match()
    robot = match.add_robot(make_characteristics(piece_capacity=1), Pose2d(0, 0, 0))
    match.spawn_piece(WIDGET, (100, 100))
    assert utility.collect_outcomes(match, robot)  # room while empty-handed

    give(match, robot)
    # Full: the station is dropped by station_options, the loose piece by
    # our own capacity check. Neither could ever complete.
    assert utility.collect_outcomes(match, robot) == []


def test_collect_outcomes_empty_when_no_supply_exists():
    match = make_match(make_field(intake_locations=()))
    robot = match.add_robot(make_characteristics(), Pose2d(0, 0, 0))
    assert utility.collect_outcomes(match, robot) == []
