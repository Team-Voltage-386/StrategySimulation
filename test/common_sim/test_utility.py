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


def test_reliability_scales_the_type_by_the_action():
    """Two axes that have to stay independent: the type says how good
    this robot's mechanism for that game piece is, the action says how
    hard that particular target is. A REEF branch and a trough take the
    same CORAL, so a per-type number alone cannot tell them apart."""
    characteristics = make_characteristics(
        scoring_reliability_by_type={WIDGET: 0.8},
        scoring_reliability_by_action={"score_widget": 0.5},
    )
    assert characteristics.reliability_for(WIDGET, "score_widget") == 0.4
    # An action nobody configured does not scale its type at all, and
    # asking without an action is the old per-type question unchanged.
    assert characteristics.reliability_for(WIDGET, "something_else") == 0.8
    assert characteristics.reliability_for(WIDGET) == 0.8
    assert make_characteristics().reliability_for(WIDGET, "score_widget") == 1.0


def test_score_outcomes_charge_the_action_they_would_attempt():
    """The number a tactic plans on has to be the number the sim rolls
    against, or every context weight built on it is measuring a
    different robot than the one playing."""
    match = make_match()
    characteristics = make_characteristics(
        scoring_reliability_by_type={WIDGET: 0.8},
        scoring_reliability_by_action={"score_widget": 0.5},
    )
    robot = match.add_robot(characteristics, Pose2d(0, 0, 0))
    give(match, robot)
    outcome = utility.score_outcomes(match, robot)[0]
    assert outcome.success_probability == 0.4
    assert outcome.success_probability == characteristics.reliability_for(WIDGET, outcome.payload.action)


def test_the_sim_rolls_against_the_action_being_attempted():
    """`piece.target_action` is settled by the time a deposit completes
    -- it is what `_try_score` scores against -- so the roll can and
    must use it."""
    match = make_match()
    robot = match.add_robot(
        make_characteristics(scoring_reliability_by_action={"sure_thing": 1.0, "never": 0.0}),
        Pose2d(0, 0, 0),
    )
    piece = match.spawn_piece(WIDGET, (0, 0))

    piece.target_action = "never"
    assert match._roll_scoring_success(robot, piece) is False
    piece.target_action = "sure_thing"
    assert match._roll_scoring_success(robot, piece) is True


# --- expected_rate ---------------------------------------------------


def _outcome(points, duration, probability):
    return utility.Outcome(
        kind="score", label="x", points=points, duration=duration,
        success_probability=probability, payload=None,
    )


def test_expected_rate_discounts_by_how_often_the_attempt_lands():
    """The ranking currency. `value_rate` stays beside it as the gross
    number, so anything reporting on a candidate can still say what the
    target pays separately from this robot's odds of collecting it."""
    sure = _outcome(6.0, 2.0, 1.0)
    assert sure.expected_rate == sure.value_rate == 3.0

    flaky = _outcome(6.0, 2.0, 0.5)
    assert flaky.expected_rate == 1.5
    assert flaky.value_rate == 3.0  # gross is untouched

    # The same 1e-6 floor: a free attempt is very good, not a crash.
    assert _outcome(1.0, 0.0, 0.5).expected_rate == 0.5 / 1e-6
    # A target this robot never lands is worth nothing, however much it
    # pays -- which is the whole difference from `value_rate`.
    assert _outcome(100.0, 1.0, 0.0).expected_rate == 0.0


# --- score_outcome (one already-chosen candidate) --------------------


def test_score_outcome_prices_one_named_legal_option():
    """The Outcome-currency twin of `build_option`, for a caller that
    has already narrowed to one candidate and still needs it priced the
    way the planner priced it -- same numbers, same reliability."""
    match = make_match()
    characteristics = make_characteristics(scoring_reliability_by_action={"score_widget": 0.6})
    robot = match.add_robot(characteristics, Pose2d(0, 0, 0))
    give(match, robot)

    legal = world_view.scoring_options(match, robot)[0]
    priced = utility.score_outcome(match, robot, legal, (0.0, 0.0))
    assert priced == utility.score_outcomes(match, robot)[0]
    assert priced.success_probability == 0.6
    assert priced.payload == utility.build_option(match, robot, legal, (0.0, 0.0))


# --- ranking on expected points --------------------------------------


def _two_action_match():
    """One region, two ways to use it: `rich` pays more per second than
    `cheap` on gross points, and less once the misses are counted. The
    REEFSCAPE L3-vs-L4 shape, which is where this actually bites."""
    goal = ScoringRegion(
        name="goal", vertices=((380, -60), (420, -60), (420, 60), (380, 60)),
        actions=frozenset({"cheap", "rich"}), piece_types=frozenset({WIDGET}),
    )
    match = Match(
        FieldConfig(width=500, height=200, scoring_regions=(goal,)),
        TableScoringRules({("cheap", "auto"): 4.0, ("rich", "auto"): 5.0}),
        MatchConfig(auto_duration=1000, teleop_duration=1000),
    )
    return match


def _two_action_characteristics(**overrides):
    return make_characteristics(
        deposit_time_by_action={"cheap": 1.0, "rich": 1.8},
        **overrides,
    )


def test_the_lookahead_payoff_is_ranked_on_expected_points():
    """`best_score_for_type` is what a pickup is worth going to get, so
    it has to name the deposit the robot will actually make when it
    arrives -- the expected-points one. Ranking the payoff on gross
    points prices the fetch against a plan nobody follows."""
    match = _two_action_match()
    misses = match.add_robot(
        _two_action_characteristics(scoring_reliability_by_action={"cheap": 0.9, "rich": 0.82}),
        Pose2d(0, 0, 0),
    )
    assert utility.best_score_for_type(match, misses, WIDGET, (0.0, 0.0)).payload.action == "cheap"

    # The flip is the reliability and nothing else: same geometry, same
    # points, a robot that never misses takes the richer target. If this
    # one ever fails, the fixture's travel time has drifted out of the
    # window where the two rankings disagree at all, and the assertion
    # above has stopped testing anything.
    never_misses = match.add_robot(_two_action_characteristics(), Pose2d(0, 0, 0))
    assert utility.best_score_for_type(match, never_misses, WIDGET, (0.0, 0.0)).payload.action == "rich"
