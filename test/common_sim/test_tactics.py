from common_sim.control import tactics
from common_sim.control.behavior import BehaviorContext, Sequence, Status, Wait
from common_sim.field.field_config import FieldConfig, IntakeLocation, ScoringRegion
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.match.scoring import TableScoringRules
from common_sim.robot.characteristics import RobotCharacteristics

WIDGET = "widget"


def make_field(**kwargs) -> FieldConfig:
    region = ScoringRegion(
        name="goal", vertices=((80, 40), (250, 40), (250, 160), (80, 160)),
        actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}),
    )
    kwargs.setdefault("scoring_regions", (region,))
    return FieldConfig(width=300, height=200, **kwargs)


def make_characteristics(**overrides) -> RobotCharacteristics:
    defaults = dict(
        name="test-bot", max_speed=150.0, max_accel=400.0, max_angular_speed=6.0,
        max_angular_accel=20.0, width=28.0, length=28.0, piece_capacity=1,
        intake_time=0.1, deposit_time=0.1, intake_range=6.0,
        accepted_piece_types=frozenset({WIDGET}),
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)


def make_match(field=None, **config_overrides):
    field = field or make_field()
    rules = TableScoringRules({("score_widget", "auto"): 3.0, ("score_widget", "teleop"): 1.0})
    return Match(field, rules, MatchConfig(**config_overrides))


def run(match, tactic, robot, ticks=1500, dt=1.0 / 60.0):
    ctx = BehaviorContext(robot=robot, dt=dt, match=match)
    status = Status.RUNNING
    for _ in range(ticks):
        ctx.dt = dt
        status = tactic.tick(ctx)
        match.step(dt)
        ctx.elapsed += dt
        if status != Status.RUNNING:
            break
    return status


def test_collect_picks_nearest_and_succeeds():
    match = make_match(auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    match.spawn_piece(WIDGET, (40, 100))
    match.spawn_piece(WIDGET, (280, 100))

    tactic = tactics.Collect(piece_type=WIDGET)
    status = run(match, tactic, robot)

    assert status == Status.SUCCESS
    assert len(robot.held_pieces) == 1


def test_collect_fails_when_nothing_collectable():
    match = make_match(auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))

    tactic = tactics.Collect(piece_type=WIDGET)
    status = run(match, tactic, robot, ticks=10)
    assert status == Status.FAILURE


def test_collect_prefers_station_when_configured():
    # Station centered well away from the robot's start pose -- a robot
    # spawned already overlapping a sensor zone never gets a collision
    # "begin" event (nothing transitioned into contact), so the test
    # needs Collect to actually drive it there.
    station = IntakeLocation(
        name="feeder", vertices=((60, 90), (80, 90), (80, 110), (60, 110)),
        piece_type=WIDGET, dispense_time=0.1, starting_pieces=5,
    )
    field = make_field(intake_locations=(station,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))

    tactic = tactics.Collect(piece_type=WIDGET, prefer_station=True)
    status = run(match, tactic, robot)
    assert status == Status.SUCCESS
    assert len(robot.held_pieces) == 1


def test_score_deposits_held_piece():
    match = make_match(auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    piece = match.spawn_piece(WIDGET, (20, 100))
    piece.held_by = robot
    piece.last_holder_alliance = robot.alliance
    robot.held_pieces.append(piece)

    tactic = tactics.Score()
    status = run(match, tactic, robot, ticks=2000)

    assert status == Status.SUCCESS
    assert piece.scored
    assert match.scores.get("blue", 0.0) > 0


def test_score_pinned_region_and_action():
    region = ScoringRegion(
        name="reef", vertices=((80, 40), (250, 40), (250, 160), (80, 160)),
        actions=frozenset({"l1", "l4"}), piece_types=frozenset({WIDGET}),
    )
    field = make_field(scoring_regions=(region,))
    rules = TableScoringRules({("l1", "auto"): 2.0, ("l4", "auto"): 7.0})
    match = Match(field, rules, MatchConfig(auto_duration=1000, teleop_duration=1000))
    robot = match.add_robot(make_characteristics(deposit_time_by_action={"l1": 0.1, "l4": 0.1}), Pose2d(20, 100, 0))
    piece = match.spawn_piece(WIDGET, (20, 100))
    piece.held_by = robot
    piece.last_holder_alliance = robot.alliance
    robot.held_pieces.append(piece)

    tactic = tactics.Score(region="reef", action="l1")
    status = run(match, tactic, robot, ticks=2000)

    assert status == Status.SUCCESS
    assert piece.scored
    assert piece.target_action == "l1"
    assert match.scores["blue"] == 2.0


# Two same-size scoring regions, each barely bigger than one 28x28
# robot, one nearer the robot's start than the other -- so "best value"
# and "has room" can disagree.
NEAR_REGION = ScoringRegion(
    name="near", vertices=((100, 90), (120, 90), (120, 110), (100, 110)),
    actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}),
)
FAR_REGION = ScoringRegion(
    name="far", vertices=((100, 150), (120, 150), (120, 170), (100, 170)),
    actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}),
)


def _score_tactic_holding_piece(match, robot, **kwargs):
    piece = match.spawn_piece(WIDGET, (robot.pose.x, robot.pose.y))
    piece.held_by = robot
    piece.last_holder_alliance = robot.alliance
    robot.held_pieces.append(piece)
    tactic = tactics.Score(**kwargs)
    tactic.tick(BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match))
    return tactic


def test_score_prefers_nearest_region_when_uncontested():
    # Control for the test below: nothing about crowding is in play, so
    # the planner's own "best value rate" choice stands.
    field = make_field(scoring_regions=(NEAR_REGION, FAR_REGION))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))

    tactic = _score_tactic_holding_piece(match, robot)
    assert tactic._current.region.name == "near"


def test_score_picks_another_region_when_the_best_one_is_taken():
    # A region only one robot fits in, already claimed by a robot that's
    # on its way there: going anyway just puts the two of them nose to
    # nose on the same spot, so take the farther (still free) one.
    field = make_field(scoring_regions=(NEAR_REGION, FAR_REGION))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    partner = match.add_robot(make_characteristics(), Pose2d(20, 40, 0))
    partner.controller = _FakeController(target_region="near")

    tactic = _score_tactic_holding_piece(match, robot)
    assert tactic._current.region.name == "far"


def test_score_shares_a_region_with_room_for_both():
    # The default "goal" region is 170x120 -- several robots wide. A
    # claim on it shouldn't push anyone away, but the two robots must
    # still aim at different spots within it.
    match = make_match(auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    partner = match.add_robot(make_characteristics(), Pose2d(165, 100, 0))  # region centroid
    partner.controller = _FakeController(target_region="goal")

    tactic = _score_tactic_holding_piece(match, robot)
    assert tactic._current.region.name == "goal"

    target = tactic._provide_target(BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match))
    assert target.distance_to(partner.pose) >= 28.0


def test_score_contests_a_full_region_rather_than_stalling():
    # Every region taken -- doubling up still eventually scores; waiting
    # forever holding the piece never does.
    field = make_field(scoring_regions=(NEAR_REGION,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    partner = match.add_robot(make_characteristics(), Pose2d(20, 40, 0))
    partner.controller = _FakeController(target_region="near")

    tactic = _score_tactic_holding_piece(match, robot)
    assert tactic._current is not None
    assert tactic._current.region.name == "near"


def test_idle_never_terminates():
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Idle()
    for _ in range(20):
        assert tactic.tick(ctx) == Status.RUNNING


def test_run_script_wraps_sequence_and_completes():
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.RunScript([Wait(0.05), Wait(0.05)])
    status = Status.RUNNING
    for _ in range(20):
        status = tactic.tick(ctx)
        ctx.elapsed += ctx.dt
        if status != Status.RUNNING:
            break
    assert status == Status.SUCCESS


class _FakeIntent:
    def __init__(self, target_region):
        self.target_region = target_region


class _FakeController:
    def __init__(self, target_region):
        self.intent = _FakeIntent(target_region)

    def tick(self, ctx):
        pass


def test_defend_positions_between_opponent_and_its_intent():
    match = make_match(auto_duration=1000, teleop_duration=1000)
    defender = match.add_robot(make_characteristics(), Pose2d(150, 100, 0), alliance="blue")
    opponent = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="red")
    opponent.controller = _FakeController(target_region="goal")

    tactic = tactics.Defend(target="opponent_intent", standoff=24.0)
    ctx = BehaviorContext(robot=defender, dt=1.0 / 60.0, match=match)
    for _ in range(600):
        status = tactic.tick(ctx)
        match.step(ctx.dt)
        ctx.elapsed += ctx.dt
        assert status == Status.RUNNING  # Defend never terminates

    region_center = (165.0, 100.0)  # centroid of the "goal" region
    dist_to_region_center = defender.pose.distance_to(Pose2d(*region_center, 0))
    assert abs(dist_to_region_center - 24.0) < 5.0


def test_defend_explicit_region_ignores_opponent_intent():
    match = make_match(auto_duration=1000, teleop_duration=1000)
    defender = match.add_robot(make_characteristics(), Pose2d(150, 100, 0), alliance="blue")
    match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="red")

    tactic = tactics.Defend(target="goal", standoff=24.0)
    ctx = BehaviorContext(robot=defender, dt=1.0 / 60.0, match=match)
    for _ in range(600):
        status = tactic.tick(ctx)
        match.step(ctx.dt)
        ctx.elapsed += ctx.dt
        assert status == Status.RUNNING

    region_center = (165.0, 100.0)
    dist_to_region_center = defender.pose.distance_to(Pose2d(*region_center, 0))
    assert abs(dist_to_region_center - 24.0) < 5.0
