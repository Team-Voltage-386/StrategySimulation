import math

from common_sim.control import tactics
from common_sim.control.behavior import BehaviorContext, Sequence, Status, Wait
from common_sim.control.navigation import convex_overlap, footprint_polygon
from common_sim.field.field_config import FieldConfig, IntakeLocation, Obstacle, ScoringRegion
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
        piece_type=WIDGET, starting_pieces=5,
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


# A REEFSCAPE REEF face in miniature: a solid structure, with a scoring
# zone only as deep as the real one (10in) sitting against its face. A
# robot that aims its chassis *center* at that zone is asking to put
# half its own length inside the structure.
STRUCTURE = Obstacle(name="structure", vertices=((140, 60), (200, 60), (200, 140), (140, 140)))
FACE_REGION = ScoringRegion(
    name="face", vertices=((130, 89), (140, 89), (140, 111), (130, 111)),
    actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}),
)


def test_score_stands_off_a_structure_instead_of_driving_into_it():
    field = make_field(scoring_regions=(FACE_REGION,), obstacles=(STRUCTURE,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    piece = match.spawn_piece(WIDGET, (20, 100))
    piece.held_by = robot
    piece.last_holder_alliance = robot.alliance
    robot.held_pieces.append(piece)

    tactic = tactics.Score()
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    status = Status.RUNNING
    for _ in range(2000):
        status = tactic.tick(ctx)
        match.step(ctx.dt)
        ctx.elapsed += ctx.dt
        chassis = footprint_polygon(
            (robot.pose.x, robot.pose.y), robot.pose.heading,
            robot.characteristics.width, robot.characteristics.length,
        )
        assert not convex_overlap(chassis, STRUCTURE.vertices), "drove into the structure it was scoring on"
        if status != Status.RUNNING:
            break

    assert status == Status.SUCCESS
    assert piece.scored


def test_collect_stands_off_a_piece_lying_against_a_structure():
    # REEFSCAPE spawns ALGAE about 7in off the REEF wall. Aiming the
    # chassis *center* at one asks for half the robot's length inside
    # the structure -- and a goal that deep inside the structure's
    # robot-radius inflation also caps how much clearance the planner is
    # allowed to keep on the way there, so it clips the corner en route.
    field = make_field(obstacles=(STRUCTURE,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 40, 0))
    piece = match.spawn_piece(WIDGET, (133, 100))  # 7in off the structure's near face

    tactic = tactics.Collect(piece_type=WIDGET)
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic.tick(ctx)  # picks the piece and builds a target for it
    target = tactic._provide_target(ctx)

    chassis = footprint_polygon(
        (target.x, target.y), target.heading,
        robot.characteristics.width, robot.characteristics.length,
    )
    assert not convex_overlap(chassis, STRUCTURE.vertices), "target pose puts the chassis in the structure"

    # ...and it's still a pose that can actually pick the piece up: the
    # piece sits outside the chassis but inside the intake's reach.
    half_extent = robot.characteristics.length / 2.0
    reach = math.hypot(piece.position.x - target.x, piece.position.y - target.y)
    assert half_extent <= reach <= half_extent + robot.characteristics.intake_range

    # And end to end it does collect it.
    status = run(match, tactic, robot)
    assert status == Status.SUCCESS
    assert len(robot.held_pieces) == 1


def test_score_keeps_moving_when_its_target_shifts_every_tick():
    # Score's target is derived from the robot's own live pose, so it
    # drifts a little every tick and every tick triggers a replan. A
    # replan restarts the path at waypoint 0, so anything that advances
    # only one waypoint per tick pins the robot at waypoint 1 -- which,
    # right at an inflated obstacle's corner, is inches from where it
    # already stands. It sat there for seconds at a time.
    field = make_field(scoring_regions=(FACE_REGION,), obstacles=(STRUCTURE,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    # Behind the structure, so the route has to round a corner and the
    # path has an intermediate waypoint at all.
    robot = match.add_robot(make_characteristics(), Pose2d(250, 30, 0))
    piece = match.spawn_piece(WIDGET, (250, 30))
    piece.held_by = robot
    piece.last_holder_alliance = robot.alliance
    robot.held_pieces.append(piece)

    tactic = tactics.Score()
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    start = (robot.pose.x, robot.pose.y)
    for _ in range(120):  # 2s -- ample to clear the corner it used to stall on
        tactic.tick(ctx)
        match.step(ctx.dt)
        ctx.elapsed += ctx.dt

    travelled = math.hypot(robot.pose.x - start[0], robot.pose.y - start[1])
    assert travelled > 40.0, f"only travelled {travelled:.1f}in in 2s -- stalled on a waypoint"


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


GADGET = "gadget"


def _hold(match, robot, piece_type):
    """Put a piece in the robot's hands without making it drive there."""
    piece = match.spawn_piece(piece_type, (robot.pose.x, robot.pose.y))
    piece.held_by = robot
    robot.held_pieces.append(piece)
    return piece


def _two_type_characteristics(**overrides):
    return make_characteristics(
        piece_capacity_by_type={WIDGET: 1, GADGET: 1},
        accepted_piece_types=frozenset({WIDGET, GADGET}),
        **overrides,
    )


def test_collect_keeps_working_while_holding_a_type_it_cannot_score():
    """Capacity is per piece type, so a robot full of one type is not
    full. A coral cycler that scooped an ALGAE it has no rule to score
    still has its coral slot free -- counting both against one shared
    limit called it full, so it stopped collecting, stopped driving, and
    sat out the rest of the match holding one algae."""
    match = make_match(auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(_two_type_characteristics(), Pose2d(20, 100, 0))
    _hold(match, robot, GADGET)
    match.spawn_piece(WIDGET, (60, 100))

    assert not robot.is_full_for(WIDGET), "gadget in hand should not fill the widget slot"
    assert not robot.is_full_for(), "still has room for a widget, so not full of everything"

    assert run(match, tactics.Collect(piece_type=WIDGET), robot) == Status.SUCCESS
    assert any(p.piece_type == WIDGET for p in robot.held_pieces)


def test_collect_stops_when_full_of_the_type_it_wants():
    match = make_match(auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(_two_type_characteristics(), Pose2d(20, 100, 0))
    _hold(match, robot, WIDGET)
    match.spawn_piece(WIDGET, (60, 100))

    assert robot.is_full_for(WIDGET)
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    assert tactics.Collect(piece_type=WIDGET).tick(ctx) == Status.SUCCESS


def test_full_of_every_type_is_full_untyped():
    match = make_match(auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(_two_type_characteristics(), Pose2d(20, 100, 0))
    _hold(match, robot, WIDGET)
    _hold(match, robot, GADGET)
    assert robot.is_full_for()


def test_shared_pool_capacity_still_fills_on_any_type():
    """Without piece_capacity_by_type the capacity really is one shared
    pool, and that legacy meaning is unchanged."""
    match = make_match(auto_duration=1000, teleop_duration=1000)
    characteristics = make_characteristics(
        piece_capacity=1, accepted_piece_types=frozenset({WIDGET, GADGET}))
    robot = match.add_robot(characteristics, Pose2d(20, 100, 0))
    _hold(match, robot, GADGET)

    assert robot.is_full_for(WIDGET)
    assert robot.is_full_for()


# Two feeders, each 36x36 -- barely bigger than the 28x28 robot, so
# region_robot_capacity is 1 apiece. This is the REEFSCAPE CORAL STATION
# geometry in miniature: the corner fits exactly one robot.
NEAR_FEEDER = IntakeLocation(
    name="near_feeder", vertices=((42, 82), (78, 82), (78, 118), (42, 118)),
    piece_type=WIDGET, starting_pieces=5,
)
FAR_FEEDER = IntakeLocation(
    name="far_feeder", vertices=((42, 22), (78, 22), (78, 58), (42, 58)),
    piece_type=WIDGET, starting_pieces=5,
)


def _collect_tactic_at_stations(match, robot, feeders=(NEAR_FEEDER, FAR_FEEDER)):
    tactic = tactics.Collect(piece_type=WIDGET, prefer_station=True)
    tactic.tick(BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match))
    return tactic


def test_collect_picks_another_station_when_the_nearest_is_taken():
    field = make_field(intake_locations=(NEAR_FEEDER, FAR_FEEDER))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    partner = match.add_robot(make_characteristics(), Pose2d(20, 130, 0))
    partner.controller = _FakeController(target_region="near_feeder")

    tactic = _collect_tactic_at_stations(match, robot)
    assert tactic._target_station.name == "far_feeder"


def test_collect_takes_the_nearest_station_when_nobody_is_on_it():
    field = make_field(intake_locations=(NEAR_FEEDER, FAR_FEEDER))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))

    tactic = _collect_tactic_at_stations(match, robot)
    assert tactic._target_station.name == "near_feeder"


def test_collect_waits_clear_of_a_station_it_cannot_share():
    # Only one feeder, and it's taken: keep it as the target (it will
    # free up) but hold a full footprint back rather than driving into
    # the robot being served.
    field = make_field(intake_locations=(NEAR_FEEDER,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    partner = match.add_robot(make_characteristics(), Pose2d(60, 100, 0))
    partner.controller = _FakeController(target_region="near_feeder")

    tactic = _collect_tactic_at_stations(match, robot)
    assert tactic._target_station.name == "near_feeder"

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    waiting = tactic._provide_target(ctx)
    assert waiting.distance_to(partner.pose) >= 28.0

    # ...and closes in once the partner is done with it.
    partner.controller = _FakeController(target_region=None)
    match.robots.remove(partner)
    serving = tactic._provide_target(ctx)
    assert serving.distance_to(Pose2d(60, 100, 0)) < waiting.distance_to(Pose2d(60, 100, 0))


def test_collect_does_not_yield_a_station_it_is_already_being_served_by():
    # The robot queueing outside claims the station too, so the incumbent
    # reads it as crowded. If it deferred to that, the two would swap
    # places forever and neither would ever collect.
    field = make_field(intake_locations=(NEAR_FEEDER, FAR_FEEDER))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    tactic = _collect_tactic_at_stations(match, robot)
    assert tactic._target_station.name == "near_feeder"

    # Drive it onto the feeder, then have a partner queue up behind it.
    robot.chassis.body.position = (60, 100)
    queuer = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    queuer.controller = _FakeController(target_region="near_feeder")

    assert not tactic._better_station_exists(BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match))


def test_collect_leaves_a_blocked_station_for_a_free_one():
    # Committed to a station an opponent then parks in: waiting is
    # pointless while the other feeder is open.
    field = make_field(intake_locations=(NEAR_FEEDER, FAR_FEEDER))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    tactic = _collect_tactic_at_stations(match, robot)
    assert tactic._target_station.name == "near_feeder"

    blocker = match.add_robot(make_characteristics(), Pose2d(60, 100, 0), alliance="red")
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    assert tactic._better_station_exists(ctx)

    tactic.tick(ctx)
    assert tactic._target_station.name == "far_feeder"
