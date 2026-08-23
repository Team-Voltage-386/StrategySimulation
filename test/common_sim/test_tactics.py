import math

from common_sim.control import tactics
from common_sim.control.behavior import BehaviorContext, Sequence, Status, Wait
from common_sim.control.navigation import convex_overlap, footprint_polygon
from common_sim.control.strategy import Strategy, StrategyController
from common_sim.field.field_config import (
    FieldConfig, IntakeLocation, Obstacle, ScoringRegion, polygon_centroid,
)
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


def test_collect_uses_station_when_it_is_the_only_option():
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

    tactic = tactics.Collect(piece_type=WIDGET)
    status = run(match, tactic, robot)
    assert status == Status.SUCCESS
    assert len(robot.held_pieces) == 1


def test_collect_picks_whichever_of_field_or_station_is_closer():
    # Station and loose piece both collectable -- "nearest" should mean
    # nearest of either, not station-always-first (that was the bug:
    # a robot would drive past a closer field piece to reach a station).
    near_station = IntakeLocation(
        name="near_feeder", vertices=((30, 90), (50, 90), (50, 110), (30, 110)),
        piece_type=WIDGET, starting_pieces=5,
    )
    far_station = IntakeLocation(
        name="far_feeder", vertices=((260, 90), (280, 90), (280, 110), (260, 110)),
        piece_type=WIDGET, starting_pieces=5,
    )

    field = make_field(intake_locations=(near_station,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    match.spawn_piece(WIDGET, (270, 100))  # much farther than the station

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(ctx)
    assert tactic._target_station is not None and tactic._target_station.name == "near_feeder"

    field = make_field(intake_locations=(far_station,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    match.spawn_piece(WIDGET, (40, 100))  # much closer than the station

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(ctx)
    assert tactic._target_piece is not None
    assert tactic._target_piece.position.x == 40


def test_collect_avoids_piece_a_closer_teammate_is_already_heading_for():
    # Two pieces, one right next to a teammate that's already declared it
    # as its intent -- going for that one too just means two robots idle
    # nose-to-nose on the same spot, so the far piece (which nothing else
    # wants) should win even though it's farther from us.
    field = make_field()
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    partner = match.add_robot(make_characteristics(), Pose2d(38, 100, 0))
    contested = match.spawn_piece(WIDGET, (40, 100))  # very close to us and to partner
    alternative = match.spawn_piece(WIDGET, (120, 100))  # far from everyone
    partner.controller = _FakeController(target_piece=contested)

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(ctx)
    assert tactic._target_piece is alternative


def test_collect_contests_a_claimed_piece_when_it_is_the_only_option():
    # Same setup, but with no alternative piece on the field -- standing
    # idle scores zero for certain, so contesting the one piece that
    # exists beats refusing to move.
    field = make_field()
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    partner = match.add_robot(make_characteristics(), Pose2d(38, 100, 0))
    contested = match.spawn_piece(WIDGET, (40, 100))
    partner.controller = _FakeController(target_piece=contested)

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(ctx)
    assert tactic._target_piece is contested


def test_collect_still_targets_a_piece_it_would_reach_first():
    # An opponent's claim on a piece isn't automatically off limits the
    # way a teammate's is -- if we're closer, there's no reason to give
    # up a piece we'd win the race for.
    field = make_field()
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="blue")
    opponent = match.add_robot(make_characteristics(), Pose2d(250, 100, 0), alliance="red")
    piece = match.spawn_piece(WIDGET, (40, 100))  # much closer to us than to the opponent
    opponent.controller = _FakeController(target_piece=piece)

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(ctx)
    assert tactic._target_piece is piece


def test_collect_leaves_a_teammates_piece_and_goes_to_a_farther_station():
    # The one piece on the field is a race we'd lose to a teammate, and
    # the only station is much farther off than the piece. Contesting on
    # ETA alone would still send us at the piece, which is exactly the
    # two-robots-on-one-piece pileup -- following a teammate in scores
    # nothing at all, so the station wins however far away it is.
    station = IntakeLocation(
        name="feeder", vertices=((260, 90), (280, 90), (280, 110), (260, 110)),
        piece_type=WIDGET, starting_pieces=5,
    )
    field = make_field(intake_locations=(station,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    partner = match.add_robot(make_characteristics(), Pose2d(38, 100, 0))
    contested = match.spawn_piece(WIDGET, (40, 100))
    partner.controller = _FakeController(target_piece=contested)

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(ctx)
    assert tactic._target_piece is None
    assert tactic._target_station is station


def test_collect_still_weighs_a_station_on_eta_against_an_opponents_piece():
    # Same shape, but the rival is an *opponent*: contesting a piece
    # they'd reach first is still real play (they may fumble it), so
    # this stays the plain ETA comparison rather than handing the
    # station an automatic win the way a teammate's claim does.
    station = IntakeLocation(
        name="feeder", vertices=((260, 90), (280, 90), (280, 110), (260, 110)),
        piece_type=WIDGET, starting_pieces=5,
    )
    field = make_field(intake_locations=(station,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="blue")
    opponent = match.add_robot(make_characteristics(), Pose2d(38, 100, 0), alliance="red")
    contested = match.spawn_piece(WIDGET, (40, 100))
    opponent.controller = _FakeController(target_piece=contested)

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(ctx)
    assert tactic._target_piece is contested


def _two_sided_field(**kwargs) -> FieldConfig:
    """A field with an owned goal at each end, which is what
    world_view.own_side_test reads the halves off -- blue's at low x,
    red's at high x, dividing line at x=150."""
    kwargs.setdefault("scoring_regions", (
        ScoringRegion(
            name="blue_goal", vertices=((10, 80), (50, 80), (50, 120), (10, 120)),
            actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}), alliance="blue",
        ),
        ScoringRegion(
            name="red_goal", vertices=((250, 80), (290, 80), (290, 120), (250, 120)),
            actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}), alliance="red",
        ),
    ))
    return FieldConfig(width=300, height=200, **kwargs)


def test_collect_takes_a_farther_own_side_piece_over_a_closer_opposing_one():
    # Nothing is contested here -- the near piece is simply parked on the
    # opponents' half. Crossing the field for it costs most of a cycle,
    # so the uncontested one back home wins despite being farther.
    match = make_match(_two_sided_field(), auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(160, 100, 0), alliance="blue")
    opposing = match.spawn_piece(WIDGET, (180, 100))
    own = match.spawn_piece(WIDGET, (60, 100))

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(ctx)
    assert tactic._target_piece is own

    # ...and with the gate off, the game says a cross-field piece is a
    # normal option, so the nearest one wins as it always did.
    tactic = tactics.Collect(piece_type=WIDGET, opposing_side="allow")
    tactic.tick(ctx)
    assert tactic._target_piece is opposing


def test_collect_crosses_the_field_when_its_own_half_is_empty():
    # The gate is "only if there's nothing else", not "never" -- with our
    # own half picked clean and no station to fall back on, the piece
    # across the field is better than standing still.
    match = make_match(_two_sided_field(), auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(100, 100, 0), alliance="blue")
    opposing = match.spawn_piece(WIDGET, (250, 100))

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(ctx)
    assert tactic._target_piece is opposing


def test_collect_prefers_a_station_over_a_piece_across_the_field():
    # With the own half empty but a station on it, the station is the
    # better job than the cross-field trip regardless of which is nearer
    # -- same rule that settles a teammate's claim.
    station = IntakeLocation(
        name="feeder", vertices=((10, 90), (30, 90), (30, 110), (10, 110)),
        piece_type=WIDGET, starting_pieces=5,
    )
    match = make_match(_two_sided_field(intake_locations=(station,)), auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(140, 100, 0), alliance="blue")
    match.spawn_piece(WIDGET, (160, 100))  # just over the line, far nearer than the station

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(ctx)
    assert tactic._target_piece is None
    assert tactic._target_station is station


def test_collect_prefers_station_over_a_piece_rolling_away():
    # The piece is closer right now, but it's rolling away fast enough
    # that the station -- farther off but standing still -- is actually
    # quicker to reach.
    station = IntakeLocation(
        name="feeder", vertices=((150, 90), (170, 90), (170, 110), (150, 110)),
        piece_type=WIDGET, starting_pieces=5,
    )
    field = make_field(intake_locations=(station,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    piece = match.spawn_piece(WIDGET, (60, 100))
    piece.body.velocity = (140.0, 0.0)  # rolling away, almost as fast as the robot can chase

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(ctx)
    assert tactic._target_station is not None and tactic._target_station.name == "feeder"


def test_collect_abandons_a_far_piece_needing_a_detour_once_a_near_one_appears():
    # Only one piece exists at pick time, so it's targeted even though a
    # wall sits directly between the robot and it -- but once a second,
    # unobstructed piece spawns much closer, the real (obstacle-routed)
    # travel time to the far one should lose out on the next
    # reconsideration instead of the robot staying committed to it for
    # the rest of the match.
    wall = Obstacle(name="wall", vertices=((90, 60), (150, 60), (150, 140), (90, 140)))
    field = make_field(obstacles=(wall,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    far_piece = match.spawn_piece(WIDGET, (200, 100))

    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match))
    assert tactic._target_piece is far_piece

    near_piece = match.spawn_piece(WIDGET, (40, 100))

    # The re-evaluation is throttled to replan_period (see
    # _reconsider_now), not run every tick.
    tactic.tick(BehaviorContext(robot=robot, dt=tactic.replan_period, match=match))
    assert tactic._target_piece is near_piece


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


def test_score_stops_once_the_pose_already_scores():
    """Entering a big region is arriving. A robot whose current pose
    already resolves to the target region must hold it and deposit, not
    keep driving to the nominal aim point -- leaving the region cancels
    the deposit it just started, so on a region much larger than the aim
    tolerance the robot drives in one side and out the other forever."""
    match = make_match(auto_duration=1000, teleop_duration=1000)
    # Just inside the goal's near edge (x=80), far from its centroid.
    robot = match.add_robot(make_characteristics(), Pose2d(95, 100, 0))
    tactic = _score_tactic_holding_piece(match, robot)
    assert tactic._current.region.name == "goal"

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    aim = tactic._provide_target(ctx)
    assert aim.distance_to(robot.pose) > 20.0  # it would otherwise drive on

    for _ in range(30):
        tactic.tick(ctx)
        match.step(ctx.dt)
        ctx.elapsed += ctx.dt

    assert robot.pose.distance_to(Pose2d(95, 100, 0)) < 2.0
    assert not robot.held_pieces  # deposited where it stood


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
    def __init__(self, target_region=None, target_piece=None):
        self.target_region = target_region
        self.target_piece = target_piece


class _FakeController:
    def __init__(self, target_region=None, target_piece=None):
        self.intent = _FakeIntent(target_region, target_piece)

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
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match))
    return tactic


def test_collect_picks_another_station_when_the_nearest_is_taken():
    field = make_field(intake_locations=(NEAR_FEEDER, FAR_FEEDER))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    # Closer to the near feeder than we are, so it really does get there
    # first -- following it in would just leave us queueing behind it.
    rival = match.add_robot(make_characteristics(), Pose2d(40, 100, 0))
    rival.controller = _FakeController(target_region="near_feeder")

    tactic = _collect_tactic_at_stations(match, robot)
    assert tactic._target_station.name == "far_feeder"


def test_collect_races_a_claim_it_can_beat_rather_than_conceding_the_station():
    """A declaration is not possession. A defender takes a station away
    by *arriving* at it; if it announces one from across the field and we
    are closer, conceding hands it a denial it never earned -- and on a
    two-station field that is the difference between one defender
    controlling both and controlling neither."""
    field = make_field(intake_locations=(NEAR_FEEDER, FAR_FEEDER))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    denier = match.add_robot(make_characteristics(), Pose2d(250, 100, 0), alliance="red")
    denier.controller = _FakeController(target_region="near_feeder")

    tactic = _collect_tactic_at_stations(match, robot)
    assert tactic._target_station.name == "near_feeder"

    # Once it has actually parked on the feeder, the race is over and
    # there is nothing to win by driving into it.
    denier.chassis.body.position = (60, 100)
    tactic.reset()
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


def test_collect_teammates_do_not_deadlock_on_a_single_remaining_station():
    # Two teammates approach a capacity-1 station at the same time -- the
    # only piece source left. Before racing intent-only claimants by ETA,
    # each robot read the *other's* mere intent as filling the one slot
    # and both backed off to queue outside forever, since neither was
    # ever physically at the station to break the tie: nobody collected.
    # The faster of the two should win the race and go in, the other
    # should queue and then get its turn once the first is done.
    station = IntakeLocation(
        name="feeder", vertices=((130, 90), (150, 90), (150, 110), (130, 110)),
        piece_type=WIDGET, starting_pieces=50,
    )
    field = make_field(intake_locations=(station,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot_a = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    robot_b = match.add_robot(make_characteristics(), Pose2d(20, 60, 0))

    strategy_a = Strategy(name="collect_only", fallback=tactics.Collect(piece_type=WIDGET))
    strategy_b = Strategy(name="collect_only", fallback=tactics.Collect(piece_type=WIDGET))
    robot_a.controller = StrategyController(strategy_a, robot_a)
    robot_b.controller = StrategyController(strategy_b, robot_b)

    dt = 1.0 / 60.0
    ctx_a = BehaviorContext(robot=robot_a, dt=dt, match=match)
    ctx_b = BehaviorContext(robot=robot_b, dt=dt, match=match)
    for _ in range(1800):  # 30s
        robot_a.controller.tick(ctx_a)
        robot_b.controller.tick(ctx_b)
        match.step(dt)
        ctx_a.elapsed += dt
        ctx_b.elapsed += dt

    assert len(robot_a.held_pieces) >= 1
    assert len(robot_b.held_pieces) >= 1


def test_collect_ignores_an_opponents_claim_on_a_station_it_cannot_use():
    """An opponent's *declared* intent must not cost us a slot.

    The capacity race exists so two teammates racing one feeder don't
    defer to each other forever. But it raced every claimant, and a
    defender that declares a feeder and parks near it -- never entering,
    so never `engaged` and never `_held_by_opponent` -- posts the
    shortest ETA and wins a race it has no intention of running. Both
    our robots then read the one slot as taken and back off a footprint
    to queue behind a robot that will never arrive.

    It cannot take the slot because it cannot take the piece: intake
    locations are alliance-scoped. Measured on block/any seed 5014, this
    froze both blue robots for 108 seconds over their last ALGAE and
    ended 56-point matches.
    """
    station = IntakeLocation(
        name="feeder", vertices=((130, 90), (150, 90), (150, 110), (130, 110)),
        piece_type=WIDGET, starting_pieces=50,
    )
    field = make_field(intake_locations=(station,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    tactic = _collect_tactic_at_stations(match, robot)
    assert tactic._target_station.name == "feeder"

    # Closer to the feeder than we are, so it wins any ETA race, but
    # clear of it -- the stance a denier actually takes.
    denier = match.add_robot(make_characteristics(), Pose2d(100, 100, 0), alliance="red")
    denier.controller = _FakeController(target_region="feeder")
    assert not tactics._robot_engaged_with_station(denier, station)

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    assert not tactic._held_by_opponent(ctx, station)
    assert tactic._station_has_room_for(ctx, station)
    # Room means aim for the feed itself, not one footprint short of it.
    aim_x, aim_y = tactic._station_aim(ctx)
    feed_x, feed_y = polygon_centroid(station.vertices)
    assert math.hypot(aim_x - feed_x, aim_y - feed_y) < 1e-6


def test_collect_still_yields_to_an_opponent_standing_on_the_feed():
    """The body still counts, only the announcement stopped counting.
    A defender actually parked in the station occupies real capacity,
    and leaving is then the right answer -- which is what
    `_held_by_opponent` is for."""
    station = IntakeLocation(
        name="feeder", vertices=((130, 90), (150, 90), (150, 110), (130, 110)),
        piece_type=WIDGET, starting_pieces=50,
    )
    field = make_field(intake_locations=(station,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    tactic = _collect_tactic_at_stations(match, robot)

    occupier = match.add_robot(make_characteristics(), Pose2d(140, 100, 0), alliance="red")
    assert tactics._robot_engaged_with_station(occupier, station)

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    assert not tactic._station_has_room_for(ctx, station)
    assert tactic._held_by_opponent(ctx, station)


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

    # The re-evaluation that acts on `_better_station_exists` is throttled
    # to `replan_period` (see `_reconsider_now`), not run every tick, so
    # advance by a full period rather than a single physics tick.
    tactic.tick(BehaviorContext(robot=robot, dt=tactic.replan_period, match=match))
    assert tactic._target_station.name == "far_feeder"


def test_collect_queues_behind_a_teammate_rather_than_behind_an_opponent():
    """With every feeder crowded, who is crowding it decides where to
    wait. A teammate on the feed leaves in a couple of seconds; an
    opponent is at a feeder it cannot use precisely in order to stay
    there. Nearest-of-the-crowded treats the two as the same thing."""
    field = make_field(intake_locations=(NEAR_FEEDER, FAR_FEEDER))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    match.add_robot(make_characteristics(), Pose2d(60, 100, 0), alliance="red")   # on near_feeder
    match.add_robot(make_characteristics(), Pose2d(60, 40, 0), alliance="blue")   # on far_feeder

    tactic = _collect_tactic_at_stations(match, robot)
    assert tactic._target_station.name == "far_feeder"


def _run_collect(tactic, match, robot, seconds, dt=1.0 / 60.0):
    ctx = BehaviorContext(robot=robot, dt=dt, match=match)
    for _ in range(int(seconds / dt)):
        tactic.tick(ctx)
        ctx.elapsed += dt


def test_collect_gives_up_a_station_that_never_delivers():
    """A defender that denies a station by standing in the *approach* is
    invisible to every instantaneous test: it is not on the feed, so the
    station never reads as full, and `_better_station_exists` never
    fires. Only elapsed time sees it -- and once the trip has overrun its
    budget, the other feeder is worth trying however far off it is.

    Staged by ticking the tactic without stepping physics, so the robot
    stays where it is: from inside the tactic that is indistinguishable
    from an approach it cannot make progress along, which is the whole
    point -- the only thing it can see is that time is passing."""
    field = make_field(intake_locations=(NEAR_FEEDER, FAR_FEEDER))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    tactic = _collect_tactic_at_stations(match, robot)
    assert tactic._target_station.name == "near_feeder"

    # Parked between the robot and the feeder but clear of the polygon,
    # the way a defender denying a station actually stands. Every
    # instantaneous test still says the station is available.
    match.add_robot(make_characteristics(), Pose2d(34, 100, 0), alliance="red")
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    assert tactic._station_has_room_for(ctx, NEAR_FEEDER)
    assert not tactic._better_station_exists(ctx)

    _run_collect(tactic, match, robot, tactics._STATION_PATIENCE_MIN + 1.0)
    assert tactic._target_station.name == "far_feeder"
    assert "near_feeder" in tactic._station_cooldowns


def test_collect_keeps_the_only_station_however_long_it_takes():
    """Giving up needs somewhere to give up to. With one feeder there is
    no alternative, so the escape must not fire and must not leave the
    robot with no target at all."""
    field = make_field(intake_locations=(NEAR_FEEDER,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    tactic = _collect_tactic_at_stations(match, robot)

    _run_collect(tactic, match, robot, tactics._STATION_PATIENCE_MIN + 1.0)
    assert tactic._target_station is not None
    assert not tactic._station_cooldowns


def test_collect_does_not_ping_pong_between_two_denied_stations():
    """Having given up on one feeder for the other, a robot must not give
    up on that one straight back again. The cooldown has to outlast the
    trip it sent us on, or the two clocks line up and the robot spends
    the match alternating -- measured at 96 of 150s on one seed."""
    field = make_field(intake_locations=(NEAR_FEEDER, FAR_FEEDER))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    tactic = _collect_tactic_at_stations(match, robot)
    assert tactic._target_station.name == "near_feeder"

    _run_collect(tactic, match, robot, tactics._STATION_PATIENCE_MIN + 1.0)
    assert tactic._target_station.name == "far_feeder"

    # Just as denied at the second feeder, and for just as long. The
    # first is still on cooldown, so there is nowhere fresh to go.
    _run_collect(tactic, match, robot, tactics._STATION_PATIENCE_MIN + 1.0)
    assert tactic._target_station.name == "far_feeder"


def test_collect_does_not_time_a_queue_behind_a_teammate():
    """Waiting behind a teammate on the feed is a queue that is moving,
    and it is priced into the trip already. At 3v3 -- three robots to two
    feeders -- that wait is almost all of the waiting there is, so timing
    it sends robots touring the field for nothing."""
    field = make_field(intake_locations=(NEAR_FEEDER, FAR_FEEDER))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    tactic = _collect_tactic_at_stations(match, robot)
    match.add_robot(make_characteristics(), Pose2d(60, 100, 0))  # teammate, on the feed

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    for _ in range(int((tactics._STATION_PATIENCE_MIN + 5.0) * 60)):
        assert not tactic._station_stalled(ctx)
    assert tactic._station_elapsed == 0.0


def test_collect_does_not_abandon_a_station_it_is_already_being_fed_by():
    """The clock stops at the feed. An intake under way always gets to
    finish -- the same rule Score follows for a deposit that is already
    legal -- or a slow feed would look identical to a denied trip."""
    field = make_field(intake_locations=(NEAR_FEEDER, FAR_FEEDER))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(60, 100, 0))
    tactic = _collect_tactic_at_stations(match, robot)
    assert tactic._target_station.name == "near_feeder"

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    assert tactic._holds_station(robot)
    for _ in range(int((tactics._STATION_PATIENCE_MIN + 5.0) * 60)):
        assert not tactic._station_stalled(ctx)


def test_collect_station_cooldown_survives_the_reset_between_cycles():
    """Collect is re-entered from scratch every cycle, so a cooldown
    cleared by `reset()` would be wiped before it was ever read -- which
    is exactly how Score's equivalent was a silent no-op the first time
    it was written."""
    field = make_field(intake_locations=(NEAR_FEEDER, FAR_FEEDER))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    tactic = _collect_tactic_at_stations(match, robot)
    match.add_robot(make_characteristics(), Pose2d(34, 100, 0), alliance="red")

    _run_collect(tactic, match, robot, tactics._STATION_PATIENCE_MIN + 1.0)
    assert "near_feeder" in tactic._station_cooldowns
    tactic.reset()
    assert "near_feeder" in tactic._station_cooldowns


def test_collect_gives_up_a_piece_that_rolls_well_onto_the_opposing_half():
    # Committed to the nearer of two pieces, which then gets knocked over
    # the line. Following it is the cross-field trip the gate exists to
    # refuse, so it's handed off to the one still on our own half -- but
    # only once it's clearly over, not the instant it crosses.
    match = make_match(_two_sided_field(), auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(100, 100, 0), alliance="blue")
    rolling = match.spawn_piece(WIDGET, (140, 100))
    stationary = match.spawn_piece(WIDGET, (40, 100))   # farther off, but safely home

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(ctx)
    assert tactic._target_piece is rolling

    # Just over the line (x=150) is still inside the 10% margin (~24in),
    # so a piece drifting around midfield doesn't flip this every replan.
    rolling.shape.body.position = (160, 100)
    for _ in range(20):
        tactic.tick(ctx)
    assert tactic._target_piece is rolling

    rolling.shape.body.position = (220, 100)
    for _ in range(20):
        tactic.tick(ctx)
    assert tactic._target_piece is stationary


def test_collect_gives_up_a_piece_it_cannot_get_any_closer_to():
    """The corner-piece stall: a piece at rest behind a parked defender is
    uncontested (a defender declares the robot it marks, not the piece) and
    routes as a few inches away, since `estimate_travel_time` never models
    robots. So every instantaneous release is satisfied while the robot
    goes nowhere -- measured at a 22s commitment, and 132s across 8 seeds.

    Staged by ticking without stepping physics, so the robot stays put: from
    inside the tactic that is exactly a trip making no progress, which is
    the only thing it can actually see."""
    match = make_match(auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    blocked = match.spawn_piece(WIDGET, (60, 100))
    reachable = match.spawn_piece(WIDGET, (240, 100))   # much farther, so only a give-up reaches it

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(ctx)
    assert tactic._target_piece is blocked

    _run_collect(tactic, match, robot, tactics._PIECE_PATIENCE_MIN + 1.0)
    assert tactic._target_piece is reachable
    assert blocked in tactic._piece_cooldowns


def test_collect_keeps_a_piece_it_is_still_closing_on():
    """The budget is spent on time making no progress, not elapsed time.
    A loose piece can legitimately be most of a cycle away -- tucked under
    field structure, out at a wall -- so plain elapsed time cannot tell
    "denied" from "far away and awkward". Measured: with elapsed time
    alone this cost 1.2 points on the *undefended* control, where every
    firing was a good trip thrown away."""
    match = make_match(auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    piece = match.spawn_piece(WIDGET, (280, 100))

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(ctx)
    assert tactic._target_piece is piece

    # Creeping in far slower than the budget would allow, but always
    # closing. `_PIECE_PROGRESS_EPSILON` is what it has to beat each time.
    for _ in range(int((tactics._PIECE_PATIENCE_MIN + 5.0) * 60)):
        robot.chassis.body.position = (robot.pose.x + 0.1, 100)
        assert not tactic._piece_stalled(ctx)


def test_collect_does_not_abandon_a_piece_already_in_its_intake():
    """The clock stops in intake range, so a capture under way always gets
    to finish -- the same rule the station escape follows at the feed."""
    match = make_match(auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    piece = match.spawn_piece(WIDGET, (34, 100))
    match.step(1.0 / 60.0)   # let the intake-range collision callbacks land

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(ctx)
    assert tactic._target_piece is piece
    assert robot.accepts(piece)

    for _ in range(int((tactics._PIECE_PATIENCE_MIN + 5.0) * 60)):
        assert not tactic._piece_stalled(ctx)
    assert tactic._piece_elapsed == 0.0


def test_collect_reports_failure_when_its_only_piece_is_one_it_gave_up_on():
    """A given-up piece is NOT re-offered when it is the last one -- the
    one place this deliberately differs from the station cooldown, whose
    fallback is to go back and wait. Waiting on an unreachable piece is the
    stall itself. Failing instead is what lets the *strategy* switch jobs,
    which no tactic can decide from inside its own scope."""
    match = make_match(auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    piece = match.spawn_piece(WIDGET, (60, 100))

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    _run_collect(tactic, match, robot, tactics._PIECE_PATIENCE_MIN + 1.0)
    assert piece in tactic._piece_cooldowns
    assert tactic.tick(ctx) is Status.FAILURE


def test_collect_piece_cooldown_survives_the_reset_between_cycles():
    """Same trap as the station cooldown: Collect is re-entered from
    scratch every cycle, so a cooldown cleared by `reset()` is wiped before
    it is ever read."""
    match = make_match(auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    piece = match.spawn_piece(WIDGET, (60, 100))
    match.spawn_piece(WIDGET, (240, 100))

    tactic = tactics.Collect(piece_type=WIDGET)
    _run_collect(tactic, match, robot, tactics._PIECE_PATIENCE_MIN + 1.0)
    assert piece in tactic._piece_cooldowns
    tactic.reset()
    assert piece in tactic._piece_cooldowns


class _DefenseIntent:
    def __init__(self, *, defending=True, marking=None, target_region=None):
        self.defending = defending
        self.marking = marking
        self.target_region = target_region
        self.target_piece = None


class _DefenseController:
    def __init__(self, intent):
        self.intent = intent

    def tick(self, ctx):
        pass


def _drive(match, tactic, robot, ticks, dt=1.0 / 60.0):
    ctx = BehaviorContext(robot=robot, dt=dt, match=match)
    for _ in range(ticks):
        tactic.tick(ctx)
        match.step(dt)
        ctx.elapsed += dt
    return ctx


def test_defend_publishes_the_robot_it_marked():
    match = make_match(auto_duration=1000, teleop_duration=1000)
    defender = match.add_robot(make_characteristics(), Pose2d(150, 100, 0), alliance="blue")
    opponent = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="red")
    opponent.controller = _FakeController(target_region="goal")

    tactic = tactics.Defend(target="opponent_intent")
    _drive(match, tactic, defender, 60)

    assert tactic.marked_robot is opponent
    assert tactic.target_region_name == "goal"


def test_defend_holds_its_mark_rather_than_chasing_whoever_is_nearest():
    """A defender that re-picks the nearest opponent every tick escorts
    the midpoint of the pair instead of denying either of them."""
    match = make_match(auto_duration=1000, teleop_duration=1000)
    defender = match.add_robot(make_characteristics(), Pose2d(150, 100, 0), alliance="blue")
    far = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="red")
    near = match.add_robot(make_characteristics(), Pose2d(140, 100, 0), alliance="red")
    for opponent in (far, near):
        opponent.controller = _FakeController(target_region="goal")

    tactic = tactics.Defend(target="opponent_intent")
    ctx = BehaviorContext(robot=defender, dt=1.0 / 60.0, match=match)
    tactic.tick(ctx)
    assert tactic.marked_robot is near  # nothing held yet: closest wins

    # Hand the far robot a piece: it becomes the bigger threat, but the
    # dwell window has to expire before the mark is allowed to move.
    far.held_pieces.append(match.spawn_piece(WIDGET, (20, 100)))
    for _ in range(30):  # 0.5s, inside _MARK_DWELL
        tactic.tick(ctx)
        match.step(ctx.dt)
    assert tactic.marked_robot is near

    for _ in range(180):  # past the dwell
        tactic.tick(ctx)
        match.step(ctx.dt)
    assert tactic.marked_robot is far


def test_defend_ignores_opponents_beyond_engage_range():
    match = make_match(auto_duration=1000, teleop_duration=1000)
    defender = match.add_robot(make_characteristics(), Pose2d(280, 100, 0), alliance="blue")
    opponent = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="red")
    opponent.controller = _FakeController(target_region="goal")

    tactic = tactics.Defend(target="opponent_intent", engage_range=50.0)
    _drive(match, tactic, defender, 60)
    assert tactic.marked_robot is None

    # With the range wide enough, the same robot is marked.
    reachable = tactics.Defend(target="opponent_intent", engage_range=400.0)
    _drive(match, reachable, defender, 60)
    assert reachable.marked_robot is opponent


def test_defend_shadow_mode_sits_on_the_mark_not_on_the_region():
    match = make_match(auto_duration=1000, teleop_duration=1000)
    defender = match.add_robot(make_characteristics(), Pose2d(150, 100, 0), alliance="blue")
    opponent = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="red")
    opponent.controller = _FakeController(target_region="goal")

    tactic = tactics.Defend(target="opponent_intent", mode="shadow", standoff=30.0)
    _drive(match, tactic, defender, 900)

    # "goal" centroid is (165, 100); the mark sits way back at x=20, so
    # block mode would park near the region and shadow parks by the mark.
    assert defender.pose.distance_to(Pose2d(20, 100, 0)) < 60.0
    assert defender.pose.x < 100.0


def test_defend_guesses_a_region_when_the_mark_has_declared_none():
    """A defender that waits for its mark to commit arrives after it."""
    match = make_match(auto_duration=1000, teleop_duration=1000)
    defender = match.add_robot(make_characteristics(), Pose2d(150, 100, 0), alliance="blue")
    opponent = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="red")

    tactic = tactics.Defend(target="opponent_intent")
    _drive(match, tactic, defender, 60)

    assert tactic.marked_robot is opponent
    assert tactic.target_region_name == "goal"


def _feeder_field():
    return make_field(intake_locations=(NEAR_FEEDER, FAR_FEEDER))


def test_defend_can_deny_a_named_intake_location():
    """A station is a polygon an opponent has to reach, exactly like a
    scoring region -- and in a game that protects its scoring zones it is
    the only one a defender may make contact at."""
    match = make_match(_feeder_field(), auto_duration=1000, teleop_duration=1000)
    defender = match.add_robot(make_characteristics(), Pose2d(150, 100, 0), alliance="blue")
    match.add_robot(make_characteristics(), Pose2d(20, 60, 0), alliance="red")

    tactic = tactics.Defend(target="near_feeder", standoff=24.0)
    _drive(match, tactic, defender, 900)

    assert tactic.target_region_name == "near_feeder"
    assert abs(defender.pose.distance_to(Pose2d(60, 100, 0)) - 24.0) < 10.0


def test_defend_supply_mode_camps_the_feeder_an_empty_handed_mark_must_return_to():
    match = make_match(_feeder_field(), auto_duration=1000, teleop_duration=1000)
    defender = match.add_robot(make_characteristics(), Pose2d(150, 100, 0), alliance="blue")
    match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="red")

    tactic = tactics.Defend(target="opponent_intent", deny="supply")
    _drive(match, tactic, defender, 60)

    assert tactic.target_region_name == "near_feeder"


def test_defend_scoring_mode_leaves_the_feeder_alone():
    """`deny` is a filter on the guess as well as on the declaration --
    a defender told to attack scoring must not be pulled to the feeder
    just because that is where its mark said it was going."""
    match = make_match(_feeder_field(), auto_duration=1000, teleop_duration=1000)
    defender = match.add_robot(make_characteristics(), Pose2d(150, 100, 0), alliance="blue")
    opponent = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="red")
    opponent.controller = _FakeController(target_region="near_feeder")

    tactic = tactics.Defend(target="opponent_intent", deny="scoring")
    _drive(match, tactic, defender, 60)

    assert tactic.target_region_name == "goal"


def test_defend_any_mode_follows_the_mark_between_supply_and_scoring():
    match = make_match(_feeder_field(), auto_duration=1000, teleop_duration=1000)
    defender = match.add_robot(make_characteristics(), Pose2d(150, 100, 0), alliance="blue")
    opponent = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="red")
    opponent.controller = _FakeController(target_region="near_feeder")

    tactic = tactics.Defend(target="opponent_intent", deny="any")
    _drive(match, tactic, defender, 60)
    assert tactic.target_region_name == "near_feeder"

    opponent.controller.intent.target_region = "goal"
    _drive(match, tactic, defender, 60)
    assert tactic.target_region_name == "goal"


def _stall(tactic, ctx, ticks=600):
    """Let the patience budget run out without the robot moving or
    scoring, which is what makes Score give up on its target."""
    for _ in range(ticks):
        tactic.tick(ctx)
        ctx.elapsed += ctx.dt


def test_score_does_not_immediately_re_pick_a_target_it_gave_up_on():
    """Otherwise a denied robot ping-pongs between its top two options:
    it gives up on A while standing at A (where B now ranks best), drives
    to B, gives up on B at B, and drives straight back."""
    field = make_field(scoring_regions=(NEAR_REGION, FAR_REGION))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))

    tactic = _score_tactic_holding_piece(match, robot)
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    assert tactic._current.region.name == "near"

    # Someone parks in "near", so the next re-pick has to go elsewhere.
    squatter = match.add_robot(make_characteristics(), Pose2d(110, 100, 0))
    _stall(tactic, ctx, ticks=300)  # 5s: past the patience budget, well inside the cooldown
    assert tactic._current.region.name == "far"
    assert ("near", "score_widget") in tactic._cooldowns

    # "near" is free again and ranks best on value, but it was just given
    # up on, so the next re-pick does not send the robot straight back.
    squatter.chassis.body.position = (20, 20)
    _stall(tactic, ctx, ticks=180)
    assert tactic._current.region.name == "far"


def test_score_contests_a_cooled_down_target_rather_than_holding_the_piece():
    """The cooldown is a preference, not a filter. With only one place to
    put the piece, contesting it eventually scores; refusing to go back
    never does."""
    field = make_field(scoring_regions=(NEAR_REGION,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))

    tactic = _score_tactic_holding_piece(match, robot)
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic._cooldowns[("near", "score_widget")] = 8.0

    _stall(tactic, ctx, ticks=120)
    assert tactic._current is not None
    assert tactic._current.region.name == "near"


def test_score_cooldowns_survive_a_reset():
    """`reset` fires whenever the strategy arbiter switches rules, which
    for a collect/score cycle is once per piece. Clearing cooldowns there
    would wipe every one of them before it was ever consulted."""
    field = make_field(scoring_regions=(NEAR_REGION, FAR_REGION))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))

    tactic = _score_tactic_holding_piece(match, robot)
    match.add_robot(make_characteristics(), Pose2d(110, 100, 0))  # takes "near"
    _stall(tactic, BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match))
    assert tactic._cooldowns

    tactic.reset()
    assert tactic._cooldowns
    assert tactic._current is None  # everything else about the run is cleared


def test_score_re_picks_when_its_target_never_completes():
    """The stall escape: a Score committed to something it cannot finish
    must not hold the robot for the rest of the match."""
    match = make_match(auto_duration=1000, teleop_duration=1000)
    # Outside the goal on purpose: a robot that is *already* in a scoring
    # pose is supposed to stay in it (see test_score_stops_once_the_pose
    # _already_scores), so the stall escape can only be exercised from
    # somewhere the deposit doesn't resolve.
    robot = match.add_robot(make_characteristics(), Pose2d(20, 180, 0))
    robot.held_pieces.append(match.spawn_piece(WIDGET, (20, 180)))

    tactic = tactics.Score()
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic.tick(ctx)
    assert tactic._current is not None
    committed = tactic._current

    # Freeze the robot where it is and let the patience budget run out;
    # the target should be re-priced rather than left untouched forever.
    for _ in range(600):
        tactic.tick(ctx)
        ctx.elapsed += ctx.dt
    assert tactic._current is not committed


def test_collect_lets_go_of_a_station_that_has_run_dry():
    """A committed station that runs out is not a trip going badly, it
    is a target that has stopped being a target -- so it is dropped at
    once, the way a piece somebody else picked up is.

    The robot is parked on the feed on purpose, because that is the case
    every other release path misses: `_station_stalled` stops its clock
    while the robot is being served, on the theory that an intake under
    way should finish, and `_better_station_exists` wants the committed
    station to be *full*, which an empty one is not. Measured before the
    fix: a robot reached a REEFSCAPE REEF ALGAE position, found it
    emptied, and sat on it for the remaining 126 seconds of the match."""
    field = make_field(intake_locations=(NEAR_FEEDER, FAR_FEEDER))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(60, 100, 0))

    tactic = _collect_tactic_at_stations(match, robot)
    assert tactic._target_station.name == "near_feeder"

    match.station_supply[NEAR_FEEDER] = 0
    tactic.tick(BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match))
    assert tactic._target_station.name == "far_feeder"


def test_a_station_that_ran_dry_is_not_put_on_cooldown():
    """Cooldown is for a station this robot failed to get served at, so
    it stops going back. An empty one needs no such memory: it is
    already excluded by `world_view.station_options` for exactly as long
    as it is empty, and a station an emitter refills should be available
    again the tick it is."""
    field = make_field(intake_locations=(NEAR_FEEDER, FAR_FEEDER))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(60, 100, 0))

    tactic = _collect_tactic_at_stations(match, robot)
    match.station_supply[NEAR_FEEDER] = 0
    tactic.tick(BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match))
    assert not tactic._station_cooldowns


def test_collect_gives_up_when_the_last_station_runs_dry():
    """Nothing left to collect at all -- which is a FAILURE the arbiter
    above can act on (strategy._FAILED_RULE_SUPPRESSION), not a wait."""
    field = make_field(intake_locations=(NEAR_FEEDER,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(60, 100, 0))

    tactic = _collect_tactic_at_stations(match, robot)
    assert tactic._target_station is not None

    match.station_supply[NEAR_FEEDER] = 0
    status = tactic.tick(BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match))
    assert status is Status.FAILURE


def _two_action_match(**characteristics_overrides):
    """One region, two ways to use it -- `rich` better on gross points
    per second, `cheap` better once the misses are counted -- with the
    region pinned so `_pick_option` goes through the re-pick path rather
    than the planner."""
    goal = ScoringRegion(
        name="goal", vertices=((380, 40), (420, 40), (420, 160), (380, 160)),
        actions=frozenset({"cheap", "rich"}), piece_types=frozenset({WIDGET}),
    )
    field = FieldConfig(width=500, height=200, scoring_regions=(goal,))
    rules = TableScoringRules({("cheap", "auto"): 4.0, ("rich", "auto"): 5.0})
    match = Match(field, rules, MatchConfig(auto_duration=1000, teleop_duration=1000))
    robot = match.add_robot(
        make_characteristics(deposit_time_by_action={"cheap": 1.0, "rich": 1.8},
                             **characteristics_overrides),
        Pose2d(20, 100, 0),
    )
    return match, robot


def test_score_repicks_on_expected_points_like_the_plan_did():
    """`_best_valued` is the re-pick path -- a pinned region, or the
    planner's choice turning out to be crowded. It ranks on expected
    points for the same reason the planner does, and it has to be the
    *same* reason: a re-pick ranking on a different quantity would
    quietly undo the plan it is re-picking within."""
    match, robot = _two_action_match(
        scoring_reliability_by_action={"cheap": 0.9, "rich": 0.82},
    )
    tactic = _score_tactic_holding_piece(match, robot, region="goal")
    assert tactic._current.action == "cheap"


def test_score_repick_still_takes_the_richer_target_when_it_lands():
    """Control for the test above -- same geometry, a robot that never
    misses. Its failure would mean the fixture's travel time has drifted
    out of the window where the two rankings disagree at all."""
    match, robot = _two_action_match()
    tactic = _score_tactic_holding_piece(match, robot, region="goal")
    assert tactic._current.action == "rich"


def _lone_feeder_match(**field_overrides):
    """One feeder of the collected type and nothing else of it -- the
    shape in which "give up only if there is somewhere better" becomes
    "never give up"."""
    feeder = IntakeLocation(
        name="only_feeder", vertices=((260, 90), (280, 90), (280, 110), (260, 110)),
        piece_type=WIDGET, starting_pieces=5,
    )
    field = make_field(intake_locations=(feeder,), **field_overrides)
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    return match, feeder


def _park_on(match, station, alliance):
    """A robot physically on the feed -- what `_held_by_opponent` and
    `_served_by_teammate` both test, as opposed to merely claiming it."""
    return match.add_robot(make_characteristics(), Pose2d(270, 100, 0), alliance=alliance)


def _wait_out_patience(tactic, ctx, seconds=30.0):
    for _ in range(int(seconds / ctx.dt)):
        tactic.tick(ctx)


def test_collect_stops_queueing_behind_an_opponent_at_the_only_feeder():
    """The escape's "somewhere to give up to" rule presumes the trip
    will eventually complete. Behind a parked defender it will not, and
    at the last feeder of a type there is nowhere better by definition,
    so the presumption was unfalsifiable and the robot waited out the
    match. Releasing hands the choice up to whoever chose the piece
    type -- the only layer that can change it."""
    match, feeder = _lone_feeder_match()
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="blue")
    _park_on(match, feeder, "red")

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic.tick(ctx)
    assert tactic._target_station is feeder

    status = Status.RUNNING
    for _ in range(int(30.0 / ctx.dt)):
        status = tactic.tick(ctx)
        if status is Status.FAILURE:
            break
    # Nothing of this type left to try, so the tactic fails upward
    # rather than standing there -- that is what lets Pursue re-arbitrate
    # to the other piece type, or a rule strategy fall to its next rule.
    assert status is Status.FAILURE
    assert "only_feeder" in tactic._station_cooldowns


def test_collect_keeps_queueing_behind_a_teammate():
    """The control. A teammate on the feed is a queue that is moving --
    it loads and leaves -- so the wait is worth having and none of the
    above should fire."""
    match, feeder = _lone_feeder_match()
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="blue")
    _park_on(match, feeder, "blue")

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    for _ in range(int(30.0 / ctx.dt)):
        assert tactic.tick(ctx) is Status.RUNNING
    assert tactic._target_station is feeder
    assert not tactic._station_cooldowns


def test_a_feeder_an_opponent_holds_is_not_resurrected_by_the_fallback():
    """Without this the release above is a no-op: `_best_station` falls
    back to the cooled-down list when that leaves nothing, so the robot
    re-picks the one feeder of its type on the very next tick and the
    cooldown does nothing but reset a clock."""
    match, feeder = _lone_feeder_match()
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="blue")
    _park_on(match, feeder, "red")

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    tactic = tactics.Collect(piece_type=WIDGET)
    tactic._station_cooldowns["only_feeder"] = 20.0
    assert tactic._best_station(ctx)[0] is None

    # A teammate holding it instead is still a queue, so the fallback
    # still offers it -- the exception is about *who* is in the way.
    friendly, friendly_feeder = _lone_feeder_match()
    ours = friendly.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="blue")
    _park_on(friendly, friendly_feeder, "blue")
    friendly_ctx = BehaviorContext(robot=ours, dt=1.0 / 60.0, match=friendly)
    friendly_tactic = tactics.Collect(piece_type=WIDGET)
    friendly_tactic._station_cooldowns["only_feeder"] = 20.0
    assert friendly_tactic._best_station(friendly_ctx)[0] is friendly_feeder


# A feeder on the far side of the STRUCTURE from the robot's start, so
# the only way there is around it -- and a robot that ends up with its
# bumper against the obstacle is not going to arrive.
BEHIND_FEEDER = IntakeLocation(
    name="behind_feeder", vertices=((205, 82), (241, 82), (241, 118), (205, 118)),
    piece_type=WIDGET, starting_pieces=5,
)


def test_collect_releases_a_station_it_is_wedged_short_of():
    """The third instance of "committed to a target the robot can never
    reach", after the emptied REEF ALGAE position and the denied feeder.

    Measured on blue={pursue_tuned} vs block/supply: a robot routing to
    an ALGAE staging position on the far REEF face wedged its bumper on
    the hex and commanded ~107 in/s into it for 120s of a 150s match.
    Every other release is blind to it -- the station is not full, not
    empty, and has no opponent on it -- so the elapsed clock overruns and
    then `_better_station_exists` asks for somewhere else of the same
    type to go, which the alliance's last ALGAE position does not have.
    Three of 24 seeds collapsed to 55-64 points against a 212 median.
    """
    field = make_field(intake_locations=(BEHIND_FEEDER,), obstacles=(STRUCTURE,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    # Parked against the structure, one feeder, no alternative to leave
    # for -- exactly the shape that made the commitment permanent.
    robot = match.add_robot(make_characteristics(), Pose2d(139, 100, 0))
    tactic = _collect_tactic_at_stations(match, robot, feeders=(BEHIND_FEEDER,))
    assert tactic._target_station is not None

    _run_collect(tactic, match, robot, tactics._STATION_PATIENCE_MIN * 2 + 1.0)
    assert tactic._target_station is None or tactic._station_cooldowns, (
        "a robot wedged on field geometry must let go of the station it "
        "cannot reach, even with no alternative of that type"
    )


def test_collect_still_waits_out_a_defender_in_open_space():
    """The other half of the same decision, and the reason the release
    above is gated on being against an obstacle: a robot held off the
    only feeder by a *defender* is stationary too, and from inside the
    tactic the two look identical. A defender moves eventually and
    touring the field instead measured as a loss, so this one must keep
    waiting -- clear of geometry, nothing is released."""
    field = make_field(intake_locations=(NEAR_FEEDER,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    # Clear of the wall by a comfortable margin, not by inches: this
    # asserts the *unwedged* branch, so a robot that is only barely clear
    # would let the test keep passing while measuring something else.
    robot = match.add_robot(make_characteristics(), Pose2d(28, 100, 0))
    tactic = _collect_tactic_at_stations(match, robot, feeders=(NEAR_FEEDER,))
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    assert not tactic._wedged(ctx)

    _run_collect(tactic, match, robot, tactics._STATION_PATIENCE_MIN * 2 + 1.0)
    assert tactic._target_station is not None
    assert not tactic._station_cooldowns


def test_collect_counts_the_field_boundary_as_geometry_it_can_wedge_on():
    """The walls are geometry too, and are not in `field.obstacles` --
    that list holds the REEFs, while the boundary is just `width` and
    `height`. Testing obstacles alone left the corners uncovered, which
    is the worst place to miss: a robot in a corner is against two
    surfaces at once and least able to shake itself loose.

    Found by `apps/run_stall_audit`, which reported a robot motionless at
    (21,21) for 129 seconds of a 150 second match with the nearest listed
    obstacle 149 inches away."""
    field = make_field(intake_locations=(NEAR_FEEDER,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    corner = match.add_robot(make_characteristics(), Pose2d(6, 6, 0))
    tactic = _collect_tactic_at_stations(match, corner, feeders=(NEAR_FEEDER,))
    assert tactic._wedged(BehaviorContext(robot=corner, dt=1.0 / 60.0, match=match))

    # ...and a robot the same distance from the middle of nowhere is not.
    open_field = match.add_robot(make_characteristics(), Pose2d(150, 100, 0))
    other = _collect_tactic_at_stations(match, open_field, feeders=(NEAR_FEEDER,))
    assert not other._wedged(BehaviorContext(robot=open_field, dt=1.0 / 60.0, match=match))


def test_collect_yields_to_an_opponent_whose_bumpers_cover_the_feed():
    """Engagement is the sim's *dispensing* test and it is alliance-
    scoped, so against an opponent it collapses to "is its centre in the
    polygon". A station 20in across and a chassis 28in wide means a
    defender parked squarely over one sits outside that test with its
    bumpers on the spot.

    Measured on block/scoring seed 5035: a defender at (177,206) against
    a polygon ending at x=176.4 read as absent, and the robot whose only
    approach it occupied orbited for 110 seconds -- ten times its
    patience -- because this predicate is that robot's escape."""
    station = IntakeLocation(
        name="feeder", vertices=((130, 90), (150, 90), (150, 110), (130, 110)),
        piece_type=WIDGET, starting_pieces=50,
    )
    field = make_field(intake_locations=(station,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    tactic = _collect_tactic_at_stations(match, robot)

    # Centre 6in clear of the polygon, so the old test says absent --
    # but a 28in chassis reaches 14in, so the feed is under its bumpers.
    blocker = match.add_robot(make_characteristics(), Pose2d(156, 100, 0), alliance="red")
    assert not tactics._robot_engaged_with_station(blocker, station)

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    assert tactic._held_by_opponent(ctx, station)


def test_collect_does_not_call_a_passing_opponent_a_blocked_feed():
    """The radius is the inscribed half-extent -- the distance inside
    which an overlap is certain -- because this decides whether to
    abandon a trip. A robot merely near the feeder is not holding it,
    and treating it as though it were hands away the same denial for
    free that the claim rules already refuse to pay for."""
    station = IntakeLocation(
        name="feeder", vertices=((130, 90), (150, 90), (150, 110), (130, 110)),
        piece_type=WIDGET, starting_pieces=50,
    )
    field = make_field(intake_locations=(station,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    tactic = _collect_tactic_at_stations(match, robot)

    passer = match.add_robot(make_characteristics(), Pose2d(180, 100, 0), alliance="red")
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    assert not tactic._held_by_opponent(ctx, station)


def test_collect_treats_a_teammate_on_the_feed_as_a_queue_not_a_blockage():
    """The whole distinction the escape turns on. A teammate covering
    the feed will load and leave, so the wait is a queue that is moving
    and the trip we chose is still the right one. Widening the body test
    must not swallow that."""
    station = IntakeLocation(
        name="feeder", vertices=((130, 90), (150, 90), (150, 110), (130, 110)),
        piece_type=WIDGET, starting_pieces=50,
    )
    field = make_field(intake_locations=(station,))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    tactic = _collect_tactic_at_stations(match, robot)

    match.add_robot(make_characteristics(), Pose2d(156, 100, 0), alliance=robot.alliance)
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    assert not tactic._held_by_opponent(ctx, station)


def _overrun_until_target_changes(tactic, ctx, seconds=60.0):
    """Drive `_reconsider_target` for `seconds` of simulated time with
    the robot neither moving nor depositing -- i.e. an attempt that
    simply never completes -- and report how long it took for the
    committed target to change, or None if it never did."""
    started = tactic._current.region.name
    elapsed = 0.0
    while elapsed < seconds:
        tactic._reconsider_target(ctx)
        elapsed += ctx.dt
        if tactic._current.region.name != started:
            return elapsed
    return None


def test_score_keeps_a_target_through_one_overrun():
    """Control for the test below. Overrunning the patience budget once
    and re-choosing the same target is the robot still trying, which is
    usually right: a region briefly crowded, a defender passing through,
    a piece about to be accepted. It must not be treated as failure."""
    field = make_field(scoring_regions=(NEAR_REGION, FAR_REGION))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    tactic = _score_tactic_holding_piece(match, robot)
    assert tactic._current.region.name == "near"

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    budget = tactic._patience
    _overrun_until_target_changes(tactic, ctx, seconds=budget * 1.5)

    assert tactic._current.region.name == "near"
    assert ("near", "score_widget") not in tactic._cooldowns


def test_score_gives_up_on_a_target_it_keeps_failing_at():
    """Twice in a row is not "still trying", it is a fixed point.

    `_pick_option` is deterministic in the field state, so a target that
    is still the best-valued option gets re-chosen, `_commit` restarts
    the patience clock, and nothing goes on cooldown to break the loop.
    Without a ratchet the robot re-attempts the identical impossible
    thing until the buzzer -- 110 seconds of a 150 second match, in the
    case that found this (DRY_RUN_LOG.md, F2). Note that the robot
    there was standing exactly on its own computed scoring pose and
    commanding no translation at all, so nothing about *motion*
    distinguished it from a robot correctly waiting."""
    field = make_field(scoring_regions=(NEAR_REGION, FAR_REGION))
    match = make_match(field, auto_duration=1000, teleop_duration=1000)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    tactic = _score_tactic_holding_piece(match, robot)
    assert tactic._current.region.name == "near"

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    took = _overrun_until_target_changes(tactic, ctx)

    assert took is not None, "Score re-chose an unreachable target forever"
    assert tactic._current.region.name == "far"
    assert ("near", "score_widget") in tactic._cooldowns
    # A second overrun, not the first: the ratchet must not turn a
    # momentary crowd into a lost target.
    assert took > tactics._STALL_PATIENCE_MIN
