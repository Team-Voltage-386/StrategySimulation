"""
Pursue tests against a synthetic two-piece-type game.

The field is built so the arbitration has a right answer that is obvious
by hand: a CHEAP piece with its deposit right next to the robot, and a
RICH piece that has to be fetched from a feeder. Moving the feeder or the
deposits around is what flips which job wins, so most of these assert
"which child is running", not a score.
"""
import math

from common_sim.control import tactics, utility
from common_sim.control.behavior import BehaviorContext, Status
from common_sim.control.strategy import Rule, Strategy, StrategyController
from common_sim.control.triggers import Always
from common_sim.field.field_config import FieldConfig, IntakeLocation, ScoringRegion
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.match.scoring import TableScoringRules

CHEAP = "cheap"
RICH = "rich"

# Points and durations are chosen so the two rates are far apart on
# purpose. An arbitration test that only just resolves is a test of
# pathfinding noise.
POINTS = {("drop_cheap", "auto"): 2.0, ("drop_rich", "auto"): 30.0}


def make_field(*, feeder_at=(20, 100), rich_goal_at=(60, 100)) -> FieldConfig:
    def box(cx, cy, half=20):
        return ((cx - half, cy - half), (cx + half, cy - half), (cx + half, cy + half), (cx - half, cy + half))

    cheap_goal = ScoringRegion(
        name="cheap_goal", vertices=box(100, 100),
        actions=frozenset({"drop_cheap"}), piece_types=frozenset({CHEAP}),
    )
    rich_goal = ScoringRegion(
        name="rich_goal", vertices=box(*rich_goal_at),
        actions=frozenset({"drop_rich"}), piece_types=frozenset({RICH}),
    )
    feeder = IntakeLocation(
        name="feeder", vertices=box(*feeder_at), piece_type=RICH, starting_pieces=99,
    )
    return FieldConfig(
        width=1200, height=200, scoring_regions=(cheap_goal, rich_goal), intake_locations=(feeder,),
    )


def make_characteristics(**overrides):
    from common_sim.robot.characteristics import RobotCharacteristics
    defaults = dict(
        name="test-bot", max_speed=150.0, max_accel=400.0, max_angular_speed=6.0,
        max_angular_accel=20.0, width=28.0, length=28.0,
        piece_capacity_by_type={CHEAP: 1, RICH: 1},
        intake_time_by_type={CHEAP: 0.2, RICH: 0.2}, station_intake_time=0.4,
        deposit_time_by_action={"drop_cheap": 0.2, "drop_rich": 0.2},
        deposit_time=0.2, intake_range=6.0,
        accepted_piece_types=frozenset({CHEAP, RICH}),
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)


def make_match(field=None) -> Match:
    return Match(
        field or make_field(),
        TableScoringRules(dict(POINTS)),
        MatchConfig(auto_duration=1000, teleop_duration=1000),
    )


def give(match, robot, piece_type, position=(0, 0)):
    piece = match.spawn_piece(piece_type, position)
    piece.held_by = robot
    robot.held_pieces.append(piece)
    return piece


def context(match, robot, dt=1.0 / 60.0) -> BehaviorContext:
    return BehaviorContext(robot=robot, dt=dt, match=match)


def settle(tactic, ctx, ticks=1):
    """Tick without stepping physics -- for asserting on the *decision*
    rather than on where the robot ends up."""
    status = Status.RUNNING
    for _ in range(ticks):
        status = tactic.tick(ctx)
    return status


# --- the arbitration -------------------------------------------------


def test_scores_what_it_holds_when_the_deposit_is_close():
    """The motivating case, and the reason a hand-written `priority`
    could not express it: holding a cheap piece a step away from its
    deposit beats setting off for a richer one across the field, even
    though the richer piece is worth fifteen times as much."""
    match = make_match(make_field(feeder_at=(1150, 100), rich_goal_at=(1100, 100)))
    robot = match.add_robot(make_characteristics(), Pose2d(100, 100, 0))
    give(match, robot, CHEAP)

    tactic = tactics.Pursue()
    settle(tactic, context(match, robot))
    assert isinstance(tactic.active_tactic, tactics.Score)


def test_fetches_when_the_cycle_it_buys_beats_the_piece_in_hand():
    """The same robot, holding the same cheap piece, with the rich
    supply now underfoot and the cheap deposit far off -- the arbitration
    has to flip, or it isn't arbitrating."""
    field = make_field(feeder_at=(20, 100), rich_goal_at=(60, 100))
    match = make_match(field)
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    give(match, robot, CHEAP)

    tactic = tactics.Pursue()
    settle(tactic, context(match, robot))
    assert isinstance(tactic.active_tactic, tactics.Collect)


def test_a_pickup_with_nowhere_to_put_it_is_worth_nothing():
    """`enables is None` means there is no legal deposit for that type,
    so the whole trip buys zero points -- not "some points, eventually"."""
    tactic = tactics.Pursue()
    worthless = utility.Outcome(
        kind="collect", label="x", points=0.0, duration=1.0,
        success_probability=1.0, payload=None, enables=None,
    )
    assert tactic._cycle_rate(worthless) == 0.0


def test_cycle_rate_prices_the_drive_out_and_the_drive_back():
    """Points over (fetch + deposit), not points over deposit -- pricing
    a pickup by only the half it can see is what would make every distant
    feeder look free."""
    tactic = tactics.Pursue()
    payoff = utility.Outcome(kind="score", label="s", points=30.0, duration=4.0,
                             success_probability=1.0, payload=None)
    pickup = utility.Outcome(kind="collect", label="c", points=0.0, duration=2.0,
                             success_probability=1.0, payload=None, enables=payoff)
    assert tactic._cycle_rate(pickup) == 30.0 / 6.0

    tactic.lookahead_weight = 0.5
    assert tactic._cycle_rate(pickup) == 0.5 * 30.0 / 6.0


# --- commitment ------------------------------------------------------


def test_min_commit_holds_a_job_through_a_better_offer():
    """Without this the robot re-decides on its replan cadence and drives
    the midpoint of the two jobs."""
    match = make_match(make_field(feeder_at=(1150, 100), rich_goal_at=(1100, 100)))
    robot = match.add_robot(make_characteristics(), Pose2d(100, 100, 0))
    give(match, robot, CHEAP)

    tactic = tactics.Pursue(min_commit=10.0, replan_period=0.01)
    ctx = context(match, robot, dt=0.1)
    settle(tactic, ctx)
    assert isinstance(tactic.active_tactic, tactics.Score)

    # Move the rich supply and its goal on top of the robot, which makes
    # fetching far and away the better job -- and change nothing else.
    match.field = make_field(feeder_at=(100, 100), rich_goal_at=(110, 100))
    settle(tactic, ctx, ticks=20)  # 2.0s, well short of min_commit
    assert isinstance(tactic.active_tactic, tactics.Score)

    settle(tactic, ctx, ticks=90)  # now past it
    assert isinstance(tactic.active_tactic, tactics.Collect)


def test_switch_margin_ignores_a_job_that_is_only_marginally_better():
    """Driven at the rates rather than through the geometry: the gate
    under test is "how much better does the other job have to look", and
    a fixture that has to be positioned until two pathfinding results
    land a few percent apart would be testing the pathfinder."""
    def run_with(margin, collect_rate):
        match = make_match()
        robot = match.add_robot(make_characteristics(), Pose2d(100, 100, 0))
        give(match, robot, CHEAP)
        tactic = tactics.Pursue(switch_margin=margin, min_commit=0.0)
        ctx = context(match, robot)
        tactic._rank_with_type = lambda _ctx: (10.0, collect_rate, RICH)
        tactic._select(ctx, tactic._score)
        tactic._arbitrate(ctx)
        return tactic.active_tactic

    # 12.0 beats 10.0 outright, but not by the quarter the default asks.
    assert isinstance(run_with(0.0, 12.0), tactics.Collect)
    assert isinstance(run_with(0.25, 12.0), tactics.Score)
    assert isinstance(run_with(0.25, 13.0), tactics.Collect)


def test_the_priced_piece_type_is_handed_to_the_collect_child():
    """Pursue prices *what kind of thing* is worth fetching -- the whole
    difference between a 2-point piece and a 30-point one. An untyped
    Collect would be free to grab the cheap one on a shorter drive and
    throw that comparison away."""
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(100, 100, 0))
    tactic = tactics.Pursue()
    ctx = context(match, robot)

    tactic._rank_with_type = lambda _ctx: (0.0, 5.0, RICH)
    tactic._arbitrate(ctx)
    assert tactic.active_tactic is tactic._collect
    assert tactic._collect.piece_type == RICH


def test_a_fetch_already_under_way_is_not_retyped_mid_trip():
    """The type is chosen when a fetch begins and not re-litigated on the
    way. Two piece types whose rates sit close together would otherwise
    trade the argmax every arbitration and reset the trip each time --
    the robot drives half a second toward one, half a second toward the
    other, and arrives at neither."""
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    tactic = tactics.Pursue()
    ctx = context(match, robot)

    match.spawn_piece(CHEAP, (40, 100))  # the only CHEAP supply; the feeder dispenses RICH
    tactic._select(ctx, tactic._collect)
    tactic._collect.piece_type = CHEAP
    tactic._collect.tick(ctx)
    committed = tactic._collect._target_piece
    assert committed is not None

    tactic._aim_collect_at(ctx, RICH)
    assert tactic._collect.piece_type == CHEAP
    assert tactic._collect._target_piece is committed


def test_an_idle_collect_is_retyped_freely():
    """The guard above is about not disturbing a trip in progress, not
    about the type being immutable -- with nothing committed there is no
    trip to disturb."""
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    tactic = tactics.Pursue()
    ctx = context(match, robot)

    tactic._collect.piece_type = CHEAP
    assert not tactic._collect.has_target
    tactic._aim_collect_at(ctx, RICH)
    assert tactic._collect.piece_type == RICH


def test_collect_skips_a_piece_there_is_no_room_for():
    """`world_view.collectable_pieces` reports physical capability, not
    how full the robot is, and `station_options` applies the capacity
    check itself -- so a loose piece was the one target a robot could
    commit to with nowhere to put it. It then parks in intake range and
    waits forever, and even the stall escape cannot see it: that clock
    stops while `robot.accepts(piece)` is true, so an intake that can
    never complete is indistinguishable from one about to."""
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(100, 100, 0))
    give(match, robot, CHEAP)                       # fills the CHEAP slot
    match.spawn_piece(CHEAP, (110, 100))            # right there, and unusable
    match.spawn_piece(RICH, (300, 100))             # farther, but takeable

    collect = tactics.Collect()
    collect.tick(context(match, robot))
    assert collect._target_piece is None or collect._target_piece.piece_type == RICH


# --- handing off to the children -------------------------------------


def test_publishes_the_child_s_intent_not_its_own():
    """Everyone else's coordination reads `robot.intent` -- station claim
    races, piece contention, a defender identifying what its mark is up
    to. A Pursue robot that published only "Pursue" would be invisible to
    all of it (see strategy._delegate)."""
    match = make_match(make_field(feeder_at=(1150, 100), rich_goal_at=(1100, 100)))
    robot = match.add_robot(make_characteristics(), Pose2d(100, 100, 0))
    give(match, robot, CHEAP)
    strategy = Strategy(name="p", rules=[Rule(name="pursue", trigger=Always(), tactic=tactics.Pursue())])
    robot.controller = StrategyController(strategy, robot)

    robot.controller.tick(context(match, robot))
    assert robot.intent.tactic_name == "Score"
    assert robot.intent.target_region == "cheap_goal"


def test_switching_off_a_collect_releases_the_intake():
    """Collect commands the intake on every tick it runs and only turns
    it off in its own SUCCESS/FAILURE branch, so an arbiter that swapped
    away mid-collect without this would leave it latched for the rest of
    the match -- and the robot would scoop up whatever it drove past.
    StrategyController does exactly this when it preempts a tactic; an
    arbiter that runs children has to as well."""
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    tactic = tactics.Pursue()
    ctx = context(match, robot)

    tactic._select(ctx, tactic._collect)
    tactic._collect.tick(ctx)
    assert robot._commanded_intake

    tactic._select(ctx, tactic._score)
    assert not robot._commanded_intake


def test_a_job_that_reports_it_cannot_be_done_is_dropped_at_once():
    """Collect FAILURE means "nothing gettable", which `min_commit` must
    not sit on: the commitment window exists to stop dithering between
    two workable jobs, not to hold a robot on one that just declared
    itself impossible."""
    field = make_field()
    match = Match(field, TableScoringRules(dict(POINTS)), MatchConfig(auto_duration=1000, teleop_duration=1000))
    # A feeder with nothing in it and no loose pieces: fetching is
    # impossible, but the robot is holding something it can score.
    match.field = FieldConfig(
        width=1200, height=200, scoring_regions=field.scoring_regions, intake_locations=(),
    )
    robot = match.add_robot(make_characteristics(), Pose2d(100, 100, 0))
    give(match, robot, CHEAP)

    tactic = tactics.Pursue(min_commit=60.0)
    ctx = context(match, robot)
    tactic._select(ctx, tactic._collect)
    status = tactic.tick(ctx)

    assert status is Status.RUNNING
    assert isinstance(tactic.active_tactic, tactics.Score)


def test_reports_failure_when_neither_job_has_anything_to_do():
    """FAILURE is the arbiter's channel for "let another rule have the
    robot" (strategy._FAILED_RULE_SUPPRESSION). Nothing held, nothing to
    fetch: there is no job here."""
    match = Match(
        FieldConfig(width=1200, height=200, scoring_regions=make_field().scoring_regions, intake_locations=()),
        TableScoringRules(dict(POINTS)), MatchConfig(auto_duration=1000, teleop_duration=1000),
    )
    robot = match.add_robot(make_characteristics(), Pose2d(100, 100, 0))

    tactic = tactics.Pursue()
    assert tactic.tick(context(match, robot)) is Status.FAILURE
    assert tactic.active_tactic is None


def test_never_reports_success():
    """A standing job, not a task that completes. An `Always`-triggered
    rule whose tactic reports SUCCESS is re-selected by the arbiter on
    the next tick and logs a behavior change every tick forever."""
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(100, 100, 0))
    give(match, robot, CHEAP)

    tactic = tactics.Pursue()
    ctx = context(match, robot)
    for _ in range(400):
        status = tactic.tick(ctx)
        match.step(ctx.dt)
        assert status is not Status.SUCCESS


def test_reset_leaves_the_children_s_cooldowns_alone():
    """Score's failed-target cooldown and Collect's station cooldown
    deliberately outlive their own `reset()` -- "this spot would not take
    my piece a moment ago" is knowledge about the field, not state
    belonging to one activation. An arbiter that cleared them on every
    cycle would make both mechanisms the silent no-op their comments
    warn about."""
    tactic = tactics.Pursue()
    tactic._score._cooldowns[("cheap_goal", "drop_cheap")] = 8.0
    tactic._collect._station_cooldowns["feeder"] = 20.0

    tactic.reset()

    assert tactic._score._cooldowns == {("cheap_goal", "drop_cheap"): 8.0}
    assert tactic._collect._station_cooldowns == {"feeder": 20.0}
    assert tactic.active_tactic is None


# --- the cost control the arbitration depends on ---------------------


def test_travel_cache_agrees_with_estimating_directly():
    from common_sim.control import navigation
    match = make_match()
    characteristics = make_characteristics()
    cache = utility.TravelCache(match.field, characteristics)

    for goal in ((100.0, 100.0), (60.0, 100.0), (100.0, 100.0)):
        assert cache.time((0.0, 0.0), goal) == navigation.estimate_travel_time(
            match.field, (0.0, 0.0), goal, characteristics,
        )


def test_travel_cache_asks_once_per_distinct_route():
    """The reason it exists: a scoring region carries several actions, so
    pricing every legal slot re-derives one route many times over. In
    REEFSCAPE that is 24 CORAL slots across 6 REEF faces."""
    from common_sim.control import navigation
    match = make_match()
    cache = utility.TravelCache(match.field, make_characteristics())

    calls = []
    real = navigation.estimate_travel_time
    navigation.estimate_travel_time = lambda *a: (calls.append(a), real(*a))[1]
    try:
        for _ in range(5):
            cache.time((0.0, 0.0), (100.0, 100.0))
        cache.time((0.0, 0.0), (60.0, 100.0))
    finally:
        navigation.estimate_travel_time = real

    assert len(calls) == 2


def test_pricing_a_batch_matches_pricing_each_option_alone():
    """The cache may only ever be a speedup. Every number it feeds into
    an Outcome has to be the one an uncached call would have produced,
    or Phase A's equivalence gate was measuring the wrong thing."""
    match = make_match()
    robot = match.add_robot(make_characteristics(), Pose2d(100, 100, 0))
    give(match, robot, CHEAP)

    from common_sim.control import world_view
    shared = utility.TravelCache(match.field, robot.characteristics)
    for legal in world_view.scoring_options(match, robot):
        cached = utility.build_option(match, robot, legal, (0.0, 0.0), shared)
        alone = utility.build_option(match, robot, legal, (0.0, 0.0))
        assert cached == alone
        assert not math.isnan(cached.travel_time)
