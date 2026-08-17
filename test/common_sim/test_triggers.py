from common_sim.control import triggers as trg
from common_sim.control.behavior import BehaviorContext
from common_sim.field.field_config import FieldConfig, ScoringRegion
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.match.scoring import TableScoringRules
from common_sim.robot.characteristics import RobotCharacteristics

WIDGET = "widget"


def make_field() -> FieldConfig:
    region = ScoringRegion(
        name="goal", vertices=((80, -60), (250, -60), (250, 60), (80, 60)),
        actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}),
    )
    return FieldConfig(width=300, height=200, scoring_regions=(region,))


def make_characteristics(**overrides) -> RobotCharacteristics:
    defaults = dict(
        name="test-bot", max_speed=150.0, max_accel=400.0, max_angular_speed=6.0,
        max_angular_accel=20.0, width=28.0, length=28.0, piece_capacity=1,
        intake_time=0.1, deposit_time=0.1, intake_range=6.0,
        accepted_piece_types=frozenset({WIDGET}),
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)


def make_match_and_robot(**config_overrides):
    match = Match(make_field(), TableScoringRules({("score_widget", "auto"): 3.0}), MatchConfig(**config_overrides))
    robot = match.add_robot(make_characteristics(), Pose2d(150, 100, 0))
    return match, robot


def ctx_for(match, robot) -> BehaviorContext:
    return BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)


def test_always():
    match, robot = make_match_and_robot()
    assert trg.Always().evaluate(ctx_for(match, robot)) is True


def test_pieces_available_min_max_and_within():
    match, robot = make_match_and_robot()
    match.spawn_piece(WIDGET, (150, 100 + 10))
    match.spawn_piece(WIDGET, (150, 100 + 200))

    assert trg.PiecesAvailable(piece_type=WIDGET, min_count=2).evaluate(ctx_for(match, robot))
    assert not trg.PiecesAvailable(piece_type=WIDGET, min_count=3).evaluate(ctx_for(match, robot))
    assert trg.PiecesAvailable(piece_type=WIDGET, within=20).evaluate(ctx_for(match, robot))
    assert not trg.PiecesAvailable(piece_type=WIDGET, min_count=2, within=20).evaluate(ctx_for(match, robot))


def test_match_time_phase_and_bounds():
    match, robot = make_match_and_robot(auto_duration=1.0, teleop_duration=10.0)
    ctx = ctx_for(match, robot)
    assert trg.MatchTime(phase="auto").evaluate(ctx)
    assert not trg.MatchTime(phase="teleop").evaluate(ctx)
    assert not trg.MatchTime(after=5.0).evaluate(ctx)

    match.step(2.0)
    ctx = ctx_for(match, robot)
    assert trg.MatchTime(phase="teleop").evaluate(ctx)
    assert trg.MatchTime(after=1.5).evaluate(ctx)
    assert not trg.MatchTime(before=1.5).evaluate(ctx)


def test_match_time_remaining_under():
    match, robot = make_match_and_robot(auto_duration=1.0, teleop_duration=10.0)
    match.step(10.0)  # 1s left of 11s total
    ctx = ctx_for(match, robot)
    assert trg.MatchTime(remaining_under=2.0).evaluate(ctx)
    assert not trg.MatchTime(remaining_under=0.5).evaluate(ctx)


def test_pieces_held_and_at_capacity():
    match, robot = make_match_and_robot()
    ctx = ctx_for(match, robot)
    assert trg.PiecesHeld(piece_type=WIDGET, max_count=0).evaluate(ctx)
    assert not trg.AtCapacity(piece_type=WIDGET).evaluate(ctx)

    piece = match.spawn_piece(WIDGET, (0, 0))
    piece.held_by = robot
    robot.held_pieces.append(piece)
    ctx = ctx_for(match, robot)
    assert trg.PiecesHeld(piece_type=WIDGET, min_count=1).evaluate(ctx)
    assert trg.AtCapacity(piece_type=WIDGET).evaluate(ctx)


def test_scoring_available():
    match, robot = make_match_and_robot()
    ctx = ctx_for(match, robot)
    assert not trg.ScoringAvailable().evaluate(ctx)

    piece = match.spawn_piece(WIDGET, (0, 0))
    piece.held_by = robot
    robot.held_pieces.append(piece)
    ctx = ctx_for(match, robot)
    assert trg.ScoringAvailable().evaluate(ctx)
    assert trg.ScoringAvailable(min_value=3.0).evaluate(ctx)
    assert not trg.ScoringAvailable(min_value=100.0).evaluate(ctx)
    assert trg.ScoringAvailable(region="goal").evaluate(ctx)
    assert not trg.ScoringAvailable(region="nope").evaluate(ctx)


def test_opponent_near():
    match, robot = make_match_and_robot()
    opponent = match.add_robot(make_characteristics(), Pose2d(160, 100, 0), alliance="red")
    ctx = ctx_for(match, robot)
    assert trg.OpponentNear(within=20).evaluate(ctx)
    assert not trg.OpponentNear(within=5).evaluate(ctx)


def test_opponent_near_region():
    # region "goal" centroid is (165, 0) -- place the opponent 30in away.
    match, robot = make_match_and_robot()
    match.add_robot(make_characteristics(), Pose2d(165, 30, 0), alliance="red")
    ctx = ctx_for(match, robot)
    assert trg.OpponentNear(region="goal", within=50).evaluate(ctx)
    assert not trg.OpponentNear(region="goal", within=5).evaluate(ctx)


def test_combinators():
    match, robot = make_match_and_robot()
    ctx = ctx_for(match, robot)
    always_true = trg.Always()
    always_false = trg.Not(trigger=trg.Always())

    assert trg.AllOf(triggers=(always_true, always_true)).evaluate(ctx)
    assert not trg.AllOf(triggers=(always_true, always_false)).evaluate(ctx)
    assert trg.AnyOf(triggers=(always_false, always_true)).evaluate(ctx)
    assert not trg.AnyOf(triggers=(always_false, always_false)).evaluate(ctx)
    assert always_false.evaluate(ctx) is False


def test_describe_does_not_crash():
    for trigger in (
        trg.Always(), trg.PiecesAvailable(piece_type=WIDGET, min_count=1),
        trg.MatchTime(phase="auto"), trg.PiecesHeld(min_count=1), trg.AtCapacity(),
        trg.ScoringAvailable(min_value=1.0), trg.OpponentNear(region="goal"),
        trg.BeingDefended(region="goal", within=60.0), trg.BeingDefended(),
        trg.AllOf(triggers=(trg.Always(),)), trg.AnyOf(triggers=(trg.Always(),)),
        trg.Not(trigger=trg.Always()),
    ):
        assert isinstance(trigger.describe(), str)


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


def test_being_defended_needs_declared_intent_not_mere_proximity():
    match, robot = make_match_and_robot()
    opponent = match.add_robot(make_characteristics(), Pose2d(160, 100, 0), alliance="red")
    ctx = ctx_for(match, robot)

    # Right on top of us, but not there to deny us.
    assert not trg.BeingDefended().evaluate(ctx)
    assert trg.OpponentNear(within=20).evaluate(ctx)

    opponent.controller = _DefenseController(_DefenseIntent(marking=robot, target_region="goal"))
    assert trg.BeingDefended().evaluate(ctx)


def test_being_defended_filters_on_range_and_region():
    match, robot = make_match_and_robot()
    opponent = match.add_robot(make_characteristics(), Pose2d(160, 100, 0), alliance="red")
    opponent.controller = _DefenseController(_DefenseIntent(marking=robot, target_region="goal"))
    ctx = ctx_for(match, robot)

    assert trg.BeingDefended(within=20).evaluate(ctx)
    assert not trg.BeingDefended(within=5).evaluate(ctx)
    assert trg.BeingDefended(region="goal").evaluate(ctx)
    assert not trg.BeingDefended(region="elsewhere").evaluate(ctx)


def test_being_defended_ignores_a_defender_marking_a_teammate():
    match, robot = make_match_and_robot()
    teammate = match.add_robot(make_characteristics(), Pose2d(150, 140, 0), alliance="blue")
    opponent = match.add_robot(make_characteristics(), Pose2d(160, 100, 0), alliance="red")
    opponent.controller = _DefenseController(_DefenseIntent(marking=teammate))

    assert not trg.BeingDefended().evaluate(ctx_for(match, robot))
    assert trg.BeingDefended().evaluate(ctx_for(match, teammate))


def test_children_exposes_the_tree_for_composites_only():
    leaf = trg.Always()
    assert leaf.children() == ()
    assert trg.AllOf(triggers=(leaf,)).children() == (leaf,)
    assert trg.AnyOf(triggers=(leaf,)).children() == (leaf,)
    assert trg.Not(trigger=leaf).children() == (leaf,)
    assert trg.Not().children() == ()
