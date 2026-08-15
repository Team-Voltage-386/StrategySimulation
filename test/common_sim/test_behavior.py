from common_sim.control.behavior import (
    Behavior,
    BehaviorContext,
    Branch,
    DriveToPose,
    Parallel,
    Repeat,
    RunIntake,
    RunManipulator,
    Sequence,
    Status,
    Wait,
)
from common_sim.field.field_config import FieldConfig, ScoringRegion
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.match.scoring import TableScoringRules
from common_sim.robot.characteristics import RobotCharacteristics

WIDGET = "widget"

# The default test chassis is 28in (half=14) with a 6in intake reach,
# and GamePiece's default radius is 8in. The intake wedge starts flush
# against the bumper (no gap), so a piece's near edge -- piece_radius
# closer to the robot than its center -- can reach the solid bumper
# before the wedge safely detects it if the approach targets the wedge's
# *center* relative to the piece's *center point*. Targeting the wedge's
# far edge against the piece's near edge (with a couple inches of
# margin) keeps the piece inside the wedge without ever touching the
# non-sensor bumper.
PIECE_RADIUS = 8.0
APPROACH_OFFSET = PIECE_RADIUS + 14.0 + 6.0 - 2.0


ROBOT_START = Pose2d(30, 100, 0)  # well clear of the perimeter walls (chassis half-size is 14)


def make_match_with_robot(**char_overrides):
    # Field y spans [0, 200]; keep the region entirely inside that (a
    # robot start pose sitting IN a wall, e.g. the (0,0) corner, causes
    # pymunk to fire a large one-tick overlap-resolution velocity that
    # derails a precision DriveToPose test).
    region = ScoringRegion(
        name="goal",
        vertices=((80, 40), (250, 40), (250, 160), (80, 160)),
        actions=frozenset({"score"}),
        piece_types=frozenset({WIDGET}),
    )
    field = FieldConfig(width=300, height=200, scoring_regions=(region,))
    rules = TableScoringRules({("score", "auto"): 3.0, ("score", "teleop"): 1.0})
    match = Match(field, rules, MatchConfig(auto_duration=1000, teleop_duration=1000))
    defaults = dict(
        max_speed=150.0, max_accel=400.0, max_angular_speed=6.0, max_angular_accel=20.0,
        width=28.0, length=28.0, piece_capacity=1, intake_time=0.05, deposit_time=0.05,
        intake_range=6.0, accepted_piece_types=frozenset({WIDGET}),
    )
    defaults.update(char_overrides)
    robot = match.add_robot(RobotCharacteristics(**defaults), ROBOT_START)
    return match, robot


def run_behavior(match: Match, robot, behavior: Behavior, n_ticks: int, dt: float = 1.0 / 60.0):
    ctx = BehaviorContext(robot=robot, dt=dt, match=match)
    for _ in range(n_ticks):
        ctx.dt = dt
        behavior.tick(ctx)
        match.step(dt)
        ctx.elapsed += dt
    return ctx


# -- leaves ----------------------------------------------------------------


def test_wait_runs_then_succeeds():
    node = Wait(0.1)
    ctx = BehaviorContext(robot=None, dt=1.0 / 60.0)
    statuses = [node.tick(ctx) for _ in range(12)]
    # 0.1s / (1/60)s = 6 ticks; allow +-1 for float accumulation drift
    # rather than pin an exact tick boundary.
    first_success = statuses.index(Status.SUCCESS)
    assert 5 <= first_success <= 7
    assert all(s == Status.RUNNING for s in statuses[:first_success])
    assert all(s == Status.SUCCESS for s in statuses[first_success:])


def test_drive_to_pose_reaches_target():
    match, robot = make_match_with_robot()
    target = Pose2d(150, 100, 0)
    node = DriveToPose(target, position_tolerance=2.0)
    ctx = run_behavior(match, robot, node, n_ticks=600)
    assert robot.pose.distance_to(target) <= 2.0 + 1e-6


def test_run_intake_captures_a_piece_end_to_end():
    match, robot = make_match_with_robot()
    piece = match.spawn_piece(WIDGET, (60, 100))
    # Target pose is offset short of the piece by (half chassis length +
    # half intake reach) so the piece lands inside the forward intake
    # wedge once the robot arrives, not just near the chassis center.
    approach = Pose2d(60 - APPROACH_OFFSET, 100, 0)
    node = Sequence([DriveToPose(approach, position_tolerance=0.5), RunIntake(timeout=5.0)])
    run_behavior(match, robot, node, n_ticks=1200)
    assert robot.held_pieces == [piece]


def test_run_intake_times_out_if_never_captures():
    match, robot = make_match_with_robot()
    node = RunIntake(timeout=0.1)
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0)
    status = Status.RUNNING
    for _ in range(20):
        status = node.tick(ctx)
        match.step(1.0 / 60.0)
        if status != Status.RUNNING:
            break
    assert status == Status.FAILURE


def test_run_manipulator_fails_immediately_with_nothing_held():
    match, robot = make_match_with_robot()
    node = RunManipulator("score")
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0)
    assert node.tick(ctx) == Status.FAILURE


# -- full autonomous routine ------------------------------------------------


def test_full_autonomous_routine_scores():
    """Drive to a piece, intake it, drive to the goal, deposit -- the
    exact composition an autonomous routine or a scripted alliance/
    opponent robot would use."""
    match, robot = make_match_with_robot()
    piece = match.spawn_piece(WIDGET, (60, 100))

    routine = Sequence([
        DriveToPose(Pose2d(60 - APPROACH_OFFSET, 100, 0), position_tolerance=0.5),
        RunIntake(timeout=5.0),
        DriveToPose(Pose2d(150, 100, 0), position_tolerance=4.0),
        RunManipulator("score", timeout=5.0),
    ])

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0)
    for _ in range(3000):
        routine.tick(ctx)
        match.step(1.0 / 60.0)
        ctx.elapsed += 1.0 / 60.0
        if piece.scored:
            break

    assert piece.scored
    assert match.scores.get("blue", 0.0) == 3.0
    assert match.events.of_kind("score")[0].data["action"] == "score"


# -- composites --------------------------------------------------------------


class _CountingLeaf(Behavior):
    """Succeeds on the Nth tick since its last reset; counts completions."""

    def __init__(self, ticks_to_succeed: int):
        self.ticks_to_succeed = ticks_to_succeed
        self.completions = 0
        self._count = 0

    def tick(self, ctx):
        self._count += 1
        if self._count >= self.ticks_to_succeed:
            self.completions += 1
            return Status.SUCCESS
        return Status.RUNNING

    def reset(self):
        self._count = 0


def test_sequence_advances_only_after_success():
    a, b = _CountingLeaf(2), _CountingLeaf(1)
    seq = Sequence([a, b])
    ctx = BehaviorContext(robot=None, dt=1.0 / 60.0)
    assert seq.tick(ctx) == Status.RUNNING  # a still running
    assert b._count == 0  # b not ticked yet
    assert seq.tick(ctx) == Status.SUCCESS  # a succeeds, b ticks+succeeds same pass
    assert b.completions == 1


def test_sequence_fails_when_a_child_fails():
    class AlwaysFail(Behavior):
        def tick(self, ctx):
            return Status.FAILURE

    seq = Sequence([AlwaysFail(), _CountingLeaf(1)])
    ctx = BehaviorContext(robot=None, dt=1.0 / 60.0)
    assert seq.tick(ctx) == Status.FAILURE


def test_parallel_succeeds_when_all_children_succeed():
    a, b = _CountingLeaf(1), _CountingLeaf(3)
    par = Parallel([a, b])
    ctx = BehaviorContext(robot=None, dt=1.0 / 60.0)
    assert par.tick(ctx) == Status.RUNNING
    assert par.tick(ctx) == Status.RUNNING
    assert par.tick(ctx) == Status.SUCCESS
    assert a.completions == 1
    assert b.completions == 1


def test_parallel_succeed_on_partial_count():
    a, b = _CountingLeaf(1), _CountingLeaf(100)
    par = Parallel([a, b], succeed_on=1)
    ctx = BehaviorContext(robot=None, dt=1.0 / 60.0)
    assert par.tick(ctx) == Status.SUCCESS


def test_branch_evaluates_condition_once():
    calls = []

    def condition(ctx):
        calls.append(1)
        return True

    branch = Branch(condition, if_true=_CountingLeaf(2), if_false=_CountingLeaf(1))
    ctx = BehaviorContext(robot=None, dt=1.0 / 60.0)
    branch.tick(ctx)
    branch.tick(ctx)
    assert len(calls) == 1  # only evaluated on the first tick


def test_branch_chooses_false_path():
    branch = Branch(lambda ctx: False, if_true=_CountingLeaf(1), if_false=_CountingLeaf(1))
    ctx = BehaviorContext(robot=None, dt=1.0 / 60.0)
    assert branch.tick(ctx) == Status.SUCCESS
    assert branch.if_true.completions == 0
    assert branch.if_false.completions == 1


def test_repeat_never_terminates_and_resets_child_each_cycle():
    leaf = _CountingLeaf(2)
    loop = Repeat(leaf)
    ctx = BehaviorContext(robot=None, dt=1.0 / 60.0)
    for _ in range(10):
        assert loop.tick(ctx) == Status.RUNNING
    assert leaf.completions == 5  # 10 ticks / 2-ticks-per-cycle
