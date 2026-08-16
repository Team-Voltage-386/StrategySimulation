"""
navigation.py tests: plan_path routes around obstacles instead of
through them, estimate_travel_time behaves sanely, and NavigateTo
actually gets a robot to a target on a field with an obstacle in the
direct line -- the build-order gate BEHAVIOR_PLAN.md calls out before
anything downstream (tactics) can depend on robots actually arriving.
"""
import math

from common_sim.control.behavior import BehaviorContext, Status
from common_sim.control.navigation import NavigateTo, estimate_travel_time, plan_path
from common_sim.field.field_config import FieldConfig, Obstacle, point_in_polygon
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.match.scoring import TableScoringRules
from common_sim.robot.characteristics import RobotCharacteristics

SQUARE_OBSTACLE = Obstacle(name="box", vertices=((90, -40), (150, -40), (150, 40), (90, 40)))


def make_field_with_obstacle() -> FieldConfig:
    return FieldConfig(width=300, height=200, obstacles=(SQUARE_OBSTACLE,))


def make_characteristics(**overrides) -> RobotCharacteristics:
    defaults = dict(
        name="test-bot", max_speed=150.0, max_accel=400.0, max_angular_speed=6.0,
        max_angular_accel=20.0, width=28.0, length=28.0, piece_capacity=1,
        intake_time=0.1, deposit_time=0.1, intake_range=6.0,
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)


def test_plan_path_direct_when_unobstructed():
    field = FieldConfig(width=300, height=200)
    path = plan_path(field, (0, 0), (100, 0), robot_radius=14.0)
    assert len(path) == 2
    assert (path[0].x, path[0].y) == (0, 0)
    assert (path[-1].x, path[-1].y) == (100, 0)


def test_plan_path_routes_around_obstacle():
    field = make_field_with_obstacle()
    start, goal = (0, 0), (240, 0)
    path = plan_path(field, start, goal, robot_radius=14.0)

    assert len(path) > 2, "should detour, not go straight through the obstacle"
    for i in range(len(path) - 1):
        a, b = (path[i].x, path[i].y), (path[i + 1].x, path[i + 1].y)
        mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
        assert not point_in_polygon(mid, SQUARE_OBSTACLE.vertices)
    # actually reaches the goal
    assert math.isclose(path[-1].x, goal[0], abs_tol=1e-6)
    assert math.isclose(path[-1].y, goal[1], abs_tol=1e-6)


def test_estimate_travel_time_zero_distance():
    field = FieldConfig(width=300, height=200)
    characteristics = make_characteristics()
    assert estimate_travel_time(field, (50, 50), (50, 50), characteristics) == 0.0


def test_estimate_travel_time_increases_with_detour():
    field_clear = FieldConfig(width=300, height=200)
    field_blocked = make_field_with_obstacle()
    characteristics = make_characteristics()

    direct = estimate_travel_time(field_clear, (0, 0), (240, 0), characteristics)
    detour = estimate_travel_time(field_blocked, (0, 0), (240, 0), characteristics)
    assert detour > direct


def test_navigate_to_reaches_target_around_obstacle():
    # y=100, well clear of the field boundary walls -- a robot's bumper
    # starting flush against a wall (e.g. pose (0, 0)) gets an initial
    # depenetration "pop" from pymunk that has nothing to do with
    # NavigateTo and would just add test noise. Obstacle is repositioned
    # to straddle this route the same way SQUARE_OBSTACLE straddles y=0.
    obstacle = Obstacle(name="box", vertices=((90, 60), (150, 60), (150, 140), (90, 140)))
    field = FieldConfig(width=300, height=200, obstacles=(obstacle,))
    match = Match(field, TableScoringRules({}), MatchConfig(auto_duration=1000, teleop_duration=1000))
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))

    target = Pose2d(240, 100, 0)
    nav = NavigateTo(lambda ctx: target, heading_mode="face_travel", replan_period=0.25)
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)

    status = Status.RUNNING
    ticks = 0
    while status == Status.RUNNING and ticks < 3000:
        ctx.dt = 1.0 / 60.0
        status = nav.tick(ctx)
        match.step(ctx.dt)
        ctx.elapsed += ctx.dt
        # never sits inside the obstacle's footprint mid-route
        assert not point_in_polygon((robot.pose.x, robot.pose.y), obstacle.vertices)
        ticks += 1

    assert status == Status.SUCCESS
    assert robot.pose.distance_to(target) <= 2.0 + 1e-6


def test_navigate_to_without_match_falls_back_to_direct_drive():
    field = FieldConfig(width=300, height=200)
    match = Match(field, TableScoringRules({}), MatchConfig(auto_duration=1000, teleop_duration=1000))
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    target = Pose2d(70, 100, 0)
    nav = NavigateTo(lambda ctx: target)
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=None)

    status = Status.RUNNING
    for _ in range(600):
        status = nav.tick(ctx)
        match.step(ctx.dt)
        if status != Status.RUNNING:
            break
    assert status == Status.SUCCESS


def test_navigate_to_avoids_head_on_robot_and_both_arrive():
    # Two robots driving straight at each other on the same line used to
    # deadlock forever (each robot's straight-line path plows through the
    # other, so neither yields and both sit jammed together). With
    # avoid_robots (the default), each treats the other as a dynamic
    # obstacle and routes around it.
    # Goals overshoot past each robot's own side (rather than landing
    # exactly on the other's starting pose) so the two robots are genuinely
    # passing through each other's path, not racing for a shared point --
    # a contested destination is a separate scenario, exercised below.
    field = FieldConfig(width=300, height=200)
    match = Match(field, TableScoringRules({}), MatchConfig(auto_duration=1000, teleop_duration=1000))
    robot_a = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    robot_b = match.add_robot(make_characteristics(), Pose2d(280, 100, math.pi))

    nav_a = NavigateTo(lambda ctx: Pose2d(275, 100, 0), replan_period=0.25)
    nav_b = NavigateTo(lambda ctx: Pose2d(25, 100, 0), replan_period=0.25)
    ctx_a = BehaviorContext(robot=robot_a, dt=1.0 / 60.0, match=match)
    ctx_b = BehaviorContext(robot=robot_b, dt=1.0 / 60.0, match=match)

    status_a = status_b = Status.RUNNING
    for _ in range(3000):
        if status_a == Status.RUNNING:
            status_a = nav_a.tick(ctx_a)
        if status_b == Status.RUNNING:
            status_b = nav_b.tick(ctx_b)
        match.step(1.0 / 60.0)
        if status_a != Status.RUNNING and status_b != Status.RUNNING:
            break

    assert status_a == Status.SUCCESS
    assert status_b == Status.SUCCESS


def test_navigate_to_reaches_target_near_another_robot_without_deadlock():
    # A robot standing right where a teammate wants to score (or right
    # next to a piece another robot is also collecting) used to deadlock:
    # once the target sits inside the other robot's avoidance radius, the
    # visibility graph had no edge reaching the goal at all, A* reported
    # it unreachable, and NavigateTo just... never got there. (Two robots
    # can't both occupy the *exact* same point -- that's a target-picking
    # contention the tactic layer avoids, not something navigation alone
    # can promise -- but getting close to one is a routine scoring
    # approach and must still succeed.)
    field = FieldConfig(width=300, height=200)
    match = Match(field, TableScoringRules({}), MatchConfig(auto_duration=1000, teleop_duration=1000))
    robot_a = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    robot_b = match.add_robot(make_characteristics(), Pose2d(160, 100, math.pi))

    nav_a = NavigateTo(lambda ctx: Pose2d(150, 100, 0), replan_period=0.25)
    ctx_a = BehaviorContext(robot=robot_a, dt=1.0 / 60.0, match=match)

    status_a = Status.RUNNING
    for _ in range(3000):
        status_a = nav_a.tick(ctx_a)
        match.step(1.0 / 60.0)
        if status_a != Status.RUNNING:
            break

    assert status_a == Status.SUCCESS
