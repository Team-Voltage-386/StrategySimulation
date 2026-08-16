"""
navigation.py tests: plan_path routes around obstacles instead of
through them, estimate_travel_time behaves sanely, and NavigateTo
actually gets a robot to a target on a field with an obstacle in the
direct line -- the build-order gate BEHAVIOR_PLAN.md calls out before
anything downstream (tactics) can depend on robots actually arriving.
"""
import math

from common_sim.control.behavior import BehaviorContext, Status
from common_sim.control.navigation import (
    NavigateTo,
    _PREDICTION_MIN_SHIFT,
    _inflate,
    _octagon,
    clear_standoff,
    convex_overlap,
    estimate_travel_time,
    footprint_polygon,
    plan_path,
    polygon_distance,
)
from common_sim.field.field_config import FieldConfig, Obstacle, point_in_polygon
from common_sim.geometry import Pose2d, Vec2d
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


def _hexagon(center, apothem):
    radius = apothem / math.cos(math.radians(30))
    return tuple(
        (center[0] + radius * math.cos(math.radians(30 + 60 * i)),
         center[1] + radius * math.sin(math.radians(30 + 60 * i)))
        for i in range(6)
    )


def test_inflate_clears_every_edge_by_the_full_radius():
    # The REEF is a hexagon, and a hexagon is exactly where the old
    # centroid-radial inflation lost the most: it only reached `radius`
    # at the six vertices, leaving each *face* 13% short -- inches of
    # robot inside the real structure for a path that hugs the polygon.
    apothem, radius = 32.75, 19.8
    hexagon = _hexagon((150, 100), apothem)
    inflated = _inflate(hexagon, radius)

    for i in range(len(hexagon)):
        a, b = hexagon[i], hexagon[(i + 1) % len(hexagon)]
        midpoint = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
        outward = (midpoint[0] - 150, midpoint[1] - 100)
        # A hair inside the full radius -- exactly on the offset boundary
        # is a ray-casting coin flip, which says nothing either way.
        scale = radius * 0.999 / math.hypot(*outward)
        just_outside = (midpoint[0] + outward[0] * scale, midpoint[1] + outward[1] * scale)
        assert point_in_polygon(just_outside, inflated), "face inflated by less than the full radius"


def test_inflate_square_is_exact():
    square = ((0, 0), (10, 0), (10, 10), (0, 10))
    inflated = _inflate(square, 2.0)
    assert all(
        any(math.isclose(v[0], x, abs_tol=1e-9) and math.isclose(v[1], y, abs_tol=1e-9) for v in inflated)
        for x, y in ((-2, -2), (12, -2), (12, 12), (-2, 12))
    )


def test_plan_path_still_detours_when_the_goal_hugs_the_obstacle():
    # A scoring target sits right against the structure it scores on, so
    # it lands inside the obstacle-inflated-by-robot-radius polygon. That
    # used to make the goal unreachable in the visibility graph, and
    # plan_path fell back to a straight line -- losing avoidance for the
    # whole route, which is how robots ended up driving through the REEF.
    field = make_field_with_obstacle()  # box spans x 90..150, y -40..40
    goal = (155, 0)  # 5in off the box's far face; well inside a 14in inflation
    path = plan_path(field, (0, 0), goal, robot_radius=14.0)

    assert len(path) > 2, "should still route around the box, not cut straight to the goal"
    for i in range(len(path) - 1):
        a, b = (path[i].x, path[i].y), (path[i + 1].x, path[i + 1].y)
        mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
        assert not point_in_polygon(mid, SQUARE_OBSTACLE.vertices)
    assert math.isclose(path[-1].x, goal[0]) and math.isclose(path[-1].y, goal[1])


def test_polygon_distance_is_zero_inside_and_perpendicular_outside():
    assert polygon_distance((120, 0), SQUARE_OBSTACLE.vertices) == 0.0
    assert math.isclose(polygon_distance((80, 0), SQUARE_OBSTACLE.vertices), 10.0)


def test_clear_standoff_puts_the_bumper_on_the_aim_point():
    field = make_field_with_obstacle()
    aim = (85, 0)  # on the box's near face
    x, y, heading = clear_standoff(
        field, aim, from_point=(0, 0), distance=14.0,
        width=28.0, length=28.0, side_local_angle=0.0,  # "front"
    )
    # Approached straight on from -x, so it parks 14in back along that line...
    assert math.isclose(x, 71.0, abs_tol=1e-6) and math.isclose(y, 0.0, abs_tol=1e-6)
    # ...with the front facing the aim, and the chassis in free space.
    assert math.isclose(heading, 0.0, abs_tol=1e-6)
    chassis = footprint_polygon((x, y), heading, 28.0, 28.0)
    assert not convex_overlap(chassis, SQUARE_OBSTACLE.vertices)


def test_clear_standoff_rotates_away_from_an_unreachable_approach():
    # Aim on the box's *far* face while the robot is on the near side:
    # approaching "straight from where I am" would park the chassis
    # inside the box, so it has to swing around to a real approach.
    field = make_field_with_obstacle()
    aim = (155, 0)  # just off the box's +x face
    x, y, heading = clear_standoff(
        field, aim, from_point=(0, 0), distance=14.0,
        width=28.0, length=28.0, side_local_angle=0.0,
    )
    chassis = footprint_polygon((x, y), heading, 28.0, 28.0)
    assert not convex_overlap(chassis, SQUARE_OBSTACLE.vertices)
    assert math.isclose(math.hypot(x - aim[0], y - aim[1]), 14.0, abs_tol=1e-6)


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


def test_navigate_to_rounds_an_obstacle_with_a_target_that_moves_every_tick():
    # A target derived from the robot's own live pose (Score's standoff,
    # Collect's approach) shifts a little every tick, so every tick
    # replans -- and a replan restarts the path at waypoint 0. Advancing
    # one waypoint per tick then caps progress at waypoint 1, which is
    # exactly the waypoint the robot is arriving at as it rounds a
    # corner: it parks a fraction of an inch short of it, creeping at a
    # speed proportional to a distance already inside the tolerance,
    # until the route's shape happens to change. On the REEFSCAPE field
    # that stalled robots at a REEF corner for four seconds at a time.
    hexagon = _hexagon((150, 100), 32.75)  # a REEF, in the middle of the route
    field = FieldConfig(width=300, height=200, obstacles=(Obstacle(name="reef", vertices=hexagon),))
    match = Match(field, TableScoringRules({}), MatchConfig(auto_duration=1000, teleop_duration=1000))
    robot = match.add_robot(make_characteristics(), Pose2d(30, 100, 0))

    goal = (270.0, 100.0)
    tick_count = [0]

    def jittering_target(ctx):
        # +/-0.6in every tick: never settles, always past
        # _TARGET_MOVE_EPSILON, so every single tick replans.
        tick_count[0] += 1
        return Pose2d(goal[0], goal[1] + (0.6 if tick_count[0] % 2 else -0.6), 0.0)

    nav = NavigateTo(jittering_target, heading_mode="face_travel", replan_period=0.25)
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    for _ in range(600):  # 10s -- the clear route is under 2s of driving
        nav.tick(ctx)
        match.step(ctx.dt)
        ctx.elapsed += ctx.dt
        assert not point_in_polygon((robot.pose.x, robot.pose.y), hexagon)

    remaining = math.hypot(robot.pose.x - goal[0], robot.pose.y - goal[1])
    assert remaining < 5.0, f"stopped {remaining:.1f}in short of the target -- stalled on a waypoint"


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


def test_plan_path_routes_out_of_an_obstacle_it_starts_inside():
    # A robot obstacle is inflated by both robots' radii plus a margin,
    # so a robot closing on another is inside that circle well before
    # they touch. Every edge out of `start` then crosses the polygon
    # `start` is already in, leaving A* with an unreachable goal and
    # plan_path falling back to the direct line -- avoidance switching
    # *off* at exactly the moment it was needed. The route must detour.
    field = FieldConfig(width=300, height=200)
    blocker = _octagon((100, 100), 45.0)
    path = plan_path(field, (80, 100), (250, 100), robot_radius=14.0, extra_obstacles=[blocker])

    assert len(path) > 2, "fell back to the straight line instead of routing around"
    # Only the first leg may be inside the circle -- that's the way out
    # of where the robot already is. Nothing after it may re-enter.
    for i in range(1, len(path) - 1):
        assert not _segment_hits_polygon(path[i], path[i + 1], blocker)
    # And the way out must be the near boundary, not straight across.
    assert path[1].x < 100, "escaped through the far side instead of routing around"


def test_plan_path_routes_around_an_obstacle_the_goal_is_inside():
    # Mirror image: a goal sitting inside another robot's circle (a
    # contested piece, a scoring standoff someone is parked on) must
    # still be approached around the obstacle, never straight through it.
    field = FieldConfig(width=300, height=200)
    blocker = _octagon((150, 100), 45.0)
    path = plan_path(field, (20, 100), (160, 100), robot_radius=14.0, extra_obstacles=[blocker])

    assert len(path) > 2, "fell back to the straight line instead of routing around"
    # Only the final leg may enter the circle, and only to reach the goal.
    for i in range(len(path) - 2):
        assert not _segment_hits_polygon(path[i], path[i + 1], blocker)


def test_plan_path_detour_stays_inside_the_field():
    # The field perimeter is a wall that never appears in
    # field.obstacles. A detour around something near the edge used to
    # be free to round it on the outside, through the wall -- the robot
    # drove into the wall and sat there pushing.
    field = FieldConfig(width=300, height=200)
    blocker = _octagon((150, 30), 45.0)
    path = plan_path(field, (60, 30), (250, 30), robot_radius=14.0, extra_obstacles=[blocker])

    for point in path[1:-1]:
        assert 14.0 <= point.x <= 300 - 14.0
        assert 14.0 <= point.y <= 200 - 14.0


def test_other_robot_obstacle_never_shrinks_below_contact():
    # The obstacle may give back its comfort margin so a goal near
    # another robot stays directly reachable, but never the radius at
    # which the two chassis actually touch: shrinking into that let the
    # planner route a line through a body it was supposed to avoid and
    # drive into it at full speed.
    field = FieldConfig(width=300, height=200)
    match = Match(field, TableScoringRules({}), MatchConfig(auto_duration=1000, teleop_duration=1000))
    robot_a = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    robot_b = match.add_robot(make_characteristics(), Pose2d(150, 100, math.pi))

    nav = NavigateTo(lambda ctx: Pose2d(155, 100, 0))
    own_radius = math.hypot(14.0, 14.0)
    contact = own_radius * 2.0
    # Goal 5in from robot_b -- the case that used to collapse the
    # obstacle to a 4in circle sitting inside robot_b's own chassis.
    obstacles = nav._other_robot_obstacles(robot_a, own_radius, match, (20, 100), (155, 100))

    assert len(obstacles) == 1
    for x, y in obstacles[0]:
        assert math.hypot(x - 150, y - 100) >= contact - 1e-6


def test_head_on_robots_pass_on_opposite_sides():
    # Two robots meeting head-on are mirror images, and a symmetric
    # obstacle hands them mirror-image shortest paths: both dodge the
    # same way, meet there, and shove -- a livelock replanning can't
    # break. Each biases the other to its own left, so both keep right.
    field = FieldConfig(width=600, height=200)
    match = Match(field, TableScoringRules({}), MatchConfig(auto_duration=1000, teleop_duration=1000))
    robot_a = match.add_robot(make_characteristics(), Pose2d(100, 100, 0))
    robot_b = match.add_robot(make_characteristics(), Pose2d(500, 100, math.pi))

    # Each goal is past the other robot but well clear of it, so what's
    # measured is which way each rounds the other -- not how it copes
    # with a destination someone is parked on (covered separately above).
    nav_a = NavigateTo(lambda ctx: Pose2d(580, 100, 0))
    nav_b = NavigateTo(lambda ctx: Pose2d(20, 100, 0))
    own_radius = math.hypot(14.0, 14.0)
    path_a = plan_path(field, (100, 100), (580, 100), own_radius,
                       extra_obstacles=nav_a._other_robot_obstacles(robot_a, own_radius, match, (100, 100), (580, 100)))
    path_b = plan_path(field, (500, 100), (20, 100), own_radius,
                       extra_obstacles=nav_b._other_robot_obstacles(robot_b, own_radius, match, (500, 100), (20, 100)))

    # Each detours off the shared y=100 line, and to opposite sides of it.
    offset_a = max((p.y - 100 for p in path_a), key=abs)
    offset_b = max((p.y - 100 for p in path_b), key=abs)
    assert abs(offset_a) > 1.0 and abs(offset_b) > 1.0
    assert offset_a * offset_b < 0, "both robots dodged to the same side"


def _segment_hits_polygon(a, b, poly) -> bool:
    """Whether segment a->b passes through `poly`'s interior, sampled
    along its length. Tested against `poly` shrunk slightly toward its
    own centroid, so a leg that legitimately rides the boundary (every
    route around an obstacle hugs it) doesn't read as a hit, while a leg
    that genuinely cuts through still does."""
    ax, ay = (a.x, a.y) if hasattr(a, "x") else a
    bx, by = (b.x, b.y) if hasattr(b, "x") else b
    cx = sum(p[0] for p in poly) / len(poly)
    cy = sum(p[1] for p in poly) / len(poly)
    inner = tuple((cx + (x - cx) * 0.95, cy + (y - cy) * 0.95) for x, y in poly)
    for i in range(1, 20):
        t = i / 20.0
        if point_in_polygon((ax + (bx - ax) * t, ay + (by - ay) * t), inner):
            return True
    return False


def _empty_match(width=600, height=400):
    field = FieldConfig(width=width, height=height)
    return Match(field, TableScoringRules({}), MatchConfig(auto_duration=1000, teleop_duration=1000))


def test_speed_is_capped_on_remaining_path_not_the_next_waypoint():
    """Intermediate waypoints are corners to drive through, not stops.
    Gaining on the distance to the next one made the robot brake to a
    crawl at every corner -- invisible on a straight run, which has no
    intermediate waypoints, and crippling on a detour, which is exactly
    what robot avoidance inserts."""
    match = _empty_match()
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    nav = NavigateTo(lambda ctx: Pose2d(560, 100, 0))
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0, match=match)
    nav.tick(ctx)

    # A nearby waypoint partway along a long, straight route. Braking for
    # it would command 20 * speed_gain = 60 in/s instead of full speed.
    nav._path = [Vec2d(20, 100), Vec2d(40, 100), Vec2d(560, 100)]
    nav._waypoint_index = 1
    nav._replan_timer = 10.0

    commanded = []
    robot.drive_field_relative = lambda dt, vx, vy, omega: commanded.append(math.hypot(vx, vy))
    nav.tick(ctx)
    assert math.isclose(commanded[-1], robot.characteristics.max_speed, rel_tol=1e-9)


def test_moving_robot_also_gets_an_obstacle_where_it_is_heading():
    """A snapshot obstacle is where someone *was* at plan time. Planning
    against where they're going too is what commits the detour early,
    while it is still cheap."""
    match = _empty_match()
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0))
    other = match.add_robot(make_characteristics(), Pose2d(300, 100, 0))
    nav = NavigateTo(lambda ctx: Pose2d(560, 100, 0))
    start, goal = (20.0, 100.0), (560.0, 100.0)

    parked = nav._other_robot_obstacles(robot, 20.0, match, start, goal)
    other.chassis.body.velocity = (0.0, 100.0)
    moving = nav._other_robot_obstacles(robot, 20.0, match, start, goal)

    assert len(moving) == len(parked) + 1
    centers = [(sum(p[1] for p in poly) / len(poly)) for poly in moving]
    assert max(centers) > 100.0 + _PREDICTION_MIN_SHIFT, "predicted obstacle is not ahead of it"
    # ...and the robot's actual position is still covered. A couple of
    # inches of slack: the pass-side bulge pulls an octagon's centroid
    # slightly off the center it was built around.
    assert math.isclose(min(centers), 100.0, abs_tol=3.0)
