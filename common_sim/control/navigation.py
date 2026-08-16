"""
Obstacle-aware driving. `DriveToPose` (behavior.py) is a pure P-controller
with zero obstacle awareness -- fine for a synthetic game with an empty
field, but on a real field with 2-4 convex obstacles (REEF hexes, a
charge station, ...) it will happily drive a robot straight into one.
This module adds a cheap, deterministic, re-plannable path layer on top
of the same P-control math, without touching DriveToPose or anything
that depends on it.
"""
from __future__ import annotations

import math
from typing import Callable, Union

from common_sim.control.behavior import Behavior, BehaviorContext, Status
from common_sim.field.field_config import FieldConfig, point_in_polygon
from common_sim.geometry import Pose2d, Vec2d, wrap_angle

Point = tuple[float, float]

HeadingMode = Union[str, float]


def _inflate(vertices: tuple[Point, ...], radius: float) -> tuple[Point, ...]:
    """Radially expand a convex polygon's vertices away from its own
    centroid by `radius`. Not a true Minkowski-sum offset (corners of a
    non-round polygon end up inflated a bit less than `radius` along
    their edge normals) -- an acceptable approximation for the small,
    roughly-regular obstacles (hex REEFs, etc.) this sim's fields use,
    and cheap/deterministic, which matters more here than exactness."""
    n = len(vertices)
    cx = sum(v[0] for v in vertices) / n
    cy = sum(v[1] for v in vertices) / n
    inflated = []
    for x, y in vertices:
        dx, dy = x - cx, y - cy
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            inflated.append((x, y))
            continue
        scale = (dist + radius) / dist
        inflated.append((cx + dx * scale, cy + dy * scale))
    return tuple(inflated)


def _segments_intersect(p1: Point, p2: Point, p3: Point, p4: Point) -> bool:
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    d1 = cross(p3, p4, p1)
    d2 = cross(p3, p4, p2)
    d3 = cross(p1, p2, p3)
    d4 = cross(p1, p2, p4)
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    return False


def _close(a: Point, b: Point, tol: float = 1e-6) -> bool:
    return math.hypot(a[0] - b[0], a[1] - b[1]) <= tol


def _visible(p1: Point, p2: Point, inflated_obstacles: list[tuple[Point, ...]]) -> bool:
    if _close(p1, p2):
        return True
    mid = ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)
    for poly in inflated_obstacles:
        n = len(poly)
        for i in range(n):
            a, b = poly[i], poly[(i + 1) % n]
            if _segments_intersect(p1, p2, a, b):
                if not (_close(p1, a) or _close(p1, b) or _close(p2, a) or _close(p2, b)):
                    return False
        if point_in_polygon(mid, poly):
            return False
    return True


def _octagon(center: Point, radius: float) -> tuple[Point, ...]:
    """Cheap circle approximation for a moving robot treated as a
    round dynamic obstacle -- 8 vertices is plenty for the visibility
    graph planner and avoids favoring any particular pass side the way
    a 4-gon's flat faces would."""
    cx, cy = center
    return tuple(
        (cx + radius * math.cos(a), cy + radius * math.sin(a))
        for a in (i * math.pi / 4.0 for i in range(8))
    )


def plan_path(
    field: FieldConfig,
    start: Point,
    goal: Point,
    robot_radius: float,
    extra_obstacles: list[tuple[Point, ...]] | None = None,
) -> list[Vec2d]:
    """Shortest path from `start` to `goal` that stays clear of
    `field.obstacles`, each inflated by `robot_radius`, via a visibility
    graph (nodes = start, goal, and every inflated obstacle vertex) and
    A*. Cheap and deterministic for the 2-4 convex obstacles this sim's
    fields declare -- not a general-purpose planner. Falls back to the
    direct line if no obstacle-clear path exists at all (shouldn't
    happen on a field with any legal path, but never leaves a tactic
    with no waypoints to drive toward).

    `extra_obstacles`, when given, are additional already-inflated
    polygons (e.g. other robots' footprints) folded into the same
    visibility graph -- not re-inflated by `robot_radius`, since the
    caller already sized them for the pair of bodies involved."""
    inflated = [_inflate(o.vertices, robot_radius) for o in field.obstacles]
    if extra_obstacles:
        inflated.extend(extra_obstacles)

    if _visible(start, goal, inflated):
        return [Vec2d(*start), Vec2d(*goal)]

    nodes: list[Point] = [start, goal]
    boundary_edges: set[tuple[int, int]] = set()
    for poly in inflated:
        base = len(nodes)
        n = len(poly)
        nodes.extend(poly)
        # A polygon's own consecutive vertices are always mutually
        # visible -- that segment *is* the polygon's boundary. Add them
        # unconditionally rather than through the general _visible()
        # check below: an edge's midpoint sits exactly on the polygon
        # boundary, where point_in_polygon's ray-casting is a coin flip
        # on floating-point noise, which would otherwise randomly sever
        # the one connection a path needs to hug the obstacle round
        # (e.g. a robot dead ahead, inflated to a circle-ish polygon).
        for i in range(n):
            boundary_edges.add((base + i, base + (i + 1) % n))

    start_idx, goal_idx = 0, 1
    edges: dict[int, list[tuple[int, float]]] = {i: [] for i in range(len(nodes))}

    def _add_edge(i: int, j: int) -> None:
        dist = math.hypot(nodes[i][0] - nodes[j][0], nodes[i][1] - nodes[j][1])
        edges[i].append((j, dist))
        edges[j].append((i, dist))

    for i, j in boundary_edges:
        _add_edge(i, j)

    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            if (i, j) in boundary_edges or (j, i) in boundary_edges:
                continue
            if _visible(nodes[i], nodes[j], inflated):
                _add_edge(i, j)

    path_indices = _astar(nodes, edges, start_idx, goal_idx)
    if path_indices is None:
        return [Vec2d(*start), Vec2d(*goal)]
    return [Vec2d(*nodes[i]) for i in path_indices]


def _astar(nodes: list[Point], edges: dict[int, list[tuple[int, float]]], start: int, goal: int) -> list[int] | None:
    import heapq

    def h(i):
        return math.hypot(nodes[i][0] - nodes[goal][0], nodes[i][1] - nodes[goal][1])

    open_set = [(h(start), start)]
    came_from: dict[int, int] = {}
    g_score = {start: 0.0}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            return list(reversed(path))
        for neighbor, weight in edges[current]:
            tentative = g_score[current] + weight
            if tentative < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative
                came_from[neighbor] = current
                heapq.heappush(open_set, (tentative + h(neighbor), neighbor))
    return None


def _path_length(path: list[Vec2d]) -> float:
    return sum(path[i].get_distance(path[i + 1]) for i in range(len(path) - 1))


def estimate_travel_time(field: FieldConfig, start: Point, goal: Point, characteristics) -> float:
    """Time to cover `plan_path`'s route under a trapezoidal velocity
    profile (accelerate at max_accel to max_speed, cruise, decelerate --
    or a triangular profile if the distance is too short to reach
    max_speed) -- no simulation required, so a planner can score many
    candidate options per tick cheaply."""
    radius = _robot_radius(characteristics)
    path = plan_path(field, start, goal, radius)
    distance = _path_length(path)
    return _trapezoidal_time(distance, characteristics.max_speed, characteristics.max_accel)


def _robot_radius(characteristics) -> float:
    return math.hypot(characteristics.length / 2.0, characteristics.width / 2.0)


def _trapezoidal_time(distance: float, max_speed: float, max_accel: float) -> float:
    if distance <= 1e-9:
        return 0.0
    accel_dist = max_speed * max_speed / max_accel
    if distance >= accel_dist:
        return 2.0 * (max_speed / max_accel) + (distance - accel_dist) / max_speed
    return 2.0 * math.sqrt(distance / max_accel)


class NavigateTo(Behavior):
    """Drives toward a (possibly moving) target via `plan_path`,
    following waypoints with the same P-control math `DriveToPose` uses,
    re-planning on a timer so a moving target or a piece taken by
    someone else doesn't leave the robot committed to a stale route.

    `target_provider(ctx) -> Pose2d` is called fresh each tick so a
    tactic can retarget without rebuilding this node.
    `heading_mode`: "face_travel" (face the next waypoint), "face_target"
    (face `target_provider(ctx)`'s own heading -- e.g. Score computes the
    heading that presents its scoring side to the region and bakes it
    into the target pose), or a fixed angle in radians.
    `standoff` pulls the final waypoint back toward the prior one by that
    many inches, so the robot stops short of the target instead of
    driving to its exact center.

    `avoid_robots` folds every other robot on the field into the same
    replan as a circular dynamic obstacle (see `_replan`), so two
    robots on a collision course route around each other instead of
    shoving head-on and stalling forever. Tactics whose whole point is
    to make contact or hold a blocking pose (`Defend`) pass False.
    """

    def __init__(
        self,
        target_provider: Callable[[BehaviorContext], Pose2d],
        *,
        heading_mode: HeadingMode = "face_travel",
        standoff: float = 0.0,
        replan_period: float = 0.25,
        position_tolerance: float = 2.0,
        heading_tolerance: float = 0.05,
        speed_gain: float = 3.0,
        heading_gain: float = 4.0,
        avoid_robots: bool = True,
        robot_avoid_margin: float = 6.0,
    ):
        self.target_provider = target_provider
        self.heading_mode = heading_mode
        self.standoff = standoff
        self.replan_period = replan_period
        self.position_tolerance = position_tolerance
        self.heading_tolerance = heading_tolerance
        self.speed_gain = speed_gain
        self.heading_gain = heading_gain
        self.avoid_robots = avoid_robots
        self.robot_avoid_margin = robot_avoid_margin

        self._path: list[Vec2d] | None = None
        self._waypoint_index = 0
        self._replan_timer = 0.0
        self._last_target: Pose2d | None = None

    def reset(self) -> None:
        self._path = None
        self._waypoint_index = 0
        self._replan_timer = 0.0
        self._last_target = None

    def _replan(self, robot, target: Pose2d, field: FieldConfig | None, match=None) -> None:
        start = (robot.pose.x, robot.pose.y)
        goal = (target.x, target.y)
        if field is not None:
            own_radius = _robot_radius(robot.characteristics)
            extra_obstacles = self._other_robot_obstacles(robot, own_radius, match, start, goal)
            path = plan_path(field, start, goal, own_radius, extra_obstacles=extra_obstacles)
        else:
            path = [Vec2d(*start), Vec2d(*goal)]

        if self.standoff > 0.0 and len(path) >= 2:
            last, prev = path[-1], path[-2]
            direction = last - prev
            length = direction.length
            if length > 1e-6 and length > self.standoff:
                path[-1] = last - direction.scale_to_length(self.standoff)

        self._path = path
        self._waypoint_index = 0

    def _other_robot_obstacles(
        self, robot, own_radius: float, match, start: Point, goal: Point
    ) -> list[tuple[Point, ...]]:
        """Other robots as circular obstacles, shrunk so they never swallow
        `goal` outright. Without the shrink, a robot standing right on or
        near where this robot is headed (a shared piece, a contested
        scoring spot) leaves the visibility graph with no edge reaching
        goal_idx at all; A* reports unreachable, and `plan_path` falls all
        the way back to the direct start->goal line -- silently dropping
        avoidance for the whole route, not just the last few inches, which
        is exactly the nose-to-nose shoving this is meant to prevent.
        Shrinking keeps the obstacle's push-back everywhere else on the
        route while letting the final approach actually reach a point
        near another robot instead of stalling short of it forever.

        Deliberately *not* shrunk against `start` the same way: as two
        robots close on each other, `start` (each one's own current
        position, refreshed every replan) naturally ends up near/inside
        the other's obstacle well before their bodies actually touch --
        that's the normal, expected trigger for a detour, not a
        degenerate input. Excluding this obstacle from blocking `start`'s
        own edges to suppress that non-issue was tried and made things
        worse: it also suppressed the direct start->goal edge itself,
        since that edge shares the now-exempted endpoint, so the two
        robots stopped detouring at all right as they closed in."""
        if not self.avoid_robots or match is None:
            return []
        others = getattr(match, "robots", None)
        if not others:
            return []
        obstacles = []
        for other in others:
            if other is robot:
                continue
            other_pos = (other.pose.x, other.pose.y)
            other_radius = _robot_radius(other.characteristics)
            radius = own_radius + other_radius + self.robot_avoid_margin
            clearance_to_goal = math.hypot(other_pos[0] - goal[0], other_pos[1] - goal[1]) - 1.0
            radius = min(radius, clearance_to_goal)
            if radius <= 0.0:
                continue
            obstacles.append(_octagon(other_pos, radius))
        return obstacles

    def tick(self, ctx: BehaviorContext) -> Status:
        robot = ctx.robot
        target = self.target_provider(ctx)
        field = getattr(ctx.match, "field", None) if ctx.match is not None else None

        self._replan_timer -= ctx.dt
        target_moved = self._last_target is None or (target.x, target.y) != (self._last_target.x, self._last_target.y)
        if self._path is None or self._replan_timer <= 0.0 or target_moved:
            self._replan(robot, target, field, match=ctx.match)
            self._replan_timer = self.replan_period
            self._last_target = target

        assert self._path is not None
        waypoint = self._path[self._waypoint_index]
        pose = robot.pose
        delta = waypoint - pose.translation
        distance = delta.length
        is_final = self._waypoint_index == len(self._path) - 1

        desired_heading = self._desired_heading(pose, waypoint, target, delta)
        heading_error = wrap_angle(desired_heading - pose.heading)

        at_waypoint = distance <= self.position_tolerance
        if at_waypoint and is_final and abs(heading_error) <= self.heading_tolerance:
            robot.drive_field_relative(ctx.dt, 0.0, 0.0, 0.0)
            return Status.SUCCESS
        if at_waypoint and not is_final:
            self._waypoint_index += 1
            waypoint = self._path[self._waypoint_index]
            delta = waypoint - pose.translation
            distance = delta.length
            is_final = self._waypoint_index == len(self._path) - 1
            desired_heading = self._desired_heading(pose, waypoint, target, delta)
            heading_error = wrap_angle(desired_heading - pose.heading)

        vx, vy = 0.0, 0.0
        if distance > 1e-6:
            direction = delta / distance
            speed = min(robot.characteristics.max_speed, distance * self.speed_gain)
            vx, vy = direction.x * speed, direction.y * speed
        max_omega = robot.characteristics.max_angular_speed
        omega = max(-max_omega, min(max_omega, heading_error * self.heading_gain))

        robot.drive_field_relative(ctx.dt, vx, vy, omega)
        return Status.RUNNING

    def _desired_heading(self, pose: Pose2d, waypoint: Vec2d, target: Pose2d, delta: Vec2d) -> float:
        if self.heading_mode == "face_travel":
            return math.atan2(delta.y, delta.x) if delta.length > 1e-6 else pose.heading
        if self.heading_mode == "face_target":
            return target.heading
        if isinstance(self.heading_mode, (int, float)):
            return float(self.heading_mode)
        raise ValueError(f"unknown heading_mode {self.heading_mode!r}")


__all__ = [
    "plan_path",
    "estimate_travel_time",
    "NavigateTo",
]
