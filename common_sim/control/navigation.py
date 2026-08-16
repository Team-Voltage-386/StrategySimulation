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
    """Offset a convex polygon outward by `radius` along its own edge
    normals -- the Minkowski sum with a disc of that radius, with each
    rounded corner left as the sharp intersection of its two adjacent
    offset edges (outside the true offset, so conservative).

    This used to scale the vertices radially away from the centroid
    instead, which only gives the full `radius` *at* the vertices: every
    edge between them fell short by cos(half the vertex angle) -- 13% on
    a hexagon, so a REEF face inflated by a 19.8in robot radius only
    stood 17.1in off. A planner hugs the inflated polygon exactly, so
    those missing inches are inches of robot inside the real structure,
    which is what had robots clipping the REEF's corners on the way
    past."""
    n = len(vertices)
    if n < 3 or radius <= 0.0:
        return tuple(vertices)
    cx = sum(v[0] for v in vertices) / n
    cy = sum(v[1] for v in vertices) / n

    # Each edge's offset line, as (point on it, unit direction).
    lines: list[tuple[Point, Point] | None] = []
    for i in range(n):
        ax, ay = vertices[i]
        bx, by = vertices[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        length = math.hypot(ex, ey)
        if length < 1e-9:  # duplicate vertex -- no edge to offset
            lines.append(None)
            continue
        dx, dy = ex / length, ey / length
        nx, ny = dy, -dx
        if nx * ((ax + bx) / 2.0 - cx) + ny * ((ay + by) / 2.0 - cy) < 0.0:
            nx, ny = -nx, -ny  # point the normal away from the interior
        lines.append(((ax + nx * radius, ay + ny * radius), (dx, dy)))

    inflated = []
    for i in range(n):
        # Vertex i is where edges i-1 and i meet, so its offset copy is
        # where those two offset lines meet. Collinear or degenerate
        # edges have no single intersection -- there the radial estimate
        # is exact anyway (a straight-through "corner" turns no corner).
        point = _line_intersection(lines[i - 1], lines[i])
        inflated.append(point if point is not None else _radial(vertices[i], (cx, cy), radius))
    return tuple(inflated)


def _line_intersection(a: tuple[Point, Point] | None, b: tuple[Point, Point] | None) -> Point | None:
    if a is None or b is None:
        return None
    (p1, d1), (p2, d2) = a, b
    cross = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(cross) < 1e-9:
        return None
    t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / cross
    return (p1[0] + d1[0] * t, p1[1] + d1[1] * t)


def _radial(vertex: Point, center: Point, radius: float) -> Point:
    dx, dy = vertex[0] - center[0], vertex[1] - center[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return vertex
    scale = (dist + radius) / dist
    return (center[0] + dx * scale, center[1] + dy * scale)


def _segment_distance(point: Point, a: Point, b: Point) -> float:
    ex, ey = b[0] - a[0], b[1] - a[1]
    length_sq = ex * ex + ey * ey
    if length_sq < 1e-12:
        return math.hypot(point[0] - a[0], point[1] - a[1])
    t = max(0.0, min(1.0, ((point[0] - a[0]) * ex + (point[1] - a[1]) * ey) / length_sq))
    return math.hypot(point[0] - (a[0] + ex * t), point[1] - (a[1] + ey * t))


def polygon_distance(point: Point, vertices: tuple[Point, ...]) -> float:
    """Distance from `point` to a polygon's boundary, or 0.0 if `point`
    is inside it."""
    if point_in_polygon(point, vertices):
        return 0.0
    n = len(vertices)
    return min(_segment_distance(point, vertices[i], vertices[(i + 1) % n]) for i in range(n))


def footprint_polygon(center: Point, heading: float, width: float, length: float) -> tuple[Point, ...]:
    """A robot's chassis rectangle in world coordinates, given where its
    center is and which way it faces (+x local is `length`-wise, matching
    RobotCharacteristics and SIDE_OUTWARD's "front")."""
    half_l, half_w = length / 2.0, width / 2.0
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    return tuple(
        (center[0] + x * cos_h - y * sin_h, center[1] + x * sin_h + y * cos_h)
        for x, y in ((half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w))
    )


def convex_overlap(a: tuple[Point, ...], b: tuple[Point, ...]) -> bool:
    """Separating-axis test for two convex polygons. Exactly touching
    counts as clear, so a footprint flush against a structure's face --
    the whole point of a scoring standoff -- isn't rejected as a
    collision."""
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
            axis = (y1 - y2, x2 - x1)
            length = math.hypot(*axis)
            if length < 1e-9:
                continue
            axis = (axis[0] / length, axis[1] / length)
            a_min, a_max = _project(a, axis)
            b_min, b_max = _project(b, axis)
            if a_max <= b_min + 1e-9 or b_max <= a_min + 1e-9:
                return False
    return True


def _project(poly: tuple[Point, ...], axis: Point) -> tuple[float, float]:
    values = [p[0] * axis[0] + p[1] * axis[1] for p in poly]
    return min(values), max(values)


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


# Fractions along a candidate segment sampled for "is this inside an
# obstacle". Deliberately not symmetric about 0.5 and never 0 or 1: the
# endpoints are usually obstacle vertices, where point_in_polygon is a
# coin flip on floating-point noise.
_VISIBILITY_SAMPLES = (0.07, 0.23, 0.41, 0.5, 0.59, 0.77, 0.93)


def _visible(p1: Point, p2: Point, inflated_obstacles: list[tuple[Point, ...]]) -> bool:
    if _close(p1, p2):
        return True
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    for poly in inflated_obstacles:
        n = len(poly)
        for i in range(n):
            a, b = poly[i], poly[(i + 1) % n]
            if _segments_intersect(p1, p2, a, b):
                if not (_close(p1, a) or _close(p1, b) or _close(p2, a) or _close(p2, b)):
                    return False
        # The edge-crossing test above only catches a *strict* straddle,
        # so a segment that enters or leaves exactly through a vertex
        # slips through it: at a vertex, neither of the two edges meeting
        # there is straddled. That is not a measure-zero curiosity here.
        # `_octagon` puts a vertex straight along +x, which is exactly
        # where a robot approaching along that axis exits; and any
        # diagonal between two vertices of a convex polygon crosses it
        # while touching nothing but its own endpoints. One midpoint
        # sample used to be the backstop, and it misses both whenever the
        # midpoint happens to land outside. Sampling along the segment
        # catches them.
        for t in _VISIBILITY_SAMPLES:
            if point_in_polygon((p1[0] + dx * t, p1[1] + dy * t), poly):
                return False
    return True


# How many of a trapping polygon's nearest vertices an endpoint inside
# it may escape through -- see `plan_path`'s `escape_nodes`. Three is
# enough that an octagon always offers a way out to either side, few
# enough that cutting clear across it is never one of them.
_ESCAPE_VERTICES = 3


# How far a robot obstacle is bulged on the planning robot's left, as a
# fraction of its radius. Big enough that the two ways around are never
# a near-tie for the A* heuristic to split on noise, small enough that
# the detour it picks is still roughly the short way.
_PASS_SIDE_BIAS = 0.1


def _octagon(center: Point, radius: float, observer: Point | None = None) -> tuple[Point, ...]:
    """Cheap circle approximation for a moving robot treated as a round
    dynamic obstacle -- 8 vertices is plenty for the visibility graph
    planner and, unlike a 4-gon's flat faces, doesn't favor a pass side
    by accident.

    It does favor one on purpose, when `observer` (the planning robot's
    position) is given: the circle is bulged on the observer's left, so
    the cheaper way past is on its right. Two robots meeting head-on are
    mirror images of each other, and a symmetric obstacle hands them
    mirror-image shortest paths -- both dodge to the same side, meet
    there, and shove. Replanning can't break that: every replan hands
    each of them the same tie it just lost, so they grind against each
    other until something else knocks them loose. Biasing left breaks
    the tie the way road rules do -- both keep right, which for a
    head-on means opposite sides of the field -- and does it from each
    robot's own frame, so neither has to know what the other decided.
    The bulge only ever *adds* clearance, so the obstacle still covers
    the whole robot it stands for."""
    cx, cy = center
    left = math.atan2(cy - observer[1], cx - observer[0]) + math.pi / 2.0 if observer is not None else None
    points = []
    for i in range(8):
        angle = i * math.pi / 4.0
        scale = 1.0 if left is None else 1.0 + _PASS_SIDE_BIAS * max(0.0, math.cos(angle - left))
        points.append((cx + radius * scale * math.cos(angle), cy + radius * scale * math.sin(angle)))
    return tuple(points)


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
    caller already sized them for the pair of bodies involved.

    Either endpoint may lie *inside* an obstacle -- routine for the robot
    obstacles `NavigateTo` passes, which are sized to overlap long before
    two chassis do. Such an endpoint is routed out to (or in from) the
    near boundary rather than straight across; see `_edge_visible`. Route
    corners are also kept `robot_radius` clear of the field perimeter,
    which is a wall no FieldConfig lists as an obstacle."""
    inflated = []
    for obstacle in field.obstacles:
        clearance = _clearance_for_goal(obstacle.vertices, robot_radius, goal)
        if clearance is not None:
            inflated.append(_inflate(obstacle.vertices, clearance))
    if extra_obstacles:
        inflated.extend(extra_obstacles)

    nodes: list[Point] = [start, goal]
    # Which polygon each node came from (None for start/goal), and which
    # polygons each endpoint is sitting inside -- see `_edge_visible`.
    node_poly: list[int | None] = [None, None]
    trapped = {
        0: {i for i, poly in enumerate(inflated) if point_in_polygon(start, poly)},
        1: {i for i, poly in enumerate(inflated) if point_in_polygon(goal, poly)},
    }

    # An endpoint inside an obstacle is never "directly visible" through
    # it, whatever the segment does. `_visible` alone can miss that: it
    # asks whether the segment *crosses* an edge, and a segment leaving a
    # polygon exactly through one of its vertices straddles neither of
    # the two edges meeting there, so it reads as clear. A robot obstacle
    # is an octagon with a vertex straight along +x, which is precisely
    # where a robot approaching along that axis would exit -- so this is
    # the aligned head-on case, not a measure-zero curiosity.
    if not trapped[0] and not trapped[1] and _visible(start, goal, inflated):
        return [Vec2d(*start), Vec2d(*goal)]

    # The field perimeter is a wall too, and it's the one obstacle that
    # never appears in `field.obstacles` -- so a route hugging an
    # obstacle near the edge of the field used to be free to round it on
    # the outside, through the wall. A robot then drove into the wall and
    # sat there pushing, having "reached" nothing. Dropping the
    # out-of-bounds vertices makes A* round it on the inside instead.
    # `start` and `goal` are exempt: a robot legitimately parks closer to
    # the wall than its own radius (a CORAL STATION sits in the corner),
    # and the inset is only about where a *route* may turn.
    def _in_bounds(p: Point) -> bool:
        return (robot_radius <= p[0] <= field.width - robot_radius
                and robot_radius <= p[1] <= field.height - robot_radius)

    boundary_edges: set[tuple[int, int]] = set()
    for poly_index, poly in enumerate(inflated):
        n = len(poly)
        kept: dict[int, int] = {}
        for k, vertex in enumerate(poly):
            if _in_bounds(vertex):
                kept[k] = len(nodes)
                nodes.append(vertex)
                node_poly.append(poly_index)
        # A polygon's own consecutive vertices are always mutually
        # visible -- that segment *is* the polygon's boundary. Add them
        # unconditionally rather than through the general _visible()
        # check below: an edge's midpoint sits exactly on the polygon
        # boundary, where point_in_polygon's ray-casting is a coin flip
        # on floating-point noise, which would otherwise randomly sever
        # the one connection a path needs to hug the obstacle round
        # (e.g. a robot dead ahead, inflated to a circle-ish polygon).
        # Only between two surviving neighbors, though -- bridging across
        # a dropped one would cut the corner it was there to round.
        for k in range(n):
            nxt = (k + 1) % n
            if k in kept and nxt in kept:
                boundary_edges.add((kept[k], kept[nxt]))

    start_idx, goal_idx = 0, 1
    edges: dict[int, list[tuple[int, float]]] = {i: [] for i in range(len(nodes))}

    def _add_edge(i: int, j: int) -> None:
        dist = math.hypot(nodes[i][0] - nodes[j][0], nodes[i][1] - nodes[j][1])
        edges[i].append((j, dist))
        edges[j].append((i, dist))

    escape_obstacles = {
        endpoint: [poly for i, poly in enumerate(inflated) if i not in inside]
        for endpoint, inside in trapped.items()
    }
    # Which vertices a trapped endpoint may use to get out (or in): the
    # `_ESCAPE_VERTICES` nearest ones on each polygon trapping it. The
    # whole polygon would technically do -- an endpoint inside a convex
    # shape can see all of it -- but "all of it" includes the vertex
    # straight ahead on the far side, and A* would take it: the escape
    # then *is* the straight line through the obstacle, which is the
    # thing being avoided. Leaving by the near boundary and rounding it
    # is the detour.
    escape_nodes: dict[int, set[int]] = {}
    for endpoint, inside in trapped.items():
        allowed: set[int] = set()
        for poly_index in inside:
            members = [n for n in range(2, len(nodes)) if node_poly[n] == poly_index]
            members.sort(key=lambda n: math.hypot(
                nodes[n][0] - nodes[endpoint][0], nodes[n][1] - nodes[endpoint][1]))
            allowed.update(members[:_ESCAPE_VERTICES])
        escape_nodes[endpoint] = allowed

    def _edge_visible(i: int, j: int) -> bool:
        """Normal visibility, plus an escape hatch for an endpoint that
        has ended up *inside* an inflated obstacle.

        For robot obstacles that is the routine case, not a rare one:
        they're inflated by both bodies' radii plus a margin, so two
        robots closing on each other are inside each other's inflated
        polygon well before they touch, and a target next to a robot
        (a contested piece, an occupied scoring spot) is inside it from
        the start. Left alone it is also the worst possible case -- every
        edge out of that endpoint crosses the polygon it's already
        inside, so the endpoint gets no edges at all, A* reports the goal
        unreachable, and `plan_path` falls back to the direct
        start->goal line. Avoidance doesn't degrade there, it switches
        *off*, at exactly the moment it was needed: robots then drive
        straight at each other and shove until the match ends.

        So an edge from a trapped endpoint to a vertex of a polygon it is
        stuck inside is checked only against the polygons it is *not*
        stuck inside. Those edges are the way out (or the way in), and
        the endpoint already sits where they run. Every other polygon
        still blocks them, and -- crucially -- the start->goal edge
        itself is never exempted, since neither endpoint is a polygon
        vertex. So A* can't answer with the straight line it just
        rejected: it has to enter and leave via boundary vertices and
        route around, which is the detour we wanted."""
        endpoint, other = (i, j) if i in trapped else (j, i) if j in trapped else (None, None)
        if other is not None and other in escape_nodes[endpoint]:
            obstacles = escape_obstacles[endpoint]
        else:
            obstacles = inflated
        # Same vertex-exit blind spot as the direct start->goal check
        # above, so rule out an endpoint that is inside anything still
        # being enforced. Only start/goal are asked -- an obstacle vertex
        # sits *on* its own polygon, where point_in_polygon is a coin
        # flip, and severing those edges is what `boundary_edges` exists
        # to prevent.
        for node in (i, j):
            if node in trapped and any(point_in_polygon(nodes[node], poly) for poly in obstacles):
                return False
        return _visible(nodes[i], nodes[j], obstacles)

    for i, j in boundary_edges:
        _add_edge(i, j)

    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            if (i, j) in boundary_edges or (j, i) in boundary_edges:
                continue
            if _edge_visible(i, j):
                _add_edge(i, j)

    path_indices = _astar(nodes, edges, start_idx, goal_idx)
    if path_indices is None:
        return [Vec2d(*start), Vec2d(*goal)]
    return [Vec2d(*nodes[i]) for i in path_indices]


# How far outside an inflated obstacle a goal has to stay for the
# visibility graph to be able to reach it at all.
_GOAL_CLEARANCE = 1.0


def _clearance_for_goal(vertices: tuple[Point, ...], robot_radius: float, goal: Point) -> float | None:
    """How far to inflate one obstacle: `robot_radius`, capped so the
    inflated polygon never swallows `goal`. None means skip the obstacle
    entirely.

    Scoring targets sit right up against the structure being scored on
    (a REEF face's zone starts at the REEF wall), so a goal inside the
    inflated hex is the normal case, not a corner case. Left alone it is
    also the worst case: no visibility-graph edge reaches the goal, A*
    reports it unreachable, and `plan_path` falls back to the direct
    start->goal line -- dropping obstacle avoidance for the *entire*
    route, which is how a robot ends up driving through the REEF's
    corner on its way to a face. Capping trades a couple of inches of
    clearance on that one obstacle for keeping the detour at all, and is
    the same trade `NavigateTo._other_robot_obstacles` already makes for
    robots."""
    room = polygon_distance(goal, vertices) - _GOAL_CLEARANCE
    if room <= 0.0:
        # Goal on or inside the structure itself. Keeping the raw
        # polygon still bends the route around it; a goal genuinely
        # inside leaves nothing to plan around.
        return None if point_in_polygon(goal, vertices) else 0.0
    return min(robot_radius, room)


# Directions tried around a target when the straight-on approach would
# park the chassis in a structure. 24 gives 15-degree steps -- fine
# enough that a robot never gives up a workable approach angle, coarse
# enough to stay a trivial loop.
_STANDOFF_SAMPLES = 24


def clear_standoff(
    field: FieldConfig,
    aim: Point,
    from_point: Point,
    distance: float,
    *,
    width: float,
    length: float,
    side_local_angle: float,
    margin: float = 1.0,
) -> tuple[float, float, float]:
    """Where a robot should park to put one of its sides `distance` in
    front of its center onto `aim`, and the heading that presents that
    side (`side_local_angle` is the side's outward normal in the robot's
    own frame). Returns (x, y, heading).

    Prefers approaching from wherever the robot already is, and rotates
    away from that bearing only as far as it must to keep the chassis --
    its real `width` x `length` rectangle, not a point -- out of the
    field's obstacles. Falls back to the preferred bearing when no angle
    is clear, since a pose the robot can't quite reach still beats no
    target at all."""
    bearing = math.atan2(from_point[1] - aim[1], from_point[0] - aim[0])
    step = 2.0 * math.pi / _STANDOFF_SAMPLES
    preferred = None
    for offset in _spiral_offsets(step, _STANDOFF_SAMPLES):
        angle = bearing + offset
        center = (aim[0] + math.cos(angle) * distance, aim[1] + math.sin(angle) * distance)
        # The side's outward normal must point from the center at `aim`,
        # i.e. back along the standoff direction.
        heading = wrap_angle(angle + math.pi - side_local_angle)
        if preferred is None:
            preferred = (center[0], center[1], heading)
        chassis = footprint_polygon(center, heading, width + 2.0 * margin, length + 2.0 * margin)
        if not any(convex_overlap(chassis, o.vertices) for o in field.obstacles):
            return (center[0], center[1], heading)
    assert preferred is not None
    return preferred


def _spiral_offsets(step: float, samples: int):
    """0, +step, -step, +2*step, ... -- nearest-first around a preferred
    bearing, so the returned approach is the least rotation that works."""
    yield 0.0
    for i in range(1, samples // 2 + 1):
        yield i * step
        yield -i * step


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


def _robot_velocity(robot) -> tuple[float, float] | None:
    """Field-frame velocity of a robot, or None for anything that isn't
    backed by a physics body (test doubles, mostly)."""
    body = getattr(getattr(robot, "chassis", None), "body", None)
    velocity = getattr(body, "velocity", None)
    if velocity is None:
        return None
    return (float(velocity[0]), float(velocity[1]))


# Dynamic-obstacle prediction, used by `_other_robot_obstacles`.
#
# A robot obstacle is a snapshot of where someone *was* when the route
# was planned. Two robots closing at full speed cover a lot of ground
# before the next plan, so a route that cleared the snapshot can run
# straight into where the other robot actually got to -- which is why
# avoidance still ended in glancing contact even with no deadlock left.
# Planning against where they're *heading* commits the detour early,
# while it's still cheap, instead of reacting once they're on top of
# each other.
#
# The horizon caps how far ahead we're willing to believe a constant
# velocity; past a second a robot has usually turned or arrived, and
# extrapolating further just invents obstacles in empty space.
_PREDICTION_HORIZON = 1.0
# Below this the robot is parked or jittering and its heading means
# nothing worth planning around.
_PREDICTION_MIN_SPEED = 12.0
# A predicted position this close to the current one is already covered
# by the snapshot obstacle; adding it would only cost planning time.
_PREDICTION_MIN_SHIFT = 6.0


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

    `position_tolerance` is how close counts as arrived, and applies only
    to the last waypoint. Intermediate ones use the looser
    `waypoint_tolerance`: they are hints about which side of an obstacle
    to pass, not places to be, and hitting one precisely costs a stop.

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
        replan_period: float = 0.1,
        position_tolerance: float = 2.0,
        waypoint_tolerance: float = 3.0,
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
        self.waypoint_tolerance = waypoint_tolerance
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
        """Other robots as circular obstacles: `own_radius + their radius`
        (the separation at which the two chassis can still touch, since
        both radii circumscribe the chassis) plus `robot_avoid_margin` of
        breathing room.

        The margin -- and only the margin -- is given back when `goal`
        sits inside the circle, so a target that merely wants a polite
        berth from a nearby robot can still be planned straight to. What
        is never given back is the contact radius itself. Trimming into
        it used to be allowed, which meant a goal close to another robot
        (a contested piece, a scoring standoff someone is parked on)
        shrank that robot's obstacle to *smaller than the robot*, and the
        planner would route a line through a body it was supposed to
        avoid and drive into it at full speed. That is one half of a
        permanent deadlock: the shover pins its victim against the REEF,
        the victim can't get out from under it, and neither ever
        re-plans into anything different.

        A goal inside the contact radius is instead left inside the
        obstacle, where `plan_path`'s trapped-endpoint handling routes
        around to a boundary vertex and approaches from there. Same for
        `start`: two robots closing on each other are inside each other's
        circle well before they touch, which is the normal trigger for a
        detour, not a degenerate input to be shrunk away.

        A moving robot also gets a second obstacle at where it is
        predicted to be by the time we reach it -- see the prediction
        constants above. That one is soft: it is dropped entirely rather
        than floored at the contact radius when it would swallow the
        goal, because a prediction is a guess about a robot that may
        well turn away, and a guess must never be able to make a
        destination unreachable. The snapshot at the robot's actual
        position is the hard constraint; this one only buys an earlier,
        cheaper detour."""
        if not self.avoid_robots or match is None:
            return []
        others = getattr(match, "robots", None)
        if not others:
            return []
        own_speed = max(1.0, robot.characteristics.max_speed)
        obstacles = []
        for other in others:
            if other is robot:
                continue
            other_pos = (other.pose.x, other.pose.y)
            contact_radius = own_radius + _robot_radius(other.characteristics)
            clearance_to_goal = math.hypot(other_pos[0] - goal[0], other_pos[1] - goal[1]) - 1.0
            radius = max(contact_radius, min(contact_radius + self.robot_avoid_margin, clearance_to_goal))
            obstacles.append(_octagon(other_pos, radius, observer=start))

            velocity = _robot_velocity(other)
            if velocity is None or math.hypot(*velocity) < _PREDICTION_MIN_SPEED:
                continue
            # How long until we get there, straight-line and optimistic:
            # overestimating the horizon puts the predicted obstacle
            # somewhere we'll never meet it.
            reach_time = min(
                _PREDICTION_HORIZON,
                math.hypot(other_pos[0] - start[0], other_pos[1] - start[1]) / own_speed,
            )
            shift = (velocity[0] * reach_time, velocity[1] * reach_time)
            if math.hypot(*shift) < _PREDICTION_MIN_SHIFT:
                continue
            predicted = (other_pos[0] + shift[0], other_pos[1] + shift[1])
            clearance = math.hypot(predicted[0] - goal[0], predicted[1] - goal[1]) - 1.0
            predicted_radius = min(contact_radius + self.robot_avoid_margin, clearance)
            if predicted_radius > contact_radius:
                obstacles.append(_octagon(predicted, predicted_radius, observer=start))
        return obstacles

    # How far a target has to drift before it's worth rebuilding the
    # whole visibility graph for it. A target derived from the robot's
    # own live pose (Score's standoff, Collect's approach) jitters by
    # thousandths of an inch every tick; replanning on that is pure
    # cost, and the route it returns is the same one.
    _TARGET_MOVE_EPSILON = 0.5

    def _target_moved(self, target: Pose2d) -> bool:
        if self._last_target is None:
            return True
        return math.hypot(target.x - self._last_target.x, target.y - self._last_target.y) > self._TARGET_MOVE_EPSILON

    def tick(self, ctx: BehaviorContext) -> Status:
        robot = ctx.robot
        target = self.target_provider(ctx)
        field = getattr(ctx.match, "field", None) if ctx.match is not None else None

        self._replan_timer -= ctx.dt
        if self._path is None or self._replan_timer <= 0.0 or self._target_moved(target):
            self._replan(robot, target, field, match=ctx.match)
            self._replan_timer = self.replan_period
            self._last_target = target

        assert self._path is not None
        pose = robot.pose

        # Skip every waypoint already reached, not just one per tick. A
        # replan restarts the path at index 0 -- and index 0 is the
        # robot's own position, so it's always "reached" -- which means a
        # single advance per tick caps progress at waypoint 1. That is
        # fine while replans are 0.25s apart, but a target that moves
        # every tick (any Score target, since clear_standoff picks its
        # approach from where the robot currently is) replans every tick,
        # and then the robot can *never* get past waypoint 1: it creeps
        # toward it at a speed proportional to a distance that is already
        # inside the tolerance, and sits there until the route's shape
        # happens to change. That's what parked robots a few inches shy
        # of an inflated REEF corner for seconds at a time.
        while self._waypoint_index < len(self._path) - 1:
            if (self._path[self._waypoint_index] - pose.translation).length > self.waypoint_tolerance:
                break
            self._waypoint_index += 1

        waypoint = self._path[self._waypoint_index]
        delta = waypoint - pose.translation
        distance = delta.length
        is_final = self._waypoint_index == len(self._path) - 1

        # Distance left to drive, not distance to the next corner. The
        # speed command below is proportional to this, so measuring it to
        # the next waypoint made the robot brake for every intermediate
        # one as though it were the destination -- invisible on a
        # straight run, which has none, and crippling the moment
        # avoidance inserted a detour corner. It braked to a crawl to
        # round each robot it passed, which is the "slows down to get
        # by" everyone was watching. Braking belongs to the end of the
        # path; corners are driven through.
        remaining = distance
        for i in range(self._waypoint_index, len(self._path) - 1):
            remaining += self._path[i].get_distance(self._path[i + 1])

        desired_heading = self._desired_heading(pose, waypoint, target, delta)
        heading_error = wrap_angle(desired_heading - pose.heading)

        if distance <= self.position_tolerance and is_final and abs(heading_error) <= self.heading_tolerance:
            robot.drive_field_relative(ctx.dt, 0.0, 0.0, 0.0)
            return Status.SUCCESS

        vx, vy = 0.0, 0.0
        if distance > 1e-6:
            direction = delta / distance
            speed = min(robot.characteristics.max_speed, remaining * self.speed_gain)
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
    "clear_standoff",
    "convex_overlap",
    "footprint_polygon",
    "polygon_distance",
    "NavigateTo",
]
