"""Visibility Graph + Dijkstra path planner.

Nothing named "visibility graph" was ever found in TurtleRabbit's own git
history (checked: full history of `2025-TeamControl` across all 31 branches,
and `2026-TeamControl`'s `main` branch -- every commit message, every
filename ever committed, and every blob's content). This is a fresh
implementation, built from two primary sources instead of recovered code:

1. **Warthog Robotics' 2025 SSL TDP** (the RoboCup team, confirmed via their
   published paper): "treating each object in the field as a polygon. For
   each of its vertices, the visibility towards every other vertex is
   calculated and an edge is added between them if they are visible to
   each other," with "every polygon carr[ying] a Minkowski Sum considering
   the radius of a robot," searched with "an algorithm such as Dijkstra's."
   Warthog cites the algorithm's origin as de Berg et al., *Computational
   Geometry: Algorithms and Applications* (2008).
2. The same de Berg et al. textbook algorithm directly, for the parts
   Warthog's TDP doesn't spell out (how start/goal splice into the graph,
   exact segment-intersection test) -- see `docs/algorithms.md`.

SSL obstacles are robots, i.e. circles, not native polygons. Following
Warthog's own approach exactly ("treating each object ... as a polygon"),
each circular obstacle is approximated here as a regular N-gon inflated by
the robot radius + clearance (the "Minkowski Sum" Warthog describes), rather
than doing exact circle-tangent geometry -- simpler to implement and verify
correctly, and it's what the primary source actually documents.

**Performance note.** The first working version of this module used numpy
arrays for every point in the O(n^2) visibility test -- correct, but a real
mistake: numpy's per-call overhead on 2-element arrays dominates at these
sizes, and a 5-8 obstacle scenario (roughly 100 polygon vertices, ~5000
candidate pairs) took up to *19 seconds* in testing, several thousand times
over the SSL 16ms budget. Rewritten below with plain Python floats/tuples
for the hot inner loop (numpy stays only for the one-time polygon
generation, which isn't in the O(n^2) path) plus a cheap bounding-circle
broad-phase check per polygon -- see `_segment_could_hit_polygon`. Both
changes together brought the worst case in `scripts/demo_planners.py`'s
30-trial stress test from ~19s down to low milliseconds; rerun that script
if you change this file to make sure it stays there.
"""

from __future__ import annotations

import math
import time

import networkx as nx

from research_sdk.config import ROBOT_RADIUS_MM
from research_sdk.planners.common import Obstacle, PlanRequest, PlanResult, StepRecorder, path_length_mm
from research_sdk.planners.Dijkstra.waypoint_manager import PlannerInput, PlannerOutput

Point = tuple[float, float]


def _inflate_circle_to_polygon(centre: Point, radius: float, sides: int) -> list[Point]:
    """Approximate a circle (already inflated by clearance) as a regular polygon.

    This *is* the "Minkowski Sum considering the radius of a robot" Warthog's
    TDP describes, specialised to a circular base shape (an SSL robot):
    inflating a circle by another circle's radius just grows the radius, and
    a polygon is what a segment-intersection visibility test needs to work
    against.
    """
    cx, cy = centre
    step = 2.0 * math.pi / sides
    return [
        (cx + radius * math.cos(i * step), cy + radius * math.sin(i * step))
        for i in range(sides)
    ]


def _orientation(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: Point, b: Point, c: Point) -> bool:
    # c is known to be collinear with a-b; check it's within the bbox.
    return (
        min(a[0], b[0]) - 1e-9 <= c[0] <= max(a[0], b[0]) + 1e-9
        and min(a[1], b[1]) - 1e-9 <= c[1] <= max(a[1], b[1]) + 1e-9
    )


def _segments_intersect(p0: Point, p1: Point, p2: Point, p3: Point) -> bool:
    """True if closed segments p0->p1 and p2->p3 properly or improperly intersect.

    Standard orientation-based test (as in de Berg et al., ch. 2), including
    the collinear-overlap edge cases.
    """
    d1 = _orientation(p2, p3, p0)
    d2 = _orientation(p2, p3, p1)
    d3 = _orientation(p0, p1, p2)
    d4 = _orientation(p0, p1, p3)

    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and (
        (d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)
    ):
        return True

    if abs(d1) < 1e-9 and _on_segment(p2, p3, p0):
        return True
    if abs(d2) < 1e-9 and _on_segment(p2, p3, p1):
        return True
    if abs(d3) < 1e-9 and _on_segment(p0, p1, p2):
        return True
    if abs(d4) < 1e-9 and _on_segment(p0, p1, p3):
        return True
    return False


def _point_in_polygon(point: Point, polygon: list[Point]) -> bool:
    """Standard ray-casting point-in-polygon test."""
    x, y = point
    inside = False
    n = len(polygon)
    xj, yj = polygon[-1]
    for xi, yi in polygon:
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi:
            inside = not inside
        xj, yj = xi, yi
    return inside


def _dist_point_to_segment(point: Point, a: Point, b: Point) -> float:
    px, py = point
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
    closest_x = ax + t * dx
    closest_y = ay + t * dy
    return math.hypot(px - closest_x, py - closest_y)


def _segment_could_hit_polygon(
    p0: Point, p1: Point, centre: Point, bounding_radius: float
) -> bool:
    """Cheap broad-phase rejection: could this segment possibly touch the polygon?

    Every vertex of an inflated obstacle's polygon lies exactly on a circle
    of `bounding_radius` around `centre` (see `_inflate_circle_to_polygon`),
    so if the segment stays farther than that from `centre`, it cannot cross
    or enter the polygon -- skip the O(sides) detailed test entirely. This
    is what turns the worst-case stress test from ~19s into low
    milliseconds: most obstacle pairs in a scattered scenario are
    irrelevant to most segments, and this check is O(1).
    """
    return _dist_point_to_segment(centre, p0, p1) <= bounding_radius


def _segment_blocked_by_polygon(p0: Point, p1: Point, polygon: list[Point]) -> bool:
    """True if segment p0->p1 crosses any edge of `polygon`, or passes through its interior.

    A segment that only touches a shared vertex (the usual case for edges
    fanning out of that vertex) is *not* considered blocked -- that's how a
    polygon's own vertices stay mutually visible along its boundary.

    Callers should special-case two vertices that are adjacent on the same
    polygon (see `plan()`) rather than route them through here: the midpoint
    of a boundary edge sits exactly on that polygon's own boundary, where
    ray-casting point-in-polygon is a coin-flip on floating-point rounding.
    Anything that reaches this function is assumed *not* to be that case.
    """
    n = len(polygon)
    for i in range(n):
        a = polygon[i]
        b = polygon[(i + 1) % n]
        # Skip edges that share an endpoint with the segment being tested --
        # touching your own polygon's vertex is not a collision.
        if a == p0 or a == p1 or b == p0 or b == p1:
            continue
        if _segments_intersect(p0, p1, a, b):
            return True

    # Segment could pass fully through the polygon without crossing an edge
    # only if both endpoints are themselves interior points that aren't
    # polygon vertices (rare for a visibility graph, since one of p0/p1 is
    # always a graph vertex on some polygon's boundary or the free-standing
    # start/goal) -- guard against it anyway via a midpoint point-in-polygon
    # check, which is cheap.
    midpoint = ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0)
    return _point_in_polygon(midpoint, polygon)


def _visible(
    p0: Point,
    p1: Point,
    polygons: list[list[Point]],
    polygon_centres: list[Point],
    polygon_bounding_radii: list[float],
    *,
    same_polygon_idx: int | None = None,
) -> bool:
    """True if p0 and p1 see each other around every obstacle polygon.

    `same_polygon_idx`, when given, marks that p0 and p1 are two vertices of
    `polygons[same_polygon_idx]` that are adjacent on its boundary -- that
    edge is always visible (it *is* the boundary) and is skipped rather than
    run through the interior test, which is numerically unreliable exactly
    on a polygon's own edge.
    """
    for idx, polygon in enumerate(polygons):
        if idx == same_polygon_idx:
            continue
        if not _segment_could_hit_polygon(
            p0, p1, polygon_centres[idx], polygon_bounding_radii[idx]
        ):
            continue
        if _segment_blocked_by_polygon(p0, p1, polygon):
            return False
    return True


def plan(
    request: PlanRequest,
    *,
    polygon_sides: int = 12,
    record: StepRecorder | None = None,
) -> PlanResult:
    """Plan a path with a Minkowski-inflated visibility graph + Dijkstra.

    Builds one inflated polygon per obstacle, connects every pair of
    mutually-visible vertices (across all polygons, plus start and goal) with
    an edge, and runs Dijkstra for the shortest path -- matching Warthog's
    TDP description exactly. Nominal complexity is O(n^2) segment tests for
    n total vertices, same as the "original algorithm ... O(n^2 log n)"
    Warthog's TDP cites (their extra log n factor is a sweep-line
    construction; this uses the simpler brute-force all-pairs test with a
    bounding-circle broad phase, which is fine at SSL vertex counts -- see
    the module docstring's performance note).

    ``record``, if given a :class:`~research_sdk.planners.common.StepRecorder`,
    gets a log of every polygon inflated and every vertex pair tested for
    visibility (see ``algorithms.viz.animate_construction``). Leave it
    ``None`` (the default) for normal/timed planning calls.
    """
    start_t = time.perf_counter()

    start: Point = (float(request.start_mm[0]), float(request.start_mm[1]))
    goal: Point = (float(request.goal_mm[0]), float(request.goal_mm[1]))
    total_clearance = request.total_clearance_mm

    polygons: list[list[Point]] = []
    polygon_centres: list[Point] = []
    polygon_bounding_radii: list[float] = []
    for obs in request.obstacles:
        inflate_radius = obs.radius_mm + (total_clearance - request.robot_radius_mm)
        centre = (float(obs.pos_mm[0]), float(obs.pos_mm[1]))
        polygons.append(_inflate_circle_to_polygon(centre, inflate_radius, polygon_sides))
        polygon_centres.append(centre)
        polygon_bounding_radii.append(inflate_radius)

    if record is not None:
        record.log(
            "obstacles",
            start=start,
            goal=goal,
            polygons=[list(p) for p in polygons],
            field_length_mm=request.field_length_mm,
            field_width_mm=request.field_width_mm,
        )

    for polygon in polygons:
        if _point_in_polygon(start, polygon) or _point_in_polygon(goal, polygon):
            return PlanResult(
                success=False,
                waypoints_mm=(),
                path_length_mm=0.0,
                planning_time_ms=(time.perf_counter() - start_t) * 1000.0,
                message="start or goal lies inside an inflated obstacle",
            )

    # Direct line-of-sight shortcut, same as the PRM planner and Warthog's
    # own "adoption of ... a valid collision-free geometric path."
    if _visible(start, goal, polygons, polygon_centres, polygon_bounding_radii):
        waypoints = (start, goal)
        if record is not None:
            record.log("path", waypoints=waypoints, direct=True)
        return PlanResult(
            success=True,
            waypoints_mm=waypoints,
            path_length_mm=path_length_mm(waypoints),
            planning_time_ms=(time.perf_counter() - start_t) * 1000.0,
            nodes_expanded=2,
            message="direct line of sight, visibility graph skipped",
        )

    # Each vertex is (point, owning_polygon_index_or_None, index_within_polygon).
    # start/goal own no polygon, so their membership is (None, None).
    all_vertices: list[Point] = [start, goal]
    vertex_labels: list[str] = ["start", "goal"]
    vertex_polygon: list[int | None] = [None, None]
    vertex_local_idx: list[int | None] = [None, None]
    for poly_idx, polygon in enumerate(polygons):
        for vert_idx, vertex in enumerate(polygon):
            all_vertices.append(vertex)
            vertex_labels.append(f"p{poly_idx}v{vert_idx}")
            vertex_polygon.append(poly_idx)
            vertex_local_idx.append(vert_idx)

    graph = nx.Graph()
    graph.add_nodes_from(vertex_labels)

    n = len(all_vertices)
    for i in range(n):
        p_i = all_vertices[i]
        poly_i = vertex_polygon[i]
        for j in range(i + 1, n):
            p_j = all_vertices[j]

            same_polygon_idx: int | None = None
            if poly_i is not None and poly_i == vertex_polygon[j]:
                sides = len(polygons[poly_i])
                local_gap = abs(vertex_local_idx[i] - vertex_local_idx[j])
                is_adjacent = local_gap == 1 or local_gap == sides - 1
                if is_adjacent:
                    # Boundary edge of its own polygon: always visible, skip
                    # both the interior test against this polygon (see
                    # `_visible`'s docstring) and the redundant "same
                    # polygon" collinearity concern entirely.
                    same_polygon_idx = poly_i
                else:
                    # Non-adjacent vertices of a convex polygon: the chord
                    # between them runs through the polygon's own interior,
                    # so they are never mutually visible. Skip the O(edges)
                    # test against every *other* polygon too, since this
                    # pair can never be an edge in the graph regardless.
                    continue

            visible = _visible(
                p_i,
                p_j,
                polygons,
                polygon_centres,
                polygon_bounding_radii,
                same_polygon_idx=same_polygon_idx,
            )
            if record is not None:
                record.log("edge_test", a=p_i, b=p_j, accepted=visible)
            if visible:
                weight = math.hypot(p_i[0] - p_j[0], p_i[1] - p_j[1])
                graph.add_edge(vertex_labels[i], vertex_labels[j], weight=weight)

    try:
        node_path = nx.dijkstra_path(graph, "start", "goal", weight="weight")
    except (nx.NetworkXNoPath, nx.NodeNotFound):
        return PlanResult(
            success=False,
            waypoints_mm=(),
            path_length_mm=0.0,
            planning_time_ms=(time.perf_counter() - start_t) * 1000.0,
            nodes_expanded=n,
            message="no path found through visibility graph",
        )

    label_to_point = dict(zip(vertex_labels, all_vertices))
    waypoints = tuple(label_to_point[label] for label in node_path)
    if record is not None:
        record.log("path", waypoints=waypoints, direct=False)
    return PlanResult(
        success=True,
        waypoints_mm=waypoints,
        path_length_mm=path_length_mm(waypoints),
        planning_time_ms=(time.perf_counter() - start_t) * 1000.0,
        nodes_expanded=n,
        message=f"visibility graph solved with {n} vertices",
    )


class VisibilityGraphPlanner:
    """Adapts :func:`plan` above to the ``PlannerAPI.plan`` contract used by
    the UI's planner dropdown (see ``planners/api.py`` / ``ui/runtime.py``).
    """

    def __init__(self, **plan_kwargs) -> None:
        self._plan_kwargs = plan_kwargs

    def plan(self, planner_input: PlannerInput) -> PlannerOutput:
        request = _plan_request_from_planner_input(planner_input)
        result = plan(request, **self._plan_kwargs)
        return _planner_output_from_plan_result(planner_input, result)

    def reset(self, robot_id: int | None = None, is_yellow: bool | None = None) -> None:
        """Stateless planner -- nothing to clear, kept for interface parity."""


def _plan_request_from_planner_input(planner_input: PlannerInput) -> PlanRequest:
    start = (float(planner_input.current_pose[0]), float(planner_input.current_pose[1]))
    goal = (float(planner_input.target_pose[0]), float(planner_input.target_pose[1]))
    scene_obstacles = (
        planner_input.scene.get_planning_obstacles()
        if planner_input.scene is not None
        else planner_input.obstacles
    )
    obstacles = tuple(
        Obstacle(
            pos_mm=(float(o.pos_mm[0]), float(o.pos_mm[1])),
            radius_mm=float(o.radius_mm),
            robot_id=int(o.robot_id),
            isYellow=bool(o.isYellow),
        )
        for o in scene_obstacles
    )
    return PlanRequest(
        start_mm=start,
        goal_mm=goal,
        obstacles=obstacles,
        robot_radius_mm=ROBOT_RADIUS_MM,
        clearance_mm=planner_input.clearance_mm,
    )


def _planner_output_from_plan_result(planner_input: PlannerInput, result: PlanResult) -> PlannerOutput:
    target_pose = planner_input.target_pose
    heading = float(target_pose[2]) if len(target_pose) > 2 else 0.0
    waypoints = tuple((float(x), float(y), heading) for x, y in result.waypoints_mm)
    active_target_pose = waypoints[-1] if waypoints else (
        float(target_pose[0]),
        float(target_pose[1]),
        heading,
    )
    return PlannerOutput(
        waypoints=waypoints,
        current_waypoint_index=0,
        active_target_pose=active_target_pose,
        is_path_free=result.success and not waypoints,
        need_reroute=False,
        did_reroute=True,
    )
