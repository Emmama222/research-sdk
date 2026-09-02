"""Visibility Graph + Dijkstra path planner.

Origin
------
This is a fresh implementation, from two sources:

1. **Warthog Robotics' 2025 SSL TDP**: "treating each object in the field
   as a polygon. For each of its vertices, the visibility towards every
   other vertex is calculated and an edge is added between them if they are
   visible to each other," with "every polygon carr[ying] a Minkowski Sum
   considering the radius of a robot," searched with "an algorithm such as
   Dijkstra's." Warthog cites de Berg et al., *Computational Geometry:
   Algorithms and Applications* (2008) as the algorithm's origin.
2. That same de Berg et al. textbook, for the parts Warthog's TDP doesn't
   spell out (how start/goal splice into the graph) -- see `docs/algorithms.md`.

Why this isn't a plain polygon-edge test
-----------------------------------------
SSL obstacles are robots, i.e. circles. An earlier version of this module
followed Warthog's "treat each object as a polygon" description literally:
inflate each circle into a regular N-gon, then test every candidate edge
against that polygon's *edges*. A straight-edged polygon can't equal a
circle, so that design was always forced to pick one of two flaws:

- **Circumscribed** (safe for edge-blocking, since it never cuts inside the
  circle) -- but its corners reach past the true clearance circle, which
  false-rejected valid start/goal points landing in a corner's gap.
- **Inscribed** (accurate for containment) -- but its edges cut inside the
  true clearance, letting a path graze closer to an obstacle than intended.

No single polygon gets both right, because the obstacle isn't really a
polygon at all -- it's a circle.

The fix: check against the real circle
----------------------------------------
This version drops the polygon-edge test and checks every candidate edge
against the exact circle instead (point/segment-to-centre distance vs. the
true clearance radius, via `_dist_point_to_segment`). That settles both
concerns exactly, with zero approximation error:

- **Containment** ("is this point inside the obstacle?")
- **Edge-blocking** ("does this segment cross the obstacle?")

It's simpler too -- no segment-intersection code, no separate broad-phase
pass, since the exact test is already O(1) per obstacle.

One polygon still remains: waypoint placement
-----------------------------------------------
A regular N-gon is still used, but only to decide *where the candidate
waypoints go* (Warthog's "treat each object as a polygon" idea, applied
just to node placement). It's still circumscribed slightly outside the true
clearance circle -- not for the old reason, but because a hop between two
*adjacent* waypoints of the same obstacle is a chord of that obstacle's
circle, and any chord passes strictly inside its circle except at its own
endpoints. Waypoints placed exactly on the true circle would make every
boundary-hugging hop cut a little inside the real clearance. Circumscribing
them (`radius / cos(pi / sides)`, same formula as before) makes that chord
exactly tangent to the true circle instead, at essentially no extra detour.

Containment checks never look at this polygon -- they compare straight-line
distance to the true clearance radius directly -- so this can't reintroduce
the corner-gap bug that motivated the rewrite.

Performance note
-----------------
The first working version of this module used numpy arrays for every point
in the O(n^2) visibility test -- correct, but a real mistake: numpy's
per-call overhead on 2-element arrays dominates at these sizes, and a 5-8
obstacle scenario (roughly 100 polygon vertices, ~5000 candidate pairs)
took up to *19 seconds* in testing, several thousand times over the SSL
16ms budget. This version keeps the fix: plain Python floats/tuples for the
hot inner loop (numpy stays only for the one-time polygon generation, which
isn't in the O(n^2) path). Rerun `scripts/demo_planners.py` if you change
this file to make sure it stays fast.
"""

from __future__ import annotations

import math
import time

import networkx as nx

from research_sdk.config import ROBOT_RADIUS_MM, VISIBILITY_POLYGON_SIDES
from research_sdk.planners.common import (
    Obstacle,
    PlanRequest,
    PlanResult,
    StepRecorder,
    path_length_mm,
)
from research_sdk.planners.Dijkstra.waypoint_manager import PlannerInput, PlannerOutput

Point = tuple[float, float]

# Waypoints sit exactly on their obstacle's clearance circle (see
# `_circle_waypoints`), so a segment ending at one has its closest approach
# to that circle's centre land exactly on the waypoint itself, at a distance
# equal to the radius up to floating-point noise. Without this tolerance,
# that exact boundary *touch* -- not a real intrusion -- would compare as
# "inside" and self-block every waypoint from ever connecting to anything.
_BOUNDARY_EPS_MM = 1e-6


def _circle_waypoints(centre: Point, clearance_radius: float, sides: int) -> list[Point]:
    """Candidate waypoints spread evenly around an obstacle's clearance circle.

    Circumscribed slightly outside `clearance_radius` (see the module
    docstring) purely so a hop between two adjacent waypoints stays tangent
    to the true clearance circle instead of cutting inside it -- this only
    affects where waypoints sit, never whether a point or segment counts as
    colliding (that's always checked against `clearance_radius` directly).
    """
    if sides < 3:
        raise ValueError("polygon_sides must be at least 3")
    cx, cy = centre
    vertex_radius = clearance_radius / math.cos(math.pi / sides)
    step = 2.0 * math.pi / sides
    return [
        (cx + vertex_radius * math.cos(i * step), cy + vertex_radius * math.sin(i * step))
        for i in range(sides)
    ]


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


def _visible(
    p0: Point,
    p1: Point,
    centres: list[Point],
    radii: list[float],
    *,
    skip_idx: int | None = None,
) -> bool:
    """True if the segment p0->p1 stays outside every obstacle's clearance circle.

    `skip_idx`, when given, marks that p0 and p1 are two waypoints generated
    from the *same* obstacle (`skip_idx`) that are adjacent on its
    circle -- that chord is the obstacle's own boundary and is always
    visible, so its own circle is excluded from the check (its distance to
    that chord equals the clearance radius almost exactly, which is a
    floating-point coin-flip rather than a real collision).
    """
    for idx, (centre, radius) in enumerate(zip(centres, radii)):
        if idx == skip_idx:
            continue
        if _dist_point_to_segment(centre, p0, p1) < radius - _BOUNDARY_EPS_MM:
            return False
    return True


def plan(
    request: PlanRequest,
    *,
    polygon_sides: int = VISIBILITY_POLYGON_SIDES,
    skip_direct_path: bool = False,
    record: StepRecorder | None = None,
) -> PlanResult:
    """Plan a path with a circle-exact visibility graph + Dijkstra.

    Generates `polygon_sides` candidate waypoints spread around each
    obstacle's true clearance circle, connects every pair of
    mutually-visible waypoints (across all obstacles, plus start and goal)
    with an edge -- checked against the exact clearance circle, not a
    polygon approximation of it -- and runs Dijkstra for the shortest path.
    This matches Warthog's TDP description ("treating each object ... as a
    polygon[,] ... visibility towards every other vertex ... calculated")
    for where the graph's nodes come from, while checking collisions
    against the real circle those nodes were generated from (see the module
    docstring for why: no single polygon can both safely bound a circle and
    accurately test containment against it).

    ``record``, if given a :class:`~research_sdk.planners.common.StepRecorder`,
    gets a log of every obstacle's candidate waypoints and every vertex pair
    tested for visibility (see ``algorithms.viz.animate_construction``).
    Leave it ``None`` (the default) for normal/timed planning calls.

    ``skip_direct_path=True`` forces the full visibility-graph build even
    when start and goal see each other directly -- for planner comparisons
    that want every call measuring the algorithm's actual graph-construction
    cost, not the trivial straight-line case. Leave it ``False`` (the
    default) for normal/production planning, where taking the free direct
    path is exactly the right thing to do.
    """
    start_t = time.perf_counter()

    start: Point = (float(request.start_mm[0]), float(request.start_mm[1]))
    goal: Point = (float(request.goal_mm[0]), float(request.goal_mm[1]))
    total_clearance = request.total_clearance_mm

    centres: list[Point] = []
    radii: list[float] = []
    waypoints_by_obstacle: list[list[Point]] = []
    for obs in request.obstacles:
        # total_clearance is robot_radius_mm + clearance_mm (see
        # PlanRequest.total_clearance_mm) -- the Minkowski-sum radius is the
        # obstacle's own physical radius plus that, so both bodies (and the
        # safety margin) stay clear of each other.
        centre = (float(obs.pos_mm[0]), float(obs.pos_mm[1]))
        radius = obs.radius_mm + total_clearance
        centres.append(centre)
        radii.append(radius)
        waypoints_by_obstacle.append(_circle_waypoints(centre, radius, polygon_sides))

    if record is not None:
        record.log(
            "obstacles",
            start=start,
            goal=goal,
            polygons=[list(p) for p in waypoints_by_obstacle],
            field_length_mm=request.field_length_mm,
            field_width_mm=request.field_width_mm,
        )

    for centre, radius in zip(centres, radii):
        dist_start = math.hypot(start[0] - centre[0], start[1] - centre[1])
        dist_goal = math.hypot(goal[0] - centre[0], goal[1] - centre[1])
        if dist_start <= radius or dist_goal <= radius:
            return PlanResult(
                success=False,
                waypoints_mm=(),
                path_length_mm=0.0,
                planning_time_ms=(time.perf_counter() - start_t) * 1000.0,
                message="start or goal lies inside an inflated obstacle",
            )

    # Direct line-of-sight shortcut, same as the PRM planner and Warthog's
    # own "adoption of ... a valid collision-free geometric path."
    if not skip_direct_path and _visible(start, goal, centres, radii):
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

    # Each vertex is (point, owning_obstacle_index_or_None, index_within_obstacle).
    # start/goal own no obstacle, so their membership is (None, None).
    all_vertices: list[Point] = [start, goal]
    vertex_labels: list[str] = ["start", "goal"]
    vertex_obstacle: list[int | None] = [None, None]
    vertex_local_idx: list[int | None] = [None, None]
    for obs_idx, waypoints in enumerate(waypoints_by_obstacle):
        for local_idx, point in enumerate(waypoints):
            all_vertices.append(point)
            vertex_labels.append(f"p{obs_idx}v{local_idx}")
            vertex_obstacle.append(obs_idx)
            vertex_local_idx.append(local_idx)

    graph = nx.Graph()
    graph.add_nodes_from(vertex_labels)

    n = len(all_vertices)
    for i in range(n):
        p_i = all_vertices[i]
        obs_i = vertex_obstacle[i]
        for j in range(i + 1, n):
            p_j = all_vertices[j]

            skip_idx: int | None = None
            if obs_i is not None and obs_i == vertex_obstacle[j]:
                sides = len(waypoints_by_obstacle[obs_i])
                local_gap = abs(vertex_local_idx[i] - vertex_local_idx[j])
                is_adjacent = local_gap == 1 or local_gap == sides - 1
                if is_adjacent:
                    # Chord along its own obstacle's boundary: always
                    # visible with respect to that obstacle, skip it (see
                    # `_visible`'s docstring).
                    skip_idx = obs_i
                else:
                    # Non-adjacent waypoints of the same obstacle: the chord
                    # between two points on a circle always passes through
                    # that circle's own interior, so they can never be
                    # mutually visible around their own obstacle. Skip the
                    # test against every *other* obstacle too, since this
                    # pair can never be an edge in the graph regardless.
                    continue

            visible = _visible(p_i, p_j, centres, radii, skip_idx=skip_idx)
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
