"""Dijkstra path planning over the bounded Voronoi world map."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Iterable

import networkx as nx

from research_sdk.config import (
    DEFENCE_X_MM,
    DEFENCE_Y_MM,
    FIELD_X_MIN,
    FIELD_X_MAX,
    FIELD_Y_MIN,
    FIELD_Y_MAX,
    GOAL_DEPTH_MM,
    GOAL_HALF_WIDTH_MM,
    ROBOT_RADIUS_MM,
    VORONOI_BOUNDARY_INSET_MM,
    VORONOI_CONNECTION_COUNT,
    VORONOI_CONNECTION_RADIUS_MM,
    VORONOI_DENSITY_PERCENT,
    VORONOI_ESCAPE_MARGIN_MM,
    VORONOI_HORIZON_MS,
    VORONOI_MAX_DENSITY_NODES,
    VORONOI_MIN_ESCAPE_STEP_MM,
    VORONOI_OBSTACLE_COST_WEIGHT,
    VORONOI_TARGET_DEAD_ZONE_MM,
)
from research_sdk.planners.common import StepRecorder
from research_sdk.world.map.geometry import distance_2_segment
from research_sdk.world.map.voronoi.voronoi_generator import (
    VoronoiObstacle,
    generate_voronoi_map_from_scene,
)


Point = tuple[float, float]
RobotKey = tuple[bool, int]


@dataclass(slots=True)
class PlannerState:
    """Per-robot reusable path state."""

    last_target_mm: Point | None = None
    waypoints_mm: tuple[Point, ...] = ()
    generated_at_s: float = 0.0


@dataclass(frozen=True, slots=True)
class PlanResult:
    """Serializable result returned through the WorldModel proxy."""

    target_mm: Point
    waypoints_mm: tuple[Point, ...]
    reused_previous: bool = False
    used_direct_path: bool = False


class VoronoiDijkstraPlanner:
    """Plan center-point waypoints while respecting WorldMap path clearance."""

    START_ID = -1
    TARGET_ID = -2

    def __init__(
        self,
        *,
        target_dead_zone_mm: float = VORONOI_TARGET_DEAD_ZONE_MM,
        connection_count: int = VORONOI_CONNECTION_COUNT,
        connection_radius_mm: float = VORONOI_CONNECTION_RADIUS_MM,
        horizon_ms: int | float = VORONOI_HORIZON_MS,
        density_percent: float = VORONOI_DENSITY_PERCENT,
        max_density_nodes: int = VORONOI_MAX_DENSITY_NODES,
        obstacle_cost_weight: float = VORONOI_OBSTACLE_COST_WEIGHT,
        boundary_inset_mm: float = VORONOI_BOUNDARY_INSET_MM,
    ) -> None:
        self.target_dead_zone_mm = float(target_dead_zone_mm)
        self.connection_count = int(connection_count)
        self.connection_radius_mm = float(connection_radius_mm)
        self.horizon_ms = horizon_ms
        self.density_percent = float(density_percent)
        self.max_density_nodes = int(max_density_nodes)
        self.obstacle_cost_weight = float(obstacle_cost_weight)
        self.boundary_inset_mm = float(boundary_inset_mm)

    def plan(
        self,
        scene,
        start_pos_mm: Point,
        target_pos_mm: Point,
        *,
        now_s: float | None = None,
        ignore_robots: set[RobotKey] | None = None,
        previous_state: PlannerState | None = None,
        stay_in_field: bool = True,
        skip_direct_path: bool = False,
        record: StepRecorder | None = None,
    ) -> PlanResult:
        """Return waypoints from *start_pos_mm* toward *target_pos_mm*.

        When *stay_in_field* is True (default) the target is clamped to the
        field and all returned waypoints are validated to stay within it.
        Set to False to allow planning toward out-of-field targets (e.g. the
        ball rolling out of bounds) while still being aware of the goal posts.

        *skip_direct_path*, when True, forces the full Voronoi-map build even
        when start and target see each other directly -- for planner
        comparisons that want every call measuring the algorithm's actual
        map-generation cost, matching ``visibility_graph.plan()`` and
        ``prm_dijkstra.plan()``'s own ``skip_direct_path``. Leave it False
        (the default) for normal/production planning, where taking the free
        direct path is exactly the right thing to do. Does not affect the
        escape-waypoint or reused-previous-path shortcuts below, which are
        different mechanisms from the direct-line-of-sight check.

        *record*, if given a
        :class:`~research_sdk.planners.common.StepRecorder`, gets a coarse
        map-vs-search split: one ``"map"`` event bracketing the Voronoi map
        build (``generate_voronoi_map_from_scene`` plus splicing start/target
        into the graph) and one ``"path"`` event when the Dijkstra search
        finishes. This is coarser than ``visibility_graph.py``/
        ``prm_dijkstra.py``'s per-edge/per-sample logging (those log every
        candidate tested; this logs only phase boundaries), because map
        generation here happens inside a separate module
        (``voronoi_generator.py``) that doesn't accept a recorder -- adding
        two timestamps around the existing call boundaries was enough to
        get a real split without threading ``record`` through that module's
        internals too. Leave it ``None`` (the default) for normal/timed
        planning calls, matching the other two planners.
        """
        if ignore_robots is None:
            ignore_robots = set()
        if previous_state is None:
            previous_state = PlannerState()

        start = _point2(start_pos_mm)
        raw = _point2(target_pos_mm)
        target = clamp_to_field(raw) if stay_in_field else raw

        escape_waypoint = self._escape_waypoint_from_containing_obstacles(
            scene,
            start,
            target,
            now_s=now_s,
            ignore_robots=ignore_robots,
        )
        if escape_waypoint is not None:
            return PlanResult(target_mm=target, waypoints_mm=(escape_waypoint,))

        if not skip_direct_path and scene.is_path_free(
            start,
            target,
            ignore_robots=ignore_robots,
            horizon_ms=self.horizon_ms,
        ):
            return PlanResult(
                target_mm=target,
                waypoints_mm=(),
                used_direct_path=True,
            )

        if self._previous_path_is_valid(
            scene,
            start,
            target,
            previous_state,
            ignore_robots,
        ):
            return PlanResult(
                target_mm=target,
                waypoints_mm=previous_state.waypoints_mm,
                reused_previous=True,
            )

        if record is not None:
            record.log("map_started")
        voronoi_map = generate_voronoi_map_from_scene(
            scene,
            now_s=now_s,
            horizon_ms=self.horizon_ms,
            ignore_robots=ignore_robots,
            density_percent=self.density_percent,
            max_density_nodes=self.max_density_nodes,
            obstacle_cost_weight=self.obstacle_cost_weight,
            boundary_inset_mm=self.boundary_inset_mm,
        )
        node_pos = {node.id: (node.x, node.y) for node in voronoi_map.nodes}
        adjacency: dict[int, list[tuple[int, float]]] = {}
        for edge in voronoi_map.edges:
            adjacency.setdefault(edge.start_id, []).append((edge.end_id, edge.cost))
            adjacency.setdefault(edge.end_id, []).append((edge.start_id, edge.cost))

        obstacles = tuple(voronoi_map.obstacles)
        self._connect_temporary_node(
            adjacency,
            node_pos,
            scene,
            self.START_ID,
            start,
            ignore_robots,
            obstacles,
        )
        self._connect_temporary_node(
            adjacency,
            node_pos,
            scene,
            self.TARGET_ID,
            target,
            ignore_robots,
            obstacles,
        )

        if record is not None:
            record.log("map_built", node_count=len(node_pos))

        if self.START_ID not in adjacency or self.TARGET_ID not in adjacency:
            return PlanResult(target_mm=target, waypoints_mm=())

        ids = self._dijkstra(adjacency, self.START_ID, self.TARGET_ID)
        if not ids:
            return PlanResult(target_mm=target, waypoints_mm=())

        intermediate = tuple(node_pos[nid] for nid in ids[1:-1])

        # Goal-post awareness is always active regardless of stay_in_field.
        # A path crossing through the physical goal structure is never valid.
        full_path = (start, *intermediate, target)
        if any(
            _segment_crosses_goal_zone(full_path[i], full_path[i + 1])
            for i in range(len(full_path) - 1)
        ):
            return PlanResult(target_mm=target, waypoints_mm=())

        # Field-boundary validation only when stay_in_field is set.
        if stay_in_field and any(
            not is_in_field(wp)
            or is_in_goal_zone(wp)
            for wp in intermediate
        ):
            return PlanResult(target_mm=target, waypoints_mm=())

        waypoints = (*intermediate, target)
        if record is not None:
            record.log("path", waypoints=waypoints, direct=False)
        return PlanResult(target_mm=target, waypoints_mm=waypoints)

    def _escape_waypoint_from_containing_obstacles(
        self,
        scene,
        start: Point,
        target: Point,
        *,
        now_s: float | None,
        ignore_robots: set[RobotKey],
    ) -> Point | None:
        try:
            obstacles = scene.get_planning_obstacles(
                now_s=now_s,
                horizon_ms=self.horizon_ms,
                ignore_robots=ignore_robots,
            )
        except Exception:
            return None

        push_x = 0.0
        push_y = 0.0
        max_overlap = 0.0
        for obstacle in obstacles:
            pos = _obstacle_pos(obstacle)
            radius = _obstacle_radius(obstacle) + ROBOT_RADIUS_MM
            dx = start[0] - pos[0]
            dy = start[1] - pos[1]
            dist = hypot(dx, dy)
            overlap = radius - dist
            if overlap < 0.0:
                continue
            if dist <= 1e-6:
                dx = target[0] - pos[0]
                dy = target[1] - pos[1]
                dist = hypot(dx, dy)
            if dist <= 1e-6:
                dx, dy, dist = 1.0, 0.0, 1.0
            weight = max(overlap, 1.0)
            push_x += (dx / dist) * weight
            push_y += (dy / dist) * weight
            max_overlap = max(max_overlap, overlap)

        push_len = hypot(push_x, push_y)
        if push_len <= 1e-6:
            return None

        step_mm = max(
            VORONOI_MIN_ESCAPE_STEP_MM,
            max_overlap + VORONOI_ESCAPE_MARGIN_MM,
        )
        x_min, x_max, y_min, y_max = float(FIELD_X_MIN), float(FIELD_X_MAX), float(FIELD_Y_MIN), float(FIELD_Y_MAX)
        m = VORONOI_TARGET_DEAD_ZONE_MM
        return (
            max(x_min + m, min(x_max - m,
                start[0] + (push_x / push_len) * step_mm)),
            max(y_min + m, min(y_max - m,
                start[1] + (push_y / push_len) * step_mm)),
        )

    def _previous_path_is_valid(
        self,
        scene,
        start: Point,
        target: Point,
        previous_state: PlannerState,
        ignore_robots: set[RobotKey],
    ) -> bool:
        if previous_state.last_target_mm is None or not previous_state.waypoints_mm:
            return False
        if _distance(previous_state.last_target_mm, target) > self.target_dead_zone_mm:
            return False

        next_waypoint = previous_state.waypoints_mm[0]
        return scene.is_path_free(
            start,
            next_waypoint,
            ignore_robots=ignore_robots,
            horizon_ms=self.horizon_ms,
        )

    def _connect_temporary_node(
        self,
        adjacency: dict[int, list[tuple[int, float]]],
        node_pos: dict[int, Point],
        scene,
        temp_id: int,
        temp_pos: Point,
        ignore_robots: set[RobotKey],
        obstacles: tuple[VoronoiObstacle, ...],
    ) -> None:
        node_pos[temp_id] = temp_pos
        candidates = sorted(
            (
                (_distance(temp_pos, pos), node_id, pos)
                for node_id, pos in node_pos.items()
                if node_id >= 0
            ),
            key=lambda item: item[0],
        )
        connected = 0
        for distance, node_id, pos in candidates:
            if distance > self.connection_radius_mm and connected > 0:
                break
            if not scene.is_path_free(
                temp_pos,
                pos,
                ignore_robots=ignore_robots,
                horizon_ms=self.horizon_ms,
            ):
                continue
            cost = distance * self._obstacle_cost_multiplier(temp_pos, pos, obstacles)
            adjacency.setdefault(temp_id, []).append((node_id, cost))
            adjacency.setdefault(node_id, []).append((temp_id, cost))
            connected += 1
            if connected >= self.connection_count:
                break

    def _obstacle_cost_multiplier(
        self,
        start: Point,
        end: Point,
        obstacles: Iterable[VoronoiObstacle],
    ) -> float:
        if self.obstacle_cost_weight <= 0:
            return 1.0
        risk = 0.0
        influence_mm = max(1.0, self.boundary_inset_mm + 600.0)
        for obstacle in obstacles:
            clearance = (
                distance_2_segment(obstacle.pos_mm, start, end)
                - obstacle.radius_mm
            )
            if clearance <= 0:
                risk += 2.0
            elif clearance < influence_mm:
                risk += (influence_mm - clearance) / influence_mm
        return 1.0 + self.obstacle_cost_weight * risk

    def _dijkstra(
        self,
        adjacency: dict[int, list[tuple[int, float]]],
        start_id: int,
        target_id: int,
    ) -> list[int]:
        """Shortest path over ``adjacency`` via ``networkx.dijkstra_path`` --
        the same search VisibilityGraph and PRM already use (see
        ``planners/VisibilityGraph/visibility_graph.py`` and
        ``planners/PRM/prm_dijkstra.py``), so all three planners are compared
        on identical shortest-path behaviour and only differ in how they
        build the graph/roadmap/map handed to it. Previously a hand-rolled
        heapq Dijkstra -- algorithmically equivalent (same textbook
        algorithm, same non-negative edge weights), but keeping a second,
        bespoke implementation around was an unforced source of doubt when
        comparing path quality across planners, for no behavioural benefit.
        """
        graph = nx.Graph()
        graph.add_nodes_from(adjacency)
        for node_id, edges in adjacency.items():
            for neighbour_id, cost in edges:
                graph.add_edge(node_id, neighbour_id, weight=cost)
        try:
            return nx.dijkstra_path(graph, start_id, target_id, weight="weight")
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            return []


def _point2(point: tuple[float, ...] | list[float]) -> Point:
    return (float(point[0]), float(point[1]))


def _obstacle_pos(obstacle: object) -> Point:
    pos = getattr(obstacle, "pos_mm", obstacle)
    return (float(pos[0]), float(pos[1]))


def _obstacle_radius(obstacle: object) -> float:
    if hasattr(obstacle, "radius_mm"):
        return float(getattr(obstacle, "radius_mm"))
    if hasattr(obstacle, "safe_radius"):
        return float(getattr(obstacle, "safe_radius"))
    if hasattr(obstacle, "radius"):
        return float(getattr(obstacle, "radius"))
    if isinstance(obstacle, (tuple, list)) and len(obstacle) >= 3:
        return float(obstacle[2])
    return 0.0


def is_in_goal_zone(point: Point) -> bool:
    """Return True if *point* is inside either physical goal box.

    The goal box extends GOAL_DEPTH_MM past each end line, centred on the
    field.  It is physically walled on three sides — no path may pass through it.
    """
    x, y = float(point[0]), float(point[1])
    goal_half_width = float(GOAL_HALF_WIDTH_MM)
    if abs(y) > goal_half_width:
        return False
    x_min, x_max = float(FIELD_X_MIN), float(FIELD_X_MAX)
    return x > x_max or x < x_min


def _segment_crosses_goal_zone(p1: Point, p2: Point) -> bool:
    """Return True if the segment p1→p2 enters either physical goal box."""
    x1, y1 = float(p1[0]), float(p1[1])
    x2, y2 = float(p2[0]), float(p2[1])
    goal_half_width = float(GOAL_HALF_WIDTH_MM)
    x_min, x_max = float(FIELD_X_MIN), float(FIELD_X_MAX)
    for wall_x in (x_min, x_max):
        dx = x2 - x1
        if abs(dx) < 1e-9:
            continue
        t = (wall_x - x1) / dx
        if 0.0 <= t <= 1.0:
            cross_y = y1 + t * (y2 - y1)
            if abs(cross_y) <= goal_half_width:
                return True
    return False


def clamp_to_field(point: Point) -> Point:
    """Clamp a point to the full playable field rectangle."""
    x_min, x_max, y_min, y_max = float(FIELD_X_MIN), float(FIELD_X_MAX), float(FIELD_Y_MIN), float(FIELD_Y_MAX)
    return (
        max(x_min, min(x_max, float(point[0]))),
        max(y_min, min(y_max, float(point[1]))),
    )


def is_in_box(
    point: Point,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    margin: float = 0.0,
) -> bool:
    """Return True if *point* lies inside the box inset by *margin* on every side.

    A positive margin shrinks the effective box, so the point must be at least
    *margin* mm clear of each boundary edge.
    """
    x, y = float(point[0]), float(point[1])
    return (
        x_min + margin <= x <= x_max - margin
        and y_min + margin <= y <= y_max - margin
    )


def is_in_field(point: Point, margin: float = VORONOI_TARGET_DEAD_ZONE_MM) -> bool:
    """Return True if *point* is inside the full field, inset by *margin* mm.

    The default uses the planner target dead zone so target clamping and field
    checks share one safety-margin setting. Callers checking whether a waypoint
    is far enough from the boundary can call without arguments.
    Pass margin=0.0 to test the exact field rectangle.
    """
    x_min, x_max, y_min, y_max = float(FIELD_X_MIN), float(FIELD_X_MAX), float(FIELD_Y_MIN), float(FIELD_Y_MAX)
    return is_in_box(point, x_min, x_max, y_min, y_max, margin)


def is_in_penalty_box(
    point: Point,
    *,
    positive_side: bool = True,
    margin: float = 0.0,
) -> bool:
    """Return True if *point* is inside the penalty/defence area on the chosen side.

    *positive_side=True* checks the positive-x goal end; False checks the
    negative-x goal end.  *margin* insets all four edges (use ROBOT_RADIUS_MM
    to test with a robot-body clearance).
    """
    defence_x = float(DEFENCE_X_MM)
    defence_y = float(DEFENCE_Y_MM)
    x_min, x_max = float(FIELD_X_MIN), float(FIELD_X_MAX)
    if positive_side:
        box_x_min = x_max - defence_x
        box_x_max = x_max
    else:
        box_x_min = x_min
        box_x_max = x_min + defence_x
    return is_in_box(point, box_x_min, box_x_max, -defence_y, defence_y, margin)


def _distance(a: Point, b: Point) -> float:
    return hypot(a[0] - b[0], a[1] - b[1])
