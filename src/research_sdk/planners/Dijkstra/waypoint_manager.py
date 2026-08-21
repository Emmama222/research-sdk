"""Waypoint-manager adapter for the Skill Intent Executor planner contract."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Iterable

from research_sdk.planners.Dijkstra.voronoi_dijkstra import (
    PlannerState,
    VoronoiDijkstraPlanner,
)
from research_sdk.config import (
    FIELD_X_MIN,
    FIELD_X_MAX,
    FIELD_Y_MIN,
    FIELD_Y_MAX,
    ROBOT_RADIUS_MM,
    VORONOI_BOUNDARY_INSET_MM,
    VORONOI_DENSITY_PERCENT,
    VORONOI_ENDPOINT_REACH_MM,
    VORONOI_HORIZON_MS,
    VORONOI_MAX_DENSITY_NODES,
    VORONOI_OBSTACLE_COST_WEIGHT,
    VORONOI_TARGET_DEAD_ZONE_MM,
)
from research_sdk.world.map.geometry import distance_2_segment
from research_sdk.world.map.voronoi.voronoi_generator import VoronoiObstacle
from research_sdk.world.scene import PlanningScene


Point2D = tuple[float, float]
Pose2D = tuple[float, float, float]
RobotKey = tuple[bool, int]


@dataclass(frozen=True, slots=True)
class PlannerInput:
    robot_id: int
    is_yellow: bool
    current_pose: Pose2D | Point2D
    target_pose: Pose2D | Point2D
    obstacles: tuple[object, ...] | list[object] = ()
    clearance_mm: float = 0.0
    robot_reached_current_waypoint: bool = False
    reroute_target_deadzone_mm: int = int(VORONOI_TARGET_DEAD_ZONE_MM)
    ignore_obstacles_containing_target: bool = False
    ignored_obstacle_keys_containing_target: tuple[RobotKey, ...] = ()
    endpoint_reach_mm: float = VORONOI_ENDPOINT_REACH_MM
    scene: PlanningScene | None = None
    now_s: float | None = None
    stay_in_field: bool = True


@dataclass(frozen=True, slots=True)
class PlannerOutput:
    waypoints: tuple[Pose2D, ...]
    current_waypoint_index: int
    active_target_pose: Pose2D
    is_path_free: bool
    need_reroute: bool
    did_reroute: bool
    endpoint_was_adjusted: bool = False
    endpoint_precision_mode: bool = False


@dataclass(frozen=True, slots=True)
class TargetClearanceStatus:
    target: Point2D
    in_safety_clearance: bool
    in_reach_clearance: bool
    nearest_obstacle_key: RobotKey | None = None
    nearest_obstacle_distance_mm: float | None = None
    safety_clearance_radius_mm: float | None = None
    reach_clearance_radius_mm: float | None = None
    safety_clearance_overlap_mm: float = 0.0
    reach_clearance_overlap_mm: float = 0.0


@dataclass(slots=True)
class _WaypointState:
    last_target_pose: Pose2D | None = None
    waypoints: tuple[Pose2D, ...] = ()
    current_waypoint_index: int = 0


@dataclass(frozen=True, slots=True)
class _EndpointResolution:
    target: Point2D
    was_adjusted: bool = False
    precision_mode: bool = False


class VoronoiWaypointManager:
    """Stateful waypoint manager matching the task PDF planner API."""

    def __init__(
        self,
        *,
        horizon_ms: int | float = VORONOI_HORIZON_MS,
        density_percent: float = VORONOI_DENSITY_PERCENT,
        max_density_nodes: int = VORONOI_MAX_DENSITY_NODES,
        obstacle_cost_weight: float = VORONOI_OBSTACLE_COST_WEIGHT,
        boundary_inset_mm: float = VORONOI_BOUNDARY_INSET_MM,
    ) -> None:
        self.horizon_ms = horizon_ms
        self.density_percent = density_percent
        self.max_density_nodes = max_density_nodes
        self.obstacle_cost_weight = obstacle_cost_weight
        self.boundary_inset_mm = boundary_inset_mm
        self._state_by_robot: dict[RobotKey, _WaypointState] = {}

    def reset(self, robot_id: int | None = None, is_yellow: bool | None = None) -> None:
        """Clear one robot's waypoint state, or all state when no robot is given."""
        if robot_id is None or is_yellow is None:
            self._state_by_robot.clear()
            return
        self._state_by_robot.pop((bool(is_yellow), int(robot_id)), None)

    def update(self, planner_input: PlannerInput) -> PlannerOutput:
        """Return the active waypoint target for this control tick."""
        robot_key = (bool(planner_input.is_yellow), int(planner_input.robot_id))
        state = self._state_by_robot.setdefault(robot_key, _WaypointState())

        if planner_input.robot_reached_current_waypoint:
            state.waypoints = state.waypoints[1:]
            state.current_waypoint_index = 0

        start = _pose_xy(planner_input.current_pose)
        requested_target_pose = _pose3(planner_input.target_pose)
        raw_target = _pose_xy(requested_target_pose)
        if planner_input.stay_in_field:
            x_min, x_max, y_min, y_max = float(FIELD_X_MIN), float(FIELD_X_MAX), float(FIELD_Y_MIN), float(FIELD_Y_MAX)
            m = VORONOI_TARGET_DEAD_ZONE_MM
            target = (
                max(x_min + m, min(x_max - m, float(raw_target[0]))),
                max(y_min + m, min(y_max - m, float(raw_target[1]))),
            )
        else:
            target = raw_target
        ignore_robots = {robot_key}
        path_map = planner_input.scene or _ObstaclePathMap(
            planner_input.obstacles,
            clearance_mm=planner_input.clearance_mm,
            target_pos=target,
            ignore_obstacles_containing_target=(
                planner_input.ignore_obstacles_containing_target
            ),
            ignored_obstacle_keys_containing_target=(
                planner_input.ignored_obstacle_keys_containing_target
            ),
        )
        endpoint = _resolve_clearance_endpoint(
            path_map,
            start,
            target,
            clearance_mm=planner_input.clearance_mm,
            reach_mm=planner_input.endpoint_reach_mm,
            ignore_robots=ignore_robots,
            horizon_ms=self.horizon_ms,
            now_s=planner_input.now_s,
        )
        target = endpoint.target
        target_pose = _with_heading(target, requested_target_pose[2])
        active_waypoint = self._active_waypoint(state)
        active_target = _pose_xy(active_waypoint) if active_waypoint else target

        is_path_free = path_map.is_path_free(
            start,
            target,
            ignore_robots=ignore_robots,
            clearance=planner_input.clearance_mm,
            horizon_ms=self.horizon_ms,
        )
        target_moved = (
            state.last_target_pose is not None
            and _distance(_pose_xy(state.last_target_pose), target)
            > planner_input.reroute_target_deadzone_mm
        )
        active_route_blocked = (
            active_waypoint is not None
            and not path_map.is_path_free(
                start,
                active_target,
                ignore_robots=ignore_robots,
                clearance=planner_input.clearance_mm,
                horizon_ms=self.horizon_ms,
            )
        )
        route_finished = not state.waypoints
        need_reroute = (
            (not is_path_free)
            and (target_moved or route_finished or active_route_blocked)
        )

        did_reroute = False
        if is_path_free:
            state.waypoints = ()
            state.current_waypoint_index = 0
            state.last_target_pose = target_pose
            return PlannerOutput(
                waypoints=(),
                current_waypoint_index=0,
                active_target_pose=target_pose,
                is_path_free=True,
                need_reroute=False,
                did_reroute=False,
                endpoint_was_adjusted=endpoint.was_adjusted,
                endpoint_precision_mode=endpoint.precision_mode,
            )

        if need_reroute:
            previous = PlannerState(
                last_target_mm=_pose_xy(state.last_target_pose)
                if state.last_target_pose is not None
                else None,
                waypoints_mm=tuple(_pose_xy(point) for point in state.waypoints),
            )
            planner = VoronoiDijkstraPlanner(
                target_dead_zone_mm=planner_input.reroute_target_deadzone_mm,
                horizon_ms=self.horizon_ms,
                density_percent=self.density_percent,
                max_density_nodes=self.max_density_nodes,
                obstacle_cost_weight=self.obstacle_cost_weight,
                boundary_inset_mm=self.boundary_inset_mm,
            )
            result = planner.plan(
                path_map,
                start,
                target,
                now_s=planner_input.now_s,
                ignore_robots=ignore_robots,
                previous_state=previous,
                stay_in_field=planner_input.stay_in_field,
            )
            state.waypoints = tuple(
                _with_heading(point, target_pose[2])
                for point in result.waypoints_mm
            )
            state.current_waypoint_index = 0
            state.last_target_pose = target_pose
            did_reroute = not result.reused_previous

        active = self._active_waypoint(state) or target_pose
        return PlannerOutput(
            waypoints=state.waypoints,
            current_waypoint_index=state.current_waypoint_index,
            active_target_pose=active,
            is_path_free=False,
            need_reroute=need_reroute,
            did_reroute=did_reroute,
            endpoint_was_adjusted=endpoint.was_adjusted,
            endpoint_precision_mode=endpoint.precision_mode,
        )

    def _active_waypoint(self, state: _WaypointState) -> Pose2D | None:
        if state.waypoints:
            return state.waypoints[0]
        return None


class _ObstaclePathMap:
    """Small WorldMap-compatible view over explicit planner obstacles."""

    def __init__(
        self,
        obstacles: Iterable[object],
        *,
        clearance_mm: float,
        target_pos: Point2D | None = None,
        ignore_obstacles_containing_target: bool = False,
        ignored_obstacle_keys_containing_target: Iterable[RobotKey] = (),
    ) -> None:
        self._obstacles = tuple(obstacles)
        self._target_pos = target_pos
        self._ignore_obstacles_containing_target = (
            bool(ignore_obstacles_containing_target)
        )
        self._ignored_obstacle_keys_containing_target = {
            (bool(key[0]), int(key[1]))
            for key in ignored_obstacle_keys_containing_target
        }

    def get_planning_obstacles(
        self,
        now_s=None,
        horizon_ms=None,
        ignore_robots: set[RobotKey] | None = None,
    ) -> tuple[object, ...]:
        if ignore_robots is None:
            ignore_robots = set()
        return tuple(
            _planning_obstacle(obstacle)
            for obstacle in self._obstacles
            if (
                _obstacle_key(obstacle) not in ignore_robots
                and not self._contains_ignored_target(obstacle)
            )
        )

    def is_path_free(
        self,
        start_pos: Point2D,
        end_pos: Point2D,
        ignore_robots: set[RobotKey] | None = None,
        clearance: float = 0.0,
        horizon_ms: int | float | None = None,
    ) -> bool:
        if ignore_robots is None:
            ignore_robots = set()
        for obstacle in self._obstacles:
            if _obstacle_key(obstacle) in ignore_robots:
                continue
            if self._contains_ignored_target(obstacle):
                continue
            if hasattr(obstacle, "intersects_path"):
                if obstacle.intersects_path(
                    start_pos_mm=start_pos,
                    end_pos_mm=end_pos,
                    extra_clearance_mm=clearance,
                    horizon_ms=horizon_ms or 0,
                ):
                    return False
                continue

            pos = _obstacle_pos(obstacle)
            radius = _obstacle_radius(obstacle)
            if (
                distance_2_segment(pos, start_pos, end_pos)
                <= radius + ROBOT_RADIUS_MM + clearance
            ):
                return False
        return True

    def _contains_ignored_target(self, obstacle: object) -> bool:
        if (
            not self._ignore_obstacles_containing_target
            and not self._ignored_obstacle_keys_containing_target
            or self._target_pos is None
        ):
            return False
        key = _obstacle_key(obstacle)
        if (
            self._ignored_obstacle_keys_containing_target
            and key not in self._ignored_obstacle_keys_containing_target
        ):
            return False
        pos = _obstacle_pos(obstacle)
        return _distance(pos, self._target_pos) <= _obstacle_radius(obstacle)


def _obstacle_key(obstacle: object) -> RobotKey | None:
    if hasattr(obstacle, "isYellow") and hasattr(obstacle, "robot_id"):
        return (bool(getattr(obstacle, "isYellow")), int(getattr(obstacle, "robot_id")))
    if hasattr(obstacle, "is_yellow") and hasattr(obstacle, "robot_id"):
        return (bool(getattr(obstacle, "is_yellow")), int(getattr(obstacle, "robot_id")))
    return None


def _obstacle_pos(obstacle: object) -> Point2D:
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


def _obstacle_physical_radius(obstacle: object) -> float:
    if hasattr(obstacle, "radius"):
        return float(getattr(obstacle, "radius"))
    if isinstance(obstacle, (tuple, list)) and len(obstacle) >= 3:
        return float(obstacle[2])
    return _obstacle_radius(obstacle)


def _planning_obstacle(obstacle: object) -> object:
    if hasattr(obstacle, "pos_mm"):
        return obstacle
    return VoronoiObstacle(
        pos_mm=_obstacle_pos(obstacle),
        radius_mm=_obstacle_radius(obstacle),
    )


def check_target_clearance(
    target_pose: Pose2D | Point2D,
    obstacles: Iterable[object],
    *,
    clearance_mm: float = 0.0,
    endpoint_reach_mm: float = VORONOI_ENDPOINT_REACH_MM,
    ignore_robots: Iterable[RobotKey] = (),
) -> TargetClearanceStatus:
    """Classify whether a target is inside obstacle clearance envelopes."""
    target = clamp_to_field(_pose_xy(target_pose))
    ignored_keys = {(bool(key[0]), int(key[1])) for key in ignore_robots}
    nearest = None

    for obstacle in obstacles:
        if _obstacle_key(obstacle) in ignored_keys:
            continue
        planning_obstacle = _planning_obstacle(obstacle)
        pos = _obstacle_pos(planning_obstacle)
        distance = _distance(target, pos)
        safety_radius = _safety_clearance_radius(
            planning_obstacle,
            clearance_mm,
        )
        reach_radius = _inflated_obstacle_radius(
            planning_obstacle,
            clearance_mm,
            endpoint_reach_mm,
        )
        safety_overlap = max(0.0, safety_radius - distance)
        reach_overlap = max(0.0, reach_radius - distance)
        sort_key = (safety_overlap, reach_overlap, -distance)
        if nearest is None or sort_key > nearest[0]:
            nearest = (
                sort_key,
                planning_obstacle,
                distance,
                safety_radius,
                reach_radius,
                safety_overlap,
                reach_overlap,
            )

    if nearest is None:
        return TargetClearanceStatus(
            target=target,
            in_safety_clearance=False,
            in_reach_clearance=False,
        )

    _, obstacle, distance, safety_radius, reach_radius, safety_overlap, reach_overlap = nearest
    return TargetClearanceStatus(
        target=target,
        in_safety_clearance=safety_overlap > 0.0,
        in_reach_clearance=reach_overlap > 0.0,
        nearest_obstacle_key=_obstacle_key(obstacle),
        nearest_obstacle_distance_mm=distance,
        safety_clearance_radius_mm=safety_radius,
        reach_clearance_radius_mm=reach_radius,
        safety_clearance_overlap_mm=safety_overlap,
        reach_clearance_overlap_mm=reach_overlap,
    )


def _resolve_clearance_endpoint(
    path_map: object,
    start: Point2D,
    target: Point2D,
    *,
    clearance_mm: float,
    reach_mm: float,
    ignore_robots: set[RobotKey],
    horizon_ms: int | float,
    now_s: float | None,
) -> _EndpointResolution:
    try:
        obstacles = path_map.get_planning_obstacles(
            now_s=now_s,
            horizon_ms=horizon_ms,
            ignore_robots=ignore_robots,
        )
    except TypeError:
        try:
            obstacles = path_map.get_planning_obstacles(
                horizon_ms=horizon_ms,
                ignore_robots=ignore_robots,
            )
        except Exception:
            return _EndpointResolution(target=target)
    except Exception:
        return _EndpointResolution(target=target)

    containing = tuple(
        obstacle
        for obstacle in obstacles
        if _point_inside_inflated_obstacle(
            target,
            obstacle,
            clearance_mm=clearance_mm,
            reach_mm=reach_mm,
        )
    )
    if not containing:
        return _EndpointResolution(target=target)

    adjusted = target
    for obstacle in sorted(
        containing,
        key=lambda item: _inflated_obstacle_radius(item, clearance_mm, reach_mm)
        - _distance(_obstacle_pos(item), target),
        reverse=True,
    ):
        adjusted = _offset_to_inflated_circle(
            obstacle,
            adjusted,
            start,
            clearance_mm=clearance_mm,
            reach_mm=reach_mm,
        )

    precision_mode = any(
        _point_inside_inflated_obstacle(
            adjusted,
            obstacle,
            clearance_mm=clearance_mm,
            reach_mm=reach_mm,
        )
        for obstacle in obstacles
    ) or adjusted != target
    return _EndpointResolution(
        target=adjusted,
        was_adjusted=True,
        precision_mode=precision_mode,
    )


def _offset_to_inflated_circle(
    obstacle: object,
    target: Point2D,
    start: Point2D,
    *,
    clearance_mm: float,
    reach_mm: float,
) -> Point2D:
    pos = _obstacle_pos(obstacle)
    dx = target[0] - pos[0]
    dy = target[1] - pos[1]
    dist = hypot(dx, dy)
    if dist <= 1e-6:
        dx = start[0] - pos[0]
        dy = start[1] - pos[1]
        dist = hypot(dx, dy)
    if dist <= 1e-6:
        dx, dy, dist = 1.0, 0.0, 1.0

    radius = max(
        _inflated_obstacle_radius(obstacle, clearance_mm, reach_mm),
        _safety_clearance_radius(obstacle, clearance_mm),
    ) + 5.0
    x_min, x_max, y_min, y_max = float(FIELD_X_MIN), float(FIELD_X_MAX), float(FIELD_Y_MIN), float(FIELD_Y_MAX)
    m = VORONOI_TARGET_DEAD_ZONE_MM
    return (
        max(x_min + m, min(x_max - m, pos[0] + (dx / dist) * radius)),
        max(y_min + m, min(y_max - m, pos[1] + (dy / dist) * radius)),
    )


def _point_inside_inflated_obstacle(
    point: Point2D,
    obstacle: object,
    *,
    clearance_mm: float,
    reach_mm: float,
) -> bool:
    return (
        _distance(point, _obstacle_pos(obstacle))
        <= _inflated_obstacle_radius(obstacle, clearance_mm, reach_mm)
    )


def _inflated_obstacle_radius(
    obstacle: object,
    clearance_mm: float,
    reach_mm: float,
) -> float:
    return _obstacle_physical_radius(obstacle) + float(reach_mm) + float(clearance_mm)


def _safety_clearance_radius(obstacle: object, clearance_mm: float) -> float:
    return _obstacle_radius(obstacle) + ROBOT_RADIUS_MM + float(clearance_mm)


def _pose_xy(pose: Pose2D | Point2D) -> Point2D:
    if hasattr(pose, "x") and hasattr(pose, "y"):
        return (float(getattr(pose, "x")), float(getattr(pose, "y")))
    return (float(pose[0]), float(pose[1]))


def _pose3(pose: Pose2D | Point2D) -> Pose2D:
    if hasattr(pose, "x") and hasattr(pose, "y"):
        heading = getattr(pose, "theta", getattr(pose, "heading", 0.0))
        return (float(getattr(pose, "x")), float(getattr(pose, "y")), float(heading))
    heading = float(pose[2]) if len(pose) > 2 else 0.0
    return (float(pose[0]), float(pose[1]), heading)


def _with_heading(point: Point2D, heading: float) -> Pose2D:
    return (float(point[0]), float(point[1]), float(heading))


def clamp_to_field(point: Point2D) -> Point2D:
    x_min, x_max, y_min, y_max = float(FIELD_X_MIN), float(FIELD_X_MAX), float(FIELD_Y_MIN), float(FIELD_Y_MAX)
    return (
        max(x_min, min(x_max, float(point[0]))),
        max(y_min, min(y_max, float(point[1]))),
    )


def _distance(a: Point2D, b: Point2D) -> float:
    return hypot(a[0] - b[0], a[1] - b[1])
