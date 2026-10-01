"""Shared cheap-check-first reroute gate used by every planner wrapper.

``VoronoiWaypointManager`` originally computed this inline: before paying for
a full graph rebuild + search, check whether the robot's already-active path
segment is still obstacle-free, and only reroute when it genuinely isn't (or
the target moved, or the route ran out, or a periodic safety-net cadence is
due). ``PRMPlanner``/``VisibilityGraphPlanner`` had no equivalent -- they
rebuilt their whole graph on every single call. This module extracts that
gate so all three planners can share it.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from math import hypot
from typing import Protocol

from research_sdk.config import (
    FIELD_X_MAX,
    FIELD_X_MIN,
    FIELD_Y_MAX,
    FIELD_Y_MIN,
    VORONOI_ESCAPE_MARGIN_MM,
    VORONOI_MIN_ESCAPE_STEP_MM,
    VORONOI_TARGET_DEAD_ZONE_MM,
)

Point2D = tuple[float, float]
Pose2D = tuple[float, float, float]
RobotKey = tuple[bool, int]

DEFAULT_TARGET_DEADZONE_MM = float(VORONOI_TARGET_DEAD_ZONE_MM)
DEFAULT_PERIODIC_REROUTE_FRAMES = 5


class RouteScene(Protocol):
    """Structural shape ``evaluate_route`` needs -- ``PlanningScene`` already satisfies it."""

    def is_path_free(
        self,
        start_pos: Point2D,
        end_pos: Point2D,
        ignore_robots: set[RobotKey] | None = None,
        clearance: float = 0.0,
        horizon_ms: int | float | None = None,
    ) -> bool: ...


@dataclass(slots=True)
class RouteState:
    """Per-robot memory needed to ask "is my current route still good?" cheaply."""

    last_target_pose: Pose2D | None = None
    waypoints: tuple[Pose2D, ...] = ()
    frames_since_reroute: int = 0


@dataclass(frozen=True, slots=True)
class RerouteDecision:
    need_reroute: bool
    is_path_free: bool
    active_target: Point2D


def evaluate_route(
    scene: RouteScene,
    start: Point2D,
    target: Point2D,
    state: RouteState,
    *,
    ignore_robots: set[RobotKey] | None = None,
    clearance_mm: float = 0.0,
    horizon_ms: int | float = 0.0,
    target_deadzone_mm: float = DEFAULT_TARGET_DEADZONE_MM,
    periodic_reroute_frames: int | None = DEFAULT_PERIODIC_REROUTE_FRAMES,
    check_full_route: bool = False,
) -> RerouteDecision:
    """Return whether ``state``'s cached route still clears ``scene``.

    Cheap in the common case -- this only runs distance-to-segment checks via
    ``scene.is_path_free``, never a full graph rebuild. A caller reroutes
    (redoes the expensive part of its own ``plan()``) only when
    ``need_reroute`` comes back ``True``.

    ``check_full_route`` also treats a blocked *later* segment of the cached
    route as an event (by default only the segment to the active waypoint is
    checked). This matters when predicted obstacles sweep across the part of
    the route the robot has not reached yet.
    """
    ignore = ignore_robots or set()
    active_waypoint = state.waypoints[0] if state.waypoints else None
    active_target = _pose_xy(active_waypoint) if active_waypoint is not None else target

    is_free = scene.is_path_free(
        start, target, ignore_robots=ignore, clearance=clearance_mm, horizon_ms=horizon_ms
    )
    target_moved = (
        state.last_target_pose is not None
        and _distance(_pose_xy(state.last_target_pose), target) > target_deadzone_mm
    )
    active_route_blocked = active_waypoint is not None and not scene.is_path_free(
        start, active_target, ignore_robots=ignore, clearance=clearance_mm, horizon_ms=horizon_ms
    )
    if check_full_route and not active_route_blocked and state.waypoints:
        route = [_pose_xy(point) for point in state.waypoints]
        if _distance(route[-1], target) > 1e-6:
            route.append(target)
        active_route_blocked = any(
            not scene.is_path_free(
                first, second, ignore_robots=ignore, clearance=clearance_mm, horizon_ms=horizon_ms
            )
            for first, second in zip(route, route[1:])
        )
    route_finished = not state.waypoints
    periodic_due = (
        periodic_reroute_frames is not None
        and state.frames_since_reroute >= periodic_reroute_frames
    )
    need_reroute = (not is_free) and (
        target_moved or route_finished or active_route_blocked or periodic_due
    )
    return RerouteDecision(need_reroute=need_reroute, is_path_free=is_free, active_target=active_target)


def escape_waypoint(
    start: Point2D,
    target: Point2D,
    zones: Iterable[tuple[Point2D, float]],
) -> Point2D | None:
    """Step out of every obstacle zone that contains ``start``; ``None`` if none does.

    Shared by all three planners (ACRA 2026 fairness fix, 1 Oct): each planner
    passes its own inflated obstacle zones (centre, radius), i.e. exactly the
    region inside which it would otherwise refuse to plan from ``start``. The
    push direction is the overlap-weighted sum of "away from centre" vectors;
    the step is ``max(min_step, max_overlap + margin)``, clamped to the field.
    Originally the Voronoi planner's private escape step; moved here so PRM
    and the visibility graph get the same behaviour.
    """
    push_x = push_y = max_overlap = 0.0
    for (cx, cy), radius in zones:
        dx, dy = start[0] - cx, start[1] - cy
        dist = hypot(dx, dy)
        overlap = radius - dist
        if overlap < 0.0:
            continue
        if dist <= 1e-6:
            dx, dy = target[0] - cx, target[1] - cy
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
    step = max(float(VORONOI_MIN_ESCAPE_STEP_MM), max_overlap + float(VORONOI_ESCAPE_MARGIN_MM))
    m = float(VORONOI_TARGET_DEAD_ZONE_MM)
    return (
        max(float(FIELD_X_MIN) + m, min(float(FIELD_X_MAX) - m, start[0] + push_x / push_len * step)),
        max(float(FIELD_Y_MIN) + m, min(float(FIELD_Y_MAX) - m, start[1] + push_y / push_len * step)),
    )


def commit_reroute(
    state: RouteState, waypoints: tuple[Pose2D, ...], target_pose: Pose2D
) -> None:
    """Record that a full reroute just happened -- resets the periodic cadence."""
    state.waypoints = waypoints
    state.last_target_pose = target_pose
    state.frames_since_reroute = 0


def note_no_reroute(state: RouteState) -> None:
    """Record that this call kept the cached route -- advances the periodic cadence.

    Deliberately does *not* touch ``last_target_pose``: ``target_moved`` must
    keep measuring drift since the last *committed* route, not since the last
    call, or small per-call moves under the dead zone would silently reset
    the baseline and mask a real cumulative drift.
    """
    state.frames_since_reroute += 1


def _pose_xy(pose: Pose2D) -> Point2D:
    return (float(pose[0]), float(pose[1]))


def _distance(a: Point2D, b: Point2D) -> float:
    return hypot(a[0] - b[0], a[1] - b[1])
