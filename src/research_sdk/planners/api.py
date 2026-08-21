"""Public planner API used by robot behaviours and future skill execution."""

from __future__ import annotations

from research_sdk.planners.Dijkstra.waypoint_manager import (
    PlannerInput,
    PlannerOutput,
    TargetClearanceStatus,
    VoronoiWaypointManager,
    check_target_clearance,
)


class PlannerAPI:
    """Small facade around the stateful waypoint manager."""

    def __init__(self, **manager_kwargs) -> None:
        self._manager = VoronoiWaypointManager(**manager_kwargs)

    def plan(self, planner_input: PlannerInput) -> PlannerOutput:
        """Return the next active target and route status for one robot tick."""
        return self._manager.update(planner_input)

    def check_target_clearance(
        self,
        target_pose,
        obstacles,
        *,
        clearance_mm: float = 0.0,
        endpoint_reach_mm: float | None = None,
        ignore_robots=(),
    ) -> TargetClearanceStatus:
        """Classify a target against safety and close-reach obstacle clearance."""
        kwargs = {
            "clearance_mm": clearance_mm,
            "ignore_robots": ignore_robots,
        }
        if endpoint_reach_mm is not None:
            kwargs["endpoint_reach_mm"] = endpoint_reach_mm
        return check_target_clearance(target_pose, obstacles, **kwargs)

    def reset(self, robot_id: int | None = None, is_yellow: bool | None = None) -> None:
        """Clear planner state for one robot or every robot."""
        self._manager.reset(robot_id=robot_id, is_yellow=is_yellow)


def plan(
    planner_input: PlannerInput,
    *,
    planner_api: PlannerAPI | None = None,
    **manager_kwargs,
) -> PlannerOutput:
    """One-shot helper for callers that do not need persistent state."""
    api = planner_api or PlannerAPI(**manager_kwargs)
    return api.plan(planner_input)
