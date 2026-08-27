"""Immutable, planner-facing view of a tracked world state."""

from __future__ import annotations

from dataclasses import dataclass

from research_sdk.config import FIELD_LENGTH_MM, FIELD_WIDTH_MM, ROBOT_RADIUS_MM
from research_sdk.world.map.geometry import distance_2_segment


Point2D = tuple[float, float]
RobotKey = tuple[bool, int]


@dataclass(frozen=True, slots=True)
class PlanningObstacle:
    """Predicted circular obstacle supplied identically to every planner."""

    robot_id: int
    isYellow: bool
    pos_mm: Point2D
    radius_mm: float
    vel_mmps: Point2D = (0.0, 0.0)
    observation_age_ms: float = 0.0
    prediction_horizon_ms: float = 0.0

    @property
    def key(self) -> RobotKey:
        return (self.isYellow, self.robot_id)


@dataclass(frozen=True, slots=True)
class FieldDimensions:
    """Minimal field geometry required by path planners."""

    field_length: float = FIELD_LENGTH_MM
    field_width: float = FIELD_WIDTH_MM

    @property
    def bounds_mm(self) -> tuple[float, float, float, float]:
        return (
            -self.field_length / 2.0,
            self.field_length / 2.0,
            -self.field_width / 2.0,
            self.field_width / 2.0,
        )


@dataclass(frozen=True, slots=True)
class PlanningScene:
    """Frozen input boundary shared by all path-planning algorithms.

    Obstacles are predicted when the scene is built, so a planning run cannot
    observe live world-state mutations halfway through its calculation.
    """

    timestamp: float
    obstacles: tuple[PlanningObstacle, ...]
    field: FieldDimensions = FieldDimensions()
    prediction_horizon_ms: float = 0.0

    def get_field_size(self) -> FieldDimensions:
        return self.field

    def get_planning_obstacles(
        self,
        now_s: float | None = None,
        horizon_ms: int | float | None = None,
        ignore_robots: set[RobotKey] | None = None,
    ) -> tuple[PlanningObstacle, ...]:
        """Return scene obstacles, optionally excluding specified robots."""
        del now_s, horizon_ms
        ignored = ignore_robots or set()
        return tuple(obstacle for obstacle in self.obstacles if obstacle.key not in ignored)

    def is_path_free(
        self,
        start_pos: Point2D,
        end_pos: Point2D,
        ignore_robots: set[RobotKey] | None = None,
        clearance: float = 0.0,
        horizon_ms: int | float | None = None,
    ) -> bool:
        """Return whether a robot centre can traverse a segment collision-free."""
        del horizon_ms
        ignored = ignore_robots or set()
        return all(
            obstacle.key in ignored
            or distance_2_segment(obstacle.pos_mm, start_pos, end_pos)
            > obstacle.radius_mm + ROBOT_RADIUS_MM + clearance
            for obstacle in self.obstacles
        )
