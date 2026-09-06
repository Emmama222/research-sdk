"""Planner-independent two-dimensional obstacle geometry.

The primitives in this module use millimetres and radians.  They deliberately
do not depend on a particular planner so mapping, route validation, and every
planner adapter can share exactly the same collision rules.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, sin


Point2D = tuple[float, float]
_EPSILON = 1e-9


@dataclass(frozen=True, slots=True)
class OrientedRectangle:
    """A rectangle centred at ``center_mm`` and rotated in the world frame."""

    center_mm: Point2D
    length_mm: float
    width_mm: float
    orientation_rad: float = 0.0

    def __post_init__(self) -> None:
        values = (*self.center_mm, self.length_mm, self.width_mm, self.orientation_rad)
        if not all(isfinite(value) for value in values):
            raise ValueError("rectangle values must be finite")
        if self.length_mm <= 0.0 or self.width_mm <= 0.0:
            raise ValueError("rectangle length and width must be positive")

    @classmethod
    def from_motion(
        cls,
        position_mm: Point2D,
        velocity_mmps: Point2D,
        *,
        horizon_ms: float,
        radius_mm: float,
        buffer_mm: float = 0.0,
        lateral_growth_factor: float = 0.01,
        stationary_orientation_rad: float = 0.0,
        stationary_speed_threshold_mmps: float = 1e-6,
    ) -> "OrientedRectangle":
        """Build the velocity-aligned prediction envelope for a circular robot.

        ``velocity_mmps`` is expressed in millimetres per second while
        ``horizon_ms`` is milliseconds, so travelled distance includes the
        required division by 1000.  The rectangle is centred halfway along the
        predicted displacement so it covers both the observed and predicted
        robot positions.
        """
        scalars = (
            horizon_ms,
            radius_mm,
            buffer_mm,
            lateral_growth_factor,
            stationary_orientation_rad,
            stationary_speed_threshold_mmps,
        )
        if not all(isfinite(value) for value in (*position_mm, *velocity_mmps, *scalars)):
            raise ValueError("motion prediction values must be finite")
        if horizon_ms < 0.0:
            raise ValueError("horizon_ms must be non-negative")
        if radius_mm <= 0.0:
            raise ValueError("radius_mm must be positive")
        if buffer_mm < 0.0:
            raise ValueError("buffer_mm must be non-negative")
        if lateral_growth_factor < 0.0:
            raise ValueError("lateral_growth_factor must be non-negative")
        if stationary_speed_threshold_mmps < 0.0:
            raise ValueError("stationary_speed_threshold_mmps must be non-negative")

        vx, vy = velocity_mmps
        horizon_s = horizon_ms / 1000.0
        displacement = (vx * horizon_s, vy * horizon_s)
        travel_mm = hypot(*displacement)
        speed_mmps = hypot(vx, vy)
        base_diameter_mm = 2.0 * (radius_mm + buffer_mm)

        orientation_rad = (
            atan2(vy, vx)
            if speed_mmps > stationary_speed_threshold_mmps
            else stationary_orientation_rad
        )
        return cls(
            center_mm=(
                position_mm[0] + displacement[0] / 2.0,
                position_mm[1] + displacement[1] / 2.0,
            ),
            length_mm=base_diameter_mm + travel_mm,
            width_mm=base_diameter_mm + lateral_growth_factor * travel_mm,
            orientation_rad=orientation_rad,
        )

    @property
    def corners_mm(self) -> tuple[Point2D, Point2D, Point2D, Point2D]:
        """Return the four corners in counter-clockwise local order."""
        half_length = self.length_mm / 2.0
        half_width = self.width_mm / 2.0
        return tuple(
            self._to_world(local)
            for local in (
                (-half_length, -half_width),
                (half_length, -half_width),
                (half_length, half_width),
                (-half_length, half_width),
            )
        )

    def inflated(self, clearance_mm: float) -> "OrientedRectangle":
        """Return the Minkowski-style axis inflation by ``clearance_mm``."""
        if not isfinite(clearance_mm) or clearance_mm < 0.0:
            raise ValueError("clearance_mm must be finite and non-negative")
        return OrientedRectangle(
            center_mm=self.center_mm,
            length_mm=self.length_mm + 2.0 * clearance_mm,
            width_mm=self.width_mm + 2.0 * clearance_mm,
            orientation_rad=self.orientation_rad,
        )

    def contains_point(self, point_mm: Point2D) -> bool:
        """Return whether ``point_mm`` lies inside or on the rectangle."""
        x, y = self._to_local(point_mm)
        return (
            abs(x) <= self.length_mm / 2.0 + _EPSILON
            and abs(y) <= self.width_mm / 2.0 + _EPSILON
        )

    def intersects_segment(self, start_mm: Point2D, end_mm: Point2D) -> bool:
        """Return whether a closed line segment touches or crosses the rectangle."""
        if self.contains_point(start_mm) or self.contains_point(end_mm):
            return True
        corners = self.corners_mm
        return any(
            _segments_intersect(start_mm, end_mm, corners[index], corners[(index + 1) % 4])
            for index in range(4)
        )

    def clearance_to_segment(self, start_mm: Point2D, end_mm: Point2D) -> float:
        """Return the shortest distance from a segment to the rectangle boundary."""
        if self.intersects_segment(start_mm, end_mm):
            return 0.0
        corners = self.corners_mm
        return min(
            _segment_distance(start_mm, end_mm, corners[index], corners[(index + 1) % 4])
            for index in range(4)
        )

    def _to_local(self, point_mm: Point2D) -> Point2D:
        dx = point_mm[0] - self.center_mm[0]
        dy = point_mm[1] - self.center_mm[1]
        c = cos(self.orientation_rad)
        s = sin(self.orientation_rad)
        return (dx * c + dy * s, -dx * s + dy * c)

    def _to_world(self, point: Point2D) -> Point2D:
        c = cos(self.orientation_rad)
        s = sin(self.orientation_rad)
        return (
            self.center_mm[0] + point[0] * c - point[1] * s,
            self.center_mm[1] + point[0] * s + point[1] * c,
        )


def _orientation(a: Point2D, b: Point2D, c: Point2D) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: Point2D, b: Point2D, point: Point2D) -> bool:
    return (
        min(a[0], b[0]) - _EPSILON <= point[0] <= max(a[0], b[0]) + _EPSILON
        and min(a[1], b[1]) - _EPSILON <= point[1] <= max(a[1], b[1]) + _EPSILON
    )


def _segments_intersect(a: Point2D, b: Point2D, c: Point2D, d: Point2D) -> bool:
    orientations = (
        _orientation(a, b, c),
        _orientation(a, b, d),
        _orientation(c, d, a),
        _orientation(c, d, b),
    )
    if orientations[0] * orientations[1] < 0.0 and orientations[2] * orientations[3] < 0.0:
        return True
    return (
        (abs(orientations[0]) <= _EPSILON and _on_segment(a, b, c))
        or (abs(orientations[1]) <= _EPSILON and _on_segment(a, b, d))
        or (abs(orientations[2]) <= _EPSILON and _on_segment(c, d, a))
        or (abs(orientations[3]) <= _EPSILON and _on_segment(c, d, b))
    )


def _point_segment_distance(point: Point2D, start: Point2D, end: Point2D) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= _EPSILON:
        return hypot(point[0] - start[0], point[1] - start[1])
    projection = (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / length_squared
    projection = max(0.0, min(1.0, projection))
    closest = (start[0] + projection * dx, start[1] + projection * dy)
    return hypot(point[0] - closest[0], point[1] - closest[1])


def _segment_distance(a: Point2D, b: Point2D, c: Point2D, d: Point2D) -> float:
    if _segments_intersect(a, b, c, d):
        return 0.0
    return min(
        _point_segment_distance(a, c, d),
        _point_segment_distance(b, c, d),
        _point_segment_distance(c, a, b),
        _point_segment_distance(d, a, b),
    )
