"""Shared types for standalone path-planning algorithms.

Units are millimetres throughout, matching
``process_workers/voronoi_map_runner.py`` and the RoboCup SSL convention (a
Division B field is 9000mm x 6000mm; ``FIELD_LENGTH_MM``/``FIELD_WIDTH_MM``
below match that). The origin is the field centre, x along the long axis,
matching ``_field_bounds_from_request`` in ``voronoi_map_runner.py``
(``-length/2 .. +length/2``, ``-width/2 .. +width/2``).

``Obstacle`` intentionally mirrors the attributes ``voronoi_map_runner.py``
already reads off obstacle objects (``pos_mm``, ``radius_mm``, ``robot_id``,
``isYellow``) so a real vision-derived obstacle list can be passed to these
planners without an adapter, once one exists.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from itertools import pairwise
from time import perf_counter

FIELD_LENGTH_MM = 9000.0
FIELD_WIDTH_MM = 6000.0

# Standard SSL robot radius is ~90mm (max diameter 180mm per the rules).
DEFAULT_ROBOT_RADIUS_MM = 90.0


@dataclass(frozen=True)
class Obstacle:
    """A circular obstacle (another robot, typically) in field coordinates.

    ``pos_mm`` is ``(x, y)`` -- a 2-tuple, not the 3-tuple
    ``voronoi_map_runner.py`` uses for robots (which also carries heading in
    slot 2); planners here only need position, so keep this the minimal
    shape and let a caller slice ``(x, y, theta)`` down to ``(x, y)``.
    """

    pos_mm: tuple[float, float]
    radius_mm: float = DEFAULT_ROBOT_RADIUS_MM
    robot_id: int = -1
    isYellow: bool = True

    def x(self) -> float:
        return self.pos_mm[0]

    def y(self) -> float:
        return self.pos_mm[1]


@dataclass(frozen=True)
class PlanRequest:
    """One planning query: get from ``start_mm`` to ``goal_mm`` avoiding ``obstacles``."""

    start_mm: tuple[float, float]
    goal_mm: tuple[float, float]
    obstacles: tuple[Obstacle, ...] = field(default_factory=tuple)
    robot_radius_mm: float = DEFAULT_ROBOT_RADIUS_MM
    clearance_mm: float = 30.0
    field_length_mm: float = FIELD_LENGTH_MM
    field_width_mm: float = FIELD_WIDTH_MM

    @property
    def total_clearance_mm(self) -> float:
        """Distance a path must keep from an obstacle's *centre*.

        This is the sum of the obstacle's own radius, the planning robot's
        radius, and the extra safety margin -- i.e. the Minkowski-sum radius
        used to inflate obstacles before graph construction.
        """
        return self.robot_radius_mm + self.clearance_mm


@dataclass(frozen=True)
class PlanResult:
    """Output of a planner: a polyline from start to goal, or failure."""

    success: bool
    waypoints_mm: tuple[tuple[float, float], ...]
    path_length_mm: float
    planning_time_ms: float
    nodes_expanded: int = 0
    message: str = ""


def path_length_mm(waypoints: tuple[tuple[float, float], ...]) -> float:
    """Sum of Euclidean segment lengths along a waypoint polyline."""
    total = 0.0
    for (x0, y0), (x1, y1) in pairwise(waypoints):
        total += ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
    return total


class StepRecorder:
    """Optional recorder for "how the algorithm built this path" animation frames.

    Pass an instance to ``plan(..., record=recorder)`` on either planner and
    it logs each meaningful construction step (a sample drawn, an edge
    tested and accepted/rejected, an obstacle inflated, the final path) into
    ``.steps`` as it goes. Not passing one (the default, ``record=None``) is
    completely free -- every call site in the planners does
    ``if record is not None: record.log(...)`` first, so normal planning
    calls (including every existing unit test) never construct or touch a
    recorder at all.

    Consumed by ``algorithms.viz.animate_construction`` to turn ``.steps``
    into a saved GIF -- see ``scripts/animate_planner.py`` for a runnable
    example.
    """

    def __init__(self) -> None:
        self.steps: list[dict] = []
        self.started_at = perf_counter()
        self.first_map_event_at: float | None = None
        self.last_map_event_at: float | None = None
        self.path_event_at: float | None = None

    def log(self, kind: str, **data) -> None:
        now = perf_counter()
        if kind == "path":
            self.path_event_at = now
        else:
            if self.first_map_event_at is None:
                self.first_map_event_at = now
            self.last_map_event_at = now
        self.steps.append({"kind": kind, **data})

    @property
    def map_time_ms(self) -> float | None:
        if self.first_map_event_at is None or self.last_map_event_at is None:
            return None
        return (self.last_map_event_at - self.first_map_event_at) * 1000.0

    @property
    def search_time_ms(self) -> float | None:
        if self.path_event_at is None:
            return None
        start = self.last_map_event_at or self.started_at
        return (self.path_event_at - start) * 1000.0
