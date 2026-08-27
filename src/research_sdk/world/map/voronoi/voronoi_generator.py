"""Bounded Voronoi map generation for field navigation."""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, floor, hypot, sqrt
import random
from typing import Iterable, Literal

from research_sdk.config import (
    FIELD_LENGTH_MM,
    FIELD_WIDTH_MM,
    VORONOI_BOUNDARY_INSET_MM,
    VORONOI_MIN_CLEARANCE_MM,
    VORONOI_RENDER_DENSITY_PERCENT,
    VORONOI_RENDER_MAX_DENSITY_NODES,
    VORONOI_OBSTACLE_COST_WEIGHT,
)
from research_sdk.world.map.geometry import distance_2_segment
from research_sdk.ui.renderer import RenderCircle, RenderLayer, RenderPolyline
from research_sdk.world.map.map_graph import MapEdge, MapNode


Point = tuple[float, float]
Bounds = tuple[float, float, float, float]
PlacementMode = Literal["density_grid", "grid", "random"]

EPSILON = 1e-9


@dataclass(frozen=True, slots=True)
class VoronoiCell:
    site_id: int
    site_mm: Point
    polygon_mm: tuple[Point, ...]
    kind: str = "virtual"


@dataclass(frozen=True, slots=True)
class VoronoiObstacle:
    """Circular obstacle used to shape and filter the Voronoi graph."""

    pos_mm: Point
    radius_mm: float
    label: str = ""


@dataclass(frozen=True, slots=True)
class BoundedVoronoiMap:
    """Closed Voronoi cells and clearance-filtered navigation graph."""

    bounds_mm: Bounds
    cells: tuple[VoronoiCell, ...]
    nodes: tuple[MapNode, ...]
    edges: tuple[MapEdge, ...]
    min_clearance_mm: float
    virtual_sites_mm: tuple[Point, ...]
    obstacles: tuple[VoronoiObstacle, ...]
    field_bounds_mm: Bounds
    boundary_inset_mm: float
    placement_mode: str

    @property
    def sites_mm(self) -> tuple[Point, ...]:
        """All sites used by the Voronoi diagram, virtual first."""
        return self.virtual_sites_mm + tuple(obstacle.pos_mm for obstacle in self.obstacles)

    @property
    def clipped_bounds_mm(self) -> Bounds:
        """Navigation bounds inset again by clearance for safe cell clipping."""
        return _inset_bounds(self.bounds_mm, self.min_clearance_mm)

    def render_layer(
        self,
        name: str = "Voronoi map",
        *,
        visible_by_default: bool = True,
    ) -> RenderLayer:
        """Return a renderer layer for debug display."""
        polylines = [
            RenderPolyline(cell.polygon_mm, color="#8fd3ff", closed=True)
            for cell in self.cells
            if len(cell.polygon_mm) >= 3
        ]

        node_by_id = {node.id: node for node in self.nodes}
        for edge in self.edges:
            start = node_by_id[edge.start_id]
            end = node_by_id[edge.end_id]
            polylines.append(
                RenderPolyline(
                    points_mm=((start.x, start.y), (end.x, end.y)),
                    color="#f2c94c",
                )
            )

        circles = tuple(
            RenderCircle(
                center_mm=obstacle.pos_mm,
                radius_mm=obstacle.radius_mm + self.min_clearance_mm,
                color="#8e7cc3",
                label=obstacle.label,
            )
            for obstacle in self.obstacles
        )

        return RenderLayer(
            name,
            circles=circles,
            polylines=tuple(polylines),
            visible_by_default=visible_by_default,
        )


def generate_bounded_voronoi_map(
    virtual_node_count: int | None = None,
    *,
    field_length_mm: float = FIELD_LENGTH_MM,
    field_width_mm: float = FIELD_WIDTH_MM,
    min_clearance_mm: float = VORONOI_MIN_CLEARANCE_MM,
    boundary_inset_mm: float = VORONOI_BOUNDARY_INSET_MM,
    placement_mode: PlacementMode = "density_grid",
    density_percent: float = VORONOI_RENDER_DENSITY_PERCENT,
    grid_spacing_x_mm: float | None = None,
    grid_spacing_y_mm: float | None = None,
    max_density_nodes: int = VORONOI_RENDER_MAX_DENSITY_NODES,
    obstacles: Iterable[VoronoiObstacle | object] = (),
    obstacle_cost_weight: float = VORONOI_OBSTACLE_COST_WEIGHT,
    include_perimeter_edges: bool = True,
    seed: int | None = None,
) -> BoundedVoronoiMap:
    """
    Generate a closed 9x6m-style Voronoi map and keep only safe graph edges.

    ``bounds_mm`` is the inset navigation region. ``field_bounds_mm`` is the
    true field rectangle. Every returned edge has at least ``min_clearance_mm``
    from virtual sites and remains inside the inset navigation region.
    """
    if virtual_node_count is not None and virtual_node_count < 2:
        raise ValueError("virtual_node_count must be at least 2")
    if field_length_mm <= 0 or field_width_mm <= 0:
        raise ValueError("field dimensions must be positive")
    if min_clearance_mm < 0:
        raise ValueError("min_clearance_mm must be non-negative")
    if boundary_inset_mm < 0:
        raise ValueError("boundary_inset_mm must be non-negative")
    if placement_mode not in ("density_grid", "grid", "random"):
        raise ValueError("placement_mode must be density_grid, grid, or random")
    if obstacle_cost_weight < 0:
        raise ValueError("obstacle_cost_weight must be non-negative")

    field_bounds = (
        -field_length_mm / 2.0,
        field_length_mm / 2.0,
        -field_width_mm / 2.0,
        field_width_mm / 2.0,
    )
    bounds = _inset_bounds(field_bounds, boundary_inset_mm)
    clipped_bounds = _inset_bounds(bounds, min_clearance_mm)
    sites = _generate_virtual_sites(
        virtual_node_count=virtual_node_count,
        bounds=bounds,
        min_clearance_mm=min_clearance_mm,
        placement_mode=placement_mode,
        density_percent=density_percent,
        grid_spacing_x_mm=grid_spacing_x_mm,
        grid_spacing_y_mm=grid_spacing_y_mm,
        max_density_nodes=max_density_nodes,
        seed=seed,
    )
    obstacle_sites = normalize_voronoi_obstacles(obstacles)
    all_sites = sites + tuple(obstacle.pos_mm for obstacle in obstacle_sites)
    cells = _build_cells(
        all_sites,
        clipped_bounds,
        virtual_site_count=len(sites),
    )
    nodes, edges = _build_navigation_graph(
        cells,
        all_sites,
        bounds,
        min_clearance_mm=min_clearance_mm,
        obstacles=obstacle_sites,
        obstacle_cost_weight=obstacle_cost_weight,
        include_perimeter_edges=include_perimeter_edges,
    )

    return BoundedVoronoiMap(
        bounds_mm=bounds,
        cells=cells,
        nodes=nodes,
        edges=edges,
        min_clearance_mm=min_clearance_mm,
        virtual_sites_mm=sites,
        obstacles=obstacle_sites,
        field_bounds_mm=field_bounds,
        boundary_inset_mm=boundary_inset_mm,
        placement_mode=placement_mode,
    )


def generate_voronoi_map_from_scene(
    scene,
    *,
    now_s: float | None = None,
    horizon_ms: int | float | None = None,
    ignore_robots: set[tuple[bool, int]] | None = None,
    **kwargs,
) -> BoundedVoronoiMap:
    """Generate a Voronoi map from a frozen planning scene or compatible view."""
    field_dimensions = _field_dimensions_from_world_map(scene)
    if field_dimensions is not None:
        field_length_mm, field_width_mm = field_dimensions
        kwargs.setdefault("field_length_mm", field_length_mm)
        kwargs.setdefault("field_width_mm", field_width_mm)

    return generate_bounded_voronoi_map(
        obstacles=scene.get_planning_obstacles(
            now_s=now_s,
            horizon_ms=horizon_ms,
            ignore_robots=ignore_robots,
        ),
        **kwargs,
    )


# Compatibility name for UI code that still renders directly from WorldMap.
generate_voronoi_map_from_world_map = generate_voronoi_map_from_scene


def _field_dimensions_from_world_map(world_map) -> tuple[float, float] | None:
    field = getattr(world_map, "field", None)
    if field is None and hasattr(world_map, "get_field_size"):
        try:
            field = world_map.get_field_size()
        except Exception:
            field = None
    if field is None:
        return None

    length = getattr(field, "field_length", None)
    width = getattr(field, "field_width", None)
    if length is None or width is None:
        return None
    length = float(length)
    width = float(width)
    if length <= 0.0 or width <= 0.0:
        return None
    return length, width


def normalize_voronoi_obstacles(
    obstacles: Iterable[VoronoiObstacle | object],
) -> tuple[VoronoiObstacle, ...]:
    """Accept local VoronoiObstacle objects or WorldMap PlanningObstacle objects."""
    normalized = []
    for obstacle in obstacles:
        if isinstance(obstacle, VoronoiObstacle):
            normalized.append(obstacle)
            continue

        pos = getattr(obstacle, "pos_mm")
        radius = float(getattr(obstacle, "radius_mm", 0.0))
        label = getattr(obstacle, "label", "")
        if not label and hasattr(obstacle, "robot_id"):
            team = "Y" if getattr(obstacle, "isYellow", False) else "B"
            label = f"{team}{getattr(obstacle, 'robot_id')}"
        normalized.append(
            VoronoiObstacle(
                pos_mm=(float(pos[0]), float(pos[1])),
                radius_mm=radius,
                label=str(label),
            )
        )
    return tuple(normalized)


def edge_clearance_mm(
    start_mm: Point,
    end_mm: Point,
    sites_mm: Iterable[Point],
    bounds_mm: Bounds,
) -> float:
    """Return the minimum clearance from a segment to sites and boundaries."""
    site_clearance = min(
        distance_2_segment(site, start_mm, end_mm)
        for site in sites_mm
    )
    boundary_clearance = min(
        start_mm[0] - bounds_mm[0],
        bounds_mm[1] - start_mm[0],
        start_mm[1] - bounds_mm[2],
        bounds_mm[3] - start_mm[1],
        end_mm[0] - bounds_mm[0],
        bounds_mm[1] - end_mm[0],
        end_mm[1] - bounds_mm[2],
        bounds_mm[3] - end_mm[1],
    )
    return min(site_clearance, boundary_clearance)


def obstacle_clearance_mm(
    start_mm: Point,
    end_mm: Point,
    obstacles: Iterable[VoronoiObstacle],
) -> float:
    """Return segment clearance after subtracting obstacle radii."""
    obstacles = tuple(obstacles)
    if not obstacles:
        return float("inf")
    return min(
        distance_2_segment(obstacle.pos_mm, start_mm, end_mm) - obstacle.radius_mm
        for obstacle in obstacles
    )


def _generate_virtual_sites(
    *,
    virtual_node_count: int | None,
    bounds: Bounds,
    min_clearance_mm: float,
    placement_mode: PlacementMode,
    density_percent: float,
    grid_spacing_x_mm: float | None,
    grid_spacing_y_mm: float | None,
    max_density_nodes: int,
    seed: int | None,
) -> tuple[Point, ...]:
    if placement_mode == "grid":
        return tuple(
            _grid_sites_by_spacing(
                bounds,
                spacing_x_mm=grid_spacing_x_mm or min_clearance_mm * 2.0,
                spacing_y_mm=grid_spacing_y_mm or min_clearance_mm * 2.0,
            )
        )

    if placement_mode == "density_grid":
        if virtual_node_count is not None:
            return tuple(_grid_sites(virtual_node_count, bounds))
        return tuple(
            _density_grid_sites(
                bounds,
                min_clearance_mm=min_clearance_mm,
                density_percent=density_percent,
                max_density_nodes=max_density_nodes,
            )
        )

    count = virtual_node_count
    if count is None:
        raise ValueError("virtual_node_count is required for random placement")
    rng = random.Random(seed)
    spacing = max(1.0, min_clearance_mm * 2.0)

    sites: list[Point] = []
    max_attempts = max(200, count * 80)
    for _ in range(max_attempts):
        candidate = (
            rng.uniform(bounds[0], bounds[1]),
            rng.uniform(bounds[2], bounds[3]),
        )
        if all(_distance(candidate, site) >= spacing for site in sites):
            sites.append(candidate)
            if len(sites) == count:
                return tuple(sites)

    return tuple(_grid_sites(count, bounds))


def _density_grid_sites(
    bounds: Bounds,
    *,
    min_clearance_mm: float,
    density_percent: float,
    max_density_nodes: int,
) -> list[Point]:
    density_percent = max(10.0, min(100.0, density_percent))
    if max_density_nodes < 8:
        raise ValueError("max_density_nodes must be at least 8")

    x_min, x_max, y_min, y_max = bounds
    width = x_max - x_min
    height = y_max - y_min
    min_spacing = max(1.0, min_clearance_mm * 2.0)
    max_cols = max(4, floor(width / min_spacing))
    max_rows = max(2, floor(height / min_spacing))

    if max_cols * max_rows > max_density_nodes:
        scale = sqrt(max_density_nodes / (max_cols * max_rows))
        max_cols = max(4, floor(max_cols * scale))
        max_rows = max(2, floor(max_rows * scale))

    t = (density_percent - 10.0) / 90.0
    cols = max(4, round(4 + (max_cols - 4) * t))
    rows = max(2, round(2 + (max_rows - 2) * t))

    return _grid_sites_by_shape(bounds, cols, rows)


def _grid_sites_by_spacing(
    bounds: Bounds,
    *,
    spacing_x_mm: float,
    spacing_y_mm: float,
) -> list[Point]:
    if spacing_x_mm <= 0 or spacing_y_mm <= 0:
        raise ValueError("grid spacing must be positive")
    x_min, x_max, y_min, y_max = bounds
    cols = max(2, floor((x_max - x_min) / spacing_x_mm) + 1)
    rows = max(2, floor((y_max - y_min) / spacing_y_mm) + 1)
    return _grid_sites_by_shape(bounds, cols, rows)


def _grid_sites_by_shape(bounds: Bounds, cols: int, rows: int) -> list[Point]:
    x_min, x_max, y_min, y_max = bounds
    if cols < 2 or rows < 2:
        raise ValueError("grid must have at least 2 columns and 2 rows")
    x_step = (x_max - x_min) / (cols - 1)
    y_step = (y_max - y_min) / (rows - 1)
    return [
        (x_min + col * x_step, y_min + row * y_step)
        for row in range(rows)
        for col in range(cols)
    ]


def _grid_sites(count: int, bounds: Bounds) -> list[Point]:
    x_min, x_max, y_min, y_max = bounds
    cols = max(1, ceil(sqrt(count * (x_max - x_min) / (y_max - y_min))))
    rows = max(1, ceil(count / cols))
    x_step = (x_max - x_min) / cols
    y_step = (y_max - y_min) / rows

    sites = []
    for row in range(rows):
        for col in range(cols):
            sites.append(
                (
                    x_min + (col + 0.5) * x_step,
                    y_min + (row + 0.5) * y_step,
                )
            )
            if len(sites) == count:
                return sites
    return sites


def _build_cells(
    sites: tuple[Point, ...],
    bounds: Bounds,
    *,
    virtual_site_count: int,
) -> tuple[VoronoiCell, ...]:
    x_min, x_max, y_min, y_max = bounds
    box = ((x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max))
    cells = []

    for site_id, site in enumerate(sites):
        polygon = list(box)
        for other_id, other in enumerate(sites):
            if other_id == site_id:
                continue
            polygon = _clip_to_nearer_half_plane(polygon, site, other)
            if not polygon:
                break
        if len(polygon) >= 3:
            cells.append(
                VoronoiCell(
                    site_id=site_id,
                    site_mm=site,
                    polygon_mm=tuple(_dedupe_polygon(polygon)),
                    kind="virtual" if site_id < virtual_site_count else "obstacle",
                )
            )

    return tuple(cells)


def _clip_to_nearer_half_plane(
    polygon: list[Point],
    site: Point,
    other: Point,
) -> list[Point]:
    a = 2.0 * (other[0] - site[0])
    b = 2.0 * (other[1] - site[1])
    c = other[0] ** 2 + other[1] ** 2 - site[0] ** 2 - site[1] ** 2

    def inside(point: Point) -> bool:
        return a * point[0] + b * point[1] <= c + EPSILON

    def intersection(start: Point, end: Point) -> Point:
        sx, sy = start
        ex, ey = end
        denominator = a * (ex - sx) + b * (ey - sy)
        if abs(denominator) < EPSILON:
            return end
        t = (c - a * sx - b * sy) / denominator
        t = max(0.0, min(1.0, t))
        return (sx + (ex - sx) * t, sy + (ey - sy) * t)

    clipped = []
    previous = polygon[-1]
    previous_inside = inside(previous)
    for current in polygon:
        current_inside = inside(current)
        if current_inside:
            if not previous_inside:
                clipped.append(intersection(previous, current))
            clipped.append(current)
        elif previous_inside:
            clipped.append(intersection(previous, current))
        previous = current
        previous_inside = current_inside

    return clipped


def _build_navigation_graph(
    cells: tuple[VoronoiCell, ...],
    sites: tuple[Point, ...],
    bounds: Bounds,
    *,
    min_clearance_mm: float,
    obstacles: tuple[VoronoiObstacle, ...],
    obstacle_cost_weight: float,
    include_perimeter_edges: bool,
) -> tuple[tuple[MapNode, ...], tuple[MapEdge, ...]]:
    node_ids: dict[Point, int] = {}
    nodes: list[MapNode] = []
    unique_edges: dict[tuple[Point, Point], float] = {}

    for cell in cells:
        points = cell.polygon_mm
        for index, start in enumerate(points):
            end = points[(index + 1) % len(points)]
            if _distance(start, end) < EPSILON:
                continue
            clearance = edge_clearance_mm(start, end, sites, bounds)
            obstacle_clearance = obstacle_clearance_mm(start, end, obstacles)
            clearance = min(clearance, obstacle_clearance)
            if clearance + EPSILON < min_clearance_mm:
                clipped = _clip_segment_to_inset_bounds(
                    start,
                    end,
                    bounds,
                    min_clearance_mm,
                )
                if clipped is None:
                    continue
                start, end = clipped
                if _distance(start, end) < EPSILON:
                    continue
                clearance = edge_clearance_mm(start, end, sites, bounds)
                obstacle_clearance = obstacle_clearance_mm(start, end, obstacles)
                clearance = min(clearance, obstacle_clearance)
                if clearance + EPSILON < min_clearance_mm:
                    continue
            key = _segment_key(start, end)
            unique_edges[key] = max(clearance, unique_edges.get(key, 0.0))

    if include_perimeter_edges:
        _add_perimeter_edges(
            unique_edges,
            sites,
            bounds,
            min_clearance_mm=min_clearance_mm,
            obstacles=obstacles,
        )

    edges: list[MapEdge] = []
    for (start, end), clearance in unique_edges.items():
        start_id = _node_id(start, node_ids, nodes)
        end_id = _node_id(end, node_ids, nodes)
        obstacle_risk = _obstacle_proximity_risk(
            start,
            end,
            obstacles,
            min_clearance_mm=min_clearance_mm,
        )
        edges.append(
            MapEdge(
                start_id=start_id,
                end_id=end_id,
                cost=_edge_cost(
                    start,
                    end,
                    clearance=clearance,
                    min_clearance_mm=min_clearance_mm,
                    obstacle_cost_weight=obstacle_cost_weight,
                    obstacle_risk=obstacle_risk,
                ),
                clearance=clearance,
            )
        )

    return tuple(nodes), tuple(edges)


def _clip_segment_to_inset_bounds(
    start: Point,
    end: Point,
    bounds: Bounds,
    inset_mm: float,
) -> tuple[Point, Point] | None:
    """Clip a segment to the clearance-inset bounds."""
    try:
        x_min, x_max, y_min, y_max = _inset_bounds(bounds, inset_mm)
    except ValueError:
        return None

    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    t0 = 0.0
    t1 = 1.0

    for p, q in (
        (-dx, sx - x_min),
        (dx, x_max - sx),
        (-dy, sy - y_min),
        (dy, y_max - sy),
    ):
        if abs(p) < EPSILON:
            if q < 0.0:
                return None
            continue
        r = q / p
        if p < 0.0:
            if r > t1:
                return None
            t0 = max(t0, r)
        else:
            if r < t0:
                return None
            t1 = min(t1, r)

    clipped_start = (sx + dx * t0, sy + dy * t0)
    clipped_end = (sx + dx * t1, sy + dy * t1)
    if _distance(clipped_start, clipped_end) < EPSILON:
        return None
    return clipped_start, clipped_end


def _add_perimeter_edges(
    unique_edges: dict[tuple[Point, Point], float],
    sites: tuple[Point, ...],
    bounds: Bounds,
    *,
    min_clearance_mm: float,
    obstacles: tuple[VoronoiObstacle, ...],
) -> None:
    """Add a safe rectangular corridor just inside the navigation bounds."""
    try:
        corridor = _inset_bounds(bounds, min_clearance_mm)
    except ValueError:
        return

    x_min, x_max, y_min, y_max = corridor
    points = (
        (x_min, y_min),
        (x_max, y_min),
        (x_max, y_max),
        (x_min, y_max),
    )
    for index, start in enumerate(points):
        end = points[(index + 1) % len(points)]
        clearance = edge_clearance_mm(start, end, sites, bounds)
        clearance = min(clearance, obstacle_clearance_mm(start, end, obstacles))
        if clearance + EPSILON < min_clearance_mm:
            continue
        key = _segment_key(start, end)
        unique_edges[key] = max(clearance, unique_edges.get(key, 0.0))

def _node_id(
    point: Point,
    node_ids: dict[Point, int],
    nodes: list[MapNode],
) -> int:
    point = _rounded_point(point)
    node_id = node_ids.get(point)
    if node_id is None:
        node_id = len(nodes)
        node_ids[point] = node_id
        nodes.append(MapNode(node_id, point[0], point[1], kind="voronoi"))
    return node_id


def _segment_key(start: Point, end: Point) -> tuple[Point, Point]:
    a = _rounded_point(start)
    b = _rounded_point(end)
    return (a, b) if a <= b else (b, a)


def _rounded_point(point: Point) -> Point:
    return (round(point[0], 6), round(point[1], 6))


def _dedupe_polygon(points: list[Point]) -> list[Point]:
    clean = []
    for point in points:
        rounded = _rounded_point(point)
        if not clean or rounded != clean[-1]:
            clean.append(rounded)
    if len(clean) > 1 and clean[0] == clean[-1]:
        clean.pop()
    return clean


def _distance(start: Point, end: Point) -> float:
    return hypot(end[0] - start[0], end[1] - start[1])


def _edge_cost(
    start: Point,
    end: Point,
    *,
    clearance: float,
    min_clearance_mm: float,
    obstacle_cost_weight: float,
    obstacle_risk: float = 0.0,
) -> float:
    length = _distance(start, end)
    if obstacle_cost_weight <= 0 or clearance == float("inf"):
        return length
    clearance = max(clearance, EPSILON)
    risk = min_clearance_mm / clearance + obstacle_risk
    return length * (1.0 + obstacle_cost_weight * risk)


def _obstacle_proximity_risk(
    start: Point,
    end: Point,
    obstacles: tuple[VoronoiObstacle, ...],
    *,
    min_clearance_mm: float,
) -> float:
    """Return cumulative risk from every obstacle near this segment."""
    if not obstacles:
        return 0.0
    influence_mm = max(min_clearance_mm * 4.0, 1.0)
    risk = 0.0
    for obstacle in obstacles:
        clearance = (
            distance_2_segment(obstacle.pos_mm, start, end)
            - obstacle.radius_mm
        )
        if clearance <= 0.0:
            risk += 2.0
        elif clearance < influence_mm:
            risk += (influence_mm - clearance) / influence_mm
    return risk


def _inset_bounds(bounds: Bounds, inset_mm: float) -> Bounds:
    x_min, x_max, y_min, y_max = bounds
    if inset_mm * 2.0 >= min(x_max - x_min, y_max - y_min):
        raise ValueError("boundary_inset_mm is too large for the field")
    return (
        x_min + inset_mm,
        x_max - inset_mm,
        y_min + inset_mm,
        y_max - inset_mm,
    )
