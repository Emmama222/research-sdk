"""Background worker for debug world-map render generation."""

from __future__ import annotations

import time
from multiprocessing import Queue

from research_sdk.process_workers.worker import BaseWorker
from research_sdk.config import (
    BALL_R,
    FIELD_LENGTH_MM,
    FIELD_WIDTH_MM,
    VORONOI_OBSTACLE_COST_WEIGHT,
    VORONOI_RENDER_DENSITY_PERCENT,
    VORONOI_RENDER_MAX_DENSITY_NODES,
)
from research_sdk.ui.renderer import (
    BALL,
    BLUE,
    MapRenderData,
    PREDICTION,
    RenderCircle,
    RenderLayer,
    RenderPolyline,
    RenderRobot,
    RenderVector,
    VELOCITY,
    YELLOW,
)
from research_sdk.world.map.voronoi.voronoi_generator import generate_bounded_voronoi_map


class WorldMapRenderWorker(BaseWorker):
    """Generate complete debug map render data outside the UI/world processes."""

    def __init__(self, is_running, logger):
        super().__init__(is_running=is_running, logger=logger)
        self.delay_time = 0.005

    def setup(self, *args):
        self.request_q: Queue = args[0]
        self.response_q: Queue = args[1]
        self.logger.info("[world-map-render] setup completed")

    def step(self):
        request = self._latest_request()
        if request is None:
            time.sleep(self.delay_time)
            return

        started_s = time.perf_counter()
        try:
            render_data, voronoi_ms = _build_render_data(request)
            generation_ms = (time.perf_counter() - started_s) * 1000.0
            self.response_q.put(
                {
                    "render_data": render_data,
                    "generation_ms": generation_ms,
                    "voronoi_generation_ms": voronoi_ms,
                    "request_id": request.get("request_id", 0),
                }
            )
        except Exception as exc:
            self.response_q.put(
                {
                    "error": str(exc),
                    "request_id": request.get("request_id", 0),
                }
            )

    def _latest_request(self):
        latest = None
        while True:
            try:
                latest = self.request_q.get_nowait()
            except Exception:
                return latest


def _build_render_data(request: dict) -> tuple[MapRenderData, float | None]:
    obstacles = tuple(request.get("obstacles", ()))
    planning_obstacles = tuple(request.get("planning_obstacles", ()))
    ball = request.get("ball")
    ball_visible = bool(request.get("ball_visible", False))
    ball_vel_mmps = tuple(request.get("ball_vel_mmps", (0.0, 0.0)))
    planner_paths = tuple(request.get("planner_paths", ()))
    velocity_vector_seconds = float(request.get("velocity_vector_seconds", 0.25))

    robots = []
    velocity_vectors = []
    for obs in obstacles:
        color = YELLOW if obs.isYellow else BLUE
        center = (obs.pos_mm[0], obs.pos_mm[1])
        robots.append(
            RenderRobot(
                center_mm=center,
                orientation_rad=obs.pos_mm[2],
                color=color,
                label=str(obs.robot_id),
            )
        )
        velocity_vectors.append(
            RenderVector(
                start_mm=center,
                end_mm=(
                    center[0] + obs.vel_mmps[0] * velocity_vector_seconds,
                    center[1] + obs.vel_mmps[1] * velocity_vector_seconds,
                ),
                color=VELOCITY,
                label=f"{obs.speed_mmps:.0f} mm/s",
            )
        )

    predicted_circles = tuple(
        RenderCircle(
            center_mm=obs.pos_mm,
            radius_mm=obs.radius_mm,
            color=PREDICTION,
            label=str(getattr(obs, "robot_id", getattr(obs, "label", ""))),
        )
        for obs in planning_obstacles
    )

    ball_circles = ()
    ball_vectors = ()
    if ball is not None:
        ball_color = BALL if ball_visible else "#a86320"
        ball_circles = (
            RenderCircle(ball, BALL_R, ball_color, filled=True),
        )
        ball_vectors = (
            RenderVector(
                start_mm=ball,
                end_mm=(
                    ball[0] + ball_vel_mmps[0] * velocity_vector_seconds,
                    ball[1] + ball_vel_mmps[1] * velocity_vector_seconds,
                ),
                color=ball_color,
                label=f"{ball_vel_mmps}",
            ),
        )

    layers = [
        RenderLayer("Robots", robots=tuple(robots)),
        RenderLayer("Velocity vectors", vectors=tuple(velocity_vectors)),
        RenderLayer(
            "Predicted clearance",
            circles=predicted_circles,
            visible_by_default=False,
        ),
        RenderLayer("Ball", circles=ball_circles, vectors=ball_vectors),
    ]

    path_polylines = []
    field_bounds = _field_bounds_from_request(request)
    for path in planner_paths:
        points = tuple(tuple(point[:2]) for point in path.get("points", ()))
        if len(points) < 2:
            continue
        color = YELLOW if path.get("is_yellow", True) else BLUE
        for start, end in zip(points, points[1:]):
            clipped = _clip_segment_to_bounds(start, end, field_bounds)
            if clipped is None:
                continue
            path_polylines.append(
                RenderPolyline(
                    points_mm=clipped,
                    color=color,
                    closed=False,
                )
            )
    if path_polylines:
        layers.append(
            RenderLayer(
                "Planned paths",
                polylines=tuple(path_polylines),
                visible_by_default=True,
            )
        )

    voronoi_ms = None
    if request.get("include_voronoi", False):
        started_s = time.perf_counter()
        field_kwargs = _field_dimension_kwargs(request)
        voronoi_map = generate_bounded_voronoi_map(
            placement_mode="density_grid",
            density_percent=float(
                request.get("density_percent", VORONOI_RENDER_DENSITY_PERCENT)
            ),
            max_density_nodes=int(
                request.get("max_density_nodes", VORONOI_RENDER_MAX_DENSITY_NODES)
            ),
            obstacle_cost_weight=float(
                request.get("obstacle_cost_weight", VORONOI_OBSTACLE_COST_WEIGHT)
            ),
            obstacles=planning_obstacles,
            **field_kwargs,
        )
        voronoi_ms = (time.perf_counter() - started_s) * 1000.0
        layers.append(
            voronoi_map.render_layer(
                "Voronoi map",
                visible_by_default=False,
            )
        )

    return MapRenderData(layers=tuple(layers)), voronoi_ms


def _field_dimension_kwargs(request: dict) -> dict[str, float]:
    length = _positive_float(request.get("field_length_mm"))
    width = _positive_float(request.get("field_width_mm"))
    if length is None or width is None:
        return {}
    return {
        "field_length_mm": length,
        "field_width_mm": width,
    }


def _field_bounds_from_request(request: dict) -> tuple[float, float, float, float]:
    length = _positive_float(request.get("field_length_mm")) or float(FIELD_LENGTH_MM)
    width = _positive_float(request.get("field_width_mm")) or float(FIELD_WIDTH_MM)
    return (
        -length / 2.0,
        length / 2.0,
        -width / 2.0,
        width / 2.0,
    )


def _clip_segment_to_bounds(
    start: tuple[float, float],
    end: tuple[float, float],
    bounds: tuple[float, float, float, float],
) -> tuple[tuple[float, float], tuple[float, float]] | None:
    x_min, x_max, y_min, y_max = bounds
    sx, sy = float(start[0]), float(start[1])
    ex, ey = float(end[0]), float(end[1])
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
        if abs(p) < 1e-9:
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
    if clipped_start == clipped_end:
        return None
    return clipped_start, clipped_end


def _positive_float(value) -> float | None:
    if value is None:
        return None
    value = float(value)
    if value <= 0.0:
        return None
    return value


VoronoiMapWorker = WorldMapRenderWorker
