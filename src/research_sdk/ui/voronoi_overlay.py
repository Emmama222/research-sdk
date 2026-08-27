"""Realtime render-layer helpers for the bounded Voronoi map."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

from research_sdk.config import (
    VORONOI_HORIZON_MS,
    VORONOI_OBSTACLE_COST_WEIGHT,
    VORONOI_RENDER_DENSITY_PERCENT,
    VORONOI_RENDER_MAX_DENSITY_NODES,
)
from research_sdk.ui.renderer import RenderLayer
from research_sdk.world.map.voronoi.voronoi_generator import (
    BoundedVoronoiMap,
    generate_voronoi_map_from_world_map,
)


@dataclass(frozen=True, slots=True)
class VoronoiOverlay:
    layer: RenderLayer
    voronoi_map: BoundedVoronoiMap
    generation_ms: float


def build_voronoi_overlay(
    world_map,
    *,
    now_s: float | None = None,
    horizon_ms: int | float = VORONOI_HORIZON_MS,
    density_percent: float = VORONOI_RENDER_DENSITY_PERCENT,
    max_density_nodes: int = VORONOI_RENDER_MAX_DENSITY_NODES,
    obstacle_cost_weight: float = VORONOI_OBSTACLE_COST_WEIGHT,
    ignore_robots: set[tuple[bool, int]] | None = None,
) -> VoronoiOverlay:
    """Build a hidden-by-default Voronoi render layer and measure its cost."""
    start_s = perf_counter()
    voronoi_map = generate_voronoi_map_from_world_map(
        world_map,
        now_s=now_s,
        horizon_ms=horizon_ms,
        ignore_robots=ignore_robots,
        placement_mode="density_grid",
        density_percent=density_percent,
        max_density_nodes=max_density_nodes,
        obstacle_cost_weight=obstacle_cost_weight,
    )
    generation_ms = (perf_counter() - start_s) * 1000.0
    return VoronoiOverlay(
        layer=voronoi_map.render_layer(
            "Voronoi map",
            visible_by_default=False,
        ),
        voronoi_map=voronoi_map,
        generation_ms=generation_ms,
    )
