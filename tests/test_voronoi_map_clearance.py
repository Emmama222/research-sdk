from math import hypot

import pytest

from research_sdk.config import FIELD_LENGTH_MM, FIELD_WIDTH_MM
from research_sdk.world.map.geometry import distance_2_segment
from research_sdk.world.map.voronoi.voronoi_generator import (
    VoronoiObstacle,
    generate_bounded_voronoi_map,
    generate_voronoi_map_from_scene,
)
from research_sdk.world.scene import FieldDimensions, PlanningObstacle, PlanningScene


def _node_positions(voronoi_map) -> dict[int, tuple[float, float]]:
    return {node.id: (node.x, node.y) for node in voronoi_map.nodes}


def test_default_field_and_navigation_bounds_come_from_configuration() -> None:
    boundary_inset = 100.0
    voronoi_map = generate_bounded_voronoi_map(
        virtual_node_count=8,
        placement_mode="random",
        boundary_inset_mm=boundary_inset,
        seed=7,
    )

    assert voronoi_map.field_bounds_mm == (
        -FIELD_LENGTH_MM / 2,
        FIELD_LENGTH_MM / 2,
        -FIELD_WIDTH_MM / 2,
        FIELD_WIDTH_MM / 2,
    )
    assert voronoi_map.bounds_mm == (
        -FIELD_LENGTH_MM / 2 + boundary_inset,
        FIELD_LENGTH_MM / 2 - boundary_inset,
        -FIELD_WIDTH_MM / 2 + boundary_inset,
        FIELD_WIDTH_MM / 2 - boundary_inset,
    )


def test_random_site_generation_is_deterministic_for_a_seed() -> None:
    first = generate_bounded_voronoi_map(
        virtual_node_count=12,
        placement_mode="random",
        seed=42,
    )
    second = generate_bounded_voronoi_map(
        virtual_node_count=12,
        placement_mode="random",
        seed=42,
    )

    assert first.virtual_sites_mm == second.virtual_sites_mm
    assert first.nodes == second.nodes
    assert first.edges == second.edges


def test_every_navigation_edge_respects_boundary_and_obstacle_clearance() -> None:
    clearance = 50.0
    obstacle = VoronoiObstacle(pos_mm=(0.0, 0.0), radius_mm=150.0)
    voronoi_map = generate_bounded_voronoi_map(
        virtual_node_count=12,
        placement_mode="random",
        field_length_mm=2000.0,
        field_width_mm=1200.0,
        boundary_inset_mm=100.0,
        min_clearance_mm=clearance,
        obstacles=(obstacle,),
        seed=3,
    )
    positions = _node_positions(voronoi_map)
    clipped_x_min, clipped_x_max, clipped_y_min, clipped_y_max = (
        voronoi_map.clipped_bounds_mm
    )

    assert voronoi_map.edges
    for edge in voronoi_map.edges:
        start = positions[edge.start_id]
        end = positions[edge.end_id]
        for x, y in (start, end):
            assert clipped_x_min - 1e-6 <= x <= clipped_x_max + 1e-6
            assert clipped_y_min - 1e-6 <= y <= clipped_y_max + 1e-6
        obstacle_clearance = (
            distance_2_segment(obstacle.pos_mm, start, end) - obstacle.radius_mm
        )
        assert obstacle_clearance >= clearance - 1e-6
        assert edge.clearance is None or edge.clearance >= clearance - 1e-6


def test_obstacle_cost_weight_never_reduces_an_edge_cost() -> None:
    kwargs = {
        "virtual_node_count": 12,
        "placement_mode": "random",
        "obstacles": (VoronoiObstacle((0.0, 0.0), 100.0),),
        "seed": 5,
    }
    unweighted = generate_bounded_voronoi_map(**kwargs, obstacle_cost_weight=0.0)
    weighted = generate_bounded_voronoi_map(**kwargs, obstacle_cost_weight=2.0)

    unweighted_costs = {
        (edge.start_id, edge.end_id): edge.cost for edge in unweighted.edges
    }
    weighted_costs = {(edge.start_id, edge.end_id): edge.cost for edge in weighted.edges}
    shared = unweighted_costs.keys() & weighted_costs.keys()

    assert shared
    assert all(weighted_costs[key] >= unweighted_costs[key] for key in shared)
    assert any(weighted_costs[key] > unweighted_costs[key] for key in shared)


def test_scene_generation_uses_scene_dimensions_and_robot_filtering() -> None:
    ignored = PlanningObstacle(1, True, (0.0, 0.0), 100.0)
    retained = PlanningObstacle(2, False, (400.0, 0.0), 120.0)
    scene = PlanningScene(
        timestamp=1.0,
        obstacles=(ignored, retained),
        field=FieldDimensions(3000.0, 2000.0),
    )

    voronoi_map = generate_voronoi_map_from_scene(
        scene,
        ignore_robots={(True, 1)},
        virtual_node_count=8,
        placement_mode="random",
        seed=9,
    )

    assert voronoi_map.field_bounds_mm == (-1500.0, 1500.0, -1000.0, 1000.0)
    assert len(voronoi_map.obstacles) == 1
    assert voronoi_map.obstacles[0].pos_mm == retained.pos_mm
    assert voronoi_map.obstacles[0].radius_mm == retained.radius_mm


def test_render_layer_contains_obstacle_and_navigation_geometry() -> None:
    voronoi_map = generate_bounded_voronoi_map(
        virtual_node_count=8,
        placement_mode="random",
        obstacles=(VoronoiObstacle((0.0, 0.0), 100.0, "blocker"),),
        seed=4,
    )

    layer = voronoi_map.render_layer("test map", visible_by_default=False)

    assert layer.name == "test map"
    assert not layer.visible_by_default
    assert layer.polylines
    assert any(circle.label == "blocker" for circle in layer.circles)


@pytest.mark.parametrize(
    "kwargs",
    (
        {"virtual_node_count": 1},
        {"field_length_mm": 0.0},
        {"field_width_mm": -1.0},
        {"min_clearance_mm": -1.0},
        {"boundary_inset_mm": -1.0},
        {"boundary_inset_mm": FIELD_WIDTH_MM / 2},
        {"placement_mode": "hexagonal"},
        {"obstacle_cost_weight": -0.1},
    ),
)
def test_invalid_generation_parameters_fail_fast(kwargs) -> None:
    with pytest.raises(ValueError):
        generate_bounded_voronoi_map(**kwargs)


def test_edge_cost_is_at_least_geometric_length() -> None:
    voronoi_map = generate_bounded_voronoi_map(
        virtual_node_count=10,
        placement_mode="random",
        obstacle_cost_weight=2.0,
        seed=11,
    )
    positions = _node_positions(voronoi_map)

    for edge in voronoi_map.edges:
        start = positions[edge.start_id]
        end = positions[edge.end_id]
        length = hypot(end[0] - start[0], end[1] - start[1])
        assert edge.cost >= length - 1e-6
