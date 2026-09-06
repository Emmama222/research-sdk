import pytest

from research_sdk.planners.common import Obstacle, PlanRequest, StepRecorder
from research_sdk.planners.VisibilityGraph.visibility_graph import plan


def test_direct_path_when_clear():
    request = PlanRequest(start_mm=(-1000.0, 0.0), goal_mm=(1000.0, 0.0), obstacles=())
    result = plan(request)
    assert result.success
    assert result.waypoints_mm == ((-1000.0, 0.0), (1000.0, 0.0))
    assert "skipped" in result.message


def test_skip_direct_path_forces_full_graph_build_even_when_clear():
    """Same clear-field request as test_direct_path_when_clear, but with
    skip_direct_path=True -- used by the offline planner comparison
    (scripts/demo_planners.py) so every trial measures the algorithm's
    actual graph-construction cost, not the trivial straight-line case."""
    request = PlanRequest(start_mm=(-1000.0, 0.0), goal_mm=(1000.0, 0.0), obstacles=())
    result = plan(request, skip_direct_path=True)
    assert result.success
    assert "skipped" not in result.message
    assert result.nodes_expanded == 2, "no obstacles means the full-build graph still has only start+goal"


def test_routes_around_single_obstacle():
    obstacle = Obstacle(pos_mm=(0.0, 0.0), radius_mm=200.0, robot_id=1)
    request = PlanRequest(
        start_mm=(-1500.0, 0.0),
        goal_mm=(1500.0, 0.0),
        obstacles=(obstacle,),
        robot_radius_mm=90.0,
        clearance_mm=30.0,
    )
    result = plan(request)
    assert result.success
    assert len(result.waypoints_mm) >= 3, "should detour via at least one polygon vertex"

    total_clearance = request.total_clearance_mm
    inflated_radius = obstacle.radius_mm + total_clearance
    for (x0, y0), (x1, y1) in zip(result.waypoints_mm, result.waypoints_mm[1:]):
        for t in [i / 20 for i in range(21)]:
            px = x0 + (x1 - x0) * t
            py = y0 + (y1 - y0) * t
            dist = ((px - obstacle.pos_mm[0]) ** 2 + (py - obstacle.pos_mm[1]) ** 2) ** 0.5
            # The default hexagon is circumscribed, so its edges retain the
            # complete circular safety clearance rather than cutting inside it.
            assert dist >= inflated_radius - 1e-6, "path enters inflated obstacle"


def test_default_obstacle_approximation_is_a_hexagon():
    obstacle = Obstacle(pos_mm=(0.0, 0.0), radius_mm=200.0)
    request = PlanRequest(
        start_mm=(-1000.0, 0.0),
        goal_mm=(1000.0, 0.0),
        obstacles=(obstacle,),
    )
    recorder = StepRecorder()

    plan(request, record=recorder)

    obstacle_step = next(step for step in recorder.steps if step["kind"] == "obstacles")
    assert len(obstacle_step["polygons"][0]) == 6


def test_polygon_requires_at_least_three_sides():
    request = PlanRequest(
        start_mm=(-1000.0, 0.0),
        goal_mm=(1000.0, 0.0),
        obstacles=(Obstacle(pos_mm=(0.0, 0.0), radius_mm=200.0),),
    )

    with pytest.raises(ValueError, match="at least 3"):
        plan(request, polygon_sides=2)


def test_two_obstacles_forces_wider_detour():
    obstacles = (
        Obstacle(pos_mm=(-100.0, 150.0), radius_mm=150.0, robot_id=1),
        Obstacle(pos_mm=(100.0, -150.0), radius_mm=150.0, robot_id=2),
    )
    request = PlanRequest(start_mm=(-1500.0, 0.0), goal_mm=(1500.0, 0.0), obstacles=obstacles)
    result = plan(request)
    assert result.success


def test_start_equals_goal_returns_trivial_path():
    request = PlanRequest(start_mm=(0.0, 0.0), goal_mm=(0.0, 0.0), obstacles=())
    result = plan(request)
    assert result.success
    assert result.path_length_mm == 0.0


def test_start_inside_obstacle_fails_cleanly():
    obstacle = Obstacle(pos_mm=(0.0, 0.0), radius_mm=300.0, robot_id=1)
    request = PlanRequest(start_mm=(0.0, 0.0), goal_mm=(1000.0, 0.0), obstacles=(obstacle,))
    result = plan(request)
    assert not result.success
    assert "inflated obstacle" in result.message


def test_direct_path_grazing_polygon_vertex_region_is_rejected():
    """Regression test for a broad-phase bug: the visibility check's
    bounding-circle rejection used the polygon's apothem (inradius) instead
    of its true circumradius (vertex radius). A straight line whose closest
    approach to the obstacle centre falls between those two values can still
    cross the polygon near a vertex, but was wrongly skipped by the broad
    phase and reported as clear -- letting the direct-path shortcut return a
    path straight through the obstacle."""
    obstacle = Obstacle(pos_mm=(0.0, 0.0), radius_mm=100.0, robot_id=1)
    request = PlanRequest(
        start_mm=(140.0, -2000.0),
        goal_mm=(140.0, 2000.0),
        obstacles=(obstacle,),
    )
    result = plan(request, polygon_sides=6)
    assert result.success
    assert "direct" not in result.message, "direct line crosses the obstacle and must not be taken"

    total_clearance = request.total_clearance_mm
    inflated_radius = obstacle.radius_mm + total_clearance
    for (x0, y0), (x1, y1) in zip(result.waypoints_mm, result.waypoints_mm[1:]):
        for t in [i / 20 for i in range(21)]:
            px = x0 + (x1 - x0) * t
            py = y0 + (y1 - y0) * t
            dist = ((px - obstacle.pos_mm[0]) ** 2 + (py - obstacle.pos_mm[1]) ** 2) ** 0.5
            assert dist >= inflated_radius - 1e-6, "path enters inflated obstacle"


def test_polygon_boundary_vertices_stay_connected():
    """Regression test for the adjacency bug caught during implementation:
    adjacent vertices of the same inflated polygon must connect (they form
    that polygon's own boundary), not get dropped by the floating-point
    midpoint-on-boundary ambiguity in the interior test."""
    obstacle = Obstacle(pos_mm=(0.0, 0.0), radius_mm=250.0, robot_id=1)
    request = PlanRequest(start_mm=(-2000.0, 0.0), goal_mm=(2000.0, 5.0), obstacles=(obstacle,))
    result = plan(request, polygon_sides=16)
    assert result.success, "boundary-hugging path around the obstacle must exist"
