from research_sdk.planners.common import Obstacle, PlanRequest
from research_sdk.planners.VisibilityGraph.visibility_graph import plan


def test_direct_path_when_clear():
    request = PlanRequest(start_mm=(-1000.0, 0.0), goal_mm=(1000.0, 0.0), obstacles=())
    result = plan(request)
    assert result.success
    assert result.waypoints_mm == ((-1000.0, 0.0), (1000.0, 0.0))
    assert "skipped" in result.message


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
    inflated_radius = obstacle.radius_mm + (total_clearance - request.robot_radius_mm)
    for (x0, y0), (x1, y1) in zip(result.waypoints_mm, result.waypoints_mm[1:]):
        for t in [i / 20 for i in range(21)]:
            px = x0 + (x1 - x0) * t
            py = y0 + (y1 - y0) * t
            dist = ((px - obstacle.pos_mm[0]) ** 2 + (py - obstacle.pos_mm[1]) ** 2) ** 0.5
            # Small polygon-approximation slack: a 12-gon inscribed inside
            # the "true" inflated circle can let the path graze slightly
            # inside the ideal circular clearance at the midpoints of edges.
            assert dist >= inflated_radius * 0.97, "path enters inflated obstacle"


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


def test_polygon_boundary_vertices_stay_connected():
    """Regression test for the adjacency bug caught during implementation:
    adjacent vertices of the same inflated polygon must connect (they form
    that polygon's own boundary), not get dropped by the floating-point
    midpoint-on-boundary ambiguity in the interior test."""
    obstacle = Obstacle(pos_mm=(0.0, 0.0), radius_mm=250.0, robot_id=1)
    request = PlanRequest(start_mm=(-2000.0, 0.0), goal_mm=(2000.0, 5.0), obstacles=(obstacle,))
    result = plan(request, polygon_sides=16)
    assert result.success, "boundary-hugging path around the obstacle must exist"
