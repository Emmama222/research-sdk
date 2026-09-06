from research_sdk.planners.Dijkstra.voronoi_dijkstra import VoronoiDijkstraPlanner
from research_sdk.world.scene import PlanningObstacle, PlanningScene


def test_direct_path_when_clear():
    scene = PlanningScene(timestamp=0.0, obstacles=())
    result = VoronoiDijkstraPlanner().plan(scene, (-1000.0, 0.0), (1000.0, 0.0))
    assert result.used_direct_path
    assert result.waypoints_mm == ()


def test_skip_direct_path_forces_full_map_build_even_when_clear():
    """Same clear-field query as test_direct_path_when_clear, but with
    skip_direct_path=True -- used by the offline planner comparison
    (scripts/demo_planners.py) so every trial measures the actual
    density-grid Voronoi map cost, not the trivial straight-line case."""
    scene = PlanningScene(timestamp=0.0, obstacles=())
    result = VoronoiDijkstraPlanner().plan(
        scene, (-1000.0, 0.0), (1000.0, 0.0), skip_direct_path=True
    )
    assert not result.used_direct_path
    assert result.waypoints_mm != ()


def test_routes_around_single_obstacle():
    obstacle = PlanningObstacle(robot_id=1, isYellow=True, pos_mm=(0.0, 0.0), radius_mm=200.0)
    scene = PlanningScene(timestamp=0.0, obstacles=(obstacle,))
    result = VoronoiDijkstraPlanner().plan(scene, (-1500.0, 0.0), (1500.0, 0.0))
    assert not result.used_direct_path
    assert result.waypoints_mm != ()
