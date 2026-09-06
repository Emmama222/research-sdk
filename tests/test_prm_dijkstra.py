from research_sdk.planners.common import Obstacle, PlanRequest
from research_sdk.planners.PRM.prm_dijkstra import plan


def test_direct_path_when_clear():
    request = PlanRequest(start_mm=(-1000.0, 0.0), goal_mm=(1000.0, 0.0), obstacles=())
    result = plan(request, seed=1)
    assert result.success
    assert result.waypoints_mm == ((-1000.0, 0.0), (1000.0, 0.0))
    assert "skipped" in result.message


def test_skip_direct_path_forces_full_sampling_even_when_clear():
    """Same clear-field request as test_direct_path_when_clear, but with
    skip_direct_path=True -- used by the offline planner comparison
    (scripts/demo_planners.py) so every trial measures PRM's actual
    sampling/roadmap cost, not the trivial straight-line case."""
    request = PlanRequest(start_mm=(-1000.0, 0.0), goal_mm=(1000.0, 0.0), obstacles=())
    result = plan(request, seed=1, num_samples=20, skip_direct_path=True)
    assert result.success
    assert "skipped" not in result.message
    assert result.nodes_expanded > 2, "should have sampled milestones, not just start+goal"


def test_routes_around_single_obstacle():
    obstacle = Obstacle(pos_mm=(0.0, 0.0), radius_mm=200.0, robot_id=1)
    request = PlanRequest(
        start_mm=(-1500.0, 0.0),
        goal_mm=(1500.0, 0.0),
        obstacles=(obstacle,),
        robot_radius_mm=90.0,
        clearance_mm=30.0,
    )
    result = plan(request, seed=1, num_samples=40)
    assert result.success
    assert len(result.waypoints_mm) >= 2

    total_clearance = request.total_clearance_mm
    inflated_radius = obstacle.radius_mm + total_clearance
    for (x0, y0), (x1, y1) in zip(result.waypoints_mm, result.waypoints_mm[1:]):
        # sample along the segment and check clearance from the obstacle centre
        for t in [i / 20 for i in range(21)]:
            px = x0 + (x1 - x0) * t
            py = y0 + (y1 - y0) * t
            dist = ((px - obstacle.pos_mm[0]) ** 2 + (py - obstacle.pos_mm[1]) ** 2) ** 0.5
            assert dist >= inflated_radius - 1e-6, "path segment enters inflated obstacle"


def test_start_equals_goal_returns_trivial_path():
    request = PlanRequest(start_mm=(0.0, 0.0), goal_mm=(0.0, 0.0), obstacles=())
    result = plan(request, seed=1)
    assert result.success
    assert result.path_length_mm == 0.0


def test_start_inside_obstacle_fails_cleanly():
    obstacle = Obstacle(pos_mm=(0.0, 0.0), radius_mm=300.0, robot_id=1)
    request = PlanRequest(start_mm=(0.0, 0.0), goal_mm=(1000.0, 0.0), obstacles=(obstacle,))
    result = plan(request, seed=1)
    assert not result.success
    assert "inflated obstacle" in result.message


def test_fully_enclosed_goal_is_unreachable():
    # A goal sealed off is expected to fail rather than hang or crash.
    obstacle = Obstacle(pos_mm=(500.0, 0.0), radius_mm=100.0, robot_id=1)
    request = PlanRequest(
        start_mm=(-500.0, 0.0),
        goal_mm=(500.0, 0.0),
        obstacles=(obstacle,),
    )
    result = plan(request, seed=1)
    # Goal is inside the obstacle itself here, so this should fail via the
    # "inside an inflated obstacle" branch, not hang searching for a path.
    assert not result.success
