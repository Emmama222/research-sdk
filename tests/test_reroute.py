from research_sdk.planners.reroute import (
    RouteState,
    commit_reroute,
    evaluate_route,
    note_no_reroute,
)
from research_sdk.world.scene import PlanningObstacle, PlanningScene


def _scene(*obstacles: PlanningObstacle) -> PlanningScene:
    return PlanningScene(timestamp=0.0, obstacles=obstacles)


def _blocking_obstacle() -> PlanningObstacle:
    """Sits exactly on the (0,0)->(1000,0) line, well past the clearance radius."""
    return PlanningObstacle(robot_id=1, isYellow=False, pos_mm=(500.0, 0.0), radius_mm=150.0)


def test_clear_line_is_free_and_needs_no_reroute():
    decision = evaluate_route(_scene(), (0.0, 0.0), (1000.0, 0.0), RouteState())
    assert decision.is_path_free
    assert not decision.need_reroute


def test_first_call_blocked_with_no_cached_route_forces_reroute():
    decision = evaluate_route(_scene(_blocking_obstacle()), (0.0, 0.0), (1000.0, 0.0), RouteState())
    assert not decision.is_path_free
    assert decision.need_reroute  # route_finished: nothing cached yet


def test_cached_route_with_clear_active_segment_needs_no_reroute():
    """Direct line stays blocked, but the robot's already-active waypoint
    segment is untouched -- the cheap check should keep the cached route."""
    state = RouteState(
        waypoints=((0.0, 1000.0, 0.0), (1000.0, 0.0, 0.0)),
        last_target_pose=(1000.0, 0.0, 0.0),
    )
    decision = evaluate_route(
        _scene(_blocking_obstacle()),
        (0.0, 0.0),
        (1000.0, 0.0),
        state,
        periodic_reroute_frames=None,
    )
    assert not decision.is_path_free
    assert not decision.need_reroute


def test_obstacle_on_active_segment_forces_reroute():
    """Same cached route as above, but now an obstacle sits on the active
    segment itself -- this is the "obstacle entered the path" trigger."""
    state = RouteState(
        waypoints=((0.0, 1000.0, 0.0), (1000.0, 0.0, 0.0)),
        last_target_pose=(1000.0, 0.0, 0.0),
    )
    blocking_active_segment = PlanningObstacle(
        robot_id=2, isYellow=False, pos_mm=(0.0, 500.0), radius_mm=150.0
    )
    decision = evaluate_route(
        _scene(_blocking_obstacle(), blocking_active_segment),
        (0.0, 0.0),
        (1000.0, 0.0),
        state,
        periodic_reroute_frames=None,
    )
    assert not decision.is_path_free
    assert decision.need_reroute


def test_target_moved_past_deadzone_forces_reroute():
    state = RouteState(
        waypoints=((0.0, 1000.0, 0.0), (1000.0, 0.0, 0.0)),
        last_target_pose=(1000.0, 900.0, 0.0),
    )
    decision = evaluate_route(
        _scene(_blocking_obstacle()),
        (0.0, 0.0),
        (1000.0, 0.0),
        state,
        target_deadzone_mm=150.0,
        periodic_reroute_frames=None,
    )
    assert not decision.is_path_free
    assert decision.need_reroute  # target moved 900mm, past the 150mm dead zone


def test_periodic_cadence_forces_reroute_even_when_nothing_blocked():
    state = RouteState(
        waypoints=((0.0, 1000.0, 0.0), (1000.0, 0.0, 0.0)),
        last_target_pose=(1000.0, 0.0, 0.0),
        frames_since_reroute=3,
    )
    decision = evaluate_route(
        _scene(_blocking_obstacle()),
        (0.0, 0.0),
        (1000.0, 0.0),
        state,
        periodic_reroute_frames=3,
    )
    assert decision.need_reroute


def test_periodic_cadence_not_yet_due():
    state = RouteState(
        waypoints=((0.0, 1000.0, 0.0), (1000.0, 0.0, 0.0)),
        last_target_pose=(1000.0, 0.0, 0.0),
        frames_since_reroute=2,
    )
    decision = evaluate_route(
        _scene(_blocking_obstacle()),
        (0.0, 0.0),
        (1000.0, 0.0),
        state,
        periodic_reroute_frames=3,
    )
    assert not decision.need_reroute


def test_periodic_reroute_frames_none_disables_cadence():
    state = RouteState(
        waypoints=((0.0, 1000.0, 0.0), (1000.0, 0.0, 0.0)),
        last_target_pose=(1000.0, 0.0, 0.0),
        frames_since_reroute=999,
    )
    decision = evaluate_route(
        _scene(_blocking_obstacle()),
        (0.0, 0.0),
        (1000.0, 0.0),
        state,
        periodic_reroute_frames=None,
    )
    assert not decision.need_reroute


def test_commit_reroute_resets_cadence_and_updates_target_baseline():
    state = RouteState(last_target_pose=(0.0, 0.0, 0.0), frames_since_reroute=4)
    commit_reroute(state, ((100.0, 0.0, 0.0),), (1000.0, 0.0, 0.0))
    assert state.waypoints == ((100.0, 0.0, 0.0),)
    assert state.last_target_pose == (1000.0, 0.0, 0.0)
    assert state.frames_since_reroute == 0


def test_note_no_reroute_advances_cadence_without_touching_target_baseline():
    state = RouteState(last_target_pose=(0.0, 0.0, 0.0), frames_since_reroute=0)
    note_no_reroute(state)
    assert state.frames_since_reroute == 1
    assert state.last_target_pose == (0.0, 0.0, 0.0)
