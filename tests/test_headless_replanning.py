import json

import pytest

from research_sdk.headless import (
    PLANNER_NAMES,
    SimulationConfig,
    _obstacle_position,
    main,
    run_experiments,
    simulate,
    summarize_results,
)
from research_sdk.scenario_generator import GeneratorConfig, perturb_scenario, random_scenario
from research_sdk.ui.scenarios import Scenario, ScenarioObstacle, ScenarioRobot
from research_sdk.world.scene import FieldDimensions


def _crossing_scenario() -> Scenario:
    """Two robots whose straight paths cross at the origin at the same time.

    Each direct line is clear at t=0 (the other robot is still at its start),
    so an open-loop plan collides; replanning sees the other robot arrive.
    """
    return Scenario(
        "crossing",
        robots=[
            ScenarioRobot(0, False, (-2500.0, 0.0), (2500.0, 0.0)),
            ScenarioRobot(1, False, (0.0, -2500.0), (0.0, 2500.0)),
        ],
    )


def _config(policy: str) -> SimulationConfig:
    return SimulationConfig(max_simulation_s=20.0, replan_policy=policy)


@pytest.mark.parametrize("planner_name", PLANNER_NAMES)
def test_once_policy_plans_exactly_once(planner_name: str) -> None:
    result = simulate(_crossing_scenario(), planner_name, config=_config("once"))
    assert result.planner_calls == 2  # one call per robot
    assert result.replan_count == 0
    assert result.replan_policy == "once"


@pytest.mark.parametrize("planner_name", PLANNER_NAMES)
def test_event_policy_replans_only_when_blocked(planner_name: str) -> None:
    event = simulate(_crossing_scenario(), planner_name, config=_config("event"))
    cycle = simulate(_crossing_scenario(), planner_name, config=_config("cycle"))

    assert event.completed and cycle.completed
    assert event.planner_calls > 1
    # The gate is consulted every cycle but rebuilds far less often than per-tick.
    assert event.replan_count >= 1
    assert event.replan_count < cycle.replan_count


def test_replanning_avoids_the_crossing_obstacle() -> None:
    once = simulate(_crossing_scenario(), "voronoi", config=_config("once"))
    event = simulate(_crossing_scenario(), "voronoi", config=_config("event"))
    assert once.collision_episodes == 1
    assert event.replan_count >= 1
    assert event.collision_episodes == 0


def test_moving_obstacles_bounce_inside_the_field() -> None:
    obstacle = ScenarioObstacle(0, True, (0.0, 0.0), 90.0, (1500.0, 900.0))
    x_min, x_max, y_min, y_max = FieldDimensions().bounds_mm
    for step in range(0, 600):
        x, y = _obstacle_position(obstacle, step * 0.05)
        assert x_min + 90.0 - 1e-6 <= x <= x_max - 90.0 + 1e-6
        assert y_min + 90.0 - 1e-6 <= y <= y_max - 90.0 + 1e-6


def test_random_scenarios_are_reproducible_and_valid() -> None:
    config = GeneratorConfig(robots=3, obstacles=10, moving_fraction=0.3)
    first = random_scenario(7, seed=3, config=config)
    assert first.to_dict() == random_scenario(7, seed=3, config=config).to_dict()
    assert first.to_dict() != random_scenario(8, seed=3, config=config).to_dict()
    first.require_complete()

    keys = [(r.is_yellow, r.robot_id) for r in first.robots]
    keys += [(o.is_yellow, o.obstacle_id) for o in first.obstacles]
    assert len(keys) == len(set(keys))
    assert sum(o.velocity_mmps != (0.0, 0.0) for o in first.obstacles) == 3
    for robot in first.robots:
        dx = robot.target_mm[0] - robot.start_mm[0]
        dy = robot.target_mm[1] - robot.start_mm[1]
        assert (dx * dx + dy * dy) ** 0.5 >= config.min_travel_mm
        for obstacle in first.obstacles:
            ox, oy = obstacle.position_mm
            sx, sy = robot.start_mm
            assert ((ox - sx) ** 2 + (oy - sy) ** 2) ** 0.5 >= 180.0


def test_perturbed_scenario_keeps_structure() -> None:
    base = _crossing_scenario()
    base.obstacles.append(ScenarioObstacle(3, True, (1000.0, 1000.0), 90.0, (0.0, -500.0)))
    variant = perturb_scenario(base, 0, seed=1, config=GeneratorConfig(jitter_mm=100.0))
    assert variant.name == "crossing~p000"
    assert [r.robot_id for r in variant.robots] == [0, 1]
    assert variant.obstacles[0].velocity_mmps == (0.0, -500.0)
    moved = variant.robots[0].start_mm
    assert ((moved[0] + 2500.0) ** 2 + moved[1] ** 2) ** 0.5 <= 100.0 + 1e-6


def test_parallel_batch_matches_serial_and_groups_by_set() -> None:
    scenarios = [random_scenario(i, seed=5) for i in range(3)]
    kwargs = dict(
        policies=["once", "event"],
        config=SimulationConfig(max_simulation_s=15.0),
        scenario_sets=["random"] * 3,
    )
    serial = run_experiments(scenarios, ["visibility"], workers=1, **kwargs)
    parallel = run_experiments(scenarios, ["visibility"], workers=2, **kwargs)

    def outcome(r):
        return (r.scenario, r.replan_policy, r.completed, r.collision_episodes, r.replan_count)

    assert [outcome(r) for r in serial] == [outcome(r) for r in parallel]
    summary = summarize_results(serial)
    assert {(row["scenario"], row["replan_policy"]) for row in summary} == {
        ("random", "once"),
        ("random", "event"),
    }
    assert all(row["scenario_count"] == 3 for row in summary)


def test_grsim_backend_rejects_replanning_policies() -> None:
    with pytest.raises(ValueError, match="only --policy once"):
        run_experiments([_crossing_scenario()], ["voronoi"], backend="grsim", policies=["event"])


def test_cli_random_matrix_writes_scenarios_and_manifest(tmp_path) -> None:
    output = tmp_path / "batch"
    exit_code = main(
        [
            "--random", "2",
            "--planner", "visibility", "prm",
            "--policy", "all",
            "--max-sim-seconds", "15",
            "--output-dir", str(output),
        ]
    )
    assert exit_code == 0
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["run_count"] == 2 * 2 * 4
    assert manifest["replan_policies"] == ["cycle", "event", "event_route", "once"]
    assert manifest["scenario_sets"] == ["random"]
    assert len(list((output / "scenarios").glob("*.json"))) == 2


def test_prediction_moves_and_grows_moving_obstacles() -> None:
    from research_sdk.headless import _predicted_obstacle

    still = _predicted_obstacle(1, True, (0.0, 0.0), 90.0, (0.0, 0.0), 250.0)
    moving = _predicted_obstacle(1, True, (0.0, 0.0), 90.0, (1000.0, 0.0), 250.0)
    assert still.pos_mm == (0.0, 0.0) and still.radius_mm == 90.0
    assert moving.pos_mm == pytest.approx((250.0, 0.0))
    assert moving.radius_mm == pytest.approx(340.0)
    # The grown circle still covers the obstacle's current position.
    assert moving.radius_mm >= 250.0 + 90.0


def test_prediction_axis_runs_and_skips_redundant_plan_once() -> None:
    results = run_experiments(
        [_crossing_scenario()],
        ["voronoi"],
        policies=["once", "event"],
        predictions_ms=[0.0, 250.0],
        config=SimulationConfig(max_simulation_s=20.0),
    )
    labels = sorted((r.replan_policy, r.prediction_horizon_ms) for r in results)
    assert labels == [("event", 0.0), ("event", 250.0), ("once", 0.0)]
    summary = summarize_results(results)
    assert {row["replan_policy"] for row in summary} == {"once", "event", "event+pred250"}
    # Robots stop inside the 60 mm final tolerance, so efficiency can exceed 1 slightly.
    assert all(0.0 < row["path_efficiency_mean"] <= 1.05 for row in summary)


def test_prediction_is_rejected_on_grsim_backend() -> None:
    with pytest.raises(ValueError, match="only --policy once"):
        run_experiments(
            [_crossing_scenario()], ["voronoi"], backend="grsim", predictions_ms=[250.0]
        )


def test_patrol_obstacle_goes_back_and_forth_along_spawn_and_waypoints() -> None:
    from research_sdk.headless import _obstacle_velocity

    obstacle = ScenarioObstacle(
        0, True, (0.0, 0.0), 90.0, patrol_waypoints=((1000.0, 0.0), (1000.0, 500.0)),
        patrol_speed_mmps=500.0,
    )
    # route length 1500 mm at 500 mm/s: out in 3 s, back in 3 s
    assert _obstacle_position(obstacle, 0.0) == pytest.approx((0.0, 0.0))
    assert _obstacle_position(obstacle, 1.0) == pytest.approx((500.0, 0.0))
    assert _obstacle_position(obstacle, 2.5) == pytest.approx((1000.0, 250.0))
    assert _obstacle_velocity(obstacle, 2.5) == pytest.approx((0.0, 500.0))
    assert _obstacle_position(obstacle, 3.5) == pytest.approx((1000.0, 250.0))
    assert _obstacle_velocity(obstacle, 3.5) == pytest.approx((0.0, -500.0))
    assert _obstacle_position(obstacle, 6.0) == pytest.approx((0.0, 0.0))
    assert _obstacle_position(obstacle, 7.0) == pytest.approx((500.0, 0.0))


def test_patrol_scenarios_round_trip_through_json() -> None:
    scenario = Scenario(
        "patrol",
        robots=[ScenarioRobot(0, False, (-2000.0, 0.0), (2000.0, 0.0))],
        obstacles=[ScenarioObstacle(1, True, (0.0, 1500.0), 90.0,
                                    patrol_waypoints=((0.0, -1500.0),), patrol_speed_mmps=900.0)],
    )
    loaded = Scenario.from_dict(json.loads(json.dumps(scenario.to_dict())))
    assert loaded.obstacles[0].patrol_waypoints == ((0.0, -1500.0),)
    assert loaded.obstacles[0].patrol_speed_mmps == 900.0
    # an old file without the new keys still loads
    legacy = scenario.to_dict()
    del legacy["obstacles"][0]["patrol_waypoints"], legacy["obstacles"][0]["patrol_speed_mmps"]
    assert Scenario.from_dict(legacy).obstacles[0].patrol_waypoints == ()


def test_event_replanning_reacts_to_a_patrolling_obstacle() -> None:
    scenario = Scenario(
        "patrol-cross",
        robots=[ScenarioRobot(0, False, (-2500.0, 0.0), (2500.0, 0.0))],
        obstacles=[ScenarioObstacle(1, True, (0.0, 1500.0), 90.0,
                                    patrol_waypoints=((0.0, -1500.0),), patrol_speed_mmps=1500.0)],
    )
    event = simulate(scenario, "voronoi", config=_config("event"))
    assert event.completed
    assert event.planner_calls > 1


def test_random_patrol_generation_is_seeded() -> None:
    config = GeneratorConfig(obstacles=8, moving_fraction=1.0, patrol_fraction=1.0, patrol_points=2)
    first = random_scenario(3, seed=9, config=config)
    assert first.to_dict() == random_scenario(3, seed=9, config=config).to_dict()
    assert all(len(o.patrol_waypoints) == 2 for o in first.obstacles)
    assert all(
        config.min_obstacle_speed_mmps <= o.patrol_speed_mmps <= config.max_obstacle_speed_mmps
        for o in first.obstacles
    )
    variant = perturb_scenario(first, 0, seed=1)
    assert [len(o.patrol_waypoints) for o in variant.obstacles] == [2] * 8


def test_replan_period_and_prediction_are_sweep_axes() -> None:
    results = run_experiments(
        [_crossing_scenario()],
        ["prm"],
        policies=["once", "event"],
        predictions_ms=[0.0, 100.0],
        replan_periods_ms=[20.0, 100.0],
        config=SimulationConfig(max_simulation_s=20.0),
    )
    combos = sorted((r.replan_policy, r.prediction_horizon_ms, r.replan_period_ms) for r in results)
    assert combos == [
        ("event", 0.0, 20.0), ("event", 0.0, 100.0),
        ("event", 100.0, 20.0), ("event", 100.0, 100.0),
        ("once", 0.0, 20.0),
    ]
    slow = next(r for r in results if r.replan_policy == "event" and r.replan_period_ms == 100.0
                and r.prediction_horizon_ms == 0.0)
    fast = next(r for r in results if r.replan_policy == "event" and r.replan_period_ms == 20.0
                and r.prediction_horizon_ms == 0.0)
    assert slow.planner_calls < fast.planner_calls
    rows = summarize_results(results)
    assert {(row["prediction_horizon_ms"], row["replan_period_ms"]) for row in rows} == {
        (0.0, 20.0), (0.0, 100.0), (100.0, 20.0), (100.0, 100.0)
    }


def test_invalid_replan_period_is_rejected() -> None:
    with pytest.raises(ValueError, match="replan periods"):
        run_experiments([_crossing_scenario()], ["prm"], replan_periods_ms=[0.0])


def test_planning_time_is_split_into_rebuilds_and_checks() -> None:
    event = simulate(_crossing_scenario(), "voronoi", config=_config("event"))
    assert event.rebuild_calls == event.robot_count + event.replan_count
    assert event.check_calls == event.planner_calls - event.rebuild_calls
    assert event.planning_time_ms_rebuild + event.planning_time_ms_check == pytest.approx(
        event.planning_time_ms_total
    )
    reasons = (
        event.replans_active_blocked + event.replans_route_finished
        + event.replans_route_blocked + event.replans_other + event.replans_scheduled
    )
    assert reasons == event.replan_count
    assert event.replans_scheduled == 0

    cycle = simulate(_crossing_scenario(), "prm", config=_config("cycle"))
    assert cycle.replans_scheduled == cycle.replan_count


def test_motion_quality_and_split_clearance_metrics() -> None:
    straight = simulate(
        Scenario("straight", robots=[ScenarioRobot(0, True, (-1000.0, 0.0), (1000.0, 0.0))]),
        "visibility",
        config=_config("once"),
    )
    assert straight.heading_change_rad_per_m == pytest.approx(0.0, abs=1e-9)
    assert straight.sharp_turns == 0
    assert straight.minimum_robot_clearance_mm is None
    assert straight.minimum_obstacle_clearance_mm is None
    assert straight.contact_time_ms == 0.0

    once = simulate(_crossing_scenario(), "voronoi", config=_config("once"))
    assert once.minimum_robot_clearance_mm < 0  # the two robots collide head-on
    assert once.contact_time_ms > 0
    assert once.minimum_obstacle_clearance_mm is None


def test_route_shift_measures_how_far_a_rebuild_moved_the_route() -> None:
    from research_sdk.headless import _route_shift

    old = ((0.0, 0.0), (2000.0, 0.0))
    assert _route_shift(old, old) == pytest.approx(0.0)
    assert _route_shift(old, ((0.0, 100.0), (2000.0, 100.0))) == pytest.approx(100.0)


def test_summary_reports_collision_probability_with_confidence_interval() -> None:
    results = run_experiments(
        [_crossing_scenario()], ["voronoi"], policies=["once", "event"],
        config=SimulationConfig(max_simulation_s=20.0),
    )
    rows = {row["replan_policy"]: row for row in summarize_results(results)}
    assert rows["once"]["collision_probability"] == 1.0
    low, high = map(float, rows["once"]["collision_probability_ci95"].split("-"))
    assert 0.0 < low < 1.0 == high
    assert rows["event"]["collision_probability"] == 0.0


def test_full_route_check_triggers_on_a_blocked_later_segment() -> None:
    from research_sdk.planners.reroute import RouteState, evaluate_route
    from research_sdk.world.scene import PlanningObstacle, PlanningScene

    # Route: (0,0) -> (1000,0) -> (1000,1000) -> target (2000,1000). The only
    # obstacle sits on the second segment (and on the direct line), not on the
    # active segment to the first waypoint.
    state = RouteState(
        last_target_pose=(2000.0, 1000.0, 0.0),
        waypoints=((1000.0, 0.0, 0.0), (1000.0, 1000.0, 0.0)),
    )
    scene = PlanningScene(0.0, (PlanningObstacle(4, True, (1000.0, 500.0), 90.0),))
    start, target = (0.0, 0.0), (2000.0, 1000.0)
    assert not scene.is_path_free(start, target)
    active_only = evaluate_route(scene, start, target, state, periodic_reroute_frames=None)
    full = evaluate_route(
        scene, start, target, state, periodic_reroute_frames=None, check_full_route=True
    )
    assert not active_only.need_reroute
    assert full.need_reroute


@pytest.mark.parametrize("planner_name", PLANNER_NAMES)
def test_event_route_policy_runs_for_every_planner(planner_name: str) -> None:
    result = simulate(_crossing_scenario(), planner_name, config=_config("event_route"))
    assert result.completed
    assert result.replan_policy == "event_route"
