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
    assert manifest["run_count"] == 2 * 2 * 3
    assert manifest["replan_policies"] == ["cycle", "event", "once"]
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
