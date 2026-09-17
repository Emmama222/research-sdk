import csv
import json

import pytest

from research_sdk.headless import (
    PLANNER_NAMES,
    SimulationConfig,
    main,
    run_experiments,
    simulate,
    summarize_results,
    write_results,
)
from research_sdk.ui.scenarios import Scenario, ScenarioObstacle, ScenarioRobot


def _straight_scenario() -> Scenario:
    return Scenario(
        "straight",
        robots=[ScenarioRobot(0, True, (-1000.0, 0.0), (1000.0, 0.0))],
    )


@pytest.mark.parametrize("planner_name", PLANNER_NAMES)
def test_every_planner_runs_without_qt_or_grsim(planner_name: str) -> None:
    result = simulate(
        _straight_scenario(),
        planner_name,
        config=SimulationConfig(max_simulation_s=10.0, max_speed_mmps=1000.0),
    )

    assert result.completed
    assert result.status == "completed"
    assert result.planner_calls == 1
    assert result.failed_plans == 0
    assert result.simulated_duration_ms > 0
    assert result.wall_time_ms > 0
    assert result.max_final_error_mm <= 60.0
    assert result.planned_path_length_mm == pytest.approx(2000.0)


def test_timeout_is_reported_on_virtual_clock() -> None:
    result = simulate(
        _straight_scenario(),
        "visibility",
        config=SimulationConfig(max_simulation_s=0.1, max_speed_mmps=100.0),
    )

    assert not result.completed
    assert result.timed_out
    assert result.status == "timed_out"
    assert result.simulated_duration_ms == pytest.approx(100.0)


def test_collision_episode_is_not_counted_once_per_tick() -> None:
    scenario = Scenario(
        "overlap",
        robots=[ScenarioRobot(0, True, (0.0, 0.0), (0.0, 0.0))],
        obstacles=[ScenarioObstacle(1, False, (0.0, 0.0), 90.0)],
    )

    result = simulate(scenario, "visibility")

    assert result.completed
    assert result.obstacle_collision_episodes == 1
    assert result.collision_episodes == 1
    assert result.minimum_clearance_mm == pytest.approx(-180.0)


def test_batch_summary_and_exports_are_research_ready(tmp_path) -> None:
    config = SimulationConfig(max_simulation_s=10.0)
    results = run_experiments(
        [_straight_scenario()],
        ["visibility"],
        trials=2,
        seed=41,
        config=config,
    )

    assert [result.seed for result in results] == [41, 42]
    summary = summarize_results(results)
    assert summary[0]["runs"] == 2
    assert summary[0]["completion_rate"] == 1.0

    paths = write_results(results, tmp_path, config=config)
    assert set(paths) == {
        "runs_csv",
        "runs_json",
        "summary_csv",
        "summary_json",
        "manifest",
    }
    with paths["runs_csv"].open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 2
    assert rows[0]["scenario"] == "straight"
    manifest = json.loads(paths["manifest"].read_text(encoding="utf-8"))
    assert manifest["physics_equivalence"] is False
    assert manifest["run_count"] == 2


def test_cli_accepts_a_scenario_file_and_writes_results(tmp_path) -> None:
    scenario_path = tmp_path / "scenario.json"
    scenario_path.write_text(
        json.dumps(_straight_scenario().to_dict()),
        encoding="utf-8",
    )
    output = tmp_path / "results"

    exit_code = main(
        [
            str(scenario_path),
            "--planner",
            "visibility",
            "--output-dir",
            str(output),
        ]
    )

    assert exit_code == 0
    assert (output / "runs.csv").is_file()
    assert (output / "summary.csv").is_file()
    assert (output / "manifest.json").is_file()
