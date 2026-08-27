import csv

import pytest

from research_sdk.ui.scenarios import (
    Scenario,
    ScenarioBall,
    ScenarioObstacle,
    ScenarioRobot,
    ScenarioStore,
)
from research_sdk.ui.session import (
    RESULT_COLUMNS_A,
    RESULT_COLUMNS_B,
    ExperimentRecorder,
    RunMetrics,
    SessionController,
    SessionState,
    discover_planners,
    export_planner_results,
    export_results,
)


def test_scenario_json_round_trip(tmp_path) -> None:
    scenario = Scenario(
        "two robots",
        robots=[
            ScenarioRobot(1, True, (-1000.0, 0.0), (1000.0, 0.0)),
            ScenarioRobot(2, False, (0.0, -1000.0), (0.0, 1000.0)),
        ],
        obstacles=[
            ScenarioObstacle(
                15,
                False,
                (0.0, 0.0),
                120.0,
                planner_keys=("example.PRMPlanner",),
            )
        ],
        ball=ScenarioBall((250.0, -125.0), (10.0, 20.0)),
    )
    store = ScenarioStore(tmp_path)

    path = store.save(scenario)
    loaded = store.load(path)

    assert path.name == "two_robots.json"
    assert loaded == scenario


def test_scenario_filters_shared_and_algorithm_specific_obstacles() -> None:
    shared = ScenarioObstacle(1, False, (0.0, 0.0), 90.0)
    prm_only = ScenarioObstacle(
        2, False, (100.0, 0.0), 90.0, planner_keys=("example.PRMPlanner",)
    )
    scenario = Scenario("layouts", obstacles=[shared, prm_only])

    assert scenario.obstacles_for("example.PRMPlanner") == (shared, prm_only)
    assert scenario.obstacles_for("example.VisibilityGraphPlanner") == (shared,)


def test_scenario_store_updates_existing_file_without_renaming(tmp_path) -> None:
    store = ScenarioStore(tmp_path)
    scenario = Scenario("original", robots=[ScenarioRobot(1, True, (0, 0), (1, 1))])
    path = store.save(scenario)
    scenario.name = "renamed inside file"
    scenario.ball = ScenarioBall((50.0, 75.0))

    updated = store.update(path, scenario)

    assert updated == path
    assert updated.name == "original.json"
    assert store.load(updated).name == "renamed inside file"
    assert store.load(updated).ball.position_mm == (50.0, 75.0)


def test_setting_same_team_robot_moves_instead_of_duplicating() -> None:
    scenario = Scenario("move robot")

    first_index = scenario.set_robot(ScenarioRobot(1, True, (-1000.0, 0.0), (1000.0, 0.0)))
    moved_index = scenario.set_robot(ScenarioRobot(1, True, (-500.0, 250.0), (1000.0, 0.0)))
    scenario.set_robot(ScenarioRobot(1, False, (0.0, 0.0), (0.0, 0.0)))

    assert first_index == moved_index == 0
    assert len(scenario.robots) == 2
    assert scenario.robots[0].start_mm == (-500.0, 250.0)


def test_robot_and_grsim_obstacle_cannot_share_team_identity() -> None:
    scenario = Scenario(
        "unique identities",
        robots=[ScenarioRobot(3, False, (0.0, 0.0), (100.0, 0.0))],
    )

    scenario.set_obstacle(ScenarioObstacle(3, False, (200.0, 0.0), 90.0))
    assert scenario.robots == []
    assert len(scenario.obstacles) == 1

    scenario.set_robot(ScenarioRobot(3, False, (300.0, 0.0), (400.0, 0.0)))
    assert scenario.obstacles == []
    assert len(scenario.robots) == 1


def test_clear_obstacles_does_not_remove_planned_robots() -> None:
    scenario = Scenario(
        "clear obstacles",
        robots=[ScenarioRobot(1, True, (0.0, 0.0), (100.0, 0.0))],
        obstacles=[ScenarioObstacle(2, False, (200.0, 0.0), 90.0)],
    )

    scenario.clear_obstacles()

    assert scenario.obstacles == []
    assert len(scenario.robots) == 1


def test_session_control_lifecycle_and_guards() -> None:
    session = SessionController()
    assert session.state is SessionState.AFTER_RESET
    assert session.can_change_vision_source

    session.set_vision(True)
    assert not session.can_change_vision_source
    session.set_vision(False)
    session.scenario_forwarded()
    with pytest.raises(RuntimeError):
        session.set_vision(True)

    session.run()
    assert session.state is SessionState.RUNNING
    with pytest.raises(RuntimeError):
        session.erase_plan()
    session.stop()
    assert session.has_plan
    session.erase_plan()
    assert not session.has_plan
    session.reset()
    assert session.state is SessionState.AFTER_RESET
    assert not session.has_scenario


def test_session_can_reset_a_forwarded_course_before_running() -> None:
    session = SessionController()
    session.scenario_forwarded()

    session.reset()

    assert session.state is SessionState.AFTER_RESET
    assert not session.has_scenario
    assert not session.has_plan


@pytest.mark.parametrize(
    ("format_name", "columns"),
    (("a", RESULT_COLUMNS_A), ("b", RESULT_COLUMNS_B)),
)
def test_result_export_writes_calculated_values(
    tmp_path,
    format_name,
    columns,
) -> None:
    metrics = RunMetrics(started_at=10.0, cpu_started_at=4.0)
    metrics.record_pipeline(2.0, 1.0)
    metrics.record_pipeline(4.0, 3.0)
    metrics.record_planning((5.0, 15.0), failures=1)
    metrics.start_execution(now=20.0)
    metrics.finish(completed=True, now=21.25, cpu_now=4.025)
    path = export_results(tmp_path / f"result_{format_name}.csv", format_name, metrics)

    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert tuple(rows[0]) == columns
    assert rows[0]["input_latency_ms"] == "3.0"
    if format_name == "a":
        assert rows[0]["mapping_time_ms"] == "2.0"
        assert rows[0]["planning_time_ms"] == "20.0"
        assert rows[0]["number_of_fails"] == "1"
    else:
        assert rows[0]["average_planner_execution_time_ms"] == "10.0"
        assert rows[0]["robot_arrival_time_ms"] == "1250.0"
        assert rows[0]["total_plans_made"] == "2"
        assert rows[0]["resources_used"] == "25.0"


def test_planner_comparison_export_writes_one_labeled_row_per_planner(tmp_path) -> None:
    first = RunMetrics()
    first.record_planning((1.0,))
    second = RunMetrics()
    second.record_planning((2.0,))

    path = export_planner_results(
        tmp_path / "comparison.csv",
        "a",
        {"PRM": first, "Visibility Graph": second},
    )

    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert [row["planner"] for row in rows] == ["PRM", "Visibility Graph"]
    assert [row["planning_time_ms"] for row in rows] == ["1.0", "2.0"]


def test_collision_metric_counts_episodes_not_control_ticks() -> None:
    from research_sdk.ui.runtime import LiveRobot

    metrics = RunMetrics()
    touching = {
        (True, 1): LiveRobot(1, True, (0.0, 0.0), 0.0),
        (False, 2): LiveRobot(2, False, (100.0, 0.0), 0.0),
    }
    metrics.observe_robots(touching)
    metrics.observe_robots(touching)
    touching[(False, 2)] = LiveRobot(2, False, (1000.0, 0.0), 0.0)
    metrics.observe_robots(touching)
    touching[(False, 2)] = LiveRobot(2, False, (100.0, 0.0), 0.0)
    metrics.observe_robots(touching)

    assert metrics.number_of_collisions == 2


def test_current_planner_is_discoverable() -> None:
    planners = discover_planners()
    assert any("VoronoiDijkstraPlanner" in name for name in planners)


def test_experiment_recorder_writes_lifecycle_events(tmp_path) -> None:
    recorder = ExperimentRecorder("demo", "Voronoi", tmp_path)
    recorder.record("plans_generated", robots=2)
    recorder.finish(completed=False)
    recorder.close()

    lines = recorder.path.read_text(encoding="utf-8").splitlines()
    assert '"event":"run_started"' in lines[0]
    assert '"event":"plans_generated"' in lines[1]
    assert '"event":"metrics_finalized"' in lines[2]
