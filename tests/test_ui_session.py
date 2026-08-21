import csv

import pytest

from research_sdk.ui.scenarios import (
    Scenario,
    ScenarioObstacle,
    ScenarioRobot,
    ScenarioStore,
)
from research_sdk.ui.session import (
    RESULT_COLUMNS_A,
    RESULT_COLUMNS_B,
    ExperimentRecorder,
    SessionController,
    SessionState,
    discover_planners,
    export_placeholder_results,
)


def test_scenario_json_round_trip(tmp_path) -> None:
    scenario = Scenario(
        "two robots",
        robots=[
            ScenarioRobot(1, True, (-1000.0, 0.0), (1000.0, 0.0)),
            ScenarioRobot(2, False, (0.0, -1000.0), (0.0, 1000.0)),
        ],
        obstacles=[ScenarioObstacle(15, False, (0.0, 0.0), 120.0)],
    )
    store = ScenarioStore(tmp_path)

    path = store.save(scenario)
    loaded = store.load(path)

    assert path.name == "two_robots.json"
    assert loaded == scenario


def test_setting_same_team_robot_moves_instead_of_duplicating() -> None:
    scenario = Scenario("move robot")

    first_index = scenario.set_robot(
        ScenarioRobot(1, True, (-1000.0, 0.0), (1000.0, 0.0))
    )
    moved_index = scenario.set_robot(
        ScenarioRobot(1, True, (-500.0, 250.0), (1000.0, 0.0))
    )
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


@pytest.mark.parametrize(
    ("format_name", "columns"),
    (("a", RESULT_COLUMNS_A), ("b", RESULT_COLUMNS_B)),
)
def test_placeholder_result_export_writes_headings_only(
    tmp_path,
    format_name,
    columns,
) -> None:
    path = export_placeholder_results(tmp_path / f"result_{format_name}.csv", format_name)

    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.reader(stream))
    assert rows == [list(columns)]


def test_current_planner_is_discoverable() -> None:
    planners = discover_planners()
    assert any("VoronoiDijkstraPlanner" in name for name in planners)


def test_experiment_recorder_writes_lifecycle_events(tmp_path) -> None:
    recorder = ExperimentRecorder("demo", "Voronoi", tmp_path)
    recorder.record("plans_generated", robots=2)
    recorder.close()

    lines = recorder.path.read_text(encoding="utf-8").splitlines()
    assert '"event":"run_started"' in lines[0]
    assert '"event":"plans_generated"' in lines[1]
