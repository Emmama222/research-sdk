import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtCore import Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from research_sdk.ui.app import ScenarioPlannerPage
from research_sdk.ui.runtime import ResearchRuntime
from research_sdk.ui.scenarios import Scenario, ScenarioObstacle, ScenarioRobot, ScenarioStore


@pytest.fixture(scope="module")
def application():
    app = QApplication.instance() or QApplication([])
    yield app


@pytest.fixture
def planner_page(application, tmp_path):
    page = ScenarioPlannerPage(ResearchRuntime(), ScenarioStore(tmp_path))
    page.resize(1200, 760)
    page.show()
    application.processEvents()
    yield page
    page.close()


def click_world(application, page, point_mm):
    screen = page.canvas._to_screen(point_mm).toPoint()
    QTest.mouseClick(page.canvas, Qt.LeftButton, pos=screen)
    application.processEvents()


def test_incomplete_robot_requires_target() -> None:
    scenario = Scenario(
        "incomplete",
        robots=[ScenarioRobot(2, False, (0.0, 0.0), None)],
    )

    assert scenario.validation_errors() == ("blue robot 2 has no target position",)
    with pytest.raises(ValueError, match="blue robot 2 has no target position"):
        scenario.require_complete()


def test_blank_scenario_can_be_saved_but_not_planned() -> None:
    scenario = Scenario("blank")

    assert scenario.validation_errors(require_robot=False) == ()
    assert scenario.validation_errors() == (
        "Select at least one robot and define its starting position",
    )


def test_editor_converts_and_relocates_obstacle_start_and_target(
    application, planner_page
) -> None:
    page = planner_page
    page.scenario = Scenario(
        "interactive",
        robots=[ScenarioRobot(1, True, (-1000.0, 0.0), (1000.0, 0.0))],
    )
    page._install_scenario("ready")

    page._select_tool("obstacles")
    click_world(application, page, (-1000.0, 0.0))
    assert not page.scenario.robots
    assert page.scenario.obstacles[0].position_mm == (-1000.0, 0.0)

    click_world(application, page, (-500.0, 500.0))
    assert page.scenario.obstacles[0].position_mm == pytest.approx(
        (-500.0, 500.0), abs=3.0
    )

    page._select_tool("starting_pos")
    click_world(application, page, (-500.0, 500.0))
    assert not page.scenario.obstacles
    assert page.scenario.robots[0].target_mm is None
    assert page.tool_buttons["target_pos"].isEnabled()

    click_world(application, page, (-250.0, 250.0))
    assert page.scenario.robots[0].start_mm == pytest.approx(
        (-250.0, 250.0), abs=10.0
    )

    page._select_tool("target_pos")
    click_world(application, page, (500.0, 0.0))
    click_world(application, page, (750.0, -250.0))
    assert page.scenario.robots[0].target_mm == pytest.approx(
        (750.0, -250.0), abs=10.0
    )


def test_target_tool_is_guarded_without_starting_robot(planner_page) -> None:
    page = planner_page
    page.scenario = Scenario("blank")
    page._install_scenario("blank")

    page._select_tool("target_pos")

    assert page.canvas.mode == "starting_pos"
    assert not page.tool_buttons["target_pos"].isEnabled()


def test_preview_plans_all_algorithms_without_starting_execution(planner_page) -> None:
    page = planner_page
    page.scenario = Scenario(
        "preview",
        robots=[ScenarioRobot(1, True, (-1000.0, 0.0), (1000.0, 0.0))],
        obstacles=[ScenarioObstacle(2, False, (0.0, 800.0), 90.0)],
    )
    page._install_scenario("ready")

    page._plan()

    assert len(page.previews) == page.planner_selector.count() == 3
    assert page.runtime.active_paths == ()
    assert all(preview.map_time_ms >= 0.0 for preview in page.previews.values())
    assert all(preview.planning_time_ms >= 0.0 for preview in page.previews.values())

    map_text = page.map_time.text()
    plan_text = page.plan_time.text()
    page._clear_preview()
    assert page.canvas.paths == ()
    assert page.map_time.text() == map_text
    assert page.plan_time.text() == plan_text
    page.planner_selector.setCurrentIndex(1)
    assert page.canvas.paths == ()
