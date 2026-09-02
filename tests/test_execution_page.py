import os
from time import monotonic

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

import research_sdk.ui.execution.page as page_module
from research_sdk.ui.execution.controller import ExecutionInput, ExecutionState
from research_sdk.ui.execution.page import ExecutionConsolePage
from research_sdk.ui.runtime import PlannedRobotPath, ResearchRuntime
from research_sdk.ui.scenarios import Scenario, ScenarioRobot, ScenarioStore
from research_sdk.world.snapshot import RobotSnapshot, WorldSnapshot, empty_robot_team


class PlannerA:
    def __init__(self, **_kwargs) -> None:
        pass


class PlannerB:
    def __init__(self, **_kwargs) -> None:
        pass


def _application() -> QApplication:
    return QApplication.instance() or QApplication([])


def _scenario() -> Scenario:
    return Scenario(
        "course",
        robots=[ScenarioRobot(1, True, (0.0, 0.0), (1000.0, 0.0))],
    )


def _snapshot(x: float = 0.0) -> WorldSnapshot:
    yellow = list(empty_robot_team())
    yellow[1] = RobotSnapshot(True, 1, x, 0.0, 0.0)
    return WorldSnapshot(
        version=1,
        timestamp=monotonic(),
        frame_number=1,
        ball=None,
        yellow=tuple(yellow),
        blue=empty_robot_team(),
        us_yellow=True,
        us_positive=True,
    )


def _page(monkeypatch, tmp_path) -> ExecutionConsolePage:
    _application()
    monkeypatch.setattr(
        page_module,
        "discover_planners",
        lambda: {"Planner A": PlannerA, "Planner B": PlannerB},
    )
    return ExecutionConsolePage(ResearchRuntime(), ScenarioStore(tmp_path))


def _load_input(page: ExecutionConsolePage) -> None:
    scenario = _scenario()
    path = PlannedRobotPath(1, True, ((0.0, 0.0), (1000.0, 0.0)))
    page.controller.load(
        ExecutionInput.create(
            scenario,
            {"Planner A": (path,), "Planner B": (path,)},
            {"Planner A": PlannerA, "Planner B": PlannerB},
        )
    )
    page.canvas.set_scenario(scenario)
    page._refresh_ui()


def test_run_controls_wait_for_vision_confirmed_apply(monkeypatch, tmp_path) -> None:
    page = _page(monkeypatch, tmp_path)
    _load_input(page)
    monkeypatch.setattr(page.runtime, "apply_scenario", lambda *_args, **_kwargs: None)

    assert not page.planner_rows["Planner A"][2].isEnabled()
    page._apply_scenario()
    assert page.controller.state is ExecutionState.APPLYING

    page.process_snapshot(_snapshot())
    page.process_snapshot(_snapshot())
    assert page.controller.state is ExecutionState.APPLYING
    page.process_snapshot(_snapshot())

    assert page.controller.state is ExecutionState.READY
    assert page.planner_rows["Planner A"][2].isEnabled()
    page.shutdown()


def test_owner_is_green_shadow_is_yellow_and_all_choices_lock(monkeypatch, tmp_path) -> None:
    page = _page(monkeypatch, tmp_path)
    _load_input(page)
    page.controller.begin_apply()
    page.controller.confirm_apply()
    page.controller.set_shadow("Planner B", True)
    page.controller.run("Planner A")
    page._refresh_ui()

    owner_checkbox, owner_role, owner_run, owner_row = page.planner_rows["Planner A"]
    shadow_checkbox, shadow_role, shadow_run, shadow_row = page.planner_rows["Planner B"]
    assert owner_role.text() == "EXECUTING"
    assert shadow_role.text() == "SHADOW"
    assert "#2e7d32" in owner_row.styleSheet()
    assert "#f9a825" in shadow_row.styleSheet()
    assert not owner_checkbox.isEnabled() and not shadow_checkbox.isEnabled()
    assert not owner_run.isEnabled() and not shadow_run.isEnabled()
    page.shutdown()


def test_add_scenario_navigates_without_editing_execution_page(monkeypatch, tmp_path) -> None:
    page = _page(monkeypatch, tmp_path)
    emitted = []
    page.navigate_to_planner.connect(lambda: emitted.append(True))

    page.add_scenario_button.click()

    assert emitted == [True]
    assert page.controller.state is ExecutionState.NO_SCENARIO
    page.shutdown()


def test_reset_control_is_labeled_as_estop_reset(monkeypatch, tmp_path) -> None:
    page = _page(monkeypatch, tmp_path)

    assert page.reset_button.text() == "Reset E-Stop"
    page.shutdown()


def test_one_planner_failure_does_not_hide_successful_planner(monkeypatch, tmp_path) -> None:
    page = _page(monkeypatch, tmp_path)
    page.store.save(_scenario())
    page.refresh_scenarios()
    calls = 0

    def plan(_scenario):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise RuntimeError("no route")
        return (PlannedRobotPath(1, True, ((0.0, 0.0), (1000.0, 0.0))),)

    monkeypatch.setattr(page.runtime, "plan", plan)

    page._load_scenario()

    assert page.controller.state is ExecutionState.SCENARIO_LOADED
    assert page.planner_rows["Planner B"][1].text() == "ERROR"
    assert "#b71c1c" in page.planner_rows["Planner B"][3].styleSheet()
    assert page.controller.execution_input.paths_by_planner["Planner A"]
    page.shutdown()


def test_loading_scenario_installs_full_visual_model_on_execution_canvas(
    monkeypatch, tmp_path
) -> None:
    page = _page(monkeypatch, tmp_path)
    scenario = _scenario()
    page.store.save(scenario)
    page.refresh_scenarios()

    page._load_scenario()

    assert page.canvas.scenario is not None
    assert page.canvas.scenario.to_dict() == scenario.to_dict()
    assert page.canvas.expected_keys == {(True, 1)}
    page.shutdown()
