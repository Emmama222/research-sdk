import os
from time import monotonic

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

import research_sdk.ui.execution.page as page_module
from research_sdk.ui.execution.checkpoints import CheckpointStore
from research_sdk.ui.execution.controller import ExecutionInput, ExecutionState
from research_sdk.ui.execution.page import ExecutionConsolePage
from research_sdk.ui.runtime import LiveRobot, PlannedRobotPath, ResearchRuntime
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

    assert not page.run_button.isEnabled()
    page._apply_scenario()
    assert page.controller.state is ExecutionState.APPLYING

    page.process_snapshot(_snapshot())
    page.process_snapshot(_snapshot())
    assert page.controller.state is ExecutionState.APPLYING
    page.process_snapshot(_snapshot())

    assert page.controller.state is ExecutionState.READY
    assert page.run_button.isEnabled()
    page.shutdown()


def test_owner_is_green_and_exclusive_planner_choices_lock(monkeypatch, tmp_path) -> None:
    page = _page(monkeypatch, tmp_path)
    _load_input(page)
    page.controller.begin_apply()
    page.controller.confirm_apply()
    page.planner_rows["Planner A"][0].setChecked(True)
    page.controller.run("Planner A")
    page._refresh_ui()

    owner_radio, owner_role, owner_row = page.planner_rows["Planner A"]
    other_radio, other_role, other_row = page.planner_rows["Planner B"]
    assert owner_role.text() == "EXECUTING"
    assert other_role.text() == "INACTIVE"
    assert "#2e7d32" in owner_row.styleSheet()
    assert other_row.styleSheet() == ""
    assert owner_radio.isChecked() and not other_radio.isChecked()
    assert not owner_radio.isEnabled() and not other_radio.isEnabled()
    assert not page.run_button.isEnabled()
    page.shutdown()


def test_single_run_button_uses_exclusively_selected_planner(monkeypatch, tmp_path) -> None:
    page = _page(monkeypatch, tmp_path)
    _load_input(page)
    page.controller.begin_apply()
    page.controller.confirm_apply()
    page._refresh_ui()
    monkeypatch.setattr(page.runtime, "start_execution", lambda *_args, **_kwargs: None)

    page.planner_rows["Planner B"][0].setChecked(True)
    page.run_button.click()

    assert page.controller.velocity_owner == "Planner B"
    assert sum(radio.isChecked() for radio, _role, _row in page.planner_rows.values()) == 1
    page.shutdown()


def test_add_scenario_navigates_without_editing_execution_page(monkeypatch, tmp_path) -> None:
    page = _page(monkeypatch, tmp_path)
    emitted = []
    page.navigate_to_planner.connect(lambda: emitted.append(True))

    page.add_scenario_button.click()

    assert emitted == [True]
    assert page.controller.state is ExecutionState.NO_SCENARIO
    page.shutdown()


def test_reset_control_is_labeled_reset(monkeypatch, tmp_path) -> None:
    page = _page(monkeypatch, tmp_path)

    assert page.reset_button.text() == "Reset"
    assert not page.reset_button.isEnabled()
    page.shutdown()


def test_checkpoint_selector_is_labeled_for_restart_action(monkeypatch, tmp_path) -> None:
    page = _page(monkeypatch, tmp_path)

    assert page.checkpoint_label.text() == "Last checkpoint"
    assert page.resume_checkpoint_button.text() == "Restart checkpoint"
    page.shutdown()


def test_reset_stays_disabled_until_scenario_is_confirmed_in_grsim(
    monkeypatch, tmp_path
) -> None:
    page = _page(monkeypatch, tmp_path)
    _load_input(page)

    assert page.controller.state is ExecutionState.SCENARIO_LOADED
    assert not page.reset_button.isEnabled()

    page.controller.begin_apply()
    page._refresh_ui()
    assert not page.reset_button.isEnabled()

    page.controller.confirm_apply()
    page._refresh_ui()
    assert page.reset_button.isEnabled()
    page.shutdown()


def test_debug_console_is_collapsible_and_does_not_expand_by_default(
    monkeypatch, tmp_path
) -> None:
    page = _page(monkeypatch, tmp_path)

    assert page.debug_console.isHidden()
    assert page.debug_console.height() == 96
    page.debug_toggle.setChecked(True)
    assert not page.debug_console.isHidden()
    page._log_debug("command sent")
    assert "command sent" in page.debug_console.toPlainText()
    page.debug_toggle.setChecked(False)
    assert page.debug_console.isHidden()
    page.shutdown()


def test_initial_planner_selection_waits_until_ui_is_fully_built(monkeypatch, tmp_path) -> None:
    page = _page(monkeypatch, tmp_path)

    assert page.summary is not None
    assert page.planner_rows["Planner A"][0].isChecked()
    assert sum(radio.isChecked() for radio, _role, _row in page.planner_rows.values()) == 1
    page.shutdown()


def test_error_toast_is_canvas_overlay_and_starts_at_half_opacity(
    monkeypatch, tmp_path
) -> None:
    page = _page(monkeypatch, tmp_path)

    page._show_error_toast("send failed")

    assert page.error_toast.parent() is page.canvas
    assert page.error_toast.text() == "send failed"
    assert 0.0 < page._toast_opacity.opacity() <= 0.5
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
    assert "#b71c1c" in page.planner_rows["Planner B"][2].styleSheet()
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


def test_unload_scenario_clears_execution_page_and_visual_model(
    monkeypatch, tmp_path
) -> None:
    page = _page(monkeypatch, tmp_path)
    _load_input(page)

    assert page.unload_button.isEnabled()
    page.unload_button.click()

    assert page.controller.state is ExecutionState.NO_SCENARIO
    assert page.controller.execution_input is None
    assert page.canvas.scenario is None
    assert page.canvas.paths == ()
    assert not page.unload_button.isEnabled()
    assert not page.reset_button.isEnabled()
    page.shutdown()


def _run_checkpointable(page: ExecutionConsolePage, path: PlannedRobotPath) -> None:
    page.controller.load(
        ExecutionInput.create(
            _scenario(),
            {"Planner A": (path,), "Planner B": (path,)},
            {"Planner A": PlannerA, "Planner B": PlannerB},
        )
    )
    page.canvas.set_scenario(_scenario())
    page.controller.begin_apply()
    page.controller.confirm_apply()
    page.controller.run("Planner A")
    page.run_id = "run-test"
    page.checkpoint_store = CheckpointStore(page.store.folder / "checkpoints.jsonl")
    page.runtime.start_execution((path,))


def test_reaching_the_final_waypoint_does_not_create_a_checkpoint(
    monkeypatch, tmp_path
) -> None:
    page = _page(monkeypatch, tmp_path)
    path = PlannedRobotPath(1, True, ((0.0, 0.0), (500.0, 0.0), (1000.0, 0.0)))
    _run_checkpointable(page, path)

    page.runtime.world_pipeline.store.publish(_snapshot(500.0))
    page.runtime.execute_tick({(True, 1): LiveRobot(1, True, (500.0, 0.0), 0.0)})
    page._record_transitions(page.runtime.last_waypoint_transitions)
    assert len(page.checkpoints) == 1

    page.runtime.world_pipeline.store.publish(_snapshot(1000.0))
    page.runtime.execute_tick({(True, 1): LiveRobot(1, True, (1000.0, 0.0), 0.0)})
    page._record_transitions(page.runtime.last_waypoint_transitions)
    assert len(page.checkpoints) == 1
    page.shutdown()


def test_delete_selected_result_row_removes_it_from_both_result_tables(
    monkeypatch, tmp_path
) -> None:
    page = _page(monkeypatch, tmp_path)
    page.result_rows_a = [
        {"run_id": "run-1", "planner": "Planner A", "score": "1"},
        {"run_id": "run-2", "planner": "Planner A", "score": "2"},
    ]
    page.result_rows_b = [
        {"run_id": "run-1", "planner": "Planner A", "detail": "x"},
        {"run_id": "run-2", "planner": "Planner A", "detail": "y"},
    ]
    page._populate_table(page.result_a_table, page.result_rows_a)
    page._populate_table(page.result_b_table, page.result_rows_b)

    page.result_a_table.selectRow(1)
    page._delete_selected_result_rows(page.result_a_table)

    assert [row["run_id"] for row in page.result_rows_a] == ["run-1"]
    assert [row["run_id"] for row in page.result_rows_b] == ["run-1"]
    assert page.result_a_table.rowCount() == 1
    assert page.result_b_table.rowCount() == 1
    page.shutdown()
