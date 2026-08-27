import os
from dataclasses import replace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

import research_sdk.ui.app as ui_app
from research_sdk.ui.app import ResearchConsole
from research_sdk.ui.scenarios import ScenarioStore
from research_sdk.world.snapshot import (
    BallSnapshot,
    RobotSnapshot,
    WorldSnapshot,
    empty_robot_team,
)


def _application() -> QApplication:
    return QApplication.instance() or QApplication([])


def _snapshot() -> WorldSnapshot:
    yellow = list(empty_robot_team())
    yellow[2] = RobotSnapshot(True, 2, 100.0, -200.0, 0.5)
    blue = list(empty_robot_team())
    blue[4] = RobotSnapshot(False, 4, -300.0, 400.0, -0.25)
    return WorldSnapshot(
        version=1,
        timestamp=1.0,
        frame_number=1,
        ball=BallSnapshot(10.0, 20.0),
        yellow=tuple(yellow),
        blue=tuple(blue),
        us_yellow=True,
        us_positive=True,
    )


def test_plan_course_tab_stays_open_while_waiting_for_snapshot(monkeypatch) -> None:
    _application()
    monkeypatch.setattr(ResearchConsole, "_start_live_grsim_display", lambda self: None)
    window = ResearchConsole()

    window.field_tabs.setCurrentIndex(1)

    assert window.field_tabs.currentIndex() == 1
    assert "waiting" in window.status.text().lower()
    window.close()


def test_plan_course_captures_shared_world_snapshot(monkeypatch) -> None:
    _application()
    monkeypatch.setattr(ResearchConsole, "_start_live_grsim_display", lambda self: None)
    window = ResearchConsole()
    snapshot = _snapshot()
    window.runtime.world_pipeline.store.publish(snapshot)

    window.field_tabs.setCurrentIndex(1)

    assert window.field_tabs.currentIndex() == 1
    assert (True, 2) in window.canvas.live_robots
    assert window.canvas.live_ball_mm == (10.0, 20.0)
    assert "fresh snapshot captured" in window.status.text().lower()
    window.close()


def test_save_and_update_plan_persist_all_robots_and_ball(monkeypatch, tmp_path) -> None:
    _application()
    monkeypatch.setattr(ResearchConsole, "_start_live_grsim_display", lambda self: None)
    monkeypatch.setattr(
        ui_app.QInputDialog,
        "getText",
        lambda *_args, **_kwargs: ("complete field", True),
    )
    window = ResearchConsole()
    window.store = ScenarioStore(tmp_path)
    window.runtime.world_pipeline.store.publish(_snapshot())
    window.field_tabs.setCurrentIndex(1)
    window._select_plan_robot(window.canvas.live_robots[(True, 2)])

    window._save_plan_as()

    path = tmp_path / "complete_field.json"
    saved = window.store.load(path)
    assert {(robot.is_yellow, robot.robot_id) for robot in saved.robots} == {
        (True, 2),
        (False, 4),
    }
    assert saved.ball.position_mm == (10.0, 20.0)

    window.current_scenario.robots[0] = replace(
        window.current_scenario.robots[0], target_mm=(900.0, 600.0)
    )
    window._update_plan_file()

    updated = window.store.load(path)
    assert updated.robots[0].target_mm == (900.0, 600.0)
    assert updated.ball.position_mm == (10.0, 20.0)
    window.close()
