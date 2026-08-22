import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from research_sdk.ui.app import ResearchConsole
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
    return WorldSnapshot(
        version=1,
        timestamp=1.0,
        frame_number=1,
        ball=BallSnapshot(10.0, 20.0),
        yellow=tuple(yellow),
        blue=empty_robot_team(),
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
