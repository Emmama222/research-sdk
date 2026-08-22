"""PySide research console for scenario-based path-planner experiments."""

from __future__ import annotations

import sys
import time
from dataclasses import replace
from math import cos, sin
from pathlib import Path

import yaml
from PySide6.QtCore import QPointF, QRectF, Qt, QThread, QTimer, Signal
from PySide6.QtGui import QAction, QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from research_sdk.config import (
    DEFENCE_X_MM,
    DEFENCE_Y_MM,
    FIELD_LENGTH_MM,
    FIELD_WIDTH_MM,
    GOAL_DEPTH_MM,
    GOAL_WIDTH_MM,
    ROBOT_RADIUS_MM,
)
from research_sdk.network.ssl_sockets import Vision, grSimVision
from research_sdk.ui.runtime import (
    LiveRobot,
    PlannedRobotPath,
    ResearchRuntime,
    live_world_from_vision_packet,
)
from research_sdk.ui.scenarios import (
    Scenario,
    ScenarioBall,
    ScenarioObstacle,
    ScenarioRobot,
    ScenarioStore,
)
from research_sdk.ui.session import (
    ExperimentRecorder,
    SessionController,
    SessionState,
    discover_planners,
    export_placeholder_results,
)

CONFIG_FOLDER = Path(__file__).resolve().parents[1] / "config"
EDITABLE_CONFIGS = (
    "ssl_field_config.yaml",
    "planner_variables.yaml",
    "network_input.yaml",
    "robot_speed_config.yaml",
)


class VisionMonitor(QThread):
    packet_received = Signal(object, float)
    failed = Signal(str)

    def __init__(self, source: str, parent=None) -> None:
        super().__init__(parent)
        self.source = source
        self._running = True

    def is_set(self) -> bool:
        return self._running

    def run(self) -> None:
        while self._running:
            try:
                config = yaml.safe_load((CONFIG_FOLDER / "network_input.yaml").read_text())
                if self.source == "grSim vision":
                    receiver = grSimVision(self, port=int(config["grsim_vision_port"]))
                else:
                    receiver = Vision(
                        self,
                        port=int(config["ssl_vision_port"]),
                        group=str(config["ssl_vision_multicast_group"]),
                    )
                previous = time.perf_counter()
                while self._running:
                    packet = receiver.listen()
                    if packet is None:
                        continue
                    now = time.perf_counter()
                    self.packet_received.emit(packet, (now - previous) * 1000.0)
                    previous = now
            except Exception as exc:
                if not self._running:
                    break
                self.failed.emit(str(exc))
                self.msleep(1000)

    def stop(self) -> None:
        self._running = False
        self.wait(1200)


class LatencyLabel(QLabel):
    def set_latency(self, latency_ms: float | None) -> None:
        if latency_ms is None:
            self.setText("-- ms")
            color = "#ef5350"
        elif latency_ms < 50:
            self.setText(f"{latency_ms:.1f} ms")
            color = "#4caf50"
        elif latency_ms < 200:
            self.setText(f"{latency_ms:.1f} ms")
            color = "#ffca28"
        else:
            self.setText(f"{latency_ms:.1f} ms")
            color = "#ef5350"
        self.setStyleSheet(f"color: {color}; font-weight: 700;")


class FieldCanvas(QWidget):
    scenario_changed = Signal()
    live_robot_selected = Signal(object)

    def __init__(self) -> None:
        super().__init__()
        self.setMinimumSize(720, 480)
        self.scenario: Scenario | None = None
        self.paths: tuple[PlannedRobotPath, ...] = ()
        self.mode = "select"
        self.selected_robot: int | None = None
        self.robot_id = 0
        self.robot_yellow = True
        self.obstacle_id = 15
        self.obstacle_yellow = False
        self.obstacle_radius = ROBOT_RADIUS_MM
        self.live_robots: dict[tuple[bool, int], LiveRobot] = {}
        self.live_robot_seen_at: dict[tuple[bool, int], float] = {}
        self.live_ball_mm: tuple[float, float] | None = None
        self.live_ball_seen_at: float | None = None

    def update_live_world(self, packet) -> None:
        frame = live_world_from_vision_packet(packet)
        if frame is None:
            return
        self.update_live_frame(frame)

    def update_live_frame(self, frame) -> None:
        now = time.monotonic()
        for robot in frame.robots:
            key = (robot.is_yellow, robot.robot_id)
            self.live_robots[key] = robot
            self.live_robot_seen_at[key] = now
        stale_keys = [
            key for key, seen_at in self.live_robot_seen_at.items() if now - seen_at > 0.5
        ]
        for key in stale_keys:
            self.live_robots.pop(key, None)
            self.live_robot_seen_at.pop(key, None)
        if frame.ball_mm is not None:
            self.live_ball_mm = frame.ball_mm
            self.live_ball_seen_at = now
        elif self.live_ball_seen_at is not None and now - self.live_ball_seen_at > 0.5:
            self.live_ball_mm = None
            self.live_ball_seen_at = None
        self.update()

    def capture_world_snapshot(self, snapshot) -> None:
        """Render a frozen copy of the shared WorldSnapshot contract."""
        robots = (*snapshot.yellow, *snapshot.blue)
        self.live_robots = {
            (robot.isYellow, robot.robot_id): LiveRobot(
                robot.robot_id,
                robot.isYellow,
                robot.position,
                robot.theta,
            )
            for robot in robots
            if robot is not None
        }
        now = time.monotonic()
        self.live_robot_seen_at = {key: now for key in self.live_robots}
        self.live_ball_mm = None if snapshot.ball is None else snapshot.ball.position
        self.live_ball_seen_at = now if self.live_ball_mm is not None else None
        self.update()

    def current_live_robots(self) -> dict[tuple[bool, int], LiveRobot]:
        now = time.monotonic()
        return {
            key: robot
            for key, robot in self.live_robots.items()
            if now - self.live_robot_seen_at.get(key, 0.0) <= 0.5
        }

    def set_scenario(self, scenario: Scenario | None) -> None:
        self.scenario = scenario
        self.paths = ()
        self.selected_robot = None
        self.update()

    def set_paths(self, paths: tuple[PlannedRobotPath, ...]) -> None:
        self.paths = paths
        self.update()

    def clear_paths(self) -> None:
        self.paths = ()
        self.update()

    def _field_rect(self) -> QRectF:
        margin = 45.0
        available = self.rect().adjusted(margin, margin, -margin, -margin)
        scale = min(available.width() / FIELD_LENGTH_MM, available.height() / FIELD_WIDTH_MM)
        width = FIELD_LENGTH_MM * scale
        height = FIELD_WIDTH_MM * scale
        return QRectF(
            self.rect().center().x() - width / 2,
            self.rect().center().y() - height / 2,
            width,
            height,
        )

    def _to_screen(self, point: tuple[float, float]) -> QPointF:
        field = self._field_rect()
        return QPointF(
            field.center().x() + point[0] * field.width() / FIELD_LENGTH_MM,
            field.center().y() - point[1] * field.height() / FIELD_WIDTH_MM,
        )

    def _to_world(self, point: QPointF) -> tuple[float, float]:
        field = self._field_rect()
        x = (point.x() - field.center().x()) * FIELD_LENGTH_MM / field.width()
        y = (field.center().y() - point.y()) * FIELD_WIDTH_MM / field.height()
        return (
            max(-FIELD_LENGTH_MM / 2, min(FIELD_LENGTH_MM / 2, x)),
            max(-FIELD_WIDTH_MM / 2, min(FIELD_WIDTH_MM / 2, y)),
        )

    def paintEvent(self, event) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#101820"))
        field = self._field_rect()
        painter.fillRect(field, QColor("#176b3a"))
        painter.setPen(QPen(QColor("#f1f5f3"), 2))
        painter.drawRect(field)
        painter.drawLine(field.center().x(), field.top(), field.center().x(), field.bottom())
        centre_radius = 500 * field.width() / FIELD_LENGTH_MM
        painter.drawEllipse(field.center(), centre_radius, centre_radius)
        self._draw_penalty_boxes(painter, field)
        self._draw_goals(painter, field)
        self._draw_live_world(painter)
        if self.scenario is not None:
            self._draw_paths(painter)
            for obstacle in self.scenario.obstacles:
                self._draw_obstacle(painter, obstacle)
            for index, robot in enumerate(self.scenario.robots):
                self._draw_robot(painter, robot, index == self.selected_robot)

    def _draw_live_world(self, painter: QPainter) -> None:
        radius = ROBOT_RADIUS_MM * self._field_rect().width() / FIELD_LENGTH_MM
        for robot in self.live_robots.values():
            centre = self._to_screen(robot.position_mm)
            color = QColor("#ffd740" if robot.is_yellow else "#42a5f5")
            painter.setBrush(color)
            painter.setPen(QPen(color.darker(), 2))
            painter.drawEllipse(centre, radius, radius)
            painter.setPen(QPen(QColor("#101820"), 2))
            painter.drawLine(
                centre,
                centre
                + QPointF(
                    radius * cos(robot.orientation_rad),
                    -radius * sin(robot.orientation_rad),
                ),
            )
            painter.drawText(centre + QPointF(-4, 5), str(robot.robot_id))
        if self.live_ball_mm is not None:
            centre = self._to_screen(self.live_ball_mm)
            ball_radius = max(4.0, 21.5 * self._field_rect().width() / FIELD_LENGTH_MM)
            painter.setBrush(QColor("#ff7043"))
            painter.setPen(QPen(QColor("#ffccbc"), 1))
            painter.drawEllipse(centre, ball_radius, ball_radius)

    def _draw_penalty_boxes(self, painter: QPainter, field: QRectF) -> None:
        depth = DEFENCE_X_MM * field.width() / FIELD_LENGTH_MM
        height = DEFENCE_Y_MM * field.height() / FIELD_WIDTH_MM
        painter.drawRect(QRectF(field.left(), field.center().y() - height / 2, depth, height))
        painter.drawRect(QRectF(field.right() - depth, field.center().y() - height / 2, depth, height))

    def _draw_goals(self, painter: QPainter, field: QRectF) -> None:
        depth = GOAL_DEPTH_MM * field.width() / FIELD_LENGTH_MM
        height = GOAL_WIDTH_MM * field.height() / FIELD_WIDTH_MM
        painter.drawRect(QRectF(field.left() - depth, field.center().y() - height / 2, depth, height))
        painter.drawRect(QRectF(field.right(), field.center().y() - height / 2, depth, height))

    def _draw_robot(self, painter: QPainter, robot: ScenarioRobot, selected: bool) -> None:
        start = self._to_screen(robot.start_mm)
        target = self._to_screen(robot.target_mm)
        radius = ROBOT_RADIUS_MM * self._field_rect().width() / FIELD_LENGTH_MM
        color = QColor("#ffd740" if robot.is_yellow else "#42a5f5")
        painter.setBrush(QColor(color.red(), color.green(), color.blue(), 75))
        painter.setPen(QPen(QColor("white") if selected else color, 3 if selected else 2, Qt.DashLine))
        painter.drawEllipse(start, radius, radius)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(color, 2, Qt.DashLine))
        painter.drawEllipse(target, radius, radius)
        painter.drawLine(start, target)
        painter.setPen(QColor("#101820"))
        painter.drawText(start + QPointF(-4, 5), str(robot.robot_id))

    def _draw_obstacle(self, painter: QPainter, obstacle: ScenarioObstacle) -> None:
        centre = self._to_screen(obstacle.position_mm)
        radius = obstacle.radius_mm * self._field_rect().width() / FIELD_LENGTH_MM
        painter.setBrush(QColor(220, 70, 70, 180))
        painter.setPen(QPen(QColor("#ff8a80"), 2))
        painter.drawEllipse(centre, radius, radius)

    def _draw_paths(self, painter: QPainter) -> None:
        painter.setBrush(Qt.NoBrush)
        for path in self.paths:
            if len(path.points_mm) < 2:
                continue
            drawing = QPainterPath(self._to_screen(path.points_mm[0]))
            for point in path.points_mm[1:]:
                drawing.lineTo(self._to_screen(point))
            painter.setPen(QPen(QColor("#f5f5f5"), 3))
            painter.drawPath(drawing)

    def mousePressEvent(self, event) -> None:
        if not self._field_rect().contains(event.position()):
            return
        if self.mode == "select_live_robot":
            robot = self._nearest_live_robot(event.position())
            if robot is not None:
                self.live_robot_selected.emit(robot)
            return
        if self.scenario is None:
            return
        point = self._to_world(event.position())
        if self.mode == "relocate_start" and self.selected_robot is not None:
            robot = self.scenario.robots[self.selected_robot]
            self.scenario.robots[self.selected_robot] = replace(robot, start_mm=point)
        elif self.mode == "add_robot":
            existing = next(
                (
                    robot
                    for robot in self.scenario.robots
                    if robot.robot_id == self.robot_id
                    and robot.is_yellow == self.robot_yellow
                ),
                None,
            )
            target = existing.target_mm if existing is not None else point
            self.selected_robot = self.scenario.set_robot(
                ScenarioRobot(self.robot_id, self.robot_yellow, point, target)
            )
        elif self.mode == "set_target" and self.selected_robot is not None:
            robot = self.scenario.robots[self.selected_robot]
            self.scenario.robots[self.selected_robot] = replace(robot, target_mm=point)
        elif self.mode == "add_obstacle":
            self.scenario.set_obstacle(
                ScenarioObstacle(
                    self.obstacle_id,
                    self.obstacle_yellow,
                    point,
                    float(self.obstacle_radius),
                )
            )
        else:
            self.selected_robot = self._nearest_robot(event.position())
        self.scenario_changed.emit()
        self.update()

    def _nearest_live_robot(self, point: QPointF) -> LiveRobot | None:
        candidates = [
            ((self._to_screen(robot.position_mm) - point).manhattanLength(), robot)
            for robot in self.live_robots.values()
        ]
        if not candidates:
            return None
        distance, robot = min(candidates, key=lambda item: item[0])
        return robot if distance < 35 else None

    def _nearest_robot(self, point: QPointF) -> int | None:
        if self.scenario is None:
            return None
        candidates = [
            ((self._to_screen(robot.start_mm) - point).manhattanLength(), index)
            for index, robot in enumerate(self.scenario.robots)
        ]
        if not candidates:
            return None
        distance, index = min(candidates)
        return index if distance < 35 else None


class ConfigurationsPage(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.dirty = False
        self.applied = True
        self.selector = QComboBox()
        self.selector.addItems(EDITABLE_CONFIGS)
        self.editor = QPlainTextEdit()
        self.editor.setLineWrapMode(QPlainTextEdit.NoWrap)
        load_button = QPushButton("Load")
        save_button = QPushButton("Save")
        apply_button = QPushButton("Apply / validate")
        self.notice = QLabel()
        controls = QHBoxLayout()
        controls.addWidget(self.selector)
        controls.addWidget(load_button)
        controls.addWidget(save_button)
        controls.addWidget(apply_button)
        layout = QVBoxLayout(self)
        layout.addLayout(controls)
        layout.addWidget(self.editor)
        layout.addWidget(self.notice)
        self.selector.currentTextChanged.connect(self.load)
        load_button.clicked.connect(lambda: self.load(self.selector.currentText()))
        save_button.clicked.connect(self.save)
        apply_button.clicked.connect(self.apply)
        self.editor.textChanged.connect(self._changed)
        self.load(self.selector.currentText())

    def _changed(self) -> None:
        self.dirty = True
        self.applied = False
        self.notice.setText("Unsaved changes")

    def load(self, name: str) -> None:
        if self.dirty and not self._discard_changes():
            return
        path = CONFIG_FOLDER / name
        self.editor.blockSignals(True)
        self.editor.setPlainText(path.read_text(encoding="utf-8"))
        self.editor.blockSignals(False)
        self.dirty = False
        self.applied = True
        self.notice.setText(f"Loaded {path}")

    def save(self) -> None:
        try:
            yaml.safe_load(self.editor.toPlainText())
            path = CONFIG_FOLDER / self.selector.currentText()
            path.write_text(self.editor.toPlainText(), encoding="utf-8")
            self.dirty = False
            self.applied = False
            self.notice.setText("Saved. Apply to validate; runtime controls reload on use.")
        except Exception as exc:
            QMessageBox.critical(self, "Cannot save configuration", str(exc))

    def apply(self) -> None:
        if self.dirty:
            self.save()
        try:
            value = yaml.safe_load(
                (CONFIG_FOLDER / self.selector.currentText()).read_text(encoding="utf-8")
            )
            if not isinstance(value, dict):
                raise ValueError("Configuration must contain a YAML mapping")
            self.applied = True
            if self.selector.currentText() == "network_input.yaml":
                self.notice.setText("Validated. Vision reloads it the next time it is enabled.")
            else:
                self.notice.setText(
                    "Validated and saved. Restart the console to apply imported defaults."
                )
        except Exception as exc:
            QMessageBox.critical(self, "Invalid configuration", str(exc))

    def _discard_changes(self) -> bool:
        answer = QMessageBox.question(
            self,
            "Unsaved configuration",
            "Discard unsaved configuration changes?",
        )
        return answer == QMessageBox.Yes


class ResearchConsole(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Research SDK · Planner Lab")
        self.resize(1320, 820)
        self.store = ScenarioStore()
        self.session = SessionController()
        self.runtime = ResearchRuntime()
        self.vision_thread: VisionMonitor | None = None
        self.display_vision_thread: VisionMonitor | None = None
        self.current_scenario: Scenario | None = None
        self.current_scenario_path: Path | None = None
        self.recorder: ExperimentRecorder | None = None
        self.control_timer = QTimer(self)
        self.control_timer.setInterval(50)
        self.control_timer.timeout.connect(self._control_tick)

        self.tabs = QTabWidget()
        self.experiment_page = QWidget()
        self.config_page = ConfigurationsPage()
        self.tabs.addTab(self.experiment_page, "Experiment")
        self.tabs.addTab(self.config_page, "Configurations")
        self.setCentralWidget(self.tabs)
        self._build_experiment_page()
        self._build_toolbar()
        self._refresh_scenarios()
        self._refresh_controls()
        self._start_live_grsim_display()

    def _start_live_grsim_display(self) -> None:
        self.display_vision_thread = VisionMonitor("grSim vision", self)
        self.display_vision_thread.packet_received.connect(self._live_grsim_packet_received)
        self.display_vision_thread.failed.connect(self._live_grsim_failed)
        self.display_vision_thread.start()

    def _build_toolbar(self) -> None:
        toolbar = QToolBar("View")
        self.addToolBar(toolbar)
        map_action = QAction("Display map", self, checkable=True)
        map_action.setChecked(True)
        map_action.toggled.connect(self.live_canvas.setVisible)
        map_action.toggled.connect(self.canvas.setVisible)
        toolbar.addAction(map_action)

    def _build_experiment_page(self) -> None:
        self.live_canvas = FieldCanvas()
        self.canvas = FieldCanvas()
        self.canvas.scenario_changed.connect(self._scenario_edited)
        self.canvas.live_robot_selected.connect(self._select_plan_robot)
        self.field_tabs = QTabWidget()
        self.field_tabs.addTab(self.live_canvas, "Existing Field")
        self.field_tabs.addTab(self.canvas, "Plan Course")
        self.field_tabs.currentChanged.connect(self._field_mode_changed)
        panel = QWidget()
        form = QFormLayout(panel)

        self.scenario_selector = QComboBox()
        load_scenario = QPushButton("Load scenario")
        self.save_plan_button = QPushButton("Save plan as new file")
        self.update_plan_button = QPushButton("Update loaded plan")
        self.fresh_snapshot_button = QPushButton("Fresh snapshot")
        self.apply_plan_button = QPushButton("Apply to grSim")
        test_grsim = QPushButton("Test grSim connection")
        form.addRow("Saved scenarios", self.scenario_selector)
        form.addRow(load_scenario)
        form.addRow(self.save_plan_button)
        form.addRow(self.update_plan_button)
        form.addRow(self.fresh_snapshot_button)
        form.addRow(self.apply_plan_button)
        form.addRow(test_grsim)

        self.edit_mode = QComboBox()
        self.edit_mode.addItems(("select_live_robot", "relocate_start", "set_target"))
        self.robot_id = QSpinBox(); self.robot_id.setRange(0, 15)
        self.team = QComboBox(); self.team.addItems(("Yellow", "Blue"))
        self.obstacle_radius = QSpinBox(); self.obstacle_radius.setRange(1, 2000); self.obstacle_radius.setValue(int(ROBOT_RADIUS_MM))
        form.addRow("Plan Course tool", self.edit_mode)

        self.planner_selector = QComboBox()
        planners = discover_planners()
        self.planner_selector.addItems(planners or {"No planners discovered": object})
        form.addRow("Active planner", self.planner_selector)

        self.run_button = QPushButton("Run")
        self.stop_button = QPushButton("Stop")
        self.erase_button = QPushButton("Erase plan")
        self.reset_button = QPushButton("Reset")
        row = QHBoxLayout()
        for button in (self.run_button, self.stop_button, self.erase_button, self.reset_button):
            row.addWidget(button)
        form.addRow(row)

        self.vision_button = QCheckBox("Vision enabled")
        self.vision_source = QComboBox(); self.vision_source.addItems(("grSim vision", "real-life SSL vision"))
        form.addRow(self.vision_button)
        form.addRow("Vision source", self.vision_source)

        self.receive_latency = LatencyLabel(); self.receive_latency.set_latency(None)
        self.send_latency = LatencyLabel(); self.send_latency.set_latency(None)
        form.addRow("Network receive", self.receive_latency)
        form.addRow("Network send", self.send_latency)

        export_a = QPushButton("Export results A")
        export_b = QPushButton("Export results B")
        form.addRow(export_a)
        form.addRow(export_b)
        self.status = QLabel("After reset · ready")
        self.status.setWordWrap(True)
        form.addRow("Status", self.status)

        splitter = QSplitter()
        splitter.addWidget(self.field_tabs)
        splitter.addWidget(panel)
        splitter.setStretchFactor(0, 1)
        layout = QVBoxLayout(self.experiment_page)
        layout.addWidget(splitter)

        self.fresh_snapshot_button.clicked.connect(self._new_scenario)
        self.apply_plan_button.clicked.connect(self._save_scenario)
        test_grsim.clicked.connect(self._test_grsim_connection)
        load_scenario.clicked.connect(self._load_scenario)
        self.save_plan_button.clicked.connect(self._save_plan_as)
        self.update_plan_button.clicked.connect(self._update_plan_file)
        self.edit_mode.currentTextChanged.connect(self._update_canvas_tool)
        self.robot_id.valueChanged.connect(self._update_canvas_tool)
        self.team.currentIndexChanged.connect(self._update_canvas_tool)
        self.obstacle_radius.valueChanged.connect(self._update_canvas_tool)
        self.run_button.clicked.connect(self._run)
        self.stop_button.clicked.connect(self._stop)
        self.erase_button.clicked.connect(self._erase)
        self.reset_button.clicked.connect(self._reset)
        self.vision_button.toggled.connect(self._toggle_vision)
        export_a.clicked.connect(lambda: self._export("a"))
        export_b.clicked.connect(lambda: self._export("b"))

    def _new_scenario(self) -> None:
        self._capture_plan_snapshot()

    def _field_mode_changed(self, index: int) -> None:
        if index == 1:
            self._capture_plan_snapshot()

    def _capture_plan_snapshot(self) -> None:
        """Start every Plan Course visit from the latest runtime feedback."""
        snapshot = self.runtime.world_snapshot
        if snapshot is None:
            self.current_scenario = None
            self.canvas.set_scenario(None)
            self.field_tabs.blockSignals(True)
            self.field_tabs.setCurrentIndex(1)
            self.field_tabs.blockSignals(False)
            self.status.setText(
                "Plan Course is waiting for the first complete world snapshot from grSim."
            )
            return
        self.current_scenario = None
        self.current_scenario_path = None
        self.canvas.set_scenario(None)
        self.canvas.capture_world_snapshot(snapshot)
        self.edit_mode.setCurrentText("select_live_robot")
        self.field_tabs.blockSignals(True)
        self.field_tabs.setCurrentIndex(1)
        self.field_tabs.blockSignals(False)
        count = len(self.canvas.live_robots)
        self.status.setText(
            f"Fresh snapshot captured ({count} robots). Click a robot to plan its course."
        )

    def _select_plan_robot(self, robot: LiveRobot) -> None:
        robots = [
            ScenarioRobot(
                robot.robot_id,
                robot.is_yellow,
                robot.position_mm,
                robot.position_mm,
                robot.orientation_rad,
            )
        ]
        robots.extend(
            ScenarioRobot(
                other.robot_id,
                other.is_yellow,
                other.position_mm,
                other.position_mm,
                other.orientation_rad,
            )
            for other in self.canvas.live_robots.values()
            if (other.is_yellow, other.robot_id) != (robot.is_yellow, robot.robot_id)
        )
        self.current_scenario = Scenario(
            f"snapshot_{int(time.time())}",
            robots=robots,
            ball=(
                ScenarioBall(self.canvas.live_ball_mm)
                if self.canvas.live_ball_mm is not None
                else None
            ),
        )
        self.current_scenario_path = None
        self.canvas.set_scenario(self.current_scenario)
        self.canvas.selected_robot = 0
        self.edit_mode.setCurrentText("relocate_start")
        self.status.setText(
            f"Selected {'yellow' if robot.is_yellow else 'blue'} robot {robot.robot_id}. "
            "Click its proposed start, then choose Set target and click the destination."
        )
        self._refresh_controls()

    def _save_plan_as(self) -> None:
        if self.current_scenario is None or not self.current_scenario.robots:
            QMessageBox.warning(self, "Nothing to save", "Capture and select a robot first.")
            return
        name, accepted = QInputDialog.getText(
            self,
            "Save plan",
            "Course name",
            text=self.current_scenario.name,
        )
        if not accepted or not name.strip():
            return
        try:
            self.current_scenario.name = name.strip()
            self.current_scenario.schema_version = 2
            path = self.store.save(self.current_scenario)
            self.current_scenario_path = path
            self._refresh_scenarios()
            index = self.scenario_selector.findData(str(path))
            if index >= 0:
                self.scenario_selector.setCurrentIndex(index)
            self.status.setText(
                f"Saved {path.name}: {len(self.current_scenario.robots)} robots and "
                f"ball={'yes' if self.current_scenario.ball is not None else 'no'}"
            )
            self._refresh_controls()
        except Exception as exc:
            QMessageBox.critical(self, "Cannot save plan", str(exc))

    def _update_plan_file(self) -> None:
        if self.current_scenario is None or not self.current_scenario.robots:
            QMessageBox.warning(self, "Nothing to update", "Capture or load a plan first.")
            return
        if self.current_scenario_path is None:
            QMessageBox.information(
                self,
                "Save required",
                "This is a new plan. Use Save plan as new file first.",
            )
            return
        try:
            self.current_scenario.schema_version = 2
            path = self.store.update(self.current_scenario_path, self.current_scenario)
            self.status.setText(
                f"Updated {path.name}: {len(self.current_scenario.robots)} robots and "
                f"ball={'yes' if self.current_scenario.ball is not None else 'no'}"
            )
        except Exception as exc:
            QMessageBox.critical(self, "Cannot update plan", str(exc))

    def _save_scenario(self) -> None:
        if self.current_scenario is None or not self.current_scenario.robots:
            QMessageBox.warning(self, "Incomplete course", "Select a robot first.")
            return
        robot = self.current_scenario.robots[0]
        if robot.start_mm == robot.target_mm:
            QMessageBox.warning(
                self,
                "Incomplete course",
                "Choose Set target and click a destination before applying.",
            )
            return
        try:
            self.runtime.apply_scenario(self.current_scenario, include_obstacles=False)
            self.send_latency.set_latency(self.runtime.last_send_latency_ms)
            self.session.scenario_forwarded()
            self.status.setText("Plan applied to grSim; ready to generate and run the course")
            self._refresh_controls()
        except Exception as exc:
            QMessageBox.critical(self, "Scenario forwarding failed", str(exc))

    def _test_grsim_connection(self) -> None:
        try:
            probe = self.runtime.test_grsim_connection()
            vision_confirmed = (
                self.vision_thread is not None
                and self.vision_thread.isRunning()
                and self.runtime.last_receive_latency_ms is not None
            )
            confirmation = (
                "Vision is also receiving packets from grSim."
                if vision_confirmed
                else "UDP has no acknowledgement; enable grSim vision to confirm receipt."
            )
            self.status.setText(
                f"UDP route ready: {probe.local_address[0]}:{probe.local_address[1]} → "
                f"{probe.destination[0]}:{probe.destination[1]}. {confirmation}"
            )
        except Exception as exc:
            QMessageBox.critical(self, "grSim connection test failed", str(exc))

    def _clear_obstacles(self) -> None:
        if self.current_scenario is None:
            return
        self.current_scenario.clear_obstacles()
        self._scenario_edited()
        self.canvas.update()

    def _load_scenario(self) -> None:
        if not self.scenario_selector.currentData():
            return
        try:
            path = Path(self.scenario_selector.currentData())
            self.current_scenario = self.store.load(path)
            self.current_scenario_path = path
            self.canvas.set_scenario(self.current_scenario)
            if self.current_scenario.ball is not None:
                self.canvas.live_ball_mm = self.current_scenario.ball.position_mm
                self.canvas.live_ball_seen_at = time.monotonic()
            self.status.setText(f"Loaded {self.current_scenario.name}; forward it before running")
            self._refresh_controls()
        except Exception as exc:
            QMessageBox.critical(self, "Cannot load scenario", str(exc))

    def _refresh_scenarios(self) -> None:
        self.scenario_selector.clear()
        for path in self.store.list_paths():
            self.scenario_selector.addItem(path.stem, str(path))

    def _update_canvas_tool(self) -> None:
        self.canvas.mode = self.edit_mode.currentText()
        self.canvas.robot_id = self.robot_id.value()
        self.canvas.obstacle_id = self.robot_id.value()
        self.canvas.robot_yellow = self.team.currentIndex() == 0
        self.canvas.obstacle_yellow = self.team.currentIndex() == 0
        self.canvas.obstacle_radius = self.obstacle_radius.value()

    def _scenario_edited(self) -> None:
        self.canvas.clear_paths()
        self.status.setText("Plan edited; use Update loaded plan or Save plan as new file")

    def _run(self) -> None:
        if self.current_scenario is None:
            return
        try:
            self.session.run()
            paths = self.runtime.plan(self.current_scenario)
            self.canvas.set_paths(paths)
            self.live_canvas.set_paths(paths)
            self.runtime.start_execution(paths)
            self.control_timer.start()
            self.recorder = ExperimentRecorder(
                self.current_scenario.name,
                self.planner_selector.currentText(),
            )
            self.recorder.record(
                "plans_generated",
                robots=[
                    {
                        "robot_id": path.robot_id,
                        "is_yellow": path.is_yellow,
                        "points_mm": path.points_mm,
                    }
                    for path in paths
                ],
            )
            self.status.setText(
                f"Running · recording events in {self.recorder.folder}"
            )
            self._refresh_controls()
        except Exception as exc:
            QMessageBox.critical(self, "Cannot run scenario", str(exc))

    def _stop(self) -> None:
        try:
            self.control_timer.stop()
            self.runtime.stop_execution()
            self.session.stop()
            if self.recorder is not None:
                self.recorder.record("run_stopped")
                self.recorder.close()
            self.status.setText("Stopped · plan retained")
            self._refresh_controls()
        except Exception as exc:
            QMessageBox.warning(self, "Cannot stop", str(exc))

    def _erase(self) -> None:
        try:
            self.control_timer.stop()
            self.runtime.stop_execution()
            self.session.erase_plan()
            self.runtime.reset_planner()
            self.canvas.clear_paths()
            self.live_canvas.clear_paths()
            if self.recorder is not None:
                if not self.recorder.closed:
                    self.recorder.record("plan_erased")
                    self.recorder.close()
                self.recorder = None
            self.status.setText("Active plan erased")
            self._refresh_controls()
        except Exception as exc:
            QMessageBox.warning(self, "Cannot erase", str(exc))

    def _reset(self) -> None:
        try:
            self.control_timer.stop()
            self.runtime.stop_execution()
            self.session.reset()
            self.runtime.reset_planner()
            self.canvas.clear_paths()
            self.live_canvas.clear_paths()
            if self.current_scenario is not None and self.session.has_scenario:
                self.runtime.apply_scenario(self.current_scenario)
                self.send_latency.set_latency(self.runtime.last_send_latency_ms)
                message = "Reset to the last forwarded scenario"
            else:
                message = "Reset complete; using grSim default positions"
            self.status.setText(message)
            self._refresh_controls()
        except Exception as exc:
            QMessageBox.warning(self, "Cannot reset", str(exc))

    def _toggle_vision(self, enabled: bool) -> None:
        try:
            self.session.set_vision(enabled)
            if enabled:
                self.vision_thread = VisionMonitor(self.vision_source.currentText(), self)
                self.vision_thread.packet_received.connect(self._vision_packet_received)
                self.vision_thread.failed.connect(self._vision_failed)
                self.vision_thread.start()
                self.status.setText("Vision started with current network_input.yaml")
            elif self.vision_thread is not None:
                self.vision_thread.stop()
                self.vision_thread = None
                self.runtime.last_receive_latency_ms = None
                self.receive_latency.set_latency(None)
            self._refresh_controls()
        except Exception as exc:
            self.vision_button.blockSignals(True)
            self.vision_button.setChecked(not enabled)
            self.vision_button.blockSignals(False)
            QMessageBox.warning(self, "Vision state unchanged", str(exc))

    def _vision_failed(self, message: str) -> None:
        self.runtime.last_receive_latency_ms = None
        self.receive_latency.set_latency(None)
        self.status.setText(f"Vision error: {message}")

    def _control_tick(self) -> None:
        try:
            if not self.runtime.execute_tick(self.runtime.live_robots):
                return
            self.control_timer.stop()
            self.runtime.stop_execution()
            self.session.stop()
            if self.recorder is not None and not self.recorder.closed:
                self.recorder.record("robots_arrived")
                self.recorder.close()
            self.status.setText("Completed · all robots reached their targets")
            self._refresh_controls()
        except Exception as exc:
            self.control_timer.stop()
            try:
                self.runtime.stop_execution()
            except Exception:
                pass
            try:
                self.session.stop()
            except Exception:
                pass
            self.status.setText(f"Execution stopped: {exc}")
            self._refresh_controls()

    def _vision_packet_received(self, packet, latency_ms: float) -> None:
        del packet
        self.runtime.last_receive_latency_ms = latency_ms
        self.receive_latency.set_latency(latency_ms)

    def _live_grsim_packet_received(self, packet, latency_ms: float) -> None:
        frame = self.runtime.ingest_vision_packet(packet)
        if frame is not None:
            self.live_canvas.update_live_frame(frame)
            if (
                self.field_tabs.currentIndex() == 1
                and self.current_scenario is None
                and not self.canvas.live_robots
            ):
                self._capture_plan_snapshot()
        self.runtime.last_receive_latency_ms = latency_ms
        self.receive_latency.set_latency(latency_ms)

    def _live_grsim_failed(self, message: str) -> None:
        self.status.setText(f"Live grSim display unavailable: {message}")

    def _export(self, format_name: str) -> None:
        destination, _ = QFileDialog.getSaveFileName(
            self,
            f"Export result format {format_name.upper()}",
            f"results_{format_name}.csv",
            "CSV files (*.csv)",
        )
        if destination:
            path = export_placeholder_results(destination, format_name)
            self.status.setText(f"Exported placeholder result headings to {path}")

    def _refresh_controls(self) -> None:
        state = self.session.state
        self.run_button.setEnabled(state in (SessionState.SCENARIO_READY, SessionState.STOPPED) and self.session.has_scenario)
        self.stop_button.setEnabled(state is SessionState.RUNNING)
        self.erase_button.setEnabled(state is not SessionState.RUNNING and self.session.has_plan)
        self.reset_button.setEnabled(state in (SessionState.STOPPED, SessionState.AFTER_RESET))
        self.vision_button.setEnabled(self.session.can_change_vision)
        self.vision_source.setEnabled(self.session.can_change_vision_source)
        self.planner_selector.setEnabled(state is not SessionState.RUNNING)
        self.save_plan_button.setEnabled(
            state is not SessionState.RUNNING and self.current_scenario is not None
        )
        self.update_plan_button.setEnabled(
            state is not SessionState.RUNNING and self.current_scenario_path is not None
        )

    def closeEvent(self, event) -> None:
        if self.config_page.dirty:
            answer = QMessageBox.question(
                self,
                "Unsaved configuration",
                "Close and discard unsaved configuration changes?",
            )
            if answer != QMessageBox.Yes:
                event.ignore()
                return
        if self.vision_thread is not None:
            self.vision_thread.stop()
        self.control_timer.stop()
        self.runtime.stop_execution()
        if self.display_vision_thread is not None:
            self.display_vision_thread.stop()
        if self.recorder is not None:
            self.recorder.close()
        event.accept()


def main() -> int:
    application = QApplication.instance() or QApplication(sys.argv)
    application.setStyle("Fusion")
    window = ResearchConsole()
    window.show()
    return application.exec()


if __name__ == "__main__":
    raise SystemExit(main())
