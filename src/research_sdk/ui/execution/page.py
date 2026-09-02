"""Qt execution console for applying and safely running scenarios in grSim."""

# Qt slots are process safety boundaries: they catch failures to stop robots and
# report them in the UI instead of unwinding through the Qt event loop.
# ruff: noqa: BLE001, S110

from __future__ import annotations

import csv
import time
from copy import deepcopy
from dataclasses import replace
from datetime import UTC, datetime
from math import atan2, cos, hypot, sin
from pathlib import Path
from uuid import uuid4

from PySide6.QtCore import (
    QEasingCurve,
    QEvent,
    QPointF,
    QPropertyAnimation,
    QRectF,
    Qt,
    QTimer,
    Signal,
)
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (
    QButtonGroup,
    QComboBox,
    QFileDialog,
    QGraphicsOpacityEffect,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMenu,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QRadioButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QToolButton,
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
from research_sdk.network.robot_command import RobotCommand
from research_sdk.planners.common import StepRecorder
from research_sdk.ui.execution.apply_verifier import ApplyReport, ScenarioApplyVerifier
from research_sdk.ui.execution.checkpoints import CheckpointRecord, CheckpointStore
from research_sdk.ui.execution.controller import (
    ExecutionController,
    ExecutionInput,
    ExecutionState,
)
from research_sdk.ui.runtime import LiveRobot, PlannedRobotPath, ResearchRuntime
from research_sdk.ui.scenarios import (
    Scenario,
    ScenarioBall,
    ScenarioObstacle,
    ScenarioRobot,
    ScenarioStore,
)
from research_sdk.ui.session import (
    ExperimentRecorder,
    RunMetrics,
    discover_planners,
    planner_debug_geometry,
)
from research_sdk.world.snapshot import WorldSnapshot


class ExecutionFieldCanvas(QWidget):
    """Live grSim field with execution progress and safety indicators."""

    resized = Signal()

    def __init__(self) -> None:
        super().__init__()
        # Leave vertical room for the collapsible console. The map is the
        # flexible region and intentionally shrinks when the console opens.
        self.setMinimumSize(600, 300)
        self.scenario: Scenario | None = None
        self.robots: dict[tuple[bool, int], LiveRobot] = {}
        self.last_seen: dict[tuple[bool, int], float] = {}
        self.expected_keys: set[tuple[bool, int]] = set()
        self.paths: tuple[PlannedRobotPath, ...] = ()
        self.waypoint_indices: dict[tuple[bool, int], int] = {}
        self.state = ExecutionState.NO_SCENARIO
        self.colliding_keys: set[tuple[bool, int]] = set()
        self.show_map_layer = True
        self.debug_nodes: tuple[tuple[float, float], ...] = ()
        self.debug_edges: tuple[tuple[tuple[float, float], tuple[float, float]], ...] = ()

    def set_scenario(self, scenario: Scenario | None) -> None:
        self.scenario = scenario
        self.expected_keys = set()
        if scenario is not None:
            self.expected_keys = {
                *((robot.is_yellow, robot.robot_id) for robot in scenario.robots),
                *((obstacle.is_yellow, obstacle.obstacle_id) for obstacle in scenario.obstacles),
            }
        self.update()

    def update_snapshot(self, snapshot: WorldSnapshot) -> None:
        now = time.monotonic()
        for robot in (*snapshot.yellow, *snapshot.blue):
            if robot is None:
                continue
            key = (robot.isYellow, robot.robot_id)
            self.robots[key] = LiveRobot(
                robot.robot_id, robot.isYellow, robot.position, robot.theta
            )
            self.last_seen[key] = now
        self._update_collisions()
        self.update()

    def _field_rect(self) -> QRectF:
        margin = 42.0
        available = self.rect().adjusted(margin, margin, -margin, -margin)
        scale = min(
            available.width() / FIELD_LENGTH_MM,
            available.height() / FIELD_WIDTH_MM,
        )
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
        self._draw_boxes(painter, field)
        if self.show_map_layer:
            self._draw_debug_map(painter)
        self._draw_scenario(painter)
        self._draw_paths(painter)
        self._draw_robots(painter)
        if self.state in (
            ExecutionState.PAUSED,
            ExecutionState.STOPPED,
            ExecutionState.ERROR,
        ):
            painter.setPen(QColor("#ffffff"))
            painter.setBrush(QColor(0, 0, 0, 150))
            badge = QRectF(field.center().x() - 85, field.top() + 14, 170, 34)
            painter.drawRoundedRect(badge, 8, 8)
            painter.drawText(badge, Qt.AlignCenter, self.state.value)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self.resized.emit()

    def _draw_debug_map(self, painter: QPainter) -> None:
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor(144, 202, 249, 120), 1))
        for first, second in self.debug_edges:
            painter.drawLine(self._to_screen(first), self._to_screen(second))
        painter.setBrush(QColor(144, 202, 249, 170))
        for node in self.debug_nodes:
            painter.drawEllipse(self._to_screen(node), 2.5, 2.5)

    def _draw_scenario(self, painter: QPainter) -> None:
        """Draw the loaded experiment before live execution begins."""
        if self.scenario is None:
            return
        if self.scenario.ball is not None:
            centre = self._to_screen(self.scenario.ball.position_mm)
            radius = max(4.0, 21.5 * self._field_rect().width() / FIELD_LENGTH_MM)
            painter.setBrush(QColor("#ff7043"))
            painter.setPen(QPen(QColor("#ffccbc"), 1))
            painter.drawEllipse(centre, radius, radius)
        for obstacle in self.scenario.obstacles:
            self._draw_scenario_obstacle(painter, obstacle)
        for robot in self.scenario.robots:
            self._draw_scenario_robot(painter, robot)

    def _draw_scenario_obstacle(
        self, painter: QPainter, obstacle: ScenarioObstacle
    ) -> None:
        centre = self._to_screen(obstacle.position_mm)
        radius = obstacle.radius_mm * self._field_rect().width() / FIELD_LENGTH_MM
        painter.setBrush(QColor(220, 70, 70, 180))
        painter.setPen(QPen(QColor("#ff8a80"), 2))
        painter.drawEllipse(centre, radius, radius)
        painter.drawLine(centre + QPointF(-radius, -radius), centre + QPointF(radius, radius))
        painter.drawLine(centre + QPointF(-radius, radius), centre + QPointF(radius, -radius))
        painter.drawText(centre + QPointF(-8, -radius - 5), f"O{obstacle.obstacle_id}")

    def _draw_scenario_robot(self, painter: QPainter, robot: ScenarioRobot) -> None:
        start = self._to_screen(robot.start_mm)
        radius = ROBOT_RADIUS_MM * self._field_rect().width() / FIELD_LENGTH_MM
        color = QColor("#ffd740" if robot.is_yellow else "#42a5f5")
        painter.setBrush(QColor(color.red(), color.green(), color.blue(), 100))
        painter.setPen(QPen(color, 3))
        painter.drawEllipse(start, radius, radius)
        painter.drawText(start + QPointF(-8, -radius - 5), f"S{robot.robot_id}")
        if robot.target_mm is not None:
            target = self._to_screen(robot.target_mm)
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(QColor("#f5f5f5"), 2, Qt.DashLine))
            painter.drawEllipse(target, radius, radius)
            painter.drawLine(target + QPointF(-8, 0), target + QPointF(8, 0))
            painter.drawLine(target + QPointF(0, -8), target + QPointF(0, 8))
            painter.drawText(target + QPointF(-8, -radius - 5), f"T{robot.robot_id}")

    def _draw_boxes(self, painter: QPainter, field: QRectF) -> None:
        depth = DEFENCE_X_MM * field.width() / FIELD_LENGTH_MM
        height = DEFENCE_Y_MM * field.height() / FIELD_WIDTH_MM
        painter.drawRect(QRectF(field.left(), field.center().y() - height / 2, depth, height))
        painter.drawRect(
            QRectF(field.right() - depth, field.center().y() - height / 2, depth, height)
        )
        goal_depth = GOAL_DEPTH_MM * field.width() / FIELD_LENGTH_MM
        goal_height = GOAL_WIDTH_MM * field.height() / FIELD_WIDTH_MM
        painter.drawRect(
            QRectF(field.left() - goal_depth, field.center().y() - goal_height / 2, goal_depth, goal_height)
        )
        painter.drawRect(
            QRectF(field.right(), field.center().y() - goal_height / 2, goal_depth, goal_height)
        )

    def _draw_paths(self, painter: QPainter) -> None:
        painter.setBrush(Qt.NoBrush)
        for path in self.paths:
            if len(path.points_mm) < 2:
                continue
            # A previous robot's waypoint marker leaves a yellow brush active.
            # Paths can form closed regions, so reset the brush per robot to
            # prevent Qt from filling those regions yellow.
            painter.setBrush(Qt.NoBrush)
            key = (path.is_yellow, path.robot_id)
            index = self.waypoint_indices.get(key, 1)
            completed = path.points_mm[: max(1, index)]
            remaining = path.points_mm[max(0, index - 1) :]
            if len(completed) >= 2:
                painter.setBrush(Qt.NoBrush)
                drawing = QPainterPath(self._to_screen(completed[0]))
                for point in completed[1:]:
                    drawing.lineTo(self._to_screen(point))
                painter.setPen(QPen(QColor("#66bb6a"), 4))
                painter.drawPath(drawing)
            if len(remaining) >= 2:
                painter.setBrush(Qt.NoBrush)
                drawing = QPainterPath(self._to_screen(remaining[0]))
                for point in remaining[1:]:
                    drawing.lineTo(self._to_screen(point))
                painter.setPen(QPen(QColor("#ffffff"), 3))
                painter.drawPath(drawing)
            if index < len(path.points_mm):
                waypoint = self._to_screen(path.points_mm[index])
                painter.setBrush(QColor("#ffee58"))
                painter.drawEllipse(waypoint, 6, 6)

    def _draw_robots(self, painter: QPainter) -> None:
        now = time.monotonic()
        field = self._field_rect()
        radius = ROBOT_RADIUS_MM * field.width() / FIELD_LENGTH_MM
        keys = self.expected_keys | set(self.robots)
        for key in keys:
            robot = self.robots.get(key)
            if robot is None:
                continue
            stale = now - self.last_seen.get(key, 0.0) > 0.5
            centre = self._to_screen(robot.position_mm)
            color = QColor("#ffd740" if robot.is_yellow else "#42a5f5")
            color.setAlpha(128 if stale else 255)
            painter.setBrush(color)
            painter.setPen(QPen(color.darker(), 2))
            painter.drawEllipse(centre, radius, radius)
            painter.drawLine(
                centre,
                centre
                + QPointF(
                    radius * cos(robot.orientation_rad),
                    -radius * sin(robot.orientation_rad),
                ),
            )
            painter.setPen(QColor("#101820"))
            painter.drawText(centre + QPointF(-4, 5), str(robot.robot_id))
            if stale:
                painter.setPen(QPen(QColor("#ffffff"), 2))
                painter.drawText(centre + QPointF(radius + 3, -radius), "?")
            if key in self.colliding_keys:
                painter.setPen(QPen(QColor("#ff1744"), 3))
                painter.drawText(centre + QPointF(-3, -radius - 6), "!")

    def _update_collisions(self) -> None:
        self.colliding_keys.clear()
        values = list(self.robots.items())
        for index, (first_key, first) in enumerate(values):
            for second_key, second in values[index + 1 :]:
                if hypot(
                    first.position_mm[0] - second.position_mm[0],
                    first.position_mm[1] - second.position_mm[1],
                ) <= 2 * ROBOT_RADIUS_MM:
                    self.colliding_keys.update((first_key, second_key))


class ExecutionConsolePage(QWidget):
    """Main-page UI that owns safe grSim execution, never scenario editing."""

    navigate_to_planner = Signal()

    def __init__(self, runtime: ResearchRuntime, store: ScenarioStore) -> None:
        super().__init__()
        self.runtime = runtime
        self.store = store
        self.controller = ExecutionController()
        self.verifier: ScenarioApplyVerifier | None = None
        self.last_apply_report: ApplyReport | None = None
        self.canvas = ExecutionFieldCanvas()
        self.planners = discover_planners()
        self.planner_rows: dict[str, tuple[QRadioButton, QLabel, QWidget]] = {}
        self.planner_failures: dict[str, str] = {}
        self.debug_geometry: dict[
            str,
            tuple[
                tuple[tuple[float, float], ...],
                tuple[tuple[tuple[float, float], tuple[float, float]], ...],
            ],
        ] = {}
        self.current_metrics: dict[str, RunMetrics] = {}
        self.metric_templates: dict[str, RunMetrics] = {}
        self.result_rows_a: list[dict] = []
        self.result_rows_b: list[dict] = []
        self.recorder: ExperimentRecorder | None = None
        self.checkpoint_store: CheckpointStore | None = None
        self.checkpoints: list[CheckpointRecord] = []
        self.recorded_boundaries: set[tuple[bool, int, int]] = set()
        self.run_id = ""
        self.run_kind = "experiment"
        self.parent_run_id: str | None = None
        self.source_checkpoint_id: str | None = None
        self.run_started_at: float | None = None
        self._stepping = False
        self._pending_checkpoint: CheckpointRecord | None = None
        self._motion_test_key: tuple[bool, int] | None = None
        self._motion_test_start_theta: float | None = None
        self._motion_test_started_at: float | None = None
        self._motion_test_samples = 0
        self._last_snapshot_received_at: float | None = None
        self._toast_animation: QPropertyAnimation | None = None
        self.execution_timer = QTimer(self)
        self.execution_timer.setInterval(50)
        self.execution_timer.timeout.connect(self._execution_tick)
        self.motion_stop_timer = QTimer(self)
        self.motion_stop_timer.setSingleShot(True)
        self.motion_stop_timer.timeout.connect(self._finish_motion_test)
        self.motion_command_timer = QTimer(self)
        self.motion_command_timer.setInterval(50)
        self.motion_command_timer.timeout.connect(self._send_motion_test_command)
        self.watchdog_timer = QTimer(self)
        self.watchdog_timer.setInterval(100)
        self.watchdog_timer.timeout.connect(self._watchdog_tick)
        self.watchdog_timer.start()
        self._build_ui()
        self.refresh_scenarios()
        self._refresh_ui()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        top = QGridLayout()
        self.vision_test_button = QPushButton("Vision health test")
        self.motion_test_button = QPushButton("Motion feedback test")
        top.addWidget(self.vision_test_button, 0, 0)
        top.addWidget(self.motion_test_button, 1, 0)

        self.state_badge = QLabel()
        self.state_badge.setAlignment(Qt.AlignCenter)
        self.state_badge.setStyleSheet("font-size: 16px; font-weight: 800; padding: 8px;")
        transport = QHBoxLayout()
        self.run_button = QPushButton("Run")
        self.pause_button = QPushButton("Pause")
        self.previous_step_button = QPushButton("Previous step")
        self.step_button = QPushButton("Step")
        self.checkpoint_selector = QComboBox()
        self.resume_checkpoint_button = QPushButton("Restart checkpoint")
        self.reset_button = QPushButton("Reset")
        for button in (
            self.run_button,
            self.pause_button,
            self.previous_step_button,
            self.step_button,
            self.reset_button,
        ):
            transport.addWidget(button)
        transport_widget = QWidget()
        transport_widget.setLayout(transport)
        top.addWidget(self.state_badge, 0, 1)
        top.addWidget(transport_widget, 1, 1)
        checkpoint_row = QHBoxLayout()
        self.checkpoint_label = QLabel("Last checkpoint")
        checkpoint_row.addWidget(self.checkpoint_label)
        checkpoint_row.addWidget(self.checkpoint_selector, 1)
        checkpoint_row.addWidget(self.resume_checkpoint_button)
        self.emergency_button = QPushButton("EMERGENCY STOP")
        self.emergency_button.setStyleSheet(
            "background:#b71c1c;color:white;font-weight:800;font-size:12px;padding:4px 10px;"
        )
        checkpoint_row.addWidget(self.emergency_button)
        checkpoint_widget = QWidget()
        checkpoint_widget.setLayout(checkpoint_row)
        top.addWidget(checkpoint_widget, 2, 1)

        self.scenario_selector = QComboBox()
        self.refresh_button = QPushButton("Refresh")
        self.add_scenario_button = QPushButton("Add Scenario")
        self.load_button = QPushButton("Load Scenario")
        self.unload_button = QPushButton("Unload Scenario")
        self.apply_button = QPushButton("Load scenario into grSim")
        scenario_controls = QGridLayout()
        scenario_controls.addWidget(self.scenario_selector, 0, 0)
        scenario_controls.addWidget(self.refresh_button, 0, 1)
        scenario_controls.addWidget(self.add_scenario_button, 1, 0)
        scenario_controls.addWidget(self.load_button, 1, 1)
        scenario_controls.addWidget(self.unload_button, 2, 0, 1, 2)
        scenario_controls.addWidget(self.apply_button, 3, 0, 1, 2)
        scenario_widget = QWidget()
        scenario_widget.setLayout(scenario_controls)
        top.addWidget(scenario_widget, 0, 2, 3, 1)
        top.setColumnStretch(1, 1)
        root.addLayout(top)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        planner_group = QGroupBox("Planners")
        planner_layout = QVBoxLayout(planner_group)
        self.planner_group = QButtonGroup(self)
        self.planner_group.setExclusive(True)
        for label, planner_cls in self.planners.items():
            del planner_cls
            row_widget = QWidget()
            row = QHBoxLayout(row_widget)
            radio = QRadioButton()
            name = QLabel(label)
            role = QLabel("INACTIVE")
            row.addWidget(radio)
            row.addWidget(name, 1)
            row.addWidget(role)
            self.planner_group.addButton(radio)
            radio.toggled.connect(lambda _checked: self._refresh_ui())
            planner_layout.addWidget(row_widget)
            self.planner_rows[label] = (radio, role, row_widget)
        if self.planner_rows:
            first_radio = next(iter(self.planner_rows.values()))[0]
            first_radio.blockSignals(True)
            first_radio.setChecked(True)
            first_radio.blockSignals(False)
        right_layout.addWidget(planner_group)

        self.results_tabs = QTabWidget()
        self.result_a_table = self._create_table()
        self.result_b_table = self._create_table()
        self.results_tabs.addTab(self.result_a_table, "Result A")
        self.results_tabs.addTab(self.result_b_table, "Result B")
        right_layout.addWidget(self.results_tabs, 1)
        export_row = QHBoxLayout()
        self.export_a_button = QPushButton("Export Result A")
        self.export_b_button = QPushButton("Export Result B")
        export_row.addWidget(self.export_a_button)
        export_row.addWidget(self.export_b_button)
        right_layout.addLayout(export_row)

        map_panel = QWidget()
        map_layout = QVBoxLayout(map_panel)
        map_layout.setContentsMargins(0, 0, 0, 0)
        map_layout.setSpacing(4)
        self.map_layer_toggle = QToolButton()
        self.map_layer_toggle.setText("Map")
        self.map_layer_toggle.setCheckable(True)
        self.map_layer_toggle.setChecked(True)
        map_layout.addWidget(self.map_layer_toggle, 0, Qt.AlignLeft)
        map_layout.addWidget(self.canvas, 1)

        self.debug_toggle = QToolButton()
        self.debug_toggle.setText("Debug console")
        self.debug_toggle.setCheckable(True)
        self.debug_toggle.setChecked(False)
        self.debug_toggle.setArrowType(Qt.RightArrow)
        self.debug_toggle.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.debug_console = QPlainTextEdit()
        self.debug_console.setReadOnly(True)
        self.debug_console.setMaximumBlockCount(500)
        self.debug_console.setFixedHeight(96)
        self.debug_console.setStyleSheet(
            "background:#0b1117;color:#cfd8dc;font-family:monospace;"
            "border:1px solid #263238;"
        )
        self.debug_console.hide()
        map_layout.addWidget(self.debug_toggle)
        map_layout.addWidget(self.debug_console)

        self.error_toast = QLabel(self.canvas)
        self.error_toast.setWordWrap(True)
        self.error_toast.setMaximumWidth(320)
        self.error_toast.setStyleSheet(
            "background:#b71c1c;color:white;border-radius:6px;padding:8px;font-weight:700;"
        )
        self.error_toast.hide()
        self._toast_opacity = QGraphicsOpacityEffect(self.error_toast)
        self.error_toast.setGraphicsEffect(self._toast_opacity)
        self.canvas.resized.connect(self._position_error_toast)

        splitter = QSplitter()
        splitter.addWidget(map_panel)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 1)
        splitter.setSizes((900, 420))
        root.addWidget(splitter, 1)

        self.summary = QLabel("run -- | elapsed -- ms | plans 0 | collisions 0")
        root.addWidget(self.summary)

        self.vision_test_button.clicked.connect(self._vision_test)
        self.motion_test_button.clicked.connect(self._motion_test)
        self.refresh_button.clicked.connect(self.refresh_scenarios)
        self.add_scenario_button.clicked.connect(self.navigate_to_planner.emit)
        self.load_button.clicked.connect(self._load_scenario)
        self.unload_button.clicked.connect(self._unload_scenario)
        self.apply_button.clicked.connect(self._apply_scenario)
        self.pause_button.clicked.connect(self._pause)
        self.previous_step_button.clicked.connect(self._previous_step)
        self.step_button.clicked.connect(self._step)
        self.run_button.clicked.connect(lambda checked=False: self._run_or_continue())
        self.resume_checkpoint_button.clicked.connect(self._resume_checkpoint)
        self.reset_button.clicked.connect(self._reset)
        self.emergency_button.clicked.connect(lambda: self.emergency_stop())
        self.export_a_button.clicked.connect(lambda: self._export_table("a"))
        self.export_b_button.clicked.connect(lambda: self._export_table("b"))
        self.result_a_table.customContextMenuRequested.connect(
            lambda pos: self._show_result_table_menu(self.result_a_table, pos)
        )
        self.result_b_table.customContextMenuRequested.connect(
            lambda pos: self._show_result_table_menu(self.result_b_table, pos)
        )
        self.result_a_table.installEventFilter(self)
        self.result_b_table.installEventFilter(self)
        self.debug_toggle.toggled.connect(self._toggle_debug_console)
        self.map_layer_toggle.toggled.connect(self._toggle_map_layer)
        self.checkpoint_selector.currentIndexChanged.connect(self._refresh_ui)

    def _toggle_map_layer(self, checked: bool) -> None:
        self.canvas.show_map_layer = checked
        self.canvas.update()

    def _toggle_debug_console(self, expanded: bool) -> None:
        self.debug_toggle.setArrowType(Qt.DownArrow if expanded else Qt.RightArrow)
        self.debug_console.setVisible(expanded)

    def _log_debug(self, message: str, *, level: str = "INFO") -> None:
        stamp = datetime.now().astimezone().strftime("%H:%M:%S.%f")[:-3]
        self.debug_console.appendPlainText(f"{stamp} [{level}] {message}")

    def _show_error_toast(self, message: str) -> None:
        self.error_toast.setText(message)
        self.error_toast.adjustSize()
        self._position_error_toast()
        self._toast_opacity.setOpacity(0.5)
        self.error_toast.show()
        self.error_toast.raise_()
        animation = QPropertyAnimation(self._toast_opacity, b"opacity", self)
        animation.setDuration(500)
        animation.setStartValue(0.5)
        animation.setEndValue(0.0)
        animation.setEasingCurve(QEasingCurve.OutCubic)
        animation.finished.connect(self.error_toast.hide)
        self._toast_animation = animation
        animation.start()

    def _position_error_toast(self) -> None:
        if not hasattr(self, "error_toast"):
            return
        self.error_toast.move(
            max(8, self.canvas.width() - self.error_toast.width() - 12),
            12,
        )

    @staticmethod
    def _create_table() -> QTableWidget:
        table = QTableWidget()
        table.setColumnCount(0)
        table.setRowCount(0)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        table.setSortingEnabled(True)
        table.setSelectionBehavior(QTableWidget.SelectRows)
        table.setSelectionMode(QTableWidget.ExtendedSelection)
        table.setContextMenuPolicy(Qt.CustomContextMenu)
        return table

    def refresh_scenarios(self) -> None:
        selected = self.scenario_selector.currentData()
        self.scenario_selector.clear()
        paths = self.store.list_paths()
        for path in paths:
            self.scenario_selector.addItem(path.stem, str(path))
        if selected:
            index = self.scenario_selector.findData(selected)
            if index >= 0:
                self.scenario_selector.setCurrentIndex(index)
        self._log_debug(f"[SCENARIO] refreshed scenario list: {len(paths)} available")

    def _load_scenario(self) -> None:
        path_text = self.scenario_selector.currentData()
        if not path_text:
            return
        try:
            scenario = self.store.load(path_text)
            scenario.require_complete()
            paths_by_planner = {}
            planner_classes = {}
            metrics = {}
            failures = {}
            debug_geometry = {}
            for label, planner_cls in self.planners.items():
                recorder = StepRecorder()
                self.runtime.set_planner(planner_cls, record=recorder)
                planner_classes[label] = planner_cls
                planner_metrics = RunMetrics()
                try:
                    paths_by_planner[label] = self.runtime.plan(scenario)
                except Exception as exc:
                    paths_by_planner[label] = ()
                    failures[label] = str(exc)
                update = self.runtime.last_pipeline_update
                if update is not None:
                    planner_metrics.record_pipeline(
                        update.processing_latency_ms, update.mapping_time_ms
                    )
                planner_metrics.record_planning(
                    self.runtime.last_plan_durations_ms,
                    self.runtime.last_plan_failures,
                )
                metrics[label] = planner_metrics
                debug_geometry[label] = planner_debug_geometry(
                    label, recorder, tuple(scenario.obstacles)
                )
            if not any(paths_by_planner.values()):
                details = "; ".join(
                    f"{planner}: {message}" for planner, message in failures.items()
                )
                raise RuntimeError(f"No planner produced an executable path. {details}")
            self.planner_failures = failures
            self.metric_templates = metrics
            self.current_metrics = deepcopy(metrics)
            self.debug_geometry = debug_geometry
            self.controller.load(
                ExecutionInput.create(scenario, paths_by_planner, planner_classes)
            )
            self.canvas.set_scenario(scenario)
            self._log_debug(f"[SCENARIO] loaded to view: {scenario.name}")
            self._refresh_ui()
        except Exception as exc:
            QMessageBox.critical(self, "Cannot load execution scenario", str(exc))

    def _unload_scenario(self) -> None:
        try:
            self.controller.unload()
            self.verifier = None
            self.last_apply_report = None
            self.planner_failures.clear()
            self.debug_geometry.clear()
            self.canvas.set_scenario(None)
            self.canvas.paths = ()
            self.canvas.waypoint_indices.clear()
            self._log_debug("[SCENARIO] unloaded")
            self._refresh_ui()
        except Exception as exc:
            message = f"Cannot unload scenario: {exc}"
            self._log_debug(message, level="ERROR")
            self._show_error_toast(message)

    def _apply_scenario(self) -> None:
        execution_input = self.controller.execution_input
        if execution_input is None:
            return
        try:
            self.controller.begin_apply()
            self.verifier = ScenarioApplyVerifier(execution_input.scenario)
            self.runtime.apply_scenario(execution_input.scenario, include_obstacles=True)
            self._log_debug("[APPLY] replacement sent; waiting for 3 stable snapshots")
            self._refresh_ui()
        except Exception as exc:
            self._fail(f"Scenario application failed: {exc}")

    def process_snapshot(self, snapshot: WorldSnapshot) -> None:
        self._last_snapshot_received_at = time.monotonic()
        self.canvas.update_snapshot(snapshot)
        if self._motion_test_key is not None and snapshot.robot(*self._motion_test_key):
            self._motion_test_samples += 1
        if self.controller.state in (ExecutionState.APPLYING, ExecutionState.RESETTING):
            if self.verifier is None:
                self._fail("Apply verifier is missing")
                return
            report = self.verifier.observe(snapshot, now=snapshot.timestamp)
            self.last_apply_report = report
            if report.ready:
                self._log_debug(f"[APPLY] {self._format_apply_report(report)}")
                self.controller.confirm_apply()
                self.verifier = None
                if self._pending_checkpoint is not None:
                    self._finish_checkpoint_restore()
                self._refresh_ui()
            elif self.verifier.timed_out:
                self._log_debug(f"[APPLY] {self._format_apply_report(report)}", level="ERROR")
                self._fail("Scenario apply confirmation timed out")

    def _watchdog_tick(self) -> None:
        state = self.controller.state
        if state in (ExecutionState.APPLYING, ExecutionState.RESETTING):
            if self.verifier is not None and self.verifier.timed_out:
                self._fail("Scenario apply confirmation timed out; robots stopped")
            return
        if state not in (ExecutionState.RUNNING, ExecutionState.PAUSED):
            return
        if (
            self._last_snapshot_received_at is None
            or time.monotonic() - self._last_snapshot_received_at > 0.5
        ):
            self._fail("Fatal vision error: live world snapshot is stale; robots stopped")

    @staticmethod
    def _format_apply_report(report: ApplyReport) -> str:
        position = (
            "--" if report.maximum_position_error_mm is None else f"{report.maximum_position_error_mm:.1f} mm"
        )
        orientation = (
            "--"
            if report.maximum_orientation_error_rad is None
            else f"{report.maximum_orientation_error_rad:.3f} rad"
        )
        summary = (
            f"{report.confirmed}/{report.expected} robots confirmed, "
            f"max position error {position}, max orientation error {orientation}, "
            f"vision age {report.vision_age_ms or 0.0:.1f} ms, "
            f"stable snapshots {report.stable_snapshots}/3"
        )
        if report.mismatches:
            summary += f", mismatches: {', '.join(report.mismatches)}"
        return summary

    def _selected_planner(self) -> str:
        for planner, (radio, _role, _widget) in self.planner_rows.items():
            if radio.isChecked():
                return planner
        return ""

    def _run(self, planner: str) -> None:
        try:
            paths = self.controller.run(planner)
            execution_input = self.controller.execution_input
            assert execution_input is not None
            self.runtime.set_planner(execution_input.planner_classes[planner])
            self.runtime.start_execution(paths)
            self.canvas.paths = paths
            self.current_metrics = deepcopy(self.metric_templates)
            self.run_id = f"run-{datetime.now(UTC).strftime('%Y%m%dT%H%M%S')}-{uuid4().hex[:6]}"
            self.run_kind = "experiment"
            self.parent_run_id = None
            self.source_checkpoint_id = None
            self.run_started_at = time.perf_counter()
            self.recorded_boundaries.clear()
            self.recorder = ExperimentRecorder(execution_input.scenario.name, planner)
            self.recorder.record(
                "execution_metadata",
                run_id=self.run_id,
                run_kind=self.run_kind,
                parent_run_id=self.parent_run_id,
                checkpoint_id=self.source_checkpoint_id,
            )
            self.checkpoint_store = CheckpointStore(self.recorder.path)
            for metrics in self.current_metrics.values():
                metrics.start_execution()
            self.execution_timer.start()
            self._log_debug(
                f"Run {self.run_id} started with {planner}; "
                f"{len(paths)} robot command stream(s) active"
            )
            self._refresh_ui()
        except Exception as exc:
            self._fail(f"Cannot run planner: {exc}")

    def _run_or_continue(self) -> None:
        if self.controller.state is ExecutionState.PAUSED:
            self._continue()
        else:
            self._run(self._selected_planner())

    def _pause(self) -> None:
        try:
            self.runtime.pause_execution()
            self.controller.pause()
            self._refresh_ui()
        except Exception as exc:
            self._fail(f"Pause failed: {exc}")

    def _continue(self) -> None:
        try:
            self.runtime.continue_execution()
            self.controller.continue_run()
            self._stepping = False
            self.execution_timer.start()
            self._refresh_ui()
        except Exception as exc:
            self._fail(f"Continue failed: {exc}")

    def _previous_step(self) -> None:
        index = self.checkpoint_selector.currentIndex()
        if index > 0:
            self.checkpoint_selector.setCurrentIndex(index - 1)
            self._log_debug(
                f"[CHECKPOINT] selected previous checkpoint: "
                f"{self.checkpoint_selector.currentText()}"
            )

    def _step(self) -> None:
        try:
            self.runtime.step_execution()
            self._stepping = True
            self.execution_timer.start()
            self._refresh_ui()
        except Exception as exc:
            self._fail(f"Step failed: {exc}")

    def _execution_tick(self) -> None:
        try:
            if self.controller.state not in (ExecutionState.RUNNING, ExecutionState.PAUSED):
                return
            if self.controller.state is ExecutionState.PAUSED and not self._stepping:
                return
            robots = self.runtime.live_robots
            for metrics in self.current_metrics.values():
                metrics.observe_robots(robots)
            completed = self.runtime.execute_tick(robots)
            self.canvas.waypoint_indices = self.runtime.waypoint_indices
            self._record_transitions(self.runtime.last_waypoint_transitions)
            if self.runtime.step_completed:
                self._stepping = False
            if completed:
                self.execution_timer.stop()
                self.runtime.emergency_stop()
                self.controller.complete()
                self._finalize_run(completed=True)
            self._refresh_summary()
            self._refresh_ui()
        except Exception as exc:
            self._fail(f"Fatal execution error: {exc}")

    def _record_transitions(
        self, transitions: tuple[tuple[tuple[bool, int], int], ...]
    ) -> None:
        path_lengths = {
            (path.is_yellow, path.robot_id): len(path.points_mm)
            for path in self.runtime.active_paths
        }
        new = [
            (key, index)
            for key, index in transitions
            if (key[0], key[1], index) not in self.recorded_boundaries
            # The final waypoint of a path means that robot has finished, not
            # reached a resumable mid-run point, so it is not checkpoint-worthy.
            and index < path_lengths.get(key, index + 1) - 1
        ]
        if not new or self.checkpoint_store is None or self.controller.execution_input is None:
            return
        snapshot = self.runtime.world_snapshot
        if snapshot is None:
            return
        for key, index in new:
            self.recorded_boundaries.add((key[0], key[1], index))
        checkpoint = self._make_checkpoint(snapshot, new)
        self.checkpoint_store.append(checkpoint)
        self.checkpoints.append(checkpoint)
        elapsed_ms = float(checkpoint.metrics["elapsed_ms"])
        self.checkpoint_selector.addItem(
            f"{elapsed_ms:.0f} ms · {checkpoint.checkpoint_id} · "
            f"{checkpoint.triggers[0]['robot_key']}",
            checkpoint.checkpoint_id,
        )
        self.checkpoint_selector.setCurrentIndex(self.checkpoint_selector.count() - 1)
        trigger_text = ", ".join(
            f"{item['robot_key']} waypoint {item['reached_waypoint_index']}"
            for item in checkpoint.triggers
        )
        self._log_debug(
            f"[CHECKPOINT] {elapsed_ms:.1f} ms since run: {trigger_text}; "
            f"checkpoint saved as {checkpoint.checkpoint_id}"
        )

    def _make_checkpoint(
        self,
        snapshot: WorldSnapshot,
        transitions: list[tuple[tuple[bool, int], int]],
    ) -> CheckpointRecord:
        execution_input = self.controller.execution_input
        assert execution_input is not None
        robots = tuple(
            {
                "is_yellow": robot.isYellow,
                "robot_id": robot.robot_id,
                "pose": [robot.x, robot.y, robot.theta],
            }
            for robot in (*snapshot.yellow, *snapshot.blue)
            if robot is not None
        )
        indexes = {
            f"{'Y' if key[0] else 'B'}{key[1]}": index
            for key, index in self.runtime.waypoint_indices.items()
        }
        return CheckpointRecord.create(
            run_id=self.run_id,
            checkpoint_id=f"cp-{len(self.checkpoints) + 1:04d}",
            parent_run_id=self.parent_run_id,
            scenario_name=execution_input.scenario.name,
            scenario_hash=execution_input.content_hash,
            triggers=tuple(
                {
                    "robot_key": f"{'Y' if key[0] else 'B'}{key[1]}",
                    "reached_waypoint_index": index,
                }
                for key, index in transitions
            ),
            robots=robots,
            ball=(
                None
                if snapshot.ball is None
                else {"position_mm": [snapshot.ball.x, snapshot.ball.y]}
            ),
            waypoint_indexes=indexes,
            velocity_owner=self.controller.velocity_owner or "",
            path_ids={
                f"{'Y' if path.is_yellow else 'B'}{path.robot_id}": f"path-{path.robot_id}"
                for path in self.runtime.active_paths
            },
            metrics={
                "elapsed_ms": self._elapsed_ms(),
                "collisions": max(
                    (metric.number_of_collisions for metric in self.current_metrics.values()),
                    default=0,
                ),
            },
            state=self.controller.state.value,
        )

    def _reset(self) -> None:
        execution_input = self.controller.execution_input
        if execution_input is None:
            return
        try:
            self.emergency_stop(finalize=True)
            self.checkpoints.clear()
            self.checkpoint_selector.clear()
            self.recorded_boundaries.clear()
            self._log_debug("[CHECKPOINT] checkpoint list cleared by reset")
            self.controller.begin_reset()
            self.verifier = ScenarioApplyVerifier(execution_input.scenario)
            self.runtime.apply_scenario(execution_input.scenario, include_obstacles=True)
            self._refresh_ui()
        except Exception as exc:
            self._fail(f"Reset failed: {exc}")

    def _resume_checkpoint(self) -> None:
        checkpoint_id = self.checkpoint_selector.currentData()
        if not checkpoint_id:
            return
        checkpoint = next(
            (item for item in self.checkpoints if item.checkpoint_id == checkpoint_id), None
        )
        if checkpoint is None or self.controller.execution_input is None:
            return
        try:
            if checkpoint.scenario_hash != self.controller.execution_input.content_hash:
                raise ValueError("Checkpoint scenario does not match the loaded execution input")
            self.emergency_stop(finalize=True)
            self.controller.request_checkpoint_resume(checkpoint_id)
            restored = self._scenario_from_checkpoint(checkpoint)
            self._pending_checkpoint = checkpoint
            self.verifier = ScenarioApplyVerifier(restored)
            self.runtime.apply_scenario(restored, include_obstacles=True)
            self._refresh_ui()
        except Exception as exc:
            self._fail(f"Checkpoint restore failed: {exc}")

    def _scenario_from_checkpoint(self, checkpoint: CheckpointRecord) -> Scenario:
        execution_input = self.controller.execution_input
        assert execution_input is not None
        by_key = {
            (bool(item["is_yellow"]), int(item["robot_id"])): item["pose"]
            for item in checkpoint.robots
        }
        scenario = execution_input.scenario
        robots = [
            replace(
                robot,
                start_mm=tuple(by_key[(robot.is_yellow, robot.robot_id)][:2]),
                orientation_rad=float(by_key[(robot.is_yellow, robot.robot_id)][2]),
            )
            for robot in scenario.robots
        ]
        obstacles = [
            replace(
                obstacle,
                position_mm=tuple(by_key[(obstacle.is_yellow, obstacle.obstacle_id)][:2]),
            )
            for obstacle in scenario.obstacles
        ]
        ball = (
            None
            if checkpoint.ball is None
            else ScenarioBall(tuple(checkpoint.ball["position_mm"]))
        )
        return Scenario(
            scenario.name,
            robots=robots,
            obstacles=obstacles,
            ball=ball,
            schema_version=scenario.schema_version,
        )

    def _finish_checkpoint_restore(self) -> None:
        checkpoint = self._pending_checkpoint
        execution_input = self.controller.execution_input
        assert checkpoint is not None and execution_input is not None
        owner = checkpoint.velocity_owner
        self.controller.velocity_owner = None
        self.controller.selections_locked = False
        paths = self.controller.run(owner)
        indexes = {_parse_robot_key(key): value for key, value in checkpoint.waypoint_indexes.items()}
        self.runtime.start_execution(paths, waypoint_indices=indexes, paused=True)
        self.controller.pause()
        self.canvas.paths = paths
        self.canvas.waypoint_indices = indexes
        self.current_metrics = deepcopy(self.metric_templates)
        for metrics in self.current_metrics.values():
            metrics.start_execution()
        self.run_id = f"debug-{uuid4().hex[:8]}"
        self.run_kind = "debug_replay"
        self.parent_run_id = checkpoint.run_id
        self.source_checkpoint_id = checkpoint.checkpoint_id
        self.run_started_at = time.perf_counter()
        self.recorded_boundaries.clear()
        self.recorder = ExperimentRecorder(execution_input.scenario.name, owner)
        self.recorder.record(
            "checkpoint_restored",
            run_id=self.run_id,
            run_kind=self.run_kind,
            parent_run_id=self.parent_run_id,
            checkpoint_id=self.source_checkpoint_id,
        )
        self.checkpoint_store = CheckpointStore(self.recorder.path)
        self._pending_checkpoint = None
        self._refresh_ui()

    def emergency_stop(self, *, error: str | None = None, finalize: bool = True) -> None:
        self.execution_timer.stop()
        execution_input = self.controller.execution_input
        extra = ()
        if execution_input is not None:
            extra = tuple(
                (robot.is_yellow, robot.robot_id) for robot in execution_input.scenario.robots
            )
        stop_errors = self.runtime.emergency_stop(extra)
        message = error or ("; ".join(stop_errors) if stop_errors else None)
        self.controller.stop(error=message)
        if finalize:
            self._finalize_run(completed=False)
        log_message = message or "Execution stopped"
        self._log_debug(log_message, level="ERROR" if message else "INFO")
        if message:
            self._show_error_toast(message)
        self._refresh_ui()

    def _fail(self, message: str) -> None:
        self.emergency_stop(error=message)

    def _finalize_run(self, *, completed: bool) -> None:
        if not self.run_id:
            return
        now = datetime.now(UTC).isoformat()
        owner = self.controller.velocity_owner
        metrics = self.current_metrics.get(owner) if owner is not None else None
        if owner is not None and metrics is not None:
            metrics.finish(completed=completed)
            common = {
                "run_id": self.run_id,
                "scenario": self.controller.execution_input.scenario.name
                if self.controller.execution_input
                else "",
                "planner": owner,
                "role": "EXECUTING",
                "run_kind": self.run_kind,
                "parent_run_id": self.parent_run_id or "",
                "checkpoint_id": self.source_checkpoint_id or "",
                "state": self.controller.state.value,
                "timestamp": now,
            }
            self.result_rows_a.append({**common, **metrics.row("a")})
            self.result_rows_b.append({**common, **metrics.row("b")})
        if self.recorder is not None and not self.recorder.closed:
            self.recorder.finish(completed=completed)
            self.recorder.close()
        self.recorder = None
        self.run_id = ""
        self._populate_table(self.result_a_table, self.result_rows_a)
        self._populate_table(self.result_b_table, self.result_rows_b)

    def _vision_test(self) -> None:
        snapshot = self.runtime.world_snapshot
        if snapshot is None or self._last_snapshot_received_at is None:
            self._log_debug(
                "[VISION] FAIL · no complete grSim vision snapshot received", level="ERROR"
            )
            return

        age_s = time.monotonic() - self._last_snapshot_received_at
        yellow = sum(robot is not None for robot in snapshot.yellow)
        blue = sum(robot is not None for robot in snapshot.blue)
        ball = "yes" if snapshot.ball is not None else "no"
        if age_s > 0.5:
            self._log_debug(
                f"[VISION] FAIL · grSim vision stale ({age_s:.2f}s old) · "
                f"yellow={yellow} · blue={blue} · ball={ball}",
                level="ERROR",
            )
            return

        self._log_debug(
            f"[VISION] PASS · live grSim vision ({age_s * 1000.0:.0f}ms old) · "
            f"frame={snapshot.frame_number} · yellow={yellow} · "
            f"blue={blue} · ball={ball}"
        )

    def _motion_test(self) -> None:
        if self.controller.state not in (
            ExecutionState.NO_SCENARIO,
            ExecutionState.SCENARIO_LOADED,
            ExecutionState.READY,
        ):
            return
        answer = QMessageBox.warning(
            self,
            "Motion feedback test",
            "Rotate yellow robot 0 at 0.5 rad/s for five seconds? The robot will move.",
            QMessageBox.Yes | QMessageBox.Cancel,
        )
        if answer != QMessageBox.Yes:
            return
        robot = self.runtime.live_robots.get((True, 0))
        if robot is None:
            QMessageBox.warning(self, "Motion test unavailable", "Yellow robot 0 is not visible.")
            return
        self._motion_test_key = (True, 0)
        self._motion_test_start_theta = robot.orientation_rad
        self._motion_test_started_at = time.monotonic()
        self._motion_test_samples = 0
        if not self._send_motion_test_command():
            return
        self.motion_command_timer.start()
        self.motion_stop_timer.start(5000)
        self._log_debug("[MOTION] command streaming at 20 Hz; awaiting feedback")

    def _send_motion_test_command(self) -> bool:
        key = self._motion_test_key
        if key is None:
            self.motion_command_timer.stop()
            return False
        is_yellow, robot_id = key
        try:
            self.runtime.send_robot_command(
                RobotCommand(robot_id, w=0.5, isYellow=is_yellow)
            )
            return True
        except Exception as exc:
            self.motion_command_timer.stop()
            self.motion_stop_timer.stop()
            try:
                self.runtime.stop_robot(is_yellow, robot_id)
            except Exception:
                pass
            self._motion_test_key = None
            self._log_debug(
                f"[MOTION] command stream failed: {exc}; stop attempted", level="ERROR"
            )
            return False

    def _finish_motion_test(self) -> None:
        self.motion_command_timer.stop()
        key = self._motion_test_key
        try:
            if key is not None:
                self.runtime.stop_robot(*key)
            robot = self.runtime.live_robots.get(key) if key is not None else None
            if robot is None or self._motion_test_start_theta is None:
                self._log_debug(
                    "[MOTION] feedback missing; stop command sent", level="ERROR"
                )
            else:
                delta = abs(
                    atan2(
                        sin(robot.orientation_rad - self._motion_test_start_theta),
                        cos(robot.orientation_rad - self._motion_test_start_theta),
                    )
                )
                elapsed = max(
                    0.001,
                    time.monotonic() - (self._motion_test_started_at or time.monotonic()),
                )
                measured = delta / elapsed
                passed = self._motion_test_samples > 0 and delta >= 0.25
                self._log_debug(
                    f"[MOTION] {'PASS' if passed else 'FAIL'} · command 0.500 rad/s · "
                    f"measured {measured:.3f} rad/s · {self._motion_test_samples} samples · "
                    f"stop command sent",
                    level="INFO" if passed else "ERROR",
                )
        except Exception as exc:
            self._log_debug(f"[MOTION] stop failed: {exc}", level="ERROR")
        finally:
            self._motion_test_key = None
            self._motion_test_start_theta = None
            self._motion_test_started_at = None
            self._motion_test_samples = 0

    def _refresh_summary(self) -> None:
        plans = sum(
            len(metric.planner_execution_samples_ms) for metric in self.current_metrics.values()
        )
        collisions = max(
            (metric.number_of_collisions for metric in self.current_metrics.values()), default=0
        )
        self.summary.setText(
            f"run {self.run_id or '--'} | elapsed {self._elapsed_ms():.1f} ms | "
            f"plans {plans} | collisions {collisions} | "
            f"owner {self.controller.velocity_owner or '--'}"
        )

    def _elapsed_ms(self) -> float:
        if self.run_started_at is None:
            return 0.0
        return (time.perf_counter() - self.run_started_at) * 1000.0

    def _refresh_ui(self) -> None:
        state = self.controller.state
        self.state_badge.setText(state.value)
        colors = {
            ExecutionState.RUNNING: "#2e7d32",
            ExecutionState.READY: "#1565c0",
            ExecutionState.PAUSED: "#ef6c00",
            ExecutionState.ERROR: "#b71c1c",
        }
        self.state_badge.setStyleSheet(
            f"font-size:16px;font-weight:800;padding:8px;color:white;"
            f"background:{colors.get(state, '#455a64')};"
        )
        loadable = state in (
            ExecutionState.NO_SCENARIO,
            ExecutionState.SCENARIO_LOADED,
            ExecutionState.READY,
        )
        self.refresh_button.setEnabled(loadable)
        self.load_button.setEnabled(loadable)
        self.apply_button.setEnabled(state is ExecutionState.SCENARIO_LOADED)
        self.unload_button.setEnabled(
            state
            in (
                ExecutionState.SCENARIO_LOADED,
                ExecutionState.READY,
                ExecutionState.COMPLETED,
                ExecutionState.STOPPED,
                ExecutionState.ERROR,
            )
        )
        self.pause_button.setEnabled(state is ExecutionState.RUNNING)
        is_paused = state is ExecutionState.PAUSED
        self.step_button.setEnabled(is_paused and not self._stepping)
        self.previous_step_button.setEnabled(self.checkpoint_selector.currentIndex() > 0)
        selected_planner = self._selected_planner()
        debug_nodes, debug_edges = self.debug_geometry.get(selected_planner, ((), ()))
        self.canvas.debug_nodes = debug_nodes
        self.canvas.debug_edges = debug_edges
        self.map_layer_toggle.setEnabled(bool(debug_nodes or debug_edges))
        selected_paths = (
            ()
            if self.controller.execution_input is None
            else self.controller.execution_input.paths_by_planner.get(selected_planner, ())
        )
        self.run_button.setText("Continue" if is_paused else "Run")
        self.run_button.setEnabled(
            (
                state is ExecutionState.READY
                and not self.controller.selections_locked
                and bool(selected_paths)
            )
            or (is_paused and not self._stepping)
        )
        self.resume_checkpoint_button.setEnabled(
            bool(self.checkpoints)
            and state
            in (
                ExecutionState.PAUSED,
                ExecutionState.COMPLETED,
                ExecutionState.STOPPED,
                ExecutionState.ERROR,
            )
        )
        self.reset_button.setEnabled(
            state
            in (
                ExecutionState.READY,
                ExecutionState.RUNNING,
                ExecutionState.PAUSED,
                ExecutionState.COMPLETED,
                ExecutionState.STOPPED,
                ExecutionState.ERROR,
            )
        )
        self.motion_test_button.setEnabled(loadable)
        for planner, (radio, role, widget) in self.planner_rows.items():
            is_owner = planner == self.controller.velocity_owner
            failure = self.planner_failures.get(planner)
            radio.setEnabled(state is ExecutionState.READY and not self.controller.selections_locked)
            if is_owner:
                role.setText("EXECUTING")
                widget.setStyleSheet("background:#2e7d32;color:white;")
            elif failure:
                role.setText("ERROR")
                role.setToolTip(failure)
                widget.setStyleSheet("background:#b71c1c;color:white;")
            else:
                role.setText("INACTIVE")
                role.setToolTip("")
                widget.setStyleSheet("")
        self.canvas.state = state
        self.canvas.update()
        self._refresh_summary()

    @staticmethod
    def _populate_table(table: QTableWidget, rows: list[dict]) -> None:
        table.setSortingEnabled(False)
        if not rows:
            table.setRowCount(0)
            table.setSortingEnabled(True)
            return
        columns = list(rows[0])
        table.setColumnCount(len(columns))
        table.setHorizontalHeaderLabels(columns)
        table.setRowCount(len(rows))
        for row_index, row in enumerate(rows):
            for column_index, column in enumerate(columns):
                item = QTableWidgetItem(str(row.get(column, "")))
                if row.get("role") == "EXECUTING":
                    item.setBackground(QColor("#2e7d32"))
                    item.setForeground(QColor("#ffffff"))
                table.setItem(row_index, column_index, item)
        table.setSortingEnabled(True)

    def eventFilter(self, obj, event) -> bool:
        if (
            obj in (self.result_a_table, self.result_b_table)
            and event.type() == QEvent.KeyPress
            and event.key() in (Qt.Key_Delete, Qt.Key_Backspace)
        ):
            self._delete_selected_result_rows(obj)
            return True
        return super().eventFilter(obj, event)

    def _show_result_table_menu(self, table: QTableWidget, pos) -> None:
        if not table.selectionModel().hasSelection():
            return
        menu = QMenu(self)
        delete_action = menu.addAction(
            "Delete selected run" if self._selected_result_row_count(table) == 1 else
            "Delete selected runs"
        )
        chosen = menu.exec(table.viewport().mapToGlobal(pos))
        if chosen is delete_action:
            self._delete_selected_result_rows(table)

    @staticmethod
    def _selected_result_row_count(table: QTableWidget) -> int:
        return len({index.row() for index in table.selectedIndexes()})

    def _delete_selected_result_rows(self, table: QTableWidget) -> None:
        columns = [table.horizontalHeaderItem(i).text() for i in range(table.columnCount())]
        if "run_id" not in columns or "planner" not in columns:
            return
        run_id_column = columns.index("run_id")
        planner_column = columns.index("planner")
        keys = {
            (
                table.item(row, run_id_column).text(),
                table.item(row, planner_column).text(),
            )
            for row in {index.row() for index in table.selectedIndexes()}
        }
        if not keys:
            return
        self.result_rows_a = [
            row for row in self.result_rows_a if (row.get("run_id"), row.get("planner")) not in keys
        ]
        self.result_rows_b = [
            row for row in self.result_rows_b if (row.get("run_id"), row.get("planner")) not in keys
        ]
        self._populate_table(self.result_a_table, self.result_rows_a)
        self._populate_table(self.result_b_table, self.result_rows_b)
        self._log_debug(f"[RESULTS] deleted {len(keys)} run entry(ies) from the results tables")

    def _export_table(self, format_name: str) -> None:
        rows = self.result_rows_a if format_name == "a" else self.result_rows_b
        if not rows:
            QMessageBox.warning(self, "No results", "Complete or stop a run first.")
            return
        destination, _ = QFileDialog.getSaveFileName(
            self,
            f"Export Result {format_name.upper()}",
            f"exports/results_{format_name}.csv",
            "CSV files (*.csv)",
        )
        if not destination:
            return
        path = Path(destination)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    def shutdown(self) -> None:
        self.watchdog_timer.stop()
        self.motion_command_timer.stop()
        self.motion_stop_timer.stop()
        if self._motion_test_key is not None:
            try:
                self.runtime.stop_robot(*self._motion_test_key)
            except Exception:
                pass
        if self.controller.state in (
            ExecutionState.APPLYING,
            ExecutionState.READY,
            ExecutionState.RUNNING,
            ExecutionState.PAUSED,
            ExecutionState.RESETTING,
        ):
            self.emergency_stop(error="Application shutdown")


def _parse_robot_key(value: str) -> tuple[bool, int]:
    if len(value) < 2 or value[0] not in "YB":
        raise ValueError(f"Invalid robot key: {value}")
    return value[0] == "Y", int(value[1:])
