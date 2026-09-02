"""PySide research console for scenario-based path-planner experiments."""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass, replace
from math import cos, hypot, pi, sin
from pathlib import Path

import yaml
from PySide6.QtCore import QPointF, QRectF, Qt, QThread, QTimer, Signal
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QSplitter,
    QTabWidget,
    QToolTip,
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
    VISIBILITY_POLYGON_SIDES,
)
from research_sdk.network.ssl_sockets import Vision, grSimVision
from research_sdk.planners.common import StepRecorder
from research_sdk.ui.execution.controller import ExecutionState
from research_sdk.ui.execution.page import ExecutionConsolePage
from research_sdk.ui.runtime import (
    LiveRobot,
    PlannedRobotPath,
    ResearchRuntime,
    live_world_from_vision_packet,
    planner_key,
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
    RunMetrics,
    SessionController,
    SessionState,
    discover_planners,
    export_planner_results,
    planner_debug_geometry,
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
        self.planner_key: str | None = None
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
            for obstacle in self.scenario.obstacles_for(self.planner_key):
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

    def _plan_failed_for(self, robot_id: int, is_yellow: bool) -> bool:
        """True if the most recent ``plan()`` call marked this robot's path
        ``failed`` (no route found) -- drawing code uses this to flag the
        robot red and skip its path line instead of pretending it's fine."""
        return any(
            path.robot_id == robot_id and path.is_yellow == is_yellow and path.failed
            for path in self.paths
        )

    def _draw_robot(self, painter: QPainter, robot: ScenarioRobot, selected: bool) -> None:
        start = self._to_screen(robot.start_mm)
        radius = ROBOT_RADIUS_MM * self._field_rect().width() / FIELD_LENGTH_MM
        failed = self._plan_failed_for(robot.robot_id, robot.is_yellow)
        color = QColor("#e53935") if failed else QColor("#ffd740" if robot.is_yellow else "#42a5f5")
        painter.setBrush(QColor(color.red(), color.green(), color.blue(), 75))
        painter.setPen(QPen(QColor("white") if selected else color, 3 if selected else 2, Qt.DashLine))
        painter.drawEllipse(start, radius, radius)
        if robot.target_mm is not None:
            target = self._to_screen(robot.target_mm)
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
            target = existing.target_mm if existing is not None else None
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
                    planner_keys=((self.planner_key,) if self.planner_key else ()),
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


@dataclass(frozen=True, slots=True)
class PlannerPreview:
    paths: tuple[PlannedRobotPath, ...]
    # None means this planner has no real map-vs-search split available
    # (no StepRecorder support) -- see _plan()'s comment for why that must
    # not be papered over with a fabricated number.
    map_time_ms: float | None
    planning_time_ms: float
    nodes: tuple[tuple[float, float], ...] = ()
    edges: tuple[tuple[tuple[float, float], tuple[float, float]], ...] = ()


class ScenarioPlannerCanvas(FieldCanvas):
    """Interactive field editor and layered preview for the new workspace."""

    selection_changed = Signal()
    edit_committed = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.setMouseTracking(True)
        self.mode = "starting_pos"
        self.selected_obstacle: int | None = None
        self.cursor_mm: tuple[float, float] | None = None
        self.show_map_layer = True
        self.show_robot_layer = True
        self.show_shape_layer = True
        self.show_path_layer = True
        self.preview = PlannerPreview((), 0.0, 0.0)
        self.planner_label = ""

    def set_preview(self, preview: PlannerPreview, planner_label: str) -> None:
        self.preview = preview
        self.planner_label = planner_label
        self.set_paths(preview.paths)

    def clear_preview(self) -> None:
        self.preview = PlannerPreview((), 0.0, 0.0)
        self.clear_paths()

    def mouseMoveEvent(self, event) -> None:
        if self._field_rect().contains(event.position()):
            self.cursor_mm = self._to_world(event.position())
            path = self._nearest_path(event.position())
            if path is not None:
                length = sum(
                    hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(path.points_mm, path.points_mm[1:])
                )
                QToolTip.showText(
                    event.globalPosition().toPoint(),
                    f"{self.planner_label}\nRobot {path.robot_id}\n"
                    f"Length: {length:.1f} mm\nPlan: {self.preview.planning_time_ms:.3f} ms",
                    self,
                )
        else:
            self.cursor_mm = None
        self.update()

    def leaveEvent(self, event) -> None:
        self.cursor_mm = None
        self.update()
        super().leaveEvent(event)

    def mousePressEvent(self, event) -> None:
        if self.scenario is None or not self._field_rect().contains(event.position()):
            return
        selected_path = self._nearest_path(event.position())
        if selected_path is not None:
            for index, robot in enumerate(self.scenario.robots):
                if (robot.is_yellow, robot.robot_id) == (
                    selected_path.is_yellow,
                    selected_path.robot_id,
                ):
                    self.selected_robot = index
                    self.selected_obstacle = None
                    self.selection_changed.emit()
                    self.update()
                    return
        point = self._to_world(event.position())
        robot_index = self._nearest_robot(event.position())
        obstacle_index = self._nearest_scenario_obstacle(event.position())

        if self.mode == "obstacles":
            if robot_index is not None:
                robot = self.scenario.robots.pop(robot_index)
                self.scenario.set_obstacle(
                    ScenarioObstacle(
                        robot.robot_id,
                        robot.is_yellow,
                        robot.start_mm,
                        ROBOT_RADIUS_MM,
                    )
                )
                self.selected_robot = None
                self.selected_obstacle = next(
                    i
                    for i, obstacle in enumerate(self.scenario.obstacles)
                    if (obstacle.is_yellow, obstacle.obstacle_id)
                    == (robot.is_yellow, robot.robot_id)
                )
                self._commit_edit("Robot converted to obstacle")
                return
            if obstacle_index is not None:
                self.selected_obstacle = obstacle_index
                self.selected_robot = None
                self.selection_changed.emit()
                self.update()
                return
            if self.selected_obstacle is not None:
                obstacle = self.scenario.obstacles[self.selected_obstacle]
                self.scenario.obstacles[self.selected_obstacle] = replace(
                    obstacle, position_mm=point
                )
                self._commit_edit("Obstacle position changed")
                return

        if self.mode == "starting_pos":
            if obstacle_index is not None:
                obstacle = self.scenario.obstacles.pop(obstacle_index)
                self.selected_robot = self.scenario.set_robot(
                    ScenarioRobot(
                        obstacle.obstacle_id,
                        obstacle.is_yellow,
                        obstacle.position_mm,
                        None,
                    )
                )
                self.selected_obstacle = None
                self._commit_edit("Starting-position robot selected")
                return
            if robot_index is not None:
                self.selected_robot = robot_index
                self.selected_obstacle = None
                self.selection_changed.emit()
                self.update()
                return
            if self.selected_robot is not None:
                robot = self.scenario.robots[self.selected_robot]
                self.scenario.robots[self.selected_robot] = replace(robot, start_mm=point)
                self._commit_edit("Starting position changed")
                return

        if self.mode == "target_pos" and self.selected_robot is not None:
            robot = self.scenario.robots[self.selected_robot]
            self.scenario.robots[self.selected_robot] = replace(robot, target_mm=point)
            self._commit_edit("Target position changed")

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

        if self.show_map_layer:
            self._draw_debug_map(painter)
        if self.scenario is not None:
            for index, obstacle in enumerate(self.scenario.obstacles):
                self._draw_planner_obstacle(painter, obstacle, index)
            for index, robot in enumerate(self.scenario.robots):
                self._draw_planned_robot(painter, robot, index)
        if self.show_path_layer:
            self._draw_paths(painter)
        if self.cursor_mm is not None:
            self._draw_cursor_guides(painter)
        if self.planner_label:
            painter.setPen(QColor("#f5f5f5"))
            painter.drawText(field.adjusted(8, 8, -8, -8), Qt.AlignTop, self.planner_label)

    def _draw_planner_obstacle(
        self, painter: QPainter, obstacle: ScenarioObstacle, index: int
    ) -> None:
        centre = self._to_screen(obstacle.position_mm)
        scale = self._field_rect().width() / FIELD_LENGTH_MM
        radius = obstacle.radius_mm * scale
        if self.show_shape_layer:
            inflated = (obstacle.radius_mm + ROBOT_RADIUS_MM) * scale
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(QColor(255, 138, 128, 150), 2, Qt.DashLine))
            if "Visibility" in self.planner_label:
                sides = VISIBILITY_POLYGON_SIDES
                vertex_radius = inflated / cos(pi / sides)
                polygon = [
                    centre
                    + QPointF(
                        vertex_radius * cos(2 * pi * side / sides),
                        -vertex_radius * sin(2 * pi * side / sides),
                    )
                    for side in range(sides)
                ]
                for first, second in zip(polygon, polygon[1:] + polygon[:1]):
                    painter.drawLine(first, second)
            else:
                painter.drawEllipse(centre, inflated, inflated)
        painter.setBrush(QColor(220, 70, 70, 180) if self.show_robot_layer else Qt.NoBrush)
        painter.setPen(
            QPen(QColor("white") if index == self.selected_obstacle else QColor("#ff8a80"), 3)
        )
        painter.drawEllipse(centre, radius, radius)
        painter.drawLine(centre + QPointF(-radius, -radius), centre + QPointF(radius, radius))
        painter.drawLine(centre + QPointF(-radius, radius), centre + QPointF(radius, -radius))
        painter.drawText(centre + QPointF(-8, -radius - 5), f"O{obstacle.obstacle_id}")

    def _draw_planned_robot(
        self, painter: QPainter, robot: ScenarioRobot, index: int
    ) -> None:
        start = self._to_screen(robot.start_mm)
        radius = ROBOT_RADIUS_MM * self._field_rect().width() / FIELD_LENGTH_MM
        failed = self._plan_failed_for(robot.robot_id, robot.is_yellow)
        color = QColor("#e53935") if failed else QColor("#ffd740" if robot.is_yellow else "#42a5f5")
        painter.setBrush(
            QColor(color.red(), color.green(), color.blue(), 100)
            if self.show_robot_layer
            else Qt.NoBrush
        )
        painter.setPen(QPen(QColor("white") if index == self.selected_robot else color, 3))
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

    def _draw_debug_map(self, painter: QPainter) -> None:
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor(144, 202, 249, 120), 1))
        for first, second in self.preview.edges:
            painter.drawLine(self._to_screen(first), self._to_screen(second))
        painter.setBrush(QColor(144, 202, 249, 170))
        for node in self.preview.nodes:
            painter.drawEllipse(self._to_screen(node), 2.5, 2.5)

    def _draw_cursor_guides(self, painter: QPainter) -> None:
        assert self.cursor_mm is not None
        field = self._field_rect()
        point = self._to_screen(self.cursor_mm)
        painter.setPen(QPen(QColor(255, 255, 255, 128), 1, Qt.DashLine))
        painter.drawLine(field.left(), point.y(), field.right(), point.y())
        painter.drawLine(point.x(), field.top(), point.x(), field.bottom())
        label = f"x {self.cursor_mm[0]:.0f} mm   y {self.cursor_mm[1]:.0f} mm"
        painter.setPen(QColor("#ffffff"))
        painter.drawText(point + QPointF(12, -12), label)

    def _nearest_scenario_obstacle(self, point: QPointF) -> int | None:
        if self.scenario is None or not self.scenario.obstacles:
            return None
        distance, index = min(
            (
                (self._to_screen(obstacle.position_mm) - point).manhattanLength(),
                index,
            )
            for index, obstacle in enumerate(self.scenario.obstacles)
        )
        return index if distance < 35 else None

    def _nearest_path(self, point: QPointF) -> PlannedRobotPath | None:
        best: tuple[float, PlannedRobotPath] | None = None
        for path in self.preview.paths:
            for first, second in zip(path.points_mm, path.points_mm[1:]):
                a = self._to_screen(first)
                b = self._to_screen(second)
                distance = _point_segment_distance(point, a, b)
                if best is None or distance < best[0]:
                    best = (distance, path)
        return best[1] if best is not None and best[0] < 10 else None

    def _commit_edit(self, message: str) -> None:
        self.clear_preview()
        self.scenario_changed.emit()
        self.selection_changed.emit()
        self.edit_committed.emit(message)
        self.update()


def _point_segment_distance(point: QPointF, first: QPointF, second: QPointF) -> float:
    dx = second.x() - first.x()
    dy = second.y() - first.y()
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-9:
        return hypot(point.x() - first.x(), point.y() - first.y())
    ratio = ((point.x() - first.x()) * dx + (point.y() - first.y()) * dy) / length_sq
    ratio = max(0.0, min(1.0, ratio))
    nearest_x = first.x() + ratio * dx
    nearest_y = first.y() + ratio * dy
    return hypot(point.x() - nearest_x, point.y() - nearest_y)


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


class ScenarioPlannerPage(QWidget):
    """Preview-only scenario editor specified by UI Redesign Plan v1."""

    def __init__(self, runtime: ResearchRuntime, store: ScenarioStore) -> None:
        super().__init__()
        self.runtime = runtime
        self.store = store
        self.scenario: Scenario | None = None
        self.scenario_path: Path | None = None
        self.previews: dict[str, PlannerPreview] = {}
        self.metrics: dict[str, RunMetrics] = {}
        self.canvas = ScenarioPlannerCanvas()
        self._build_ui()
        self._refresh_files()
        self._refresh_inspectors()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        title = QLabel("Plan a Scenario")
        title.setStyleSheet("font-size: 18px; font-weight: 700;")
        root.addWidget(title)

        tools = QHBoxLayout()
        self.tool_buttons: dict[str, QPushButton] = {}
        for key, label in (
            ("obstacles", "△  Obstacles"),
            ("starting_pos", "○  Starting position"),
            ("target_pos", "×  Target position"),
        ):
            button = QPushButton(label)
            button.setCheckable(True)
            button.clicked.connect(lambda checked=False, mode=key: self._select_tool(mode))
            self.tool_buttons[key] = button
            tools.addWidget(button)
        tools.addStretch(1)
        root.addLayout(tools)
        self.tool_buttons["starting_pos"].setChecked(True)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.addWidget(self._build_plans_panel())
        right_layout.addWidget(self._build_inspector("Obstacles", obstacle=True))
        right_layout.addWidget(self._build_inspector("Planned robots", obstacle=False))
        right_layout.addStretch(1)

        splitter = QSplitter()
        splitter.addWidget(self.canvas)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 1)
        splitter.setSizes((900, 330))
        root.addWidget(splitter, 1)

        bottom = QHBoxLayout()
        snapshot = QPushButton("Export snapshot")
        export = QPushButton("Export result A")
        snapshot.clicked.connect(self._export_snapshot)
        export.clicked.connect(self._export_results)
        bottom.addWidget(snapshot)
        bottom.addWidget(export)
        bottom.addStretch(1)
        self.layer_buttons: dict[str, QPushButton] = {}
        for key, label in (
            ("map", "Map"),
            ("robots", "Robots"),
            ("shape", "Obstacle shape"),
            ("path", "Path"),
        ):
            button = QPushButton(label)
            button.setCheckable(True)
            button.setChecked(True)
            button.toggled.connect(lambda checked, layer=key: self._toggle_layer(layer, checked))
            self.layer_buttons[key] = button
            bottom.addWidget(button)
        root.addLayout(bottom)

        self.status = QLabel("Load from grSim or create a blank scenario")
        self.status.setWordWrap(True)
        root.addWidget(self.status)

        self.canvas.selection_changed.connect(self._refresh_inspectors)
        self.canvas.edit_committed.connect(self._scenario_edited)

    def _build_plans_panel(self) -> QGroupBox:
        panel = QGroupBox("Plans")
        layout = QGridLayout(panel)
        load_grsim = QPushButton("Load From grSim")
        save = QPushButton("Save Scenario")
        self.file_selector = QComboBox()
        load = QPushButton("Load")
        add = QPushButton("Add New")
        delete = QPushButton("Delete")
        self.planner_selector = QComboBox()
        for label, planner_cls in discover_planners().items():
            self.planner_selector.addItem(label, planner_cls)
        self.plan_button = QPushButton("Plan")
        self.clear_button = QPushButton("Clear")
        self.map_time = QLabel("Map: -- ms")
        self.plan_time = QLabel("Plan: -- ms")

        layout.addWidget(load_grsim, 0, 0)
        layout.addWidget(save, 0, 1)
        layout.addWidget(self.file_selector, 1, 0)
        layout.addWidget(load, 1, 1)
        layout.addWidget(self.planner_selector, 2, 0)
        layout.addWidget(add, 2, 1)
        layout.addWidget(self.plan_button, 3, 0)
        layout.addWidget(self.clear_button, 3, 1)
        layout.addWidget(delete, 4, 1)
        layout.addWidget(self.map_time, 5, 0)
        layout.addWidget(self.plan_time, 5, 1)

        load_grsim.clicked.connect(self._load_from_grsim)
        save.clicked.connect(self._save)
        load.clicked.connect(self._load)
        add.clicked.connect(self._add_new)
        delete.clicked.connect(self._delete)
        self.plan_button.clicked.connect(self._plan)
        self.clear_button.clicked.connect(self._clear_preview)
        self.planner_selector.currentIndexChanged.connect(self._planner_changed)
        return panel

    def _build_inspector(self, title: str, *, obstacle: bool) -> QGroupBox:
        panel = QGroupBox(title)
        layout = QVBoxLayout(panel)
        count = QLabel("n: 0")
        listing = QListWidget()
        layout.addWidget(count)
        layout.addWidget(listing)
        if obstacle:
            self.obstacle_count = count
            self.obstacle_list = listing
            listing.currentRowChanged.connect(self._obstacle_row_selected)
        else:
            self.robot_count = count
            self.robot_list = listing
            listing.currentRowChanged.connect(self._robot_row_selected)
        return panel

    def _select_tool(self, mode: str) -> None:
        if mode == "target_pos" and self.canvas.selected_robot is None:
            self.status.setText("Select a starting-position robot before setting a target")
            self._refresh_controls()
            return
        self.canvas.mode = mode
        for key, button in self.tool_buttons.items():
            button.blockSignals(True)
            button.setChecked(key == mode)
            button.blockSignals(False)

    def _load_from_grsim(self) -> None:
        snapshot = self.runtime.world_snapshot
        if snapshot is None:
            QMessageBox.warning(self, "No snapshot", "No complete grSim snapshot is available yet.")
            return
        obstacles = [
            ScenarioObstacle(
                robot.robot_id,
                robot.isYellow,
                robot.position,
                ROBOT_RADIUS_MM,
            )
            for robot in (*snapshot.yellow, *snapshot.blue)
            if robot is not None
        ]
        self.scenario = Scenario(
            f"snapshot_{int(time.time())}",
            obstacles=obstacles,
            ball=ScenarioBall(snapshot.ball.position) if snapshot.ball is not None else None,
        )
        self.scenario_path = None
        self._install_scenario("Snapshot loaded; select a starting-position robot")

    def _add_new(self) -> None:
        self.scenario = Scenario("untitled")
        self.scenario_path = None
        self._install_scenario("Blank scenario created")

    def _install_scenario(self, message: str) -> None:
        self.previews.clear()
        self.metrics.clear()
        self.canvas.set_scenario(self.scenario)
        self.canvas.selected_obstacle = None
        self.canvas.clear_preview()
        self._refresh_inspectors()
        self._refresh_controls()
        self.status.setText(message)

    def _save(self) -> None:
        if self.scenario is None:
            QMessageBox.warning(self, "No scenario", "Load or create a scenario first.")
            return
        errors = self.scenario.validation_errors(require_robot=False)
        if errors:
            QMessageBox.warning(self, "Incomplete scenario", "\n".join(errors))
            return
        name, accepted = QInputDialog.getText(
            self, "Save scenario", "Scenario name", text=self.scenario.name
        )
        if not accepted or not name.strip():
            return
        try:
            self.scenario.name = name.strip()
            self.scenario.schema_version = 3
            self.scenario_path = self.store.save(self.scenario)
            self._refresh_files()
            self.status.setText(f"Saved {self.scenario_path.name}")
        except Exception as exc:
            QMessageBox.critical(self, "Cannot save scenario", str(exc))

    def _load(self) -> None:
        path_text = self.file_selector.currentData()
        if not path_text:
            return
        try:
            self.scenario_path = Path(path_text)
            self.scenario = self.store.load(self.scenario_path)
            self._install_scenario(f"Loaded {self.scenario_path.name}")
        except Exception as exc:
            QMessageBox.critical(self, "Cannot load scenario", str(exc))

    def _delete(self) -> None:
        path_text = self.file_selector.currentData()
        if not path_text:
            return
        path = Path(path_text)
        answer = QMessageBox.question(
            self, "Delete scenario", f"Delete {path.name}? This cannot be undone."
        )
        if answer != QMessageBox.Yes:
            return
        try:
            path.unlink()
            if self.scenario_path == path:
                self.scenario_path = None
            self._refresh_files()
            self.status.setText(f"Deleted {path.name}")
        except Exception as exc:
            QMessageBox.critical(self, "Cannot delete scenario", str(exc))

    def _refresh_files(self) -> None:
        self.file_selector.clear()
        for path in self.store.list_paths():
            self.file_selector.addItem(path.stem, str(path))

    def _plan(self) -> None:
        """Plan with the single currently-selected planner only.

        Previously looped over every planner in ``self.planner_selector`` to
        build a preview for all of them at once -- fine as a pure offline
        preview, but running every planner back-to-back against a live grSim
        connection turned out to fight the execution model's single
        ``velocity_owner`` assumption (see ``ExecutionController.run()``,
        ``ui/execution/controller.py``) in practice. A 3-way comparison still
        exists, offline and grSim-free: ``scripts/demo_planners.py``.
        """
        if self.scenario is None:
            QMessageBox.warning(self, "No scenario", "Load or create a scenario first.")
            return
        errors = self.scenario.validation_errors()
        if errors:
            QMessageBox.warning(self, "Cannot plan", "\n".join(errors))
            return

        label = self.planner_selector.currentText()
        planner_cls = self.planner_selector.currentData()
        recorder = StepRecorder()
        self.runtime.set_planner(planner_cls, record=recorder)
        failure_note = ""
        try:
            paths = self.runtime.plan(self.scenario)
        except Exception as exc:
            paths = ()
            failure_note = f" · {label}: {exc}"
        else:
            if self.runtime.last_plan_failures:
                failed_ids = ", ".join(
                    f"{'Y' if p.is_yellow else 'B'}{p.robot_id}" for p in paths if p.failed
                )
                failure_note = (
                    f" · {self.runtime.last_plan_failures} robot(s) found no route "
                    f"({failed_ids}) -- flagged red, held in place"
                )
        planning_ms = sum(self.runtime.last_plan_durations_ms)
        nodes, edges = planner_debug_geometry(label, recorder, tuple(self.scenario.obstacles))
        if recorder.map_time_ms is not None:
            # A full build happened this call and logged into StepRecorder
            # -- all three planners support this now (VisibilityGraph, PRM,
            # and VoronoiDijkstraPlanner via a coarser two-timestamp split,
            # see voronoi_dijkstra.py's plan() docstring) -- a real
            # map-vs-search split.
            map_ms: float | None = recorder.map_time_ms
            search_ms = (
                recorder.search_time_ms
                if recorder.search_time_ms is not None
                else max(0.0, planning_ms - map_ms)
            )
        else:
            # This call took a direct-line-of-sight or reused-previous-path
            # shortcut and never built anything, so there's genuinely no map
            # cost to report -- not a planner limitation. (Voronoi used to
            # have no recorder support at all, and this branch used to fall
            # back to timing _debug_geometry()'s *separate*, cheaper
            # debug-only map build and subtracting it from the total, which
            # dumped almost the entire real map-generation cost into
            # "search" whenever Voronoi *did* do a full build -- confirmed
            # wrong by decision 6 in docs/decisions/0005-parallel-planner-
            # execution.md: swapping its search implementation changed
            # nothing. Fixed by giving Voronoi real recorder support instead
            # of working around the gap.) Show the honest total.
            map_ms = None
            search_ms = planning_ms
        run_metrics = RunMetrics()
        run_metrics.record_pipeline(0.0, map_ms or 0.0)
        run_metrics.record_planning((search_ms,), failures=self.runtime.last_plan_failures)
        run_metrics.finish(completed=False)

        self.previews = {label: PlannerPreview(paths, map_ms, search_ms, nodes, edges)}
        self.metrics = {label: run_metrics}
        self._planner_changed()
        self.status.setText(f"Planned {label}; no velocities sent{failure_note}")

    def _clear_preview(self) -> None:
        self.previews = {
            label: replace(preview, paths=(), nodes=(), edges=())
            for label, preview in self.previews.items()
        }
        self.canvas.clear_preview()
        self.layer_buttons["map"].setEnabled(False)
        self.status.setText("Planner geometry cleared; timing values retained")

    def _planner_changed(self) -> None:
        label = self.planner_selector.currentText()
        preview = self.previews.get(label, PlannerPreview((), 0.0, 0.0))
        self.canvas.planner_key = planner_key(self.planner_selector.currentData())
        self.canvas.set_preview(preview, label)
        if label in self.previews:
            if preview.map_time_ms is not None:
                self.map_time.setText(f"Map: {preview.map_time_ms:.3f} ms")
                self.plan_time.setText(f"Plan (search only): {preview.planning_time_ms:.3f} ms")
            else:
                # All three planners support a real map-vs-search split when
                # they actually build one (see StepRecorder usage in each
                # planners/*.py plan()) -- None here means *this call* took a
                # direct-line-of-sight/reused-previous-path shortcut and
                # never built anything, not that the planner lacks support.
                self.map_time.setText("Map: n/a (shortcut taken, no full build)")
                self.plan_time.setText(f"Plan (total): {preview.planning_time_ms:.3f} ms")
        else:
            self.map_time.setText("Map: -- ms")
            self.plan_time.setText("Plan: -- ms")
        self.layer_buttons["map"].setEnabled(bool(preview.nodes or preview.edges))
        self.canvas.update()

    def _scenario_edited(self, message: str) -> None:
        self.previews.clear()
        self.metrics.clear()
        self.map_time.setText("Map: -- ms")
        self.plan_time.setText("Plan: -- ms")
        self.status.setText(f"{message}; press Plan to replan")
        self._refresh_inspectors()
        self._refresh_controls()

    def _refresh_inspectors(self) -> None:
        self.obstacle_list.blockSignals(True)
        self.robot_list.blockSignals(True)
        self.obstacle_list.clear()
        self.robot_list.clear()
        if self.scenario is None:
            obstacles = ()
            robots = ()
        else:
            obstacles = self.scenario.obstacles
            robots = self.scenario.robots
        for obstacle in obstacles:
            team = "Y" if obstacle.is_yellow else "B"
            self.obstacle_list.addItem(
                f"{team}{obstacle.obstacle_id} | "
                f"({obstacle.position_mm[0]:.0f}, {obstacle.position_mm[1]:.0f}) mm | "
                f"v=({obstacle.velocity_mmps[0]:.0f}, {obstacle.velocity_mmps[1]:.0f})"
            )
        for robot in robots:
            team = "Y" if robot.is_yellow else "B"
            target = (
                "not set"
                if robot.target_mm is None
                else f"({robot.target_mm[0]:.0f}, {robot.target_mm[1]:.0f})"
            )
            self.robot_list.addItem(
                f"{team}{robot.robot_id} | start=({robot.start_mm[0]:.0f}, "
                f"{robot.start_mm[1]:.0f}) | target={target}"
            )
        self.obstacle_count.setText(f"n: {len(obstacles)}")
        self.robot_count.setText(f"n: {len(robots)}")
        if self.canvas.selected_obstacle is not None:
            self.obstacle_list.setCurrentRow(self.canvas.selected_obstacle)
        if self.canvas.selected_robot is not None:
            self.robot_list.setCurrentRow(self.canvas.selected_robot)
        self.obstacle_list.blockSignals(False)
        self.robot_list.blockSignals(False)

    def _refresh_controls(self) -> None:
        has_robot = (
            self.scenario is not None
            and self.canvas.selected_robot is not None
            and self.canvas.selected_robot < len(self.scenario.robots)
        )
        self.tool_buttons["target_pos"].setEnabled(has_robot)
        label = self.planner_selector.currentText()
        preview = self.previews.get(label)
        self.layer_buttons["map"].setEnabled(
            preview is not None and bool(preview.nodes or preview.edges)
        )
        if not has_robot and self.canvas.mode == "target_pos":
            self._select_tool("starting_pos")

    def _obstacle_row_selected(self, row: int) -> None:
        if row < 0:
            return
        self.canvas.selected_obstacle = row
        self.canvas.selected_robot = None
        self.canvas.update()

    def _robot_row_selected(self, row: int) -> None:
        if row < 0:
            return
        self.canvas.selected_robot = row
        self.canvas.selected_obstacle = None
        self._refresh_controls()
        self.canvas.update()

    def _toggle_layer(self, layer: str, checked: bool) -> None:
        if layer == "map":
            self.canvas.show_map_layer = checked
        elif layer == "robots":
            self.canvas.show_robot_layer = checked
        elif layer == "shape":
            self.canvas.show_shape_layer = checked
        elif layer == "path":
            self.canvas.show_path_layer = checked
        self.canvas.update()

    def _export_snapshot(self) -> None:
        if self.scenario is None:
            QMessageBox.warning(self, "No scenario", "Load or create a scenario first.")
            return
        destination, _ = QFileDialog.getSaveFileName(
            self,
            "Export field snapshot",
            "exports/scenario_snapshot.png",
            "PNG images (*.png)",
        )
        if not destination:
            return
        path = Path(destination)
        path.parent.mkdir(parents=True, exist_ok=True)
        cursor = self.canvas.cursor_mm
        self.canvas.cursor_mm = None
        self.canvas.update()
        self.canvas.grab().save(str(path), "PNG")
        self.canvas.cursor_mm = cursor
        self.status.setText(f"Exported field snapshot to {path}")

    def _export_results(self) -> None:
        if not self.metrics:
            QMessageBox.warning(self, "No results", "Plan the scenario before exporting.")
            return
        destination, _ = QFileDialog.getSaveFileName(
            self,
            "Export planner results",
            "exports/results_a.csv",
            "CSV files (*.csv)",
        )
        if not destination:
            return
        path = export_planner_results(destination, "a", self.metrics)
        self.status.setText(f"Exported planner results to {path}")


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
        self.planner_metrics: dict[str, RunMetrics] = {}
        self.active_planner_name: str | None = None
        self.active_plan_paths: tuple[PlannedRobotPath, ...] = ()
        self.control_timer = QTimer(self)
        self.control_timer.setInterval(50)
        self.control_timer.timeout.connect(self._control_tick)

        self.tabs = QTabWidget()
        self.experiment_page = QWidget()
        self.execution_page = ExecutionConsolePage(self.runtime, self.store)
        self.scenario_planner_page = ScenarioPlannerPage(self.runtime, self.store)
        self.config_page = ConfigurationsPage()
        self.tabs.addTab(self.execution_page, "Execution")
        self.tabs.addTab(self.scenario_planner_page, "Scenario Planner")
        self.tabs.addTab(self.config_page, "Configurations")
        self.setCentralWidget(self.tabs)
        self._build_experiment_page()
        self.execution_page.navigate_to_planner.connect(
            lambda: self.tabs.setCurrentWidget(self.scenario_planner_page)
        )
        self._refresh_scenarios()
        self._refresh_controls()
        self._start_live_grsim_display()

    def _start_live_grsim_display(self) -> None:
        self.display_vision_thread = VisionMonitor("grSim vision", self)
        self.display_vision_thread.packet_received.connect(self._live_grsim_packet_received)
        self.display_vision_thread.failed.connect(self._live_grsim_failed)
        self.display_vision_thread.start()

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
        self.fresh_snapshot_button = QPushButton("Fresh snapshot")
        self.apply_plan_button = QPushButton("Apply to grSim")
        test_grsim = QPushButton("Test grSim connection")
        form.addRow(self.fresh_snapshot_button)
        form.addRow(self.apply_plan_button)
        form.addRow(test_grsim)

        self.edit_mode = QComboBox()
        self.edit_mode.addItems(
            ("select_live_robot", "relocate_start", "set_target", "add_obstacle")
        )
        self.robot_id = QSpinBox(); self.robot_id.setRange(0, 15)
        self.team = QComboBox(); self.team.addItems(("Yellow", "Blue"))
        self.obstacle_radius = QSpinBox(); self.obstacle_radius.setRange(1, 2000); self.obstacle_radius.setValue(int(ROBOT_RADIUS_MM))
        form.addRow("Plan Course tool", self.edit_mode)

        self.planner_selector = QComboBox()
        planners = discover_planners()
        if planners:
            for label, planner_cls in planners.items():
                self.planner_selector.addItem(label, planner_cls)
        else:
            self.planner_selector.addItem("No planners discovered", None)
        form.addRow("Active planner", self.planner_selector)
        self.planner_selector.currentIndexChanged.connect(self._planner_changed)

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
        self._planner_changed()

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
            self.current_scenario.schema_version = 3
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
            self.current_scenario.schema_version = 3
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
        if robot.target_mm is None or robot.start_mm == robot.target_mm:
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

    def _planner_changed(self) -> None:
        self.canvas.planner_key = planner_key(self.planner_selector.currentData())
        # Changing the obstacle-layout preview must not erase or replace the
        # plan that is currently active for execution.
        self.canvas.update()

    def _scenario_edited(self) -> None:
        self.canvas.clear_paths()
        self.live_canvas.clear_paths()
        self.active_planner_name = None
        self.active_plan_paths = ()
        self.status.setText("Course edited · apply it to grSim when ready")

    def _run(self) -> None:
        if self.current_scenario is None:
            return
        selected_index = self.planner_selector.currentIndex()
        selected_name = self.planner_selector.currentText()
        selected_class = self.planner_selector.currentData()
        self.recorder = ExperimentRecorder(
            self.current_scenario.name,
            selected_name,
        )
        self.planner_metrics = {}
        planning_recorded = False
        try:
            self.session.run()
            paths = ()
            comparison = []
            selected_error: Exception | None = None
            for index in range(self.planner_selector.count()):
                planner_name = self.planner_selector.itemText(index)
                planner_class = self.planner_selector.itemData(index)
                metrics = (
                    self.recorder.metrics if index == selected_index else RunMetrics()
                )
                update = self.runtime.last_pipeline_update
                if update is not None:
                    metrics.record_pipeline(
                        update.processing_latency_ms,
                        update.mapping_time_ms,
                    )
                self.runtime.set_planner(planner_class)
                try:
                    candidate_paths = self.runtime.plan(self.current_scenario)
                except Exception as exc:
                    metrics.record_planning(
                        self.runtime.last_plan_durations_ms,
                        max(1, self.runtime.last_plan_failures),
                    )
                    comparison.append({"planner": planner_name, "error": str(exc)})
                    if index == selected_index:
                        selected_error = exc
                else:
                    metrics.record_planning(
                        self.runtime.last_plan_durations_ms,
                        self.runtime.last_plan_failures,
                    )
                    comparison.append(
                        {"planner": planner_name, "planning": metrics.row("a")}
                    )
                    if index == selected_index:
                        paths = candidate_paths
                if index != selected_index:
                    metrics.finish(completed=False)
                self.planner_metrics[planner_name] = metrics
            self.runtime.set_planner(selected_class)
            if selected_error is not None:
                raise selected_error
            planning_recorded = True
            # Only the selected planner becomes active. The other candidates
            # above are benchmarked, but their paths never reach the renderer
            # or velocity controller.
            self.active_planner_name = selected_name
            self.active_plan_paths = paths
            self._show_active_plan()
            self.runtime.start_execution(paths)
            self.recorder.metrics.start_execution()
            self.control_timer.start()
            self.recorder.record(
                "planner_comparison_generated",
                planners=comparison,
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
                f"Benchmarked {len(self.planner_metrics)} planners; executing {selected_name} · "
                f"recording events in {self.recorder.folder}"
            )
            self._refresh_controls()
        except Exception as exc:
            if not planning_recorded:
                if selected_name not in self.planner_metrics:
                    self.recorder.metrics.record_planning(
                        self.runtime.last_plan_durations_ms,
                        self.runtime.last_plan_failures,
                    )
            self.recorder.finish(completed=False)
            self.recorder.close()
            if self.session.state is SessionState.RUNNING:
                self.session.stop()
            QMessageBox.critical(self, "Cannot run scenario", str(exc))

    def _stop(self) -> None:
        try:
            self.control_timer.stop()
            self.runtime.stop_execution()
            self.session.stop()
            if self.recorder is not None:
                self.recorder.record("run_stopped")
                self.recorder.finish(completed=False)
                self.recorder.close()
            self.status.setText("Stopped · plan retained")
            self._refresh_controls()
        except Exception as exc:
            QMessageBox.warning(self, "Cannot stop", str(exc))

    def _show_active_plan(self) -> None:
        """Keep the one executable plan pinned on both field views."""
        self.canvas.set_paths(self.active_plan_paths)
        self.live_canvas.set_paths(self.active_plan_paths)

    def _erase(self) -> None:
        try:
            self.control_timer.stop()
            self.runtime.stop_execution()
            self.session.erase_plan()
            self.runtime.reset_planner()
            self.canvas.clear_paths()
            self.live_canvas.clear_paths()
            self.active_planner_name = None
            self.active_plan_paths = ()
            if self.recorder is not None:
                if not self.recorder.closed:
                    self.recorder.record("plan_erased")
                    self.recorder.finish(completed=False)
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
            if self.recorder is not None and not self.recorder.closed:
                self.recorder.record("course_reset")
                self.recorder.finish(completed=False)
                self.recorder.close()
            self.recorder = None
            self.planner_metrics = {}
            self.live_canvas.clear_paths()
            self.active_planner_name = None
            self.active_plan_paths = ()
            self.current_scenario = None
            self.current_scenario_path = None
            self.canvas.set_scenario(None)
            self.canvas.selected_robot = None
            self.scenario_selector.setCurrentIndex(-1)
            self.edit_mode.setCurrentText("select_live_robot")
            self.robot_id.setValue(0)
            self.team.setCurrentIndex(0)
            self.obstacle_radius.setValue(int(ROBOT_RADIUS_MM))
            if self.planner_selector.count():
                self.planner_selector.setCurrentIndex(0)
            self.field_tabs.setCurrentIndex(0)
            self.status.setText(
                "Course planning reset · capture a fresh snapshot to start again"
            )
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
            live_robots = self.runtime.live_robots
            if self.recorder is not None and not self.recorder.closed:
                self.recorder.metrics.observe_robots(live_robots)
            if not self.runtime.execute_tick(live_robots):
                return
            self.control_timer.stop()
            self.runtime.stop_execution()
            self.session.stop()
            if self.recorder is not None and not self.recorder.closed:
                self.recorder.record("robots_arrived")
                self.recorder.finish(completed=True)
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
            if self.recorder is not None and not self.recorder.closed:
                self.recorder.record("execution_failed", error=str(exc))
                self.recorder.finish(completed=False)
                self.recorder.close()
            self.status.setText(f"Execution stopped: {exc}")
            self._refresh_controls()

    def _vision_packet_received(self, packet, latency_ms: float) -> None:
        del packet
        self.runtime.last_receive_latency_ms = latency_ms
        self.receive_latency.set_latency(latency_ms)

    def _live_grsim_packet_received(self, packet, latency_ms: float) -> None:
        frame = self.runtime.ingest_vision_packet(packet)
        update = self.runtime.last_pipeline_update
        if update is not None and self.recorder is not None and not self.recorder.closed:
            self.recorder.metrics.record_pipeline(
                update.processing_latency_ms,
                update.mapping_time_ms,
            )
        if frame is not None:
            self.live_canvas.update_live_frame(frame)
            if (
                self.field_tabs.currentIndex() == 1
                and self.current_scenario is None
                and not self.canvas.live_robots
            ):
                self._capture_plan_snapshot()
        if update is not None:
            self.execution_page.process_snapshot(update.snapshot)
        self.runtime.last_receive_latency_ms = latency_ms
        self.receive_latency.set_latency(latency_ms)

    def _live_grsim_failed(self, message: str) -> None:
        self.status.setText(f"Live grSim display unavailable: {message}")
        if self.execution_page.controller.state in (
            ExecutionState.APPLYING,
            ExecutionState.RUNNING,
            ExecutionState.PAUSED,
            ExecutionState.RESETTING,
        ):
            self.execution_page.emergency_stop(
                error=f"Fatal grSim vision error: {message}"
            )

    def _export(self, format_name: str) -> None:
        destination, _ = QFileDialog.getSaveFileName(
            self,
            f"Export result format {format_name.upper()}",
            f"results_{format_name}.csv",
            "CSV files (*.csv)",
        )
        if destination:
            if self.recorder is None:
                QMessageBox.warning(self, "No results", "Run an experiment before exporting.")
                return
            metrics = self.planner_metrics or {
                self.planner_selector.currentText(): self.recorder.metrics
            }
            path = export_planner_results(destination, format_name, metrics)
            self.status.setText(
                f"Exported {len(metrics)} planner result rows to {path}"
            )

    def _refresh_controls(self) -> None:
        state = self.session.state
        self.run_button.setEnabled(state in (SessionState.SCENARIO_READY, SessionState.STOPPED) and self.session.has_scenario)
        self.stop_button.setEnabled(state is SessionState.RUNNING)
        self.erase_button.setEnabled(state is not SessionState.RUNNING and self.session.has_plan)
        self.reset_button.setEnabled(state is not SessionState.RUNNING)
        self.vision_button.setEnabled(self.session.can_change_vision)
        self.vision_source.setEnabled(self.session.can_change_vision_source)
        self.planner_selector.setEnabled(state is not SessionState.RUNNING)

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
        self.execution_page.shutdown()
        self.control_timer.stop()
        self.runtime.stop_execution()
        if self.display_vision_thread is not None:
            self.display_vision_thread.stop()
        if self.recorder is not None:
            if not self.recorder.closed:
                self.recorder.finish(completed=False)
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
