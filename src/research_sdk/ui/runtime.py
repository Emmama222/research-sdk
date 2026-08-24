"""Runtime adapter connecting scenarios to grSim and the planner API."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, sin
from pathlib import Path
from time import perf_counter

import yaml

from research_sdk.config import ROBOT_RADIUS_MM
from research_sdk.network.grSimPacketFactory import grSimPacketFactory
from research_sdk.network.robot_command import RobotCommand
from research_sdk.network.ssl_sockets import grSimSender
from research_sdk.planners import PlannerAPI, PlannerInput, VoronoiDijkstraPlanner
from research_sdk.ui.scenarios import Scenario
from research_sdk.world.pipeline import VisionWorldPipeline, WorldPipelineUpdate
from research_sdk.world.scene import PlanningObstacle, PlanningScene
from research_sdk.world.snapshot import WorldSnapshot


@dataclass(frozen=True, slots=True)
class PlannedRobotPath:
    robot_id: int
    is_yellow: bool
    points_mm: tuple[tuple[float, float], ...]


@dataclass(frozen=True, slots=True)
class ConnectionProbe:
    destination: tuple[str, int]
    local_address: tuple[str, int]


@dataclass(frozen=True, slots=True)
class LiveRobot:
    robot_id: int
    is_yellow: bool
    position_mm: tuple[float, float]
    orientation_rad: float


@dataclass(frozen=True, slots=True)
class LiveWorldFrame:
    robots: tuple[LiveRobot, ...]
    ball_mm: tuple[float, float] | None


def live_world_from_vision_packet(packet) -> LiveWorldFrame | None:
    """Convert one SSL-Vision wrapper detection into canvas-friendly values."""
    if packet is None or not packet.HasField("detection"):
        return None
    detection = packet.detection
    robots = tuple(
        LiveRobot(
            robot_id=int(robot.robot_id),
            is_yellow=is_yellow,
            position_mm=(float(robot.x), float(robot.y)),
            orientation_rad=float(robot.orientation),
        )
        for is_yellow, team in (
            (True, detection.robots_yellow),
            (False, detection.robots_blue),
        )
        for robot in team
    )
    ball = max(detection.balls, key=lambda item: item.confidence, default=None)
    ball_mm = None if ball is None else (float(ball.x), float(ball.y))
    return LiveWorldFrame(robots=robots, ball_mm=ball_mm)


def live_world_from_snapshot(snapshot: WorldSnapshot) -> LiveWorldFrame:
    robots = (*snapshot.yellow, *snapshot.blue)
    return LiveWorldFrame(
        robots=tuple(
            LiveRobot(robot.robot_id, robot.isYellow, robot.position, robot.theta)
            for robot in robots
            if robot is not None
        ),
        ball_mm=None if snapshot.ball is None else snapshot.ball.position,
    )


def waypoint_command(
    robot: LiveRobot,
    target_mm: tuple[float, float],
    *,
    gain_per_second: float = 2.0,
) -> RobotCommand:
    """Create a grSim robot-local velocity command toward a world-frame point."""
    error_x_m = (target_mm[0] - robot.position_mm[0]) / 1000.0
    error_y_m = (target_mm[1] - robot.position_mm[1]) / 1000.0
    world_vx = gain_per_second * error_x_m
    world_vy = gain_per_second * error_y_m
    heading = robot.orientation_rad
    local_vx = cos(heading) * world_vx + sin(heading) * world_vy
    local_vy = -sin(heading) * world_vx + cos(heading) * world_vy
    return RobotCommand(
        robot_id=robot.robot_id,
        isYellow=robot.is_yellow,
        vx=local_vx,
        vy=local_vy,
    )


def planner_key(planner_cls: type | None) -> str | None:
    """Return the persistent identifier used by scenario obstacle layouts."""
    if planner_cls is None:
        return None
    return f"{planner_cls.__module__}.{planner_cls.__qualname__}"


class ResearchRuntime:
    def __init__(self) -> None:
        self._sender: grSimSender | None = None
        self._planner = PlannerAPI()
        self._planner_key: str | None = None
        self.last_send_latency_ms: float | None = None
        self.last_receive_latency_ms: float | None = None
        self.last_pipeline_update: WorldPipelineUpdate | None = None
        self.last_plan_durations_ms: tuple[float, ...] = ()
        self.last_plan_failures = 0
        self.world_pipeline = VisionWorldPipeline(cameras=4)
        self.active_paths: tuple[PlannedRobotPath, ...] = ()
        self._active_paths: dict[tuple[bool, int], PlannedRobotPath] = {}
        self._waypoint_indices: dict[tuple[bool, int], int] = {}

    def ingest_vision_packet(self, packet) -> LiveWorldFrame | None:
        """Update the runtime's world-state boundary from one vision packet."""
        update = self.world_pipeline.ingest(packet)
        self.last_pipeline_update = update
        return None if update is None else live_world_from_snapshot(update.snapshot)

    @property
    def world_snapshot(self) -> WorldSnapshot | None:
        return self.world_pipeline.store.current

    @property
    def live_robots(self) -> dict[tuple[bool, int], LiveRobot]:
        snapshot = self.world_snapshot
        if snapshot is None:
            return {}
        return {
            (robot.isYellow, robot.robot_id): LiveRobot(
                robot.robot_id,
                robot.isYellow,
                robot.position,
                robot.theta,
            )
            for robot in (*snapshot.yellow, *snapshot.blue)
            if robot is not None
        }

    def apply_scenario(self, scenario: Scenario, *, include_obstacles: bool = True) -> None:
        replacements = [
            {
                "x": robot.start_mm[0] / 1000.0,
                "y": robot.start_mm[1] / 1000.0,
                "orientation": robot.orientation_rad,
                "robot_id": robot.robot_id,
                "isYellow": robot.is_yellow,
            }
            for robot in scenario.robots
        ]
        if include_obstacles:
            replacements.extend(
                {
                    "x": obstacle.position_mm[0] / 1000.0,
                    "y": obstacle.position_mm[1] / 1000.0,
                    "orientation": 0.0,
                    "robot_id": obstacle.obstacle_id,
                    "isYellow": obstacle.is_yellow,
                }
                for obstacle in scenario.obstacles_for(self._planner_key)
            )
        packet = grSimPacketFactory.scenario_replacement_command(replacements)
        started = perf_counter()
        self._get_sender().send_packet(packet)
        self.last_send_latency_ms = (perf_counter() - started) * 1000.0

    def plan(self, scenario: Scenario) -> tuple[PlannedRobotPath, ...]:
        scenario.require_complete()
        paths = []
        durations_ms: list[float] = []
        self.last_plan_durations_ms = ()
        self.last_plan_failures = 0
        for robot in scenario.robots:
            assert robot.target_mm is not None
            obstacles = tuple(
                PlanningObstacle(
                    robot_id=obstacle.obstacle_id,
                    isYellow=obstacle.is_yellow,
                    pos_mm=obstacle.position_mm,
                    radius_mm=obstacle.radius_mm,
                    vel_mmps=obstacle.velocity_mmps,
                )
                for obstacle in scenario.obstacles_for(self._planner_key)
            ) + tuple(
                PlanningObstacle(
                    robot_id=other.robot_id,
                    isYellow=other.is_yellow,
                    pos_mm=other.start_mm,
                    radius_mm=ROBOT_RADIUS_MM,
                )
                for other in scenario.robots
                if other != robot
            )
            scene = PlanningScene(timestamp=perf_counter(), obstacles=obstacles)
            started = perf_counter()
            try:
                result = self._planner.plan(
                    PlannerInput(
                        robot_id=robot.robot_id,
                        is_yellow=robot.is_yellow,
                        current_pose=(*robot.start_mm, robot.orientation_rad),
                        target_pose=(*robot.target_mm, robot.orientation_rad),
                        scene=scene,
                    )
                )
            except Exception:
                self.last_plan_failures += 1
                raise
            finally:
                durations_ms.append((perf_counter() - started) * 1000.0)
                self.last_plan_durations_ms = tuple(durations_ms)
            points = [robot.start_mm, *[(p[0], p[1]) for p in result.waypoints]]
            if points[-1] != robot.target_mm:
                points.append(robot.target_mm)
            paths.append(PlannedRobotPath(robot.robot_id, robot.is_yellow, tuple(points)))
        self.last_plan_durations_ms = tuple(durations_ms)
        return tuple(paths)

    def set_planner(self, planner_cls: type | None, *, record=None) -> None:
        """Swap the active planner, e.g. from the UI's "Active planner" dropdown.

        ``VoronoiDijkstraPlanner`` is discoverable but implements a different,
        lower-level ``.plan(scene, start, target, ...)`` signature (it is
        VoronoiWaypointManager's internal building block, not itself a
        PlannerAPI-shaped planner) -- selecting it maps back onto the default
        ``PlannerAPI()`` rather than instantiating it directly.
        """
        self._planner_key = planner_key(planner_cls)
        if planner_cls is None or planner_cls is VoronoiDijkstraPlanner:
            self._planner = PlannerAPI()
        else:
            self._planner = planner_cls(**({"record": record} if record is not None else {}))

    def reset_planner(self) -> None:
        self._planner.reset()

    def start_execution(self, paths: tuple[PlannedRobotPath, ...]) -> None:
        self.active_paths = paths
        self._active_paths = {(path.is_yellow, path.robot_id): path for path in paths}
        self._waypoint_indices = {
            key: min(1, len(path.points_mm)) for key, path in self._active_paths.items()
        }

    def execute_tick(self, live_robots: dict[tuple[bool, int], LiveRobot]) -> bool:
        """Send one control tick and return True when every path has arrived."""
        if not self._active_paths:
            return True
        all_arrived = True
        for key, path in self._active_paths.items():
            robot = live_robots.get(key)
            index = self._waypoint_indices[key]
            if robot is None:
                self._send_stop(key)
                all_arrived = False
                continue
            while index < len(path.points_mm):
                target = path.points_mm[index]
                threshold_mm = 60.0 if index == len(path.points_mm) - 1 else 120.0
                if (
                    hypot(
                        target[0] - robot.position_mm[0],
                        target[1] - robot.position_mm[1],
                    )
                    > threshold_mm
                ):
                    break
                index += 1
            self._waypoint_indices[key] = index
            if index >= len(path.points_mm):
                self._send_stop(key)
                continue
            all_arrived = False
            self._get_sender().send_robot_command(waypoint_command(robot, path.points_mm[index]))
        return all_arrived

    def stop_execution(self) -> None:
        for key in self._active_paths:
            self._send_stop(key)
        self._active_paths.clear()
        self._waypoint_indices.clear()
        self.active_paths = ()

    def _send_stop(self, key: tuple[bool, int]) -> None:
        is_yellow, robot_id = key
        self._get_sender().send_robot_command(RobotCommand(robot_id=robot_id, isYellow=is_yellow))

    def test_grsim_connection(self) -> ConnectionProbe:
        """Verify that the configured UDP destination is routable locally.

        UDP has no handshake, so receipt must be confirmed separately through
        grSim vision.
        """
        sender = self._get_sender()
        sender.sock.connect(sender.destination)
        local_address = sender.sock.getsockname()
        return ConnectionProbe(sender.destination, (str(local_address[0]), int(local_address[1])))

    def _get_sender(self) -> grSimSender:
        config_path = Path(__file__).resolve().parents[1] / "config" / "network_input.yaml"
        config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        destination = (str(config["grsim_command_ip"]), int(config["grsim_command_port"]))
        if self._sender is None or self._sender.destination != destination:
            if self._sender is not None:
                self._sender.close()
            self._sender = grSimSender(ip=destination[0], port=destination[1])
        return self._sender
