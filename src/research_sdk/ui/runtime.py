"""Runtime adapter connecting scenarios to grSim and the planner API."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

from research_sdk.config import ROBOT_RADIUS_MM
from research_sdk.network.grSimPacketFactory import grSimPacketFactory
from research_sdk.network.ssl_sockets import grSimSender
from research_sdk.planners import PlannerAPI, PlannerInput
from research_sdk.ui.scenarios import Scenario
from research_sdk.world.scene import PlanningObstacle, PlanningScene


@dataclass(frozen=True, slots=True)
class PlannedRobotPath:
    robot_id: int
    is_yellow: bool
    points_mm: tuple[tuple[float, float], ...]


class ResearchRuntime:
    def __init__(self) -> None:
        self._sender: grSimSender | None = None
        self._planner = PlannerAPI()
        self.last_send_latency_ms: float | None = None
        self.last_receive_latency_ms: float | None = None

    def apply_scenario(self, scenario: Scenario) -> None:
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
        replacements.extend(
            {
                "x": obstacle.position_mm[0] / 1000.0,
                "y": obstacle.position_mm[1] / 1000.0,
                "orientation": 0.0,
                "robot_id": obstacle.obstacle_id,
                "isYellow": obstacle.is_yellow,
            }
            for obstacle in scenario.obstacles
        )
        packet = grSimPacketFactory.scenario_replacement_command(replacements)
        started = perf_counter()
        self._get_sender().send_packet(packet)
        self.last_send_latency_ms = (perf_counter() - started) * 1000.0

    def plan(self, scenario: Scenario) -> tuple[PlannedRobotPath, ...]:
        paths = []
        for robot in scenario.robots:
            obstacles = tuple(
                PlanningObstacle(
                    robot_id=obstacle.obstacle_id,
                    isYellow=obstacle.is_yellow,
                    pos_mm=obstacle.position_mm,
                    radius_mm=obstacle.radius_mm,
                    vel_mmps=obstacle.velocity_mmps,
                )
                for obstacle in scenario.obstacles
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
            result = self._planner.plan(
                PlannerInput(
                    robot_id=robot.robot_id,
                    is_yellow=robot.is_yellow,
                    current_pose=(*robot.start_mm, robot.orientation_rad),
                    target_pose=(*robot.target_mm, robot.orientation_rad),
                    scene=scene,
                )
            )
            points = [robot.start_mm, *[(p[0], p[1]) for p in result.waypoints]]
            if points[-1] != robot.target_mm:
                points.append(robot.target_mm)
            paths.append(PlannedRobotPath(robot.robot_id, robot.is_yellow, tuple(points)))
        return tuple(paths)

    def reset_planner(self) -> None:
        self._planner.reset()

    def _get_sender(self) -> grSimSender:
        if self._sender is None:
            self._sender = grSimSender()
        return self._sender
