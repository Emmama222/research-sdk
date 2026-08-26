"""Runtime adapter connecting scenarios to grSim and the planner API."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
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
    failed: bool = False


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
    def __init__(self, *, parallel_planning: bool = True, predict_motion: bool = False) -> None:
        self._sender: grSimSender | None = None
        self._planner = PlannerAPI()
        self._planner_key: str | None = None
        self.last_send_latency_ms: float | None = None
        self.last_receive_latency_ms: float | None = None
        self.last_pipeline_update: WorldPipelineUpdate | None = None
        self.last_plan_durations_ms: tuple[float, ...] = ()
        self.last_plan_failures = 0
        # Policy switch, not a perf-critical default: each robot's plan() call
        # is independent (obstacles are a snapshot of the scene, not built from
        # other robots' *new* paths), so planning them on a thread pool instead
        # of one at a time is safe. See docs/decisions/0005-parallel-planner-
        # execution.md for the benchmark data behind this and why a thread pool
        # (not a process pool) is the default -- flip to False to restore the
        # old strictly-sequential behaviour for debugging/comparison.
        self.parallel_planning = parallel_planning
        # Policy switch, off by default (changes *what* gets planned against,
        # not just how fast): when True, a robot's teammates/opponents are
        # sourced from self.world_pipeline.latest_scene (real tracked
        # position + velocity-projected position and motion-inflated radius,
        # built by WorldMap.planning_scene() -- see
        # world/map/world_map.py:370) instead of their static scenario
        # start_mm. This is planner-agnostic: it swaps the obstacle *source*
        # before any planner sees it, so VisibilityGraph/PRM/Voronoi all get
        # the same upgrade with no per-planner code. Falls back to the static
        # behaviour automatically whenever no live scene exists yet (e.g.
        # vision hasn't produced a frame since reset), so turning it on is
        # never destructive. See docs/decisions/0005-parallel-planner-
        # execution.md, "Future work".
        self.predict_motion = predict_motion
        self.world_pipeline = VisionWorldPipeline(cameras=4)
        self.active_paths: tuple[PlannedRobotPath, ...] = ()
        self._active_paths: dict[tuple[bool, int], PlannedRobotPath] = {}
        self._waypoint_indices: dict[tuple[bool, int], int] = {}
        self._paused = False
        self._step_mode = False
        self._step_boundary_indices: dict[tuple[bool, int], int] = {}
        self.step_completed = False
        self.last_waypoint_transitions: tuple[
            tuple[tuple[bool, int], int], ...
        ] = ()

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
        if scenario.ball is not None:
            ball_packet = grSimPacketFactory.ball_replacement_command(
                x=scenario.ball.position_mm[0] / 1000.0,
                y=scenario.ball.position_mm[1] / 1000.0,
                vx=scenario.ball.velocity_mmps[0] / 1000.0,
                vy=scenario.ball.velocity_mmps[1] / 1000.0,
            )
            self._get_sender().send_packet(ball_packet)
        self.last_send_latency_ms = (perf_counter() - started) * 1000.0

    def _obstacles_for_robot(
        self, scenario: Scenario, robot
    ) -> tuple[PlanningObstacle, ...]:
        scenario_obstacles = tuple(
            PlanningObstacle(
                robot_id=obstacle.obstacle_id,
                isYellow=obstacle.is_yellow,
                pos_mm=obstacle.position_mm,
                radius_mm=obstacle.radius_mm,
                vel_mmps=obstacle.velocity_mmps,
            )
            for obstacle in scenario.obstacles_for(self._planner_key)
        )
        return scenario_obstacles + self._other_robot_obstacles(scenario, robot)

    def _other_robot_obstacles(
        self, scenario: Scenario, robot
    ) -> tuple[PlanningObstacle, ...]:
        live_scene = self.world_pipeline.latest_scene if self.predict_motion else None
        if live_scene is not None:
            robot_key = (robot.is_yellow, robot.robot_id)
            return tuple(
                obstacle for obstacle in live_scene.obstacles if obstacle.key != robot_key
            )
        # No live tracked scene (predict_motion is off, or vision hasn't
        # produced a frame yet) -- fall back to each teammate/opponent's
        # static scenario position, exactly as before this toggle existed.
        return tuple(
            PlanningObstacle(
                robot_id=other.robot_id,
                isYellow=other.is_yellow,
                pos_mm=other.start_mm,
                radius_mm=ROBOT_RADIUS_MM,
            )
            for other in scenario.robots
            if other != robot
        )

    def _plan_one_robot(self, scenario: Scenario, robot) -> tuple[PlannedRobotPath, float]:
        """Plan one robot's path and time it.

        On failure, re-raises whatever ``self._planner.plan()`` raised, with
        the elapsed time up to the failure attached as ``.duration_ms`` --
        callers use that to keep timing full even for a failed attempt,
        matching the original serial implementation's ``finally``-based
        timing.

        Only touches ``self._planner`` (read) and locals -- safe to call from
        multiple threads at once, one call per robot, since ``scenario`` is a
        snapshot and each robot's obstacle set is independent of every other
        robot's *planned* path (only their current ``start_mm``, already
        fixed before planning starts).
        """
        assert robot.target_mm is not None
        obstacles = self._obstacles_for_robot(scenario, robot)
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
        except Exception as exc:
            exc.duration_ms = (perf_counter() - started) * 1000.0
            raise
        duration_ms = (perf_counter() - started) * 1000.0
        points = [robot.start_mm, *[(p[0], p[1]) for p in result.waypoints]]
        if points[-1] != robot.target_mm:
            points.append(robot.target_mm)
        path = PlannedRobotPath(robot.robot_id, robot.is_yellow, tuple(points))
        return path, duration_ms

    def _run_and_track(self, robot, call, durations_ms: list[float]) -> PlannedRobotPath:
        """Run one already-scheduled plan attempt for ``robot``, track its
        duration and failure count -- shared by both the serial and
        thread-pool branches of ``plan()`` so they stay behaviourally
        identical.

        On failure, this does *not* raise or abort the rest of the batch:
        it returns a stationary (``points_mm`` is just the robot's current
        position), ``failed=True`` path instead. One robot's planner
        failure (no route found, etc.) shouldn't stop every other robot
        from getting a path -- the UI flags a ``failed`` robot red and
        leaves it in place (see ``app.py``'s ``_draw_robot``/
        ``_draw_planned_robot``) rather than the whole plan-all-robots call
        raising.
        """
        try:
            path, duration_ms = call()
        except Exception as exc:
            self.last_plan_failures += 1
            durations_ms.append(getattr(exc, "duration_ms", 0.0))
            self.last_plan_durations_ms = tuple(durations_ms)
            return PlannedRobotPath(robot.robot_id, robot.is_yellow, (robot.start_mm,), failed=True)
        durations_ms.append(duration_ms)
        return path

    def plan(self, scenario: Scenario) -> tuple[PlannedRobotPath, ...]:
        scenario.require_complete()
        self.last_plan_durations_ms = ()
        self.last_plan_failures = 0
        robots = list(scenario.robots)
        durations_ms: list[float] = []
        paths: list[PlannedRobotPath] = []

        if self.parallel_planning and len(robots) > 1:
            with ThreadPoolExecutor(max_workers=len(robots)) as executor:
                futures = [executor.submit(self._plan_one_robot, scenario, robot) for robot in robots]
                for robot, future in zip(robots, futures):
                    paths.append(self._run_and_track(robot, future.result, durations_ms))
        else:
            for robot in robots:
                call = lambda robot=robot: self._plan_one_robot(scenario, robot)
                paths.append(self._run_and_track(robot, call, durations_ms))

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

    def start_execution(
        self,
        paths: tuple[PlannedRobotPath, ...],
        *,
        waypoint_indices: dict[tuple[bool, int], int] | None = None,
        paused: bool = False,
    ) -> None:
        self.active_paths = paths
        self._active_paths = {(path.is_yellow, path.robot_id): path for path in paths}
        defaults = {
            key: min(1, len(path.points_mm)) for key, path in self._active_paths.items()
        }
        self._waypoint_indices = defaults
        if waypoint_indices is not None:
            for key, index in waypoint_indices.items():
                if key in self._active_paths:
                    self._waypoint_indices[key] = max(
                        0, min(int(index), len(self._active_paths[key].points_mm))
                    )
        self._paused = bool(paused)
        self._step_mode = False
        self._step_boundary_indices.clear()
        self.step_completed = False
        self.last_waypoint_transitions = ()
        if paused:
            self._send_zero_to_active()

    @property
    def waypoint_indices(self) -> dict[tuple[bool, int], int]:
        return dict(self._waypoint_indices)

    @property
    def paused(self) -> bool:
        return self._paused

    def pause_execution(self) -> None:
        if not self._active_paths:
            raise RuntimeError("No active execution to pause")
        self._paused = True
        self._step_mode = False
        self._step_boundary_indices.clear()
        self._send_zero_to_active()

    def continue_execution(self) -> None:
        if not self._active_paths:
            raise RuntimeError("No active execution to continue")
        self._paused = False
        self._step_mode = False
        self.step_completed = False

    def step_execution(self) -> None:
        if not self._active_paths or not self._paused:
            raise RuntimeError("Step requires a paused execution")
        self._step_boundary_indices = dict(self._waypoint_indices)
        self._step_mode = True
        self._paused = False
        self.step_completed = False

    def restore_waypoint_indices(
        self, waypoint_indices: dict[tuple[bool, int], int]
    ) -> None:
        if not self._active_paths:
            raise RuntimeError("No active paths are available")
        for key, index in waypoint_indices.items():
            if key not in self._active_paths:
                raise ValueError(f"Checkpoint contains unknown active robot {key}")
            self._waypoint_indices[key] = max(
                0, min(int(index), len(self._active_paths[key].points_mm))
            )

    def execute_tick(self, live_robots: dict[tuple[bool, int], LiveRobot]) -> bool:
        """Send one control tick and return True when every path has arrived."""
        if not self._active_paths:
            return True
        self.last_waypoint_transitions = ()
        if self._paused and not self._step_mode:
            return False
        all_arrived = True
        step_finished = self._step_mode
        transitions = []
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
                reached_index = index
                index += 1
                transitions.append((key, reached_index))
                if self._step_mode:
                    break
            self._waypoint_indices[key] = index
            if index >= len(path.points_mm):
                self._send_stop(key)
                continue
            if self._step_mode and index > self._step_boundary_indices.get(key, index):
                self._send_stop(key)
                continue
            if self._step_mode:
                step_finished = False
            all_arrived = False
            self._get_sender().send_robot_command(waypoint_command(robot, path.points_mm[index]))
        self.last_waypoint_transitions = tuple(transitions)
        if self._step_mode and step_finished:
            self._send_zero_to_active()
            self._step_mode = False
            self._paused = True
            self.step_completed = True
        return all_arrived

    def stop_execution(self) -> None:
        self.emergency_stop()

    def emergency_stop(
        self, extra_robot_keys: tuple[tuple[bool, int], ...] = ()
    ) -> tuple[str, ...]:
        """Best-effort zero commands; one failure never skips another robot."""
        errors = []
        keys = tuple(dict.fromkeys((*self._active_paths, *extra_robot_keys)))
        for key in keys:
            try:
                self._send_stop(key)
            except Exception as exc:  # noqa: BLE001 - continue stopping other robots
                errors.append(f"{key}: {exc}")
        self._active_paths.clear()
        self._waypoint_indices.clear()
        self.active_paths = ()
        self._paused = False
        self._step_mode = False
        self._step_boundary_indices.clear()
        self.step_completed = False
        self.last_waypoint_transitions = ()
        return tuple(errors)

    def send_robot_command(self, command: RobotCommand) -> None:
        self._get_sender().send_robot_command(command)

    def stop_robot(self, is_yellow: bool, robot_id: int) -> None:
        self._send_stop((bool(is_yellow), int(robot_id)))

    def _send_zero_to_active(self) -> None:
        for key in self._active_paths:
            self._send_stop(key)

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
