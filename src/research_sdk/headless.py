"""Fast, deterministic, grSim-free scenario execution for research batches.

The headless simulator models the part of grSim used by the SDK execution
loop: robots follow planner waypoints with bounded velocity while collisions
and arrival are measured. Virtual time never sleeps, so experiments can run
faster than real time without Qt, UDP, or a grSim process. It is not a
rigid-body replacement for grSim.

Replanning policies (``SimulationConfig.replan_policy``):

* ``once``  -- plan every robot at t=0 and execute open-loop.
* ``cycle`` -- rebuild every robot's plan from scratch on every control cycle
  (the naive per-tick baseline; planner caches are reset before each call).
* ``event`` -- consult the planner every control cycle, but only rebuild when
  the shared reroute gate (``planners/reroute.py``) reports an event: direct
  line blocked *and* the target moved, the route finished, or the active
  segment became blocked.

In ``cycle`` and ``event`` modes the scene given to the planner is rebuilt
every control cycle from the robots' current positions and velocities and the
moving obstacles' current positions. Planning latency is measured on the wall
clock but does not delay execution on the virtual clock.
"""

from __future__ import annotations

import argparse
import json
import os
from collections.abc import Sequence
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass, replace
from datetime import UTC, datetime
from itertools import pairwise
from math import hypot, isfinite
from pathlib import Path
from statistics import fmean, median
from time import perf_counter

from research_sdk.config import ROBOT_MAX_LINEAR_SPEED_MPS, ROBOT_RADIUS_MM
from research_sdk.exporters import CSVExporter, JSONExporter
from research_sdk.planners import (
    PlannerAPI,
    PlannerInput,
    PRMPlanner,
    VisibilityGraphPlanner,
    VoronoiDijkstraPlanner,
)
from research_sdk.scenario_generator import GeneratorConfig, perturb_scenario, random_scenario
from research_sdk.ui.scenarios import (
    DEFAULT_PATROL_SPEED_MMPS,
    Scenario,
    ScenarioObstacle,
    ScenarioRobot,
)
from research_sdk.world.scene import FieldDimensions, PlanningObstacle, PlanningScene

Point = tuple[float, float]
RobotKey = tuple[bool, int]
PLANNER_NAMES = ("voronoi", "prm", "visibility")
REPLAN_POLICIES = ("once", "cycle", "event")
NEAR_MISS_MM = 50.0


@dataclass(frozen=True, slots=True)
class SimulationConfig:
    """Virtual-time and controller settings for one experiment batch."""

    dt_s: float = 0.02
    max_simulation_s: float = 30.0
    max_speed_mmps: float = ROBOT_MAX_LINEAR_SPEED_MPS * 1000.0
    gain_per_second: float = 2.0
    waypoint_tolerance_mm: float = 120.0
    final_tolerance_mm: float = 60.0
    replan_policy: str = "once"
    control_period_s: float = 1.0 / 60.0
    periodic_reroute_frames: int | None = None
    planning_clearance_mm: float = 0.0
    prediction_horizon_ms: float = 0.0

    def __post_init__(self) -> None:
        for name in (
            "dt_s",
            "max_simulation_s",
            "max_speed_mmps",
            "gain_per_second",
            "waypoint_tolerance_mm",
            "final_tolerance_mm",
            "control_period_s",
        ):
            if not isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.replan_policy not in REPLAN_POLICIES:
            raise ValueError(
                f"Unknown replan policy {self.replan_policy!r}; choose from {REPLAN_POLICIES}"
            )
        if not isfinite(self.prediction_horizon_ms) or self.prediction_horizon_ms < 0:
            raise ValueError("prediction_horizon_ms must be finite and non-negative")
        if not isfinite(self.planning_clearance_mm) or self.planning_clearance_mm < 0:
            raise ValueError("planning_clearance_mm must be finite and non-negative")
        if self.periodic_reroute_frames is not None and self.periodic_reroute_frames < 1:
            raise ValueError("periodic_reroute_frames must be at least 1 or None")


@dataclass(frozen=True, slots=True)
class HeadlessRunResult:
    """Flat, exporter-friendly result for one scenario/planner/trial."""

    scenario: str
    planner: str
    trial: int
    seed: int
    status: str
    completed: bool
    timed_out: bool
    robot_count: int
    planner_calls: int
    failed_plans: int
    failed_robots: str
    planning_time_ms_total: float
    planning_time_ms_mean: float
    simulated_duration_ms: float
    wall_time_ms: float
    realtime_factor: float
    ticks: int
    robot_collision_episodes: int
    obstacle_collision_episodes: int
    collision_episodes: int
    planned_path_length_mm: float
    travelled_distance_mm: float
    mean_final_error_mm: float
    max_final_error_mm: float
    minimum_clearance_mm: float | None
    dt_ms: float
    backend: str = "kinematic"
    physics_validation_passed: bool = False
    missed_frames: int = 0
    max_frame_gap_ms: float = 0.0
    max_final_speed_mmps: float | None = None
    evidence_directory: str = ""
    error: str = ""
    scenario_set: str = ""
    replan_policy: str = "once"
    replan_count: int = 0
    replan_failures: int = 0
    planning_time_ms_initial: float = 0.0
    planning_time_ms_max_call: float = 0.0
    time_to_goal_ms: float | None = None
    prediction_horizon_ms: float = 0.0
    planning_time_ms_p95_call: float = 0.0
    straight_line_mm: float = 0.0
    replan_period_ms: float = 0.0

    def to_record(self) -> dict[str, str | bool | int | float | None]:
        return asdict(self)


@dataclass(slots=True)
class _RobotState:
    robot: ScenarioRobot
    path: tuple[Point, ...]
    failed: bool
    position: Point
    waypoint_index: int = 1
    travelled_mm: float = 0.0
    velocity_mmps: Point = (0.0, 0.0)
    reached_since_call: int = 0
    initial_path: tuple[Point, ...] = ()

    @property
    def key(self) -> RobotKey:
        return (self.robot.is_yellow, self.robot.robot_id)


def _planner_class(name: str) -> type:
    return {
        "voronoi": VoronoiDijkstraPlanner,
        "prm": PRMPlanner,
        "visibility": VisibilityGraphPlanner,
    }[name]


def _new_planner(
    name: str,
    seed: int,
    policy: str = "once",
    periodic_reroute_frames: int | None = None,
):
    # ``cycle`` disables the gate so every call is a full rebuild; ``once``
    # keeps the production default because only the first call is made.
    gate = {
        "use_reroute_gate": policy != "cycle",
        "periodic_reroute_frames": periodic_reroute_frames,
    }
    if name == "voronoi":
        return PlannerAPI(**gate)
    if name == "prm":
        return PRMPlanner(seed=seed, **gate)
    if name == "visibility":
        return VisibilityGraphPlanner(**gate)
    raise ValueError(f"Unknown planner {name!r}; choose from {PLANNER_NAMES}")


def _planner_key(name: str) -> str:
    planner_class = _planner_class(name)
    return f"{planner_class.__module__}.{planner_class.__qualname__}"


def _planning_obstacles(
    scenario: Scenario,
    robot: ScenarioRobot,
    planner_name: str,
) -> tuple[PlanningObstacle, ...]:
    configured = tuple(
        PlanningObstacle(
            robot_id=obstacle.obstacle_id,
            isYellow=obstacle.is_yellow,
            pos_mm=obstacle.position_mm,
            radius_mm=obstacle.radius_mm,
            vel_mmps=obstacle.velocity_mmps,
        )
        for obstacle in scenario.obstacles_for(_planner_key(planner_name))
    )
    other_robots = tuple(
        PlanningObstacle(
            robot_id=other.robot_id,
            isYellow=other.is_yellow,
            pos_mm=other.start_mm,
            radius_mm=ROBOT_RADIUS_MM,
        )
        for other in scenario.robots
        if other != robot
    )
    return configured + other_robots


def _distance(first: Point, second: Point) -> float:
    return hypot(first[0] - second[0], first[1] - second[1])


def _path_length(points: tuple[Point, ...]) -> float:
    return sum(_distance(first, second) for first, second in pairwise(points))


def _dedupe_path(points: Sequence[Point]) -> tuple[Point, ...]:
    result: list[Point] = []
    for point in points:
        normalized = (float(point[0]), float(point[1]))
        if not result or normalized != result[-1]:
            result.append(normalized)
    return tuple(result)


def _scene_at(
    scenario: Scenario,
    state: _RobotState,
    states: Sequence[_RobotState],
    planner_name: str,
    simulation_s: float,
    horizon_ms: float = 0.0,
) -> PlanningScene:
    """Freeze the world as this robot's planner sees it at ``simulation_s``.

    At t=0 this is identical to ``_planning_obstacles``: configured obstacles
    at their scenario positions and the other robots at their starts.
    """
    configured = tuple(
        _predicted_obstacle(
            obstacle.obstacle_id,
            obstacle.is_yellow,
            _obstacle_position(obstacle, simulation_s),
            obstacle.radius_mm,
            _obstacle_velocity(obstacle, simulation_s),
            horizon_ms,
        )
        for obstacle in scenario.obstacles_for(_planner_key(planner_name))
    )
    other_robots = tuple(
        _predicted_obstacle(
            other.robot.robot_id,
            other.robot.is_yellow,
            other.position,
            ROBOT_RADIUS_MM,
            other.velocity_mmps,
            horizon_ms,
        )
        for other in states
        if other is not state
    )
    return PlanningScene(
        timestamp=simulation_s,
        obstacles=configured + other_robots,
        prediction_horizon_ms=float(horizon_ms),
    )


def _predicted_obstacle(
    robot_id: int,
    is_yellow: bool,
    position: Point,
    radius_mm: float,
    velocity: Point,
    horizon_ms: float,
) -> PlanningObstacle:
    """Constant-velocity prediction, same model as ``WorldMap`` with Predict motion on.

    The circle moves to the predicted position and grows by the predicted travel
    (``Obstacle.dynamic_radius_0``), so it still covers the current position.
    Applied identically to every planner and to the event trigger.
    """
    horizon_s = horizon_ms / 1000.0
    return PlanningObstacle(
        robot_id=robot_id,
        isYellow=is_yellow,
        pos_mm=(position[0] + velocity[0] * horizon_s, position[1] + velocity[1] * horizon_s),
        radius_mm=radius_mm + hypot(velocity[0], velocity[1]) * horizon_s,
        vel_mmps=velocity,
        prediction_horizon_ms=float(horizon_ms),
    )


def _plan_call(
    planner,
    state: _RobotState,
    scene: PlanningScene,
    simulation_s: float,
    config: SimulationConfig,
    first: bool,
) -> tuple[float, int, int]:
    """Consult the planner once; return (elapsed ms, replanned, failed)."""
    robot = state.robot
    assert robot.target_mm is not None
    target = (float(robot.target_mm[0]), float(robot.target_mm[1]))
    if config.replan_policy == "cycle" and not first:
        # Per-tick baseline: discard cached routes so every call is a rebuild.
        planner.reset(robot_id=robot.robot_id, is_yellow=robot.is_yellow)
    reached = state.reached_since_call > 0
    state.reached_since_call = 0
    started = perf_counter()
    try:
        output = planner.plan(
            PlannerInput(
                robot_id=robot.robot_id,
                is_yellow=robot.is_yellow,
                current_pose=(*state.position, robot.orientation_rad),
                target_pose=(*target, robot.orientation_rad),
                scene=scene,
                now_s=simulation_s,
                clearance_mm=config.planning_clearance_mm,
                robot_reached_current_waypoint=reached and not first,
            )
        )
    except Exception:  # noqa: BLE001 - planner failures are experiment result data
        output = None
    elapsed_ms = (perf_counter() - started) * 1000.0
    waypoints = (
        ()
        if output is None
        else tuple((float(point[0]), float(point[1])) for point in output.waypoints)
    )

    if first:
        success = output is not None and (
            output.is_path_free
            or bool(waypoints)
            or _distance(robot.start_mm, target) <= 1e-9
        )
        state.path = (
            _dedupe_path((robot.start_mm, *waypoints, target))
            if success
            else (robot.start_mm,)
        )
        state.failed = not success
        state.initial_path = state.path
        state.waypoint_index = 1
        return elapsed_ms, 0, 0

    if output is None:
        return elapsed_ms, 0, 1
    if output.is_path_free:
        if state.path[state.waypoint_index :] != (target,):
            state.path = _dedupe_path((state.position, target))
            state.waypoint_index = 1
        return elapsed_ms, 0, 0
    if not output.did_reroute:
        return elapsed_ms, 0, 0
    if not waypoints:
        # Rebuild attempted but produced nothing: keep executing the old route.
        return elapsed_ms, 0, 1
    new_path = _dedupe_path((state.position, *waypoints, target))
    if len(new_path) <= 2:
        # PRM/visibility report a clear straight line as a two-point "route";
        # treat it like Voronoi's is_path_free so replan counts are comparable.
        if state.path[state.waypoint_index :] != (target,):
            state.path = new_path
            state.waypoint_index = 1
        return elapsed_ms, 0, 0
    state.path = new_path
    state.waypoint_index = 1
    return elapsed_ms, 1, 0


def _plan_robot(
    scenario: Scenario,
    robot: ScenarioRobot,
    planner_name: str,
    planner,
) -> tuple[_RobotState, float]:
    """Plan one robot once from its start (used by the grSim physics backend)."""
    assert robot.target_mm is not None
    state = _RobotState(robot=robot, path=(robot.start_mm,), failed=False, position=robot.start_mm)
    scene = PlanningScene(
        timestamp=0.0,
        obstacles=_planning_obstacles(scenario, robot, planner_name),
    )
    elapsed_ms, _, _ = _plan_call(planner, state, scene, 0.0, SimulationConfig(), True)
    return state, elapsed_ms


def _advance_waypoint(state: _RobotState, config: SimulationConfig) -> bool:
    """Advance reached waypoint indices and return whether the path is done."""
    if state.failed:
        return True
    while state.waypoint_index < len(state.path):
        target = state.path[state.waypoint_index]
        is_final = state.waypoint_index == len(state.path) - 1
        tolerance = config.final_tolerance_mm if is_final else config.waypoint_tolerance_mm
        if _distance(state.position, target) > tolerance:
            return False
        state.waypoint_index += 1
        state.reached_since_call += 1
    return True


def _next_position(
    state: _RobotState,
    step_s: float,
    config: SimulationConfig,
) -> Point:
    if _advance_waypoint(state, config):
        return state.position
    target = state.path[state.waypoint_index]
    distance = _distance(state.position, target)
    if distance <= 0:
        return state.position
    speed_mmps = min(config.max_speed_mmps, config.gain_per_second * distance)
    travel = min(distance, speed_mmps * step_s)
    ratio = travel / distance
    return (
        state.position[0] + (target[0] - state.position[0]) * ratio,
        state.position[1] + (target[1] - state.position[1]) * ratio,
    )


def _reflect(value: float, low: float, high: float) -> tuple[float, float]:
    """Fold an unbounded coordinate into [low, high]; return (value, direction)."""
    span = high - low
    if span <= 0:
        return (low + high) / 2.0, 0.0
    offset = (value - low) % (2.0 * span)
    if offset <= span:
        return low + offset, 1.0
    return low + 2.0 * span - offset, -1.0


def patrol_route(obstacle: ScenarioObstacle) -> tuple[Point, ...]:
    """Spawn point followed by the patrol waypoints (the line the UI draws)."""
    if not obstacle.patrol_waypoints:
        return ()
    return (tuple(obstacle.position_mm), *(tuple(p) for p in obstacle.patrol_waypoints))


def _patrol_state(obstacle: ScenarioObstacle, simulation_s: float) -> tuple[Point, Point]:
    """Constant-speed back-and-forth along spawn -> w1 -> ... -> wn -> ... -> spawn."""
    route = patrol_route(obstacle)
    segments = [(a, b, _distance(a, b)) for a, b in pairwise(route)]
    total = sum(length for _, _, length in segments)
    speed = obstacle.patrol_speed_mmps or DEFAULT_PATROL_SPEED_MMPS
    if total <= 0 or speed <= 0:
        return route[0], (0.0, 0.0)
    travelled = (speed * simulation_s) % (2.0 * total)
    direction = 1.0
    if travelled > total:
        travelled = 2.0 * total - travelled
        direction = -1.0
    for start, end, length in segments:
        if travelled <= length or (start, end, length) == segments[-1]:
            ratio = 0.0 if length <= 0 else min(1.0, travelled / length)
            ux = 0.0 if length <= 0 else (end[0] - start[0]) / length
            uy = 0.0 if length <= 0 else (end[1] - start[1]) / length
            return (
                (start[0] + (end[0] - start[0]) * ratio, start[1] + (end[1] - start[1]) * ratio),
                (direction * speed * ux, direction * speed * uy),
            )
        travelled -= length
    return route[-1], (0.0, 0.0)  # pragma: no cover - loop always returns


def _moving_obstacle_state(obstacle: ScenarioObstacle, simulation_s: float) -> tuple[Point, Point]:
    if obstacle.patrol_waypoints:
        return _patrol_state(obstacle, simulation_s)
    vx, vy = obstacle.velocity_mmps
    if vx == 0 and vy == 0:
        return obstacle.position_mm, (0.0, 0.0)
    x_min, x_max, y_min, y_max = FieldDimensions().bounds_mm
    radius = obstacle.radius_mm
    x, x_dir = _reflect(
        obstacle.position_mm[0] + vx * simulation_s, x_min + radius, x_max - radius
    )
    y, y_dir = _reflect(
        obstacle.position_mm[1] + vy * simulation_s, y_min + radius, y_max - radius
    )
    return (x, y), (vx * x_dir, vy * y_dir)


def _obstacle_position(obstacle: ScenarioObstacle, simulation_s: float) -> Point:
    """Patrol back-and-forth, or constant velocity bouncing off the field boundary."""
    return _moving_obstacle_state(obstacle, simulation_s)[0]


def _obstacle_velocity(obstacle: ScenarioObstacle, simulation_s: float) -> Point:
    return _moving_obstacle_state(obstacle, simulation_s)[1]


def _collisions(
    states: Sequence[_RobotState],
    obstacles: Sequence[ScenarioObstacle],
    simulation_s: float,
    contact_tolerance_mm: float = 0.0,
) -> tuple[set[tuple[RobotKey, RobotKey]], set[tuple[RobotKey, RobotKey]], float | None]:
    robot_pairs: set[tuple[RobotKey, RobotKey]] = set()
    obstacle_pairs: set[tuple[RobotKey, RobotKey]] = set()
    clearances: list[float] = []

    for index, first in enumerate(states):
        for second in states[index + 1 :]:
            clearance = _distance(first.position, second.position) - 2.0 * ROBOT_RADIUS_MM
            clearances.append(clearance)
            if clearance <= contact_tolerance_mm:
                robot_pairs.add(tuple(sorted((first.key, second.key))))

    for state in states:
        for obstacle in obstacles:
            obstacle_key = (obstacle.is_yellow, obstacle.obstacle_id)
            clearance = (
                _distance(state.position, _obstacle_position(obstacle, simulation_s))
                - ROBOT_RADIUS_MM
                - obstacle.radius_mm
            )
            clearances.append(clearance)
            if clearance <= contact_tolerance_mm:
                obstacle_pairs.add((state.key, obstacle_key))

    return robot_pairs, obstacle_pairs, min(clearances) if clearances else None


def simulate(
    scenario: Scenario,
    planner_name: str,
    *,
    config: SimulationConfig | None = None,
    trial: int = 1,
    seed: int = 0,
    scenario_set: str | None = None,
) -> HeadlessRunResult:
    """Plan and execute one scenario on a virtual clock without sleeping."""
    scenario.require_complete()
    if planner_name not in PLANNER_NAMES:
        raise ValueError(f"Unknown planner {planner_name!r}; choose from {PLANNER_NAMES}")
    if trial < 1:
        raise ValueError("trial must be at least 1")
    config = config or SimulationConfig()
    policy = config.replan_policy
    wall_started = perf_counter()
    planner = _new_planner(planner_name, seed, policy, config.periodic_reroute_frames)

    states = [
        _RobotState(robot=robot, path=(robot.start_mm,), failed=False, position=robot.start_mm)
        for robot in scenario.robots
    ]
    obstacles = tuple(scenario.obstacles_for(_planner_key(planner_name)))
    call_durations_ms: list[float] = []
    initial_durations_ms: list[float] = []
    replan_count = 0
    replan_failures = 0
    active_robot_collisions: set[tuple[RobotKey, RobotKey]] = set()
    active_obstacle_collisions: set[tuple[RobotKey, RobotKey]] = set()
    robot_collision_episodes = 0
    obstacle_collision_episodes = 0
    minimum_clearance_mm: float | None = None
    simulation_s = 0.0
    next_control_s = 0.0
    first_cycle = True
    ticks = 0

    while True:
        control_due = first_cycle or (
            policy != "once" and simulation_s + 1e-9 >= next_control_s
        )
        if control_due:
            for state in states:
                if state.failed:
                    continue
                if not first_cycle and _advance_waypoint(state, config):
                    continue  # arrived: stop consulting the planner
                scene = _scene_at(
                    scenario,
                    state,
                    states,
                    planner_name,
                    simulation_s,
                    config.prediction_horizon_ms,
                )
                elapsed_ms, replanned, failed = _plan_call(
                    planner, state, scene, simulation_s, config, first_cycle
                )
                call_durations_ms.append(elapsed_ms)
                if first_cycle:
                    initial_durations_ms.append(elapsed_ms)
                replan_count += replanned
                replan_failures += failed
            first_cycle = False
            while next_control_s <= simulation_s + 1e-9:
                next_control_s += config.control_period_s

        all_paths_done = all(_advance_waypoint(state, config) for state in states)
        current_robot, current_obstacle, clearance = _collisions(states, obstacles, simulation_s)
        robot_collision_episodes += len(current_robot - active_robot_collisions)
        obstacle_collision_episodes += len(current_obstacle - active_obstacle_collisions)
        active_robot_collisions = current_robot
        active_obstacle_collisions = current_obstacle
        if clearance is not None:
            minimum_clearance_mm = (
                clearance if minimum_clearance_mm is None else min(minimum_clearance_mm, clearance)
            )
        if all_paths_done or simulation_s >= config.max_simulation_s:
            break

        step_s = min(config.dt_s, config.max_simulation_s - simulation_s)
        if policy != "once":
            gap = next_control_s - simulation_s
            if gap > 1e-9:
                step_s = min(step_s, gap)
        next_positions = [_next_position(state, step_s, config) for state in states]
        for state, position in zip(states, next_positions):
            state.travelled_mm += _distance(state.position, position)
            state.velocity_mmps = (
                (position[0] - state.position[0]) / step_s,
                (position[1] - state.position[1]) / step_s,
            )
            state.position = position
        simulation_s += step_s
        ticks += 1

    failed_states = [state for state in states if state.failed]
    successful_paths_done = all(
        state.failed or _advance_waypoint(state, config) for state in states
    )
    completed = not failed_states and successful_paths_done
    timed_out = not completed and not failed_states
    status = "completed" if completed else "planning_failed" if failed_states else "timed_out"
    final_errors = [
        _distance(state.position, state.robot.target_mm)
        for state in states
        if state.robot.target_mm is not None
    ]
    wall_time_ms = (perf_counter() - wall_started) * 1000.0
    simulated_duration_ms = simulation_s * 1000.0
    realtime_factor = simulated_duration_ms / wall_time_ms if wall_time_ms > 0 else 0.0
    failed_robots = ",".join(
        f"Y{state.robot.robot_id}" if state.robot.is_yellow else f"B{state.robot.robot_id}"
        for state in failed_states
    )

    return HeadlessRunResult(
        scenario=scenario.name,
        planner=planner_name,
        trial=trial,
        seed=seed,
        status=status,
        completed=completed,
        timed_out=timed_out,
        robot_count=len(states),
        planner_calls=len(call_durations_ms),
        failed_plans=len(failed_states),
        failed_robots=failed_robots,
        planning_time_ms_total=sum(call_durations_ms),
        planning_time_ms_mean=fmean(call_durations_ms) if call_durations_ms else 0.0,
        simulated_duration_ms=simulated_duration_ms,
        wall_time_ms=wall_time_ms,
        realtime_factor=realtime_factor,
        ticks=ticks,
        robot_collision_episodes=robot_collision_episodes,
        obstacle_collision_episodes=obstacle_collision_episodes,
        collision_episodes=robot_collision_episodes + obstacle_collision_episodes,
        planned_path_length_mm=sum(_path_length(state.initial_path) for state in states),
        travelled_distance_mm=sum(state.travelled_mm for state in states),
        mean_final_error_mm=fmean(final_errors),
        max_final_error_mm=max(final_errors),
        minimum_clearance_mm=minimum_clearance_mm,
        dt_ms=config.dt_s * 1000.0,
        scenario_set=scenario_set or scenario.name,
        replan_policy=policy,
        replan_count=replan_count,
        replan_failures=replan_failures,
        planning_time_ms_initial=sum(initial_durations_ms),
        planning_time_ms_max_call=max(call_durations_ms, default=0.0),
        time_to_goal_ms=simulated_duration_ms if completed else None,
        prediction_horizon_ms=config.prediction_horizon_ms,
        replan_period_ms=config.control_period_s * 1000.0,
        planning_time_ms_p95_call=_percentile(call_durations_ms, 0.95)
        if call_durations_ms
        else 0.0,
        straight_line_mm=sum(
            _distance(state.robot.start_mm, state.robot.target_mm)
            for state in states
            if state.robot.target_mm is not None
        ),
    )


def _run_job(job: tuple) -> HeadlessRunResult:
    scenario, planner_name, config, trial, seed, scenario_set = job
    return simulate(
        scenario,
        planner_name,
        config=config,
        trial=trial,
        seed=seed,
        scenario_set=scenario_set,
    )


def run_experiments(
    scenarios: Sequence[Scenario],
    planners: Sequence[str] = PLANNER_NAMES,
    *,
    trials: int = 1,
    seed: int = 0,
    config: SimulationConfig | None = None,
    backend: str = "kinematic",
    grsim_binary: str | Path = ".local/grsim/bin/grSim",
    evidence_dir: str | Path = "results/physics-evidence",
    policies: Sequence[str] | None = None,
    workers: int = 1,
    scenario_sets: Sequence[str] | None = None,
    progress: bool = False,
    predictions_ms: Sequence[float] | None = None,
    replan_periods_ms: Sequence[float] | None = None,
) -> list[HeadlessRunResult]:
    """Run scenario x planner x policy x trial in stable order.

    ``scenario_sets`` optionally labels each scenario (for example ``random``)
    so summaries aggregate across a generated set. ``workers > 1`` runs the
    kinematic backend in a process pool; results keep the same order, but
    per-call planning latencies are noisier under parallel load.
    """
    if trials < 1:
        raise ValueError("trials must be at least 1")
    unknown = [name for name in planners if name not in PLANNER_NAMES]
    if unknown:
        raise ValueError(f"Unknown planner names: {unknown}")
    if scenario_sets is not None and len(scenario_sets) != len(scenarios):
        raise ValueError("scenario_sets must match scenarios one-to-one")
    labels = list(scenario_sets) if scenario_sets is not None else [s.name for s in scenarios]
    config = config or SimulationConfig(dt_s=1 / 120 if backend == "grsim" else 0.02)
    policies = tuple(policies) if policies else (config.replan_policy,)
    bad = [policy for policy in policies if policy not in REPLAN_POLICIES]
    if bad:
        raise ValueError(f"Unknown replan policies: {bad}")
    if workers < 1:
        raise ValueError("workers must be at least 1")
    predictions = (
        tuple(float(h) for h in predictions_ms)
        if predictions_ms
        else (config.prediction_horizon_ms,)
    )
    periods = (
        tuple(float(p) for p in replan_periods_ms)
        if replan_periods_ms
        else (config.control_period_s * 1000.0,)
    )
    if any(not isfinite(p) or p <= 0 for p in periods):
        raise ValueError("replan periods must be positive")
    if backend == "grsim":
        if any(policy != "once" for policy in policies) or any(predictions):
            raise ValueError("The grSim backend currently supports only --policy once")
        if any(o.patrol_waypoints for s in scenarios for o in s.obstacles):
            raise ValueError("The grSim backend does not drive patrol obstacles yet")
        from research_sdk.grsim_physics import simulate_physics

        return [
            replace(
                simulate_physics(
                    scenario,
                    planner_name,
                    config=config,
                    trial=trial,
                    seed=seed + trial - 1,
                    binary=Path(grsim_binary),
                    evidence_dir=Path(evidence_dir),
                ),
                scenario_set=label,
            )
            for scenario, label in zip(scenarios, labels)
            for planner_name in planners
            for trial in range(1, trials + 1)
        ]
    if backend != "kinematic":
        raise ValueError(f"Unknown simulation backend: {backend}")
    jobs = [
        (
            scenario,
            planner_name,
            replace(
                config,
                replan_policy=policy,
                prediction_horizon_ms=horizon,
                control_period_s=period / 1000.0,
            ),
            trial,
            seed + trial - 1,
            label,
        )
        for scenario, label in zip(scenarios, labels)
        for planner_name in planners
        for policy in policies
        for horizon in predictions
        for period in periods
        # Plan-once never replans, so the replan period cannot change it; and
        # with a no-prediction arm present its prediction variants are skipped.
        if not (policy == "once" and (period != periods[0] or (horizon > 0 and 0.0 in predictions)))
        for trial in range(1, trials + 1)
    ]
    step = max(1, len(jobs) // 20)
    results: list[HeadlessRunResult] = []

    def _collect(iterator) -> None:
        for count, result in enumerate(iterator, start=1):
            results.append(result)
            if progress and (count % step == 0 or count == len(jobs)):
                print(f"  {count}/{len(jobs)} runs", flush=True)

    if workers == 1 or len(jobs) < 2:
        _collect(map(_run_job, jobs))
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            _collect(pool.map(_run_job, jobs, chunksize=max(1, len(jobs) // (workers * 8))))
    return results


def _percentile(values: Sequence[float], percentile: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(len(ordered) * percentile))
    return ordered[index]


def _policy_label(result: HeadlessRunResult) -> str:
    if result.prediction_horizon_ms > 0:
        return f"{result.replan_policy}+pred{result.prediction_horizon_ms:g}"
    return result.replan_policy


def summarize_results(
    results: Sequence[HeadlessRunResult],
) -> list[dict[str, str | bool | int | float | None]]:
    """Aggregate raw trials by scenario set, planner, replan policy and backend."""
    groups: dict[tuple[str, str, str, str], list[HeadlessRunResult]] = {}
    for result in results:
        key = (
            result.scenario_set or result.scenario,
            result.planner,
            _policy_label(result),
            result.backend,
            result.prediction_horizon_ms,
            result.replan_period_ms,
        )
        groups.setdefault(key, []).append(result)

    summaries = []
    for (scenario, planner, policy, backend, horizon, period), runs in groups.items():
        planning = [run.planning_time_ms_total for run in runs]
        per_call = [run.planning_time_ms_mean for run in runs]
        wall = [run.wall_time_ms for run in runs]
        simulated = [run.simulated_duration_ms for run in runs]
        speedups = [run.realtime_factor for run in runs]
        goal_times = [run.time_to_goal_ms for run in runs if run.time_to_goal_ms is not None]
        clearances = [
            run.minimum_clearance_mm for run in runs if run.minimum_clearance_mm is not None
        ]
        summaries.append(
            {
                "scenario": scenario,
                "planner": planner,
                "replan_policy": policy,
                "prediction_horizon_ms": horizon,
                "replan_period_ms": round(period, 3),
                "backend": backend,
                "physics_validation_passes": sum(run.physics_validation_passed for run in runs),
                "runs": len(runs),
                "scenario_count": len({run.scenario for run in runs}),
                "completed_runs": sum(run.completed for run in runs),
                "completion_rate": sum(run.completed for run in runs) / len(runs),
                "failed_plans": sum(run.failed_plans for run in runs),
                "collision_episodes": sum(run.collision_episodes for run in runs),
                "collision_free_rate": sum(run.collision_episodes == 0 for run in runs)
                / len(runs),
                "planner_calls_mean": fmean(run.planner_calls for run in runs),
                "replans_mean": fmean(run.replan_count for run in runs),
                "replan_failures": sum(run.replan_failures for run in runs),
                "planning_time_ms_median": median(planning),
                "planning_time_ms_p95": _percentile(planning, 0.95),
                "planning_time_ms_mean": fmean(planning),
                "call_time_ms_median": median(per_call),
                "call_time_ms_max": max(run.planning_time_ms_max_call for run in runs),
                "time_to_goal_ms_mean": fmean(goal_times) if goal_times else None,
                "simulated_duration_ms_mean": fmean(simulated),
                "wall_time_ms_mean": fmean(wall),
                "realtime_factor_mean": fmean(speedups),
                "planned_path_length_mm_mean": fmean(run.planned_path_length_mm for run in runs),
                "travelled_distance_mm_mean": fmean(run.travelled_distance_mm for run in runs),
                "mean_final_error_mm": fmean(run.mean_final_error_mm for run in runs),
                "minimum_clearance_mm": min(clearances) if clearances else None,
                "near_miss_free_rate": sum(c >= NEAR_MISS_MM for c in clearances) / len(runs),
                "call_time_ms_p95_median": median(run.planning_time_ms_p95_call for run in runs),
                "planning_ms_per_sim_s_mean": fmean(
                    run.planning_time_ms_total / max(run.simulated_duration_ms / 1000.0, 1e-9)
                    for run in runs
                ),
                "path_efficiency_mean": fmean(
                    run.straight_line_mm / run.travelled_distance_mm
                    for run in runs
                    if run.travelled_distance_mm > 0
                )
                if any(run.travelled_distance_mm > 0 for run in runs)
                else None,
            }
        )
    return summaries


def write_results(
    results: Sequence[HeadlessRunResult],
    destination: str | Path,
    *,
    config: SimulationConfig,
    extra_manifest: dict | None = None,
) -> dict[str, Path]:
    """Write raw and aggregated CSV/JSON files plus a reproducibility manifest."""
    folder = Path(destination)
    folder.mkdir(parents=True, exist_ok=True)
    raw_records = [result.to_record() for result in results]
    summaries = summarize_results(results)
    paths = {
        "runs_csv": CSVExporter().export(raw_records, folder / "runs.csv"),
        "runs_json": JSONExporter().export(raw_records, folder / "runs.json"),
        "summary_csv": CSVExporter().export(summaries, folder / "summary.csv"),
        "summary_json": JSONExporter().export(summaries, folder / "summary.json"),
    }
    manifest = {
        "generated_at": datetime.now(UTC).isoformat(),
        "engine": "research_sdk.headless",
        "model": "See backend and evidence_directory for each run",
        "backends": sorted({result.backend for result in results}),
        "physics_equivalence": bool(results) and all(r.backend == "grsim" for r in results),
        "physics_equivalence_scope": "Recorded grSim model only, not physical hardware",
        "physics_validation_passed": bool(results)
        and all(r.physics_validation_passed for r in results),
        "config": asdict(config),
        "run_count": len(results),
        "scenarios": sorted({result.scenario for result in results}),
        "planners": sorted({result.planner for result in results}),
        "seeds": sorted({result.seed for result in results}),
        "replan_policies": sorted({result.replan_policy for result in results}),
        "prediction_horizons_ms": sorted({result.prediction_horizon_ms for result in results}),
        "replan_periods_ms": sorted({round(r.replan_period_ms, 3) for r in results}),
        "patrol_model": "constant speed back-and-forth along spawn + patrol_waypoints "
        f"(default {DEFAULT_PATROL_SPEED_MMPS:g} mm/s)",
        "prediction_model": "constant velocity; circle moved to predicted position and "
        "radius grown by predicted travel (WorldMap / Obstacle.dynamic_radius_0)",
        "scenario_sets": sorted({result.scenario_set or result.scenario for result in results}),
        "latency_model": "planning latency measured on wall clock; not applied to virtual time",
        **(extra_manifest or {}),
    }
    manifest_path = folder / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    paths["manifest"] = manifest_path
    return paths


def _scenario_paths(inputs: Sequence[str]) -> list[Path]:
    candidates = [Path(value) for value in inputs] if inputs else [Path("scenarios")]
    paths: list[Path] = []
    for candidate in candidates:
        if candidate.is_dir():
            paths.extend(sorted(candidate.glob("*.json")))
        elif candidate.is_file():
            paths.append(candidate)
        else:
            raise FileNotFoundError(f"Scenario path does not exist: {candidate}")
    return list(dict.fromkeys(paths))


def _load_scenarios(paths: Sequence[Path]) -> list[Scenario]:
    return [Scenario.from_dict(json.loads(path.read_text(encoding="utf-8"))) for path in paths]


def _default_output_folder() -> Path:
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S%fZ")
    return Path("results") / "headless" / stamp


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=("Run scenarios using fast kinematics or the native grSim physics engine.")
    )
    parser.add_argument(
        "scenarios",
        nargs="*",
        help="Scenario JSON files or directories (default: scenarios/)",
    )
    parser.add_argument(
        "--planner",
        nargs="+",
        choices=(*PLANNER_NAMES, "all"),
        default=["all"],
        help="Planner(s) to run (default: all)",
    )
    parser.add_argument("--trials", type=int, default=1, help="Trials per scenario/planner")
    parser.add_argument("--seed", type=int, default=0, help="Base PRM random seed")
    parser.add_argument("--backend", choices=("kinematic", "grsim"), default="kinematic")
    parser.add_argument("--grsim-bin", type=Path, default=Path(".local/grsim/bin/grSim"))
    parser.add_argument(
        "--dt-ms", type=float, default=None, help="Step in ms (default: kinematic 20, grSim 8.3333)"
    )
    parser.add_argument(
        "--max-sim-seconds",
        type=float,
        default=30.0,
        help="Virtual timeout per run",
    )
    parser.add_argument(
        "--speed-mps",
        type=float,
        default=ROBOT_MAX_LINEAR_SPEED_MPS,
        help="Maximum commanded robot speed (m/s)",
    )
    parser.add_argument(
        "--gain",
        type=float,
        default=2.0,
        help="Proportional waypoint controller gain per second",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Result folder (default: timestamped results/headless folder)",
    )
    parser.add_argument(
        "--policy",
        nargs="+",
        choices=(*REPLAN_POLICIES, "all"),
        default=["once"],
        help="Replanning policy/policies: once, cycle, event (default: once)",
    )
    parser.add_argument(
        "--control-hz", type=float, default=60.0, help="Replanning cycle rate (default 60)"
    )
    parser.add_argument(
        "--periodic-frames",
        type=int,
        default=None,
        help="Event policy: also reroute every N cycles while blocked (default: never)",
    )
    parser.add_argument(
        "--clearance-mm", type=float, default=0.0, help="Extra planning clearance (default 0)"
    )
    parser.add_argument(
        "--predict-ms",
        type=float,
        nargs="+",
        default=[0.0],
        help="Obstacle motion-prediction horizon(s) in ms (default 0 = off)",
    )
    parser.add_argument(
        "--replan-ms",
        type=float,
        nargs="+",
        default=None,
        help="Replanning period(s) in ms, e.g. 20 50 100 200 (default: 1000/--control-hz)",
    )
    parser.add_argument(
        "--random", type=int, default=0, help="Add N seeded random scenarios"
    )
    parser.add_argument(
        "--patrol-fraction",
        type=float,
        default=0.0,
        help="Random: share of moving obstacles that patrol waypoints instead of drifting",
    )
    parser.add_argument(
        "--patrol-points", type=int, default=3, help="Random: waypoints per patrol route"
    )
    parser.add_argument(
        "--perturb", type=int, default=0, help="Add N jittered variants of each saved scenario"
    )
    parser.add_argument(
        "--no-saved",
        action="store_true",
        help="Do not run the saved scenarios themselves (only their variants / random ones)",
    )
    parser.add_argument(
        "--scenario-seed", type=int, default=None, help="Generator seed (default: --seed)"
    )
    parser.add_argument("--robots", type=int, default=3, help="Random: controlled robots")
    parser.add_argument("--obstacles", type=int, default=8, help="Random: obstacles")
    parser.add_argument(
        "--moving-fraction", type=float, default=0.25, help="Random: share of moving obstacles"
    )
    parser.add_argument(
        "--obstacle-speed",
        type=float,
        nargs=2,
        default=(300.0, 1000.0),
        metavar=("MIN", "MAX"),
        help="Random: moving obstacle speed range in mm/s",
    )
    parser.add_argument(
        "--jitter-mm", type=float, default=150.0, help="Perturb: maximum position jitter"
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=1,
        help="Parallel processes for the kinematic backend (0 = all CPUs)",
    )
    parser.add_argument(
        "--fail-on-incomplete",
        action="store_true",
        help="Exit with status 2 if any run fails or times out",
    )
    return parser


def _build_cases(args) -> tuple[list[Scenario], list[str], GeneratorConfig]:
    generator = GeneratorConfig(
        robots=args.robots,
        obstacles=args.obstacles,
        moving_fraction=args.moving_fraction,
        patrol_fraction=args.patrol_fraction,
        patrol_points=args.patrol_points,
        min_obstacle_speed_mmps=args.obstacle_speed[0],
        max_obstacle_speed_mmps=args.obstacle_speed[1],
        jitter_mm=args.jitter_mm,
    )
    scenario_seed = args.seed if args.scenario_seed is None else args.scenario_seed
    only_random = args.random > 0 and not args.scenarios
    saved = [] if only_random else _load_scenarios(_scenario_paths(args.scenarios))
    scenarios: list[Scenario] = []
    labels: list[str] = []
    for base in saved:
        if not args.no_saved:
            scenarios.append(base)
            labels.append(base.name)
        for index in range(args.perturb):
            scenarios.append(perturb_scenario(base, index, scenario_seed, generator))
            labels.append(f"{base.name}~perturbed")
    for index in range(args.random):
        scenarios.append(random_scenario(index, scenario_seed, generator))
        labels.append("random")
    if not scenarios:
        raise ValueError("No scenarios selected")
    return scenarios, labels, generator


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        scenarios, labels, generator = _build_cases(args)
        planners = PLANNER_NAMES if "all" in args.planner else tuple(dict.fromkeys(args.planner))
        policies = (
            REPLAN_POLICIES if "all" in args.policy else tuple(dict.fromkeys(args.policy))
        )
        if args.control_hz <= 0:
            raise ValueError("--control-hz must be positive")
        workers = (os.cpu_count() or 1) if args.workers == 0 else args.workers
        config = SimulationConfig(
            dt_s=(
                args.dt_ms / 1000.0
                if args.dt_ms is not None
                else 1 / 120
                if args.backend == "grsim"
                else 0.02
            ),
            max_simulation_s=args.max_sim_seconds,
            max_speed_mmps=args.speed_mps * 1000.0,
            gain_per_second=args.gain,
            replan_policy=policies[0],
            control_period_s=1.0 / args.control_hz,
            periodic_reroute_frames=args.periodic_frames,
            planning_clearance_mm=args.clearance_mm,
        )
        output_folder = args.output_dir or _default_output_folder()
        if output_folder.exists() and any(output_folder.iterdir()):
            raise ValueError("Output directory must be new or empty to preserve previous results")
        scenario_folder = output_folder / "scenarios"
        scenario_folder.mkdir(parents=True, exist_ok=True)
        for scenario in scenarios:
            (scenario_folder / f"{scenario.name}.json").write_text(
                json.dumps(scenario.to_dict(), indent=2), encoding="utf-8"
            )
        if any(h < 0 for h in args.predict_ms):
            raise ValueError("--predict-ms must be non-negative")
        periods = args.replan_ms or [1000.0 / args.control_hz]
        total = (
            len(scenarios) * len(planners) * len(policies)
            * len(args.predict_ms) * len(periods) * args.trials
        )
        print(
            f"Running up to {total} runs: {len(scenarios)} scenarios x {len(planners)} planners"
            f" x {len(policies)} policies x {args.trials} trials on {workers} worker(s)",
            flush=True,
        )
        results = run_experiments(
            scenarios,
            planners,
            trials=args.trials,
            seed=args.seed,
            config=config,
            backend=args.backend,
            grsim_binary=args.grsim_bin,
            evidence_dir=output_folder / "evidence",
            policies=policies,
            workers=workers,
            scenario_sets=labels,
            progress=True,
            predictions_ms=args.predict_ms,
            replan_periods_ms=args.replan_ms,
        )
        write_results(
            results,
            output_folder,
            config=config,
            extra_manifest={
                "workers": workers,
                "generator": asdict(generator),
                "scenario_seed": args.seed if args.scenario_seed is None else args.scenario_seed,
                "random_scenarios": args.random,
                "perturbed_per_saved": args.perturb,
                "saved_included": not args.no_saved,
                "control_hz": args.control_hz,
                "replan_periods_ms_requested": periods,
            },
        )
    except (OSError, KeyError, TypeError, ValueError, RuntimeError) as exc:
        parser.error(str(exc))

    print(f"Completed {sum(result.completed for result in results)}/{len(results)} runs")
    if args.backend == "grsim":
        print(
            f"Physics validation passed: "
            f"{sum(r.physics_validation_passed for r in results)}/{len(results)}"
        )
    for summary in summarize_results(results):
        print(
            "{} / {} / {} @ {:g} ms: {:.0%} complete, {:.0%} collision-free, {:.1f} replans, "
            "{:.1f} ms planning/run (median), {:.0f}x real time".format(
                summary["scenario"],
                summary["planner"],
                summary["replan_policy"],
                summary["replan_period_ms"],
                summary["completion_rate"],
                summary["collision_free_rate"],
                summary["replans_mean"],
                summary["planning_time_ms_median"],
                summary["realtime_factor_mean"],
            )
        )
    print(f"Results: {output_folder.resolve()}")
    for result in results:
        if result.error:
            print(f"{result.scenario} / {result.planner}: {result.error}")
    if args.backend == "grsim" and not all(r.physics_validation_passed for r in results):
        return 2
    if args.fail_on_incomplete and not all(result.completed for result in results):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
