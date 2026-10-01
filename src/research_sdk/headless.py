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
* ``event_route`` -- as ``event``, but a blocked segment anywhere on the
  remaining route also counts as an event.

In ``cycle`` and ``event`` modes the scene given to the planner is rebuilt
every control cycle from the robots' current positions and velocities and the
moving obstacles' current positions. Planning latency is measured on the wall
clock but does not delay execution on the virtual clock.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from collections.abc import Sequence
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass, replace
from datetime import UTC, datetime
from itertools import pairwise
from math import atan2, hypot, isfinite, pi, sqrt
from pathlib import Path
from statistics import fmean, median
from time import perf_counter, sleep

from research_sdk.config import (
    ROBOT_MAX_LINEAR_SPEED_MPS,
    ROBOT_RADIUS_MM,
    planning_clearance_mm,
)
from research_sdk.exporters import CSVExporter, JSONExporter
from research_sdk.planners import (
    PlannerAPI,
    PlannerInput,
    PRMPlanner,
    VisibilityGraphPlanner,
    VoronoiDijkstraPlanner,
)
from research_sdk.planners.common import reset_search_time, search_time_ms
from research_sdk.scenario_generator import GeneratorConfig, perturb_scenario, random_scenario
from research_sdk.ui.scenarios import (
    DEFAULT_PATROL_SPEED_MMPS,
    Scenario,
    ScenarioObstacle,
    ScenarioRobot,
)
from research_sdk.world.map.geometry import distance_2_segment
from research_sdk.world.scene import FieldDimensions, PlanningObstacle, PlanningScene

Point = tuple[float, float]
RobotKey = tuple[bool, int]
PLANNER_NAMES = ("voronoi", "prm", "visibility")
REPLAN_POLICIES = ("once", "cycle", "event", "event_route")
NEAR_MISS_MM = 50.0
# Revised result model (DEC-016..018): a clearance below SAFETY_BUFFER_MM is a
# buffer violation, a clearance at or below zero is physical contact.
SAFETY_BUFFER_MM = 30.0
OUTCOME_CODES = ("S", "B", "P", "I", "X")  # strict, buffer-only, physical, incomplete, invalid
DEFAULT_SCENARIO_BANK = Path("scenarios") / "acra2026-200"
ACRA_SCENARIO_COUNT = 200
ONE_SHOT_RESULT_SET = "one-shot-validation"
# Dynamic result sets as (policy, prediction horizon ms, check interval ms) arms.
RESULT_SETS: dict[str, tuple[tuple[str, float, float], ...]] = {
    "policy-comparison": (
        ("cycle", 0.0, 20.0),
        ("event", 0.0, 20.0),
        ("cycle", 0.0, 100.0),
        ("event", 0.0, 100.0),
    ),
    # The 0 ms point of the sweep is the policy comparison's event @ 20 ms arm.
    "horizon-sweep": (
        ("event", 20.0, 20.0),
        ("event", 50.0, 20.0),
        ("event", 100.0, 20.0),
    ),
    # Supporting sets (DEC-010 ablation, DEC-019/020 evidence); compare each with
    # the matching arms of the main sets rather than reading it on its own.
    "event-route-ablation": (
        ("event_route", 0.0, 20.0),
        ("event_route", 0.0, 100.0),
    ),
    "horizon-150-check": (("event", 150.0, 20.0),),
    "no-route-500-check": (("event", 0.0, 20.0),),
}
# Result sets that change a configuration value as well as the arms.
RESULT_SET_OVERRIDES: dict[str, dict[str, float]] = {
    "no-route-500-check": {"no_route_limit_ms": 500.0},
}
TIME_SCALE_CHOICES = (1, 10, 100, 200, 500)
_PACING_INTERVAL_WALL_S = 0.01


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
    # None means "use config/planner_variables.yaml for the planner being run".
    planning_clearance_mm: float | None = None
    prediction_horizon_ms: float = 0.0
    # None preserves the original unpaced, maximum-throughput behavior.
    time_scale: float | None = None
    # Failure rules (DEC-019; None disables). A post-initial replan slower than
    # ``slow_call_limit_ms`` on the wall clock counts as no route: that robot is
    # stopped (``time_limit``) and the others continue. The initial plan (map
    # construction) is exempt and reported as one-shot timing. Any robot that
    # goes ``no_route_limit_ms`` of simulated time without a valid route fails
    # and ends the episode (``no_valid_path``).
    no_route_limit_ms: float | None = 1000.0
    slow_call_limit_ms: float | None = 100.0

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
        if self.planning_clearance_mm is not None and (
            not isfinite(self.planning_clearance_mm) or self.planning_clearance_mm < 0
        ):
            raise ValueError("planning_clearance_mm must be finite and non-negative")
        if self.periodic_reroute_frames is not None and self.periodic_reroute_frames < 1:
            raise ValueError("periodic_reroute_frames must be at least 1 or None")
        if self.time_scale is not None and (not isfinite(self.time_scale) or self.time_scale <= 0):
            raise ValueError("time_scale must be finite and positive, or None for unpaced")
        for name in ("no_route_limit_ms", "slow_call_limit_ms"):
            value = getattr(self, name)
            if value is not None and (not isfinite(value) or value <= 0):
                raise ValueError(f"{name} must be finite and positive, or None to disable")


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
    planning_clearance_mm: float = 0.0
    planning_time_ms_p95_call: float = 0.0
    straight_line_mm: float = 0.0
    replan_period_ms: float = 0.0
    # Where planning time went: calls that rebuilt a route vs cheap checks.
    rebuild_calls: int = 0
    check_calls: int = 0
    planning_time_ms_rebuild: float = 0.0
    planning_time_ms_check: float = 0.0
    # Why routes were rebuilt (executor-side classification, see _replan_reason).
    replans_active_blocked: int = 0
    replans_route_finished: int = 0
    replans_route_blocked: int = 0
    replans_scheduled: int = 0
    replans_other: int = 0
    direct_path_switches: int = 0
    # How much each rebuild moved the route (mean distance of the new route's
    # first 1.5 m from the old remaining route).
    path_shift_mm_mean: float = 0.0
    path_shift_mm_max: float = 0.0
    # Executed-motion quality.
    heading_change_rad_per_m: float = 0.0
    sharp_turns: int = 0
    minimum_robot_clearance_mm: float | None = None
    minimum_obstacle_clearance_mm: float | None = None
    contact_time_ms: float = 0.0
    # Fault attribution for robot-vs-obstacle contacts, decided at the first tick
    # of each episode (see _obstacle_initiated). Patrol obstacles follow a fixed,
    # time-based route and never react to robots, so an episode the obstacle
    # closed on its own could not have been avoided by replanning.
    obstacle_episodes_obstacle_initiated: int = 0
    obstacle_episodes_robot_initiated: int = 0
    # --- Revised result model (DEC-016/017/018) -------------------------------
    # Outcome: S strict, B buffer-only, P physical contact, I incomplete,
    # X invalid scenario (geometric overlap at t = 0).
    outcome: str = ""
    initial_overlap: bool = False
    # Swept checks between consecutive states; obstacle-obstacle is not recorded.
    rr_contact_episodes: int = 0
    ro_contact_episodes: int = 0
    rr_buffer_episodes: int = 0
    ro_buffer_episodes: int = 0
    ro_contact_obstacle_initiated: int = 0
    min_swept_clearance_mm: float | None = None
    first_contact_ms: float | None = None
    # Per-robot stops (the rest of the team keeps running).
    robots_stopped_no_path: int = 0
    robots_stopped_time_limit: int = 0
    stopped_robots: str = ""
    # "" when the episode ran to completion or the time limit; "no_valid_path"
    # when a robot went too long without a route and the episode was failed.
    episode_end_reason: str = ""
    # Initial no-route handling: the robot waits at its start and retries.
    initial_retries_total: int = 0
    initial_wait_ms_total: float = 0.0
    initial_wait_ms_max: float = 0.0
    # Route persistence (initial route included).
    route_count: int = 0
    route_lifetime_ms_total: float = 0.0
    route_lifetime_ms_max: float = 0.0
    # Why each installed route ended.
    route_ends_replaced: int = 0
    route_ends_goal: int = 0
    route_ends_stopped: int = 0
    route_ends_episode_end: int = 0
    # Per robot "B0:3r/60ms" = 3 failed initial attempts, 60 ms waiting.
    initial_route_by_robot: str = ""
    # Successful post-initial rebuilds only: failed attempts are counted in
    # ``replan_failures`` and kept out of the latency statistics (DEC-019).
    successful_rebuilds: int = 0
    rebuild_ms_min: float | None = None
    rebuild_ms_max: float | None = None
    rebuild_ms_total: float = 0.0
    event_checks: int = 0
    event_check_ms_total: float = 0.0
    # Stage split of successful initial plans: Dijkstra search vs everything else.
    initial_plan_calls: int = 0
    initial_search_ms_total: float = 0.0
    initial_map_ms_total: float = 0.0
    rebuild_search_ms_total: float = 0.0
    # Individual rebuild latencies; exported to rebuild_calls.csv, not runs.csv.
    rebuild_latencies_ms: tuple[float, ...] = ()

    def to_record(self) -> dict[str, str | bool | int | float | None]:
        record = asdict(self)
        record.pop("rebuild_latencies_ms", None)
        return record


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
    last_heading: float | None = None
    heading_change_rad: float = 0.0
    sharp_turns: int = 0
    # True while the robot has no initial route and waits at its start.
    awaiting_route: bool = False
    initial_attempts: int = 0
    first_route_s: float | None = None
    route_started_s: float | None = None
    fail_streak_start_s: float | None = None
    stop_reason: str = ""
    stopped_s: float | None = None

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
        "check_full_route": policy == "event_route",
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
    record: dict | None = None,
) -> tuple[float, int, int]:
    """Consult the planner once; return (elapsed ms, replanned, failed).

    ``record`` (optional) receives ``kind`` (initial / rebuild / switch_direct /
    check / failed), the ``reason`` a rebuild would be attributed to, and the
    rebuild's ``shift_mm``.
    """
    robot = state.robot
    assert robot.target_mm is not None
    target = (float(robot.target_mm[0]), float(robot.target_mm[1]))
    record = {} if record is None else record
    old_route = (state.position, *state.path[state.waypoint_index :])
    record["reason"] = "initial" if first else _replan_reason(state, scene, config)
    if config.replan_policy == "cycle" and not first:
        # Per-tick baseline: discard cached routes so every call is a rebuild.
        planner.reset(robot_id=robot.robot_id, is_yellow=robot.is_yellow)
    reached = state.reached_since_call > 0
    state.reached_since_call = 0
    reset_search_time()
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
    record["search_ms"] = min(search_time_ms(), elapsed_ms)
    waypoints = (
        ()
        if output is None
        else tuple((float(point[0]), float(point[1])) for point in output.waypoints)
    )

    if first:
        success = output is not None and (
            output.is_path_free or bool(waypoints) or _distance(robot.start_mm, target) <= 1e-9
        )
        state.path = (
            _dedupe_path((robot.start_mm, *waypoints, target)) if success else (robot.start_mm,)
        )
        state.failed = not success
        state.initial_path = state.path
        state.waypoint_index = 1
        record["kind"] = "initial" if success else "failed"
        return elapsed_ms, 0, 0

    record["kind"] = "check"
    if output is None:
        record["kind"] = "failed"
        return elapsed_ms, 0, 1
    if output.is_path_free:
        if state.path[state.waypoint_index :] != (target,):
            state.path = _dedupe_path((state.position, target))
            state.waypoint_index = 1
            record["kind"] = "switch_direct"
        return elapsed_ms, 0, 0
    if not output.did_reroute:
        return elapsed_ms, 0, 0
    if not waypoints:
        # Rebuild attempted but produced nothing: keep executing the old route.
        record["kind"] = "failed"
        return elapsed_ms, 0, 1
    new_path = _dedupe_path((state.position, *waypoints, target))
    if len(new_path) <= 2:
        # PRM/visibility report a clear straight line as a two-point "route";
        # treat it like Voronoi's is_path_free so replan counts are comparable.
        if state.path[state.waypoint_index :] != (target,):
            state.path = new_path
            state.waypoint_index = 1
            record["kind"] = "switch_direct"
        return elapsed_ms, 0, 0
    state.path = new_path
    state.waypoint_index = 1
    record["kind"] = "rebuild"
    record["shift_mm"] = _route_shift(old_route, new_path)
    return elapsed_ms, 1, 0


def _replan_reason(state: _RobotState, scene: PlanningScene, config: SimulationConfig) -> str:
    """Attribute a potential rebuild to the executor-visible event that explains it.

    ``scheduled`` for the every-cycle policy; otherwise ``active_blocked`` when
    the segment to the next waypoint is no longer clear, ``route_finished``
    when no waypoints remain, and ``other`` (e.g. a periodic safety reroute).
    """
    if config.replan_policy == "cycle":
        return "scheduled"
    if state.waypoint_index >= len(state.path):
        return "route_finished"
    free = scene.is_path_free(
        state.position,
        state.path[state.waypoint_index],
        ignore_robots={state.key},
        clearance=config.planning_clearance_mm,
    )
    if not free:
        return "active_blocked"
    route = (*state.path[state.waypoint_index :],)
    for first, second in pairwise(route):
        if not scene.is_path_free(
            first, second, ignore_robots={state.key}, clearance=config.planning_clearance_mm
        ):
            return "route_blocked"
    return "other"


def _route_shift(
    old_route: Sequence[Point],
    new_route: Sequence[Point],
    ahead_mm: float = 1500.0,
    step_mm: float = 100.0,
) -> float:
    """Mean distance of the new route's first ``ahead_mm`` from the old route."""
    old_segments = list(pairwise(old_route)) or [(old_route[0], old_route[0])]
    samples: list[Point] = []
    travelled = 0.0
    for start, end in pairwise(new_route):
        length = _distance(start, end)
        offset = 0.0
        while offset <= length and travelled + offset <= ahead_mm:
            ratio = 0.0 if length <= 0 else offset / length
            samples.append(
                (start[0] + (end[0] - start[0]) * ratio, start[1] + (end[1] - start[1]) * ratio)
            )
            offset += step_mm
        travelled += length
        if travelled > ahead_mm:
            break
    if not samples:
        return 0.0
    return fmean(min(distance_2_segment(point, a, b) for a, b in old_segments) for point in samples)


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
    if state.awaiting_route:
        return False
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
    if state.awaiting_route or _advance_waypoint(state, config):
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
    x, x_dir = _reflect(obstacle.position_mm[0] + vx * simulation_s, x_min + radius, x_max - radius)
    y, y_dir = _reflect(obstacle.position_mm[1] + vy * simulation_s, y_min + radius, y_max - radius)
    return (x, y), (vx * x_dir, vy * y_dir)


def _obstacle_position(obstacle: ScenarioObstacle, simulation_s: float) -> Point:
    """Patrol back-and-forth, or constant velocity bouncing off the field boundary."""
    return _moving_obstacle_state(obstacle, simulation_s)[0]


def _obstacle_velocity(obstacle: ScenarioObstacle, simulation_s: float) -> Point:
    return _moving_obstacle_state(obstacle, simulation_s)[1]


def _obstacle_initiated(
    robot_position: Point,
    robot_velocity: Point,
    obstacle: ScenarioObstacle,
    simulation_s: float,
) -> bool:
    """True when the obstacle, not the robot, closed the gap into contact.

    ``normal`` points from the obstacle to the robot, so projecting the
    obstacle's velocity onto it gives the rate at which the obstacle approaches
    the robot, and negating the robot's projection gives the rate at which the
    robot approaches the obstacle. Whichever is larger is charged with the
    contact. Patrol obstacles ignore robots entirely, so obstacle-initiated
    episodes are not attributable to the planner.
    """
    obstacle_position = _obstacle_position(obstacle, simulation_s)
    separation = _distance(robot_position, obstacle_position)
    if separation < 1e-9:
        return False
    normal = (
        (robot_position[0] - obstacle_position[0]) / separation,
        (robot_position[1] - obstacle_position[1]) / separation,
    )
    obstacle_velocity = _obstacle_velocity(obstacle, simulation_s)
    obstacle_closing = obstacle_velocity[0] * normal[0] + obstacle_velocity[1] * normal[1]
    robot_closing = -(robot_velocity[0] * normal[0] + robot_velocity[1] * normal[1])
    return obstacle_closing > robot_closing


def _collisions(
    states: Sequence[_RobotState],
    obstacles: Sequence[ScenarioObstacle],
    simulation_s: float,
    contact_tolerance_mm: float = 0.0,
    split: dict | None = None,
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

    if split is not None:
        robot_count = len(states) * (len(states) - 1) // 2
        for name, values in (
            ("robot", clearances[:robot_count]),
            ("obstacle", clearances[robot_count:]),
        ):
            if values:
                current = split.get(name)
                split[name] = min(values) if current is None else min(current, min(values))
    return robot_pairs, obstacle_pairs, min(clearances) if clearances else None


def _swept_min_distance(a0: Point, a1: Point, b0: Point, b1: Point) -> float:
    """Minimum centre distance of two points moving linearly from *0 to *1.

    Both bodies are interpolated over the same interval, so a contact that
    starts and ends between two sampled states is still found.
    """
    rx, ry = a0[0] - b0[0], a0[1] - b0[1]
    dx = (a1[0] - a0[0]) - (b1[0] - b0[0])
    dy = (a1[1] - a0[1]) - (b1[1] - b0[1])
    denom = dx * dx + dy * dy
    t = 0.0 if denom <= 1e-12 else min(1.0, max(0.0, -(rx * dx + ry * dy) / denom))
    return hypot(rx + t * dx, ry + t * dy)


def _swept_clearances(
    states: Sequence[_RobotState],
    before: Sequence[Point],
    after: Sequence[Point],
    obstacles: Sequence[ScenarioObstacle],
    t0_s: float,
    t1_s: float,
) -> tuple[dict[tuple[RobotKey, RobotKey], float], dict[tuple[RobotKey, RobotKey], float]]:
    """Minimum robot-robot and robot-obstacle clearance over [t0_s, t1_s].

    Obstacle-obstacle pairs are deliberately not evaluated (DEC-017).
    """
    robot_pairs: dict[tuple[RobotKey, RobotKey], float] = {}
    obstacle_pairs: dict[tuple[RobotKey, RobotKey], float] = {}
    for i, first in enumerate(states):
        for j in range(i + 1, len(states)):
            key = tuple(sorted((first.key, states[j].key)))
            robot_pairs[key] = (
                _swept_min_distance(before[i], after[i], before[j], after[j])
                - 2.0 * ROBOT_RADIUS_MM
            )
    for obstacle in obstacles:
        o0 = _obstacle_position(obstacle, t0_s)
        o1 = _obstacle_position(obstacle, t1_s)
        obstacle_key = (obstacle.is_yellow, obstacle.obstacle_id)
        for i, state in enumerate(states):
            obstacle_pairs[(state.key, obstacle_key)] = (
                _swept_min_distance(before[i], after[i], o0, o1)
                - ROBOT_RADIUS_MM
                - obstacle.radius_mm
            )
    return robot_pairs, obstacle_pairs


def count_new_episodes(previous: set, current: set) -> int:
    """Pairs in contact now that were not in contact in the previous interval.

    A sustained overlap is one episode; separating and touching again is a new one.
    """
    return len(current - previous)


def _initial_overlap(states: Sequence[_RobotState], obstacles: Sequence[ScenarioObstacle]) -> bool:
    """True when any robot already touches a robot or obstacle at t = 0."""
    robot_pairs, obstacle_pairs, _ = _collisions(states, obstacles, 0.0)
    return bool(robot_pairs or obstacle_pairs)


def classify_outcome(
    *, completed: bool, contact_episodes: int, buffer_episodes: int, initial_overlap: bool
) -> str:
    """Highest-severity run class: X invalid, I incomplete, P contact, B buffer, S strict."""
    if initial_overlap:
        return "X"
    if not completed:
        return "I"
    if contact_episodes > 0:
        return "P"
    if buffer_episodes > 0:
        return "B"
    return "S"


def _stop_robot(state: _RobotState, reason: str, simulation_s: float) -> None:
    """Freeze a robot in place for the rest of the episode and flag why."""
    state.failed = True
    state.awaiting_route = False
    state.stop_reason = reason
    state.stopped_s = simulation_s
    state.velocity_mmps = (0.0, 0.0)


def _pace_virtual_clock(simulation_s: float, wall_started: float, time_scale: float) -> None:
    """Prevent virtual time from running ahead of the selected wall-time ratio."""
    delay_s = wall_started + simulation_s / time_scale - perf_counter()
    if delay_s > 0:
        sleep(delay_s)


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
    if config.planning_clearance_mm is None:
        config = replace(config, planning_clearance_mm=planning_clearance_mm(planner_name))
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
    rebuild_ms: list[float] = []
    check_ms: list[float] = []
    reasons = {
        "active_blocked": 0,
        "route_blocked": 0,
        "route_finished": 0,
        "scheduled": 0,
        "other": 0,
    }
    direct_switches = 0
    shifts: list[float] = []
    split_clearance: dict = {}
    contact_time_s = 0.0
    active_robot_collisions: set[tuple[RobotKey, RobotKey]] = set()
    active_obstacle_collisions: set[tuple[RobotKey, RobotKey]] = set()
    robot_collision_episodes = 0
    obstacle_collision_episodes = 0
    obstacle_episodes_obstacle_initiated = 0
    obstacle_episodes_robot_initiated = 0
    state_by_key = {state.key: state for state in states}
    obstacle_by_key = {
        (obstacle.is_yellow, obstacle.obstacle_id): obstacle for obstacle in obstacles
    }
    minimum_clearance_mm: float | None = None
    # Revised result model trackers (DEC-016/017/018).
    swept_rr_contact: set = set()
    swept_ro_contact: set = set()
    swept_rr_buffer: set = set()
    swept_ro_buffer: set = set()
    rr_contact_episodes = 0
    ro_contact_episodes = 0
    rr_buffer_episodes = 0
    ro_buffer_episodes = 0
    ro_contact_obstacle_initiated = 0
    min_swept_clearance: float | None = None
    first_contact_s: float | None = None
    route_lifetimes_s: list[float] = []
    route_ends = {"replaced": 0, "goal": 0, "stopped": 0, "episode_end": 0}

    def close_route(state: _RobotState, end_s: float, reason: str) -> None:
        if state.route_started_s is None:
            return
        route_lifetimes_s.append(end_s - state.route_started_s)
        route_ends[reason] += 1
        state.route_started_s = None
    rebuild_attempt_ms: list[float] = []
    rebuild_search_ms: list[float] = []
    event_check_ms: list[float] = []
    initial_search_ms: list[float] = []
    initial_map_ms: list[float] = []
    initial_overlap = _initial_overlap(states, obstacles)
    no_route_limit_s = (
        None if config.no_route_limit_ms is None else config.no_route_limit_ms / 1000.0
    )
    episode_end_reason = ""
    simulation_s = 0.0
    next_control_s = 0.0
    next_pace_s = (
        config.time_scale * _PACING_INTERVAL_WALL_S if config.time_scale is not None else None
    )
    first_cycle = True
    ticks = 0

    while True:
        # Robots without an initial route retry once per check interval, even
        # under the plan-once policy.
        pending_initial = any(state.awaiting_route for state in states)
        control_due = first_cycle or (
            (policy != "once" or pending_initial) and simulation_s + 1e-9 >= next_control_s
        )
        if control_due:
            for state in states:
                if state.failed:
                    continue
                first = first_cycle or state.awaiting_route
                if policy == "once" and not first:
                    continue
                if not first and _advance_waypoint(state, config):
                    continue  # arrived: stop consulting the planner
                scene = _scene_at(
                    scenario,
                    state,
                    states,
                    planner_name,
                    simulation_s,
                    config.prediction_horizon_ms,
                )
                record: dict = {}
                elapsed_ms, replanned, failed = _plan_call(
                    planner, state, scene, simulation_s, config, first, record
                )
                call_durations_ms.append(elapsed_ms)
                kind = record.get("kind")
                # A post-initial rebuild attempt either produced a new multi-waypoint
                # route or produced no route at all. A result that only confirms the
                # direct line (``check`` / ``switch_direct``) is a check, even when the
                # planner reports it rerouted -- e.g. after the cycle policy's reset.
                record["rebuild_attempted"] = not first and kind in ("rebuild", "failed")
                if kind in ("initial", "rebuild"):
                    rebuild_ms.append(elapsed_ms)
                else:
                    check_ms.append(elapsed_ms)
                if kind == "rebuild":
                    reasons[record["reason"]] = reasons.get(record["reason"], 0) + 1
                    shifts.append(record.get("shift_mm", 0.0))
                elif kind == "switch_direct":
                    direct_switches += 1
                if first:
                    initial_durations_ms.append(elapsed_ms)
                    state.initial_attempts += 1
                    if kind == "initial":
                        state.awaiting_route = False
                        state.first_route_s = simulation_s
                        state.route_started_s = simulation_s
                        initial_search_ms.append(record["search_ms"])
                        initial_map_ms.append(elapsed_ms - record["search_ms"])
                    else:
                        # No route yet: wait at the start and retry next interval.
                        state.failed = False
                        state.awaiting_route = True
                else:
                    if record.get("rebuild_attempted"):
                        if kind == "rebuild":  # failures count, but not in latency
                            rebuild_attempt_ms.append(elapsed_ms)
                            rebuild_search_ms.append(record["search_ms"])
                    else:
                        event_check_ms.append(elapsed_ms)
                    if kind in ("rebuild", "switch_direct") and state.route_started_s is not None:
                        close_route(state, simulation_s, "replaced")
                        state.route_started_s = simulation_s
                # Per-robot stop rules: too slow a call, or too long without a route.
                if kind == "failed":
                    if state.fail_streak_start_s is None:
                        state.fail_streak_start_s = simulation_s
                else:
                    state.fail_streak_start_s = None
                stop_reason = ""
                if (
                    config.slow_call_limit_ms is not None
                    and not first
                    and record.get("rebuild_attempted")
                    and elapsed_ms > config.slow_call_limit_ms
                ):
                    stop_reason = "time_limit"
                elif (
                    no_route_limit_s is not None
                    and state.fail_streak_start_s is not None
                    and simulation_s - state.fail_streak_start_s + 1e-9 >= no_route_limit_s
                ):
                    stop_reason = "no_valid_path"
                if stop_reason:
                    close_route(state, simulation_s, "stopped")
                    _stop_robot(state, stop_reason, simulation_s)
                    if stop_reason == "no_valid_path":
                        episode_end_reason = "no_valid_path"
                replan_count += replanned
                replan_failures += failed
            first_cycle = False
            while next_control_s <= simulation_s + 1e-9:
                next_control_s += config.control_period_s
            if episode_end_reason:
                break  # a robot went too long without a route: the episode fails

        all_paths_done = all(_advance_waypoint(state, config) for state in states)
        current_robot, current_obstacle, clearance = _collisions(
            states, obstacles, simulation_s, split=split_clearance
        )
        new_obstacle_contacts = current_obstacle - active_obstacle_collisions
        robot_collision_episodes += len(current_robot - active_robot_collisions)
        obstacle_collision_episodes += len(new_obstacle_contacts)
        for robot_key, obstacle_key in new_obstacle_contacts:
            contact_state = state_by_key.get(robot_key)
            contact_obstacle = obstacle_by_key.get(obstacle_key)
            if contact_state is None or contact_obstacle is None:
                continue
            if _obstacle_initiated(
                contact_state.position,
                contact_state.velocity_mmps,
                contact_obstacle,
                simulation_s,
            ):
                obstacle_episodes_obstacle_initiated += 1
            else:
                obstacle_episodes_robot_initiated += 1
        active_robot_collisions = current_robot
        active_obstacle_collisions = current_obstacle
        if clearance is not None:
            minimum_clearance_mm = (
                clearance if minimum_clearance_mm is None else min(minimum_clearance_mm, clearance)
            )
        if all_paths_done or simulation_s >= config.max_simulation_s:
            break

        step_s = min(config.dt_s, config.max_simulation_s - simulation_s)
        if policy != "once" or any(state.awaiting_route for state in states):
            gap = next_control_s - simulation_s
            if gap > 1e-9:
                step_s = min(step_s, gap)
        if current_robot or current_obstacle:
            contact_time_s += step_s
        next_positions = [_next_position(state, step_s, config) for state in states]
        rr_now, ro_now = _swept_clearances(
            states,
            [state.position for state in states],
            next_positions,
            obstacles,
            simulation_s,
            simulation_s + step_s,
        )
        rr_contact_now = {key for key, value in rr_now.items() if value <= 0.0}
        ro_contact_now = {key for key, value in ro_now.items() if value <= 0.0}
        rr_buffer_now = {key for key, value in rr_now.items() if value < SAFETY_BUFFER_MM}
        ro_buffer_now = {key for key, value in ro_now.items() if value < SAFETY_BUFFER_MM}
        new_ro_contacts = ro_contact_now - swept_ro_contact
        rr_contact_episodes += count_new_episodes(swept_rr_contact, rr_contact_now)
        ro_contact_episodes += count_new_episodes(swept_ro_contact, ro_contact_now)
        rr_buffer_episodes += count_new_episodes(swept_rr_buffer, rr_buffer_now)
        ro_buffer_episodes += count_new_episodes(swept_ro_buffer, ro_buffer_now)
        for robot_key, obstacle_key in new_ro_contacts:
            contact_state = state_by_key.get(robot_key)
            contact_obstacle = obstacle_by_key.get(obstacle_key)
            if contact_state is not None and contact_obstacle is not None and _obstacle_initiated(
                contact_state.position, contact_state.velocity_mmps, contact_obstacle, simulation_s
            ):
                ro_contact_obstacle_initiated += 1
        if first_contact_s is None and (rr_contact_now or ro_contact_now):
            first_contact_s = simulation_s
        interval_values = [*rr_now.values(), *ro_now.values()]
        if interval_values:
            lowest = min(interval_values)
            min_swept_clearance = (
                lowest if min_swept_clearance is None else min(min_swept_clearance, lowest)
            )
        swept_rr_contact, swept_ro_contact = rr_contact_now, ro_contact_now
        swept_rr_buffer, swept_ro_buffer = rr_buffer_now, ro_buffer_now
        for state, position in zip(states, next_positions):
            moved = _distance(state.position, position)
            if moved > 1e-6:
                heading = atan2(position[1] - state.position[1], position[0] - state.position[0])
                if state.last_heading is not None:
                    turn = abs((heading - state.last_heading + pi) % (2 * pi) - pi)
                    state.heading_change_rad += turn
                    if turn > pi / 2:
                        state.sharp_turns += 1
                state.last_heading = heading
            state.travelled_mm += moved
            state.velocity_mmps = (
                (position[0] - state.position[0]) / step_s,
                (position[1] - state.position[1]) / step_s,
            )
            state.position = position
        simulation_s += step_s
        ticks += 1
        for state in states:
            # A route's lifetime ends when its robot reaches the goal.
            if state.route_started_s is not None and _advance_waypoint(state, config):
                close_route(state, simulation_s, "goal")
        if next_pace_s is not None and simulation_s + 1e-9 >= next_pace_s:
            _pace_virtual_clock(simulation_s, wall_started, config.time_scale)
            next_pace_s = simulation_s + config.time_scale * _PACING_INTERVAL_WALL_S

    for state in states:
        # Still active when the episode ended (time limit or episode failure).
        close_route(state, simulation_s, "episode_end")
        if state.awaiting_route:  # never obtained a route
            state.awaiting_route = False
            state.failed = True
    initial_waits_s = [
        state.first_route_s
        if state.first_route_s is not None
        else state.stopped_s
        if state.stopped_s is not None
        else simulation_s
        for state in states
    ]
    stopped = [state for state in states if state.stop_reason]
    initial_retries = sum(
        state.initial_attempts - (1 if state.first_route_s is not None else 0) for state in states
    )
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
    simulated_duration_ms = simulation_s * 1000.0
    if config.time_scale is not None:
        _pace_virtual_clock(simulation_s, wall_started, config.time_scale)
    wall_time_ms = (perf_counter() - wall_started) * 1000.0
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
        planning_clearance_mm=config.planning_clearance_mm,
        replan_period_ms=config.control_period_s * 1000.0,
        planning_time_ms_p95_call=_percentile(call_durations_ms, 0.95)
        if call_durations_ms
        else 0.0,
        rebuild_calls=len(rebuild_ms),
        check_calls=len(check_ms),
        planning_time_ms_rebuild=sum(rebuild_ms),
        planning_time_ms_check=sum(check_ms),
        replans_active_blocked=reasons["active_blocked"],
        replans_route_finished=reasons["route_finished"],
        replans_route_blocked=reasons["route_blocked"],
        replans_scheduled=reasons["scheduled"],
        replans_other=reasons["other"],
        direct_path_switches=direct_switches,
        path_shift_mm_mean=fmean(shifts) if shifts else 0.0,
        path_shift_mm_max=max(shifts, default=0.0),
        heading_change_rad_per_m=(
            sum(state.heading_change_rad for state in states)
            / max(sum(state.travelled_mm for state in states) / 1000.0, 1e-9)
        ),
        sharp_turns=sum(state.sharp_turns for state in states),
        minimum_robot_clearance_mm=split_clearance.get("robot"),
        minimum_obstacle_clearance_mm=split_clearance.get("obstacle"),
        contact_time_ms=contact_time_s * 1000.0,
        obstacle_episodes_obstacle_initiated=obstacle_episodes_obstacle_initiated,
        obstacle_episodes_robot_initiated=obstacle_episodes_robot_initiated,
        straight_line_mm=sum(
            _distance(state.robot.start_mm, state.robot.target_mm)
            for state in states
            if state.robot.target_mm is not None
        ),
        outcome=classify_outcome(
            completed=completed,
            contact_episodes=rr_contact_episodes + ro_contact_episodes,
            buffer_episodes=rr_buffer_episodes + ro_buffer_episodes,
            initial_overlap=initial_overlap,
        ),
        initial_overlap=initial_overlap,
        rr_contact_episodes=rr_contact_episodes,
        ro_contact_episodes=ro_contact_episodes,
        rr_buffer_episodes=rr_buffer_episodes,
        ro_buffer_episodes=ro_buffer_episodes,
        ro_contact_obstacle_initiated=ro_contact_obstacle_initiated,
        min_swept_clearance_mm=min_swept_clearance,
        first_contact_ms=None if first_contact_s is None else first_contact_s * 1000.0,
        robots_stopped_no_path=sum(state.stop_reason == "no_valid_path" for state in stopped),
        robots_stopped_time_limit=sum(state.stop_reason == "time_limit" for state in stopped),
        episode_end_reason=episode_end_reason,
        stopped_robots=",".join(
            f"{'Y' if state.robot.is_yellow else 'B'}{state.robot.robot_id}:"
            f"{state.stop_reason}@{state.stopped_s * 1000.0:.0f}ms"
            for state in stopped
        ),
        initial_retries_total=initial_retries,
        initial_wait_ms_total=sum(initial_waits_s) * 1000.0,
        initial_wait_ms_max=max(initial_waits_s, default=0.0) * 1000.0,
        route_count=len(route_lifetimes_s),
        route_lifetime_ms_total=sum(route_lifetimes_s) * 1000.0,
        route_lifetime_ms_max=max(route_lifetimes_s, default=0.0) * 1000.0,
        route_ends_replaced=route_ends["replaced"],
        route_ends_goal=route_ends["goal"],
        route_ends_stopped=route_ends["stopped"],
        route_ends_episode_end=route_ends["episode_end"],
        initial_route_by_robot=";".join(
            f"{'Y' if state.robot.is_yellow else 'B'}{state.robot.robot_id}:"
            f"{state.initial_attempts - (1 if state.first_route_s is not None else 0)}r/"
            f"{wait * 1000.0:.0f}ms"
            for state, wait in zip(states, initial_waits_s)
        ),
        successful_rebuilds=len(rebuild_attempt_ms),
        rebuild_ms_min=min(rebuild_attempt_ms, default=None),
        rebuild_ms_max=max(rebuild_attempt_ms, default=None),
        rebuild_ms_total=sum(rebuild_attempt_ms),
        event_checks=len(event_check_ms),
        event_check_ms_total=sum(event_check_ms),
        initial_plan_calls=len(initial_search_ms),
        initial_search_ms_total=sum(initial_search_ms),
        initial_map_ms_total=sum(initial_map_ms),
        rebuild_search_ms_total=sum(rebuild_search_ms),
        rebuild_latencies_ms=tuple(rebuild_attempt_ms),
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
    clearances_mm: Sequence[float] | None = None,
    arms: Sequence[tuple[str, float, float]] | None = None,
) -> list[HeadlessRunResult]:
    """Run scenario x planner x policy x trial in stable order.

    ``arms`` replaces the policy x horizon x period grid with an explicit list
    of ``(policy, prediction horizon ms, check interval ms)`` combinations, so
    a result set runs exactly the arms in ``RESULT_SETS`` and nothing else.

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
    clearances: tuple[float | None, ...] = (
        tuple(float(c) for c in clearances_mm) if clearances_mm else (config.planning_clearance_mm,)
    )
    if any(c is not None and (not isfinite(c) or c < 0) for c in clearances):
        raise ValueError("planning clearances must be non-negative")
    if backend == "grsim":
        if config.time_scale is not None:
            raise ValueError("time_scale is supported only by the kinematic backend")
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
    if arms is not None:
        arm_list = [(str(p), float(h), float(t)) for p, h, t in arms]
        bad_arms = [
            arm
            for arm in arm_list
            if arm[0] not in REPLAN_POLICIES or arm[1] < 0 or arm[2] <= 0
        ]
        if bad_arms or not arm_list:
            raise ValueError(f"Invalid arms: {bad_arms or 'none given'}")
    else:
        arm_list = [
            (policy, horizon, period)
            for policy in policies
            for horizon in predictions
            for period in periods
            # Plan-once never replans, so the replan period cannot change it; and
            # with a no-prediction arm present its prediction variants are skipped.
            if not (
                policy == "once"
                and (period != periods[0] or (horizon > 0 and 0.0 in predictions))
            )
        ]
    jobs = [
        (
            scenario,
            planner_name,
            replace(
                config,
                replan_policy=policy,
                prediction_horizon_ms=horizon,
                control_period_s=period / 1000.0,
                planning_clearance_mm=clearance,
            ),
            trial,
            seed + trial - 1,
            label,
        )
        for scenario, label in zip(scenarios, labels)
        for planner_name in planners
        for policy, horizon, period in arm_list
        for clearance in clearances
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


def _wilson(successes: int, n: int, z: float = 1.96) -> str:
    """95% Wilson score interval for a proportion, formatted 'low-high'."""
    if n == 0:
        return ""
    p = successes / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return f"{max(0.0, centre - half):.3f}-{min(1.0, centre + half):.3f}"


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
            result.planning_clearance_mm,
        )
        groups.setdefault(key, []).append(result)

    summaries = []
    for (scenario, planner, policy, backend, horizon, period, clearance), runs in groups.items():
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
                "planning_clearance_mm": clearance,
                "backend": backend,
                "physics_validation_passes": sum(run.physics_validation_passed for run in runs),
                "runs": len(runs),
                "scenario_count": len({run.scenario for run in runs}),
                "completed_runs": sum(run.completed for run in runs),
                "completion_rate": sum(run.completed for run in runs) / len(runs),
                "failed_plans": sum(run.failed_plans for run in runs),
                "collision_episodes": sum(run.collision_episodes for run in runs),
                "collision_free_rate": sum(run.collision_episodes == 0 for run in runs) / len(runs),
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
                "collision_probability": 1.0
                - sum(run.collision_episodes == 0 for run in runs) / len(runs),
                "collision_probability_ci95": _wilson(
                    sum(run.collision_episodes > 0 for run in runs), len(runs)
                ),
                "contact_time_ms_mean": fmean(run.contact_time_ms for run in runs),
                "rebuild_share_of_planning_time": (
                    sum(run.planning_time_ms_rebuild for run in runs)
                    / max(sum(run.planning_time_ms_total for run in runs), 1e-9)
                ),
                "check_call_ms_mean": (
                    sum(run.planning_time_ms_check for run in runs)
                    / max(sum(run.check_calls for run in runs), 1)
                ),
                "rebuild_call_ms_mean": (
                    sum(run.planning_time_ms_rebuild for run in runs)
                    / max(sum(run.rebuild_calls for run in runs), 1)
                ),
                "replans_active_blocked_mean": fmean(run.replans_active_blocked for run in runs),
                "replans_route_finished_mean": fmean(run.replans_route_finished for run in runs),
                "replans_route_blocked_mean": fmean(run.replans_route_blocked for run in runs),
                "direct_path_switches_mean": fmean(run.direct_path_switches for run in runs),
                "path_shift_mm_mean": fmean(run.path_shift_mm_mean for run in runs),
                "heading_change_rad_per_m_mean": fmean(
                    run.heading_change_rad_per_m for run in runs
                ),
                "sharp_turns_mean": fmean(run.sharp_turns for run in runs),
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


def _rate(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def _mean_or_none(values: Sequence[float]) -> float | None:
    return fmean(values) if values else None


def summarize_outcomes(
    results: Sequence[HeadlessRunResult],
) -> list[dict[str, str | bool | int | float | None]]:
    """One row per planner and arm in the revised result model (DEC-016/017/018).

    Denominators: ``valid_runs`` (the matched scenarios, 200 for the ACRA bank)
    for completion and incompletion; ``completed`` for strict, buffer-only and
    physical rates; failed completions ``F = B + P`` and physical runs ``P`` for
    contact severity. Per-run quantities are means over valid runs. Replan
    latency statistics pool every successful post-initial rebuild; failed
    attempts are counted separately.
    Invalid runs (initial overlap) are counted but excluded from everything else.
    """
    groups: dict[tuple, list[HeadlessRunResult]] = {}
    for result in results:
        key = (
            result.scenario_set or result.scenario,
            result.planner,
            result.replan_policy,
            round(result.replan_period_ms, 3),
            result.prediction_horizon_ms,
            result.planning_clearance_mm,
            result.backend,
        )
        groups.setdefault(key, []).append(result)

    rows = []
    for (scenario_set, planner, policy, period, horizon, clearance, backend), runs in groups.items():
        valid = [run for run in runs if run.outcome != "X"]
        count = {code: sum(run.outcome == code for run in valid) for code in OUTCOME_CODES}
        n = len(valid)
        strict, buffer_only, physical, incomplete = (count[c] for c in ("S", "B", "P", "I"))
        completed = strict + buffer_only + physical
        failed = buffer_only + physical
        contact_runs = [run for run in valid if run.outcome == "P"]
        rr = sum(run.rr_contact_episodes for run in contact_runs)
        ro = sum(run.ro_contact_episodes for run in contact_runs)
        ro_all = sum(run.ro_contact_episodes for run in valid)
        latencies = [value for run in valid for value in run.rebuild_latencies_ms]
        initial_calls = sum(run.initial_plan_calls for run in valid)
        route_count = sum(run.route_count for run in valid)
        goal_times = [run.time_to_goal_ms for run in valid if run.time_to_goal_ms is not None]
        rows.append(
            {
                "scenario_set": scenario_set,
                "planner": planner,
                "replan_policy": policy,
                "replan_period_ms": period,
                "prediction_horizon_ms": horizon,
                "planning_clearance_mm": clearance,
                "backend": backend,
                "scenarios": len({run.scenario for run in runs}),
                "runs": len(runs),
                "invalid_runs": len(runs) - n,
                "valid_runs": n,
                "completed": completed,
                "strict": strict,
                "buffer_only": buffer_only,
                "physical": physical,
                "incomplete": incomplete,
                "completed_label": f"{completed}/{n}",
                "strict_label": f"{strict}/{completed}",
                "buffer_only_label": f"{buffer_only}/{completed}",
                "physical_label": f"{physical}/{completed}",
                "incomplete_label": f"{incomplete}/{n}",
                "collision_free_completed": strict + buffer_only,
                "collision_free_completed_label": f"{strict + buffer_only}/{n}",
                # Comparison metric (DEC-021): completed with no physical contact,
                # out of all valid runs. Buffer entries count as collision-free.
                "collision_free_completion_rate": _rate(strict + buffer_only, n),
                "completion_rate": _rate(completed, n),
                "strict_rate": _rate(strict, completed),
                "buffer_only_rate": _rate(buffer_only, completed),
                "physical_rate": _rate(physical, completed),
                "safety_compromised_rate": _rate(failed, completed),
                "incomplete_rate": _rate(incomplete, n),
                "rr_contacts_per_failed_run": _rate(rr, failed),
                "ro_contacts_per_failed_run": _rate(ro, failed),
                "contacts_per_failed_run": _rate(rr + ro, failed),
                "contacts_per_physical_run": _rate(rr + ro, physical),
                "contacts_in_incomplete_runs": sum(
                    run.rr_contact_episodes + run.ro_contact_episodes
                    for run in valid
                    if run.outcome == "I"
                ),
                "ro_contacts_obstacle_initiated_share": _rate(
                    sum(run.ro_contact_obstacle_initiated for run in valid), ro_all
                ),
                "episodes_failed_no_route": sum(
                    run.episode_end_reason == "no_valid_path" for run in valid
                ),
                "robots_stopped_time_limit": sum(run.robots_stopped_time_limit for run in valid),
                "initial_retries_per_run": _mean_or_none(
                    [run.initial_retries_total for run in valid]
                ),
                "initial_wait_ms_per_run": _mean_or_none(
                    [run.initial_wait_ms_total for run in valid]
                ),
                "initial_wait_ms_max": max((run.initial_wait_ms_max for run in valid), default=None),
                "replans_per_run": _mean_or_none([run.replan_count for run in valid]),
                "failed_replans_per_run": _mean_or_none([run.replan_failures for run in valid]),
                "blocked_time_pct_mean": _mean_or_none(
                    [
                        100.0
                        * run.replan_failures
                        * run.replan_period_ms
                        / max(run.simulated_duration_ms * run.robot_count, 1e-9)
                        for run in valid
                    ]
                ),
                "routes_per_run": _mean_or_none([run.route_count for run in valid]),
                "route_lifetime_ms_mean": (
                    sum(run.route_lifetime_ms_total for run in valid) / route_count
                    if route_count
                    else None
                ),
                "route_lifetime_ms_max": max(
                    (run.route_lifetime_ms_max for run in valid), default=None
                ),
                "successful_rebuilds": len(latencies),
                "failed_replan_share": _rate(
                    sum(run.replan_failures for run in valid),
                    sum(run.replan_failures for run in valid) + len(latencies),
                ),
                "replan_ms_min": min(latencies, default=None),
                "replan_ms_mean": _mean_or_none(latencies),
                "replan_ms_p95": _percentile(latencies, 0.95) if latencies else None,
                "replan_ms_max": max(latencies, default=None),
                "event_check_ms_mean": _rate(
                    sum(run.event_check_ms_total for run in valid),
                    sum(run.event_checks for run in valid),
                ),
                "initial_plan_ms_mean": _rate(
                    sum(run.initial_search_ms_total + run.initial_map_ms_total for run in valid),
                    initial_calls,
                ),
                "initial_search_ms_mean": _rate(
                    sum(run.initial_search_ms_total for run in valid), initial_calls
                ),
                "initial_map_ms_mean": _rate(
                    sum(run.initial_map_ms_total for run in valid), initial_calls
                ),
                "time_to_goal_ms_mean": _mean_or_none(goal_times),
            }
        )
    return rows


def _polyline_turning(points: Sequence[Point]) -> tuple[float, int]:
    """Total absolute heading change (rad) and turns sharper than 90 degrees."""
    headings = [
        atan2(b[1] - a[1], b[0] - a[0]) for a, b in pairwise(points) if _distance(a, b) > 1e-6
    ]
    turns = [abs((second - first + pi) % (2 * pi) - pi) for first, second in pairwise(headings)]
    return sum(turns), sum(turn > pi / 2 for turn in turns)


def validate_one_shot(
    scenario: Scenario,
    planner_name: str,
    *,
    seed: int = 0,
    clearance_mm: float | None = None,
) -> list[dict[str, str | bool | int | float | None]]:
    """One-Shot Planning Validation: one initial route per robot, nothing executed.

    Obstacles are frozen at their initial positions, the other robots sit at
    their starts, the prediction horizon is 0 ms and nothing is replanned. The
    planning call is split into Dijkstra search and everything else (map or
    roadmap construction plus validation) with the same boundary for all
    planners.
    """
    scenario.require_complete()
    if planner_name not in PLANNER_NAMES:
        raise ValueError(f"Unknown planner {planner_name!r}; choose from {PLANNER_NAMES}")
    clearance = planning_clearance_mm(planner_name) if clearance_mm is None else clearance_mm
    config = SimulationConfig(replan_policy="once", planning_clearance_mm=clearance)
    planner = _new_planner(planner_name, seed, "once")
    rows: list[dict[str, str | bool | int | float | None]] = []
    for robot in scenario.robots:
        assert robot.target_mm is not None
        state = _RobotState(
            robot=robot, path=(robot.start_mm,), failed=False, position=robot.start_mm
        )
        scene = PlanningScene(
            timestamp=0.0, obstacles=_planning_obstacles(scenario, robot, planner_name)
        )
        record: dict = {}
        elapsed_ms, _, _ = _plan_call(planner, state, scene, 0.0, config, True, record)
        success = record.get("kind") == "initial"
        straight = _distance(robot.start_mm, robot.target_mm)
        length = _path_length(state.path) if success else None
        turning, sharp = _polyline_turning(state.path) if success else (None, None)
        rows.append(
            {
                "scenario": scenario.name,
                "planner": planner_name,
                "robot": f"{'Y' if robot.is_yellow else 'B'}{robot.robot_id}",
                "planning_clearance_mm": clearance,
                "route_available": success,
                "total_ms": elapsed_ms,
                "search_ms": record.get("search_ms"),
                "map_ms": elapsed_ms - record.get("search_ms", 0.0),
                "waypoints": len(state.path) if success else 0,
                "path_length_mm": length,
                "straight_line_mm": straight,
                "path_excess": (length / straight - 1.0) if success and straight > 0 else None,
                "turning_rad_per_m": (
                    turning / (length / 1000.0) if success and length and length > 0 else None
                ),
                "sharp_turns": sharp,
            }
        )
    return rows


def summarize_one_shot(
    rows: Sequence[dict],
) -> list[dict[str, str | bool | int | float | None]]:
    """Per-planner One-Shot Planning Validation table over all scenarios."""
    by_planner: dict[str, list[dict]] = {}
    for row in rows:
        by_planner.setdefault(str(row["planner"]), []).append(row)
    summaries = []
    for planner, planner_rows in by_planner.items():
        scenarios: dict[str, list[dict]] = {}
        for row in planner_rows:
            scenarios.setdefault(str(row["scenario"]), []).append(row)
        ok = [row for row in planner_rows if row["route_available"]]
        totals = [float(row["total_ms"]) for row in planner_rows]
        searches = [float(row["search_ms"]) for row in planner_rows]
        maps = [float(row["map_ms"]) for row in planner_rows]
        full = sum(all(r["route_available"] for r in group) for group in scenarios.values())
        # Requests with a clear straight line need no map (~0.03 ms) and dominate
        # the means; the planners differ only on requests that needed a map.
        mapped = [r for r in ok if int(r["waypoints"]) > 2]
        mapped_ms = [float(r["total_ms"]) for r in mapped]
        summaries.append(
            {
                "planner": planner,
                "scenarios": len(scenarios),
                "robot_calls": len(planner_rows),
                "scenarios_all_routes": full,
                "scenarios_all_routes_label": f"{full}/{len(scenarios)}",
                "robot_routes_available": len(ok),
                "robot_routes_label": f"{len(ok)}/{len(planner_rows)}",
                "map_ms_mean": _mean_or_none(maps),
                "map_ms_p95": _percentile(maps, 0.95) if maps else None,
                "search_ms_mean": _mean_or_none(searches),
                "search_ms_p95": _percentile(searches, 0.95) if searches else None,
                "total_ms_mean": _mean_or_none(totals),
                "total_ms_p95": _percentile(totals, 0.95) if totals else None,
                "total_ms_max": max(totals, default=None),
                "map_requiring_calls": len(mapped),
                "map_requiring_ms_median": median(mapped_ms) if mapped_ms else None,
                "map_requiring_ms_p95": _percentile(mapped_ms, 0.95) if mapped_ms else None,
                "map_requiring_path_excess_median": (
                    median([float(r["path_excess"]) for r in mapped]) if mapped else None
                ),
                "scenario_initial_plan_ms_mean": _mean_or_none(
                    [sum(float(r["total_ms"]) for r in group) for group in scenarios.values()]
                ),
                "path_excess_mean": _mean_or_none([float(r["path_excess"]) for r in ok]),
                "path_excess_median": median([float(r["path_excess"]) for r in ok]) if ok else None,
                "turning_rad_per_m_mean": _mean_or_none(
                    [float(r["turning_rad_per_m"]) for r in ok if r["turning_rad_per_m"] is not None]
                ),
                "sharp_turns_per_route": _mean_or_none([float(r["sharp_turns"]) for r in ok]),
            }
        )
    return summaries


def write_one_shot_results(
    rows: Sequence[dict],
    destination: str | Path,
    *,
    extra_manifest: dict | None = None,
) -> dict[str, Path]:
    """Write per-robot rows, the per-planner table and a manifest."""
    folder = Path(destination)
    folder.mkdir(parents=True, exist_ok=True)
    summaries = summarize_one_shot(rows)
    paths = {
        "robots_csv": CSVExporter().export(list(rows), folder / "robots.csv"),
        "summary_csv": CSVExporter().export(summaries, folder / "summary.csv"),
        "summary_json": JSONExporter().export(summaries, folder / "summary.json"),
    }
    manifest = {
        "generated_at": datetime.now(UTC).isoformat(),
        "engine": "research_sdk.headless",
        "result_set": ONE_SHOT_RESULT_SET,
        "model": "frozen obstacles at initial positions, other robots at their starts, "
        "prediction 0 ms, one initial route request per robot, no execution",
        "timing": "wall clock, single process; search_ms is time inside the shared "
        "networkx Dijkstra call, map_ms is the rest of the planning call",
        "robot_calls": len(rows),
        **_code_revision(),
        **(extra_manifest or {}),
    }
    manifest_path = folder / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    paths["manifest"] = manifest_path
    return paths


def _code_revision() -> dict:
    """Git revision of the code that wrote a batch, or None when unknown.

    Every batch under results/acra-* was produced by a different version of
    this module: runs.csv carries 40, 43, 44, 61 or 62 columns depending on the
    batch, and nothing in the manifest said which code wrote it. That is why
    analyse_canonical.py cannot run on acra-6v6-patrol, the batch it is named
    for: the batch predates the robot-initiated contact columns the analysis
    requires. Recording the revision makes that kind of drift visible at the
    point of comparison rather than at the point of failure.

    Never raises: provenance must not fail a batch. A worktree checked out on
    Windows cannot be read by git under WSL, so None is a normal outcome there.
    """
    import subprocess

    root = Path(__file__).resolve().parents[2]
    # Never take git's index lock: another tool may be using the repository.
    git_env = {**os.environ, "GIT_OPTIONAL_LOCKS": "0"}
    try:
        revision = (
            subprocess.run(
                ["git", "-C", str(root), "rev-parse", "HEAD"],
                capture_output=True,
                text=True,
                timeout=5,
                check=False,
                env=git_env,
            ).stdout.strip()
            or None
        )
        dirty = (
            subprocess.run(
                ["git", "-C", str(root), "status", "--porcelain", "--untracked-files=no"],
                capture_output=True,
                text=True,
                timeout=5,
                check=False,
                env=git_env,
            ).stdout.strip()
            != ""
            if revision
            else None
        )
    except Exception:  # noqa: BLE001 - provenance must never fail a batch
        revision, dirty = None, None
    return {"code_revision": revision, "code_dirty": dirty}


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
    summaries = summarize_outcomes(results)
    rebuild_calls = [
        {
            "scenario": result.scenario,
            "planner": result.planner,
            "replan_policy": result.replan_policy,
            "replan_period_ms": round(result.replan_period_ms, 3),
            "prediction_horizon_ms": result.prediction_horizon_ms,
            "attempt": index,
            "rebuild_ms": value,
        }
        for result in results
        for index, value in enumerate(result.rebuild_latencies_ms, start=1)
    ]
    paths = {
        "runs_csv": CSVExporter().export(raw_records, folder / "runs.csv"),
        "runs_json": JSONExporter().export(raw_records, folder / "runs.json"),
        "summary_csv": CSVExporter().export(summaries, folder / "summary.csv"),
        "summary_json": JSONExporter().export(summaries, folder / "summary.json"),
        "summary_legacy_csv": CSVExporter().export(
            summarize_results(results), folder / "summary_legacy.csv"
        ),
        "rebuild_calls_csv": CSVExporter().export(rebuild_calls, folder / "rebuild_calls.csv"),
    }
    manifest = {
        "generated_at": datetime.now(UTC).isoformat(),
        "engine": "research_sdk.headless",
        # Keep the manifest helper name aligned with the implementation above.
        "provenance": _code_revision(),
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
        "planning_clearances_mm": sorted({r.planning_clearance_mm for r in results}),
        "patrol_model": "constant speed back-and-forth along spawn + patrol_waypoints "
        f"(default {DEFAULT_PATROL_SPEED_MMPS:g} mm/s)",
        "prediction_model": "constant velocity; circle moved to predicted position and "
        "radius grown by predicted travel (WorldMap / Obstacle.dynamic_radius_0)",
        "scenario_sets": sorted({result.scenario_set or result.scenario for result in results}),
        "latency_model": "planning latency measured on wall clock; not applied to virtual time",
        "result_model": "DEC-016/017/018: outcome S/B/P/I (X = invalid initial overlap); "
        f"buffer < {SAFETY_BUFFER_MM:g} mm, contact <= 0 mm, swept between states; "
        "robot-robot and robot-obstacle only; summary.csv averages over valid runs",
        "failure_rules": {
            "replan_time_limit_ms": config.slow_call_limit_ms,
            "replan_time_limit_effect": "post-initial replan slower than this stops that robot",
            "no_route_limit_ms": config.no_route_limit_ms,
            "no_route_limit_effect": "any robot without a valid route this long fails the episode",
            "initial_plan": "exempt from the time limit; reported as one-shot timing",
        },
        **_code_revision(),
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


def load_scenario_bank(folder: str | Path, expected: int | None = None) -> list[Scenario]:
    """Load a frozen scenario bank: every ``*.json`` directly in ``folder``, nothing else.

    Raises when the folder is missing, a name repeats, or the count differs
    from ``expected``.
    """
    bank = Path(folder)
    if not bank.is_dir():
        raise FileNotFoundError(f"Scenario bank is not a directory: {bank}")
    paths = sorted(bank.glob("*.json"))
    scenarios = _load_scenarios(paths)
    names = [scenario.name for scenario in scenarios]
    duplicates = sorted({name for name in names if names.count(name) > 1})
    if duplicates:
        raise ValueError(f"Scenario bank {bank} repeats scenario names: {duplicates[:5]}")
    if expected is not None and len(scenarios) != expected:
        raise ValueError(f"Scenario bank {bank} holds {len(scenarios)} scenarios, expected {expected}")
    return scenarios


def scenario_bank_fingerprint(folder: str | Path) -> dict:
    """Path, count and SHA-256 over the bank's JSON files (line endings normalised)."""
    bank = Path(folder)
    digest = hashlib.sha256()
    paths = sorted(bank.glob("*.json"))
    for path in paths:
        digest.update(path.name.encode("utf-8"))
        digest.update(path.read_bytes().replace(b"\r\n", b"\n"))
    return {"scenario_bank": bank.as_posix(), "scenario_bank_files": len(paths),
            "scenario_bank_sha256": digest.hexdigest()}


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
        "--scenario-bank",
        type=Path,
        default=None,
        help="Explicit scenario directory for a reproducible matched bank; cannot be combined with positional scenarios",
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
    parser.add_argument(
        "--grsim-bin",
        type=Path,
        default=Path(os.environ.get("RESEARCH_GRSIM_BIN", ".local/grsim/bin/grSim")),
        help="grSim executable (default: $RESEARCH_GRSIM_BIN, else the Linux build in .local/)",
    )
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
        "--time-scale",
        type=int,
        choices=TIME_SCALE_CHOICES,
        default=None,
        help="Cap kinematic runs at 1x, 10x, 100x, 200x, or 500x (default: unpaced)",
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
        "--clearance-mm",
        type=float,
        nargs="+",
        default=None,
        help="Planning clearance(s) in mm; each value is a separate arm "
        "(default: config/planner_variables.yaml)",
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
    parser.add_argument("--random", type=int, default=0, help="Add N seeded random scenarios")
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
        "--result-set",
        choices=(ONE_SHOT_RESULT_SET, *RESULT_SETS),
        default=None,
        help=(
            "Run one ACRA result set exactly as defined (arms fixed, scenario bank "
            f"defaults to {DEFAULT_SCENARIO_BANK.as_posix()} and must hold "
            f"{ACRA_SCENARIO_COUNT} scenarios)"
        ),
    )
    parser.add_argument(
        "--expect-scenarios",
        type=int,
        default=None,
        help="Fail unless the scenario bank holds exactly this many scenarios",
    )
    parser.add_argument(
        "--no-route-limit-ms",
        type=float,
        default=1000.0,
        help="Fail the episode when any robot goes this long without a route (default 1000; 0 disables)",
    )
    parser.add_argument(
        "--slow-call-limit-ms",
        type=float,
        default=100.0,
        help="Stop a robot whose post-initial replan exceeds this wall time (0 disables)",
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
    if args.scenario_bank is not None and args.scenarios:
        raise ValueError("--scenario-bank cannot be combined with positional scenarios")
    if args.scenario_bank is not None:
        # A bank run uses exactly the bank: no generated, jittered or extra files.
        if args.random or args.perturb or args.no_saved:
            raise ValueError(
                "--scenario-bank runs exactly the bank; --random, --perturb and "
                "--no-saved are not allowed with it"
            )
        scenarios = load_scenario_bank(args.scenario_bank, expected=args.expect_scenarios)
        return scenarios, [Path(args.scenario_bank).name] * len(scenarios), generator
    scenario_inputs = [str(args.scenario_bank)] if args.scenario_bank is not None else args.scenarios
    only_random = args.random > 0 and not scenario_inputs
    saved = [] if only_random else _load_scenarios(_scenario_paths(scenario_inputs))
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
        if args.result_set is not None:
            if args.scenarios:
                raise ValueError("--result-set uses a scenario bank, not positional scenarios")
            if (
                args.policy != ["once"]
                or args.predict_ms != [0.0]
                or args.replan_ms is not None
                or args.random
                or args.perturb
            ):
                raise ValueError(
                    "--result-set fixes the arms and scenarios; drop --policy, --predict-ms, "
                    "--replan-ms, --random and --perturb"
                )
            if args.scenario_bank is None:
                args.scenario_bank = DEFAULT_SCENARIO_BANK
            if args.expect_scenarios is None and args.scenario_bank == DEFAULT_SCENARIO_BANK:
                args.expect_scenarios = ACRA_SCENARIO_COUNT
        scenarios, labels, generator = _build_cases(args)
        bank_manifest = (
            scenario_bank_fingerprint(args.scenario_bank) if args.scenario_bank is not None else {}
        )
        planners = PLANNER_NAMES if "all" in args.planner else tuple(dict.fromkeys(args.planner))
        policies = REPLAN_POLICIES if "all" in args.policy else tuple(dict.fromkeys(args.policy))
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
            planning_clearance_mm=None if args.clearance_mm is None else args.clearance_mm[0],
            time_scale=args.time_scale,
            no_route_limit_ms=args.no_route_limit_ms or None,
            slow_call_limit_ms=args.slow_call_limit_ms or None,
        )
        if args.result_set in RESULT_SET_OVERRIDES:
            config = replace(config, **RESULT_SET_OVERRIDES[args.result_set])
        output_folder = args.output_dir or _default_output_folder()
        if output_folder.exists() and any(output_folder.iterdir()):
            raise ValueError("Output directory must be new or empty to preserve previous results")
        if args.result_set == ONE_SHOT_RESULT_SET:
            print(
                f"One-shot validation: {len(scenarios)} scenarios x {len(planners)} planners, "
                "single process",
                flush=True,
            )
            rows = [
                row
                for scenario in scenarios
                for planner_name in planners
                for row in validate_one_shot(
                    scenario,
                    planner_name,
                    seed=args.seed,
                    clearance_mm=None if args.clearance_mm is None else args.clearance_mm[0],
                )
            ]
            write_one_shot_results(rows, output_folder, extra_manifest=bank_manifest)
            for summary in summarize_one_shot(rows):
                print(
                    "{planner}: routes {robot_routes_label} robots, "
                    "{scenarios_all_routes_label} scenarios; "
                    "total {total_ms_mean:.2f} ms mean / {total_ms_p95:.2f} ms p95".format(
                        **summary
                    )
                )
            return 0
        output_folder.mkdir(parents=True, exist_ok=True)
        if args.scenario_bank is None:
            # Generated or loose scenarios: keep a copy so the batch is reproducible.
            # A frozen bank is not copied; the manifest records its path, file
            # count and SHA-256 fingerprint instead.
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
            len(scenarios) * len(planners) * len(RESULT_SETS[args.result_set]) * args.trials
            if args.result_set
            else len(scenarios)
            * len(planners)
            * len(policies)
            * len(args.predict_ms)
            * len(periods)
            * len(args.clearance_mm or [1])
            * args.trials
        )
        print(
            f"Running up to {total} runs: {len(scenarios)} scenarios x {len(planners)} planners"
            + (
                f" x {len(RESULT_SETS[args.result_set])} arms ({args.result_set})"
                if args.result_set
                else f" x {len(policies)} policies"
            )
            + f" x {args.trials} trials on {workers} worker(s)"
            f" at {f'{args.time_scale}x cap' if args.time_scale else 'maximum speed'}",
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
            clearances_mm=args.clearance_mm,
            arms=RESULT_SETS.get(args.result_set) if args.result_set else None,
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
                "result_set": args.result_set,
                "arms": [list(arm) for arm in RESULT_SETS.get(args.result_set, ())]
                if args.result_set
                else None,
                "result_set_overrides": RESULT_SET_OVERRIDES.get(args.result_set),
                **bank_manifest,
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
    for summary in summarize_outcomes(results):
        print(
            "{scenario_set} / {planner} / {replan_policy} @ {replan_period_ms:g} ms, "
            "h {prediction_horizon_ms:g} ms: completed {completed_label}, strict {strict_label}, "
            "buffer-only {buffer_only_label}, physical {physical_label}".format(**summary)
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
