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
from research_sdk.ui.scenarios import Scenario, ScenarioObstacle, ScenarioRobot
from research_sdk.world.scene import PlanningObstacle, PlanningScene

Point = tuple[float, float]
RobotKey = tuple[bool, int]
PLANNER_NAMES = ("voronoi", "prm", "visibility")
REPLAN_POLICIES = ("once", "cycle", "event")


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


def _plan_robot(
    scenario: Scenario,
    robot: ScenarioRobot,
    planner_name: str,
    planner,
) -> tuple[_RobotState, float]:
    assert robot.target_mm is not None
    scene = PlanningScene(
        timestamp=0.0,
        obstacles=_planning_obstacles(scenario, robot, planner_name),
    )
    started = perf_counter()
    try:
        output = planner.plan(
            PlannerInput(
                robot_id=robot.robot_id,
                is_yellow=robot.is_yellow,
                current_pose=(*robot.start_mm, robot.orientation_rad),
                target_pose=(*robot.target_mm, robot.orientation_rad),
                scene=scene,
            )
        )
        success = (
            output.is_path_free
            or bool(output.waypoints)
            or _distance(robot.start_mm, robot.target_mm) <= 1e-9
        )
        path = _dedupe_path(
            (
                robot.start_mm,
                *((point[0], point[1]) for point in output.waypoints),
                robot.target_mm,
            )
        )
    except Exception:  # noqa: BLE001 - planner failures are experiment result data
        success = False
        path = (robot.start_mm,)
    elapsed_ms = (perf_counter() - started) * 1000.0
    if not success:
        path = (robot.start_mm,)
    return (
        _RobotState(
            robot=robot,
            path=path,
            failed=not success,
            position=robot.start_mm,
        ),
        elapsed_ms,
    )


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


def _obstacle_position(obstacle: ScenarioObstacle, simulation_s: float) -> Point:
    return (
        obstacle.position_mm[0] + obstacle.velocity_mmps[0] * simulation_s,
        obstacle.position_mm[1] + obstacle.velocity_mmps[1] * simulation_s,
    )


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
) -> HeadlessRunResult:
    """Plan and execute one scenario on a virtual clock without sleeping."""
    scenario.require_complete()
    if planner_name not in PLANNER_NAMES:
        raise ValueError(f"Unknown planner {planner_name!r}; choose from {PLANNER_NAMES}")
    if trial < 1:
        raise ValueError("trial must be at least 1")
    config = config or SimulationConfig()
    wall_started = perf_counter()
    planner = _new_planner(planner_name, seed)

    states: list[_RobotState] = []
    plan_durations_ms: list[float] = []
    for robot in scenario.robots:
        state, duration_ms = _plan_robot(scenario, robot, planner_name, planner)
        states.append(state)
        plan_durations_ms.append(duration_ms)

    obstacles = tuple(scenario.obstacles_for(_planner_key(planner_name)))
    active_robot_collisions: set[tuple[RobotKey, RobotKey]] = set()
    active_obstacle_collisions: set[tuple[RobotKey, RobotKey]] = set()
    robot_collision_episodes = 0
    obstacle_collision_episodes = 0
    minimum_clearance_mm: float | None = None
    simulation_s = 0.0
    ticks = 0

    while True:
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
        next_positions = [_next_position(state, step_s, config) for state in states]
        for state, position in zip(states, next_positions):
            state.travelled_mm += _distance(state.position, position)
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
        planner_calls=len(plan_durations_ms),
        failed_plans=len(failed_states),
        failed_robots=failed_robots,
        planning_time_ms_total=sum(plan_durations_ms),
        planning_time_ms_mean=fmean(plan_durations_ms),
        simulated_duration_ms=simulated_duration_ms,
        wall_time_ms=wall_time_ms,
        realtime_factor=realtime_factor,
        ticks=ticks,
        robot_collision_episodes=robot_collision_episodes,
        obstacle_collision_episodes=obstacle_collision_episodes,
        collision_episodes=robot_collision_episodes + obstacle_collision_episodes,
        planned_path_length_mm=sum(_path_length(state.path) for state in states),
        travelled_distance_mm=sum(state.travelled_mm for state in states),
        mean_final_error_mm=fmean(final_errors),
        max_final_error_mm=max(final_errors),
        minimum_clearance_mm=minimum_clearance_mm,
        dt_ms=config.dt_s * 1000.0,
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
) -> list[HeadlessRunResult]:
    """Run the scenario/planner/trial Cartesian product in stable order."""
    if trials < 1:
        raise ValueError("trials must be at least 1")
    unknown = [name for name in planners if name not in PLANNER_NAMES]
    if unknown:
        raise ValueError(f"Unknown planner names: {unknown}")
    config = config or SimulationConfig(dt_s=1 / 120 if backend == "grsim" else 0.02)
    if backend == "grsim":
        from research_sdk.grsim_physics import simulate_physics

        return [
            simulate_physics(
                scenario,
                planner_name,
                config=config,
                trial=trial,
                seed=seed + trial - 1,
                binary=Path(grsim_binary),
                evidence_dir=Path(evidence_dir),
            )
            for scenario in scenarios
            for planner_name in planners
            for trial in range(1, trials + 1)
        ]
    if backend != "kinematic":
        raise ValueError(f"Unknown simulation backend: {backend}")
    return [
        simulate(
            scenario,
            planner_name,
            config=config,
            trial=trial,
            seed=seed + trial - 1,
        )
        for scenario in scenarios
        for planner_name in planners
        for trial in range(1, trials + 1)
    ]


def _percentile(values: Sequence[float], percentile: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(len(ordered) * percentile))
    return ordered[index]


def summarize_results(
    results: Sequence[HeadlessRunResult],
) -> list[dict[str, str | bool | int | float | None]]:
    """Aggregate raw trials by scenario and planner."""
    groups: dict[tuple[str, str, str], list[HeadlessRunResult]] = {}
    for result in results:
        groups.setdefault((result.scenario, result.planner, result.backend), []).append(result)

    summaries = []
    for (scenario, planner, backend), runs in groups.items():
        planning = [run.planning_time_ms_total for run in runs]
        wall = [run.wall_time_ms for run in runs]
        simulated = [run.simulated_duration_ms for run in runs]
        speedups = [run.realtime_factor for run in runs]
        clearances = [
            run.minimum_clearance_mm for run in runs if run.minimum_clearance_mm is not None
        ]
        summaries.append(
            {
                "scenario": scenario,
                "planner": planner,
                "backend": backend,
                "physics_validation_passes": sum(run.physics_validation_passed for run in runs),
                "runs": len(runs),
                "completed_runs": sum(run.completed for run in runs),
                "completion_rate": sum(run.completed for run in runs) / len(runs),
                "failed_plans": sum(run.failed_plans for run in runs),
                "collision_episodes": sum(run.collision_episodes for run in runs),
                "planning_time_ms_median": median(planning),
                "planning_time_ms_p95": _percentile(planning, 0.95),
                "simulated_duration_ms_mean": fmean(simulated),
                "wall_time_ms_mean": fmean(wall),
                "realtime_factor_mean": fmean(speedups),
                "planned_path_length_mm_mean": fmean(run.planned_path_length_mm for run in runs),
                "travelled_distance_mm_mean": fmean(run.travelled_distance_mm for run in runs),
                "mean_final_error_mm": fmean(run.mean_final_error_mm for run in runs),
                "minimum_clearance_mm": min(clearances) if clearances else None,
            }
        )
    return summaries


def write_results(
    results: Sequence[HeadlessRunResult],
    destination: str | Path,
    *,
    config: SimulationConfig,
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
        "--fail-on-incomplete",
        action="store_true",
        help="Exit with status 2 if any run fails or times out",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        scenario_paths = _scenario_paths(args.scenarios)
        if not scenario_paths:
            raise ValueError("No scenario JSON files found")
        scenarios = _load_scenarios(scenario_paths)
        planners = PLANNER_NAMES if "all" in args.planner else tuple(dict.fromkeys(args.planner))
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
        )
        output_folder = args.output_dir or _default_output_folder()
        if output_folder.exists() and any(output_folder.iterdir()):
            raise ValueError("Output directory must be new or empty to preserve previous results")
        results = run_experiments(
            scenarios,
            planners,
            trials=args.trials,
            seed=args.seed,
            config=config,
            backend=args.backend,
            grsim_binary=args.grsim_bin,
            evidence_dir=output_folder / "evidence",
        )
        write_results(results, output_folder, config=config)
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
            "{} / {}: {:.0%} complete, {:.3f} ms median planning, "
            "{:.1f}x real time, {} collisions".format(
                summary["scenario"],
                summary["planner"],
                summary["completion_rate"],
                summary["planning_time_ms_median"],
                summary["realtime_factor_mean"],
                summary["collision_episodes"],
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
