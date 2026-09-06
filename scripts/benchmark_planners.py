#!/usr/bin/env python
"""Compare VisibilityGraph, PRM, and Voronoi+Dijkstra planning time head-to-head.

Every saved scenario under ``scenarios/`` is turned into the same
``PlanningScene`` (start/goal/obstacles) and fed to all three standalone
planners directly -- bypassing the Qt UI and ``VoronoiWaypointManager``'s
reroute/dead-zone/previous-path-reuse layer, so the raw per-call algorithm
work is what gets timed, not UI overhead or state-reuse shortcuts. See
``docs/decisions/0005-parallel-planner-execution.md`` for why this script
exists: it's the evidence a "should we parallelize planning?" decision
needs, instead of guessing.

Usage:
    python scripts/benchmark_planners.py
    python scripts/benchmark_planners.py --trials 50
"""

from __future__ import annotations

import argparse
from statistics import median
from time import perf_counter

from research_sdk.config import ROBOT_RADIUS_MM
from research_sdk.planners.common import Obstacle, PlanRequest
from research_sdk.planners.Dijkstra.voronoi_dijkstra import VoronoiDijkstraPlanner
from research_sdk.planners.PRM import prm_dijkstra
from research_sdk.planners.VisibilityGraph import visibility_graph
from research_sdk.ui.runtime import planner_key
from research_sdk.ui.scenarios import Scenario, ScenarioRobot, ScenarioStore
from research_sdk.world.scene import FieldDimensions, PlanningObstacle, PlanningScene


def _percentile(values: list[float], pct: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(len(ordered) * pct))
    return ordered[index]


def _obstacles_for(scenario: Scenario, robot: ScenarioRobot, key: str | None) -> tuple[PlanningObstacle, ...]:
    """Same construction ``ResearchRuntime.plan()`` uses (``ui/runtime.py:203-223``):
    the planner's own scenario obstacles plus every other robot as a circle."""
    return tuple(
        PlanningObstacle(
            robot_id=obstacle.obstacle_id,
            isYellow=obstacle.is_yellow,
            pos_mm=obstacle.position_mm,
            radius_mm=obstacle.radius_mm,
            vel_mmps=obstacle.velocity_mmps,
        )
        for obstacle in scenario.obstacles_for(key)
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


def _plan_request(robot: ScenarioRobot, obstacles: tuple[PlanningObstacle, ...]) -> PlanRequest:
    return PlanRequest(
        start_mm=robot.start_mm,
        goal_mm=robot.target_mm,
        obstacles=tuple(
            Obstacle(pos_mm=o.pos_mm, radius_mm=o.radius_mm, robot_id=o.robot_id, isYellow=o.isYellow)
            for o in obstacles
        ),
        robot_radius_mm=ROBOT_RADIUS_MM,
    )


def _voronoi_succeeded(result) -> bool:
    return result.used_direct_path or bool(result.waypoints_mm)


class Stat:
    __slots__ = ("durations_ms", "failures", "used_full_map", "used_shortcut")

    def __init__(self) -> None:
        self.durations_ms: list[float] = []
        self.failures = 0
        self.used_shortcut = 0  # direct line of sight / reused previous path
        self.used_full_map = 0  # actually built a fresh graph/roadmap/voronoi map

    def add(self, elapsed_ms: float, *, ok: bool, shortcut: bool) -> None:
        self.durations_ms.append(elapsed_ms)
        if not ok:
            self.failures += 1
        if shortcut:
            self.used_shortcut += 1
        else:
            self.used_full_map += 1

    def row(self, label: str) -> str:
        n = len(self.durations_ms)
        med = median(self.durations_ms) if n else float("nan")
        p95 = _percentile(self.durations_ms, 0.95) if n else float("nan")
        return (
            f"{label:<28} n={n:<4} median={med:8.3f}ms  p95={p95:8.3f}ms  "
            f"failures={self.failures:<3} full_map_builds={self.used_full_map}/{n}"
        )


def benchmark(scenario: Scenario, trials: int) -> dict[str, Stat]:
    stats = {"VisibilityGraph": Stat(), "PRM": Stat(), "VoronoiDijkstra": Stat()}
    voronoi = VoronoiDijkstraPlanner()

    for robot in scenario.robots:
        if robot.target_mm is None:
            continue

        vg_key = planner_key(visibility_graph.VisibilityGraphPlanner)
        prm_key = planner_key(prm_dijkstra.PRMPlanner)
        vg_obstacles = _obstacles_for(scenario, robot, vg_key)
        prm_obstacles = _obstacles_for(scenario, robot, prm_key)
        voronoi_obstacles = _obstacles_for(scenario, robot, planner_key(VoronoiDijkstraPlanner))

        vg_request = _plan_request(robot, vg_obstacles)
        prm_request_base = _plan_request(robot, prm_obstacles)
        voronoi_scene = PlanningScene(
            timestamp=0.0, obstacles=voronoi_obstacles, field=FieldDimensions()
        )

        for trial in range(trials):
            started = perf_counter()
            result = visibility_graph.plan(vg_request)
            elapsed = (perf_counter() - started) * 1000.0
            stats["VisibilityGraph"].add(elapsed, ok=result.success, shortcut=result.nodes_expanded <= 2)

            started = perf_counter()
            result = prm_dijkstra.plan(prm_request_base, seed=trial)
            elapsed = (perf_counter() - started) * 1000.0
            stats["PRM"].add(elapsed, ok=result.success, shortcut=result.nodes_expanded <= 2)

            started = perf_counter()
            result = voronoi.plan(voronoi_scene, robot.start_mm, robot.target_mm)
            elapsed = (perf_counter() - started) * 1000.0
            stats["VoronoiDijkstra"].add(
                elapsed,
                ok=_voronoi_succeeded(result),
                shortcut=result.used_direct_path or result.reused_previous,
            )

    return stats


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=20, help="Repeats per robot per planner")
    args = parser.parse_args()

    store = ScenarioStore(folder="scenarios")
    paths = store.list_paths()
    if not paths:
        print("No scenario files found under scenarios/.")
        return

    for path in paths:
        scenario = store.load(path)
        planned_robots = [r for r in scenario.robots if r.target_mm is not None]
        if not planned_robots:
            continue
        print(f"\n=== {scenario.name} ({len(planned_robots)} robot(s), {args.trials} trials each) ===")
        stats = benchmark(scenario, args.trials)
        for label, stat in stats.items():
            if stat.durations_ms:
                print("  " + stat.row(label))

    print(
        "\nNote: 'full_map_builds' counts calls that did NOT take a direct-line-of-sight\n"
        "or reused-previous-path shortcut -- i.e. actually built a fresh graph/roadmap/\n"
        "Voronoi map. A planner that looks slow but rarely hits full_map_builds is being\n"
        "measured mostly on its shortcut path, not its worst case, and vice versa."
    )


if __name__ == "__main__":
    main()
