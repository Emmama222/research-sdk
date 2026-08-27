#!/usr/bin/env python
"""Compare serial / thread-pool / process-pool execution for planning several
robots' paths in one go.

Uses ``VoronoiDijkstraPlanner`` directly against real scenario obstacle
layouts -- a *fresh* planner instance per call, bypassing
``VoronoiWaypointManager``'s reused-previous-path cache, which would
otherwise make every call after the first nearly free and hide the real
per-call cost this comparison needs to see. This is CPU-bound pure-Python
work (density-grid generation + Dijkstra), so it's the case where a thread
pool is expected to lose to (or barely help over) serial -- GIL contention
-- while a process pool should show a real win once obstacle counts/robot
counts are large enough to amortise process spawn and result-pickling
overhead. See ``docs/decisions/0005-parallel-planner-execution.md``.

Usage:
    python scripts/benchmark_parallel_planning.py
    python scripts/benchmark_parallel_planning.py --robots 6 --trials 5
"""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from itertools import cycle, islice
from statistics import median
from time import perf_counter

from research_sdk.config import ROBOT_RADIUS_MM
from research_sdk.planners.Dijkstra.voronoi_dijkstra import VoronoiDijkstraPlanner
from research_sdk.ui.runtime import planner_key
from research_sdk.ui.scenarios import ScenarioStore
from research_sdk.world.scene import FieldDimensions, PlanningObstacle, PlanningScene

Job = tuple[tuple[float, float], tuple[float, float], tuple[PlanningObstacle, ...]]


def _solve_one(job: Job):
    # Imported and instantiated inside the worker -- keeps the planner object
    # itself (and its import) out of the pickled call for the process-pool
    # path, and matches what a real per-robot call site would do anyway.
    from research_sdk.planners.Dijkstra.voronoi_dijkstra import VoronoiDijkstraPlanner

    start, target, obstacles = job
    scene = PlanningScene(timestamp=0.0, obstacles=obstacles, field=FieldDimensions())
    return VoronoiDijkstraPlanner().plan(scene, start, target)


def _jobs_from_scenarios(robot_count: int) -> list[Job]:
    store = ScenarioStore(folder="scenarios")
    all_robots = []
    for path in store.list_paths():
        scenario = store.load(path)
        for robot in scenario.robots:
            if robot.target_mm is None:
                continue
            obstacles = tuple(
                PlanningObstacle(
                    robot_id=obstacle.obstacle_id,
                    isYellow=obstacle.is_yellow,
                    pos_mm=obstacle.position_mm,
                    radius_mm=obstacle.radius_mm,
                    vel_mmps=obstacle.velocity_mmps,
                )
                for obstacle in scenario.obstacles_for(planner_key(VoronoiDijkstraPlanner))
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
            all_robots.append((robot.start_mm, robot.target_mm, obstacles))
    if not all_robots:
        raise SystemExit("No planned robots found in scenarios/ -- nothing to benchmark.")
    # Cycle to reach the requested batch size (a real SSL tick plans up to 6
    # robots per team even if the saved scenarios have fewer).
    return list(islice(cycle(all_robots), robot_count))


def _time_ms(fn) -> float:
    started = perf_counter()
    fn()
    return (perf_counter() - started) * 1000.0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, default=6, help="Robots to plan per trial")
    parser.add_argument("--trials", type=int, default=8)
    args = parser.parse_args()

    jobs = _jobs_from_scenarios(args.robots)
    print(f"Planning {len(jobs)} robots per trial, {args.trials} trials per mode.\n")

    def _run_serial() -> None:
        for job in jobs:
            _solve_one(job)

    def _run_threaded() -> None:
        with ThreadPoolExecutor(max_workers=len(jobs)) as executor:
            list(executor.map(_solve_one, jobs))

    def _run_processes() -> None:
        with ProcessPoolExecutor(max_workers=len(jobs)) as executor:
            list(executor.map(_solve_one, jobs))

    modes = {
        "serial": _run_serial,
        "thread_pool": _run_threaded,
        "process_pool": _run_processes,
    }

    for label, fn in modes.items():
        samples_ms = [_time_ms(fn) for _ in range(args.trials)]
        samples_ms.sort()
        print(
            f"{label:14s} median={median(samples_ms):8.1f}ms  "
            f"min={samples_ms[0]:8.1f}ms  max={samples_ms[-1]:8.1f}ms"
        )


if __name__ == "__main__":
    main()
