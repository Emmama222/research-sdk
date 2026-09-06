#!/usr/bin/env python3
"""Runnable offline point-to-point comparison for all three planners: PRM+Dijkstra,
Visibility Graph+Dijkstra, and Voronoi+Dijkstra.

Builds a fixed SSL-like scenario (one robot navigating across a crowd of
opponents on a 9000x6000mm field), runs all three planners against the same
start/goal/obstacles, prints a comparison table (success, path length,
planning time, waypoint count), and -- if matplotlib is available -- saves a
side-by-side plot to ``demo_output.png`` so you can eyeball all three paths
without wiring up grSim, Phoenix, or the Qt UI. This is the offline,
single-query complement to ``scripts/benchmark_planners.py`` (which runs
against saved scenario files across many trials for aggregate statistics).

Usage:
    python scripts/demo_planners.py
    python scripts/demo_planners.py --trials 20   # also run a randomized
                                                    # multi-obstacle stress
                                                    # test and report timing
"""

from __future__ import annotations

import argparse
import random
import sys
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from research_sdk.planners.common import (
    FIELD_LENGTH_MM,
    FIELD_WIDTH_MM,
    Obstacle,
    PlanRequest,
    path_length_mm,
)
from research_sdk.planners.Dijkstra.voronoi_dijkstra import VoronoiDijkstraPlanner
from research_sdk.planners.PRM import prm_dijkstra
from research_sdk.planners.VisibilityGraph import visibility_graph
from research_sdk.world.scene import PlanningObstacle, PlanningScene

PLANNER_NAMES = ("PRM+Dijkstra", "VisibilityGraph+Dijkstra", "Voronoi+Dijkstra")


@dataclass(frozen=True, slots=True)
class ComparablePlan:
    """One planner's result, normalised to the same shape for comparison.

    ``waypoints_mm`` always includes ``start_mm`` as its first point (PRM and
    VisibilityGraph already do this; VoronoiDijkstraPlanner's own
    ``PlanResult.waypoints_mm`` does not -- see ``_run_voronoi`` below), so
    every consumer here (the table, the plot) can treat all three planners
    identically instead of special-casing Voronoi's different contract.
    """

    name: str
    success: bool
    path_length_mm: float
    planning_time_ms: float
    waypoints_mm: tuple[tuple[float, float], ...]
    message: str


def _run_prm(request: PlanRequest, *, seed: int, num_samples: int) -> ComparablePlan:
    # skip_direct_path=True: this comparison wants every call measuring
    # PRM's actual sampling cost, not the trivial straight-line case --
    # same reasoning applied uniformly to all three planners here.
    result = prm_dijkstra.plan(request, seed=seed, num_samples=num_samples, skip_direct_path=True)
    return ComparablePlan(
        "PRM+Dijkstra",
        result.success,
        result.path_length_mm,
        result.planning_time_ms,
        result.waypoints_mm,
        result.message,
    )


def _run_visibility(request: PlanRequest) -> ComparablePlan:
    # skip_direct_path=True -- see _run_prm's comment for why.
    result = visibility_graph.plan(request, skip_direct_path=True)
    return ComparablePlan(
        "VisibilityGraph+Dijkstra",
        result.success,
        result.path_length_mm,
        result.planning_time_ms,
        result.waypoints_mm,
        result.message,
    )


def _run_voronoi(request: PlanRequest) -> ComparablePlan:
    """Adapt VoronoiDijkstraPlanner to the same PlanRequest-in,
    ComparablePlan-out shape the other two planners use.

    Two contract differences from PRM/VisibilityGraph handled here, not by
    the caller: (1) it takes a ``PlanningScene`` + separate start/target
    points, not a ``PlanRequest``; (2) its own ``PlanResult.waypoints_mm``
    excludes ``start_mm`` (and is empty outright for a direct-line-of-sight
    result) rather than including it, so path length and "success" have to
    be derived here rather than read straight off the result.
    """
    obstacles = tuple(
        PlanningObstacle(
            robot_id=obs.robot_id, isYellow=obs.isYellow, pos_mm=obs.pos_mm, radius_mm=obs.radius_mm
        )
        for obs in request.obstacles
    )
    scene = PlanningScene(timestamp=0.0, obstacles=obstacles)
    planner = VoronoiDijkstraPlanner()

    started = perf_counter()
    result = planner.plan(scene, request.start_mm, request.goal_mm, skip_direct_path=True)
    planning_time_ms = (perf_counter() - started) * 1000.0

    success = result.used_direct_path or bool(result.waypoints_mm)
    waypoints_mm = (request.start_mm, *result.waypoints_mm) if success else ()
    message = (
        "direct line of sight" if result.used_direct_path
        else "reused previous path" if result.reused_previous
        else "voronoi map solved" if success
        else "no path found"
    )
    return ComparablePlan(
        "Voronoi+Dijkstra",
        success,
        path_length_mm(waypoints_mm) if len(waypoints_mm) >= 2 else 0.0,
        planning_time_ms,
        waypoints_mm,
        message,
    )


def run_all(request: PlanRequest, *, seed: int = 7, num_samples: int = 40) -> tuple[ComparablePlan, ...]:
    return (
        _run_prm(request, seed=seed, num_samples=num_samples),
        _run_visibility(request),
        _run_voronoi(request),
    )


def build_fixed_scenario() -> PlanRequest:
    """One robot crossing the field through a loose defensive line of six opponents.

    One opponent (id=2) sits almost exactly on the straight start->goal line
    (y=0) so all three planners are actually forced to detour instead of
    trivially succeeding via the direct line-of-sight shortcut -- that's the
    whole point of a "compare obstacle-avoiding planners" demo.
    """
    obstacles = tuple(
        Obstacle(pos_mm=(x, y), radius_mm=90.0, robot_id=idx, isYellow=False)
        for idx, (x, y) in enumerate(
            [
                (-1000.0, 1200.0),
                (-500.0, -800.0),
                (200.0, 30.0),
                (900.0, -1300.0),
                (1600.0, 900.0),
                (-1800.0, -100.0),
            ]
        )
    )
    return PlanRequest(
        start_mm=(-4000.0, 0.0),
        goal_mm=(4000.0, 0.0),
        obstacles=obstacles,
        robot_radius_mm=90.0,
        clearance_mm=40.0,
    )


def run_once(request: PlanRequest) -> None:
    plans = run_all(request)

    print("=== Fixed scenario: 1 robot, 6 opponents, 9000x6000mm field ===")
    print(f"start={request.start_mm}  goal={request.goal_mm}")
    print()
    print(f"{'planner':<25} {'success':<8} {'length_mm':>10} {'time_ms':>9} {'waypoints':>10}")
    for plan in plans:
        print(
            f"{plan.name:<25} {plan.success!s:<8} {plan.path_length_mm:>10.1f} "
            f"{plan.planning_time_ms:>9.2f} {len(plan.waypoints_mm):>10}"
        )
    print()
    for plan in plans:
        print(f"{plan.name} message: {plan.message}")

    try:
        _plot(request, plans)
    except ImportError:
        print("\n(matplotlib not installed -- skipping demo_output.png)")


def _plot(request: PlanRequest, plans: tuple[ComparablePlan, ...]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    fig, axes = plt.subplots(1, len(plans), figsize=(8 * len(plans), 6))
    for ax, plan in zip(axes, plans):
        ax.set_title(plan.name)
        ax.set_xlim(-FIELD_LENGTH_MM / 2, FIELD_LENGTH_MM / 2)
        ax.set_ylim(-FIELD_WIDTH_MM / 2, FIELD_WIDTH_MM / 2)
        ax.set_aspect("equal")
        for obs in request.obstacles:
            ax.add_patch(Circle(obs.pos_mm, obs.radius_mm, color="red", alpha=0.5))
            inflated = obs.radius_mm + (request.total_clearance_mm - request.robot_radius_mm)
            ax.add_patch(
                Circle(obs.pos_mm, inflated, color="red", alpha=0.15, linestyle="--", fill=False)
            )
        ax.plot(*request.start_mm, "go", markersize=10, label="start")
        ax.plot(*request.goal_mm, "b*", markersize=14, label="goal")
        if plan.success and plan.waypoints_mm:
            xs = [p[0] for p in plan.waypoints_mm]
            ys = [p[1] for p in plan.waypoints_mm]
            ax.plot(xs, ys, "k-", linewidth=2)
        ax.legend(loc="upper right")
        ax.grid(alpha=0.3)

    out_path = Path(__file__).resolve().parent.parent / "demo_output.png"
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    print(f"\nSaved comparison plot to {out_path}")


def run_stress_test(trials: int) -> None:
    print(f"\n=== Randomized stress test: {trials} trials, 3-8 obstacles each ===")
    rng = random.Random(42)
    times: dict[str, list[float]] = {name: [] for name in PLANNER_NAMES}
    lengths: dict[str, list[float]] = {name: [] for name in PLANNER_NAMES}
    successes: dict[str, int] = {name: 0 for name in PLANNER_NAMES}

    for _ in range(trials):
        n_obs = rng.randint(3, 8)
        obstacles = tuple(
            Obstacle(
                pos_mm=(rng.uniform(-3500, 3500), rng.uniform(-2500, 2500)),
                radius_mm=90.0,
                robot_id=i,
            )
            for i in range(n_obs)
        )
        request = PlanRequest(
            start_mm=(rng.uniform(-4200, -3000), rng.uniform(-2500, 2500)),
            goal_mm=(rng.uniform(3000, 4200), rng.uniform(-2500, 2500)),
            obstacles=obstacles,
        )

        for plan in run_all(request, seed=rng.randint(0, 10_000), num_samples=40):
            times[plan.name].append(plan.planning_time_ms)
            successes[plan.name] += int(plan.success)
            if plan.success:
                lengths[plan.name].append(plan.path_length_mm)

    for name in PLANNER_NAMES:
        plan_times = times[name]
        avg_time = sum(plan_times) / len(plan_times)
        plan_lengths = lengths[name]
        length_note = (
            f"avg_length={sum(plan_lengths) / len(plan_lengths):.0f}mm"
            if plan_lengths
            else "avg_length=n/a"
        )
        print(
            f"{name:<25} success={successes[name]}/{trials}  "
            f"avg_time={avg_time:.2f}ms  min={min(plan_times):.2f}ms  max={max(plan_times):.2f}ms  "
            f"{length_note}"
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trials", type=int, default=0, help="also run N randomized stress-test trials"
    )
    args = parser.parse_args()

    run_once(build_fixed_scenario())
    if args.trials > 0:
        run_stress_test(args.trials)
