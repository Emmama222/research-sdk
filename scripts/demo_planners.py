#!/usr/bin/env python3
"""Runnable demo/sandbox for the PRM+Dijkstra and Visibility Graph+Dijkstra planners.

Builds a fixed SSL-like scenario (one robot navigating across a crowd of
opponents on a 9000x6000mm field), runs both planners against it, prints a
comparison table, and -- if matplotlib is available -- saves a side-by-side
plot to ``demo_output.png`` so you can eyeball both paths without wiring up
grSim, Phoenix, or the (not-yet-built) Qt UI.

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
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from research_sdk.planners.common import (  # noqa: E402
    FIELD_LENGTH_MM,
    FIELD_WIDTH_MM,
    Obstacle,
    PlanRequest,
)
from research_sdk.planners.PRM import prm_dijkstra  # noqa: E402
from research_sdk.planners.VisibilityGraph import visibility_graph  # noqa: E402


def build_fixed_scenario() -> PlanRequest:
    """One robot crossing the field through a loose defensive line of six opponents.

    One opponent (id=2) sits almost exactly on the straight start->goal line
    (y=0) so both planners are actually forced to detour instead of trivially
    succeeding via the direct line-of-sight shortcut -- that's the whole point
    of a "compare two obstacle-avoiding planners" demo.
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
    prm_result = prm_dijkstra.plan(request, seed=7, num_samples=40)
    vg_result = visibility_graph.plan(request)

    print("=== Fixed scenario: 1 robot, 6 opponents, 9000x6000mm field ===")
    print(f"start={request.start_mm}  goal={request.goal_mm}")
    print()
    print(f"{'planner':<20} {'success':<8} {'length_mm':>10} {'time_ms':>9} {'waypoints':>10}")
    for name, result in (("PRM+Dijkstra", prm_result), ("VisibilityGraph+Dijkstra", vg_result)):
        print(
            f"{name:<20} {str(result.success):<8} {result.path_length_mm:>10.1f} "
            f"{result.planning_time_ms:>9.2f} {len(result.waypoints_mm):>10}"
        )
    print()
    print(f"PRM message: {prm_result.message}")
    print(f"Visibility graph message: {vg_result.message}")

    try:
        _plot(request, prm_result, vg_result)
    except ImportError:
        print("\n(matplotlib not installed -- skipping demo_output.png)")


def _plot(request: PlanRequest, prm_result, vg_result) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    for ax, (title, result) in zip(
        axes, [("PRM + Dijkstra", prm_result), ("Visibility Graph + Dijkstra", vg_result)]
    ):
        ax.set_title(title)
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
        if result.success:
            xs = [p[0] for p in result.waypoints_mm]
            ys = [p[1] for p in result.waypoints_mm]
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
    prm_times, vg_times = [], []
    prm_successes = vg_successes = 0

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

        prm_result = prm_dijkstra.plan(request, seed=rng.randint(0, 10_000), num_samples=40)
        vg_result = visibility_graph.plan(request)

        prm_times.append(prm_result.planning_time_ms)
        vg_times.append(vg_result.planning_time_ms)
        prm_successes += prm_result.success
        vg_successes += vg_result.success

    def summarize(name: str, times: list[float], successes: int) -> None:
        avg = sum(times) / len(times)
        print(
            f"{name:<25} success={successes}/{trials}  "
            f"avg={avg:.2f}ms  min={min(times):.2f}ms  max={max(times):.2f}ms"
        )

    summarize("PRM+Dijkstra", prm_times, prm_successes)
    summarize("VisibilityGraph+Dijkstra", vg_times, vg_successes)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trials", type=int, default=0, help="also run N randomized stress-test trials"
    )
    args = parser.parse_args()

    run_once(build_fixed_scenario())
    if args.trials > 0:
        run_stress_test(args.trials)
