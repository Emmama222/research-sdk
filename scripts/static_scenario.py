#!/usr/bin/env python3
"""Replicate da Silva Costa & Tonidandel's STATIC test, their Figs 11-14.

Companion to `dynamic_scenario.py`, which covers their dynamic perspective.
Together these are Test 1 of the study: three planners, static and dynamic.

Their static perspective (JIRS 2021, section 5) uses four hand-designed
scenarios and three metrics, 200 samples each:

  computational time   ms per planning call
  path length          m
  path safety          m, the SUM of perpendicular distances from every
                       obstacle to the path (their Fig. 10). Higher is safer.

The four scenarios, in their words:

  1. worst case for DVG+A*   "placing all possible obstacles in the active
                              region without grouping them", which maximises
                              the visibility graph's vertex count
  2. worst case for RRT      "a narrow passage before reaching the goal point,
                              and a considerable distance between the start and
                              goal points". We have no RRT; PRM is our
                              sampling-based planner and suffers narrow
                              passages the same way, so this is PRM's worst case
  3. mixed                   "a considerable amount of obstacles in the path and
                              the distance between the start and goal points is
                              not short"
  4. game stoppage           the in-game positioning case, where "the rules
                              define that the robots need to keep a distance of
                              at least 0.5 m from the ball"

## Their results, to compare against

Table 4, worst case DVG+A*:  DVG+A* time 6.50 ms (sd 4.55), length 7.58 m
                             (sd 0.00), safety 7.84 m (sd 0.00)
Table 5, worst case RRT:     DVG+A* time 1.38, length 12.94, safety 4.42
Table 6, mixed:              DVG+A* time 0.94, length 9.02, safety 1.65
Table 7, stoppage:           DVG+A* time 1.92, length 3.65, safety 3.14

Note the zero standard deviations: DVG and A* are deterministic, so 200 samples
vary only the clock. Ours behaves the same way. VisibilityGraph and Voronoi are
deterministic and report sd 0.00 on length and safety; PRM samples and does not.
That is a property of the algorithms, not a bug in the harness.

## What is approximate

Obstacle coordinates are not published. These reproduce each scenario's
CHARACTER as the text describes it, scaled from their 12 x 9 m field to our
9 x 6 m Division B field. Treat the arrangement as faithful and the exact
pixels as not. Robot radius and clearance are physical and are not scaled.

Usage:
    python scripts/static_scenario.py                     # all four, 3 planners
    python scripts/static_scenario.py --samples 200
    python scripts/static_scenario.py --plot static.png   # draw all four
    python scripts/static_scenario.py --selftest
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))
sys.path.insert(0, str(HERE))

from demo_planners import PLANNER_NAMES, _run_prm, _run_visibility, _run_voronoi

from research_sdk.planners.common import DEFAULT_ROBOT_RADIUS_MM, Obstacle, PlanRequest

# Same runners the dynamic test uses, so the two halves of Test 1 measure the
# same thing. All force skip_direct_path=True: a scenario where the straight
# line is free measures nothing about a planner.
PLANNER_RUNNERS = {
    "PRM+Dijkstra": lambda req, seed: _run_prm(req, seed=seed, num_samples=40),
    "VisibilityGraph+Dijkstra": lambda req, seed: _run_visibility(req),
    "Voronoi+Dijkstra": lambda req, seed: _run_voronoi(req),
}

# Their 12 x 9 m field to our 9 x 6 m. Coordinates below are written in THEIR
# frame and scaled here, so they can be checked against the paper's figures.
SX, SY = 9000.0 / 12000.0, 6000.0 / 9000.0

# The stoppage rule: robots keep 0.5 m from the ball. Modelled as an extra
# obstacle at the ball with a 500 mm radius, which is what the rule means
# geometrically for a planner.
BALL_KEEPOUT_MM = 500.0


def _obs(*points) -> tuple[Obstacle, ...]:
    """Obstacles from their-frame coordinates, scaled to ours."""
    return tuple(
        Obstacle(pos_mm=(x * SX, y * SY), radius_mm=DEFAULT_ROBOT_RADIUS_MM, robot_id=i)
        for i, (x, y) in enumerate(points)
    )


@dataclass(frozen=True, slots=True)
class StaticScenario:
    name: str
    start_mm: tuple[float, float]
    goal_mm: tuple[float, float]
    obstacles: tuple[Obstacle, ...]
    note: str


SCENARIOS: tuple[StaticScenario, ...] = (
    StaticScenario(
        name="worst_case_graph (their Fig. 11)",
        start_mm=(-4800.0 * SX, 0.0),
        goal_mm=(4800.0 * SX, 300.0 * SY),
        # Nine obstacles spread across the corridor and NOT grouped: every one
        # contributes its own polygon, so the visibility graph carries the most
        # vertices it can. That is exactly the condition they describe.
        obstacles=_obs(
            (-3200, 700), (-2400, -600), (-1600, 800), (-800, -700), (0, 600),
            (800, -800), (1600, 700), (2400, -600), (3200, 500),
        ),
        note="all obstacles in the active region, ungrouped",
    ),
    StaticScenario(
        name="worst_case_sampling (their Fig. 12)",
        start_mm=(-5000.0 * SX, 3000.0 * SY),
        goal_mm=(5000.0 * SX, -3000.0 * SY),
        # A wall across the field with ONE door, plus a long start-goal
        # distance. Both parts are needed for a sampler's worst case.
        #
        # The first attempt at this scenario used two short rows of obstacles
        # and did not punish PRM at all: PRM 10.15 ms against VisibilityGraph
        # 14.24 ms. Measuring the geometry showed why. Adjacent obstacles left
        # 228 mm of free space, too narrow to pass, but the widest gap anywhere
        # was 3079 mm, so the sampler ignored the "passage" and walked around
        # the whole barrier. A narrow passage only bites when there is no way
        # around it.
        #
        # The passability criterion is centre-to-centre distance, not "free
        # space between inflated discs". A robot passing between two obstacles
        # only needs its OWN CENTRE to stay 210 mm from each centre, so the gap
        # is passable when centre distance >= 420 mm. A first attempt at this
        # wall used 600 mm spacing and computed free space as
        # distance - 2 * 210 = 180 mm, concluded "impassable", and was wrong:
        # 600 >= 420, so every gap in that wall was open. The selftest's
        # straight-line check caught it.
        #
        # Spacing is 600 mm in their frame, 400 mm in ours. 400 < 420, so every
        # adjacent pair is genuinely impassable. Dropping the obstacle at
        # y = 1500 leaves its neighbours 800 mm apart, a door whose passable
        # corridor is 800 - 420 = 380 mm wide. The wall spans the full 6 m
        # field width, so there is no route around its ends.
        #
        # That corridor is roughly 0.16 m^2 of a 54 m^2 field, about 0.3% of
        # uniform samples. PRM draws 40.
        obstacles=_obs(
            (0, -4500), (0, -3900), (0, -3300), (0, -2700), (0, -2100), (0, -1500),
            (0, -900), (0, -300), (0, 300), (0, 900), (0, 2100), (0, 2700),
            (0, 3300), (0, 3900), (0, 4500),
        ),
        note="wall across the field with one 380 mm corridor, long start-goal distance",
    ),
    StaticScenario(
        name="mixed (their Fig. 13)",
        start_mm=(-4600.0 * SX, 2600.0 * SY),
        goal_mm=(4600.0 * SX, -2600.0 * SY),
        obstacles=_obs(
            (-2500, 1400), (-1500, 900), (-500, 300), (500, -300), (1500, -900),
            (2500, -1500), (-1000, -600), (1000, 600),
        ),
        note="many obstacles on the path, start and goal not close",
    ),
    StaticScenario(
        name="game_stoppage (their Fig. 14)",
        start_mm=(-3000.0 * SX, 1800.0 * SY),
        goal_mm=(1200.0 * SX, -900.0 * SY),
        # Robots gathered around the ball during a stoppage. The ball itself is
        # added below as a 500 mm keep-out.
        obstacles=_obs(
            (-1400, 900), (-800, 300), (-200, 900), (-800, -400),
            (-1600, 0), (100, 200),
        )
        + (
            Obstacle(pos_mm=(-700.0 * SX, 400.0 * SY), radius_mm=BALL_KEEPOUT_MM, robot_id=99),
        ),
        note="stoppage positioning, 0.5 m ball keep-out as an obstacle",
    ),
)


def point_to_path_mm(point, waypoints) -> float:
    """Perpendicular distance from a point to a polyline."""
    if len(waypoints) < 2:
        return float("inf")
    px, py = point
    best = float("inf")
    for (x0, y0), (x1, y1) in pairwise(waypoints):
        dx, dy = x1 - x0, y1 - y0
        seg_sq = dx * dx + dy * dy
        t = 0.0 if seg_sq == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / seg_sq))
        best = min(best, math.hypot(px - (x0 + t * dx), py - (y0 + t * dy)))
    return best


def path_safety_m(waypoints, obstacles) -> float:
    """Their metric: the SUM of perpendicular obstacle-to-path distances, in m.

    A sum, not a mean, so it scales with obstacle count and is only comparable
    within a scenario. That is their definition and changing it would break
    comparability with Tables 4-7.
    """
    if len(waypoints) < 2:
        return float("nan")
    return sum(point_to_path_mm(o.pos_mm, waypoints) for o in obstacles) / 1000.0


def path_length_m(waypoints) -> float:
    return sum(math.dist(a, b) for a, b in pairwise(waypoints)) / 1000.0


def run_scenario(scenario: StaticScenario, samples: int) -> dict[str, dict]:
    """{planner: {time_ms, length_m, safety_m, failures}} over `samples` calls."""
    request = PlanRequest(
        start_mm=scenario.start_mm, goal_mm=scenario.goal_mm, obstacles=scenario.obstacles
    )
    out: dict[str, dict] = {}
    for name in PLANNER_NAMES:
        times, lengths, safeties, failures = [], [], [], 0
        for s in range(samples):
            plan = PLANNER_RUNNERS[name](request, s)
            times.append(plan.planning_time_ms)
            if plan.success and len(plan.waypoints_mm) >= 2:
                lengths.append(path_length_m(plan.waypoints_mm))
                safeties.append(path_safety_m(plan.waypoints_mm, scenario.obstacles))
            else:
                failures += 1
        out[name] = {
            "time_ms": times, "length_m": lengths, "safety_m": safeties,
            "failures": failures, "n": samples,
        }
    return out


def _stat_block(values) -> tuple[str, str, str, str]:
    if not values:
        return "--", "--", "--", "--"
    return (
        f"{statistics.mean(values):.2f}",
        f"{statistics.pstdev(values):.2f}",
        f"{min(values):.2f}",
        f"{max(values):.2f}",
    )


THEIR_RESULTS = {
    "worst_case_graph (their Fig. 11)": ("Table 4", 6.50, 7.58, 7.84),
    "worst_case_sampling (their Fig. 12)": ("Table 5", 1.38, 12.94, 4.42),
    "mixed (their Fig. 13)": ("Table 6", 0.94, 9.02, 1.65),
    "game_stoppage (their Fig. 14)": ("Table 7", 1.92, 3.65, 3.14),
}


def report(scenario: StaticScenario, results: dict, samples: int) -> None:
    print(f"\n=== {scenario.name} ===")
    print(f"  {scenario.note}; {len(scenario.obstacles)} obstacles, "
          f"start {tuple(round(v) for v in scenario.start_mm)} -> "
          f"goal {tuple(round(v) for v in scenario.goal_mm)}, n = {samples}")
    header = (
        f"{'planner':<26}{'fail':>6}"
        f"{'time ms':>10}{'sd':>7}{'min':>7}{'max':>8}"
        f"{'length m':>11}{'sd':>7}"
        f"{'safety m':>11}{'sd':>7}"
    )
    print("  " + header)
    print("  " + "-" * len(header))
    for name in PLANNER_NAMES:
        r = results[name]
        tm, tsd, tmin, tmax = _stat_block(r["time_ms"])
        lm, lsd, _, _ = _stat_block(r["length_m"])
        sm, ssd, _, _ = _stat_block(r["safety_m"])
        print(
            f"  {name:<26}{r['failures']:>6}"
            f"{tm:>10}{tsd:>7}{tmin:>7}{tmax:>8}"
            f"{lm:>11}{lsd:>7}"
            f"{sm:>11}{ssd:>7}"
        )
    table, t_time, t_len, t_safe = THEIR_RESULTS[scenario.name]
    print(
        f"\n  their DVG+A* ({table}): time {t_time:.2f} ms, length {t_len:.2f} m, "
        f"safety {t_safe:.2f} m"
        f"\n  Time is not comparable across implementations (theirs is C++ on an i7 at"
        f"\n  4.5 GHz, ours is Python). Length and safety are geometry and are."
        f"\n  sd 0.00 on length and safety means the planner is deterministic, as theirs is."
    )


def plot(all_results: dict, out: str, samples: int) -> str:
    """Four panels, one per scenario, each showing all three planners' paths."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    SURFACE, INK, INK_2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#d8d7d3"
    COLOURS = {
        "PRM+Dijkstra": "#2a78d6",
        "VisibilityGraph+Dijkstra": "#eb6834",
        "Voronoi+Dijkstra": "#1baf7a",
    }
    OBSTACLE, INFLATE = "#9a9994", "#c9c8c4"

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.patch.set_facecolor(SURFACE)
    for ax, scenario in zip(axes.flat, SCENARIOS):
        ax.set_facecolor(SURFACE)
        ax.add_patch(Rectangle((-4500, -3000), 9000, 6000, fill=False,
                               edgecolor=GRID, linewidth=1.2))
        ax.plot([0, 0], [-3000, 3000], color=GRID, linewidth=1.0)
        ax.add_patch(Circle((0, 0), 500, fill=False, edgecolor=GRID, linewidth=1.0))

        request = PlanRequest(start_mm=scenario.start_mm, goal_mm=scenario.goal_mm,
                              obstacles=scenario.obstacles)
        inflate = request.total_clearance_mm
        for o in scenario.obstacles:
            ax.add_patch(Circle(o.pos_mm, o.radius_mm + inflate, fill=False,
                                edgecolor=INFLATE, linewidth=0.7,
                                linestyle=(0, (3, 3)), zorder=2))
            ax.add_patch(Circle(o.pos_mm, o.radius_mm, color=OBSTACLE, zorder=4))

        for name in PLANNER_NAMES:
            plan = PLANNER_RUNNERS[name](request, 0)
            if plan.success and len(plan.waypoints_mm) >= 2:
                ax.plot([p[0] for p in plan.waypoints_mm], [p[1] for p in plan.waypoints_mm],
                        color=COLOURS[name], linewidth=2.0, zorder=5,
                        solid_capstyle="round", label=name.replace("+Dijkstra", ""))
        ax.scatter(*scenario.start_mm, s=90, color=INK, zorder=7)
        ax.scatter(*scenario.goal_mm, s=190, marker="*", color=INK, zorder=7)

        ax.set_xlim(-4800, 4800)
        ax.set_ylim(-3300, 3300)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.set_title(scenario.name, color=INK, fontsize=11.5, fontweight="bold", loc="left")
        ax.legend(loc="lower right", fontsize=8, frameon=False, labelcolor=INK_2)

    fig.suptitle(
        f"Static scenarios after da Silva Costa & Tonidandel Figs. 11-14, n = {samples}",
        color=INK, fontsize=13, fontweight="bold", x=0.01, ha="left",
    )
    fig.text(0.01, 0.01,
             "dot = start, star = goal, grey = obstacle, dashed = the inflation the planner avoids",
             color=INK_2, fontsize=8.5)
    fig.tight_layout(rect=(0, 0.03, 1, 0.95))
    fig.savefig(out, dpi=160, facecolor=SURFACE, bbox_inches="tight")
    plt.close(fig)
    return out


def selftest() -> None:
    # Path safety: one obstacle 300 mm from a straight path sums to 0.3 m.
    path = ((0.0, 0.0), (1000.0, 0.0))
    assert abs(path_safety_m(path, (Obstacle(pos_mm=(500.0, 300.0)),)) - 0.3) < 1e-9
    # It is a SUM, so two such obstacles give 0.6, not 0.3.
    two = (Obstacle(pos_mm=(300.0, 300.0)), Obstacle(pos_mm=(700.0, 300.0)))
    assert abs(path_safety_m(path, two) - 0.6) < 1e-9
    assert abs(path_length_m(((0.0, 0.0), (3000.0, 4000.0))) - 5.0) < 1e-9

    # Every scenario must be solvable by at least one planner, or it measures
    # nothing. Also catches an endpoint accidentally placed inside an obstacle.
    for sc in SCENARIOS:
        req = PlanRequest(start_mm=sc.start_mm, goal_mm=sc.goal_mm, obstacles=sc.obstacles)
        solved = [n for n in PLANNER_NAMES if PLANNER_RUNNERS[n](req, 0).success]
        assert solved, f"{sc.name}: no planner solved it"
        for o in sc.obstacles:
            clearance = o.radius_mm + req.total_clearance_mm
            assert math.dist(o.pos_mm, sc.start_mm) > clearance, f"{sc.name}: start inside an obstacle"
            assert math.dist(o.pos_mm, sc.goal_mm) > clearance, f"{sc.name}: goal inside an obstacle"

    # The scenarios must actually be blocked: a free straight line would make
    # every planner identical and the whole table meaningless. An obstacle
    # blocks the line when the line passes within the inflation radius of its
    # CENTRE, which is what the planner tests too.
    for sc in SCENARIOS:
        req = PlanRequest(start_mm=sc.start_mm, goal_mm=sc.goal_mm, obstacles=sc.obstacles)
        direct = (sc.start_mm, sc.goal_mm)
        assert any(
            point_to_path_mm(o.pos_mm, direct) < o.radius_mm + req.total_clearance_mm
            for o in sc.obstacles
        ), f"{sc.name}: the straight line is free, so this scenario tests nothing"

    # The narrow-passage scenario must have exactly one passable gap, or it is
    # not a narrow passage. Passable means centre distance >= 2 * inflation.
    wall = SCENARIOS[1]
    req = PlanRequest(start_mm=wall.start_mm, goal_mm=wall.goal_mm, obstacles=wall.obstacles)
    min_pass = 2 * (wall.obstacles[0].radius_mm + req.total_clearance_mm)
    ys = sorted(o.pos_mm[1] for o in wall.obstacles)
    doors = [(a, b) for a, b in pairwise(ys) if (b - a) >= min_pass]
    assert len(doors) == 1, f"expected exactly one door, found {len(doors)}: {doors}"
    # And the wall must reach past both touchlines, or the sampler walks around.
    assert ys[0] <= -3000 and ys[-1] >= 3000, f"wall spans only {ys[0]:.0f}..{ys[-1]:.0f}"
    print("selftest OK")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=200)
    parser.add_argument("--plot", metavar="PATH")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()

    if args.selftest:
        selftest()
        return

    all_results = {}
    for scenario in SCENARIOS:
        results = run_scenario(scenario, args.samples)
        all_results[scenario.name] = results
        report(scenario, results, args.samples)

    if args.plot:
        print(f"\n  figure: {plot(all_results, args.plot, args.samples)}")


if __name__ == "__main__":
    main()
