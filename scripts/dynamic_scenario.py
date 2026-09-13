#!/usr/bin/env python3
"""Replicate da Silva Costa & Tonidandel's dynamic replanning test, offline.

Their JIRS 2021 paper (docs/papers/dasilvacosta-tonidandel-2021-...) evaluates
planners in a "dynamic perspective": the robot's start and goal sit on a
horizontal line from the centre circle to the goal area (their Figs 15-16, the
red line), and 7 or 5 obstacles run on fixed rectangular loops that cross that
line. One sample = the robot leaves start and reaches goal. They report:

  accumulated computational time   sum of every planning call until arrival, ms
  paths recalculated               times the path was invalid and replanned
  navigation time                  start to goal, s
  number of collisions             contacts with any obstacle

Their replan trigger, reproduced exactly: a path is invalid when any obstacle
comes within 90 mm (the robot radius) of it. Their own words on its weakness
are the gap this work exists to fill -- "even though a new path would only need
a minor change in the invalid path, the whole algorithm needs to run again."

Their benchmark to beat (Table 9, scenario 1): DVG+A* 13.09 recalculations
mean, RRT 34.08.

## What is scaled and what is not

Their field is 12 x 9 m (Division A at the time); ours is 9 x 6 m Division B,
so the loop geometry is scaled by 0.75 in x and 0.667 in y. Robot radius (90 mm)
and the 90 mm trigger are physical and are NOT scaled. They do not state the
obstacle speed; it is a parameter here, defaulted to 1.0 m/s, and every result
prints it.

## Why offline

Everything the metric needs -- obstacle poses, the robot's progress along its
path, contact -- is kinematic. Running it in grSim would add physics noise
without adding information, and would cost minutes per sample instead of
milliseconds. grSim is for the closed-loop tier where control error matters.

Usage:
    python scripts/dynamic_scenario.py                # both scenarios, 3 planners
    python scripts/dynamic_scenario.py --samples 50 --obstacle-speed 1.5
    python scripts/dynamic_scenario.py --plot scenario.png   # their Fig 15, ours
    python scripts/dynamic_scenario.py --selftest
"""

from __future__ import annotations

import argparse
import math
import random
import statistics
import sys
from collections import deque
from dataclasses import dataclass, field
from itertools import pairwise
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "src"))
sys.path.insert(0, str(HERE))

from path_stability import Stability, point_to_polyline_mm

from demo_planners import PLANNER_NAMES, _run_prm, _run_visibility, _run_voronoi

from research_sdk.planners.common import DEFAULT_ROBOT_RADIUS_MM, Obstacle, PlanRequest

# One planner per call. demo_planners' runners all force skip_direct_path=True,
# so every replan here is a full map/graph/roadmap build -- the same condition
# under which the replication matched their Table 9 (12.73 vs 13.09).
PLANNER_RUNNERS = {
    "PRM+Dijkstra": lambda req, seed: _run_prm(req, seed=seed, num_samples=40),
    "VisibilityGraph+Dijkstra": lambda req, seed: _run_visibility(req),
    "Voronoi+Dijkstra": lambda req, seed: _run_voronoi(req),
}

ROBOT_RADIUS_MM = DEFAULT_ROBOT_RADIUS_MM
INVALIDATION_MM = 90.0          # their trigger threshold: the robot radius
CONTACT_MM = 2 * ROBOT_RADIUS_MM  # centre distance at which two robots touch
CONTROL_TICK_S = 0.05           # ui/execution/page.py:380
ARRIVED_MM = 60.0               # docs/results_metrics.md, final waypoint tolerance

# Scale from their 12 x 9 m field to ours.
SX, SY = 9000.0 / 12000.0, 6000.0 / 9000.0


@dataclass(frozen=True, slots=True)
class Loop:
    """A closed polyline an obstacle patrols at constant speed."""

    corners: tuple[tuple[float, float], ...]

    def length(self) -> float:
        pts = (*self.corners, self.corners[0])
        return sum(math.dist(a, b) for a, b in pairwise(pts))

    def position(self, s: float) -> tuple[float, float]:
        """Point at arc length `s` (mm), wrapping."""
        s %= self.length()
        pts = (*self.corners, self.corners[0])
        for a, b in pairwise(pts):
            seg = math.dist(a, b)
            if s <= seg:
                t = s / seg if seg else 0.0
                return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            s -= seg
        return self.corners[0]


def _rect(x0: float, y0: float, x1: float, y1: float) -> Loop:
    return Loop(((x0, y0), (x1, y0), (x1, y1), (x0, y1)))


def _para(x0: float, y0: float, x1: float, y1: float, shear: float) -> Loop:
    """Parallelogram: their Fig 16 loops lean, so the crossing is oblique."""
    return Loop(((x0, y0), (x1, y0), (x1 + shear, y1), (x0 + shear, y1)))


# The red line, their Figs 15-16: just outside the centre circle to the edge of
# the goal area, at y = 0. Scaled from roughly x = 700..5000 on their field.
START_MM = (700.0 * SX, 0.0)
GOAL_MM = (5000.0 * SX, 0.0)

# Loops eyeballed from their Fig 15 (7 obstacles) and Fig 16 (5 obstacles),
# then scaled. Each straddles y = 0 so the obstacle crosses the red line twice
# per lap. The exact corners are not in the paper -- these reproduce the
# arrangement, not the pixels, and the reader should treat the counts and
# crossing pattern as the faithful part.
SCENARIO_1 = tuple(
    Loop(tuple((x * SX, y * SY) for x, y in loop.corners))
    for loop in (
        _rect(1000, -600, 1600, 1400),
        _rect(1500, -1200, 2100, 900),
        _rect(2000, -800, 2500, 1200),
        _rect(2500, -1400, 3100, 600),
        _rect(3000, -700, 3500, 1500),
        _rect(3600, -1100, 4200, 1000),
        _para(1200, -1500, 4300, 1600, 800),
    )
)
SCENARIO_2 = tuple(
    Loop(tuple((x * SX, y * SY) for x, y in loop.corners))
    for loop in (
        _rect(800, -700, 1500, 900),
        _para(1600, -1600, 2200, 1500, 700),
        _para(2400, -1500, 3000, 1500, 600),
        _para(3200, -1700, 3800, 1400, 700),
        _para(4000, -1000, 4500, 1200, 500),
    )
)
SCENARIOS = {"scenario_1 (7 obstacles)": SCENARIO_1, "scenario_2 (5 obstacles)": SCENARIO_2}


# Delegates rather than reimplements: path_stability.py needs the same
# distance for its resampled points, and two copies of it would be exactly the
# kind of doubt the planners' shared-search comment warns about.
point_to_path_mm = point_to_polyline_mm


def _rooted_at(pos, path):
    """A plan as the robot sees it: starting where the robot stands.

    Waypoints within 1 mm of the robot are dropped first. The three planners
    disagree about whether the start belongs in their output, so without this
    the same route reads as two different plans depending on the backend.
    """
    if not path:
        return ()
    return (pos, *[w for w in path if math.dist(w, pos) > 1.0])


@dataclass(frozen=True, slots=True)
class TriggerContext:
    """Everything a replan trigger may look at, at one control tick."""

    remaining_path: tuple            # from the robot's current position to the goal
    obstacles: tuple                 # obstacle poses now
    history: tuple                   # recent obstacle snapshots, oldest first, current last
    ticks_since_plan: int
    dt_s: float

    @property
    def prev_obstacles(self) -> tuple | None:
        """The snapshot one tick back, or None at the start of a run."""
        return self.history[-2] if len(self.history) >= 2 else None

    def velocity_of(self, obstacle, smooth_ticks: int = 1):
        """Finite-difference velocity over `smooth_ticks`, mm/s, or None.

        A one-tick difference is what a controller gets for free and is exact
        in a noiseless simulation. Real vision is not noiseless: the measured
        per-frame step on a match log is 23 mm at 70 Hz, which a one-tick
        difference turns into 1.6 m/s of phantom speed. Averaging over a longer
        window trades responsiveness for that noise.
        """
        if len(self.history) < smooth_ticks + 1:
            return None
        older = {o.robot_id: o.pos_mm for o in self.history[-1 - smooth_ticks]}
        if obstacle.robot_id not in older:
            return None
        span = smooth_ticks * self.dt_s
        prev = older[obstacle.robot_id]
        return (
            (obstacle.pos_mm[0] - prev[0]) / span,
            (obstacle.pos_mm[1] - prev[1]) / span,
        )


def _within(path, obstacles, threshold_mm: float = INVALIDATION_MM) -> bool:
    return any(point_to_path_mm(o.pos_mm, path) <= threshold_mm for o in obstacles)


def trigger_geometric(ctx: TriggerContext) -> bool:
    """Theirs: any obstacle within 90 mm of the path still to be driven.

    Purely geometric, no prediction. Their own critique: it cannot see a robot
    heading toward the path, and it replans the whole thing for a local change.
    """
    return _within(ctx.remaining_path, ctx.obstacles)


def trigger_periodic(every_ticks: int):
    """Replan every k ticks, blind to the world. The trivial control.

    If a 'smart' trigger cannot beat this, it is not smart. k=1 is the
    replan-every-tick policy the SDK's ExecutionController effectively runs.
    """

    def _t(ctx: TriggerContext) -> bool:
        return ctx.ticks_since_plan >= every_ticks

    _t.__name__ = f"periodic_{every_ticks * CONTROL_TICK_S * 1000:.0f}ms"
    return _t


def trigger_geometric_or_periodic(every_ticks: int):
    """Geometric with a periodic safety net: what a cautious engineer ships."""

    def _t(ctx: TriggerContext) -> bool:
        return trigger_geometric(ctx) or ctx.ticks_since_plan >= every_ticks

    _t.__name__ = f"geometric+periodic_{every_ticks * CONTROL_TICK_S * 1000:.0f}ms"
    return _t


def trigger_predictive(horizon_s: float, steps: int = 4, smooth_ticks: int = 1):
    """Project each obstacle along its measured velocity for `horizon_s` and
    replan if it WILL come within 90 mm of the path -- the thing their trigger
    admits it cannot do ("doesn't detect if there is a robot moving in the
    direction of the path"). They cite an SSL algorithm using a 3 s horizon.

    Checked at `steps` points along the horizon, not just the end, so a
    crossing mid-horizon is not missed.

    `smooth_ticks` sets the window the velocity is differenced over. 1 is what
    a controller gets for free; larger windows reject vision noise at the cost
    of lagging a genuine direction change.
    """

    def _t(ctx: TriggerContext) -> bool:
        if _within(ctx.remaining_path, ctx.obstacles):
            return True
        if ctx.dt_s <= 0:
            return False
        for o in ctx.obstacles:
            v = ctx.velocity_of(o, smooth_ticks)
            if v is None:
                continue
            for k in range(1, steps + 1):
                tau = horizon_s * k / steps
                projected = (o.pos_mm[0] + v[0] * tau, o.pos_mm[1] + v[1] * tau)
                if point_to_path_mm(projected, ctx.remaining_path) <= INVALIDATION_MM:
                    return True
        return False

    suffix = "" if smooth_ticks == 1 else f"_smooth{smooth_ticks}"
    _t.__name__ = f"predictive_{horizon_s:.2f}s{suffix}"
    return _t


def trigger_near_field(segments: int):
    """Geometric, but only over the next `segments` of the path.

    An analogue of Lim et al.'s lookahead parameter -- how much of the plan you
    commit to before checking the world -- NOT their algorithm. A far-away
    obstacle crossing the tail of the path does not force a replan now; by the
    time the robot gets there the obstacle has moved on. segments=inf is the
    plain geometric trigger.
    """

    def _t(ctx: TriggerContext) -> bool:
        return _within(ctx.remaining_path[: segments + 1], ctx.obstacles)

    _t.__name__ = f"near_field_{segments}seg"
    return _t


TRIGGERS = [
    trigger_geometric,
    trigger_periodic(1),
    trigger_periodic(4),
    trigger_periodic(10),
    trigger_geometric_or_periodic(10),
    trigger_predictive(0.25),
    trigger_predictive(0.5),
    trigger_predictive(1.0),
    trigger_predictive(0.25, smooth_ticks=5),
    trigger_near_field(1),
    trigger_near_field(2),
]


@dataclass
class RunResult:
    planner: str
    arrived: bool
    navigation_s: float
    accumulated_ms: float
    recalculated: int
    collisions: int
    # Plan-to-plan change, from path_stability.py. Zero means the robot never
    # changed its mind; a high reversal rate is the oscillation the Voronoi
    # roadmap shows under timer-driven replanning.
    replans_compared: int = 0
    reversals: int = 0
    mean_heading_deg: float = 0.0
    mean_shift_mm: float = 0.0
    p95_shift_mm: float = 0.0
    trajectory: list[tuple[float, float]] = field(default_factory=list)
    obstacle_traces: list[list[tuple[float, float]]] = field(default_factory=list)


HISTORY_TICKS = 8  # deepest smoothing window any trigger asks for, plus slack


def simulate(
    planner: str,
    loops,
    *,
    obstacle_speed_mps: float,
    robot_speed_mps: float,
    phases: list[float],
    trigger=trigger_geometric,
    max_s: float = 30.0,
    keep_traces: bool = False,
    noise_mm: float = 0.0,
    noise_seed: int = 0,
) -> RunResult:
    """One sample: drive start -> goal, replanning whenever `trigger` says so.

    Kinematic only. The robot advances along its current polyline at
    `robot_speed_mps` per control tick; obstacles advance along their loops.
    """
    t = 0.0
    pos = START_MM
    trajectory = [pos]
    traces = [[] for _ in loops]
    accumulated_ms = 0.0
    recalculated = 0
    collisions = 0
    in_contact = [False] * len(loops)
    path: tuple = ()
    arrived = False
    ticks_since_plan = 0
    history: deque = deque(maxlen=HISTORY_TICKS)
    # Measurement noise is applied to what the TRIGGER sees, never to the true
    # positions used for collisions. A trigger reads vision; contact is physics.
    noise_rng = random.Random(noise_seed)

    def obstacles_at(t_s: float) -> tuple[Obstacle, ...]:
        return tuple(
            Obstacle(
                pos_mm=loop.position(phases[i] + obstacle_speed_mps * 1000.0 * t_s),
                radius_mm=ROBOT_RADIUS_MM,
                robot_id=i,
                isYellow=False,
            )
            for i, loop in enumerate(loops)
        )

    def measured(obs: tuple[Obstacle, ...]) -> tuple[Obstacle, ...]:
        """What vision reports: true position plus per-frame jitter."""
        if noise_mm <= 0:
            return obs
        return tuple(
            Obstacle(
                pos_mm=(o.pos_mm[0] + noise_rng.gauss(0, noise_mm),
                        o.pos_mm[1] + noise_rng.gauss(0, noise_mm)),
                radius_mm=o.radius_mm, robot_id=o.robot_id, isYellow=o.isYellow,
            )
            for o in obs
        )

    def plan_from(p, obs):
        nonlocal accumulated_ms
        request = PlanRequest(start_mm=p, goal_mm=GOAL_MM, obstacles=obs)
        # One planner per call, not run_all(): that ran all three and tripled
        # both the wall clock and, before it was caught, the accumulated time
        # charged to whichever planner was under test.
        chosen = PLANNER_RUNNERS[planner](request, recalculated)
        accumulated_ms += chosen.planning_time_ms
        return chosen.waypoints_mm if chosen.success else ()

    stability = Stability()
    obs = measured(obstacles_at(t))
    history.append(obs)
    path = plan_from(pos, obs)
    recalculated += 1  # the initial plan counts as a calculation in their accounting

    while t < max_s:
        true_obs = obstacles_at(t)
        obs = measured(true_obs)
        history.append(obs)
        if keep_traces:
            for i, o in enumerate(true_obs):
                traces[i].append(o.pos_mm)

        # Rising-edge collision episodes, per docs/results_metrics.md.
        # True positions: a robot either touched another or it did not,
        # regardless of what the camera reported.
        for i, o in enumerate(true_obs):
            touching = math.dist(pos, o.pos_mm) <= CONTACT_MM
            if touching and not in_contact[i]:
                collisions += 1
            in_contact[i] = touching

        if math.dist(pos, GOAL_MM) <= ARRIVED_MM:
            arrived = True
            break

        remaining = _rooted_at(pos, path)
        ctx = TriggerContext(
            remaining_path=remaining,
            obstacles=obs,
            history=tuple(history),
            ticks_since_plan=ticks_since_plan,
            dt_s=CONTROL_TICK_S,
        )
        if not path or trigger(ctx):
            # Root both plans at the robot and drop any waypoint sitting on
            # top of it, exactly as `remaining` above does. The planners
            # disagree here: the visibility graph and PRM return the start as
            # their first waypoint, Voronoi sometimes does. Prepending `pos`
            # without the filter gave the new plan a zero-length first step,
            # so every heading reading came back undefined and the metric
            # silently recorded nothing.
            previous = remaining
            path = plan_from(pos, obs)
            stability.observe(previous, _rooted_at(pos, path))
            recalculated += 1
            ticks_since_plan = 0
            if not path:
                # Stay put this tick; their planners can fail too and the
                # robot then just waits.
                t += CONTROL_TICK_S
                continue
        ticks_since_plan += 1

        # Advance along the path by one tick's travel.
        budget = robot_speed_mps * 1000.0 * CONTROL_TICK_S
        cur = pos
        rest = list(path)
        while rest and budget > 0:
            nxt = rest[0]
            d = math.dist(cur, nxt)
            if d <= budget:
                budget -= d
                cur = nxt
                rest.pop(0)
            else:
                f = budget / d
                cur = (cur[0] + (nxt[0] - cur[0]) * f, cur[1] + (nxt[1] - cur[1]) * f)
                budget = 0
        pos = cur
        path = tuple(rest)
        trajectory.append(pos)
        t += CONTROL_TICK_S

    return RunResult(
        planner=planner,
        arrived=arrived,
        navigation_s=t,
        accumulated_ms=accumulated_ms,
        recalculated=recalculated,
        collisions=collisions,
        replans_compared=stability.replans_compared,
        reversals=stability.reversals,
        mean_heading_deg=stability.mean_heading_deg,
        mean_shift_mm=stability.mean_shift_mm,
        p95_shift_mm=stability.p95_shift_mm,
        trajectory=trajectory if keep_traces else [],
        obstacle_traces=traces if keep_traces else [],
    )


def run_all_samples(loops, samples: int, obstacle_speed: float, robot_speed: float, seed: int = 0):
    rng = random.Random(seed)
    out: dict[str, list[RunResult]] = {n: [] for n in PLANNER_NAMES}
    for _ in range(samples):
        # Random starting phase per obstacle: the same loops, a different
        # crossing pattern, which is what makes 200 samples a distribution
        # rather than 200 copies of one run.
        phases = [rng.uniform(0, loop.length()) for loop in loops]
        for name in PLANNER_NAMES:
            out[name].append(
                simulate(name, loops, obstacle_speed_mps=obstacle_speed,
                         robot_speed_mps=robot_speed, phases=phases)
            )
    return out


def _stats(values):
    if not values:
        return "--", "--", "--", "--"
    return (
        f"{statistics.mean(values):.2f}",
        f"{statistics.pstdev(values):.2f}",
        f"{min(values):.2f}",
        f"{max(values):.2f}",
    )


def report(results: dict, scenario: str, samples: int, obstacle_speed: float, robot_speed: float) -> None:
    print(f"\n=== {scenario} ===")
    print(f"  n = {samples} samples (random obstacle phases), obstacles {obstacle_speed:.1f} m/s, "
          f"robot {robot_speed:.1f} m/s, tick {CONTROL_TICK_S * 1000:.0f} ms")
    print(f"  trigger: their geometric rule, obstacle within {INVALIDATION_MM:.0f} mm of the path\n")
    header = (
        f"{'planner':<26}{'arrived':>9}{'recalc mean':>13}{'sd':>7}{'min':>6}{'max':>6}"
        f"{'accum ms mean':>15}{'nav s mean':>12}{'collisions mean':>17}"
    )
    print("  " + header)
    print("  " + "-" * len(header))
    for name in PLANNER_NAMES:
        rs = results[name]
        arrived = sum(r.arrived for r in rs)
        rm, rsd, rmin, rmax = _stats([r.recalculated for r in rs])
        am = statistics.mean(r.accumulated_ms for r in rs)
        nm = statistics.mean(r.navigation_s for r in rs if r.arrived) if arrived else float("nan")
        cm = statistics.mean(r.collisions for r in rs)
        print(
            f"  {name:<26}{f'{arrived}/{len(rs)}':>9}{rm:>13}{rsd:>7}{rmin:>6}{rmax:>6}"
            f"{am:>15.2f}{nm:>12.2f}{cm:>17.2f}"
        )
    print(
        "\n  Their Table 9, scenario 1, paths recalculated: DVG+A* mean 13.09 (sd 5.05, min 5, max 48),"
        "\n  RRT mean 34.08 (sd 9.60, min 10, max 61). Scenario 2: DVG+A* 5.79, RRT 6.93."
        "\n  Their Table 8, accumulated ms, scenario 1: DVG+A* 6.80, RRT 7.12."
        "\n  recalc counts the initial plan, as theirs does (a path 'had to be generated')."
    )


def plot(loops, out: str, obstacle_speed: float, robot_speed: float, planner: str = "VisibilityGraph+Dijkstra") -> None:
    """Their Fig 15, on our field, with the robot's actual trajectory drawn."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    SURFACE, INK, INK_2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#d8d7d3"
    LOOP_COLOURS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7"]
    RED_LINE = "#e34948"

    rng = random.Random(0)
    phases = [rng.uniform(0, loop.length()) for loop in loops]
    run = simulate(planner, loops, obstacle_speed_mps=obstacle_speed,
                   robot_speed_mps=robot_speed, phases=phases, keep_traces=True)

    fig, ax = plt.subplots(figsize=(12, 8))
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)
    ax.add_patch(Rectangle((-4500, -3000), 9000, 6000, fill=False, edgecolor=GRID, linewidth=1.2))
    ax.plot([0, 0], [-3000, 3000], color=GRID, linewidth=1.0)
    ax.add_patch(Circle((0, 0), 500, fill=False, edgecolor=GRID, linewidth=1.0))
    ax.add_patch(Rectangle((4500 - 1000, -1000), 1000, 2000, fill=False, edgecolor=GRID, linewidth=1.0))

    for i, loop in enumerate(loops):
        pts = (*loop.corners, loop.corners[0])
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                color=LOOP_COLOURS[i % len(LOOP_COLOURS)], linewidth=1.6, zorder=2)
        # final obstacle position, so the reader sees a robot, not just a loop
        if run.obstacle_traces and run.obstacle_traces[i]:
            ax.add_patch(Circle(run.obstacle_traces[i][-1], ROBOT_RADIUS_MM,
                                color=LOOP_COLOURS[i % len(LOOP_COLOURS)], zorder=4))

    ax.plot([START_MM[0], GOAL_MM[0]], [START_MM[1], GOAL_MM[1]],
            color=RED_LINE, linewidth=2.4, zorder=3)
    ax.scatter(*START_MM, s=90, color=INK, zorder=6)
    ax.scatter(*GOAL_MM, s=180, marker="*", color=INK, zorder=6)

    if run.trajectory:
        ax.plot([p[0] for p in run.trajectory], [p[1] for p in run.trajectory],
                color=INK, linewidth=1.4, linestyle=(0, (4, 3)), zorder=5)

    ax.set_xlim(-4700, 4700)
    ax.set_ylim(-3200, 3200)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(
        f"Dynamic test, {len(loops)} moving obstacles, after da Silva Costa & Tonidandel Fig. "
        f"{'15' if len(loops) == 7 else '16'}",
        color=INK, fontsize=13, fontweight="bold", loc="left", pad=12,
    )
    fig.text(
        0.5, 0.04,
        f"red = the robot's start-goal line  ·  coloured loops = obstacle patrol routes at "
        f"{obstacle_speed:.1f} m/s  ·  dashed black = the robot's actual trajectory ({planner})\n"
        f"this run: {run.recalculated} plans, {run.collisions} collision episodes, "
        f"{run.navigation_s:.1f} s to {'arrive' if run.arrived else 'time out'}, "
        f"{run.accumulated_ms:.1f} ms total planning",
        color=INK_2, fontsize=9.5, ha="center",
    )
    fig.savefig(out, dpi=160, facecolor=SURFACE, bbox_inches="tight")
    print(f"wrote {out}")


def selftest() -> None:
    loop = _rect(0, 0, 1000, 500)
    assert abs(loop.length() - 3000.0) < 1e-9
    assert loop.position(0) == (0.0, 0.0)
    assert loop.position(1000) == (1000.0, 0.0)
    assert loop.position(1500) == (1000.0, 500.0)
    assert loop.position(3000) == (0.0, 0.0), "must wrap"
    # Every scenario loop must straddle the red line, or it never crosses the path.
    for name, loops in SCENARIOS.items():
        for lp in loops:
            ys = [c[1] for c in lp.corners]
            assert min(ys) < 0 < max(ys), f"{name}: a loop never crosses y=0"
    # Trigger geometry
    path = ((0.0, 0.0), (1000.0, 0.0))

    def ctx(obs, prev=None, ticks=0):
        hist = (prev, obs) if prev is not None else (obs,)
        return TriggerContext(remaining_path=path, obstacles=obs, history=hist,
                              ticks_since_plan=ticks, dt_s=CONTROL_TICK_S)

    near = (Obstacle(pos_mm=(500.0, 50.0), robot_id=0),)
    far = (Obstacle(pos_mm=(500.0, 500.0), robot_id=0),)
    assert trigger_geometric(ctx(near))
    assert not trigger_geometric(ctx(far))
    assert trigger_periodic(4)(ctx(far, ticks=4)) and not trigger_periodic(4)(ctx(far, ticks=3))
    # Predictive: an obstacle 500 mm off the path, closing at 2 m/s, reaches it
    # in 0.25 s -- inside a 0.5 s horizon, outside a 0.1 s one.
    approaching_prev = (Obstacle(pos_mm=(500.0, 600.0), robot_id=0),)
    assert trigger_predictive(0.5)(ctx(far, prev=approaching_prev))
    assert not trigger_predictive(0.1)(ctx(far, prev=approaching_prev))
    # Near-field: obstacle on the SECOND segment must not trigger a 1-segment check.
    two_seg = ((0.0, 0.0), (1000.0, 0.0), (2000.0, 0.0))
    ctx2 = TriggerContext(remaining_path=two_seg, obstacles=(Obstacle(pos_mm=(1500.0, 50.0), robot_id=0),),
                          history=((Obstacle(pos_mm=(1500.0, 50.0), robot_id=0),),),
                          ticks_since_plan=0, dt_s=CONTROL_TICK_S)
    assert not trigger_near_field(1)(ctx2) and trigger_near_field(2)(ctx2)
    # The speed sweep addresses triggers by index; a reorder of TRIGGERS must
    # fail here rather than silently sweep the wrong three.
    assert [TRIGGERS[i].__name__ for i in SPEED_SWEEP_TRIGGER_IDX] == [
        "trigger_geometric", "predictive_0.25s", "predictive_0.25s_smooth5", "near_field_2seg"
    ], [TRIGGERS[i].__name__ for i in SPEED_SWEEP_TRIGGER_IDX]

    # Smoothed velocity must reject jitter that the one-tick difference reports
    # as motion. A stationary obstacle sampled with +/-25 mm of noise looks like
    # it is moving at metres per second over one tick, and like almost nothing
    # over five.
    still = [Obstacle(pos_mm=(500.0, 500.0 + j), robot_id=0)
             for j in (0.0, 25.0, -25.0, 20.0, -20.0, 0.0)]
    jitter_ctx = TriggerContext(
        remaining_path=path, obstacles=(still[-1],), history=tuple((o,) for o in still),
        ticks_since_plan=0, dt_s=CONTROL_TICK_S,
    )
    v1 = jitter_ctx.velocity_of(still[-1], 1)
    v5 = jitter_ctx.velocity_of(still[-1], 5)
    assert abs(v1[1]) > abs(v5[1]) * 3, (v1, v5)
    # Paired-CI helper: identical samples give a zero difference with a zero-width interval.
    m, lo, hi = paired_ci([1.0, 2.0, 3.0], [1.0, 2.0, 3.0])
    assert m == 0.0 and lo == 0.0 and hi == 0.0
    # A run with no obstacles must arrive, plan exactly once, and never collide.
    r = simulate("VisibilityGraph+Dijkstra", (), obstacle_speed_mps=1.0,
                 robot_speed_mps=1.5, phases=[])
    assert r.arrived and r.recalculated == 1 and r.collisions == 0, r
    expected_s = math.dist(START_MM, GOAL_MM) / 1500.0
    assert abs(r.navigation_s - expected_s) < 0.2, (r.navigation_s, expected_s)
    print("selftest OK")


def _run_one(job: tuple) -> tuple[int, str, int, dict]:
    """Worker: one (trigger, planner, sample). Module-level so it pickles.

    Triggers are closures and do not pickle, so the job carries the trigger's
    INDEX into TRIGGERS and the worker rebuilds the same list on import.
    """
    trig_idx, planner, sample_idx, scenario_key, obstacle_speed, robot_speed, phases, noise_mm = job
    r = simulate(
        planner, SCENARIOS[scenario_key], obstacle_speed_mps=obstacle_speed,
        robot_speed_mps=robot_speed, phases=phases, trigger=TRIGGERS[trig_idx],
        noise_mm=noise_mm, noise_seed=sample_idx,
    )
    return trig_idx, planner, sample_idx, {
        "arrived": r.arrived, "navigation_s": r.navigation_s,
        "accumulated_ms": r.accumulated_ms, "recalculated": r.recalculated,
        "collisions": r.collisions,
        "replans_compared": r.replans_compared, "reversals": r.reversals,
        "mean_heading_deg": r.mean_heading_deg, "mean_shift_mm": r.mean_shift_mm,
        "p95_shift_mm": r.p95_shift_mm,
    }


def sweep_triggers(
    scenario_key: str, samples: int, obstacle_speed: float, robot_speed: float,
    seed: int = 0, workers: int = 1, noise_mm: float = 0.0,
) -> dict[tuple[str, str], list[RunResult]]:
    """{(trigger name, planner): [RunResult]} -- every trigger sees the SAME phases.

    Paired by construction: phase list k is shared across all triggers and all
    planners, so a difference between two rows is the trigger, not the draw.
    Results come back in sample order regardless of which worker ran them, so
    pairing survives parallel execution.
    """
    loops = SCENARIOS[scenario_key]
    rng = random.Random(seed)
    phase_sets = [[rng.uniform(0, loop.length()) for loop in loops] for _ in range(samples)]
    jobs = [
        (ti, name, si, scenario_key, obstacle_speed, robot_speed, ph, noise_mm)
        for ti in range(len(TRIGGERS))
        for name in PLANNER_NAMES
        for si, ph in enumerate(phase_sets)
    ]

    slots: dict[tuple[str, str], list] = {
        (t.__name__, n): [None] * samples for t in TRIGGERS for n in PLANNER_NAMES
    }
    if workers > 1:
        from concurrent.futures import ProcessPoolExecutor

        with ProcessPoolExecutor(max_workers=workers) as pool:
            for ti, name, si, d in pool.map(_run_one, jobs, chunksize=8):
                slots[(TRIGGERS[ti].__name__, name)][si] = RunResult(planner=name, **d)
    else:
        for job in jobs:
            ti, name, si, d = _run_one(job)
            slots[(TRIGGERS[ti].__name__, name)][si] = RunResult(planner=name, **d)
    return slots


# Which triggers ride along in the speed sweep: theirs, plus the two that won
# on safety and on cost at 1.0 m/s. Indices into TRIGGERS; a change to that
# list has to be reflected here.
# trigger_geometric, predictive_0.25s, predictive_0.25s_smooth5, near_field_2seg
SPEED_SWEEP_TRIGGER_IDX = (0, 5, 8, 10)


def sweep_speeds(
    scenario_key: str, speeds, samples: int, robot_speed: float,
    seed: int = 0, workers: int = 1, noise_mm: float = 0.0,
) -> dict[tuple[float, str, str], list[RunResult]]:
    """{(obstacle speed, trigger name, planner): [RunResult]}.

    da Silva Costa & Tonidandel never state their obstacle speed, so their
    Table 9 is one unlabelled point on this axis. Sweeping it shows WHERE each
    planner's replan count takes off, which is a curve and much harder to
    argue with than a number. Paired across speeds as well as triggers: phase
    draw k is identical at every speed, so the curve's shape is the speed's
    doing.
    """
    loops = SCENARIOS[scenario_key]
    rng = random.Random(seed)
    phase_sets = [[rng.uniform(0, loop.length()) for loop in loops] for _ in range(samples)]
    jobs = [
        (ti, name, si, scenario_key, float(v), robot_speed, ph, noise_mm)
        for v in speeds
        for ti in SPEED_SWEEP_TRIGGER_IDX
        for name in PLANNER_NAMES
        for si, ph in enumerate(phase_sets)
    ]
    slots: dict[tuple[float, str, str], list] = {
        (float(v), TRIGGERS[ti].__name__, n): [None] * samples
        for v in speeds for ti in SPEED_SWEEP_TRIGGER_IDX for n in PLANNER_NAMES
    }

    def _store(ti, name, si, d, v):
        slots[(v, TRIGGERS[ti].__name__, name)][si] = RunResult(planner=name, **d)

    if workers > 1:
        from concurrent.futures import ProcessPoolExecutor

        with ProcessPoolExecutor(max_workers=workers) as pool:
            for job, (ti, name, si, d) in zip(jobs, pool.map(_run_one, jobs, chunksize=8)):
                _store(ti, name, si, d, job[4])
    else:
        for job in jobs:
            ti, name, si, d = _run_one(job)
            _store(ti, name, si, d, job[4])
    return slots


def report_speeds(results: dict, scenario: str, samples: int, robot_speed: float) -> None:
    speeds = sorted({k[0] for k in results})
    trig_names = [TRIGGERS[i].__name__ for i in SPEED_SWEEP_TRIGGER_IDX]
    print(f"\n=== OBSTACLE SPEED SWEEP: {scenario} ===")
    print(f"  n = {samples} paired samples per cell, robot {robot_speed:.1f} m/s, "
          f"tick {CONTROL_TICK_S * 1000:.0f} ms. Same phase draws at every speed.")
    for name in PLANNER_NAMES:
        print(f"\n  -- {name} --")
        header = f"{'obst m/s':>10}" + "".join(
            f"{t.removeprefix('trigger_'):>26}" for t in trig_names
        )
        print("  " + header)
        print("  " + " " * 10 + "".join(f"{'recalc  collide  arrive':>26}" for _ in trig_names))
        print("  " + "-" * len(header))
        for v in speeds:
            row = f"{v:>10.1f}"
            for t in trig_names:
                rs = results[(v, t, name)]
                rec = statistics.mean(r.recalculated for r in rs)
                col = statistics.mean(r.collisions for r in rs)
                arr = sum(r.arrived for r in rs)
                row += f"{rec:>10.2f}{col:>8.2f}{f'{arr}/{len(rs)}':>8}"
            print("  " + row)
    print(
        "\n  recalc = paths recalculated, mean per run; collide = collision episodes, mean per run;"
        "\n  arrive = runs reaching the goal inside the 30 s limit. Their Table 9 is one point on"
        "\n  the geometric column at an unstated speed."
    )


def plot_speeds(results: dict, out: str, scenario: str) -> str:
    """Replans and collisions against obstacle speed. Two rows, one per metric,
    one column per planner, one line per trigger. No dual axis: the metrics
    have different units, so they get different panels."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    SURFACE, INK, INK_2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e3e2df"
    COLOURS = {"trigger_geometric": "#2a78d6", "predictive_0.25s": "#eb6834",
               "predictive_0.25s_smooth5": "#eda100", "near_field_2seg": "#1baf7a"}
    LABELS = {"trigger_geometric": "geometric (theirs)", "predictive_0.25s": "predictive 0.25 s",
              "predictive_0.25s_smooth5": "predictive 0.25 s, smoothed",
              "near_field_2seg": "near-field 2 seg"}
    speeds = sorted({k[0] for k in results})
    n = len(next(iter(results.values())))

    # sharey per ROW: a reader compares planners left to right, and three
    # different y-ranges hide that Voronoi replans a fifth as often as the
    # visibility graph. Rows keep separate ranges because replans and
    # collisions are different units.
    fig, axes = plt.subplots(2, 3, figsize=(16, 8.5), sharex=True, sharey="row")
    fig.patch.set_facecolor(SURFACE)
    for col, name in enumerate(PLANNER_NAMES):
        for row, (metric, label) in enumerate((("recalculated", "paths recalculated, mean per run"),
                                               ("collisions", "collision episodes, mean per run"))):
            ax = axes[row][col]
            ax.set_facecolor(SURFACE)
            ax.grid(True, color=GRID, linewidth=0.8, zorder=0)
            ax.set_axisbelow(True)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            for side in ("left", "bottom"):
                ax.spines[side].set_color(GRID)
            ax.tick_params(colors=INK_2, labelsize=9)
            for t, colour in COLOURS.items():
                ys = [statistics.mean(getattr(r, metric) for r in results[(v, t, name)]) for v in speeds]
                ax.plot(speeds, ys, color=colour, linewidth=2.0, marker="o", markersize=5, zorder=3)
                ax.annotate(LABELS[t], (speeds[-1], ys[-1]), xytext=(5, 0), textcoords="offset points",
                            color=colour, fontsize=8.5, va="center", fontweight="bold")
            if row == 0:
                ax.set_title(name.replace("+Dijkstra", ""), color=INK, fontsize=11.5,
                             fontweight="bold", loc="left")
            if col == 0:
                ax.set_ylabel(label, color=INK, fontsize=9.5)
            if row == 1:
                ax.set_xlabel("obstacle speed, m/s", color=INK, fontsize=9.5)
            ax.set_xticks(speeds)
    fig.suptitle(f"Replanning against obstacle speed, {scenario}, n = {n} paired samples per point",
                 color=INK, fontsize=13, fontweight="bold", x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0, 0.97, 0.95))
    fig.savefig(out, dpi=160, facecolor=SURFACE, bbox_inches="tight")
    plt.close(fig)
    return out


def save_speed_results(results: dict, path: str, meta: dict) -> None:
    import json

    payload = {"meta": meta, "results": {
        f"{v}|{t}|{p}": [r.__dict__ | {"trajectory": [], "obstacle_traces": []} for r in rs]
        for (v, t, p), rs in results.items()
    }}
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh)


def load_speed_results(path: str) -> tuple[dict, dict]:
    import json

    with open(path, encoding="utf-8") as fh:
        payload = json.load(fh)
    results = {}
    for key, rs in payload["results"].items():
        v, t, p = key.split("|", 2)
        results[(float(v), t, p)] = [RunResult(**r) for r in rs]
    return results, payload["meta"]


def save_results(results: dict, path: str, meta: dict) -> None:
    import json

    payload = {
        "meta": meta,
        "results": {
            f"{t}|{p}": [r.__dict__ | {"trajectory": [], "obstacle_traces": []} for r in rs]
            for (t, p), rs in results.items()
        },
    }
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh)


def load_results(path: str) -> tuple[dict, dict]:
    import json

    with open(path, encoding="utf-8") as fh:
        payload = json.load(fh)
    results = {}
    for key, rs in payload["results"].items():
        t, p = key.split("|", 1)
        results[(t, p)] = [RunResult(**r) for r in rs]
    return results, payload["meta"]


def paired_ci(a: list[float], b: list[float], boots: int = 2000, seed: int = 0) -> tuple[float, float, float]:
    """Mean of (a - b) with a 95% bootstrap CI over the PAIRED differences.

    The pairing is the point: resampling differences, not the two samples
    separately, keeps each obstacle-phase draw matched between the two
    triggers, which is what makes the interval about the trigger rather than
    the draw.
    """
    diffs = [x - y for x, y in zip(a, b)]
    rng = random.Random(seed)
    n = len(diffs)
    means = sorted(statistics.mean(rng.choice(diffs) for _ in range(n)) for _ in range(boots))
    return statistics.mean(diffs), means[int(0.025 * boots)], means[int(0.975 * boots)]


def report_sweep(results: dict, scenario: str, samples: int, obstacle_speed: float, robot_speed: float) -> None:
    print(f"\n=== TRIGGER SWEEP: {scenario} ===")
    print(f"  n = {samples} paired samples, obstacles {obstacle_speed:.1f} m/s, robot {robot_speed:.1f} m/s, "
          f"tick {CONTROL_TICK_S * 1000:.0f} ms. Same obstacle phases for every row.")
    baseline = TRIGGERS[0].__name__  # trigger_geometric, theirs
    for name in PLANNER_NAMES:
        print(f"\n  -- {name} --")
        header = (
            f"{'trigger':<28}{'arrived':>9}{'recalc':>8}{'sd':>7}"
            f"{'d recalc vs theirs [95% CI]':>30}"
            f"{'collide':>9}{'d collide [95% CI]':>24}{'nav s':>8}{'ms/tick':>9}"
        )
        print("  " + header)
        print("  " + "-" * len(header))
        base = results[(baseline, name)]
        base_recalc = [r.recalculated for r in base]
        base_coll = [r.collisions for r in base]
        for trig in TRIGGERS:
            rs = results[(trig.__name__, name)]
            arrived = sum(r.arrived for r in rs)
            recalc = [r.recalculated for r in rs]
            coll = [r.collisions for r in rs]
            nav = [r.navigation_s for r in rs if r.arrived]
            acc = statistics.mean(r.accumulated_ms for r in rs)
            nav_m = statistics.mean(nav) if nav else float("nan")
            # Planning cost per control tick actually driven: the number that
            # has to fit inside the 50 ms budget.
            ticks = statistics.mean(r.navigation_s / CONTROL_TICK_S for r in rs)
            if trig.__name__ == baseline:
                d_recalc, d_coll = f"{'(baseline)':>30}", f"{'':>24}"
            else:
                m, lo, hi = paired_ci(recalc, base_recalc)
                d_recalc = f"{m:+7.2f}  [{lo:+6.2f}, {hi:+6.2f}]".rjust(30)
                m, lo, hi = paired_ci(coll, base_coll)
                d_coll = f"{m:+5.2f} [{lo:+5.2f},{hi:+5.2f}]".rjust(24)
            print(
                f"  {trig.__name__:<28}{f'{arrived}/{len(rs)}':>9}"
                f"{statistics.mean(recalc):>8.2f}{statistics.pstdev(recalc):>7.2f}"
                f"{d_recalc}"
                f"{statistics.mean(coll):>9.2f}{d_coll}{nav_m:>8.2f}"
                f"{acc / ticks if ticks else float('nan'):>9.2f}"
            )
    print(
        "\n  geometric        = theirs: obstacle within 90 mm of the remaining path (Table 9 baseline)"
        "\n  periodic_Nms     = replan every N ms regardless; blind control that any smart trigger must beat"
        "\n  predictive_Ts    = geometric OR an obstacle projected along its velocity reaches the path within T"
        "\n  near_field_Kseg  = geometric over only the next K path segments (commitment depth)"
        "\n  ms/tick          = accumulated planning ms divided by control ticks driven; the 50 ms budget line"
    )


def plot_sweep(results: dict, out: str, scenario: str) -> str:
    """Replans vs collisions, one point per trigger, one panel per planner.

    The claim is a trade-off ('fewer replans without more collisions'), so the
    form is a scatter with both axes in the reader's units, per Moll et al. and
    the dataviz rules: no dual axis, direct labels on every point, n in the
    title. Families are separated by marker shape as well as colour because the
    fourth categorical slot (yellow) does not clear the all-pairs gate against
    orange.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    SURFACE, INK, INK_2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e3e2df"
    FAMILY = {
        "geometric": ("#2a78d6", "o"),
        "periodic": ("#eb6834", "s"),
        "predictive": ("#1baf7a", "^"),
        "near_field": ("#eda100", "D"),
    }

    def family_of(name: str) -> str:
        # The plain function is named `trigger_geometric`; the factories name
        # theirs without the prefix. Strip it so both map to a family.
        bare = name.removeprefix("trigger_")
        return next(f for f in FAMILY if bare.startswith(f))

    # Shared x as well as y: with per-panel x-ranges, Voronoi's 559-replan
    # periodic_50ms point stretched its axis and made the other triggers look
    # clustered at zero, while the visibility graph's 58 looked comparably
    # far right. Same axes, same meaning.
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.4), sharex=True, sharey=True)
    fig.patch.set_facecolor(SURFACE)
    n = len(next(iter(results.values())))
    for ax, name in zip(axes, PLANNER_NAMES):
        ax.set_facecolor(SURFACE)
        ax.grid(True, color=GRID, linewidth=0.8, zorder=0)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(GRID)
        ax.tick_params(colors=INK_2, labelsize=9)
        for trig in TRIGGERS:
            rs = results[(trig.__name__, name)]
            x = statistics.mean(r.recalculated for r in rs)
            y = statistics.mean(r.collisions for r in rs)
            colour, marker = FAMILY[family_of(trig.__name__)]
            ax.scatter(x, y, s=64, color=colour, marker=marker, zorder=3,
                       edgecolors=SURFACE, linewidths=1.2)
            ax.annotate(trig.__name__.replace("geometric+periodic", "geo+per"),
                        (x, y), xytext=(5, 4), textcoords="offset points",
                        color=INK_2, fontsize=7.5)
        ax.set_title(name.replace("+Dijkstra", ""), color=INK, fontsize=11.5,
                     fontweight="bold", loc="left")
        ax.set_xlabel("paths recalculated, mean per run", color=INK, fontsize=9.5)
    axes[0].set_ylabel("collision episodes, mean per run", color=INK, fontsize=9.5)
    fig.suptitle(
        f"Replan trigger trade-off, {scenario}, n = {n} paired samples per point",
        color=INK, fontsize=13, fontweight="bold", x=0.01, ha="left",
    )
    fig.text(0.01, 0.01,
             "circle = geometric (theirs)   square = periodic   triangle = predictive   "
             "diamond = near-field.   Lower-left is better on both axes.",
             color=INK_2, fontsize=8.5)
    fig.tight_layout(rect=(0, 0.04, 1, 0.94))
    fig.savefig(out, dpi=160, facecolor=SURFACE, bbox_inches="tight")
    plt.close(fig)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--obstacle-speed", type=float, default=1.0, help="m/s; the paper does not state theirs")
    parser.add_argument("--robot-speed", type=float, default=1.5, help="m/s")
    parser.add_argument("--plot", metavar="PATH", help="Draw one run's trajectory (see --scenario)")
    parser.add_argument("--scenario", type=int, choices=(1, 2), default=1,
                        help="With --plot: which scenario to draw (default: 1)")
    parser.add_argument("--sweep-triggers", action="store_true",
                        help="Compare every trigger in TRIGGERS, paired, on both scenarios")
    parser.add_argument("--sweep-plot", metavar="PATH", help="With --sweep-triggers: write the trade-off figure")
    parser.add_argument("--workers", type=int, default=1,
                        help="Process pool size. Simulations are independent; 20 cuts n=200 from ~1 h to minutes")
    parser.add_argument("--out-json", metavar="PATH",
                        help="Persist sweep results (one file per scenario, suffixed) so plots never force a rerun")
    parser.add_argument("--from-json", metavar="PATH", nargs="+",
                        help="Re-report and re-plot from saved results instead of simulating")
    parser.add_argument("--sweep-speeds", type=float, nargs="+", metavar="MPS",
                        help="Obstacle-speed sweep on scenario 1 with their trigger plus the two best; "
                             "e.g. --sweep-speeds 0.5 1.0 1.5 2.0 2.5 3.0")
    parser.add_argument("--speed-plot", metavar="PATH", help="With --sweep-speeds: write the figure")
    parser.add_argument("--speed-scenario", choices=list(SCENARIOS), default=None,
                        help="Which scenario the speed sweep runs on (default: scenario_1)")
    parser.add_argument("--noise-mm", type=float, default=0.0,
                        help="Per-frame Gaussian position noise the TRIGGER sees, mm. "
                             "The measured vision noise floor on a real match log is 23 mm. "
                             "Collisions always use true positions.")
    parser.add_argument("--from-speed-json", metavar="PATH", help="Re-report and re-plot a saved speed sweep")
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()

    if args.selftest:
        selftest()
        return
    if args.plot:
        scenario = SCENARIO_1 if args.scenario == 1 else SCENARIO_2
        plot(scenario, args.plot, args.obstacle_speed, args.robot_speed)
        return

    if args.from_speed_json:
        results, meta = load_speed_results(args.from_speed_json)
        report_speeds(results, meta["scenario"], meta["samples"], meta["robot_speed"])
        if args.speed_plot:
            print(f"\n  figure: {plot_speeds(results, args.speed_plot, meta['scenario'])}")
        return

    if args.sweep_speeds:
        scenario = args.speed_scenario or next(iter(SCENARIOS))
        results = sweep_speeds(scenario, args.sweep_speeds, args.samples, args.robot_speed,
                               workers=args.workers, noise_mm=args.noise_mm)
        if args.out_json:
            save_speed_results(results, args.out_json, {
                "scenario": scenario, "samples": args.samples, "robot_speed": args.robot_speed,
                "speeds": list(args.sweep_speeds), "noise_mm": args.noise_mm,
            })
            print(f"\n  saved {args.out_json}")
        report_speeds(results, scenario, args.samples, args.robot_speed)
        if args.speed_plot:
            print(f"\n  figure: {plot_speeds(results, args.speed_plot, scenario)}")
        return

    if args.from_json:
        for path in args.from_json:
            results, meta = load_results(path)
            report_sweep(results, meta["scenario"], meta["samples"], meta["obstacle_speed"], meta["robot_speed"])
            if args.sweep_plot and meta["scenario"].startswith("scenario_1"):
                print(f"\n  figure: {plot_sweep(results, args.sweep_plot, meta['scenario'])}")
        return

    if args.sweep_triggers:
        for scenario in SCENARIOS:
            results = sweep_triggers(scenario, args.samples, args.obstacle_speed, args.robot_speed,
                                     workers=args.workers, noise_mm=args.noise_mm)
            if args.out_json:
                stem = Path(args.out_json)
                path = stem.with_name(f"{stem.stem}_{scenario.split(' ')[0]}{stem.suffix or '.json'}")
                save_results(results, str(path), {
                    "scenario": scenario, "samples": args.samples,
                    "obstacle_speed": args.obstacle_speed, "robot_speed": args.robot_speed,
                })
                print(f"\n  saved {path}")
            report_sweep(results, scenario, args.samples, args.obstacle_speed, args.robot_speed)
            if args.sweep_plot and scenario.startswith("scenario_1"):
                print(f"\n  figure: {plot_sweep(results, args.sweep_plot, scenario)}")
        return

    for scenario, loops in SCENARIOS.items():
        results = run_all_samples(loops, args.samples, args.obstacle_speed, args.robot_speed)
        report(results, scenario, args.samples, args.obstacle_speed, args.robot_speed)


if __name__ == "__main__":
    main()
