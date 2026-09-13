#!/usr/bin/env python3
"""How much does a replan change the plan?

Every "the robot oscillates between routes" sentence in the draft is currently
a story. This turns it into two numbers, measured at each replan by comparing
the plan the robot was following against the plan that replaced it.

Both plans start at the robot's current position, so they are directly
comparable. Two things are worth measuring and they answer different questions.

  heading change    The angle between the direction the old plan pointed and
                    the direction the new one points, both measured from where
                    the robot is standing. This is what the robot is commanded
                    to do in the next tick. Above 90 degrees the new plan sends
                    it back the way it came, which is the oscillation.

  corridor shift    The mean distance between the two paths along their whole
                    length. This says whether the route changed, as opposed to
                    only its first step. A large heading change with a small
                    corridor shift is a wobble; large in both is a genuinely
                    different route.

Both read 0 when the new plan is the old plan, which is the value that means
nothing happened. Both are in the units the reader already has, degrees and
millimetres.

Usage:
    python scripts/path_stability.py --selftest
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import pairwise

Point = tuple[float, float]
Path = tuple[Point, ...]

# Above this the new plan points back the way the robot came. 90 degrees is the
# natural cut: the component of the new direction along the old one turns
# negative, so the robot is being asked to undo travel it has already done.
REVERSAL_DEG = 90.0


def point_to_polyline_mm(point: Point, path) -> float:
    """Shortest distance from `point` to the polyline `path`.

    Returns inf for a path with fewer than two points, which has no segments to
    measure against. Callers must treat that as "no answer", not as "far away".
    """
    if len(path) < 2:
        return float("inf")
    px, py = point
    best = float("inf")
    for (x0, y0), (x1, y1) in pairwise(path):
        dx, dy = x1 - x0, y1 - y0
        seg_sq = dx * dx + dy * dy
        t = 0.0 if seg_sq == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / seg_sq))
        best = min(best, math.hypot(px - (x0 + t * dx), py - (y0 + t * dy)))
    return best


def resample(path: Path, count: int) -> list[Point]:
    """`count` points spaced evenly along `path` by arc length.

    Sampling by arc length rather than by waypoint is what lets two plans with
    different waypoint counts be compared at all. A Voronoi plan with 3
    waypoints and a visibility-graph plan with 11 both become `count` points.
    """
    if count < 2 or len(path) < 2:
        return list(path)
    spans = [math.dist(a, b) for a, b in pairwise(path)]
    total = sum(spans)
    if total <= 0.0:
        return [path[0]] * count

    out: list[Point] = []
    for i in range(count):
        want = total * i / (count - 1)
        travelled = 0.0
        for (a, b), span in zip(pairwise(path), spans):
            if travelled + span >= want or span == 0.0:
                f = 0.0 if span == 0.0 else (want - travelled) / span
                out.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
                break
            travelled += span
        else:
            out.append(path[-1])
    return out


def corridor_shift_mm(old: Path, new: Path, samples: int = 32) -> float:
    """Mean distance between two paths, in mm. 0 means they coincide.

    Measured in both directions and averaged. One direction alone would report
    0 for a short new path that lies on top of a long old one, which is not
    what a reader means by "the route did not change".
    """
    if len(old) < 2 or len(new) < 2:
        return float("nan")
    a = [point_to_polyline_mm(p, old) for p in resample(new, samples)]
    b = [point_to_polyline_mm(p, new) for p in resample(old, samples)]
    return (sum(a) / len(a) + sum(b) / len(b)) / 2.0


def heading_change_deg(old: Path, new: Path) -> float:
    """Angle in degrees between the first step of each plan.

    Both plans are expected to start at the robot's position, so the first step
    is the direction the robot is about to be driven. Returns nan when either
    plan has no first step to take.
    """
    if len(old) < 2 or len(new) < 2:
        return float("nan")
    ax, ay = old[1][0] - old[0][0], old[1][1] - old[0][1]
    bx, by = new[1][0] - new[0][0], new[1][1] - new[0][1]
    na, nb = math.hypot(ax, ay), math.hypot(bx, by)
    if na == 0.0 or nb == 0.0:
        return float("nan")
    cos = max(-1.0, min(1.0, (ax * bx + ay * by) / (na * nb)))
    return math.degrees(math.acos(cos))


@dataclass
class Stability:
    """Plan-to-plan change over one run. All fields read 0 for a robot that
    never changed its mind."""

    replans_compared: int = 0
    reversals: int = 0
    heading_sum: float = 0.0
    shift_sum: float = 0.0
    shifts: list = None

    def __post_init__(self):
        if self.shifts is None:
            self.shifts = []

    def observe(self, old: Path, new: Path) -> None:
        """Record one replan. Pairs where either plan is a single point are
        skipped rather than counted as zero change: a planner that failed did
        not produce a stable plan, it produced no plan."""
        turn = heading_change_deg(old, new)
        shift = corridor_shift_mm(old, new)
        if math.isnan(turn) or math.isnan(shift):
            return
        self.replans_compared += 1
        self.heading_sum += turn
        self.shift_sum += shift
        self.shifts.append(shift)
        if turn > REVERSAL_DEG:
            self.reversals += 1

    @property
    def mean_heading_deg(self) -> float:
        n = self.replans_compared
        return self.heading_sum / n if n else 0.0

    @property
    def mean_shift_mm(self) -> float:
        n = self.replans_compared
        return self.shift_sum / n if n else 0.0

    @property
    def p95_shift_mm(self) -> float:
        if not self.shifts:
            return 0.0
        ordered = sorted(self.shifts)
        return ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))]

    @property
    def reversal_rate(self) -> float:
        n = self.replans_compared
        return self.reversals / n if n else 0.0

    def as_dict(self) -> dict:
        return {
            "replans_compared": self.replans_compared,
            "reversals": self.reversals,
            "reversal_rate": self.reversal_rate,
            "mean_heading_deg": self.mean_heading_deg,
            "mean_shift_mm": self.mean_shift_mm,
            "p95_shift_mm": self.p95_shift_mm,
        }


def selftest() -> int:
    """Pin the readings the metric is worthless without."""
    failures = []

    def check(name, got, want, tol=1e-6):
        if math.isnan(got) or abs(got - want) > tol:
            failures.append(f"{name}: got {got}, want {want}")

    straight = ((0.0, 0.0), (1000.0, 0.0), (2000.0, 0.0))

    # An unchanged plan must read zero on both metrics. This is the anchor
    # every other number is read against.
    check("identical heading", heading_change_deg(straight, straight), 0.0)
    check("identical shift", corridor_shift_mm(straight, straight), 0.0)

    # Resampling must not, by itself, move the path. Same route described with
    # different waypoints reads as no change.
    coarse = ((0.0, 0.0), (2000.0, 0.0))
    check("same route, different waypoints", corridor_shift_mm(straight, coarse), 0.0, 1e-6)

    # A plan that turns the robot around is the oscillation this exists to see.
    back = ((0.0, 0.0), (-1000.0, 0.0))
    check("reversal angle", heading_change_deg(straight, back), 180.0, 1e-6)

    # A parallel detour 500 mm to the side shifts by 500 mm everywhere except
    # the shared start, so the mean is a little under 500.
    detour = ((0.0, 0.0), (0.0, 500.0), (2000.0, 500.0))
    shift = corridor_shift_mm(straight, detour)
    if not (200.0 < shift < 500.0):
        failures.append(f"parallel detour shift: got {shift:.1f}, want between 200 and 500")

    # Turning without changing the route, and changing the route without
    # turning, must be distinguishable. That is why there are two numbers.
    wobble = ((0.0, 0.0), (600.0, 300.0), (2000.0, 0.0))
    if heading_change_deg(straight, wobble) < 20.0:
        failures.append("wobble should register as a heading change")
    if corridor_shift_mm(straight, wobble) > 200.0:
        failures.append("wobble should not register as a large corridor shift")

    # A failed plan contributes nothing rather than a spurious zero.
    s = Stability()
    s.observe(straight, ((0.0, 0.0),))
    if s.replans_compared != 0:
        failures.append("a plan with no second waypoint must not be counted")

    s = Stability()
    s.observe(straight, back)
    s.observe(straight, straight)
    check("reversal count", float(s.reversals), 1.0)
    check("reversal rate", s.reversal_rate, 0.5)

    for line in failures:
        print(f"FAIL {line}")
    if not failures:
        print("path_stability selftest OK")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(selftest())
