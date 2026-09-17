"""Seeded scenario generation for headless planner batches.

Two generators, both fully determined by ``(seed, index)`` so a batch can be
reproduced from its manifest alone:

* :func:`random_scenario` -- robots, targets and obstacles placed uniformly in
  the field with rejection sampling, a fraction of obstacles moving.
* :func:`perturb_scenario` -- jitter every position of a saved scenario while
  keeping its structure (IDs, teams, radii, velocities, planner layouts).
"""

from __future__ import annotations

import random
from dataclasses import dataclass, replace
from math import cos, hypot, pi, sin, sqrt

from research_sdk.config import ROBOT_RADIUS_MM
from research_sdk.ui.scenarios import Scenario, ScenarioObstacle, ScenarioRobot
from research_sdk.world.scene import FieldDimensions

Point = tuple[float, float]
MAX_ROBOTS_PER_TEAM = 16


@dataclass(frozen=True, slots=True)
class GeneratorConfig:
    robots: int = 3
    obstacles: int = 8
    moving_fraction: float = 0.25
    min_obstacle_speed_mmps: float = 300.0
    max_obstacle_speed_mmps: float = 1000.0
    min_travel_mm: float = 1500.0
    edge_margin_mm: float = 300.0
    min_gap_mm: float = 60.0
    jitter_mm: float = 150.0
    max_attempts: int = 2000

    def __post_init__(self) -> None:
        if self.robots < 1:
            raise ValueError("robots must be at least 1")
        if self.obstacles < 0:
            raise ValueError("obstacles must be non-negative")
        if self.robots + self.obstacles > 2 * MAX_ROBOTS_PER_TEAM:
            raise ValueError("robots + obstacles must fit in two 16-robot teams")
        if not 0.0 <= self.moving_fraction <= 1.0:
            raise ValueError("moving_fraction must be between 0 and 1")
        if not 0.0 <= self.min_obstacle_speed_mmps <= self.max_obstacle_speed_mmps:
            raise ValueError("obstacle speed range is invalid")
        if self.min_travel_mm < 0 or self.edge_margin_mm < 0 or self.jitter_mm < 0:
            raise ValueError("distances must be non-negative")


def _rng(seed: int, index: int, salt: int) -> random.Random:
    # Stable across processes and Python versions (no hash randomisation).
    return random.Random((seed * 1_000_003 + index) * 7919 + salt)


def _bounds(margin: float) -> tuple[float, float, float, float]:
    x_min, x_max, y_min, y_max = FieldDimensions().bounds_mm
    return x_min + margin, x_max - margin, y_min + margin, y_max - margin


def _sample(rng: random.Random, margin: float) -> Point:
    x_min, x_max, y_min, y_max = _bounds(margin)
    return (round(rng.uniform(x_min, x_max), 1), round(rng.uniform(y_min, y_max), 1))


def _clear(point: Point, others: list[Point], min_distance: float) -> bool:
    return all(hypot(point[0] - o[0], point[1] - o[1]) >= min_distance for o in others)


def _place(rng, margin, occupied, min_distance, attempts, extra=lambda p: True) -> Point:
    for _ in range(attempts):
        point = _sample(rng, margin)
        if _clear(point, occupied, min_distance) and extra(point):
            return point
    raise RuntimeError("Could not place an object; reduce density or margins")


def random_scenario(
    index: int, seed: int = 0, config: GeneratorConfig | None = None
) -> Scenario:
    """Return scenario ``index`` of the random set identified by ``seed``."""
    config = config or GeneratorConfig()
    rng = _rng(seed, index, 1)
    separation = 2.0 * ROBOT_RADIUS_MM + config.min_gap_mm
    margin = config.edge_margin_mm

    # Obstacles first (their initial positions), then robot starts, then targets.
    obstacle_points: list[Point] = []
    for _ in range(config.obstacles):
        obstacle_points.append(
            _place(rng, margin, obstacle_points, separation, config.max_attempts)
        )
    starts: list[Point] = []
    for _ in range(config.robots):
        starts.append(
            _place(rng, margin, obstacle_points + starts, separation, config.max_attempts)
        )
    targets: list[Point] = []
    for start in starts:
        targets.append(
            _place(
                rng,
                margin,
                obstacle_points + targets,
                separation,
                config.max_attempts,
                lambda p, s=start: hypot(p[0] - s[0], p[1] - s[1]) >= config.min_travel_mm,
            )
        )

    robots = [
        ScenarioRobot(robot_id=i, is_yellow=False, start_mm=starts[i], target_mm=targets[i])
        for i in range(config.robots)
    ]
    moving = round(config.obstacles * config.moving_fraction)
    moving_ids = set(rng.sample(range(config.obstacles), moving))
    obstacles = []
    for i, point in enumerate(obstacle_points):
        # Yellow IDs 0-15 first, then the blue IDs not used by robots.
        is_yellow = i < MAX_ROBOTS_PER_TEAM
        obstacle_id = i if is_yellow else config.robots + i - MAX_ROBOTS_PER_TEAM
        velocity = (0.0, 0.0)
        if i in moving_ids:
            speed = rng.uniform(config.min_obstacle_speed_mmps, config.max_obstacle_speed_mmps)
            heading = rng.uniform(-pi, pi)
            velocity = (round(speed * cos(heading), 1), round(speed * sin(heading), 1))
        obstacles.append(
            ScenarioObstacle(
                obstacle_id=obstacle_id,
                is_yellow=is_yellow,
                position_mm=point,
                radius_mm=ROBOT_RADIUS_MM,
                velocity_mmps=velocity,
            )
        )
    return Scenario(name=f"random-s{seed}-{index:04d}", robots=robots, obstacles=obstacles)


def _jitter(rng: random.Random, point: Point, radius: float, margin: float) -> Point:
    angle = rng.uniform(-pi, pi)
    distance = radius * sqrt(rng.random())
    x_min, x_max, y_min, y_max = _bounds(margin)
    return (
        round(min(x_max, max(x_min, point[0] + distance * cos(angle))), 1),
        round(min(y_max, max(y_min, point[1] + distance * sin(angle))), 1),
    )


def perturb_scenario(
    base: Scenario, index: int, seed: int = 0, config: GeneratorConfig | None = None
) -> Scenario:
    """Jitter every start, target and obstacle of ``base`` by up to ``jitter_mm``.

    Starts are kept clear of obstacles and of each other; if no valid sample is
    found for a start the original position is kept.
    """
    config = config or GeneratorConfig()
    salt = sum(ord(char) for char in base.name)
    rng = _rng(seed, index, salt)
    separation = 2.0 * ROBOT_RADIUS_MM + config.min_gap_mm
    margin = ROBOT_RADIUS_MM

    obstacles = [
        replace(o, position_mm=_jitter(rng, o.position_mm, config.jitter_mm, margin))
        for o in base.obstacles
    ]
    obstacle_points = [o.position_mm for o in obstacles]
    robots: list[ScenarioRobot] = []
    for robot in base.robots:
        occupied = obstacle_points + [r.start_mm for r in robots]
        start = robot.start_mm
        for _ in range(100):
            candidate = _jitter(rng, robot.start_mm, config.jitter_mm, margin)
            if _clear(candidate, occupied, separation):
                start = candidate
                break
        target = (
            None
            if robot.target_mm is None
            else _jitter(rng, robot.target_mm, config.jitter_mm, margin)
        )
        robots.append(replace(robot, start_mm=start, target_mm=target))
    return Scenario(
        name=f"{base.name}~p{index:03d}",
        robots=robots,
        obstacles=obstacles,
        ball=base.ball,
        schema_version=base.schema_version,
    )
