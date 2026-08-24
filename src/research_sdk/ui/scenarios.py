"""Scenario models and JSON persistence for repeatable planner experiments."""

from __future__ import annotations

import json
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path

Point = tuple[float, float]


@dataclass(frozen=True, slots=True)
class ScenarioRobot:
    robot_id: int
    is_yellow: bool
    start_mm: Point
    target_mm: Point
    orientation_rad: float = 0.0


@dataclass(frozen=True, slots=True)
class ScenarioBall:
    position_mm: Point
    velocity_mmps: Point = (0.0, 0.0)


@dataclass(frozen=True, slots=True)
class ScenarioObstacle:
    obstacle_id: int
    is_yellow: bool
    position_mm: Point
    radius_mm: float
    velocity_mmps: Point = (0.0, 0.0)
    planner_keys: tuple[str, ...] = ()

    def applies_to(self, planner_key: str | None) -> bool:
        """Return whether this obstacle belongs to the selected algorithm.

        Older scenarios have no ``planner_keys`` and therefore remain shared
        by every planner.
        """
        return not self.planner_keys or planner_key in self.planner_keys


@dataclass(slots=True)
class Scenario:
    name: str
    robots: list[ScenarioRobot] = field(default_factory=list)
    obstacles: list[ScenarioObstacle] = field(default_factory=list)
    ball: ScenarioBall | None = None
    schema_version: int = 3

    def set_robot(self, robot: ScenarioRobot) -> int:
        """Insert or move the single robot identified by team and robot ID."""
        key = (robot.is_yellow, robot.robot_id)
        self.obstacles[:] = [
            obstacle
            for obstacle in self.obstacles
            if (obstacle.is_yellow, obstacle.obstacle_id) != key
        ]
        for index, current in enumerate(self.robots):
            if (current.is_yellow, current.robot_id) == key:
                self.robots[index] = robot
                return index
        self.robots.append(robot)
        return len(self.robots) - 1

    def set_obstacle(self, obstacle: ScenarioObstacle) -> int:
        """Insert or move an obstacle within one planner-specific layout."""
        identity = (obstacle.is_yellow, obstacle.obstacle_id)
        key = (*identity, obstacle.planner_keys)
        self.robots[:] = [
            robot
            for robot in self.robots
            if (robot.is_yellow, robot.robot_id) != identity
        ]
        for index, current in enumerate(self.obstacles):
            if (current.is_yellow, current.obstacle_id, current.planner_keys) == key:
                self.obstacles[index] = obstacle
                return index
        self.obstacles.append(obstacle)
        return len(self.obstacles) - 1

    def clear_obstacles(self) -> None:
        self.obstacles.clear()

    def obstacles_for(self, planner_key: str | None) -> tuple[ScenarioObstacle, ...]:
        """Return shared obstacles plus those assigned to ``planner_key``."""
        return tuple(
            obstacle for obstacle in self.obstacles if obstacle.applies_to(planner_key)
        )

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: dict) -> Scenario:
        return cls(
            name=str(payload["name"]),
            robots=[
                ScenarioRobot(
                    robot_id=int(robot["robot_id"]),
                    is_yellow=bool(robot["is_yellow"]),
                    start_mm=tuple(robot["start_mm"]),
                    target_mm=tuple(robot["target_mm"]),
                    orientation_rad=float(robot.get("orientation_rad", 0.0)),
                )
                for robot in payload.get("robots", ())
            ],
            obstacles=[
                ScenarioObstacle(
                    obstacle_id=int(obstacle["obstacle_id"]),
                    is_yellow=bool(obstacle["is_yellow"]),
                    position_mm=tuple(obstacle["position_mm"]),
                    radius_mm=float(obstacle["radius_mm"]),
                    velocity_mmps=tuple(obstacle.get("velocity_mmps", (0.0, 0.0))),
                    planner_keys=tuple(str(key) for key in obstacle.get("planner_keys", ())),
                )
                for obstacle in payload.get("obstacles", ())
            ],
            ball=(
                ScenarioBall(
                    position_mm=tuple(payload["ball"]["position_mm"]),
                    velocity_mmps=tuple(payload["ball"].get("velocity_mmps", (0.0, 0.0))),
                )
                if payload.get("ball") is not None
                else None
            ),
            schema_version=int(payload.get("schema_version", 1)),
        )


class ScenarioStore:
    def __init__(self, folder: str | Path = "scenarios") -> None:
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)

    def save(self, scenario: Scenario) -> Path:
        if not scenario.name.strip():
            raise ValueError("Scenario name cannot be empty")
        path = self.folder / f"{_safe_name(scenario.name)}.json"
        path.write_text(json.dumps(scenario.to_dict(), indent=2), encoding="utf-8")
        return path

    def update(self, path: str | Path, scenario: Scenario) -> Path:
        """Update an existing course file without changing its filename."""
        destination = Path(path)
        if not destination.is_absolute() and destination.parent == Path("."):
            destination = self.folder / destination
        if destination.suffix.lower() != ".json":
            destination = destination.with_suffix(".json")
        if not destination.exists():
            raise FileNotFoundError(f"Course file does not exist: {destination}")
        destination.write_text(
            json.dumps(scenario.to_dict(), indent=2),
            encoding="utf-8",
        )
        return destination

    def load(self, name_or_path: str | Path) -> Scenario:
        path = Path(name_or_path)
        if not path.is_absolute() and path.parent == Path("."):
            path = self.folder / path
        if path.suffix.lower() != ".json":
            path = path.with_suffix(".json")
        return Scenario.from_dict(json.loads(path.read_text(encoding="utf-8")))

    def list_paths(self) -> tuple[Path, ...]:
        return tuple(sorted(self.folder.glob("*.json"), key=lambda path: path.stem.lower()))


def _safe_name(name: str) -> str:
    safe = re.sub(r"[^A-Za-z0-9._-]+", "_", name.strip()).strip("._")
    if not safe:
        raise ValueError("Scenario name must contain a letter or number")
    return safe
