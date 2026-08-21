"""Scenario models and JSON persistence for repeatable planner experiments."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
import re


Point = tuple[float, float]


@dataclass(frozen=True, slots=True)
class ScenarioRobot:
    robot_id: int
    is_yellow: bool
    start_mm: Point
    target_mm: Point
    orientation_rad: float = 0.0


@dataclass(frozen=True, slots=True)
class ScenarioObstacle:
    obstacle_id: int
    is_yellow: bool
    position_mm: Point
    radius_mm: float
    velocity_mmps: Point = (0.0, 0.0)


@dataclass(slots=True)
class Scenario:
    name: str
    robots: list[ScenarioRobot] = field(default_factory=list)
    obstacles: list[ScenarioObstacle] = field(default_factory=list)
    schema_version: int = 1

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: dict) -> "Scenario":
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
                )
                for obstacle in payload.get("obstacles", ())
            ],
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
