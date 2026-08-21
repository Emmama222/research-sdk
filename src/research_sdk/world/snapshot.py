"""Immutable world snapshots.

These are the public read objects for control, BTs, UI, recording, and
replay. They intentionally do not expose raw SSL-Vision Frame/Robot/Ball
objects.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any


MAX_ROBOTS = 16

@dataclass(frozen=True, slots=True)
class RobotSnapshot:
    isYellow: bool
    robot_id: int
    x: float
    y: float
    theta: float
    confidence: float = 1.0
    visible: bool = True

    @property
    def pose(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.theta)

    @property
    def position(self) -> tuple[float, float]:
        return (self.x, self.y)

    @property
    def id(self) -> int:
        """Compatibility alias for older UI code."""
        return self.robot_id

    @property
    def o(self) -> float:
        """Compatibility alias for older UI code."""
        return self.theta

    @property
    def team(self) -> str:
        return "yellow" if self.isYellow else "blue"


@dataclass(frozen=True, slots=True)
class BallSnapshot:
    x: float
    y: float
    confidence: float = 1.0
    visible: bool = True

    @property
    def position(self) -> tuple[float, float]:
        return (self.x, self.y)


@dataclass(frozen=True, slots=True)
class WorldSnapshot:
    version: int
    timestamp: float
    frame_number: int | None
    ball: BallSnapshot | None
    yellow: tuple[RobotSnapshot | None, ...]
    blue: tuple[RobotSnapshot | None, ...]
    us_yellow: bool
    us_positive: bool
    ball_candidates: tuple[BallSnapshot, ...] = ()
    game_state: Any = None
    active_robots: int = 6
    cards_active: int = 0
    ball_left_field: Any = None
    timeout_time_left: int = 300  # ms
    timeout_left: int = 4

    def robot(self, is_yellow: bool, robot_id: int) -> RobotSnapshot | None:
        team = self.yellow if is_yellow else self.blue
        if 0 <= robot_id < len(team):
            return team[robot_id]
        return None

    def yellow_robot(self, robot_id: int) -> RobotSnapshot | None:
        return self.robot(True, robot_id)

    def blue_robot(self, robot_id: int) -> RobotSnapshot | None:
        return self.robot(False, robot_id)

    def our_robot(self, robot_id: int) -> RobotSnapshot | None:
        return self.robot(self.us_yellow, robot_id)

    def their_robot(self, robot_id: int) -> RobotSnapshot | None:
        return self.robot(not self.us_yellow, robot_id)

    def robots_allowed(self) -> int:
        return self.active_robots - self.cards_active

    @property
    def our_robots(self) -> tuple[RobotSnapshot, ...]:
        team = self.yellow if self.us_yellow else self.blue
        return tuple(robot for robot in team if robot is not None)

    @property
    def their_robots(self) -> tuple[RobotSnapshot, ...]:
        team = self.blue if self.us_yellow else self.yellow
        return tuple(robot for robot in team if robot is not None)


def empty_robot_team() -> tuple[RobotSnapshot | None, ...]:
    return (None,) * MAX_ROBOTS


def snapshot_to_dict(snapshot: WorldSnapshot) -> dict:
    return _jsonable(asdict(snapshot))


def snapshot_from_dict(data: dict) -> WorldSnapshot:
    ball_data = data.get("ball")
    ball_candidates_data = data.get("ball_candidates", ())
    yellow_data = data.get("yellow", ())
    blue_data = data.get("blue", ())

    ball = BallSnapshot(**ball_data) if ball_data is not None else None
    ball_candidates = tuple(
        BallSnapshot(**candidate)
        for candidate in ball_candidates_data
    )
    yellow = tuple(
        RobotSnapshot(**robot) if robot is not None else None
        for robot in yellow_data
    )
    blue = tuple(
        RobotSnapshot(**robot) if robot is not None else None
        for robot in blue_data
    )

    payload = dict(data)
    payload["ball"] = ball
    payload["ball_candidates"] = ball_candidates
    payload["yellow"] = yellow
    payload["blue"] = blue
    return WorldSnapshot(**payload)


def _jsonable(value):
    if hasattr(value, "name"):
        return value.name
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return value
