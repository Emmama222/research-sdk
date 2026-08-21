from dataclasses import FrozenInstanceError
import time

import pytest

from research_sdk.world.map.world_map import WorldMap
from research_sdk.world.scene import PlanningObstacle, PlanningScene
from research_sdk.world.snapshot import RobotSnapshot, WorldSnapshot, empty_robot_team


def test_scene_filters_robots_without_mutating_input() -> None:
    obstacle = PlanningObstacle(
        robot_id=2,
        isYellow=True,
        pos_mm=(0.0, 0.0),
        radius_mm=120.0,
    )
    scene = PlanningScene(timestamp=1.0, obstacles=(obstacle,))

    assert scene.get_planning_obstacles() == (obstacle,)
    assert scene.get_planning_obstacles(ignore_robots={(True, 2)}) == ()
    with pytest.raises(FrozenInstanceError):
        scene.timestamp = 2.0  # type: ignore[misc]


def test_scene_checks_path_against_predicted_obstacle_radius() -> None:
    scene = PlanningScene(
        timestamp=1.0,
        obstacles=(
            PlanningObstacle(
                robot_id=3,
                isYellow=False,
                pos_mm=(0.0, 0.0),
                radius_mm=120.0,
            ),
        ),
    )

    assert not scene.is_path_free((-500.0, 0.0), (500.0, 0.0))
    assert scene.is_path_free((-500.0, 500.0), (500.0, 500.0))
    assert scene.is_path_free(
        (-500.0, 0.0),
        (500.0, 0.0),
        ignore_robots={(False, 3)},
    )


def test_world_map_builds_frozen_planning_scene() -> None:
    now_s = time.time()
    yellow = list(empty_robot_team())
    yellow[1] = RobotSnapshot(True, 1, 100.0, 200.0, 0.0)
    snapshot = WorldSnapshot(
        version=1,
        timestamp=now_s,
        frame_number=1,
        ball=None,
        yellow=tuple(yellow),
        blue=empty_robot_team(),
        us_yellow=True,
        us_positive=True,
    )
    world_map = WorldMap(snapshot=snapshot)

    scene = world_map.planning_scene(now_s=now_s, horizon_ms=250)

    assert scene.prediction_horizon_ms == 250.0
    assert len(scene.obstacles) == 1
    assert scene.obstacles[0].key == (True, 1)
    assert scene.obstacles[0].radius_mm >= 120.0
