import json
from pathlib import Path

from research_sdk.headless import _load_scenarios, _scenario_paths


BANK = Path(__file__).parents[1] / "scenarios" / "acra2026-200"


def test_acra_bank_loads_200_six_robot_six_moving_obstacle_scenarios() -> None:
    paths = _scenario_paths([str(BANK)])
    scenarios = _load_scenarios(paths)

    assert len(paths) == len(scenarios) == 200
    for scenario in scenarios:
        assert len(scenario.robots) == 6
        assert all(robot.target_mm is not None for robot in scenario.robots)
        assert len(scenario.obstacles) == 6
        assert all(obstacle.patrol_waypoints for obstacle in scenario.obstacles)


def test_acra_bank_has_no_initial_physical_overlaps() -> None:
    for path in sorted(BANK.glob("*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        robots = data["robots"]
        obstacles = data["obstacles"]
        for i, first in enumerate(robots):
            for second in robots[i + 1 :]:
                assert _distance(first["start_mm"], second["start_mm"]) > 180.0, path
            for obstacle in obstacles:
                assert _distance(first["start_mm"], obstacle["position_mm"]) > (
                    90.0 + obstacle["radius_mm"]
                ), path


def _distance(first: list[float], second: list[float]) -> float:
    return ((first[0] - second[0]) ** 2 + (first[1] - second[1]) ** 2) ** 0.5
