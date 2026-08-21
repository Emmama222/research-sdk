from math import hypot

import pytest

from research_sdk.config import (
    ROBOT_MAX_ANGULAR_LINEAR_RATIO,
    ROBOT_MAX_ANGULAR_SPEED_RAD_S,
    ROBOT_MAX_LINEAR_SPEED_MPS,
)
from research_sdk.network.robot_command import RobotCommand


def _expected_angular_limit(linear_speed: float) -> float:
    if linear_speed == 0.0:
        return ROBOT_MAX_ANGULAR_SPEED_RAD_S
    linear_fraction = min(linear_speed / ROBOT_MAX_LINEAR_SPEED_MPS, 1.0)
    moving_fraction = ROBOT_MAX_ANGULAR_LINEAR_RATIO + (
        1.0 - ROBOT_MAX_ANGULAR_LINEAR_RATIO
    ) * linear_fraction
    return ROBOT_MAX_ANGULAR_SPEED_RAD_S * moving_fraction


def test_linear_velocity_below_limit_is_unchanged() -> None:
    command = RobotCommand(robot_id=1, vx=0.6, vy=0.8)

    assert command.vx == pytest.approx(0.6)
    assert command.vy == pytest.approx(0.8)


def test_linear_velocity_is_scaled_without_changing_direction() -> None:
    command = RobotCommand(robot_id=1, vx=3.0, vy=4.0)

    assert hypot(command.vx, command.vy) == pytest.approx(ROBOT_MAX_LINEAR_SPEED_MPS)
    assert command.vx / command.vy == pytest.approx(3.0 / 4.0)


def test_rotation_in_place_uses_full_angular_limit() -> None:
    positive = RobotCommand(robot_id=1, w=99.0)
    negative = RobotCommand(robot_id=1, w=-99.0)

    assert positive.w == pytest.approx(ROBOT_MAX_ANGULAR_SPEED_RAD_S)
    assert negative.w == pytest.approx(-ROBOT_MAX_ANGULAR_SPEED_RAD_S)


@pytest.mark.parametrize("linear_speed", [0.1, 0.5, 1.0, 1.5, 2.0])
def test_angular_clamp_scales_with_linear_speed(linear_speed: float) -> None:
    command = RobotCommand(robot_id=1, vx=linear_speed, w=99.0)

    assert command.w == pytest.approx(_expected_angular_limit(linear_speed))


def test_angular_velocity_below_scaled_limit_is_unchanged() -> None:
    command = RobotCommand(robot_id=1, vx=0.5, w=0.75)

    assert command.w == pytest.approx(0.75)


def test_angular_clamp_uses_already_clamped_linear_speed() -> None:
    command = RobotCommand(robot_id=1, vx=30.0, vy=40.0, w=99.0)

    assert hypot(command.vx, command.vy) == pytest.approx(ROBOT_MAX_LINEAR_SPEED_MPS)
    assert command.w == pytest.approx(ROBOT_MAX_ANGULAR_SPEED_RAD_S)
