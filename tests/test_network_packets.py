import pytest

from research_sdk.network.grSimPacketFactory import grSimPacketFactory
from research_sdk.network.robot_command import RobotCommand


def test_scenario_replacement_packet_contains_all_robots() -> None:
    packet = grSimPacketFactory.scenario_replacement_command(
        (
            {
                "x": -1.0,
                "y": 0.5,
                "orientation": 0.25,
                "robot_id": 1,
                "isYellow": True,
            },
            {
                "x": 1.25,
                "y": -0.75,
                "orientation": -0.5,
                "robot_id": 4,
                "isYellow": False,
            },
        )
    )

    replacements = packet.replacement.robots
    assert len(replacements) == 2
    assert (replacements[0].id, replacements[0].yellowteam) == (1, True)
    assert replacements[0].x == pytest.approx(-1.0)
    assert replacements[0].y == pytest.approx(0.5)
    assert replacements[1].id == 4
    assert not replacements[1].yellowteam
    assert all(replacement.turnon for replacement in replacements)


def test_empty_scenario_replacement_is_rejected() -> None:
    with pytest.raises(ValueError, match="at least one robot"):
        grSimPacketFactory.scenario_replacement_command(())


def test_robot_command_round_trip_preserves_command_fields() -> None:
    original = RobotCommand(
        robot_id=7,
        vx=0.5,
        vy=-0.25,
        w=0.75,
        kick=1,
        dribble=1,
        isYellow=False,
    )

    decoded = RobotCommand.decode(original.encode())

    assert decoded.robot_id == original.robot_id
    assert decoded.vx == pytest.approx(original.vx)
    assert decoded.vy == pytest.approx(original.vy)
    assert decoded.w == pytest.approx(original.w)
    assert decoded.kick == original.kick
    assert decoded.dribble == original.dribble


def test_grsim_robot_packet_uses_command_values() -> None:
    command = RobotCommand(3, vx=0.4, vy=-0.2, w=0.3, dribble=1, isYellow=True)
    packet = grSimPacketFactory.robot_command(**command.to_dict())
    encoded = packet.commands.robot_commands[0]

    assert packet.commands.isteamyellow
    assert encoded.id == 3
    assert encoded.veltangent == pytest.approx(0.4)
    assert encoded.velnormal == pytest.approx(-0.2)
    assert encoded.velangular == pytest.approx(0.3)
    assert encoded.spinner
