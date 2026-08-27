import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import startup_check
from research_sdk.network.proto2 import ssl_vision_wrapper_pb2
from research_sdk.world.pipeline import VisionWorldPipeline


def _one_camera_update():
    packet = ssl_vision_wrapper_pb2.SSL_WrapperPacket()
    detection = packet.detection
    detection.frame_number = 1
    detection.t_capture = 1.0
    detection.t_sent = 1.0
    detection.camera_id = 0
    robot = detection.robots_yellow.add()
    robot.confidence = 1.0
    robot.robot_id = 2
    robot.x = 100.0
    robot.y = 200.0
    robot.orientation = 0.0
    robot.pixel_x = 0.0
    robot.pixel_y = 0.0
    return VisionWorldPipeline(cameras=1).ingest(packet)


def test_startup_vision_check_rejects_running_source_without_packets(
    monkeypatch, capsys
) -> None:
    monkeypatch.setattr(
        startup_check,
        "receive_grsim_vision",
        lambda _timeout: startup_check.VisionProbeResult(0, 0, (), None),
    )

    assert not startup_check.vision_check(0.1)
    assert "No UDP packets received" in capsys.readouterr().out


def test_startup_vision_check_confirms_planner_boundary(monkeypatch, capsys) -> None:
    update = _one_camera_update()
    monkeypatch.setattr(
        startup_check,
        "receive_grsim_vision",
        lambda _timeout: startup_check.VisionProbeResult(1, 100, (0,), update),
    )

    assert startup_check.vision_check(0.1)
    output = capsys.readouterr().out
    assert "WorldSnapshot" in output
    assert "Planner scene" in output
