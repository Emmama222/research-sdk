import pytest

from research_sdk.config import GRSIM_COMMAND_IP, GRSIM_COMMAND_PORT
from research_sdk.network.proto2 import ssl_vision_wrapper_pb2
from research_sdk.process_workers.vision_runner import VisionFrameAssembler
from research_sdk.ui.runtime import (
    LiveRobot,
    ResearchRuntime,
    live_world_from_vision_packet,
    waypoint_command,
)
from research_sdk.ui.scenarios import Scenario, ScenarioRobot
from research_sdk.world.snapshot import world_snapshot_from_frame


class _FakeSocket:
    def __init__(self) -> None:
        self.connected_to = None

    def connect(self, destination) -> None:
        self.connected_to = destination

    def getsockname(self):
        return ("127.0.0.1", 54321)


class _FakeSender:
    def __init__(self) -> None:
        self.destination = (GRSIM_COMMAND_IP, GRSIM_COMMAND_PORT)
        self.sock = _FakeSocket()


def test_grsim_connection_probe_reports_route_without_claiming_udp_ack() -> None:
    runtime = ResearchRuntime()
    sender = _FakeSender()
    runtime._sender = sender

    result = runtime.test_grsim_connection()

    assert sender.sock.connected_to == sender.destination
    assert result.destination == sender.destination
    assert result.local_address == ("127.0.0.1", 54321)


def test_live_world_frame_extracts_grsim_robots_and_best_ball() -> None:
    packet = ssl_vision_wrapper_pb2.SSL_WrapperPacket()
    detection = packet.detection
    detection.frame_number = 1
    detection.t_capture = 1.0
    detection.t_sent = 1.1
    detection.camera_id = 0
    yellow = detection.robots_yellow.add()
    yellow.confidence = 0.9
    yellow.robot_id = 2
    yellow.x = 100.0
    yellow.y = -200.0
    yellow.orientation = 0.5
    yellow.pixel_x = 0.0
    yellow.pixel_y = 0.0
    blue = detection.robots_blue.add()
    blue.confidence = 0.8
    blue.robot_id = 4
    blue.x = -300.0
    blue.y = 400.0
    blue.pixel_x = 0.0
    blue.pixel_y = 0.0
    for confidence, x in ((0.4, 10.0), (0.95, 20.0)):
        ball = detection.balls.add()
        ball.confidence = confidence
        ball.x = x
        ball.y = 30.0
        ball.pixel_x = 0.0
        ball.pixel_y = 0.0

    frame = live_world_from_vision_packet(packet)

    assert frame is not None
    assert [(robot.is_yellow, robot.robot_id) for robot in frame.robots] == [
        (True, 2),
        (False, 4),
    ]
    assert frame.robots[0].position_mm == (100.0, -200.0)
    assert frame.ball_mm == (20.0, 30.0)


def test_live_world_ignores_geometry_only_packet() -> None:
    packet = ssl_vision_wrapper_pb2.SSL_WrapperPacket()
    packet.geometry.field.field_length = 12000

    assert live_world_from_vision_packet(packet) is None


def test_vision_packet_reuses_world_snapshot_contract() -> None:
    packet = ssl_vision_wrapper_pb2.SSL_WrapperPacket()
    detection = packet.detection
    detection.frame_number = 7
    detection.t_capture = 12.5
    detection.t_sent = 12.6
    detection.camera_id = 0
    robot = detection.robots_yellow.add()
    robot.confidence = 0.9
    robot.robot_id = 3
    robot.x = 120.0
    robot.y = -80.0
    robot.orientation = 0.25
    robot.pixel_x = 0.0
    robot.pixel_y = 0.0
    ball = detection.balls.add()
    ball.confidence = 0.8
    ball.x = 10.0
    ball.y = 20.0
    ball.pixel_x = 0.0
    ball.pixel_y = 0.0

    assembler = VisionFrameAssembler(1)
    frame = assembler.push(detection)
    snapshot = world_snapshot_from_frame(frame, version=4)

    assert snapshot is not None
    assert snapshot.version == 4
    assert snapshot.frame_number == 7
    assert snapshot.yellow_robot(3).position == (120.0, -80.0)
    assert snapshot.ball.position == (10.0, 20.0)


def test_runtime_execution_reads_robots_from_world_snapshot() -> None:
    packet = ssl_vision_wrapper_pb2.SSL_WrapperPacket()
    detection = packet.detection
    detection.frame_number = 1
    detection.t_capture = 1.0
    detection.t_sent = 1.0
    detection.camera_id = 0
    robot = detection.robots_blue.add()
    robot.confidence = 1.0
    robot.robot_id = 2
    robot.x = 5.0
    robot.y = 6.0
    robot.orientation = 0.5
    robot.pixel_x = 0.0
    robot.pixel_y = 0.0
    runtime = ResearchRuntime()

    for camera_id in range(4):
        detection.camera_id = camera_id
        runtime.ingest_vision_packet(packet)

    live = runtime.live_robots[(False, 2)]
    assert live.position_mm == (5.0, 6.0)
    assert live.orientation_rad == 0.5


def test_waypoint_command_transforms_world_velocity_into_robot_frame() -> None:
    robot = LiveRobot(
        robot_id=5,
        is_yellow=False,
        position_mm=(0.0, 0.0),
        orientation_rad=1.5707963267948966,
    )

    command = waypoint_command(robot, (1000.0, 0.0), gain_per_second=1.0)

    assert command.robot_id == 5
    assert not command.isYellow
    assert abs(command.vx) < 1e-9
    assert command.vy == -1.0


def test_waypoint_command_uses_robot_speed_clamping() -> None:
    robot = LiveRobot(1, True, (0.0, 0.0), 0.0)

    command = waypoint_command(robot, (10000.0, 0.0))

    assert command.vx == 2.0
    assert command.vy == 0.0


def test_runtime_records_failed_planner_invocation() -> None:
    class FailingPlanner:
        def plan(self, _planner_input):
            raise RuntimeError("no route")

    runtime = ResearchRuntime()
    runtime._planner = FailingPlanner()
    scenario = Scenario(
        "blocked",
        robots=[ScenarioRobot(1, True, (0.0, 0.0), (1000.0, 0.0))],
    )

    with pytest.raises(RuntimeError, match="no route"):
        runtime.plan(scenario)

    assert runtime.last_plan_failures == 1
    assert len(runtime.last_plan_durations_ms) == 1
    assert runtime.last_plan_durations_ms[0] >= 0.0
