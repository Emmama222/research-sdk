import socket
import threading
import time
from math import pi

import pytest

from research_sdk.network.grSimPacketFactory import grSimPacketFactory
from research_sdk.network.proto2 import ssl_vision_wrapper_pb2
from research_sdk.network.ssl_sockets import grSimSender
from research_sdk.process_workers.vision_runner import VisionFrameAssembler
from research_sdk.sim import ServerConfig, SimConfig, SimServer, SimulatorProcess, SimWorld
from research_sdk.world.pipeline import VisionWorldPipeline


def _world(**kwargs) -> SimWorld:
    return SimWorld(SimConfig(robots_per_team=kwargs.pop("robots_per_team", 0), **kwargs))


def _run(world: SimWorld, seconds: float, dt: float = 1 / 240, command=None) -> None:
    steps = int(round(seconds / dt))
    for i in range(steps):
        now = i * dt
        if command is not None:
            world.apply_packet(command, now)
        world.step(dt, now)


def _place(world, robot_id=0, is_yellow=True, x=0.0, y=0.0, orientation=0.0):
    world.apply_packet(
        grSimPacketFactory.robot_replacement_command(
            x=x, y=y, orientation=orientation, robot_id=robot_id, isYellow=is_yellow
        ),
        0.0,
    )


def test_default_world_matches_a_fresh_grsim_lineup() -> None:
    world = SimWorld(SimConfig(robots_per_team=6))
    assert len(world.robots) == 12
    assert all(r.x_mm < 0 for r in world.robots.values() if not r.is_yellow)
    assert all(r.x_mm > 0 for r in world.robots.values() if r.is_yellow)


def test_replacement_uses_metres_and_degrees() -> None:
    world = _world()
    _place(world, robot_id=3, x=1.2, y=-0.5, orientation=pi / 2)
    robot = world.robots[(True, 3)]
    assert (robot.x_mm, robot.y_mm) == pytest.approx((1200.0, -500.0))
    assert robot.theta == pytest.approx(pi / 2)


def test_body_frame_command_moves_robot_in_world_frame() -> None:
    world = _world()
    _place(world, orientation=pi / 2)  # facing +y
    forward = grSimPacketFactory.robot_command(0, vx=1.0, vy=0.0, w=0.0, isYellow=True)
    _run(world, 2.0, command=forward)
    robot = world.robots[(True, 0)]
    assert robot.y_mm > 1500.0  # accelerates to 1 m/s within 0.25 s
    assert abs(robot.x_mm) < 1.0


def test_speed_and_acceleration_are_limited() -> None:
    world = _world(max_speed_mps=2.0, max_accel_mps2=4.0)
    _place(world)
    command = grSimPacketFactory.robot_command(0, vx=10.0, isYellow=True)
    _run(world, 0.25, command=command)
    assert world.robots[(True, 0)].vx_mps == pytest.approx(1.0, abs=0.02)
    _run(world, 2.0, command=command)
    assert world.robots[(True, 0)].vx_mps == pytest.approx(2.0, abs=1e-6)


def test_robot_stops_when_commands_go_stale() -> None:
    world = _world(command_timeout_s=0.2)
    _place(world)
    world.apply_packet(grSimPacketFactory.robot_command(0, vx=1.0, isYellow=True), 0.0)
    _run(world, 3.0)
    robot = world.robots[(True, 0)]
    assert robot.vx_mps == pytest.approx(0.0)
    assert 0.0 < robot.x_mm < 400.0


def test_robots_do_not_overlap_and_stay_in_the_field() -> None:
    world = _world()
    _place(world, 0, True, x=0.0)
    _place(world, 1, True, x=0.5)
    world.apply_packet(grSimPacketFactory.robot_command(0, vx=3.0, isYellow=True), 0.0)
    _run(world, 0.4)
    a, b = world.robots[(True, 0)], world.robots[(True, 1)]
    assert abs(b.x_mm - a.x_mm) >= 2 * world.config.robot_radius_mm - 1e-6
    command = grSimPacketFactory.robot_command(0, vx=5.0, isYellow=True)
    _run(world, 5.0, command=command)
    limit = world.config.field_length_mm / 2 + world.config.boundary_mm
    assert all(abs(r.x_mm) <= limit - world.config.robot_radius_mm + 1e-6 for r in world.robots.values())


def test_turn_off_removes_a_robot() -> None:
    world = _world()
    _place(world)
    packet = grSimPacketFactory.robot_replacement_command(
        x=0, y=0, orientation=0, robot_id=0, isYellow=True
    )
    packet.replacement.robots[0].turnon = False
    world.apply_packet(packet, 0.0)
    assert (True, 0) not in world.robots


def test_ball_rolls_decelerates_and_can_be_kicked() -> None:
    world = _world()
    world.apply_packet(grSimPacketFactory.ball_replacement_command(x=0.0, y=1.0, vx=1.0), 0.0)
    _run(world, 1.0)
    assert world.ball.x_mm == pytest.approx(800.0, abs=5.0)  # v*t - a*t^2/2
    assert world.ball.vx_mps == pytest.approx(0.6, abs=0.01)

    world = _world()
    _place(world)  # facing +x at the origin
    world.apply_packet(grSimPacketFactory.ball_replacement_command(x=0.12, y=0.0), 0.0)
    kick = grSimPacketFactory.robot_command(0, kick=False, isYellow=True)
    kick.commands.robot_commands[0].kickspeedx = 4.0
    world.apply_packet(kick, 0.0)
    world.step(1 / 240, 0.0)
    assert world.ball.vx_mps == pytest.approx(4.0, abs=0.05)


def test_detection_packets_form_one_complete_four_camera_frame() -> None:
    world = SimWorld(SimConfig(robots_per_team=6))
    packets = world.detection_packets(123.0, with_geometry=True)
    assert [p.detection.camera_id for p in packets] == [0, 1, 2, 3]
    assert packets[0].HasField("geometry")
    for packet in packets:
        assert packet.IsInitialized()
        ssl_vision_wrapper_pb2.SSL_WrapperPacket.FromString(packet.SerializeToString())
    assembler = VisionFrameAssembler(4)
    frames = [assembler.push(p.detection) for p in packets]
    assert frames[:3] == [None, None, None]
    frame = frames[3]
    assert len(frame.robots_yellow) + len(frame.robots_blue) == 12


def _free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def test_server_time_scale_advances_fixed_step_physics_faster_than_wall_time() -> None:
    server = SimServer(
        ServerConfig(
            command_port=0,
            vision_address="127.0.0.1",
            vision_port=_free_udp_port(),
            time_scale=5,
        ),
        SimConfig(robots_per_team=0),
    )
    started = time.perf_counter()
    try:
        server.run(duration_s=0.05)
    finally:
        server.close()

    wall_s = time.perf_counter() - started
    assert wall_s >= 0.04
    assert server.world.time_s >= 0.12


def test_server_rejects_non_positive_time_scale() -> None:
    with pytest.raises(ValueError, match="time_scale"):
        ServerConfig(time_scale=0)


def test_server_rejects_time_scale_above_ui_maximum() -> None:
    with pytest.raises(ValueError, match="one of"):
        ServerConfig(time_scale=10)


def test_server_round_trip_through_the_sdk_sender_and_vision_pipeline() -> None:
    vision_port = _free_udp_port()
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", vision_port))
    receiver.settimeout(0.5)
    server = SimServer(
        ServerConfig(command_port=0, vision_address="127.0.0.1", vision_port=vision_port),
        SimConfig(robots_per_team=0),
    )
    thread = threading.Thread(target=server.run, kwargs={"duration_s": 5.0}, daemon=True)
    thread.start()
    host, port = server.command_address
    sender = grSimSender(ip=host, port=port)
    try:
        sender.send_packet(
            grSimPacketFactory.robot_replacement_command(
                x=0.0, y=0.0, orientation=0.0, robot_id=2, isYellow=False
            )
        )
        pipeline = VisionWorldPipeline(cameras=4)
        deadline = time.monotonic() + 4.0
        last_x = None
        while time.monotonic() < deadline:
            sender.send_packet(grSimPacketFactory.robot_command(2, vx=1.0, isYellow=False))
            try:
                payload, _ = receiver.recvfrom(65535)
            except TimeoutError:
                continue
            update = pipeline.ingest_bytes(payload)
            if update is None:
                continue
            robot = update.snapshot.blue[2]
            if robot is not None:
                last_x = robot.position[0]
                if last_x > 300.0:
                    break
        assert last_x is not None and last_x > 300.0
        assert server.packets_received > 0
    finally:
        server.stop()
        thread.join(2.0)
        server.close()
        sender.sock.close()
        receiver.close()


def test_process_manager_starts_reports_ready_and_stops() -> None:
    manager = SimulatorProcess()
    manager.start(
        ["--command-port", "0", "--vision-address", "127.0.0.1",
         "--vision-port", str(_free_udp_port()), "--robots-per-team", "1"]
    )
    try:
        assert manager.running
        assert any(line.startswith("[sim] ready") for line in manager.output)
    finally:
        returncode = manager.stop()
    assert not manager.running
    assert returncode == 0, "closing stdin must stop the simulator gracefully"


def test_process_manager_surfaces_startup_failure() -> None:
    blocker = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    blocker.bind(("127.0.0.1", 0))
    port = blocker.getsockname()[1]
    try:
        with pytest.raises(RuntimeError, match="failed to start"):
            SimulatorProcess().start(["--command-port", str(port)])
    finally:
        blocker.close()
