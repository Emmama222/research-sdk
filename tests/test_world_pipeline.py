from research_sdk.network.proto2 import ssl_vision_wrapper_pb2
from research_sdk.world.pipeline import VisionWorldPipeline, WorldSnapshotStore


def _camera_packet(frame_number: int, camera_id: int):
    packet = ssl_vision_wrapper_pb2.SSL_WrapperPacket()
    detection = packet.detection
    detection.frame_number = frame_number
    detection.t_capture = float(frame_number)
    detection.t_sent = float(frame_number)
    detection.camera_id = camera_id
    robot = detection.robots_yellow.add()
    robot.confidence = 1.0
    robot.robot_id = camera_id
    robot.x = float(camera_id * 100)
    robot.y = 0.0
    robot.orientation = 0.0
    robot.pixel_x = 0.0
    robot.pixel_y = 0.0
    if camera_id == 0:
        ball = detection.balls.add()
        ball.confidence = 1.0
        ball.x = 25.0
        ball.y = -50.0
        ball.pixel_x = 0.0
        ball.pixel_y = 0.0
    return packet


def test_packets_flow_to_snapshot_and_planner_scene() -> None:
    pipeline = VisionWorldPipeline(cameras=4)

    assert pipeline.ingest(_camera_packet(1, 0)) is None
    assert pipeline.ingest(_camera_packet(1, 1)) is None
    assert pipeline.ingest(_camera_packet(1, 2)) is None
    update = pipeline.ingest(_camera_packet(1, 3))

    assert update is not None
    assert update.snapshot.frame_number == 1
    assert [robot.robot_id for robot in update.snapshot.our_robots] == [0, 1, 2, 3]
    assert update.snapshot.ball.position == (25.0, -50.0)
    assert {obstacle.robot_id for obstacle in update.planning_scene.obstacles} == {0, 1, 2, 3}
    assert pipeline.store.current is update.snapshot
    assert pipeline.latest_scene is update.planning_scene
    assert update.processing_latency_ms >= 0.0
    assert update.frame_assembly_latency_ms >= update.processing_latency_ms


def test_snapshot_store_publish_and_unsubscribe() -> None:
    store = WorldSnapshotStore()
    pipeline = VisionWorldPipeline(cameras=1, store=store)
    received = []
    unsubscribe = store.subscribe(received.append)

    first = pipeline.ingest(_camera_packet(1, 0))
    unsubscribe()
    pipeline.ingest(_camera_packet(2, 0))

    assert received == [first.snapshot]
    assert store.current.version == 2


def test_serialized_udp_payload_reaches_planner_boundary() -> None:
    pipeline = VisionWorldPipeline(cameras=1)
    payload = _camera_packet(9, 0).SerializeToString()

    update = pipeline.ingest_bytes(payload)

    assert update.snapshot.frame_number == 9
    assert update.planning_scene.obstacles[0].robot_id == 0


def test_independent_camera_frame_numbers_form_one_world_snapshot() -> None:
    pipeline = VisionWorldPipeline(cameras=4)

    for camera_id in range(3):
        assert pipeline.ingest(_camera_packet(100 + camera_id, camera_id)) is None
    update = pipeline.ingest(_camera_packet(103, 3))

    assert update is not None
    assert {robot.robot_id for robot in update.snapshot.our_robots} == {0, 1, 2, 3}


def test_repeated_camera_publishes_partial_cycle_when_grsim_uses_fewer_cameras() -> None:
    pipeline = VisionWorldPipeline(cameras=4)

    assert pipeline.ingest(_camera_packet(1, 0)) is None
    update = pipeline.ingest(_camera_packet(2, 0))

    assert update is not None
    assert update.snapshot.frame_number == 1
    assert [robot.robot_id for robot in update.snapshot.our_robots] == [0]
