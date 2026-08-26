from typing import ClassVar

from research_sdk.config import (
    GRSIM_COMMAND_IP,
    GRSIM_COMMAND_PORT,
    ROBOT_MAX_LINEAR_SPEED_MPS,
)
from research_sdk.network.proto2 import ssl_vision_wrapper_pb2
from research_sdk.planners import PlannerOutput
from research_sdk.process_workers.vision_runner import VisionFrameAssembler
from research_sdk.ui.runtime import (
    LiveRobot,
    PlannedRobotPath,
    ResearchRuntime,
    live_world_from_vision_packet,
    waypoint_command,
)
from research_sdk.ui.scenarios import Scenario, ScenarioObstacle, ScenarioRobot
from research_sdk.world.scene import PlanningObstacle, PlanningScene
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
        self.commands = []

    def send_robot_command(self, command) -> None:
        self.commands.append(command)


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

    assert command.vx == ROBOT_MAX_LINEAR_SPEED_MPS
    assert command.vy == 0.0


def test_runtime_records_failed_planner_invocation() -> None:
    """A robot whose planner raises gets flagged failed/stationary, not a
    raised exception -- one bad route shouldn't abort every other robot's
    plan (see docs/decisions/0005-parallel-planner-execution.md)."""

    class FailingPlanner:
        def plan(self, _planner_input):
            raise RuntimeError("no route")

    runtime = ResearchRuntime()
    runtime._planner = FailingPlanner()
    scenario = Scenario(
        "blocked",
        robots=[ScenarioRobot(1, True, (0.0, 0.0), (1000.0, 0.0))],
    )

    paths = runtime.plan(scenario)

    assert len(paths) == 1
    assert paths[0].failed is True
    assert paths[0].points_mm == ((0.0, 0.0),)
    assert runtime.last_plan_failures == 1
    assert len(runtime.last_plan_durations_ms) == 1
    assert runtime.last_plan_durations_ms[0] >= 0.0


def test_runtime_only_supplies_obstacles_for_selected_algorithm() -> None:
    class CapturingPlanner:
        received_obstacles = ()

        def plan(self, planner_input):
            type(self).received_obstacles = planner_input.scene.obstacles
            return PlannerOutput(
                waypoints=(),
                current_waypoint_index=0,
                active_target_pose=planner_input.target_pose,
                is_path_free=True,
                need_reroute=False,
                did_reroute=False,
            )

        def reset(self):
            pass

    selected_key = f"{CapturingPlanner.__module__}.{CapturingPlanner.__qualname__}"
    scenario = Scenario(
        "planner layouts",
        robots=[ScenarioRobot(1, True, (0.0, 0.0), (1000.0, 0.0))],
        obstacles=[
            ScenarioObstacle(2, False, (100.0, 0.0), 90.0),
            ScenarioObstacle(
                3, False, (200.0, 0.0), 90.0, planner_keys=(selected_key,)
            ),
            ScenarioObstacle(
                4, False, (300.0, 0.0), 90.0, planner_keys=("other.Planner",)
            ),
        ],
    )
    runtime = ResearchRuntime()
    runtime.set_planner(CapturingPlanner)

    runtime.plan(scenario)

    assert tuple(obstacle.robot_id for obstacle in CapturingPlanner.received_obstacles) == (2, 3)


class _CapturingPlanner:
    """Records each robot's obstacle list by robot_id, keyed per-call --
    both robots in a scenario get planned (possibly concurrently, see
    ResearchRuntime.parallel_planning), so a single shared attribute would
    just be overwritten by whichever robot happens to be planned last."""

    received_obstacles_by_robot: ClassVar[dict[int, tuple]] = {}

    def plan(self, planner_input):
        type(self).received_obstacles_by_robot[planner_input.robot_id] = (
            planner_input.scene.obstacles
        )
        return PlannerOutput(
            waypoints=(),
            current_waypoint_index=0,
            active_target_pose=planner_input.target_pose,
            is_path_free=True,
            need_reroute=False,
            did_reroute=False,
        )

    def reset(self):
        pass


def _two_robot_scenario() -> Scenario:
    return Scenario(
        "teammates",
        robots=[
            ScenarioRobot(1, True, (0.0, 0.0), (1000.0, 0.0)),
            ScenarioRobot(2, True, (500.0, 500.0), (1500.0, 500.0)),
        ],
    )


def test_predict_motion_off_uses_static_teammate_positions() -> None:
    """Default behaviour (predict_motion=False): teammates are sourced from
    their static scenario start_mm, regardless of whether a live tracked
    scene happens to exist -- unaffected by decision 5 in
    docs/decisions/0005-parallel-planner-execution.md unless explicitly
    turned on."""
    _CapturingPlanner.received_obstacles_by_robot.clear()
    runtime = ResearchRuntime(predict_motion=False)
    runtime.world_pipeline.latest_scene = PlanningScene(
        timestamp=0.0,
        obstacles=(
            PlanningObstacle(robot_id=2, isYellow=True, pos_mm=(9999.0, 9999.0), radius_mm=90.0),
        ),
    )
    runtime.set_planner(_CapturingPlanner)

    runtime.plan(_two_robot_scenario())

    robot1_view = _CapturingPlanner.received_obstacles_by_robot[1]
    teammate = next(o for o in robot1_view if o.robot_id == 2)
    assert teammate.pos_mm == (500.0, 500.0), "should use scenario start_mm, not the live scene"


def test_predict_motion_on_sources_teammates_from_live_scene() -> None:
    """predict_motion=True with a live tracked scene: teammate/opponent
    obstacles come from the real tracked position/velocity/dynamic radius
    (WorldMap.planning_scene()) instead of static scenario positions, and
    the robot being planned for is excluded from its own obstacle list."""
    _CapturingPlanner.received_obstacles_by_robot.clear()
    runtime = ResearchRuntime(predict_motion=True)
    runtime.world_pipeline.latest_scene = PlanningScene(
        timestamp=0.0,
        obstacles=(
            PlanningObstacle(
                robot_id=2, isYellow=True, pos_mm=(600.0, 550.0), radius_mm=90.0, vel_mmps=(1000.0, 0.0)
            ),
            PlanningObstacle(robot_id=1, isYellow=True, pos_mm=(0.0, 0.0), radius_mm=90.0),
        ),
    )
    runtime.set_planner(_CapturingPlanner)

    runtime.plan(_two_robot_scenario())

    robot1_view = _CapturingPlanner.received_obstacles_by_robot[1]
    robot_ids = {o.robot_id for o in robot1_view}
    assert robot_ids == {2}, "robot 1 must be excluded from its own obstacle list"
    teammate = next(o for o in robot1_view if o.robot_id == 2)
    assert teammate.pos_mm == (600.0, 550.0)
    assert teammate.vel_mmps == (1000.0, 0.0)


def test_predict_motion_on_falls_back_when_no_live_scene_exists() -> None:
    """predict_motion=True but vision hasn't produced a frame yet (fresh
    reset, or the scenario editor's offline "Plan" preview, which runs with
    no simulator/vision attached at all): falls back to the static
    behaviour automatically rather than raising or returning no obstacles."""
    _CapturingPlanner.received_obstacles_by_robot.clear()
    runtime = ResearchRuntime(predict_motion=True)
    assert runtime.world_pipeline.latest_scene is None
    runtime.set_planner(_CapturingPlanner)

    runtime.plan(_two_robot_scenario())

    robot1_view = _CapturingPlanner.received_obstacles_by_robot[1]
    teammate = next(o for o in robot1_view if o.robot_id == 2)
    assert teammate.pos_mm == (500.0, 500.0)


def test_pause_continue_and_step_preserve_execution_progress() -> None:
    runtime = ResearchRuntime()
    sender = _FakeSender()
    runtime._sender = sender
    path = PlannedRobotPath(1, True, ((0.0, 0.0), (500.0, 0.0), (1000.0, 0.0)))
    runtime.start_execution((path,))

    runtime.pause_execution()
    assert runtime.paused
    assert runtime.waypoint_indices[(True, 1)] == 1
    assert sender.commands[-1].vx == sender.commands[-1].vy == 0.0

    runtime.continue_execution()
    runtime.execute_tick({(True, 1): LiveRobot(1, True, (0.0, 0.0), 0.0)})
    assert sender.commands[-1].vx > 0.0

    runtime.pause_execution()
    runtime.step_execution()
    runtime.execute_tick({(True, 1): LiveRobot(1, True, (500.0, 0.0), 0.0)})
    assert runtime.step_completed
    assert runtime.paused
    assert runtime.waypoint_indices[(True, 1)] == 2


def test_emergency_stop_attempts_every_robot_after_sender_failure() -> None:
    class PartlyFailingSender(_FakeSender):
        def send_robot_command(self, command) -> None:
            self.commands.append(command)
            if command.robot_id == 1:
                raise RuntimeError("sender failed")

    runtime = ResearchRuntime()
    sender = PartlyFailingSender()
    runtime._sender = sender
    runtime.start_execution(
        (
            PlannedRobotPath(1, True, ((0.0, 0.0), (1.0, 0.0))),
            PlannedRobotPath(2, True, ((0.0, 0.0), (1.0, 0.0))),
        )
    )

    errors = runtime.emergency_stop()

    assert errors
    assert [command.robot_id for command in sender.commands] == [1, 2]
    assert runtime.active_paths == ()
