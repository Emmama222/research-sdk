"""Run a private grSim/ODE process and measure experiments from SSL-Vision.

Only the native simulator integrates positions. Python supplies velocity
commands, verifies telemetry, and records the evidence used for each result.
"""

from __future__ import annotations

import hashlib
import importlib.metadata
import json
import math
import os
import shutil
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from statistics import fmean
from uuid import uuid4

from google.protobuf.message import DecodeError

from research_sdk.config import FIELD_LENGTH_MM, FIELD_WIDTH_MM, ROBOT_RADIUS_MM
from research_sdk.headless import (
    HeadlessRunResult,
    SimulationConfig,
    _advance_waypoint,
    _collisions,
    _distance,
    _new_planner,
    _path_length,
    _plan_robot,
    _planner_key,
)
from research_sdk.network.proto2 import grSim_Packet_pb2, ssl_vision_wrapper_pb2
from research_sdk.ui.scenarios import Scenario

RobotKey = tuple[bool, int]


class PhysicsError(RuntimeError):
    """An engine or telemetry failure; never substituted with kinematics."""


@dataclass(frozen=True)
class Observation:
    frame: int
    time_s: float
    robots: dict[RobotKey, tuple[float, float, float]]
    ball: tuple[float, float] | None


class FrameAssembler:
    """Merge all four grSim camera packets for exactly one simulation tick."""

    def __init__(self):
        self.frame = -1
        self.last_emitted = -1
        self.parts = {}
        self.incomplete = 0

    def push(self, detection) -> Observation | None:
        number = detection.frame_number
        if number <= self.last_emitted or number < self.frame:
            return None
        if detection.camera_id not in range(4) or not math.isfinite(detection.t_capture):
            raise PhysicsError("Invalid camera ID or simulation timestamp")
        if number != self.frame:
            if self.parts:
                self.incomplete += 1
            self.frame, self.parts = number, {}
        self.parts[detection.camera_id] = detection
        if len(self.parts) != 4:
            return None
        times = {part.t_capture for part in self.parts.values()}
        if len(times) != 1:
            raise PhysicsError("Camera timestamps disagree within a frame")
        robots = {}
        balls = []
        for part in self.parts.values():
            for yellow, team in ((True, part.robots_yellow), (False, part.robots_blue)):
                for robot in team:
                    pose = (robot.x, robot.y, robot.orientation)
                    if not robot.HasField("orientation") or not all(map(math.isfinite, pose)):
                        raise PhysicsError("Invalid robot pose in vision packet")
                    robots[yellow, robot.robot_id] = pose
            balls.extend(part.balls)
        ball = max(balls, key=lambda value: value.confidence, default=None)
        if ball is not None and not all(map(math.isfinite, (ball.x, ball.y))):
            raise PhysicsError("Invalid ball position in vision packet")
        self.last_emitted = number
        self.parts = {}
        return Observation(number, times.pop(), robots, None if ball is None else (ball.x, ball.y))


def velocity_packet(yellow: bool, velocities: dict[int, tuple[float, float, float]]):
    """Robot-local forward/left velocities in m/s; angular speed in rad/s."""
    packet = grSim_Packet_pb2.grSim_Packet()
    packet.commands.timestamp = 0.0
    packet.commands.isteamyellow = yellow
    for robot_id, (forward, left, angular) in velocities.items():
        packet.commands.robot_commands.add(
            id=robot_id,
            veltangent=forward,
            velnormal=left,
            velangular=angular,
            kickspeedx=0.0,
            kickspeedz=0.0,
            spinner=False,
            wheelsspeed=False,
        )
    return packet


def local_velocity(vx_mmps: float, vy_mmps: float, heading: float):
    return (
        (math.cos(heading) * vx_mmps + math.sin(heading) * vy_mmps) / 1000.0,
        (-math.sin(heading) * vx_mmps + math.cos(heading) * vy_mmps) / 1000.0,
        0.0,
    )


def validate_scenario(scenario: Scenario, planner: str):
    scenario.require_complete()
    obstacles = scenario.obstacles_for(_planner_key(planner))
    objects = [(r.is_yellow, r.robot_id, r.start_mm) for r in scenario.robots]
    objects += [(o.is_yellow, o.obstacle_id, o.position_mm) for o in obstacles]
    keys = [(yellow, robot_id) for yellow, robot_id, _ in objects]
    if len(keys) != len(set(keys)):
        raise ValueError("Duplicate team/robot IDs in the selected scenario layout")
    if any(not 0 <= robot_id < 16 for _, robot_id in keys):
        raise ValueError("grSim robot IDs must be between 0 and 15")
    for obstacle in obstacles:
        if not math.isclose(obstacle.radius_mm, ROBOT_RADIUS_MM, abs_tol=1e-6):
            raise ValueError("Physics obstacles must match the 90 mm grSim robot model")
        if len(obstacle.velocity_mmps) != 2 or not all(map(math.isfinite, obstacle.velocity_mmps)):
            raise ValueError("Obstacle velocities must be finite")
    points = [point for _, _, point in objects] + [r.target_mm for r in scenario.robots]
    for point in points:
        if len(point) != 2 or not all(map(math.isfinite, point)):
            raise ValueError("Scenario coordinates must be finite 2D points")
        if (
            abs(point[0]) > FIELD_LENGTH_MM / 2 - ROBOT_RADIUS_MM
            or abs(point[1]) > FIELD_WIDTH_MM / 2 - ROBOT_RADIUS_MM
        ):
            raise ValueError("Robot starts and targets must lie within the playable field")
    if any(not math.isfinite(r.orientation_rad) for r in scenario.robots):
        raise ValueError("Robot headings must be finite")
    if scenario.ball:
        for vector in (scenario.ball.position_mm, scenario.ball.velocity_mmps):
            if len(vector) != 2 or not all(map(math.isfinite, vector)):
                raise ValueError("Ball position and velocity must be finite 2D vectors")
        if (
            abs(scenario.ball.position_mm[0]) > FIELD_LENGTH_MM / 2
            or abs(scenario.ball.position_mm[1]) > FIELD_WIDTH_MM / 2
        ):
            raise ValueError("The scenario ball must start within the playable field")
    return obstacles, set(keys), max(robot_id for _, robot_id in keys) + 1


def _profile(path: Path, ports: list[int], count: int, dt: float):
    root = ET.Element("VarXML")

    def var(parent, name, kind, value=None):
        child = ET.SubElement(parent, "Var", {"name": name, "type": kind})
        if value is not None:
            child.text = str(value)
        return child

    geometry = var(root, "Geometry", "list")
    game = var(geometry, "Game", "list")
    var(game, "Division", "stringenum", "Division B")
    var(game, "Robots Count", "int", count)
    var(geometry, "Blue Team", "stringenum", "Parsian")
    var(geometry, "Yellow Team", "stringenum", "Parsian")
    world = var(var(root, "Physics", "list"), "World", "list")
    var(world, "Desired FPS", "double", 1 / dt)
    var(world, "Synchronize ODE with OpenGL", "bool", "false")
    var(world, "ODE time step", "double", dt)
    var(world, "Auto reset turn-over", "bool", "false")
    communication = var(root, "Communication", "list")
    var(communication, "Vision multicast address", "string", "127.0.0.1")
    names = (
        "Vision multicast port",
        "Command listen port",
        "Simulation control port",
        "Blue team control port",
        "Yellow team control port",
        "Blue Team status send port",
        "Yellow Team status send port",
    )
    for name, port in zip(names, ports, strict=True):
        var(communication, name, "int", port)
    # Upstream divides this value by four before taking a modulo.
    var(communication, "Send geometry every X frames", "int", 4)
    var(communication, "Sending delay (milliseconds)", "int", 0)
    noise = var(communication, "Gaussian noise", "list")
    var(noise, "Noise", "bool", "false")
    var(noise, "Vanishing", "bool", "false")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


class GrSimSession:
    """Own one isolated native process, loopback sockets and its evidence files."""

    def __init__(self, binary: Path, folder: Path, count: int, dt: float):
        self.binary = binary.resolve()
        self.folder = folder
        self.dt = dt
        self.count = count
        self.process = None
        self.trace = None
        self.log = None
        self.receiver = None
        self.sender = None
        self.reservations = []
        self.assembler = FrameAssembler()
        self.geometry = None

    def __enter__(self):
        try:
            return self._start()
        except BaseException:
            self.close()
            raise

    def _start(self):
        if os.name != "posix":
            raise PhysicsError("Run the native physics backend in Ubuntu/WSL; see docs/physics.md")
        build_path = self.binary.parent.parent / "build.json"
        if not self.binary.is_file() or not build_path.is_file():
            raise PhysicsError("Build the physics engine first: bash scripts/build_grsim.sh")
        build = json.loads(build_path.read_text(encoding="utf-8"))
        if (
            not build.get("isolated_config")
            or build["binary_sha256"] != hashlib.sha256(self.binary.read_bytes()).hexdigest()
        ):
            raise PhysicsError("Engine hash mismatch or missing isolated-config build support")
        self.folder.mkdir(parents=True, exist_ok=False)
        for _ in range(7):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("127.0.0.1", 0))
            self.reservations.append(sock)
        ports = [sock.getsockname()[1] for sock in self.reservations]
        self.receiver = self.reservations.pop(0)
        self.receiver.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.destination = ("127.0.0.1", ports[1])
        profile_path = self.folder / "grsim.xml"
        _profile(profile_path, ports, self.count, self.dt)
        model_dir = self.binary.parent.parent / "share/grSim/config"
        model_path = model_dir / "Parsian.ini"
        if not model_path.is_file():
            raise PhysicsError(f"Missing grSim robot model: {model_path}")
        if hashlib.sha256(model_path.read_bytes()).hexdigest() != build["robot_model_sha256"]:
            raise PhysicsError(
                "Robot model changed since build; rebuild and record its configuration"
            )
        shutil.copy2(model_path, self.folder / "Parsian.ini")
        self.provenance = {
            "engine": "grSim",
            "physics": "ODE",
            "build": build,
            "profile_sha256": hashlib.sha256(profile_path.read_bytes()).hexdigest(),
            "robot_model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
            "physics_step_s": self.dt,
            "python_packages": {
                name: importlib.metadata.version(name)
                for name in ("numpy", "networkx", "PyYAML", "protobuf")
            },
            "clock": "SSL-Vision t_capture from grSim",
            "collision_measurement": "sampled circular proximity, clearance <= 2 mm",
        }
        self.log = (self.folder / "grsim.log").open("w", encoding="utf-8")
        self.trace = (self.folder / "trajectory.jsonl").open("w", encoding="utf-8")
        env = dict(
            os.environ,
            RESEARCH_GRSIM_CONFIG=str(profile_path.resolve()),
            LIBGL_ALWAYS_SOFTWARE="1",
            QT_QPA_PLATFORM="xcb",
        )
        for sock in self.reservations:
            sock.close()
        self.reservations.clear()
        virtual_display = shutil.which("xvfb-run")
        if virtual_display is None:
            raise PhysicsError("Install xvfb and xauth for the private headless OpenGL display")
        self.process = subprocess.Popen(
            [
                virtual_display,
                "-a",
                "--server-args=-screen 0 640x480x24",
                str(self.binary),
                "--headless",
            ],
            env=env,
            stdout=self.log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        return self

    def send(self, packet):
        self.sender.sendto(packet.SerializeToString(), self.destination)

    def read(self, timeout_s: float = 3.0) -> Observation:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise PhysicsError(f"grSim exited; inspect {self.folder / 'grsim.log'}")
            self.receiver.settimeout(min(0.1, max(0.001, deadline - time.monotonic())))
            try:
                payload = self.receiver.recv(65535)
            except TimeoutError:
                continue
            try:
                packet = ssl_vision_wrapper_pb2.SSL_WrapperPacket.FromString(payload)
            except DecodeError as exc:
                raise PhysicsError("Malformed SSL-Vision packet") from exc
            if not packet.IsInitialized():
                raise PhysicsError("SSL-Vision packet is missing required fields")
            if packet.HasField("geometry"):
                field = packet.geometry.field
                self.geometry = (field.field_length, field.field_width)
            if packet.HasField("detection"):
                observation = self.assembler.push(packet.detection)
                if observation is not None:
                    return observation
        raise PhysicsError("Timed out waiting for a complete four-camera grSim frame")

    def place(self, scenario: Scenario, obstacles, keys):
        # Fresh process and one reset per trial: disabled robots cannot leak into experiments.
        initial = self.read()
        if self.geometry != (int(FIELD_LENGTH_MM), int(FIELD_WIDTH_MM)):
            raise PhysicsError(f"Field geometry mismatch: {self.geometry}")
        packet = grSim_Packet_pb2.grSim_Packet()
        poses = {
            (r.is_yellow, r.robot_id): (*r.start_mm, r.orientation_rad) for r in scenario.robots
        }
        poses.update({(o.is_yellow, o.obstacle_id): (*o.position_mm, 0.0) for o in obstacles})
        for yellow in (False, True):
            for robot_id in range(self.count):
                pose = poses.get((yellow, robot_id), (0.0, 4000.0, 0.0))
                packet.replacement.robots.add(
                    id=robot_id,
                    yellowteam=yellow,
                    turnon=(yellow, robot_id) in poses,
                    x=pose[0] / 1000,
                    y=pose[1] / 1000,
                    dir=math.degrees(pose[2]),
                )
        ball = scenario.ball
        packet.replacement.ball.x = ball.position_mm[0] / 1000 if ball else 0.0
        packet.replacement.ball.y = ball.position_mm[1] / 1000 if ball else 4.0
        # Start ball velocity only after all robots settle (below).
        packet.replacement.ball.vx = 0.0
        packet.replacement.ball.vy = 0.0
        self.send(packet)
        settled_since = None
        deadline = time.monotonic() + 5
        previous = initial
        while time.monotonic() < deadline:
            current = self.read()
            correct = set(current.robots) == keys
            correct = correct and all(
                _distance(current.robots[key][:2], pose[:2]) <= 10
                and abs(math.remainder(current.robots[key][2] - pose[2], 2 * math.pi)) < 0.1
                for key, pose in poses.items()
            )
            delta = current.time_s - previous.time_s
            still = delta > 0 and all(
                key in previous.robots
                and _distance(current.robots[key][:2], previous.robots[key][:2]) / delta < 30
                for key in keys
                if key in current.robots
            )
            if correct and still:
                settled_since = current.time_s if settled_since is None else settled_since
                if current.time_s - settled_since >= 0.25:
                    if ball:
                        moving = grSim_Packet_pb2.grSim_Packet()
                        moving.replacement.ball.CopyFrom(packet.replacement.ball)
                        moving.replacement.ball.vx = ball.velocity_mmps[0] / 1000
                        moving.replacement.ball.vy = ball.velocity_mmps[1] / 1000
                        self.send(moving)
                    return current
            else:
                settled_since = None
            previous = current
        raise PhysicsError("Scenario placement did not settle within position/heading tolerances")

    def close(self):
        if self.process is not None and self.process.poll() is None:
            # Stop the process group created by this session only.
            os.killpg(self.process.pid, signal.SIGTERM)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=3)
        for stream in (self.trace, self.log, self.receiver, self.sender, *self.reservations):
            if stream is not None:
                stream.close()

    def __exit__(self, *_):
        self.close()


def simulate_physics(
    scenario: Scenario,
    planner_name: str,
    *,
    config: SimulationConfig,
    trial: int = 1,
    seed: int = 0,
    binary: Path = Path(".local/grsim/bin/grSim"),
    evidence_dir: Path = Path("results/physics-evidence"),
) -> HeadlessRunResult:
    """Run one measured native physics trial; pass only on complete clean evidence."""
    if config.dt_s > 0.02:
        raise ValueError("Physics step must be at most 20 ms; 8.333 ms is recommended")
    if trial < 1 or seed < 0:
        raise ValueError("trial must be positive and seed must be nonnegative")
    obstacles, keys, count = validate_scenario(scenario, planner_name)
    planner = _new_planner(planner_name, seed)
    wall_start = time.perf_counter()
    plans = [_plan_robot(scenario, robot, planner_name, planner) for robot in scenario.robots]
    states, durations = zip(*plans, strict=True)
    folder = evidence_dir.resolve() / f"{planner_name}-{trial}-{uuid4().hex[:12]}"
    elapsed = 0.0
    ticks = missed_frames = robot_contacts = obstacle_contacts = 0
    max_gap = 0.0
    final_speed = None
    clearance_min = None
    completed = False
    timed_out = False
    error = ""
    active_robot_contacts, active_obstacle_contacts = set(), set()
    initial_time = None
    settled_since = None
    previous = None
    session = GrSimSession(binary, folder, count, config.dt_s)
    with session:
        (folder / "scenario.json").write_text(
            json.dumps(scenario.to_dict(), indent=2, allow_nan=False), encoding="utf-8"
        )
        (folder / "paths.json").write_text(
            json.dumps(
                [
                    {
                        "yellow": s.key[0],
                        "robot_id": s.key[1],
                        "path_mm": s.path,
                        "failed": s.failed,
                    }
                    for s in states
                ],
                indent=2,
            ),
            encoding="utf-8",
        )
        session.provenance.update(
            {
                "scenario_sha256": hashlib.sha256(
                    (folder / "scenario.json").read_bytes()
                ).hexdigest(),
                "controller": asdict(config),
                "planner": planner_name,
                "seed": seed,
                "trial": trial,
                "seed_scope": "PRM sampling; native ODE reproducibility is not guaranteed",
                "sdk_sources_sha256": hashlib.sha256(
                    b"".join(
                        path.read_bytes() for path in sorted(Path(__file__).parent.rglob("*.py"))
                    )
                ).hexdigest(),
            }
        )
        try:
            current = session.place(scenario, obstacles, keys)
            initial_time = current.time_s
            assembly_baseline = session.assembler.incomplete
            execution_deadline = time.monotonic() + max(10.0, 3 * config.max_simulation_s)
            while True:
                if set(current.robots) != keys:
                    raise PhysicsError("Missing or unexpected robots in a complete vision frame")
                speeds = {}
                if previous is not None:
                    delta = current.time_s - previous.time_s
                    if delta <= 0 or current.frame <= previous.frame:
                        raise PhysicsError("Simulation clock or frame number regressed")
                    gap = current.frame - previous.frame
                    if not math.isclose(delta, gap * config.dt_s, abs_tol=1e-5):
                        raise PhysicsError(
                            "Observed simulation step disagrees with physics profile"
                        )
                    missed_frames += gap - 1
                    max_gap = max(max_gap, delta * 1000)
                    for state in states:
                        distance = _distance(
                            current.robots[state.key][:2], previous.robots[state.key][:2]
                        )
                        state.travelled_mm += distance
                        speeds[state.key] = distance / delta
                    final_speed = max(speeds.values(), default=0.0)
                for state in states:
                    state.position = current.robots[state.key][:2]
                elapsed = current.time_s - initial_time
                measured_obstacles = [
                    replace(
                        obstacle,
                        position_mm=current.robots[obstacle.is_yellow, obstacle.obstacle_id][:2],
                        velocity_mmps=(0.0, 0.0),
                    )
                    for obstacle in obstacles
                ]
                robot_pairs, obstacle_pairs, clearance = _collisions(
                    states, measured_obstacles, 0.0, contact_tolerance_mm=2.0
                )
                robot_contacts += len(robot_pairs - active_robot_contacts)
                obstacle_contacts += len(obstacle_pairs - active_obstacle_contacts)
                active_robot_contacts, active_obstacle_contacts = robot_pairs, obstacle_pairs
                if clearance is not None:
                    clearance_min = (
                        clearance if clearance_min is None else min(clearance_min, clearance)
                    )

                commands = {False: {}, True: {}}
                for state in states:
                    if (
                        not state.failed
                        and state.waypoint_index >= len(state.path)
                        and _distance(state.position, state.robot.target_mm)
                        > config.final_tolerance_mm
                    ):
                        state.waypoint_index = max(0, len(state.path) - 1)
                    done = _advance_waypoint(state, config)
                    velocity = (0.0, 0.0, 0.0)
                    if not done:
                        target = state.path[state.waypoint_index]
                        dx, dy = target[0] - state.position[0], target[1] - state.position[1]
                        distance = math.hypot(dx, dy)
                        scale = min(config.gain_per_second, config.max_speed_mmps / distance)
                        velocity = local_velocity(
                            dx * scale, dy * scale, current.robots[state.key][2]
                        )
                    commands[state.key[0]][state.key[1]] = velocity
                for obstacle in obstacles:
                    key = (obstacle.is_yellow, obstacle.obstacle_id)
                    commands[key[0]][key[1]] = local_velocity(
                        *obstacle.velocity_mmps, current.robots[key][2]
                    )
                all_arrived = all(
                    not state.failed
                    and _distance(state.position, state.robot.target_mm)
                    <= config.final_tolerance_mm
                    for state in states
                )
                if all_arrived and final_speed is not None and final_speed <= 50.0:
                    settled_since = current.time_s if settled_since is None else settled_since
                    completed = current.time_s - settled_since >= 0.25
                else:
                    settled_since = None
                trace_record = {
                    "frame": current.frame,
                    "t_capture": current.time_s,
                    "elapsed_s": elapsed,
                    "robots": [
                        {
                            "yellow": key[0],
                            "robot_id": key[1],
                            "pose_mm_rad": pose,
                            "command_local_mps_rad_s": commands[key[0]][key[1]],
                        }
                        for key, pose in sorted(current.robots.items())
                    ],
                    "ball_mm": current.ball,
                    "minimum_clearance_mm": clearance,
                    "robot_contacts": sorted(robot_pairs),
                    "obstacle_contacts": sorted(obstacle_pairs),
                }
                session.trace.write(json.dumps(trace_record, allow_nan=False) + "\n")
                if completed or any(state.failed for state in states):
                    break
                if elapsed >= config.max_simulation_s:
                    timed_out = True
                    break
                if time.monotonic() > execution_deadline:
                    raise PhysicsError("Physics run exceeded its wall-clock watchdog")
                for yellow, team in commands.items():
                    if team:
                        session.send(velocity_packet(yellow, team))
                previous = current
                current = session.read()
                ticks += 1
            missed_frames = max(missed_frames, session.assembler.incomplete - assembly_baseline)
        except (PhysicsError, OSError, ValueError) as exc:
            error = str(exc)
            completed = False
        finally:
            for yellow in (False, True):
                zeros = {robot_id: (0.0, 0.0, 0.0) for team, robot_id in keys if team == yellow}
                if zeros:
                    try:
                        session.send(velocity_packet(yellow, zeros))
                    except OSError:
                        pass  # The owned process is terminated when the session exits.
            session.provenance.update(
                {
                    "geometry_mm": session.geometry,
                    "missed_frames": missed_frames,
                    "max_frame_gap_ms": max_gap,
                    "error": error,
                }
            )
            (folder / "provenance.json").write_text(
                json.dumps(session.provenance, indent=2), encoding="utf-8"
            )

    failed = [state for state in states if state.failed]
    wall_ms = (time.perf_counter() - wall_start) * 1000
    errors = [_distance(state.position, state.robot.target_mm) for state in states]
    status = (
        "physics_error"
        if error
        else "planning_failed"
        if failed
        else "completed"
        if completed
        else "timed_out"
    )
    passed = (
        completed and not error and missed_frames == 0 and robot_contacts + obstacle_contacts == 0
    )
    result = HeadlessRunResult(
        scenario=scenario.name,
        planner=planner_name,
        trial=trial,
        seed=seed,
        status=status,
        completed=completed,
        timed_out=timed_out,
        robot_count=len(states),
        planner_calls=len(states),
        failed_plans=len(failed),
        failed_robots=",".join(
            ("Y" if state.key[0] else "B") + str(state.key[1]) for state in failed
        ),
        planning_time_ms_total=sum(durations),
        planning_time_ms_mean=fmean(durations),
        simulated_duration_ms=elapsed * 1000,
        wall_time_ms=wall_ms,
        realtime_factor=elapsed * 1000 / wall_ms,
        ticks=ticks,
        robot_collision_episodes=robot_contacts,
        obstacle_collision_episodes=obstacle_contacts,
        collision_episodes=robot_contacts + obstacle_contacts,
        planned_path_length_mm=sum(_path_length(state.path) for state in states),
        travelled_distance_mm=sum(state.travelled_mm for state in states),
        mean_final_error_mm=fmean(errors),
        max_final_error_mm=max(errors),
        minimum_clearance_mm=clearance_min,
        dt_ms=config.dt_s * 1000,
        backend="grsim",
        physics_validation_passed=passed,
        missed_frames=missed_frames,
        max_frame_gap_ms=max_gap,
        max_final_speed_mmps=final_speed,
        evidence_directory=str(folder),
        error=error,
    )
    (folder / "result.json").write_text(json.dumps(result.to_record(), indent=2), encoding="utf-8")
    return result
