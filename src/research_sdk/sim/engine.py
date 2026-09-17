"""Deterministic 2-D robot/ball world that speaks the grSim packet protocol.

This is a lightweight stand-in for grSim, not a rigid-body physics engine:

* robots are discs that track the commanded body-frame velocity
  (``veltangent`` forward, ``velnormal`` left, ``velangular``) under
  acceleration and speed limits;
* overlapping discs are pushed apart, and everything stays inside the field
  plus its boundary strip;
* the ball rolls with constant deceleration, is pushed by robots, and can be
  flat-kicked from the robot's front;
* vision is published like grSim: one detection packet per camera per
  frame, cameras split by field quadrant.

Use the native grSim backend (``docs/physics.md``) when results must come
from ODE physics.
"""

from __future__ import annotations

import random
import time
from dataclasses import dataclass, field
from math import cos, hypot, pi, sin

from research_sdk.config import (
    BALL_R,
    FIELD_LENGTH_MM,
    FIELD_WIDTH_MM,
    ROBOT_MAX_ANGULAR_SPEED_RAD_S,
    ROBOT_MAX_LINEAR_SPEED_MPS,
    ROBOT_RADIUS_MM,
)
from research_sdk.network.proto2 import (
    grSim_Packet_pb2,
    ssl_vision_detection_pb2,
    ssl_vision_geometry_pb2,
    ssl_vision_wrapper_pb2,
)

RobotKey = tuple[bool, int]
CAMERAS = 4


@dataclass(frozen=True, slots=True)
class SimConfig:
    """Tunable model parameters (SI units unless the name says otherwise)."""

    field_length_mm: float = FIELD_LENGTH_MM
    field_width_mm: float = FIELD_WIDTH_MM
    boundary_mm: float = 300.0
    robot_radius_mm: float = ROBOT_RADIUS_MM
    ball_radius_mm: float = BALL_R
    robots_per_team: int = 6
    max_speed_mps: float = ROBOT_MAX_LINEAR_SPEED_MPS
    max_angular_speed_rad_s: float = max(ROBOT_MAX_ANGULAR_SPEED_RAD_S, 6.0)
    max_accel_mps2: float = 4.0
    max_angular_accel_rad_s2: float = 30.0
    command_timeout_s: float = 0.5
    ball_deceleration_mps2: float = 0.4
    max_kick_speed_mps: float = 6.5
    kick_reach_mm: float = 40.0
    noise_mm: float = 0.0
    noise_rad: float = 0.0
    seed: int = 0

    def __post_init__(self) -> None:
        for name in (
            "field_length_mm",
            "field_width_mm",
            "robot_radius_mm",
            "ball_radius_mm",
            "max_speed_mps",
            "max_angular_speed_rad_s",
            "max_accel_mps2",
            "max_angular_accel_rad_s2",
            "command_timeout_s",
        ):
            if not getattr(self, name) > 0:
                raise ValueError(f"{name} must be positive")
        if not 0 <= self.robots_per_team <= 16:
            raise ValueError("robots_per_team must be between 0 and 16")
        if min(self.boundary_mm, self.noise_mm, self.noise_rad, self.ball_deceleration_mps2) < 0:
            raise ValueError("boundary, noise and ball deceleration must be non-negative")


@dataclass(slots=True)
class SimRobot:
    is_yellow: bool
    robot_id: int
    x_mm: float
    y_mm: float
    theta: float = 0.0
    vx_mps: float = 0.0  # world frame
    vy_mps: float = 0.0
    omega: float = 0.0
    cmd_forward: float = 0.0  # body frame command
    cmd_left: float = 0.0
    cmd_omega: float = 0.0
    cmd_kick_mps: float = 0.0
    cmd_received_at: float = -1e9

    @property
    def key(self) -> RobotKey:
        return (self.is_yellow, self.robot_id)


@dataclass(slots=True)
class SimBall:
    x_mm: float = 0.0
    y_mm: float = 0.0
    vx_mps: float = 0.0
    vy_mps: float = 0.0


def _wrap(angle: float) -> float:
    return (angle + pi) % (2 * pi) - pi


def _approach(current: float, target: float, max_step: float) -> float:
    if target > current:
        return min(target, current + max_step)
    return max(target, current - max_step)


@dataclass
class SimWorld:
    """The simulated state plus the grSim protocol at its boundary."""

    config: SimConfig = field(default_factory=SimConfig)
    robots: dict[RobotKey, SimRobot] = field(default_factory=dict)
    ball: SimBall = field(default_factory=SimBall)
    time_s: float = 0.0
    frame_numbers: list[int] = field(default_factory=lambda: [0] * CAMERAS)

    def __post_init__(self) -> None:
        self._rng = random.Random(self.config.seed)
        if not self.robots:
            self.reset_formation()

    # ------------------------------------------------------------------ setup
    def reset_formation(self) -> None:
        """Place ``robots_per_team`` per team in two lines, like a fresh grSim."""
        self.robots.clear()
        count = self.config.robots_per_team
        spacing = 400.0
        for is_yellow, side in ((False, -1.0), (True, 1.0)):
            for robot_id in range(count):
                y = (robot_id - (count - 1) / 2.0) * spacing
                self.robots[(is_yellow, robot_id)] = SimRobot(
                    is_yellow=is_yellow,
                    robot_id=robot_id,
                    x_mm=side * 1500.0,
                    y_mm=y,
                    theta=0.0 if side < 0 else pi,
                )
        self.ball = SimBall()

    # ------------------------------------------------------- packet boundary
    def apply_packet(self, payload: bytes | grSim_Packet_pb2.grSim_Packet, now_s: float) -> None:
        packet = (
            payload
            if isinstance(payload, grSim_Packet_pb2.grSim_Packet)
            else grSim_Packet_pb2.grSim_Packet.FromString(payload)
        )
        if packet.HasField("replacement"):
            self._apply_replacement(packet.replacement)
        if packet.HasField("commands"):
            is_yellow = bool(packet.commands.isteamyellow)
            for command in packet.commands.robot_commands:
                robot = self.robots.get((is_yellow, int(command.id)))
                if robot is None:
                    continue
                robot.cmd_forward = float(command.veltangent)
                robot.cmd_left = float(command.velnormal)
                robot.cmd_omega = float(command.velangular)
                robot.cmd_kick_mps = max(0.0, float(command.kickspeedx))
                robot.cmd_received_at = now_s

    def _apply_replacement(self, replacement) -> None:
        for item in replacement.robots:
            key = (bool(item.yellowteam), int(item.id))
            if item.HasField("turnon") and not item.turnon:
                self.robots.pop(key, None)
                continue
            robot = self.robots.get(key) or SimRobot(key[0], key[1], 0.0, 0.0)
            robot.x_mm = float(item.x) * 1000.0
            robot.y_mm = float(item.y) * 1000.0
            robot.theta = _wrap(float(item.dir) * pi / 180.0)  # grSim "dir" is degrees
            robot.vx_mps = robot.vy_mps = robot.omega = 0.0
            robot.cmd_forward = robot.cmd_left = robot.cmd_omega = robot.cmd_kick_mps = 0.0
            self.robots[key] = robot
        if replacement.HasField("ball"):
            ball = replacement.ball
            if ball.HasField("x"):
                self.ball.x_mm = float(ball.x) * 1000.0
            if ball.HasField("y"):
                self.ball.y_mm = float(ball.y) * 1000.0
            self.ball.vx_mps = float(ball.vx) if ball.HasField("vx") else 0.0
            self.ball.vy_mps = float(ball.vy) if ball.HasField("vy") else 0.0

    # --------------------------------------------------------------- physics
    def step(self, dt: float, now_s: float) -> None:
        cfg = self.config
        for robot in self.robots.values():
            stale = now_s - robot.cmd_received_at > cfg.command_timeout_s
            forward = 0.0 if stale else robot.cmd_forward
            left = 0.0 if stale else robot.cmd_left
            omega_cmd = 0.0 if stale else robot.cmd_omega
            speed = hypot(forward, left)
            if speed > cfg.max_speed_mps:
                forward *= cfg.max_speed_mps / speed
                left *= cfg.max_speed_mps / speed
            omega_cmd = max(-cfg.max_angular_speed_rad_s, min(cfg.max_angular_speed_rad_s, omega_cmd))
            c, s = cos(robot.theta), sin(robot.theta)
            target_vx = forward * c - left * s
            target_vy = forward * s + left * c
            dvx, dvy = target_vx - robot.vx_mps, target_vy - robot.vy_mps
            dv = hypot(dvx, dvy)
            max_dv = cfg.max_accel_mps2 * dt
            if dv > max_dv:
                dvx, dvy = dvx * max_dv / dv, dvy * max_dv / dv
            robot.vx_mps += dvx
            robot.vy_mps += dvy
            robot.omega = _approach(robot.omega, omega_cmd, cfg.max_angular_accel_rad_s2 * dt)
            robot.x_mm += robot.vx_mps * 1000.0 * dt
            robot.y_mm += robot.vy_mps * 1000.0 * dt
            robot.theta = _wrap(robot.theta + robot.omega * dt)
        self._separate_robots()
        self._step_ball(dt, stale_cutoff=now_s - cfg.command_timeout_s)
        self._clamp_to_field()
        self.time_s += dt

    def _separate_robots(self) -> None:
        robots = list(self.robots.values())
        minimum = 2.0 * self.config.robot_radius_mm
        for _ in range(2):
            for i, a in enumerate(robots):
                for b in robots[i + 1 :]:
                    dx, dy = b.x_mm - a.x_mm, b.y_mm - a.y_mm
                    distance = hypot(dx, dy)
                    if distance >= minimum:
                        continue
                    if distance < 1e-9:
                        dx, dy, distance = 1.0, 0.0, 1.0
                    push = (minimum - distance) / 2.0
                    nx, ny = dx / distance, dy / distance
                    a.x_mm -= nx * push
                    a.y_mm -= ny * push
                    b.x_mm += nx * push
                    b.y_mm += ny * push
                    # remove the approaching velocity component (inelastic contact)
                    rel = (b.vx_mps - a.vx_mps) * nx + (b.vy_mps - a.vy_mps) * ny
                    if rel < 0:
                        a.vx_mps += nx * rel / 2.0
                        a.vy_mps += ny * rel / 2.0
                        b.vx_mps -= nx * rel / 2.0
                        b.vy_mps -= ny * rel / 2.0

    def _step_ball(self, dt: float, stale_cutoff: float) -> None:
        cfg = self.config
        ball = self.ball
        speed = hypot(ball.vx_mps, ball.vy_mps)
        if speed > 0:
            new_speed = max(0.0, speed - cfg.ball_deceleration_mps2 * dt)
            ball.vx_mps *= new_speed / speed
            ball.vy_mps *= new_speed / speed
        ball.x_mm += ball.vx_mps * 1000.0 * dt
        ball.y_mm += ball.vy_mps * 1000.0 * dt
        contact = cfg.robot_radius_mm + cfg.ball_radius_mm
        for robot in self.robots.values():
            dx, dy = ball.x_mm - robot.x_mm, ball.y_mm - robot.y_mm
            distance = hypot(dx, dy)
            if distance < 1e-9:
                dx, dy, distance = cos(robot.theta), sin(robot.theta), 1.0
            nx, ny = dx / distance, dy / distance
            facing = nx * cos(robot.theta) + ny * sin(robot.theta)
            if (
                robot.cmd_kick_mps > 0
                and robot.cmd_received_at >= stale_cutoff
                and distance <= contact + cfg.kick_reach_mm
                and facing > cos(pi / 6)
            ):
                kick = min(robot.cmd_kick_mps, cfg.max_kick_speed_mps)
                ball.vx_mps = robot.vx_mps + kick * cos(robot.theta)
                ball.vy_mps = robot.vy_mps + kick * sin(robot.theta)
                robot.cmd_kick_mps = 0.0
                continue
            if distance < contact:
                ball.x_mm = robot.x_mm + nx * contact
                ball.y_mm = robot.y_mm + ny * contact
                rel = (ball.vx_mps - robot.vx_mps) * nx + (ball.vy_mps - robot.vy_mps) * ny
                if rel < 0:
                    ball.vx_mps -= 1.5 * rel * nx
                    ball.vy_mps -= 1.5 * rel * ny

    def _clamp_to_field(self) -> None:
        cfg = self.config
        half_x = cfg.field_length_mm / 2.0 + cfg.boundary_mm
        half_y = cfg.field_width_mm / 2.0 + cfg.boundary_mm
        for robot in self.robots.values():
            r = cfg.robot_radius_mm
            if abs(robot.x_mm) > half_x - r:
                robot.x_mm = max(-half_x + r, min(half_x - r, robot.x_mm))
                robot.vx_mps = 0.0
            if abs(robot.y_mm) > half_y - r:
                robot.y_mm = max(-half_y + r, min(half_y - r, robot.y_mm))
                robot.vy_mps = 0.0
        ball, r = self.ball, cfg.ball_radius_mm
        if abs(ball.x_mm) > half_x - r:
            ball.x_mm = max(-half_x + r, min(half_x - r, ball.x_mm))
            ball.vx_mps = -0.5 * ball.vx_mps
        if abs(ball.y_mm) > half_y - r:
            ball.y_mm = max(-half_y + r, min(half_y - r, ball.y_mm))
            ball.vy_mps = -0.5 * ball.vy_mps

    # ---------------------------------------------------------------- vision
    @staticmethod
    def camera_for(x_mm: float, y_mm: float) -> int:
        """grSim-style quadrant cameras: 0 (-,-) 1 (-,+) 2 (+,-) 3 (+,+)."""
        return (2 if x_mm >= 0 else 0) + (1 if y_mm >= 0 else 0)

    def detection_packets(
        self, capture_time_s: float | None = None, *, with_geometry: bool = False
    ) -> list[ssl_vision_wrapper_pb2.SSL_WrapperPacket]:
        """One wrapper packet per camera, together forming one complete frame."""
        now = time.time() if capture_time_s is None else capture_time_s
        cfg = self.config
        packets = []
        for camera in range(CAMERAS):
            detection = ssl_vision_detection_pb2.SSL_DetectionFrame(
                frame_number=self.frame_numbers[camera],
                t_capture=now,
                t_sent=now,
                camera_id=camera,
            )
            self.frame_numbers[camera] += 1
            for robot in self.robots.values():
                if self.camera_for(robot.x_mm, robot.y_mm) != camera:
                    continue
                team = detection.robots_yellow if robot.is_yellow else detection.robots_blue
                team.add(
                    confidence=1.0,
                    robot_id=robot.robot_id,
                    x=robot.x_mm + self._noise(cfg.noise_mm),
                    y=robot.y_mm + self._noise(cfg.noise_mm),
                    orientation=_wrap(robot.theta + self._noise(cfg.noise_rad)),
                    pixel_x=0.0,
                    pixel_y=0.0,
                    height=150.0,
                )
            if self.camera_for(self.ball.x_mm, self.ball.y_mm) == camera:
                detection.balls.add(
                    confidence=1.0,
                    x=self.ball.x_mm + self._noise(cfg.noise_mm),
                    y=self.ball.y_mm + self._noise(cfg.noise_mm),
                    z=0.0,
                    pixel_x=0.0,
                    pixel_y=0.0,
                )
            packet = ssl_vision_wrapper_pb2.SSL_WrapperPacket(detection=detection)
            if with_geometry and camera == 0:
                packet.geometry.CopyFrom(self.geometry())
            packets.append(packet)
        return packets

    def geometry(self) -> ssl_vision_geometry_pb2.SSL_GeometryData:
        cfg = self.config
        return ssl_vision_geometry_pb2.SSL_GeometryData(
            field=ssl_vision_geometry_pb2.SSL_GeometryFieldSize(
                field_length=int(cfg.field_length_mm),
                field_width=int(cfg.field_width_mm),
                goal_width=1000,
                goal_depth=180,
                boundary_width=int(cfg.boundary_mm),
            )
        )

    def _noise(self, sigma: float) -> float:
        return self._rng.gauss(0.0, sigma) if sigma > 0 else 0.0

