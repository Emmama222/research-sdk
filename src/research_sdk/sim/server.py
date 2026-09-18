"""Real-time UDP loop: grSim commands in, SSL-Vision frames out."""

from __future__ import annotations

import socket
import struct
import time
from dataclasses import dataclass
from math import isfinite

from research_sdk.sim.engine import SimConfig, SimWorld

DEFAULT_VISION_GROUP = "224.5.23.2"
READY_MARKER = "[sim] ready"
# The live Qt controller runs at 20 wall-clock Hz and vision is published at
# 60 wall-clock Hz.  Larger scales let a repeated velocity command cover many
# simulated seconds before feedback can correct it; use headless for >5x.
TIME_SCALES = (1, 2, 5)
_MAX_PHYSICS_STEPS_PER_CYCLE = 256


@dataclass(frozen=True, slots=True)
class ServerConfig:
    command_host: str = "127.0.0.1"
    command_port: int = 20010
    vision_address: str = DEFAULT_VISION_GROUP
    vision_port: int = 10020
    multicast_interface: str = "0.0.0.0"
    vision_hz: float = 60.0
    physics_hz: float = 240.0
    geometry_every_frames: int = 60
    time_scale: float = 1.0

    def __post_init__(self) -> None:
        if self.vision_hz <= 0 or self.physics_hz <= 0:
            raise ValueError("vision_hz and physics_hz must be positive")
        if self.physics_hz < self.vision_hz:
            raise ValueError("physics_hz must be at least vision_hz")
        if not isfinite(self.time_scale) or self.time_scale <= 0:
            raise ValueError("time_scale must be finite and positive")
        if self.time_scale not in TIME_SCALES:
            raise ValueError(f"time_scale must be one of {TIME_SCALES}")
        for port in (self.command_port, self.vision_port):
            if not 0 <= port <= 65535:
                raise ValueError(f"invalid UDP port {port}")

    @property
    def is_multicast(self) -> bool:
        first = int(self.vision_address.split(".")[0])
        return 224 <= first <= 239


class SimServer:
    """Owns the sockets; ``run`` blocks until ``stop`` (or Ctrl+C)."""

    def __init__(self, config: ServerConfig | None = None, sim: SimConfig | None = None) -> None:
        self.config = config or ServerConfig()
        self.world = SimWorld(sim or SimConfig())
        self._running = False
        self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_socket.bind((self.config.command_host, self.config.command_port))
        self.command_socket.setblocking(False)
        self.vision_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if self.config.is_multicast:
            self.vision_socket.setsockopt(
                socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, struct.pack("b", 1)
            )
            self.vision_socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
            if self.config.multicast_interface != "0.0.0.0":
                self.vision_socket.setsockopt(
                    socket.IPPROTO_IP,
                    socket.IP_MULTICAST_IF,
                    socket.inet_aton(self.config.multicast_interface),
                )
        self.vision_destination = (self.config.vision_address, self.config.vision_port)
        self.frames_sent = 0
        self.packets_received = 0
        self._send_error_reported = False

    @property
    def command_address(self) -> tuple[str, int]:
        return self.command_socket.getsockname()

    def drain_commands(self, now_s: float) -> None:
        while True:
            try:
                payload, _ = self.command_socket.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                return
            except ConnectionResetError:  # Windows reports ICMP port-unreachable here
                continue
            try:
                self.world.apply_packet(payload, now_s)
                self.packets_received += 1
            except Exception as exc:  # noqa: BLE001 - one bad datagram must not stop the sim
                print(f"[sim] ignored malformed packet: {exc}", flush=True)

    def publish_frame(self, now_wall: float) -> None:
        with_geometry = self.frames_sent % max(1, self.config.geometry_every_frames) == 0
        try:
            for packet in self.world.detection_packets(now_wall, with_geometry=with_geometry):
                self.vision_socket.sendto(packet.SerializeToString(), self.vision_destination)
        except OSError as exc:
            if not self._send_error_reported:
                print(f"[sim] vision send failed ({exc}); still retrying", flush=True)
                self._send_error_reported = True
        self.frames_sent += 1

    def run(self, duration_s: float | None = None) -> None:
        cfg = self.config
        physics_dt = 1.0 / cfg.physics_hz
        physics_wall_dt = physics_dt / cfg.time_scale
        vision_dt = 1.0 / cfg.vision_hz
        start = time.perf_counter()
        next_physics = start
        next_vision = start
        self._running = True
        try:
            while self._running:
                now = time.perf_counter()
                if duration_s is not None and now - start >= duration_s:
                    break
                self.drain_commands(now)
                steps = 0
                while next_physics <= now and steps < _MAX_PHYSICS_STEPS_PER_CYCLE:
                    # Physics always uses the original fixed timestep.  ``now`` remains
                    # wall time so command expiry is not shortened by fast-forwarding.
                    self.world.step(physics_dt, now)
                    next_physics += physics_wall_dt
                    steps += 1
                if next_vision <= now:
                    self.publish_frame(time.time())
                    next_vision += vision_dt
                    if next_vision <= now:
                        next_vision = now + vision_dt
                sleep_for = min(next_physics, next_vision) - time.perf_counter()
                if sleep_for > 0:
                    time.sleep(min(sleep_for, 0.005))
        finally:
            self._running = False

    def stop(self) -> None:
        self._running = False

    def close(self) -> None:
        self.stop()
        self.command_socket.close()
        self.vision_socket.close()
