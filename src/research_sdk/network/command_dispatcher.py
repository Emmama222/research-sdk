"""Fixed-rate delivery of the latest grSim command for each robot."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from threading import Event, Lock, Thread, current_thread
from time import monotonic

from research_sdk.network.robot_command import RobotCommand

RobotKey = tuple[bool, int]


@dataclass(slots=True)
class _LatestCommand:
    command: RobotCommand
    updated_at: float


class RobotCommandDispatcher:
    """Repeat fresh commands at a fixed rate and fail stale commands to zero.

    ``publish`` sends immediately to preserve low latency, then the worker
    repeats the latest command until it is replaced.  Once a command exceeds
    ``command_ttl_s`` the worker repeatedly sends a zero-velocity command.
    """

    def __init__(
        self,
        send: Callable[[RobotCommand], None],
        *,
        send_hz: float = 100.0,
        command_ttl_s: float = 0.2,
    ) -> None:
        if send_hz <= 0:
            raise ValueError("send_hz must be positive")
        if command_ttl_s <= 0:
            raise ValueError("command_ttl_s must be positive")
        self._send = send
        self._period_s = 1.0 / float(send_hz)
        self._command_ttl_s = float(command_ttl_s)
        self._commands: dict[RobotKey, _LatestCommand] = {}
        self._lock = Lock()
        self._stop = Event()
        self._thread: Thread | None = None
        self.last_error: Exception | None = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.running:
            return
        self._stop.clear()
        self._thread = Thread(target=self._run, name="grsim-command-dispatcher", daemon=True)
        self._thread.start()

    def publish(self, command: RobotCommand) -> None:
        key = (bool(command.isYellow), int(command.robot_id))
        with self._lock:
            self._commands[key] = _LatestCommand(command, monotonic())
        self._send(command)

    def stop(self, *, clear: bool = True) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not current_thread():
            thread.join(timeout=max(0.1, self._period_s * 3.0))
        self._thread = None
        if clear:
            with self._lock:
                self._commands.clear()

    def _run(self) -> None:
        while not self._stop.wait(self._period_s):
            now = monotonic()
            with self._lock:
                commands = tuple(self._commands.items())
            for key, latest in commands:
                command = latest.command
                if now - latest.updated_at > self._command_ttl_s:
                    command = RobotCommand(robot_id=key[1], isYellow=key[0])
                try:
                    self._send(command)
                except Exception as exc:  # noqa: BLE001 - keep other robots receiving commands
                    self.last_error = exc
