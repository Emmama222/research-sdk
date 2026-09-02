from threading import Lock
from time import sleep

from research_sdk.network.command_dispatcher import RobotCommandDispatcher
from research_sdk.network.robot_command import RobotCommand


class _CommandSink:
    def __init__(self) -> None:
        self._lock = Lock()
        self.commands = []

    def send(self, command: RobotCommand) -> None:
        with self._lock:
            self.commands.append(command)

    def snapshot(self):
        with self._lock:
            return tuple(self.commands)


def test_dispatcher_repeats_latest_command_and_stops_cleanly() -> None:
    sink = _CommandSink()
    dispatcher = RobotCommandDispatcher(sink.send, send_hz=200.0, command_ttl_s=0.2)
    dispatcher.start()
    dispatcher.publish(RobotCommand(3, vx=1.0, isYellow=True))

    sleep(0.04)
    dispatcher.stop()
    sent_at_stop = len(sink.snapshot())
    sleep(0.02)

    assert sent_at_stop >= 2
    assert all(command.robot_id == 3 for command in sink.snapshot())
    assert len(sink.snapshot()) == sent_at_stop


def test_dispatcher_replaces_stale_motion_with_repeated_zero_commands() -> None:
    sink = _CommandSink()
    dispatcher = RobotCommandDispatcher(sink.send, send_hz=200.0, command_ttl_s=0.02)
    dispatcher.start()
    dispatcher.publish(RobotCommand(4, vx=1.0, isYellow=False))

    sleep(0.06)
    dispatcher.stop()
    commands = sink.snapshot()

    assert any(command.vx > 0.0 for command in commands)
    assert any(command.vx == command.vy == command.w == 0.0 for command in commands)
    assert commands[-1].isYellow is False


def test_publishing_stop_replaces_cached_motion_immediately() -> None:
    sink = _CommandSink()
    dispatcher = RobotCommandDispatcher(sink.send, send_hz=200.0, command_ttl_s=1.0)
    dispatcher.start()
    dispatcher.publish(RobotCommand(5, vx=1.0, isYellow=True))
    dispatcher.publish(RobotCommand(5, isYellow=True))

    sleep(0.03)
    dispatcher.stop()
    commands = sink.snapshot()
    first_stop = next(index for index, command in enumerate(commands) if command.vx == 0.0)

    assert all(command.vx == 0.0 for command in commands[first_stop:])


def test_invalid_dispatch_rates_and_timeouts_are_rejected() -> None:
    sink = _CommandSink()

    for kwargs in ({"send_hz": 0.0}, {"command_ttl_s": 0.0}):
        try:
            RobotCommandDispatcher(sink.send, **kwargs)
        except ValueError:
            pass
        else:
            raise AssertionError(f"Expected ValueError for {kwargs}")
