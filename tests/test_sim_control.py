import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest  # noqa: E402
from PySide6.QtWidgets import QApplication, QMessageBox, QWidget  # noqa: E402

import research_sdk.ui.sim_control as sim_control  # noqa: E402
from research_sdk.ui.sim_control import SimulatorControl, simulator_args_from_config  # noqa: E402

CONFIG = {
    "grsim_command_ip": "127.0.0.1",
    "grsim_command_port": 20010,
    "grsim_vision_port": 10020,
    "ssl_vision_multicast_group": "224.5.23.2",
    "multicast_interface_ip": "0.0.0.0",
}


def _application() -> QApplication:
    return QApplication.instance() or QApplication([])


class _FakeProcess:
    def __init__(self, fail: bool = False) -> None:
        self.fail = fail
        self.running = False
        self.process = None
        self.started_with = None
        self.output = ["[sim] stopped"]

    def start(self, args):
        if self.fail:
            raise RuntimeError("Built-in simulator did not start:\n[sim] failed to start: in use")
        self.started_with = list(args)
        self.running = True
        self.process = object()

    def stop(self):
        self.running = False
        self.process = None
        return 0


def test_simulator_uses_exactly_the_consoles_grsim_ports() -> None:
    args = simulator_args_from_config(CONFIG)
    assert args == [
        "--command-host", "127.0.0.1",
        "--command-port", "20010",
        "--vision-address", "224.5.23.2",
        "--vision-port", "10020",
        "--multicast-interface", "0.0.0.0",
        "--time-scale", "1",
    ]


def test_remote_grsim_address_is_rejected_with_guidance() -> None:
    with pytest.raises(ValueError, match="127.0.0.1"):
        simulator_args_from_config({**CONFIG, "grsim_command_ip": "192.168.1.20"})


def test_toolbar_action_starts_and_stops_the_simulator(monkeypatch) -> None:
    _application()
    monkeypatch.setattr(sim_control.yaml, "safe_load", lambda _text: CONFIG)
    parent = QWidget()
    fake = _FakeProcess()
    control = SimulatorControl(parent, process=fake)
    assert [control.speed_selector.itemData(i) for i in range(control.speed_selector.count())] == [
        1,
        2,
        5,
    ]
    control.speed_selector.setCurrentText("5x")

    control.action.setChecked(True)
    assert fake.running and fake.started_with[:2] == ["--command-host", "127.0.0.1"]
    assert fake.started_with[-2:] == ["--time-scale", "5"]
    assert "5x target" in control.status.text()
    assert not control.speed_selector.isEnabled()
    assert control.watchdog.isActive()

    control.action.setChecked(False)
    assert not fake.running
    assert control.status.text() == "Simulator: external"
    assert not control.watchdog.isActive()
    assert control.speed_selector.isEnabled()


def test_start_failure_unchecks_and_warns(monkeypatch) -> None:
    _application()
    monkeypatch.setattr(sim_control.yaml, "safe_load", lambda _text: CONFIG)
    warnings = []
    monkeypatch.setattr(QMessageBox, "warning", lambda *args: warnings.append(args[2]))
    control = SimulatorControl(QWidget(), process=_FakeProcess(fail=True))

    control.action.setChecked(True)
    assert not control.action.isChecked()
    assert control.status.text() == "Simulator: failed to start"
    assert warnings and "already running" in warnings[0]


def test_watchdog_reports_an_unexpected_exit(monkeypatch) -> None:
    _application()
    monkeypatch.setattr(sim_control.yaml, "safe_load", lambda _text: CONFIG)
    warnings = []
    monkeypatch.setattr(QMessageBox, "warning", lambda *args: warnings.append(args[2]))
    fake = _FakeProcess()
    control = SimulatorControl(QWidget(), process=fake)
    control.start()
    fake.running = False  # child died

    control._check_alive()
    assert not control.action.isChecked()
    assert control.status.text() == "Simulator: stopped unexpectedly"
    assert warnings and "[sim] stopped" in warnings[0]
