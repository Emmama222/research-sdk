"""Toolbar control that runs the built-in simulator behind the console.

The simulator is a child process speaking the grSim UDP protocol on the ports
in ``config/network_input.yaml``, so every page (Apply to grSim, Execution,
live replanning) works against it unchanged. Leave it off to use an external
grSim (Docker, WSL or native) on the same ports instead.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

import yaml
from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtGui import QAction
from PySide6.QtWidgets import QLabel, QMessageBox, QWidget

from research_sdk.sim.manager import SimulatorProcess

CONFIG_FILE = Path(__file__).resolve().parents[1] / "config" / "network_input.yaml"
_LOCAL_HOSTS = {"127.0.0.1", "localhost"}


def simulator_args_from_config(config: Mapping) -> list[str]:
    """Translate ``network_input.yaml`` into ``research_sdk.sim`` arguments.

    The console sends commands to ``grsim_command_ip:grsim_command_port`` and
    listens for grSim vision on ``ssl_vision_multicast_group:grsim_vision_port``
    (see ``grSimVision``), so the simulator must use exactly those.
    """
    command_ip = str(config["grsim_command_ip"]).strip()
    if command_ip not in _LOCAL_HOSTS:
        raise ValueError(
            f"grsim_command_ip is {command_ip!r}; set it to 127.0.0.1 in Configurations "
            "to use the built-in simulator (or turn it off to use a remote grSim)."
        )
    return [
        "--command-host",
        "127.0.0.1",
        "--command-port",
        str(int(config["grsim_command_port"])),
        "--vision-address",
        str(config["ssl_vision_multicast_group"]),
        "--vision-port",
        str(int(config["grsim_vision_port"])),
        "--multicast-interface",
        str(config.get("multicast_interface_ip", "0.0.0.0")),
    ]


class SimulatorControl(QObject):
    state_changed = Signal(bool)

    def __init__(self, parent: QWidget, process: SimulatorProcess | None = None) -> None:
        super().__init__(parent)
        self._parent = parent
        self.process = process or SimulatorProcess()
        self.action = QAction("Built-in simulator", parent, checkable=True)
        self.action.setToolTip(
            "Run the lightweight grSim-compatible simulator on the configured grSim ports.\n"
            "Kinematic model (no ODE physics). Turn off to use an external grSim."
        )
        self.action.toggled.connect(self._toggled)
        self.status = QLabel("Simulator: external")
        self.watchdog = QTimer(self)
        self.watchdog.setInterval(1000)
        self.watchdog.timeout.connect(self._check_alive)

    @property
    def running(self) -> bool:
        return self.process.running

    def _toggled(self, checked: bool) -> None:
        if checked:
            self.start()
        else:
            self.stop()

    def start(self) -> bool:
        if self.process.running:
            return True
        try:
            config = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
            self.process.start(simulator_args_from_config(config))
        except (OSError, ValueError, RuntimeError, KeyError) as exc:
            self._set_checked(False)
            self.status.setText("Simulator: failed to start")
            QMessageBox.warning(
                self._parent,
                "Built-in simulator",
                f"{exc}\n\nIf another grSim is already running on these ports, "
                "stop it first or leave the built-in simulator off.",
            )
            return False
        self._set_checked(True)
        self.status.setText("Simulator: built-in (running)")
        self.watchdog.start()
        self.state_changed.emit(True)
        return True

    def stop(self) -> None:
        self.watchdog.stop()
        self.process.stop()
        self._set_checked(False)
        self.status.setText("Simulator: external")
        self.state_changed.emit(False)

    def shutdown(self) -> None:
        if self.process.process is not None:
            self.stop()

    def _check_alive(self) -> None:
        if self.process.running:
            return
        tail = "\n".join(list(self.process.output)[-10:]) or "(no output)"
        self.stop()
        self.status.setText("Simulator: stopped unexpectedly")
        QMessageBox.warning(self._parent, "Built-in simulator", f"The simulator exited:\n{tail}")

    def _set_checked(self, checked: bool) -> None:
        self.action.blockSignals(True)
        self.action.setChecked(checked)
        self.action.blockSignals(False)
