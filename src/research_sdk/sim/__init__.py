"""Built-in lightweight simulator that speaks the grSim UDP protocol.

``research-sdk-sim`` (or ``python -m research_sdk.sim``) starts it on the
ports in ``config/network_input.yaml``; the Qt console can start/stop it from
its toolbar. See ``docs/builtin-simulator.md``.
"""

from research_sdk.sim.engine import SimConfig, SimWorld
from research_sdk.sim.manager import SimulatorProcess
from research_sdk.sim.server import ServerConfig, SimServer

__all__ = ["ServerConfig", "SimConfig", "SimServer", "SimWorld", "SimulatorProcess"]
