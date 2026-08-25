"""Authoritative state machine for the grSim execution console."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from enum import Enum

from research_sdk.ui.runtime import PlannedRobotPath
from research_sdk.ui.scenarios import Scenario


class ExecutionState(str, Enum):
    NO_SCENARIO = "NO SCENARIO"
    SCENARIO_LOADED = "SCENARIO LOADED"
    APPLYING = "APPLYING"
    READY = "READY"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    COMPLETED = "COMPLETED"
    STOPPED = "STOPPED"
    ERROR = "ERROR"
    RESETTING = "RESETTING"


@dataclass(frozen=True, slots=True)
class ExecutionInput:
    scenario: Scenario
    paths_by_planner: dict[str, tuple[PlannedRobotPath, ...]]
    planner_classes: dict[str, type]
    content_hash: str

    @classmethod
    def create(
        cls,
        scenario: Scenario,
        paths_by_planner: dict[str, tuple[PlannedRobotPath, ...]],
        planner_classes: dict[str, type],
    ) -> ExecutionInput:
        encoded = json.dumps(scenario.to_dict(), sort_keys=True, separators=(",", ":"))
        return cls(
            scenario=scenario,
            paths_by_planner=dict(paths_by_planner),
            planner_classes=dict(planner_classes),
            content_hash=hashlib.sha256(encoded.encode("utf-8")).hexdigest(),
        )


class ExecutionController:
    """Validate transitions and enforce one command-producing planner."""

    def __init__(self) -> None:
        self.state = ExecutionState.NO_SCENARIO
        self.execution_input: ExecutionInput | None = None
        self.shadow_planners: set[str] = set()
        self.velocity_owner: str | None = None
        self.selections_locked = False
        self.error_message = ""
        self.pending_replay = False
        self.pending_checkpoint_id: str | None = None

    def load(self, execution_input: ExecutionInput) -> None:
        self._require_not_running("load a scenario")
        self.execution_input = execution_input
        self.shadow_planners.clear()
        self.velocity_owner = None
        self.selections_locked = False
        self.error_message = ""
        self.pending_replay = False
        self.pending_checkpoint_id = None
        self.state = ExecutionState.SCENARIO_LOADED

    def begin_apply(self) -> None:
        self._require(ExecutionState.SCENARIO_LOADED)
        self.state = ExecutionState.APPLYING

    def confirm_apply(self) -> None:
        if self.state not in (ExecutionState.APPLYING, ExecutionState.RESETTING):
            raise RuntimeError(f"Cannot confirm scenario application from {self.state.value}")
        self.state = ExecutionState.READY

    def fail_apply(self, message: str) -> None:
        if self.state not in (ExecutionState.APPLYING, ExecutionState.RESETTING):
            raise RuntimeError(f"Cannot fail scenario application from {self.state.value}")
        self.error_message = message
        self.state = ExecutionState.ERROR

    def set_shadow(self, planner: str, enabled: bool) -> None:
        self._require(ExecutionState.READY)
        if self.selections_locked:
            raise RuntimeError("Planner selections are locked until Reset E-Stop")
        self._require_planner(planner)
        if enabled:
            self.shadow_planners.add(planner)
        else:
            self.shadow_planners.discard(planner)

    def run(self, planner: str) -> tuple[PlannedRobotPath, ...]:
        self._require(ExecutionState.READY)
        self._require_planner(planner)
        if self.velocity_owner is not None:
            raise RuntimeError(f"{self.velocity_owner} already owns velocity output")
        assert self.execution_input is not None
        paths = self.execution_input.paths_by_planner.get(planner, ())
        if not paths:
            raise RuntimeError(f"{planner} has no executable path")
        self.velocity_owner = planner
        self.shadow_planners.discard(planner)
        self.selections_locked = True
        self.state = ExecutionState.RUNNING
        return paths

    def pause(self) -> None:
        self._require(ExecutionState.RUNNING)
        self.state = ExecutionState.PAUSED

    def continue_run(self) -> None:
        self._require(ExecutionState.PAUSED)
        if self.velocity_owner is None:
            raise RuntimeError("No planner owns velocity output")
        self.state = ExecutionState.RUNNING

    def complete(self) -> None:
        if self.state not in (ExecutionState.RUNNING, ExecutionState.PAUSED):
            raise RuntimeError(f"Cannot complete from {self.state.value}")
        self.state = ExecutionState.COMPLETED

    def stop(self, *, error: str | None = None) -> None:
        self.error_message = error or ""
        self.state = ExecutionState.ERROR if error else ExecutionState.STOPPED
        if self.velocity_owner is not None:
            self.selections_locked = True

    def begin_reset(self) -> None:
        if self.state is ExecutionState.NO_SCENARIO:
            raise RuntimeError("No scenario is loaded")
        self.state = ExecutionState.RESETTING
        self.shadow_planners.clear()
        self.velocity_owner = None
        self.selections_locked = False
        self.error_message = ""

    def reset_without_apply(self) -> None:
        if self.execution_input is None:
            self.state = ExecutionState.NO_SCENARIO
        else:
            self.state = ExecutionState.SCENARIO_LOADED
        self.shadow_planners.clear()
        self.velocity_owner = None
        self.selections_locked = False
        self.error_message = ""

    def request_start_replay(self) -> str:
        if self.state not in (
            ExecutionState.COMPLETED,
            ExecutionState.STOPPED,
            ExecutionState.ERROR,
        ):
            raise RuntimeError(f"Cannot replay from {self.state.value}")
        if self.velocity_owner is None:
            raise RuntimeError("No previous velocity owner is available")
        self.pending_replay = True
        return self.velocity_owner

    def request_checkpoint_resume(self, checkpoint_id: str) -> None:
        if self.state not in (
            ExecutionState.PAUSED,
            ExecutionState.COMPLETED,
            ExecutionState.STOPPED,
            ExecutionState.ERROR,
        ):
            raise RuntimeError(f"Cannot restore a checkpoint from {self.state.value}")
        self.pending_checkpoint_id = checkpoint_id
        self.state = ExecutionState.RESETTING

    @property
    def planner_names(self) -> tuple[str, ...]:
        if self.execution_input is None:
            return ()
        return tuple(self.execution_input.paths_by_planner)

    def _require_planner(self, planner: str) -> None:
        if planner not in self.planner_names:
            raise ValueError(f"Unknown planner: {planner}")

    def _require(self, expected: ExecutionState) -> None:
        if self.state is not expected:
            raise RuntimeError(f"Expected {expected.value}, got {self.state.value}")

    def _require_not_running(self, action: str) -> None:
        if self.state in (
            ExecutionState.APPLYING,
            ExecutionState.RUNNING,
            ExecutionState.PAUSED,
            ExecutionState.RESETTING,
        ):
            raise RuntimeError(f"Cannot {action} while {self.state.value}")
