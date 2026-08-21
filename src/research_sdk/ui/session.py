"""Research-session state machine, planner discovery, and placeholder exports."""

from __future__ import annotations

import csv
from datetime import datetime, timezone
from enum import Enum
import importlib
import inspect
import json
import pkgutil
from pathlib import Path

import research_sdk.planners as planner_package


class SessionState(str, Enum):
    AFTER_RESET = "after_reset"
    SCENARIO_READY = "scenario_ready"
    RUNNING = "running"
    STOPPED = "stopped"


class SessionController:
    def __init__(self) -> None:
        self.state = SessionState.AFTER_RESET
        self.has_scenario = False
        self.has_plan = False
        self.vision_enabled = False

    def scenario_forwarded(self) -> None:
        self.has_scenario = True
        self.has_plan = False
        self.state = SessionState.SCENARIO_READY

    def run(self) -> None:
        if not self.has_scenario:
            raise RuntimeError("Forward a scenario before running")
        self.has_plan = True
        self.state = SessionState.RUNNING

    def stop(self) -> None:
        if self.state is not SessionState.RUNNING:
            raise RuntimeError("Only a running session can be stopped")
        self.state = SessionState.STOPPED

    def erase_plan(self) -> None:
        if self.state is SessionState.RUNNING:
            raise RuntimeError("Stop the session before erasing its plan")
        self.has_plan = False

    def reset(self) -> None:
        if self.state not in (SessionState.STOPPED, SessionState.AFTER_RESET):
            raise RuntimeError("Stop the session before resetting")
        self.has_plan = False
        self.state = SessionState.AFTER_RESET

    def set_vision(self, enabled: bool) -> None:
        if self.state is not SessionState.AFTER_RESET:
            raise RuntimeError("Vision can only change immediately after reset")
        self.vision_enabled = bool(enabled)

    @property
    def can_change_vision(self) -> bool:
        return self.state is SessionState.AFTER_RESET

    @property
    def can_change_vision_source(self) -> bool:
        return self.can_change_vision and not self.vision_enabled


def discover_planners() -> dict[str, type]:
    """Discover concrete classes ending in ``Planner`` below planners/."""
    found: dict[str, type] = {}
    prefix = f"{planner_package.__name__}."
    for module_info in pkgutil.walk_packages(planner_package.__path__, prefix):
        module = importlib.import_module(module_info.name)
        for name, candidate in inspect.getmembers(module, inspect.isclass):
            if candidate.__module__ != module.__name__ or not name.endswith("Planner"):
                continue
            found[f"{name} ({module_info.name.rsplit('.', 1)[-1]})"] = candidate
    return dict(sorted(found.items()))


RESULT_COLUMNS_A = (
    "input_latency_ms",
    "mapping_time_ms",
    "planning_time_ms",
    "number_of_fails",
)

RESULT_COLUMNS_B = (
    "input_latency_ms",
    "average_planner_execution_time_ms",
    "robot_arrival_time_ms",
    "total_plans_made",
    "number_of_collisions",
    "resources_used",
)


def export_placeholder_results(destination: str | Path, format_name: str) -> Path:
    """Write headings only; metric collection is intentionally deferred."""
    columns = RESULT_COLUMNS_A if format_name.lower() == "a" else RESULT_COLUMNS_B
    path = Path(destination)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        csv.writer(stream).writerow(columns)
    return path


class ExperimentRecorder:
    """Append lifecycle events without pretending deferred metrics exist."""

    def __init__(
        self,
        scenario_name: str,
        planner_name: str,
        folder: str | Path = "results",
    ) -> None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        safe_scenario = "_".join(scenario_name.strip().split()) or "scenario"
        self.folder = Path(folder) / f"{stamp}_{safe_scenario}"
        self.folder.mkdir(parents=True, exist_ok=True)
        self.path = self.folder / "events.jsonl"
        self._stream = self.path.open("a", encoding="utf-8")
        self.record("run_started", scenario=scenario_name, planner=planner_name)

    def record(self, event: str, **payload) -> None:
        row = {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "event": event,
            **payload,
        }
        self._stream.write(json.dumps(row, separators=(",", ":")) + "\n")
        self._stream.flush()

    def close(self) -> None:
        if not self._stream.closed:
            self._stream.close()

    @property
    def closed(self) -> bool:
        return self._stream.closed
