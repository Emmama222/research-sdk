"""Research-session state, planner discovery, and measured result exports."""

from __future__ import annotations

import csv
import importlib
import inspect
import json
import pkgutil
from dataclasses import dataclass, field
from datetime import UTC, datetime
from enum import Enum
from math import hypot
from pathlib import Path
from statistics import fmean
from time import perf_counter, process_time

import research_sdk.planners as planner_package
from research_sdk.config import ROBOT_RADIUS_MM
from research_sdk.planners.common import StepRecorder
from research_sdk.ui.scenarios import ScenarioObstacle
from research_sdk.world.map.voronoi.voronoi_generator import generate_bounded_voronoi_map
from research_sdk.world.scene import PlanningObstacle

Point = tuple[float, float]
Edge = tuple[Point, Point]


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
        if self.state is SessionState.RUNNING:
            raise RuntimeError("Stop the session before resetting")
        self.has_scenario = False
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


def planner_debug_geometry(
    label: str,
    recorder: StepRecorder,
    obstacles: tuple[ScenarioObstacle, ...],
) -> tuple[tuple[Point, ...], tuple[Edge, ...]]:
    """Return the sampled nodes/edges behind one planner's last ``plan()`` call.

    Shared by the Scenario Planner's offline preview and the Execution
    Console's "Map" layer toggle so both draw the same debug graph. Voronoi
    has no ``StepRecorder`` sample/edge steps of its own -- its graph is
    rebuilt directly from ``generate_bounded_voronoi_map`` instead.
    """
    nodes: list[Point] = []
    edges: list[Edge] = []
    for step in recorder.steps:
        if step["kind"] == "sample" and step.get("free"):
            nodes.append(tuple(step["point"]))
        elif step["kind"] == "edge_test" and step.get("accepted"):
            first = tuple(step["a"])
            second = tuple(step["b"])
            edges.append((first, second))
            nodes.extend((first, second))
    if "Voronoi" in label:
        planning_obstacles = tuple(
            PlanningObstacle(
                robot_id=obstacle.obstacle_id,
                isYellow=obstacle.is_yellow,
                pos_mm=obstacle.position_mm,
                radius_mm=obstacle.radius_mm,
            )
            for obstacle in obstacles
        )
        voronoi_map = generate_bounded_voronoi_map(obstacles=planning_obstacles)
        by_id = {node.id: (node.x, node.y) for node in voronoi_map.nodes}
        nodes = list(by_id.values())
        edges = [
            (by_id[edge.start_id], by_id[edge.end_id])
            for edge in voronoi_map.edges
            if edge.start_id in by_id and edge.end_id in by_id
        ]
    return tuple(dict.fromkeys(nodes)), tuple(edges)


RESULT_COLUMNS_A = (
    "input_latency_ms",
    "mapping_time_ms",
    "planning_time_ms",
    "number_of_fails",
)

PLANNER_RESULT_COLUMN = "planner"

RESULT_COLUMNS_B = (
    "input_latency_ms",
    "average_planner_execution_time_ms",
    "robot_arrival_time_ms",
    "total_plans_made",
    "number_of_collisions",
    "resources_used",
)


@dataclass(slots=True)
class RunMetrics:
    """Accumulate one run's measurements using monotonic in-process clocks."""

    input_latency_samples_ms: list[float] = field(default_factory=list)
    mapping_time_samples_ms: list[float] = field(default_factory=list)
    planner_execution_samples_ms: list[float] = field(default_factory=list)
    number_of_fails: int = 0
    number_of_collisions: int = 0
    started_at: float = field(default_factory=perf_counter)
    cpu_started_at: float = field(default_factory=process_time)
    execution_started_at: float | None = None
    finished_at: float | None = None
    cpu_finished_at: float | None = None
    robot_arrival_time_ms: float | None = None
    _active_collision_pairs: set[tuple[tuple[bool, int], tuple[bool, int]]] = field(
        default_factory=set, repr=False
    )

    def record_pipeline(self, input_latency_ms: float, mapping_time_ms: float) -> None:
        self.input_latency_samples_ms.append(float(input_latency_ms))
        self.mapping_time_samples_ms.append(float(mapping_time_ms))

    def record_planning(self, durations_ms, failures: int = 0) -> None:
        self.planner_execution_samples_ms.extend(float(value) for value in durations_ms)
        self.number_of_fails += int(failures)

    def start_execution(self, *, now: float | None = None) -> None:
        self.execution_started_at = perf_counter() if now is None else now

    def observe_robots(self, robots) -> None:
        values = list(robots.values())
        colliding = set()
        for index, first in enumerate(values):
            for second in values[index + 1 :]:
                if (
                    hypot(
                        first.position_mm[0] - second.position_mm[0],
                        first.position_mm[1] - second.position_mm[1],
                    )
                    <= 2.0 * ROBOT_RADIUS_MM
                ):
                    pair = tuple(
                        sorted(
                            ((first.is_yellow, first.robot_id), (second.is_yellow, second.robot_id))
                        )
                    )
                    colliding.add(pair)
        self.number_of_collisions += len(colliding - self._active_collision_pairs)
        self._active_collision_pairs = colliding

    def finish(
        self,
        *,
        completed: bool,
        now: float | None = None,
        cpu_now: float | None = None,
    ) -> None:
        if self.finished_at is not None:
            return
        self.finished_at = perf_counter() if now is None else now
        self.cpu_finished_at = process_time() if cpu_now is None else cpu_now
        if completed and self.execution_started_at is not None:
            self.robot_arrival_time_ms = (self.finished_at - self.execution_started_at) * 1000.0

    @staticmethod
    def _mean(values: list[float]) -> float | None:
        return fmean(values) if values else None

    @staticmethod
    def _csv_value(value: float | None):
        return "" if value is None else round(value, 6) if isinstance(value, float) else value

    def row(self, format_name: str) -> dict[str, float | int | str]:
        cpu_end = self.cpu_finished_at if self.cpu_finished_at is not None else process_time()
        common_input = self._csv_value(self._mean(self.input_latency_samples_ms))
        if format_name.lower() == "a":
            return {
                "input_latency_ms": common_input,
                "mapping_time_ms": self._csv_value(self._mean(self.mapping_time_samples_ms)),
                "planning_time_ms": self._csv_value(sum(self.planner_execution_samples_ms)),
                "number_of_fails": self.number_of_fails,
            }
        if format_name.lower() != "b":
            raise ValueError("Result format must be 'a' or 'b'")
        return {
            "input_latency_ms": common_input,
            "average_planner_execution_time_ms": self._csv_value(
                self._mean(self.planner_execution_samples_ms)
            ),
            "robot_arrival_time_ms": self._csv_value(self.robot_arrival_time_ms),
            "total_plans_made": len(self.planner_execution_samples_ms),
            "number_of_collisions": self.number_of_collisions,
            "resources_used": self._csv_value((cpu_end - self.cpu_started_at) * 1000.0),
        }


def export_results(
    destination: str | Path,
    format_name: str,
    metrics: RunMetrics,
) -> Path:
    """Write one fully calculated result row for a run."""
    columns = RESULT_COLUMNS_A if format_name.lower() == "a" else RESULT_COLUMNS_B
    row = metrics.row(format_name)
    path = Path(destination)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerow(row)
    return path


def export_planner_results(
    destination: str | Path,
    format_name: str,
    planner_metrics: dict[str, RunMetrics],
) -> Path:
    """Write one labeled result row per planner from a comparison run."""
    result_columns = RESULT_COLUMNS_A if format_name.lower() == "a" else RESULT_COLUMNS_B
    columns = (PLANNER_RESULT_COLUMN, *result_columns)
    path = Path(destination)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for planner_name, metrics in planner_metrics.items():
            writer.writerow({PLANNER_RESULT_COLUMN: planner_name, **metrics.row(format_name)})
    return path


class ExperimentRecorder:
    """Record lifecycle events and the calculated metrics for one run."""

    def __init__(
        self,
        scenario_name: str,
        planner_name: str,
        folder: str | Path = "results",
    ) -> None:
        stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
        safe_scenario = "_".join(scenario_name.strip().split()) or "scenario"
        self.folder = Path(folder) / f"{stamp}_{safe_scenario}"
        self.folder.mkdir(parents=True, exist_ok=True)
        self.path = self.folder / "events.jsonl"
        self._stream = self.path.open("a", encoding="utf-8")
        self.metrics = RunMetrics()
        self.record("run_started", scenario=scenario_name, planner=planner_name)

    def finish(self, *, completed: bool) -> None:
        self.metrics.finish(completed=completed)
        self.record(
            "metrics_finalized",
            completed=completed,
            format_a=self.metrics.row("a"),
            format_b=self.metrics.row("b"),
        )

    def export(self, destination: str | Path, format_name: str) -> Path:
        return export_results(destination, format_name, self.metrics)

    def record(self, event: str, **payload) -> None:
        row = {
            "timestamp_utc": datetime.now(UTC).isoformat(),
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
