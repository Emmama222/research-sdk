"""Vision-backed verification for grSim replacement packets."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, sin
from time import monotonic

from research_sdk.ui.scenarios import Scenario
from research_sdk.world.snapshot import WorldSnapshot


@dataclass(frozen=True, slots=True)
class ApplyReport:
    confirmed: int
    expected: int
    maximum_position_error_mm: float | None
    maximum_orientation_error_rad: float | None
    vision_age_ms: float | None
    stable_snapshots: int
    ready: bool
    mismatches: tuple[str, ...] = ()


class ScenarioApplyVerifier:
    def __init__(
        self,
        scenario: Scenario,
        *,
        position_tolerance_mm: float = 75.0,
        orientation_tolerance_rad: float = 0.10,
        stable_snapshots_required: int = 3,
        timeout_s: float = 2.0,
        started_at: float | None = None,
    ) -> None:
        self.scenario = scenario
        self.position_tolerance_mm = float(position_tolerance_mm)
        self.orientation_tolerance_rad = float(orientation_tolerance_rad)
        self.stable_snapshots_required = int(stable_snapshots_required)
        self.timeout_s = float(timeout_s)
        self.started_at = monotonic() if started_at is None else float(started_at)
        self.stable_snapshots = 0

    @property
    def timed_out(self) -> bool:
        return monotonic() - self.started_at > self.timeout_s

    def observe(self, snapshot: WorldSnapshot, *, now: float | None = None) -> ApplyReport:
        observed_at = monotonic() if now is None else float(now)
        expected = [
            (robot.is_yellow, robot.robot_id, robot.start_mm, robot.orientation_rad)
            for robot in self.scenario.robots
        ] + [
            (obstacle.is_yellow, obstacle.obstacle_id, obstacle.position_mm, 0.0)
            for obstacle in self.scenario.obstacles
        ]
        position_errors = []
        orientation_errors = []
        mismatches = []
        confirmed = 0
        for is_yellow, robot_id, position, orientation in expected:
            observed = snapshot.robot(is_yellow, robot_id)
            label = f"{'Y' if is_yellow else 'B'}{robot_id}"
            if observed is None or not observed.visible:
                mismatches.append(f"{label} missing")
                continue
            position_error = hypot(observed.x - position[0], observed.y - position[1])
            orientation_error = abs(
                atan2(sin(observed.theta - orientation), cos(observed.theta - orientation))
            )
            position_errors.append(position_error)
            orientation_errors.append(orientation_error)
            if (
                position_error <= self.position_tolerance_mm
                and orientation_error <= self.orientation_tolerance_rad
            ):
                confirmed += 1
            else:
                mismatches.append(
                    f"{label}: {position_error:.1f} mm, {orientation_error:.3f} rad"
                )
        all_match = bool(expected) and confirmed == len(expected)
        self.stable_snapshots = self.stable_snapshots + 1 if all_match else 0
        vision_age_ms = max(0.0, (observed_at - snapshot.timestamp) * 1000.0)
        return ApplyReport(
            confirmed=confirmed,
            expected=len(expected),
            maximum_position_error_mm=max(position_errors, default=None),
            maximum_orientation_error_rad=max(orientation_errors, default=None),
            vision_age_ms=vision_age_ms,
            stable_snapshots=self.stable_snapshots,
            ready=self.stable_snapshots >= self.stable_snapshots_required,
            mismatches=tuple(mismatches),
        )
