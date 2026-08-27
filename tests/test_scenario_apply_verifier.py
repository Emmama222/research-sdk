from research_sdk.ui.execution.apply_verifier import ScenarioApplyVerifier
from research_sdk.ui.scenarios import Scenario, ScenarioRobot
from research_sdk.world.snapshot import RobotSnapshot, WorldSnapshot, empty_robot_team


def snapshot(*, x: float, theta: float = 0.0, timestamp: float = 10.0) -> WorldSnapshot:
    yellow = list(empty_robot_team())
    yellow[1] = RobotSnapshot(True, 1, x, 0.0, theta)
    return WorldSnapshot(
        version=1,
        timestamp=timestamp,
        frame_number=1,
        ball=None,
        yellow=tuple(yellow),
        blue=empty_robot_team(),
        us_yellow=True,
        us_positive=True,
    )


def scenario() -> Scenario:
    return Scenario(
        "apply",
        robots=[ScenarioRobot(1, True, (100.0, 0.0), (1000.0, 0.0))],
    )


def test_verifier_requires_consecutive_matching_snapshots() -> None:
    verifier = ScenarioApplyVerifier(scenario(), stable_snapshots_required=3, started_at=10.0)

    assert not verifier.observe(snapshot(x=100.0), now=10.01).ready
    assert not verifier.observe(snapshot(x=100.0), now=10.02).ready
    report = verifier.observe(snapshot(x=100.0), now=10.03)

    assert report.ready
    assert report.confirmed == report.expected == 1


def test_mismatch_resets_stable_count_and_reports_error() -> None:
    verifier = ScenarioApplyVerifier(scenario(), stable_snapshots_required=2, started_at=10.0)
    verifier.observe(snapshot(x=100.0), now=10.01)

    report = verifier.observe(snapshot(x=500.0), now=10.02)

    assert report.stable_snapshots == 0
    assert report.maximum_position_error_mm == 400.0
    assert report.mismatches


def test_missing_robot_cannot_confirm() -> None:
    verifier = ScenarioApplyVerifier(scenario(), stable_snapshots_required=1, started_at=10.0)
    missing = WorldSnapshot(
        version=1,
        timestamp=10.0,
        frame_number=1,
        ball=None,
        yellow=empty_robot_team(),
        blue=empty_robot_team(),
        us_yellow=True,
        us_positive=True,
    )

    report = verifier.observe(missing, now=10.01)

    assert not report.ready
    assert report.mismatches == ("Y1 missing",)
