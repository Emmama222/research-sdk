"""Revised ACRA result model (DEC-016/017/018): outcomes, swept contacts,
initial retries, per-robot stops, route lifetime, scenario bank and tables."""

import csv
import json
from dataclasses import replace
from itertools import pairwise

import pytest

from research_sdk import headless
from research_sdk.headless import (
    SimulationConfig,
    _build_cases,
    _parser,
    _swept_min_distance,
    classify_outcome,
    load_scenario_bank,
    run_experiments,
    simulate,
    summarize_one_shot,
    summarize_outcomes,
    validate_one_shot,
    write_results,
)
from research_sdk.ui.scenarios import Scenario, ScenarioObstacle, ScenarioRobot


def _straight(obstacles=()) -> Scenario:
    return Scenario(
        "straight",
        robots=[ScenarioRobot(0, False, (-2000.0, 0.0), (2000.0, 0.0))],
        obstacles=list(obstacles),
    )


def _crossing() -> Scenario:
    return Scenario(
        "crossing",
        robots=[
            ScenarioRobot(0, False, (-2500.0, 0.0), (2500.0, 0.0)),
            ScenarioRobot(1, False, (0.0, -2500.0), (0.0, 2500.0)),
        ],
    )


# --- geometry and classification -------------------------------------------


def test_swept_distance_finds_contact_between_two_samples() -> None:
    # Endpoints are 200 mm and 200 mm apart, but the paths pass through each other.
    start_gap = _swept_min_distance((-200.0, 0.0), (-200.0, 0.0), (0.0, 0.0), (0.0, 0.0))
    assert start_gap == pytest.approx(200.0)
    crossing = _swept_min_distance((-200.0, 0.0), (200.0, 0.0), (0.0, 0.0), (0.0, 0.0))
    assert crossing == pytest.approx(0.0)


@pytest.mark.parametrize(
    ("completed", "contacts", "buffers", "overlap", "expected"),
    [
        (True, 0, 0, False, "S"),
        (True, 0, 2, False, "B"),
        (True, 1, 3, False, "P"),
        (False, 0, 0, False, "I"),
        (False, 4, 4, False, "I"),
        (True, 0, 0, True, "X"),
    ],
)
def test_outcome_is_highest_severity(completed, contacts, buffers, overlap, expected) -> None:
    assert (
        classify_outcome(
            completed=completed,
            contact_episodes=contacts,
            buffer_episodes=buffers,
            initial_overlap=overlap,
        )
        == expected
    )


# --- simulation behaviour -------------------------------------------------


def test_open_loop_crossing_is_one_physical_contact_episode() -> None:
    result = simulate(_crossing(), "voronoi", config=SimulationConfig(replan_policy="once"))
    assert result.completed
    assert result.outcome == "P"
    assert result.rr_contact_episodes == 1  # a sustained overlap counts once
    assert result.rr_buffer_episodes >= result.rr_contact_episodes
    assert result.min_swept_clearance_mm <= 0.0
    assert result.first_contact_ms is not None


def test_obstacle_obstacle_overlap_is_ignored() -> None:
    # Two overlapping obstacles far from the robot's path.
    obstacles = [
        ScenarioObstacle(1, True, (0.0, 2500.0), 90.0),
        ScenarioObstacle(2, True, (100.0, 2500.0), 90.0),
    ]
    result = simulate(_straight(obstacles), "visibility")
    assert result.outcome == "S"
    assert result.ro_contact_episodes == 0 and not result.initial_overlap


def test_initial_overlap_marks_the_run_invalid() -> None:
    obstacles = [ScenarioObstacle(1, True, (-2000.0, 50.0), 90.0)]
    result = simulate(_straight(obstacles), "visibility")
    assert result.initial_overlap
    assert result.outcome == "X"


class _FlakyPlanner:
    """Raises for the first ``failures`` calls, then delegates to a real planner."""

    def __init__(self, inner, failures: int | None) -> None:
        self.inner = inner
        self.failures = failures
        self.calls = 0

    def plan(self, planner_input):
        self.calls += 1
        if self.failures is None or self.calls <= self.failures:
            raise RuntimeError("no route")
        return self.inner.plan(planner_input)

    def reset(self, **kwargs):
        return self.inner.reset(**kwargs)


def _patch_planner(monkeypatch, failures: int | None) -> None:
    real = headless._new_planner

    def flaky(name, seed, policy="once", periodic_reroute_frames=None):
        return _FlakyPlanner(real(name, seed, policy, periodic_reroute_frames), failures)

    monkeypatch.setattr(headless, "_new_planner", flaky)


def test_initial_no_route_waits_and_retries(monkeypatch) -> None:
    _patch_planner(monkeypatch, failures=3)
    config = SimulationConfig(replan_policy="event", control_period_s=0.02)
    result = simulate(_straight(), "visibility", config=config)
    assert result.completed and result.outcome == "S"
    assert result.initial_retries_total == 3
    assert result.initial_wait_ms_total == pytest.approx(60.0)
    assert result.robots_stopped_no_path == 0


def test_robot_without_route_for_1000_ms_fails_the_episode(monkeypatch) -> None:
    _patch_planner(monkeypatch, failures=None)
    config = SimulationConfig(replan_policy="event", control_period_s=0.02)
    result = simulate(_straight(), "visibility", config=config)
    assert not result.completed and result.outcome == "I"
    assert result.episode_end_reason == "no_valid_path"
    assert "no_valid_path@1000ms" in result.stopped_robots
    assert result.simulated_duration_ms == pytest.approx(1000.0)
    assert result.initial_wait_ms_max == pytest.approx(1000.0)
    assert result.initial_retries_total == 51  # attempts at 0, 20, ..., 1000 ms


def test_no_route_ends_the_episode_for_the_whole_team(monkeypatch) -> None:
    real = headless._new_planner

    class _FailRobotOne:
        def __init__(self, inner):
            self.inner = inner

        def plan(self, planner_input):
            if planner_input.robot_id == 1:
                raise RuntimeError("no route")
            return self.inner.plan(planner_input)

        def reset(self, **kwargs):
            return self.inner.reset(**kwargs)

    monkeypatch.setattr(headless, "_new_planner", lambda *a, **k: _FailRobotOne(real(*a, **k)))
    scenario = Scenario(
        "two",
        robots=[
            ScenarioRobot(0, False, (-2000.0, 1500.0), (2000.0, 1500.0)),
            ScenarioRobot(1, False, (-2000.0, -1500.0), (2000.0, -1500.0)),
        ],
    )
    result = simulate(scenario, "visibility", config=SimulationConfig(replan_policy="event"))
    assert result.episode_end_reason == "no_valid_path"
    assert result.outcome == "I"
    assert result.simulated_duration_ms <= 1020.0


def test_slow_replan_stops_only_that_robot_and_initial_plan_is_exempt() -> None:
    config = SimulationConfig(replan_policy="cycle", control_period_s=0.02, slow_call_limit_ms=1e-6)
    result = simulate(_crossing(), "visibility", config=config)
    assert result.initial_retries_total == 0  # the initial plans were not timed out
    assert result.robots_stopped_time_limit == 2  # first post-initial rebuild of each
    assert result.episode_end_reason == ""
    assert "time_limit" in result.stopped_robots
    assert result.outcome == "I"


def test_initial_plan_is_never_subject_to_the_replan_time_limit() -> None:
    config = SimulationConfig(replan_policy="once", slow_call_limit_ms=1e-6)
    result = simulate(_straight(), "visibility", config=config)
    assert result.completed and result.robots_stopped_time_limit == 0


def test_route_lifetime_covers_the_initial_route_until_arrival() -> None:
    result = simulate(_straight(), "visibility", config=SimulationConfig(replan_policy="once"))
    assert result.route_count == 1
    assert result.route_lifetime_ms_max == pytest.approx(result.simulated_duration_ms, abs=20.0)


# --- scenario bank --------------------------------------------------------


def _write_bank(folder, names) -> None:
    folder.mkdir()
    for name in names:
        scenario = replace(_straight(), name=name)
        (folder / f"{name}.json").write_text(json.dumps(scenario.to_dict()), encoding="utf-8")
    (folder / "README.md").write_text("not a scenario", encoding="utf-8")


def test_scenario_bank_loads_exactly_its_json_files(tmp_path) -> None:
    _write_bank(tmp_path / "bank", ["a", "b"])
    assert [s.name for s in load_scenario_bank(tmp_path / "bank", expected=2)] == ["a", "b"]
    with pytest.raises(ValueError, match="expected 3"):
        load_scenario_bank(tmp_path / "bank", expected=3)


def test_scenario_bank_rejects_extra_generated_scenarios(tmp_path) -> None:
    _write_bank(tmp_path / "bank", ["a", "b"])
    args = _parser().parse_args(["--scenario-bank", str(tmp_path / "bank"), "--random", "5"])
    with pytest.raises(ValueError, match="exactly the bank"):
        _build_cases(args)
    args = _parser().parse_args(["--scenario-bank", str(tmp_path / "bank")])
    scenarios, labels, _ = _build_cases(args)
    assert len(scenarios) == 2 and labels == ["bank", "bank"]


# --- arms and tables ------------------------------------------------------


def test_explicit_arms_run_only_those_arms() -> None:
    arms = (("cycle", 0.0, 20.0), ("event", 50.0, 20.0))
    results = run_experiments([_straight()], ["visibility"], arms=arms)
    got = {(r.replan_policy, r.prediction_horizon_ms, r.replan_period_ms) for r in results}
    assert got == {("cycle", 0.0, 20.0), ("event", 50.0, 20.0)}


def test_outcome_summary_uses_the_revised_denominators() -> None:
    base = replace(simulate(_straight(), "visibility"), scenario_set="bank")
    runs = [
        replace(base, scenario="s1", outcome="S", rebuild_latencies_ms=(1.0, 3.0)),
        replace(base, scenario="s2", outcome="S"),
        replace(base, scenario="s3", outcome="B", rr_buffer_episodes=1),
        replace(
            base, scenario="s4", outcome="P", rr_contact_episodes=2, ro_contact_episodes=1,
            rebuild_latencies_ms=(5.0,),
        ),
        replace(base, scenario="s5", outcome="I", completed=False, ro_contact_episodes=4),
        replace(base, scenario="s6", outcome="X", initial_overlap=True),
    ]
    (row,) = summarize_outcomes(runs)
    assert row["invalid_runs"] == 1 and row["valid_runs"] == 5
    assert row["completed_label"] == "4/5"
    assert row["strict_label"] == "2/4"
    assert row["buffer_only_label"] == "1/4"
    assert row["physical_label"] == "1/4"
    assert row["incomplete_label"] == "1/5"
    assert row["contacts_per_failed_run"] == pytest.approx(3 / 2)
    assert row["contacts_per_physical_run"] == pytest.approx(3.0)
    assert row["contacts_in_incomplete_runs"] == 4
    assert (row["replan_ms_min"], row["replan_ms_max"]) == (1.0, 5.0)
    assert row["replan_ms_mean"] == pytest.approx(3.0)


def test_exports_include_rebuild_calls_and_keep_runs_flat(tmp_path) -> None:
    config = SimulationConfig(replan_policy="cycle", control_period_s=0.02)
    results = [simulate(_crossing(), "visibility", config=config)]
    paths = write_results(results, tmp_path, config=config)
    with paths["runs_csv"].open(newline="") as stream:
        header = next(csv.reader(stream))
    assert "rebuild_latencies_ms" not in header and "outcome" in header
    with paths["rebuild_calls_csv"].open(newline="") as stream:
        calls = list(csv.DictReader(stream))
    assert len(calls) == results[0].successful_rebuilds


def test_one_shot_validation_reports_stage_timing_and_geometry() -> None:
    rows = validate_one_shot(_crossing(), "visibility")
    assert len(rows) == 2
    for row in rows:
        assert row["route_available"]
        assert 0.0 <= row["search_ms"] <= row["total_ms"]
        assert row["map_ms"] >= 0.0
        assert row["path_excess"] >= -1e-9
    (summary,) = summarize_one_shot(rows)
    assert summary["robot_routes_label"] == "2/2"
    assert summary["scenarios_all_routes_label"] == "1/1"


# --- gaps found in the code-adjustment-plan audit -------------------------


def _patrol_crossing(spawn_y: float, speed: float) -> Scenario:
    """A robot driving along y = 0 while a patrol obstacle crosses its line."""
    return Scenario(
        "patrol-crossing",
        robots=[ScenarioRobot(0, False, (-2000.0, 0.0), (2000.0, 0.0))],
        obstacles=[
            ScenarioObstacle(
                1, True, (0.0, spawn_y), 90.0,
                patrol_waypoints=((0.0, 1500.0),), patrol_speed_mmps=speed,
            )
        ],
    )


def test_recontact_after_separation_is_a_new_episode() -> None:
    pair = (("B", 0), ("B", 1))
    timeline = [set(), {pair}, {pair}, {pair}, set(), {pair}, set()]
    episodes = sum(
        headless.count_new_episodes(before, after) for before, after in pairwise(timeline)
    )
    assert episodes == 2  # the sustained overlap counts once, the recontact once more


def test_robot_obstacle_contact_is_kept_apart_from_robot_robot() -> None:
    result = simulate(
        _patrol_crossing(-500.0, 1250.0), "visibility", config=SimulationConfig(replan_policy="once")
    )
    assert result.outcome == "P"
    assert (result.ro_contact_episodes, result.rr_contact_episodes) == (1, 0)


def test_obstacle_initiated_contact_still_classifies_the_run_as_physical() -> None:
    result = simulate(
        _patrol_crossing(-700.0, 1250.0), "visibility", config=SimulationConfig(replan_policy="once")
    )
    assert result.ro_contact_obstacle_initiated == 1
    assert result.outcome == "P"


def test_failed_replan_does_not_end_a_still_valid_route(monkeypatch) -> None:
    real = headless._new_planner

    class _OnlyFirstCall:
        def __init__(self, inner):
            self.inner = inner
            self.calls = 0

        def plan(self, planner_input):
            self.calls += 1
            if self.calls > 1:
                raise RuntimeError("replan failed")
            return self.inner.plan(planner_input)

        def reset(self, **kwargs):
            return self.inner.reset(**kwargs)

    monkeypatch.setattr(headless, "_new_planner", lambda *a, **k: _OnlyFirstCall(real(*a, **k)))
    config = SimulationConfig(replan_policy="cycle", no_route_limit_ms=None)
    result = simulate(_straight(), "visibility", config=config)
    assert result.replan_failures > 0
    assert result.completed
    assert result.route_count == 1 and result.route_ends_goal == 1
    assert result.route_ends_replaced == 0


def test_route_end_reasons_cover_replacement_stop_and_goal() -> None:
    cycle = simulate(_crossing(), "visibility", config=SimulationConfig(replan_policy="cycle"))
    assert cycle.route_ends_replaced > 0
    assert cycle.route_count == (
        cycle.route_ends_replaced
        + cycle.route_ends_goal
        + cycle.route_ends_stopped
        + cycle.route_ends_episode_end
    )
    stopped = simulate(
        _crossing(),
        "visibility",
        config=SimulationConfig(replan_policy="cycle", slow_call_limit_ms=1e-6),
    )
    assert stopped.route_ends_stopped == 2


def test_initial_route_is_reported_per_robot(monkeypatch) -> None:
    _patch_planner(monkeypatch, failures=3)
    config = SimulationConfig(replan_policy="event", control_period_s=0.02)
    result = simulate(_straight(), "visibility", config=config)
    assert result.initial_route_by_robot == "B0:3r/60ms"


def test_rebuild_latency_statistics_match_the_raw_call_log() -> None:
    config = SimulationConfig(replan_policy="cycle", control_period_s=0.02)
    result = simulate(_crossing(), "voronoi", config=config)
    raw = result.rebuild_latencies_ms
    assert raw and result.successful_rebuilds == len(raw)
    assert (result.rebuild_ms_min, result.rebuild_ms_max) == (min(raw), max(raw))
    assert result.rebuild_ms_total == pytest.approx(sum(raw))
    (row,) = summarize_outcomes([result])
    assert row["replan_ms_p95"] == headless._percentile(list(raw), 0.95)
    assert row["replan_ms_mean"] == pytest.approx(sum(raw) / len(raw))


def test_timing_stages_sum_to_the_planning_call() -> None:
    for row in validate_one_shot(_crossing(), "prm"):
        assert row["map_ms"] + row["search_ms"] == pytest.approx(row["total_ms"])
    result = simulate(_crossing(), "visibility", config=SimulationConfig(replan_policy="once"))
    assert result.initial_search_ms_total + result.initial_map_ms_total == pytest.approx(
        result.planning_time_ms_initial
    )


def test_bank_run_does_not_copy_the_scenarios_into_the_results(tmp_path) -> None:
    _write_bank(tmp_path / "bank", ["a", "b"])
    out = tmp_path / "out"
    code = headless.main(
        ["--scenario-bank", str(tmp_path / "bank"), "--planner", "visibility", "--output-dir", str(out)]
    )
    assert code == 0
    assert not (out / "scenarios").exists()
    manifest = json.loads((out / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["scenario_bank_files"] == 2 and len(manifest["scenario_bank_sha256"]) == 64
    assert manifest["scenarios"] == ["a", "b"]


@pytest.mark.parametrize("planner_name", ["voronoi", "prm", "visibility"])
def test_clear_straight_line_is_a_check_not_a_rebuild(planner_name: str) -> None:
    # Nothing is in the way, so every cycle call only confirms the direct line.
    config = SimulationConfig(replan_policy="cycle", control_period_s=0.02)
    result = simulate(_straight(), planner_name, config=config)
    assert result.completed
    assert result.successful_rebuilds == 0
    assert result.event_checks > 0


def test_failed_replans_are_counted_but_kept_out_of_latency(monkeypatch) -> None:
    real = headless._new_planner

    class _OnlyFirstCall:
        def __init__(self, inner):
            self.inner = inner
            self.calls = 0

        def plan(self, planner_input):
            self.calls += 1
            if self.calls > 1:
                raise RuntimeError("replan failed")
            return self.inner.plan(planner_input)

        def reset(self, **kwargs):
            return self.inner.reset(**kwargs)

    monkeypatch.setattr(headless, "_new_planner", lambda *a, **k: _OnlyFirstCall(real(*a, **k)))
    config = SimulationConfig(replan_policy="cycle", no_route_limit_ms=None)
    result = simulate(_crossing(), "visibility", config=config)
    assert result.replan_failures > 0
    assert result.successful_rebuilds == 0 and result.rebuild_latencies_ms == ()
    (row,) = summarize_outcomes([result])
    assert row["replan_ms_mean"] is None
    assert row["failed_replan_share"] == 1.0


def test_supporting_result_sets_and_overrides() -> None:
    assert headless.RESULT_SETS["event-route-ablation"] == (
        ("event_route", 0.0, 20.0),
        ("event_route", 0.0, 100.0),
    )
    assert headless.RESULT_SETS["horizon-150-check"] == (("event", 150.0, 20.0),)
    assert headless.RESULT_SET_OVERRIDES["no-route-500-check"] == {"no_route_limit_ms": 500.0}


def test_no_route_500_check_runs_with_the_500_ms_limit(tmp_path) -> None:
    _write_bank(tmp_path / "bank", ["a"])
    out = tmp_path / "out"
    code = headless.main(
        ["--result-set", "no-route-500-check", "--scenario-bank", str(tmp_path / "bank"),
         "--planner", "visibility", "--output-dir", str(out)]
    )
    assert code == 0
    manifest = json.loads((out / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["config"]["no_route_limit_ms"] == 500.0
    assert manifest["result_set_overrides"] == {"no_route_limit_ms": 500.0}


def test_collision_free_completion_counts_strict_and_buffer_only() -> None:
    base = replace(simulate(_straight(), "visibility"), scenario_set="bank")
    runs = [
        replace(base, scenario="s1", outcome="S"),
        replace(base, scenario="s2", outcome="B"),
        replace(base, scenario="s3", outcome="P"),
        replace(base, scenario="s4", outcome="I", completed=False),
    ]
    (row,) = summarize_outcomes(runs)
    assert row["collision_free_completed_label"] == "2/4"
    assert row["collision_free_completion_rate"] == 0.5
