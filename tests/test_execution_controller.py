import pytest

from research_sdk.ui.execution.controller import (
    ExecutionController,
    ExecutionInput,
    ExecutionState,
)
from research_sdk.ui.runtime import PlannedRobotPath
from research_sdk.ui.scenarios import Scenario, ScenarioRobot


class PlannerA:
    pass


class PlannerB:
    pass


def execution_input() -> ExecutionInput:
    scenario = Scenario(
        "course",
        robots=[ScenarioRobot(1, True, (0.0, 0.0), (1000.0, 0.0))],
    )
    path = PlannedRobotPath(1, True, ((0.0, 0.0), (1000.0, 0.0)))
    return ExecutionInput.create(
        scenario,
        {"Planner A": (path,), "Planner B": (path,)},
        {"Planner A": PlannerA, "Planner B": PlannerB},
    )


def ready_controller() -> ExecutionController:
    controller = ExecutionController()
    controller.load(execution_input())
    controller.begin_apply()
    controller.confirm_apply()
    return controller


def test_initial_state_is_no_scenario() -> None:
    assert ExecutionController().state is ExecutionState.NO_SCENARIO


def test_run_requires_ready_and_claims_exclusive_owner() -> None:
    controller = ExecutionController()
    controller.load(execution_input())
    with pytest.raises(RuntimeError):
        controller.run("Planner A")
    controller.begin_apply()
    controller.confirm_apply()

    paths = controller.run("Planner A")

    assert paths
    assert controller.velocity_owner == "Planner A"
    assert controller.state is ExecutionState.RUNNING
    assert controller.selections_locked
    with pytest.raises(RuntimeError):
        controller.run("Planner B")


def test_shadow_selection_locks_until_reset() -> None:
    controller = ready_controller()
    controller.set_shadow("Planner B", True)
    controller.run("Planner A")

    with pytest.raises(RuntimeError):
        controller.set_shadow("Planner B", False)

    controller.stop()
    controller.begin_reset()
    controller.reset_without_apply()
    assert not controller.selections_locked
    assert not controller.shadow_planners


def test_pause_continue_and_complete_transitions() -> None:
    controller = ready_controller()
    controller.run("Planner A")
    controller.pause()
    assert controller.state is ExecutionState.PAUSED
    controller.continue_run()
    assert controller.state is ExecutionState.RUNNING
    controller.complete()
    assert controller.state is ExecutionState.COMPLETED


def test_content_hash_is_stable_for_same_scenario() -> None:
    assert execution_input().content_hash == execution_input().content_hash
