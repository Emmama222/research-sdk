# grSim Execution Console specification

Status: initial implementation complete; live grSim acceptance testing pending

Implementation note (2026-08-25): the main Execution tab, state/ownership
controller, vision-backed Apply gate, pause/continue/step controls, replay,
emergency-stop path, live field indicators, result
tables/exports, connection tests, and automated tests are now present. The
remaining acceptance step is an opt-in run against a real grSim instance; it
is intentionally not part of the no-network unit-test suite.

Reference name: **Execution Console Plan v1**

Source: annotated grSim main-page sketch and follow-up decisions, 2026-08-24

This document specifies the main page that loads a planned scenario, applies
it to grSim, runs exactly one planner as the velocity owner, observes other
planners in shadow mode, records measurements, and stops
safely. Scenario creation and preview remain in the Scenario Planner tab.

## Core safety rule

Exactly one planner may own velocity output for a robot at a time.

- A checked planner runs in **shadow mode**: it may map, plan, and record
  measurements, but it cannot send robot commands.
- Pressing one planner's Run button makes that planner the **velocity owner**.
- Every other Run button and every planner checkbox is disabled immediately.
- Planner selections stay locked through Running, Paused, Completed, Stopped,
  and Error. Reset E-Stop restores the scenario and unlocks them.
- Shadow/background planners are highlighted yellow.
- The velocity-owning planner is highlighted green.
- Inactive planners use the normal/grey style; failed planners use red.

This ownership check must also exist in the runtime/controller, not only as a
disabled-widget rule.

## Layout

The page follows the existing Qt/Fusion styling and contains:

- top-left connection and motion-feedback tests;
- top-centre execution state and transport controls;
- top-right scenario selection and Scenario Planner navigation;
- a large live grSim field on the left;
- planner shadow/Run controls on the right;
- scrollable Result A and Result B tables on the lower right;
- live timing and plan counters below the field; and
- Result A and Result B CSV export buttons.

The current execution state is prominent at the top centre, immediately above
the pause/continue/step/replay/reset controls.

## Execution state machine

```text
NO_SCENARIO
  -> SCENARIO_LOADED
  -> APPLYING
  -> READY
  -> RUNNING
  -> PAUSED
  -> COMPLETED

RUNNING / PAUSED / APPLYING
  -> STOPPED or ERROR

COMPLETED / STOPPED / ERROR
  -> RESETTING
  -> SCENARIO_LOADED or NO_SCENARIO
```

### State meanings

| State | Meaning |
| --- | --- |
| `NO_SCENARIO` | No execution input is loaded. No planner can run. |
| `SCENARIO_LOADED` | Scenario and preview paths are loaded locally but have not been confirmed in grSim. |
| `APPLYING` | A replacement packet was sent and the console is waiting for vision confirmation. |
| `READY` | The requested starting state was confirmed and a planner may be selected to run. |
| `RUNNING` | One planner owns velocity output; checked alternatives run in shadow mode. |
| `PAUSED` | Zero velocity is being sent and waypoint progress is retained. |
| `COMPLETED` | Every active robot reached its target and zero velocity was sent. |
| `STOPPED` | The operator stopped execution without completing it. |
| `ERROR` | A fatal execution, vision, planner, or control error forced an emergency stop. |
| `RESETTING` | All robots are stopped and the last loaded scenario is being restored. |

## Button and control rules

| Control | Enabled states | Effect |
| --- | --- | --- |
| Refresh scenarios | `NO_SCENARIO`, `SCENARIO_LOADED`, `READY` | Rescan scenario JSON files in the workspace. |
| Load scenario | `NO_SCENARIO`, `SCENARIO_LOADED`, `READY` | Load the selected scenario locally; does not send packets. |
| Load scenario into grSim | `SCENARIO_LOADED` | Send replacement state and enter `APPLYING`. |
| Planner checkbox | `READY` only | Enable/disable shadow planning. Locked after any Run until Reset E-Stop. |
| Planner Run | `READY` only | Claim velocity ownership, lock all planners, and enter `RUNNING`. |
| Pause | `RUNNING` | Send zero velocity and enter `PAUSED`, preserving waypoint indexes. |
| Continue | `PAUSED` | Resume from current robot positions and current waypoint progress. |
| Step | `PAUSED` | Move toward each robot's next waypoint, stop when that step completes, then return to `PAUSED`. |
| Replay from start | `COMPLETED`, `STOPPED`, `ERROR` | Restore scenario start, create a new run ID, reset current metrics, and run the same planner configuration. |
| Reset E-Stop | All states except `NO_SCENARIO` | Send zero commands first, restore the last loaded scenario, wait for vision confirmation, clear the stopped/error latch, and unlock planner selection. |
| Emergency Stop | Always | Best-effort zero velocity for every active robot and enter `STOPPED` or `ERROR`. |

No state transition may depend only on button text or color; the controller
owns the authoritative state.

## Scenario selection and application

The scenario selector lists available workspace scenario JSON files. Refresh
rescans the scenario folder. Add Scenario navigates to the Scenario Planner
tab rather than creating or editing a course on this page.

Loading a scenario is local. **Load scenario into grSim** performs application:

1. validate robot identities, starts, targets, and selected planner paths;
2. send the grSim replacement packet for robots, obstacles, orientations, and
   ball state when available;
3. record the requested positions and apply timestamp;
4. observe complete `WorldSnapshot` updates from SSL-Vision;
5. match each requested robot by team and ID;
6. require every robot to be visible and within a configurable position and
   orientation tolerance for a stable confirmation window;
7. transition to `READY` only after confirmation; and
8. otherwise remain non-runnable and report the mismatches.

The confirmation panel displays at least:

```text
Applied: 7/7 robots confirmed
Maximum position error: 34 mm
Maximum orientation error: 0.04 rad
Vision age: 18 ms
```

Recommended initial thresholds are 75 mm position error, 0.10 rad orientation
error, three consecutive complete snapshots, and a two-second timeout. These
values belong in validated configuration rather than being hidden constants.

## Connection tests

### UDP route test

A non-motion test verifies that the command destination and local socket route
can be created. It must not claim that UDP receipt was acknowledged.

### Motion feedback test

This is an explicit, confirmed, end-to-end test:

1. require healthy SSL-Vision and no active execution;
2. ask the operator to confirm the team, robot ID, angular command, and test
   duration;
3. by default command yellow robot 0 at `w = 3 rad/s` for one second;
4. observe orientation changes from SSL-Vision;
5. always send zero velocity in a `finally`-equivalent safety path;
6. show command sent, feedback observed, measured angular velocity, and a
   pass/fail explanation; and
7. disable the test while applying, running, paused, or resetting.

If feedback is missing or inconsistent, the test fails safely and leaves the
robot stopped.

## Pause, continue, and step

- Pause sends zero velocity immediately and retains each robot's current
  waypoint index.
- Continue computes commands from the robot's current observed position toward
  its retained next waypoint; it does not teleport or restart the segment.
- Step is available only while paused. It advances each active robot to its
  next waypoint, sends zero velocity when the step boundary is reached, and
  returns to Paused.

## Replay

**Replay from start** is the reproducible experiment operation: it restores the
scenario start, creates a new run ID, resets current metrics, and re-runs the
same planner configuration.

### Exact Replay from Start algorithm

1. Require `COMPLETED`, `STOPPED`, or `ERROR` and a retained loaded scenario.
2. Call Emergency Stop and wait for the stop attempt to finish.
3. Finalize and close the previous run without modifying its result rows.
4. Create a new normal experiment run ID with `replay_of_run_id`.
5. Reset planner instances, waypoint indexes, collision episodes, timers, and
   current Result A/B rows.
6. Send the original scenario start replacement packet.
7. Wait for the normal stable vision confirmation window.
8. Restore the same velocity owner and shadow selections.
9. Start the new run automatically after confirmation; if confirmation fails,
   enter `ERROR` and keep all robots stopped.

Replay from Start never inherits elapsed time, arrival time, collisions, or
plan counts from its parent run.

## Emergency and fatal-error handling

Emergency Stop is large, visually distinct, and always enabled. It sends zero
velocity to every active/planned robot even when the UI state is inconsistent.

The same stop path is triggered automatically by:

- an uncaught exception in execution/control code;
- execution timer failure;
- planner failure affecting the velocity owner;
- vision loss or stale world state beyond the configured safety timeout;
- command sender failure;
- invalid waypoint/path state; or
- application shutdown while applying, ready, running, paused, or resetting.

The stop operation is idempotent and best-effort. Errors while stopping are
recorded, but they do not prevent attempts to stop the remaining robots. Fatal
events include the exception, state, planner, affected robots, and last vision
age in `events.jsonl`.

## Planner presentation

Each planner row contains a checkbox, name, state label, and Run button.

| Planner condition | Appearance |
| --- | --- |
| Not selected | Normal/grey |
| Checked shadow planner | Yellow highlight and `SHADOW` |
| Velocity owner | Green highlight and `EXECUTING` |
| Failed | Red highlight and `ERROR` |

Once any planner runs, all Run buttons and checkboxes are disabled until
Reset E-Stop. Pausing does not transfer ownership. Background planners may continue
mapping/planning during Running and Paused but cannot call the command sender.

## Live field presentation

The field renders the latest complete grSim world snapshot plus:

- velocity owner's active path;
- current and completed path segments;
- current waypoint and final target;
- pause/step state;
- robot status and progress; and
- collision and stale-vision indicators.

A robot missing from the latest usable snapshot is rendered at its last known
pose with 50% opacity and a visible question mark. A currently colliding robot
has a small bold exclamation mark above it. These markers remain visible even
when paths are hidden.

## Results and exports

Result A and Result B use separate scrollable tabs. Tables refresh while the
run is active and retain finalized rows. Each row includes:

- run ID;
- scenario name/version;
- planner;
- planner role (`SHADOW` or `EXECUTING`);
- state;
- start and final timestamps; and
- the corresponding measured Result A or Result B fields.

The executing row is green and shadow rows are yellow. Missing measurements
remain blank, not zero.

Export Result A and Export Result B write the complete visible table to CSV,
not only the last selected row.

## Concrete UI feature inventory

### Top-left: connectivity

- `UDP route test` button and route-status label.
- `Motion feedback test` button.
- Motion test confirmation dialog containing team, robot ID, angular speed,
  duration, and a clear motion warning.
- Feedback result card containing command sent, samples received, measured
  angular velocity, stop-command status, and pass/fail reason.
- Input/vision delay label in milliseconds.

### Top-centre: state and transport

- Large state badge showing the exact controller state.
- Pause, Continue, Step, Replay from Start, Reset E-Stop, and large
  Emergency Stop buttons.
- Fatal/error summary label that remains visible until Reset E-Stop.

### Top-right: scenario application

- Workspace scenario selector.
- Refresh button.
- Add Scenario button that switches to the Scenario Planner tab.
- Load Scenario button for local loading.
- Load Scenario into grSim button for replacement/application.
- Apply-progress card showing confirmed/expected robots, maximum errors,
  vision age, stable snapshot count, timeout progress, and mismatched robots.

### Centre-left: live field

- Latest complete grSim field.
- Velocity-owner path, current waypoint, completed segments, and target.
- Robot progress labels.
- 50%-opacity last-known robot plus question mark for stale/missing robots.
- Bold exclamation mark above colliding robots.
- Visible Paused, Step, Stopped, or Error overlay when applicable.

### Centre-right: planners

One row per planner with:

- shadow checkbox;
- planner name;
- role/state badge;
- Run button;
- last map and plan times; and
- failure message when applicable.

Rows use grey inactive, yellow shadow, green executing, and red error styles.

### Lower-right: results

- Result A and Result B tabs.
- Scrollable tables with fixed headers and sortable run/timestamp columns.
- Executing rows green, shadow rows yellow, debug replay rows carrying a
  visible `DEBUG` marker.
- Export complete Result A table and Export complete Result B table buttons.

### Below field: current-run summary

- elapsed time;
- plans made;
- average map time;
- average planning time;
- collision episodes;
- velocity owner; and
- current run ID.

## Proposed implementation units

Keep state and safety behavior out of widget handlers:

| Unit | Responsibility |
| --- | --- |
| `ExecutionState` | Enum containing all authoritative states. |
| `ExecutionInput` | Immutable loaded scenario, selected paths, planner IDs, and content hash. |
| `ExecutionController` | Valid transitions, ownership, pause/step/replay/reset orchestration. |
| `ScenarioApplyVerifier` | Stable multi-snapshot position/orientation confirmation. |
| `PlannerExecutionSet` | One velocity owner plus zero or more commandless shadow planners. |
| `EmergencyStopper` | Idempotent best-effort zero commands and failure reporting. |
| `ExecutionResultsModel` | Current and finalized Result A/B table rows. |
| `ExecutionConsolePage` | Qt presentation and signal wiring only. |

Suggested module split:

```text
src/research_sdk/ui/execution/
  controller.py
  apply_verifier.py
  results_model.py
  page.py
```

`ResearchRuntime` remains the grSim/command boundary. It gains explicit
ownership, pause/step, and stop operations but does not own Qt widgets.

## Next implementation steps

1. Add a dedicated execution controller and the authoritative state machine.
2. Add scenario transfer/loading and immutable execution-input snapshots.
3. Implement replacement-packet application plus vision confirmation.
4. Add runtime-level single-planner velocity ownership and shadow planners.
5. Implement idempotent emergency stop and fatal-error hooks.
6. Implement pause, continue, and single-waypoint step semantics.
7. Add start replay.
8. Build the new main-page layout and planner/state styling.
9. Add live field progress, stale robot, and collision markers.
10. Add live Result A/B models and full-table CSV exports.
11. Add connection and confirmed motion-feedback tests.
12. Run integration tests against grSim after all no-network unit tests pass.

## Concrete automated test plan

### `tests/test_execution_controller.py`

- `test_initial_state_is_no_scenario`: new controller exposes no runnable
  controls.
- `test_only_documented_state_transitions_are_accepted`: parameterize every
  valid transition and reject representative invalid transitions.
- `test_run_requires_ready_and_confirmed_input`: Run fails before Apply
  confirmation and succeeds in Ready.
- `test_first_run_claims_exclusive_velocity_owner`: first planner owns output;
  a second claim raises without changing ownership.
- `test_planner_selections_lock_until_reset`: checkbox/run configuration cannot
  mutate in Running, Paused, Completed, Stopped, or Error.
- `test_pause_retains_waypoint_indexes`: zero commands are requested and indexes
  remain unchanged.
- `test_continue_uses_retained_next_waypoint`: resumption targets the retained
  index from current observed pose.
- `test_step_advances_once_then_pauses`: exactly one boundary is advanced and
  final state is Paused.
- `test_reset_stops_before_requesting_scenario_apply`: assert call order.

### `tests/test_scenario_apply_verifier.py`

- `test_verifier_requires_all_expected_robot_keys`.
- `test_verifier_requires_three_consecutive_matching_snapshots`.
- `test_position_or_orientation_mismatch_resets_stable_count`.
- `test_verifier_reports_maximum_errors_and_vision_age`.
- `test_apply_timeout_never_enters_ready`.
- `test_stale_or_partial_snapshot_cannot_confirm_apply`.

Use constructed immutable `WorldSnapshot` values; these tests perform no UDP
or Qt operations.

### `tests/test_planner_execution_set.py`

- `test_shadow_planner_never_receives_command_sender`.
- `test_velocity_owner_is_the_only_command_source`.
- `test_owner_failure_requests_emergency_stop`.
- `test_shadow_failure_marks_only_that_shadow_as_error`.
- `test_pause_and_completion_send_zero_to_every_active_robot`.

Use fake planners and a recording fake sender. Assert exact robot/team keys and
command counts.

### `tests/test_execution_replay.py`

- `test_replay_from_start_resets_metrics_and_indexes`.

### `tests/test_emergency_stop.py`

- `test_emergency_stop_is_available_from_every_state`.
- `test_repeated_emergency_stop_is_idempotent`.
- `test_one_sender_failure_does_not_skip_other_robot_stops`.
- `test_fatal_controller_exception_uses_emergency_stop_path`.
- `test_stale_vision_uses_emergency_stop_path`.
- `test_shutdown_while_running_attempts_stop_and_flushes_events`.

### `tests/test_motion_feedback.py`

- `test_motion_test_requires_confirmation_and_healthy_vision`.
- `test_motion_test_reports_observed_angular_velocity`.
- `test_motion_test_always_sends_zero_after_success`.
- `test_motion_test_always_sends_zero_after_timeout_or_exception`.
- `test_motion_test_is_disabled_during_execution_states`.

All commands use a fake sender; a separate grSim integration test covers real
feedback.

### `tests/test_execution_results.py`

- `test_rows_include_run_scenario_planner_role_state_and_timestamps`.
- `test_shadow_and_executing_metrics_remain_separate`.
- `test_missing_values_serialize_as_blank`.
- `test_result_a_and_b_exports_include_every_visible_row`.
- `test_completed_rows_are_not_overwritten_by_new_run`.

### `tests/test_execution_console_ui.py`

Run with `QT_QPA_PLATFORM=offscreen`:

- `test_state_badge_and_button_enablement_for_every_state`.
- `test_shadow_row_is_yellow_and_owner_row_is_green`.
- `test_planner_controls_lock_after_run_and_unlock_after_reset`.
- `test_apply_progress_displays_counts_errors_and_vision_age`.
- `test_missing_robot_is_half_opacity_with_question_mark`.
- `test_collision_marker_is_bold_exclamation`.
- `test_result_tabs_are_scrollable_and_show_all_rows`.
- `test_add_scenario_switches_to_scenario_planner_tab`.
- `test_emergency_stop_remains_enabled_in_every_rendered_state`.

### `tests/test_grsim_execution_integration.py`

Mark these tests `integration` and skip unless grSim is explicitly enabled:

- apply a small scenario and confirm positions through SSL-Vision;
- run the motion feedback test and confirm the final stop;
- execute one short path with one velocity owner;
- pause, continue, and single-step against live feedback;
- force a vision timeout and confirm stop commands were attempted.

Integration tests must use dedicated robot IDs and show a confirmation prompt
when launched interactively.

## Phase gates

### Phase 1: state and safety foundation

Deliver controller, ownership, verifier, fake sender, and emergency stopper.
Exit gate: all non-Qt tests for state, apply confirmation, ownership, and stop
behavior pass. No real command is sent.

### Phase 2: execution mechanics

Deliver Apply, Run, Pause, Continue, Step, and replay from start. Exit gate:
controller tests pass with deterministic fake snapshots and commands.

### Phase 3: UI and results

Deliver the sketched page, colors, field markers, result models, and CSV
exports. Exit gate: offscreen Qt tests and full existing suite pass.

### Phase 4: controlled grSim validation

Enable integration tests one feature at a time: route, Apply confirmation,
motion test, path execution, pause/step, then induced failure. Exit gate: every operation ends with verified zero velocity.

## Out of scope for v1

- simultaneous velocity ownership by multiple planners;
- editing scenario geometry on the execution page;
- silently continuing after stale vision or fatal control errors;
- replacing the established `WorldSnapshot`/pipeline boundary.
