# Scenario Planner UI specification

Status: implemented initial version; acceptance checks passing

Reference name: **UI Redesign Plan v1**

Source: annotated "Plan a Scenario" sketch and follow-up decisions, 2026-08-24

This document is the implementation contract for the new scenario-planning
workspace. It describes behavior, state transitions, display layers, exports,
timing, and acceptance tests. It does not describe the existing layout.

## Scope and navigation

Add a new top-level **Scenario Planner** tab beside the existing **Experiment**
and **Configurations** tabs. The new tab may use an entirely new layout. The
existing tabs remain available during the first implementation so current
live-execution, configuration, and Result B workflows are not removed by the
redesign.

The workspace follows the application's existing Qt/Fusion styling and dark
field canvas. The sketch defines information hierarchy and control placement,
not a replacement colour theme.

The new workspace contains:

1. an editing tool strip;
2. one interactive SSL field;
3. a Plans panel;
4. obstacle and planned-robot inspectors; and
5. export and render-layer controls.

The workspace reuses the established `WorldSnapshot`, `Scenario`,
`PlanningScene`, planner, and result-metric boundaries. It must not introduce a
second raw vision receiver or parallel world-state model.

## Scenario lifecycle

### Load From grSim

Capture the latest complete `WorldSnapshot` and freeze it as the editable
scenario. Every detected robot initially appears as an unassigned field robot.
The ball and robot positions, orientations, and velocities come from the same
snapshot.

### Add New

Create a blank field with no robots, obstacles, targets, paths, or timing
results. This is not a grSim snapshot and not a copy of the current scenario.

### Scenario files

The Plans panel restores the controls shown in the sketch:

- Save Scenario;
- saved-scenario selector;
- Load;
- Add New; and
- Delete, with confirmation.

Saving a blank or newly created scenario prompts for its scenario name at Save
time. The prompt must reject an empty or invalid filename before writing.

These controls belong only to the new Scenario Planner workspace. Their
presence does not require restoring the file controls previously removed from
the existing Experiment layout.

Saving preserves the complete editable state: robots, obstacles, ball,
starting positions, targets, velocities, and planner-related obstacle data.

## Editing tools and state

Only one editing tool is active at a time. The active button has a distinct
selected appearance.

| Tool | First field click | Later field click |
| --- | --- | --- |
| Obstacles | Select a robot and convert it into an obstacle shared by all planners. | Move the selected obstacle to the clicked field position. Clicking another robot selects and converts that robot instead. |
| Starting position | Select a robot or obstacle, convert it into a planned robot if needed, and set its starting position. | Move that selected planned robot's starting position. Clicking another object changes the selection. |
| Target position | Set the selected planned robot's target. | Replace that robot's previous target with the newly clicked position. |

All editable positions are stored in millimetres and clamped to valid field
bounds.

### Target guards

- Target position is disabled until a planned robot with a starting position
  is selected.
- A target action without a starting-position robot is rejected and must not
  mutate the scenario.
- A robot with a starting position but no target is incomplete. Plan and Save
  must show a clear validation error identifying that robot.
- Multiple planned robots may each have independent start and target markers.

### Converting roles and invalidating plans

Obstacle assignments are shared by all three planners. Converting any robot
between obstacle and planned-robot roles, or moving a start, target, or
obstacle, changes the planning input. After any such change:

1. erase every generated planner path;
2. clear every planner's current mapping and planning counters;
3. retain historical exported/run records;
4. mark the scenario as requiring replanning; and
5. show a message asking the user to press Plan again.

No velocity command is sent as part of editing or replanning.

## Field interaction and coordinates

The field renders standard SSL markings, robots, obstacle markers, planned
starts, targets, planner geometry, and paths.

While the pointer is inside the playable field:

- draw a virtual horizontal and vertical cursor axis at 50% opacity;
- update a small coordinate readout near the cursor or field edge;
- show `x` and `y` in millimetres;
- use the same field-centred coordinate transform as planning; and
- hide the virtual axes and coordinate readout when the pointer leaves.

The virtual cursor axes are guides only and are not included as scenario data.

## Plans panel

### Planner selection

The selector exposes Voronoi, PRM, and Visibility Graph. Selecting a planner
controls which path and planner-specific map geometry are considered active
for display and snapshot export.

### Plan action

Plan is preview-only. It must never start the velocity controller or send robot
commands.

Pressing Plan:

1. validates that every planned robot has a start and target;
2. builds and searches with all three planners against the same scenario;
3. stores one result per planner;
4. displays the selected planner's paths;
5. records separate mapping and planning durations in milliseconds; and
6. updates each planner's stored timing result.

Replanning overwrites that planner's current displayed/stored timing values;
it does not append duplicate current results. Historical exported experiment
records remain immutable.

The UI displays next to Plan, or directly below it:

- `Map: <value> ms` for planner-specific map/graph construction; and
- `Plan: <value> ms` for path search after construction.

Instrumentation must define planner phase boundaries rather than deriving one
value by subtraction from unrelated vision-pipeline mapping time.

### Clear action

Clear removes generated path and planner-map geometry from the field but keeps
the most recently measured Map and Plan milliseconds visible for reference.
Editing the scenario is different: it invalidates paths and clears the current
timing counters because the measurements no longer describe the input.

## Path definition and interaction

A path is the planner-produced ordered polyline from one planned robot's start
to its target, including intermediate waypoints.

Paths are interactable but not draggable:

- hover highlights a path and shows planner, robot identity, length, and
  planning time;
- click selects the path/robot and synchronizes the robot inspector;
- the selected planner's path is drawn as the active solid path;
- benchmark results from other planners remain stored but hidden until their
  planner is selected; and
- dragging a path is prohibited because planner output must be reproducible.

Selecting or displaying a path never sends velocities.

## Inspector panels

### Obstacles

Show the count and one row per obstacle containing:

- team and robot ID;
- position in millimetres; and
- velocity in millimetres per second.

Clicking a row selects and highlights that obstacle on the field.

### Planned robots

Show the count and one row per planned robot containing:

- team and robot ID;
- starting position in millimetres; and
- target position in millimetres, or `not set`.

Clicking a row selects the robot for start/target editing and highlights it on
the field.

## Render layers

Bottom controls are independent toggle buttons with distinct on/off styling.

### Map

Show the selected planner's graph/debug geometry:

- Voronoi: bounded Voronoi graph and relevant sites/edges;
- PRM: sampled roadmap nodes and accepted connections; and
- Visibility Graph: polygon/contact vertices and accepted visibility edges.

Disable Map when the selected result has no available geometry (for example,
before planning or after Clear).

### Robots

This replaces the proposed Show/hide Obstacles control. It hides or shows the
physical robot bodies as a group. Semantic planning markings remain visible:

- obstacle markers;
- starting-position markers;
- target markers; and
- field markings.

### Obstacle shape

Show or hide the collision/keep-out geometry actually created by the selected
planner:

- Voronoi: the planner's inflated circular keep-out envelopes;
- PRM: the collision-checking/inflated obstacle envelopes used to reject
  samples and edges; and
- Visibility Graph: the polygon approximation and inflation used to construct
  visibility vertices.

This layer is planner-derived geometry, not a generic decorative buffer.

### Path

Show or hide the selected planner's active polylines without deleting them or
their stored timings.

## Exports

### Export Snapshot

Write a PNG of the field canvas only. It includes:

- field markings;
- visible robot identities and their position markers;
- obstacle, start, and target markings;
- the selected planner's name;
- selected planner paths; and
- exactly the planner-map, robot, obstacle-shape, and path layers enabled at
  export time.

The cursor guides and surrounding application controls are excluded.

### Export Result A

Export one labeled row per planner. The export uses each planner's most recent
stored mapping and planning measurements for the current valid scenario.
Unavailable values remain blank rather than becoming zero. Result B remains
available in the existing Experiment workflow unless a later decision moves it
into this workspace.

Exports default to a consistent `exports/` folder while still allowing the
operator to select another destination.

## No execution in this workspace

The Scenario Planner is an offline planning and comparison workspace. It does
not expose Run/Stop velocity controls, call `start_execution`, call
`execute_tick`, or send robot velocity commands. The current Experiment tab
continues to own physical/grSim execution until a separate execution redesign
is approved.

## Implementation sequence

1. Add the Scenario Planner tab and new layout without removing existing tabs.
2. Extract an editor state/controller from widget event handlers.
3. Implement blank and snapshot scenario creation plus file operations.
4. Implement role conversion, relocation, target guards, and invalidation.
5. Add obstacle/robot inspector models and synchronized selection.
6. Extend planner outputs/recorders with map geometry and phase timings.
7. Add paths and render-layer toggles.
8. Add field PNG and current-result exports.
9. Add automated tests and offscreen Qt rendering checks.
10. After acceptance, review whether the old Experiment planning controls can
    be removed without affecting execution or configuration workflows.

## Acceptance tests

At minimum, automated tests must verify:

- Add New produces an empty scenario and canvas.
- Load From grSim reuses the latest complete `WorldSnapshot`.
- obstacle first-click conversion and second-click relocation;
- planned-robot first-click selection and second-click start relocation;
- repeated target clicks replace the previous target;
- Target position cannot activate or mutate state without a selected start;
- Plan and Save reject every start-without-target robot with a useful error;
- planned robot to obstacle and obstacle to planned robot conversions;
- every input edit invalidates all planner paths and current timing counters;
- Clear hides planner output but preserves timing counters;
- replanning overwrites the same planner's current timing values;
- mapping and planning timings are non-negative and measured independently;
- planning invokes all three planners and sends no velocities;
- obstacle assignments affect all planners;
- path hover/click selection synchronizes with inspectors;
- robot visibility leaves obstacle/start/target/field markings visible;
- obstacle-shape geometry matches the selected planner;
- Map shows Voronoi, PRM, or Visibility Graph geometry as appropriate;
- cursor axes use 50% opacity and coordinates match field millimetres;
- snapshot PNG respects selected planner and visibility toggles;
- Result A exports one row per planner; and
- scenario save/load round-trips the complete editable state.

## Out of scope for v1

- sending velocities from the Scenario Planner;
- dragging planner-generated paths;
- editing a generated waypoint by hand;
- replacing the established world snapshot/pipeline; and
- removing Experiment or Configurations before the new workspace is accepted.
