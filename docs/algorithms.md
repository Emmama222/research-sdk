# Path-planning algorithms

> **Status.** These are the two planners for the Path Planning System
> Comparison project: `PRM + Dijkstra` (TurtleRabbit's own historical
> approach, recovered from git history — see the project doc "TurtleRabbit
> PRM+Dijkstra - Recovered History and Source") and `Visibility Graph +
> Dijkstra` (the approach documented in Warthog Robotics' TDP — see "Warthog
> Visibility Graph + Dijkstra - Implementation Notes"). Both live in
> `src/research_sdk/planners/` as self-contained modules with no dependency
> on the `world/`, `adaptors/`, or `vendored/team_control_planner/` layers
> described in [architecture.md](architecture.md) — those layers do not exist
> in this repo yet (see the note at the bottom of this doc), so both planners
> take a plain `PlanRequest` in and return a plain `PlanResult`, with no
> adaptor wiring required to run or test them.
>
> Each module also ends with a small `PRMPlanner`/`VisibilityGraphPlanner`
> class that adapts its own `plan()` function to the `PlannerAPI.plan()`
> contract the UI's "Active planner" dropdown expects (see
> `planners/api.py`, `planners/Dijkstra/waypoint_manager.py`, and
> `ui/runtime.py::ResearchRuntime.set_planner`) — that's the only part of
> either module that touches Emma's world-model shapes
> (`PlanningScene`/`PlanningObstacle`); the `plan()` function itself stays
> untouched by it.

## Shared types (`planners/common.py`)

- `Obstacle(pos_mm, radius_mm, robot_id, isYellow)` — a circular obstacle
  (an opposing or teammate robot). Field names match the convention already
  used by `process_workers/voronoi_map_runner.py` (`pos_mm`, `radius_mm`,
  `robot_id`, `isYellow`) so a future adaptor can hand obstacles to either
  planner without translation.
- `PlanRequest(start_mm, goal_mm, obstacles, robot_radius_mm, clearance_mm,
  field_length_mm, field_width_mm)` — everything a planner needs for one
  query. `total_clearance_mm` (`robot_radius_mm + clearance_mm`) is the
  distance every returned path segment must stay from any obstacle's centre
  plus that obstacle's own radius.
- `PlanResult(success, waypoints_mm, path_length_mm, planning_time_ms,
  nodes_expanded, message)` — both planners return this shape so they can be
  swapped in a comparison harness without any call-site branching.

`FIELD_LENGTH_MM` / `FIELD_WIDTH_MM` default to a Division B field
(9000x6000mm); pass a `PlanRequest` with different values for Division A.

## PRM + Dijkstra (`planners/PRM/prm_dijkstra.py`)

`plan(request, *, num_samples=20, k_neighbours=6, max_resample_attempts=5,
seed=0) -> PlanResult`

Random-sample milestones across the field, connect each to its
`k_neighbours` nearest neighbours when the connecting segment clears every
inflated obstacle, then run `networkx.dijkstra_path` from start to goal. If
start and goal already have clear line of sight the sampling step is skipped
entirely. If the first sample doesn't yield a start-goal path, resample with
a new seed up to `max_resample_attempts` times before reporting failure —
this mirrors the probabilistic nature of PRM (a single unlucky sample set
can miss a route that exists) without hanging indefinitely.

This is a direct adaptation of the PRM+Dijkstra implementation TurtleRabbit
used previously (recovered from git history, itself adapted from
[KaleabTessera/PRM-Path-Planning](https://github.com/KaleabTessera/PRM-Path-Planning)),
with four changes documented in the module docstring: real SSL field
boundaries instead of a fixed test rectangle, circular obstacles instead of
rectangular ones, no matplotlib in the planning hot path, and `networkx`
instead of the original hand-rolled Dijkstra.

## Visibility Graph + Dijkstra (`planners/VisibilityGraph/visibility_graph.py`)

`plan(request, *, polygon_sides=6) -> PlanResult`

Inflates every obstacle into a circumscribed `polygon_sides`-gon (a hexagon
by default, conservatively approximating the Minkowski-sum-inflated circle
used in Warthog's TDP), builds a visibility
graph over start, goal, and every polygon vertex (an edge exists where the
straight segment between two vertices doesn't cross any obstacle polygon),
and runs `networkx.dijkstra_path` over that graph. Same direct-line-of-sight
shortcut and start/goal-inside-obstacle early exit as the PRM planner.

Two implementation details worth knowing if you touch this file:

1. **Same-polygon adjacency.** Two vertices on the same inflated polygon are
   either adjacent (the segment between them *is* that polygon's boundary,
   always visible) or non-adjacent (the segment always cuts through the
   polygon's interior, always blocked — inflated obstacles are convex). The
   code special-cases both rather than relying on the general
   segment-vs-polygon interior test, because that test's midpoint-in-polygon
   fallback is unreliable exactly on a polygon's own boundary (floating-point
   ambiguity in ray-casting point-in-polygon right on the edge). Regression
   test: `tests/test_visibility_graph.py::test_polygon_boundary_vertices_stay_connected`.
2. **Plain Python, not numpy, in the hot loop.** An earlier version used
   numpy arrays for every 2D point in the O(n²) visibility test. Numpy's
   per-call overhead on 2-element arrays made an 8-obstacle scenario take up
   to ~19 seconds — thousands of times over SSL's 16ms control-loop budget.
   The hot path now uses plain tuples/`math`, plus an O(1) bounding-circle
   broad-phase check (`_segment_could_hit_polygon`) that skips the detailed
   per-edge test for polygons a candidate segment can't possibly reach.
   Worst case on the same stress test dropped to well under 100ms. If you
   touch this file, rerun `scripts/demo_planners.py --trials 30` and check
   the reported max time before merging.

## Trying them out

```shell
pip install -e .[demo]   # matplotlib
PYTHONPATH=src python3 scripts/demo_planners.py --trials 30
```

This runs both planners against a fixed six-opponent scenario (prints a
comparison table and saves a side-by-side plot to `demo_output.png`), then
runs a 30-trial randomized stress test and reports success rate plus
min/avg/max planning time for each planner.

Unit tests: `pytest tests/` (`PYTHONPATH=src` if not installed with `pip
install -e .`) — covering direct-path shortcuts, obstacle clearance
(segment-sampled against the inflated radius), trivial start==goal,
start-inside-obstacle failure, and the visibility-graph adjacency regression
above.

Both planners are also selectable from the "Active planner" dropdown in the
UI sandbox (`research-sdk-ui`), via the `PRMPlanner`/`VisibilityGraphPlanner`
adapter classes described above.

## A note on scope

[architecture.md](architecture.md) and [adaptors.md](adaptors.md) describe a
target design with `world/`, `adaptors/`, `core/`, and
`vendored/team_control_planner/` layers. As of this writing none of those
paths have ever been committed to this repo, on any branch (`origin/main`,
`upstream/main`, and `visibility+PRM` are all the same commit,
`c647d50`) — this planner code originally lived in its own
`src/research_sdk/algorithms/` package before moving into
`src/research_sdk/planners/` (alongside `Dijkstra/voronoi_dijkstra.py`) so
the two `PRMPlanner`/`VisibilityGraphPlanner` adapter classes described
above could be discovered by the UI's "Active planner" dropdown the same
way `VoronoiDijkstraPlanner` already was. That's a heads-up for whoever
wires this into the rest of the SDK later, not a criticism of the docs:
they read as intentionally aspirational (architecture.md says as much
about its own layering). Outside of those two adapter classes, treat
`prm_dijkstra.py`/`visibility_graph.py` as planner logic only — pass it a
`PlanRequest`, get back a `PlanResult`, no adaptor
required.
