# Current state: planners, runtime, and comparison tooling

Living snapshot of what's actually true in the code today. For the *why*
behind each of these and the real benchmark numbers backing them, see
[decisions/0005-parallel-planner-execution.md](decisions/0005-parallel-planner-execution.md) --
this doc is the quick-orientation version, that one is the record.

## Planners

Three standalone planners, all reachable through the same contract
(`PlanRequest`/`PlanResult` for `visibility_graph.plan()`/`prm_dijkstra.plan()`,
`PlanningScene` + start/target points for `VoronoiDijkstraPlanner.plan()`):

- `planners/VisibilityGraph/visibility_graph.py` -- Minkowski-inflated
  visibility graph, `polygon_sides` from `VISIBILITY_POLYGON_SIDES`.
- `planners/PRM/prm_dijkstra.py` -- random milestone sampling,
  `num_samples`/`k_neighbours`/`max_resample_attempts` from
  `PRM_NUM_SAMPLES`/`PRM_K_NEIGHBOURS`/`PRM_MAX_RESAMPLE_ATTEMPTS`.
- `planners/Dijkstra/voronoi_dijkstra.py` -- density-grid bounded Voronoi
  map + Dijkstra, `density_percent`/`max_density_nodes` from
  `VORONOI_DENSITY_PERCENT`/`VORONOI_MAX_DENSITY_NODES`.

All tunables above live in `config/planner_variables.yaml`, validated the
same way (type/range checks in `config/__init__.py`) -- single source of
truth for both planner defaults and the UI's debug overlay.

**Shared search.** All three now search with `networkx.dijkstra_path`.
`VoronoiDijkstraPlanner` used to have its own hand-rolled heapq Dijkstra;
replaced so path-quality comparisons aren't confounded by which search
implementation ran.

**`skip_direct_path: bool = False`** on all three `plan()` calls. Default
`False` = normal/production behaviour (take the free direct path when one
exists). The offline comparison tool (below) forces it `True` on all three
uniformly, so every reported latency reflects the algorithm's actual
worst-case graph/roadmap/map-construction cost, not whichever fraction of
queries happened to have clear line of sight.

## `ResearchRuntime` (`ui/runtime.py`)

Two independent constructor flags, both policy switches rather than
numeric tunables (so they're plain Python defaults, not
`planner_variables.yaml` entries):

- **`parallel_planning: bool = True`.** Robots plan on a `ThreadPoolExecutor`
  when there's more than one. Benefit is UI responsiveness, not raw speed --
  benchmark data shows a thread pool can be *slower* than serial for this
  CPU-bound work (GIL contention); a process pool is the real throughput
  win and is deliberately not wired in yet.
- **`predict_motion: bool = False`.** When `True` and `self.world_pipeline.latest_scene`
  exists (i.e. real vision frames have been ingested), teammate/opponent
  obstacles come from real tracked position + velocity + motion-inflated
  radius (`WorldMap.planning_scene()`) instead of static scenario
  positions. Falls back to static automatically whenever no live scene
  exists yet -- never destructive to turn on.

**Per-robot failure isolation.** A robot whose planner call fails no longer
aborts the whole `plan()` call for every robot. `PlannedRobotPath` carries
`failed: bool = False`; a failed robot gets a stationary one-point path
(`(robot.start_mm,)`), which the existing (unmodified) waypoint-execution
loop already reads as "arrived, stay stopped." The UI draws a `failed`
robot in red (`#e53935`) via `_plan_failed_for()` in both `FieldCanvas`
subclasses.

## UI (`ui/app.py`)

The "Plan" button (`ScenarioPlannerPage._plan()`) plans **only the
currently-selected planner**, not every planner in the dropdown. It used to
loop over all of them to build a preview for each -- reverted because
running every planner back-to-back against a live grSim connection fought
`ExecutionController`'s single-`velocity_owner` assumption
(`ui/execution/controller.py`) in practice. There is no cross-planner
comparison table in the UI; that capability lives entirely in
`scripts/demo_planners.py` (below), which is offline and grSim-free by
design.

**Map/Plan timing display is now honest about what it can and can't
measure.** `PlannerPreview.map_time_ms` is `float | None` -- `None` for any
planner without `StepRecorder` support (`VoronoiDijkstraPlanner` has no
`record` parameter). The UI used to fill that gap by timing a *separate*,
cheaper debug-only Voronoi map build (`_debug_geometry()`'s render-density
call) and subtracting it from the real total, which dumped almost the
entire real map-generation cost into the "Plan" label -- making Voronoi
look search-bound when it's provably not (decision 6: swapping its search
implementation changed nothing). It now shows "Map: n/a (no split for this
planner)" and "Plan (total): X ms" for planners without a real split,
instead of a fabricated subtraction.

## Comparison tooling (permanent, not scratch scripts)

- **`scripts/benchmark_planners.py`** -- all three planners against every
  saved scenario in `scenarios/`, N trials each, reports median/p95
  `planning_time_ms` and a `full_map_builds` counter (calls that did *not*
  take a shortcut) so a reader can tell whether a number reflects the
  worst case or the easy case.
- **`scripts/benchmark_parallel_planning.py`** -- serial vs. thread-pool
  vs. process-pool wall-clock time for planning several robots' paths in
  one batch, using `VoronoiDijkstraPlanner` directly (forced full solves,
  no caching).
- **`scripts/demo_planners.py`** -- single point-to-point query, all three
  planners, `skip_direct_path=True` uniformly, prints a comparison table
  (success/path length/planning time/waypoint count), saves a side-by-side
  matplotlib plot (`demo_output.png`), and an optional randomized
  multi-obstacle stress test (`--trials N`) reporting success rate, timing,
  and average path length per planner.

## Test coverage

116 tests passing. Planner-level: `skip_direct_path` regression tests for
all three planners (`tests/test_visibility_graph.py`,
`tests/test_prm_dijkstra.py`, `tests/test_voronoi_dijkstra.py` -- the last
one new, no prior direct test coverage of `VoronoiDijkstraPlanner.plan()`
existed). Runtime-level: `tests/test_ui_runtime.py` covers
`predict_motion` on/off/no-live-scene-yet, and the rewritten
`test_preview_plans_selected_algorithm_only_without_starting_execution`
covers the single-planner "Plan" button contract.

## Deliberately not done (see the ADR's "Future work" for detail)

Process-pool wiring into `ResearchRuntime`, prioritized/sequential
multi-robot planning (avoiding teammates' *planned paths*, not just
current position), a shared policy/validation layer (reject-and-replan
against clearance rules), and a dedicated results/export folder. All
explicitly parked, not forgotten.
