# 0005: Parallel per-robot planning, and why threads aren't the whole answer

Date: 2026-08-26

## Context

`ResearchRuntime.plan()` (`ui/runtime.py`) plans every robot in a scenario
one at a time, in a plain `for` loop, on whatever thread calls it (the Qt UI
thread, via `app.py`). The question raised: should this be parallelized, and
if so how? Two follow-up questions came out of that: (1) is the relative
planner slowness actually a bug, or expected, and (2) threads vs processes.

Rather than guess, two throwaway benchmark scripts were written to get real
numbers: `scripts/benchmark_planners.py` (all three planners, head to head,
against every scenario under `scenarios/`) and
`scripts/benchmark_parallel_planning.py` (serial vs. thread pool vs. process
pool, planning several robots' paths in one batch).

Once robots plan concurrently instead of in one strict sequence, a second
question follows immediately: what happens when one robot's plan call
fails? The original serial code aborted the entire `plan()` call on the
first failure (`raise` inside the loop) -- reasonable when it's a strictly
ordered loop, but not once robots are independent, concurrent attempts.
The ask: flag that one robot (red, in the UI) and hold it in place, instead
of losing every other robot's plan because of it.

## Findings

1. **The observed planner ordering (Voronoi slowest, Visibility Graph next,
   PRM fastest) is real and is not a bug.** `benchmark_planners.py` against
   the two saved scenarios, 20 trials/robot, every call hitting a genuine
   full map/graph/roadmap build (no direct-line-of-sight or
   reused-previous-path shortcut):

   | Planner | median | p95 |
   |---|---|---|
   | VoronoiDijkstra | ~35.7 ms | ~36.9 ms |
   | VisibilityGraph | ~17.8 ms | ~19.1 ms |
   | PRM | ~6.8 ms | ~7.7 ms |

   This tracks each algorithm's actual per-call work, not a defect:
   `VoronoiDijkstraPlanner.plan()` rebuilds a full density-grid Voronoi
   tessellation from scratch every call it doesn't shortcut
   (`voronoi_dijkstra.py:150`, up to `voronoi_max_density_nodes=140` nodes);
   Visibility Graph does an O(n²) edge-visibility test over polygon vertices
   (`visibility_graph.py:229`); PRM only samples `prm_num_samples=20` points
   with `k=6` neighbours -- the smallest graph of the three by a wide margin.
   **Separately worth flagging**: 35ms for a single robot's Voronoi solve
   already exceeds the 16ms/tick SSL budget the Visibility Graph module
   docstring cites -- that's a planner-tuning question (`voronoi_max_density_nodes`,
   `voronoi_density_percent` in `planner_variables.yaml`), not something
   parallelism fixes; parallelism changes *throughput across robots*, not one
   robot's latency.

2. **A thread pool does not speed up planning multiple robots -- it can make
   it slower.** `benchmark_parallel_planning.py`, 6 robots' worth of forced
   full Voronoi solves per trial, median of 8 trials:

   | Mode | 6 robots | 2 robots |
   |---|---|---|
   | serial | 211.8 ms | 72.5 ms |
   | thread pool | 267.3 ms | 91.2 ms |
   | process pool | 61.0 ms | 53.3 ms |

   `VoronoiDijkstraPlanner.plan()` is CPU-bound, mostly-pure-Python work
   (density-grid generation + a hand-rolled Dijkstra) -- there's very little
   I/O or C-extension time for the GIL to release, so threads mostly just
   add scheduling/handoff overhead on top of fully-serialized execution.
   `ProcessPoolExecutor` gets genuine parallel CPU time and wins even at 2
   robots, because each job (~35ms) is far larger than process-pool
   spin-up/pickling overhead (~15-20ms fixed cost for the batch, not per
   job).

## Decisions

1. **`ResearchRuntime.plan()` gets a thread pool anyway, on by default**
   (`self.parallel_planning = True`, `ui/runtime.py`). Its justified benefit
   is *not* throughput (finding 2 above shows it can lose there) -- it's
   that each robot's `PlannerAPI.plan()` call is independent (obstacles are
   a scene snapshot, not built from other robots' newly-planned paths), so
   overlapping them off whatever thread calls `plan()` keeps the UI
   responsive without changing planning correctness. `parallel_planning`
   stays a plain constructor flag, not a `planner_variables.yaml` entry --
   it's a concurrency policy switch, not a numeric algorithm tunable, and
   forcing it through that file's strict positive-number/integer validation
   would fight the schema for no benefit.

2. **A process-pool path is deliberately not wired into `ResearchRuntime`
   yet**, despite finding 2 showing it's the actual throughput win. Doing
   that for real requires either making `PlannerAPI`'s state
   (`VoronoiWaypointManager._state_by_robot`) safe to share/reconstruct
   across a process boundary, or restructuring the call so only the
   stateless per-call solve (not the stateful reuse/dead-zone layer) crosses
   into worker processes -- both are bigger, riskier changes than this
   decision's scope. `scripts/benchmark_parallel_planning.py` stays as the
   standing comparison tool for when that work is picked up, so the next
   change is validated against real numbers instead of repeating the
   thread-pool assumption.

3. **`scripts/benchmark_planners.py` and `scripts/benchmark_parallel_planning.py`
   are kept as permanent tools, not one-off scratch scripts** -- re-run
   either after changing a planner's algorithm or its `planner_variables.yaml`
   tunables to see the real effect, the same way
   `scripts/benchmark_world_pipeline.py` is already used for the vision
   pipeline.

4. **One robot's planning failure no longer aborts `plan()` for every robot
   -- it's isolated, flagged, and the robot is held in place.**
   `PlannedRobotPath` (`ui/runtime.py`) gained a `failed: bool = False`
   field. `ResearchRuntime._run_and_track()` no longer re-raises a robot's
   planning exception; it counts the failure (`last_plan_failures`, timing
   still recorded via the exception's `.duration_ms`, same as before) and
   returns a stationary path for that robot instead --
   `PlannedRobotPath(robot_id, is_yellow, (robot.start_mm,), failed=True)`.
   `plan()` itself now only raises for a scenario-level problem
   (`scenario.require_complete()`), never for an individual robot's planner
   call.

   This was a natural consequence of decision 1, not an independent choice:
   once robots are planned concurrently rather than in strict sequence,
   "abort everything because robot 3 of 6 found no route" stops making
   sense -- the other five already have (or are getting) valid plans.

   Two things fall out of the `failed` flag for free, with no execution-loop
   changes needed:
   - **Visual flag.** `FieldCanvas._draw_robot` and
     `ScenarioPlannerCanvas._draw_planned_robot` (`ui/app.py`) both call a
     new `_plan_failed_for(robot_id, is_yellow)` lookup against `self.paths`
     and draw the robot in red (`#e53935`) instead of its team colour when
     `failed` is set.
   - **Held in place.** A failed robot's path is a single point
     (`(robot.start_mm,)`). `_draw_paths` already skips anything with
     `len(points_mm) < 2`, so no path line is drawn. More importantly,
     `ResearchRuntime.execute_tick()`'s existing waypoint-index logic
     (`ui/runtime.py:403`) initializes `_waypoint_indices` to
     `min(1, len(path.points_mm))` -- for a length-1 path that's already
     `>= len(path.points_mm)` on the very first tick, so the robot gets
     `_send_stop()` every tick and never receives a `waypoint_command`. No
     special-casing was needed in the execution loop at all; a stationary
     "failed" path already reads as "arrived, stay stopped" to code that
     predates this decision.
   - The `_plan()` preview flow (`app.py`) now reports failed-robot counts
     in the status bar (`"{label}: N robot(s) found no route (Y2) -- flagged
     red, held in place"`) instead of discarding the whole planner's paths
     for one bad robot.

   `tests/test_ui_runtime.py::test_runtime_records_failed_planner_invocation`
   was updated to match: it now asserts `plan()` returns normally with a
   `failed=True`, single-point path, rather than asserting a raised
   exception.

5. **`ResearchRuntime` gets a second, independent policy switch,
   `predict_motion` (default `False`), that sources teammate/opponent
   obstacles from `self.world_pipeline.latest_scene` -- real tracked
   position, velocity-projected position, and motion-inflated radius, via
   `WorldMap.planning_scene()` (`world/map/world_map.py:370`) -- instead of
   each robot's static scenario `start_mm`.** `ResearchRuntime._obstacles_for_robot`
   (`ui/runtime.py`) was split into itself (scenario-authored decoy
   obstacles, unchanged) plus a new `_other_robot_obstacles(scenario,
   robot)`, which checks `self.predict_motion`: if set and
   `self.world_pipeline.latest_scene` is not `None`, it returns that scene's
   obstacles filtered by `PlanningObstacle.key` to exclude the robot being
   planned for; otherwise it falls back to exactly the old static-`start_mm`
   behaviour. Because the swap happens once, upstream of
   `self._planner.plan(...)`, it applies uniformly to whichever planner is
   selected (Visibility Graph, PRM, Voronoi) with no per-planner code --
   this is the fix for the "Future work" gap below (velocity/prediction
   existed but was wired into no planning call path).

   The fallback is deliberately automatic, not an error: `predict_motion=True`
   with no vision frames ingested yet (fresh reset, or the scenario editor's
   offline "Plan" preview button, which runs with no simulator/vision
   attached at all) silently behaves exactly like `predict_motion=False`
   rather than raising or returning empty obstacles. Turning the flag on is
   therefore never destructive -- it only changes behaviour once there is
   real tracked data to use.

   Still open, not part of this change: `ScenarioRobot` (the scenario
   editor's hypothetical, not-yet-tracked robots) has no velocity field, so
   `predict_motion` only does something once a real `VisionWorldPipeline` is
   actually running (live grSim/vision session) -- purely offline scenario
   comparisons (e.g. `scripts/benchmark_planners.py`) still see the static
   fallback today. A synthetic "assume each teammate heads toward its own
   `target_mm` at some nominal speed" heuristic would extend prediction to
   the offline case too, but that's a real modeling choice (what speed?
   what if the robot has already arrived?) deliberately left undecided
   here rather than guessed at.

6. **All three planners now search with the same shortest-path
   implementation, `networkx.dijkstra_path`.** `VoronoiDijkstraPlanner._dijkstra`
   (`voronoi_dijkstra.py`) was a hand-rolled heapq Dijkstra --
   algorithmically equivalent to `nx.dijkstra_path` (same textbook
   algorithm, same non-negative edge weights, so identical shortest-path
   *cost* either way), but it meant the planner comparison had three
   graph-construction strategies paired with two different search
   implementations, which is an avoidable confound the moment path quality
   (not just latency) gets compared across planners -- a reviewer's
   reasonable next question after a latency comparison. Rewritten to build
   an `nx.Graph` from the existing `adjacency` dict and call
   `nx.dijkstra_path`, matching `visibility_graph.py`/`prm_dijkstra.py`
   exactly. Re-ran `scripts/benchmark_planners.py` after the change: Voronoi
   median moved from ~35.7ms to ~35.8ms -- noise, not a real shift, because
   almost all of Voronoi's cost is in building the density-grid map, not
   the final search over it. The planner-ordering finding in finding 1 is
   unaffected by this change.

7. **All three planners can now skip their own direct-line-of-sight
   shortcut on request (`skip_direct_path: bool = False`), and the offline
   comparison forces it on for all three.** `visibility_graph.plan()`,
   `prm_dijkstra.plan()`, and `VoronoiDijkstraPlanner.plan()` each gained
   the flag, defaulting to `False` (normal/production behaviour: take the
   free direct path when one exists). `scripts/demo_planners.py` passes
   `skip_direct_path=True` for all three, so every comparison run measures
   each algorithm's actual worst-case graph/roadmap/map-construction cost,
   not whichever fraction of trials happened to have a clear straight line.
   (Voronoi briefly kept its shortcut on while the other two forced it off,
   as an intentional asymmetry testing "how it actually runs in
   production" against the other two's worst case -- reversed after
   reflection: all three now use the same rule, since an asymmetric
   methodology invites exactly the "was that a fair comparison?" question
   decision 6 was written to close off.)

8. **The scenario-editor "Plan" button plans only the currently-selected
   planner again, not every planner in the dropdown.** `ScenarioPlannerPage._plan()`
   (`ui/app.py`) previously looped over every entry in `self.planner_selector`
   and built a preview for all of them per click -- fine as a pure offline
   preview, but running every planner back-to-back against a live grSim
   connection fought `ExecutionController`'s single-`velocity_owner`
   assumption (`ui/execution/controller.py`) in practice. Reverted to
   planning just `self.planner_selector.currentText()`/`.currentData()`,
   matching the execution model's single-active-planner design. The
   now-pointless "Planner comparison" table added earlier in this session
   (it would only ever have shown one row once `_plan()` stopped looping)
   was removed along with it -- the 3-way, offline, grSim-free comparison
   lives in `scripts/demo_planners.py` instead, which is exactly the tool
   decision 7 targets. `tests/test_scenario_planner_ui.py`'s
   `test_preview_plans_all_algorithms_without_starting_execution` was
   renamed and rewritten (`test_preview_plans_selected_algorithm_only_without_starting_execution`)
   to assert the new one-planner-per-click contract.

9. **`voronoi_density_percent` and `voronoi_max_density_nodes` were tuned
   down (60.0 → 10.0, 140 → 100), and it's a real speed win with a real,
   only partially validated cost.** Root cause behind finding 1's 35ms
   Voronoi number: the density-grid "grounding" sites that close off
   otherwise-unbounded Voronoi cells near the field boundary (a legitimate,
   necessary technique, not a bug) scale directly with these two config
   values. Swept before changing anything
   (`scripts/benchmark_planners.py`-style direct sweep, 90 samples/config
   across the two saved scenarios):

   | Config | median time | avg path length |
   |---|---|---|
   | old (60%, 140) | 35.9 ms | 3625 mm |
   | new (10%, 100) | ~5.3 ms | ~4500 mm (measured at 10%/80, not re-measured at the exact 10%/100 now shipped) |

   Re-ran `scripts/benchmark_planners.py` after the config change landed:
   Voronoi is now the *fastest* of the three (~5.3ms), not the slowest --
   the ordering in finding 1 is no longer current. Success rate held at
   100% in every config tested down to 10% density, but that is **not**
   strong evidence the lower density is safe in general: it's still only
   the same 2 saved scenarios and a light 3-8-obstacle stress test, not a
   dense/adversarial scenario designed to stress connectivity. Path length
   grew ~24% at the sparsest configs tested (3625mm → ~4500mm) -- a real,
   not negligible, quality cost for the speed win, and it applied fairly
   uniformly between 10% and 30% density in the sweep (the "knee" was
   between 30% and 60%, not between 10% and 30%).

   Not done as part of this decision: a denser/adversarial validation
   scenario to confirm 10%/100 doesn't silently start failing outside the
   two saved scenarios, and a final call on whether 10%/100 is the number
   this project reports in the paper versus a middle ground (30% looked
   like most of the speed win without going as far into the length
   penalty in the earlier sweep). Both explicitly left open.

10. **`VoronoiDijkstraPlanner.plan()` gained real `StepRecorder` support, and
    a UI display bug that predates it got fixed as a result.** It's a
    coarser split than `visibility_graph.py`/`prm_dijkstra.py`'s per-edge/
    per-sample logging -- two timestamps bracketing (a) the
    `generate_voronoi_map_from_scene()` call plus splicing start/target
    into the graph, and (b) the `self._dijkstra(...)` call -- because map
    generation happens inside a separate module (`voronoi_generator.py`)
    that doesn't accept a recorder; threading `record` through that
    module's internals for per-step granularity wasn't judged worth it for
    a "map vs. search" split. Wired through `PlannerInput` (gained a
    `record` field, `waypoint_manager.py`), `VoronoiWaypointManager.update()`
    (passes it to the `VoronoiDijkstraPlanner` it constructs per call), and
    `ResearchRuntime` (`self._recorder`, set in `set_planner()`, attached to
    every `PlannerInput` built in `_plan_one_robot()`).

    This closed a real bug in `ScenarioPlannerPage._plan()`'s "Map"/"Plan"
    UI labels, found by inspection while explaining why they looked odd for
    Voronoi: because Voronoi had no recorder, `_plan()` fell back to timing
    `_debug_geometry()`'s *separate*, cheaper debug-only Voronoi map build
    (render density/node-count, not the real planning ones) and subtracted
    that from the real total -- which dumped almost the entire real
    map-generation cost into the "Plan" (search) label, making Voronoi look
    search-bound when finding 1 and decision 6 both already showed it isn't
    (search is cheap; map generation is where the time goes). `PlannerPreview.map_time_ms`
    is now `float | None`: `None` means *this call* took a direct-path/
    reused-previous-path shortcut and never built anything (true for any of
    the three planners, not a Voronoi-specific limitation anymore), shown
    as "Map: n/a (shortcut taken, no full build)" / "Plan (total): X ms";
    a real split shows as "Map: X ms" / "Plan (search only): X ms". Two new
    tests in `tests/test_scenario_planner_ui.py` lock in both cases for
    Voronoi specifically (shortcut-taken and forced-full-build).

## Future work (not started)

**Robots don't currently avoid each other's *planned paths*, only each
other's current position.** `ResearchRuntime._obstacles_for_robot`
(`ui/runtime.py`) builds every other robot as a static `PlanningObstacle` at
`other.start_mm` with `vel_mmps` defaulted to `(0, 0)` -- so whether robots
are planned serially or in parallel (decision 1), none of them know about
teammates' newly-computed routes. Two robots can each produce an
individually valid path that still crosses another robot's route mid-run.
This is a correctness gap, not a concurrency one -- running the same
independent per-robot calls faster doesn't fix it.

The standard fix is **prioritized (sequential) planning**: plan robot 1
with no knowledge of the others, then plan robot 2 treating robot 1's
*entire chosen path* (as a time-windowed corridor, not just a static
circle) as a dynamic obstacle, then robot 3 against robots 1+2's paths, and
so on. A single fully joint plan (one search over the combined
configuration space of all robots at once) would be the "more correct"
version of this, but blows up combinatorially with robot count -- not
something to reach for at 6 robots without a strong reason.

**This is in direct tension with decision 1.** Prioritized planning is
inherently sequential -- robot N's scene depends on robot N-1's
already-computed path -- which conflicts with parallelizing robots'
independent plan calls for UI responsiveness. Adopting it would trade some
or all of decision 1's concurrency for inter-robot collision correctness,
which is a real trade worth making deliberately, not a drop-in addition.

No action taken on the prioritized-planning idea itself -- explicitly
deferred per request, to revisit when there's time to design it properly
(robot ordering/priority scheme, how a "planned path so far" becomes a
time-windowed obstacle, and how far to relax decision 1's parallelism to
make room for it). The separate, smaller gap this future-work note
originally flagged alongside it -- that the codebase's existing
velocity/prediction mechanism was never wired into any planning call path
-- has since been closed (decision 5).

- **A dedicated results/export folder.** `scripts/demo_planners.py` writes
  `demo_output.png` to the repo root; the intent is a proper results/export
  directory instead, likely superseding the ad hoc root-level PNG.
  Deliberately not built yet -- noted here so it isn't lost.
- **A policy/validation layer shared across all three planners** -- check a
  candidate path against required rules (clearance being the first
  candidate) and reject/replan if it fails, independent of whatever
  internal collision logic the originating planner used. This is the same
  shape as decision 6 (remove planner-specific implementation quirks as
  confounds) applied one level up: a shared *validator*, not just a shared
  *search*. Precedent exists both in general real-time robotics (the
  Simplex/safety-shield pattern: a fast primary planner's output is
  checked by an independent, simpler monitor before use) and in this
  project's own history (TeamControl's `voronoi_game_navigator.py` already
  layers rule checks -- penalty-box guard, field-margin sanitisation --
  on top of the raw `VoronoiDijkstraPlanner` output; see
  `docs/voronoi-navigator-stripped.md` in the sibling `2026-TeamControl`
  repo). Framing worth keeping if this is picked up: a validation layer
  strengthens the existing planner-comparison contribution (fast-but-unsafe
  shouldn't count) rather than replacing it -- scoped as a smaller,
  well-defined addition (one shared `validate_path(...)` check reusable
  across all three planners' outputs) rather than a large new subsystem,
  given the ACRA submission deadline this work is targeting.
- **A denser/adversarial validation scenario for decision 9's config
  change** -- the two saved scenarios and a light stress test aren't enough
  to trust 10%/100 density is safe outside of them. Needed before treating
  those numbers as final for the paper.
- **A final call on the exact production density config** -- decision 9
  shipped 10%/100 but flagged 30% as a possible middle ground; not yet
  decided which one is the number this project reports.

## Status

Accepted and implemented: decisions 1, 3, 4, 5, 6, 7, 8, 9, and 10 are live
in `ui/runtime.py`, `ui/app.py`, `voronoi_dijkstra.py`,
`waypoint_manager.py`, `visibility_graph.py`, `prm_dijkstra.py`,
`planner_variables.yaml`, and `scripts/demo_planners.py`; all tests pass.
Decision 2 (process pool) remains deliberately deferred, as does the
prioritized-planning idea and the policy/validation-layer idea in "Future
work" -- and decision 9 opened two more open items there (validation
scenario, final density number). Revisit decision 2 when real per-tick
planning latency for a full 6-robot team becomes a measured bottleneck (not
just single-robot cases); revisit decision 9's open items before the
planner-comparison numbers go into the paper, since they're currently the
least validated claim in this document.
