# Result metrics and calculation method

Each export contains one data row for the most recent experiment run. All
durations use Python's monotonic high-resolution clock (`perf_counter`), so a
wall-clock correction cannot make a duration negative. CSV serialization and
Qt rendering occur outside planner timings.

Let `L_i` be pipeline latency for completed vision frame `i`, `M_i` its mapping
time, and `P_j` the duration of planner invocation `j`.

## Format A

| Column | Equation | Data included |
| --- | --- | --- |
| `input_latency_ms` | `sum(L_i) / n` | From entry of the packet that completes a camera frame through snapshot creation, world-map update, and planner-scene creation. It is processing latency, not UDP travel time or packet inter-arrival time. |
| `mapping_time_ms` | `sum(M_i) / n` | `WorldMap.update(snapshot)` plus `WorldMap.planning_scene(...)` for each completed frame. Snapshot assembly is excluded. |
| `planning_time_ms` | `sum(P_j)` | All robot-specific planner calls made by the run. Scenario conversion, drawing, velocity conversion, UDP sending, and CSV writing are excluded. |
| `number_of_fails` | `sum(f_j)` | A planner call contributes `f_j = 1` when it raises an exception; otherwise `0`. |

## Format B

| Column | Equation | Data included |
| --- | --- | --- |
| `input_latency_ms` | `sum(L_i) / n` | Same completed-frame processing measurement as format A. |
| `average_planner_execution_time_ms` | `sum(P_j) / N` | Mean duration of the `N` robot-specific planner calls in the run. |
| `robot_arrival_time_ms` | `(t_arrived - t_execution_start) * 1000` | Starts immediately before control ticks begin and ends when every active path is complete. A final waypoint is reached within 60 mm; intermediate waypoints use 120 mm. It is blank for a stopped or failed run. |
| `total_plans_made` | `N` | Number of robot-specific planner invocations, normally one per planned robot in the current run. |
| `number_of_collisions` | `sum(c_k)` | Rising-edge collision episodes from SSL-Vision robot positions. A pair enters collision when centre distance is at most `2 * ROBOT_RADIUS_MM`. Repeated control ticks while still touching count once; separation followed by contact starts another episode. The ball is excluded. |
| `resources_used` | `(cpu_end - cpu_start) * 1000` | Process CPU time in milliseconds consumed by the whole console process during the run, including vision and control but excluding time the process was idle. |

If no completed camera frame arrives during a run, the two vision-derived mean
fields are blank rather than reported as zero. This distinguishes missing data
from a real zero-duration measurement.

## Data flow

```text
SSL-Vision packet
  -> camera-frame assembler
  -> immutable WorldSnapshot
  -> WorldMap update + PlanningScene       [L_i and M_i]
  -> planner call for each scenario robot  [P_j, N, failures]
  -> 50 ms control ticks + vision feedback [arrival and collision episodes]
  -> finalized events.jsonl
  -> format A or B CSV
```

## Accuracy cross-check

The automated checks calculate the expected values independently from fixed
samples: latencies `(2, 4)` ms must average `3` ms, mapping `(1, 3)` ms must
average `2` ms, and plans `(5, 15)` ms must total `20` ms and average `10` ms.
An execution interval of `1.25` seconds must export `1250` ms and a CPU interval
of `0.025` seconds must export `25` ms. A separate test verifies that persistent
contact across repeated ticks counts once and that contact after separation
counts as a new collision. Pipeline tests also assert that mapping time is
non-negative and does not exceed total processing time.

Run the cross-check with:

```bash
.venv/bin/pytest tests/test_ui_session.py tests/test_world_pipeline.py
```
