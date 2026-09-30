# Headless simulation

For validation with the actual grSim/ODE physics engine, use
[`--backend grsim`](physics.md). The fast kinematic backend described below
remains the default.

The headless runner executes saved scenarios on a deterministic virtual clock.
It loads the same scenario files and planner implementations as the Qt
application, but it does not open Qt, send UDP packets, start grSim, or sleep
between control ticks.

This makes planner experiments much faster than real time and suitable for
repeatable local batches or CI.

## Quick start

Install the project, then run every saved scenario against every planner:

~~~shell
pip install -e .
research-sdk-headless
~~~

Run one scenario and repeat each selected planner 20 times:

~~~shell
research-sdk-headless scenarios/crowded.json +  --planner visibility prm voronoi +  --trials 20 +  --seed 100 +  --output-dir results/crowded_batch
~~~

Python module invocation is equivalent:

~~~shell
python -m research_sdk.headless scenarios/crowded.json --planner all
~~~

Directories are accepted as inputs and every JSON file directly inside them is
run. With no input, the command uses the *scenarios/* directory.

## Simulation model

The runner mirrors the current application at the behavioral boundary:

1. Build planner obstacles from the selected scenario and the other robots.
2. Plan an initial route per robot with the selected production planner
   adapter. A robot without a route waits at its start and retries once per
   check interval.
3. Execute all robot waypoint paths concurrently, replanning according to the
   selected policy.
4. Apply the same proportional controller gain, configured maximum speed,
   120 mm intermediate tolerance, and 60 mm final tolerance.
5. Advance moving obstacles (patrol routes or drifting motion) as a pure
   function of time.
6. Check robot–robot and robot–obstacle clearance between consecutive states
   (swept), recording 30 mm buffer episodes and physical-contact episodes.
7. Apply the failure rules described under
   [ACRA 2026 result sets](#acra-2026-result-sets).

The virtual clock always advances by *--dt-ms*. By default the runner does not
sleep and therefore runs as fast as the host allows. The reported real-time
factor is simulated duration divided by wall-clock runtime.
`--time-scale 1|10|100|200|500` optionally adds wall-clock pacing to cap each
kinematic run at the selected ratio without changing its virtual timestep or
outcome. A target is not guaranteed when planning itself takes longer than the
corresponding wall-clock budget.

This is a point-kinematic research backend, not grSim rigid-body physics. It
does not model wheel dynamics, acceleration, friction, ball contact, radio/UDP
latency, or vision noise. Use it for fast algorithm screening and regression
batches; use real grSim for final physics validation. Every result manifest
records this distinction with *physics_equivalence* set to false.

## Result files

Each invocation creates:

| File | Contents |
|---|---|
| **runs.csv** | One flat row per scenario, planner, and trial |
| **runs.json** | The same raw records as JSON |
| **summary.csv** | One row per scenario set, planner and arm in the revised outcome model (see [Summary columns](#summary-columns-summarycsv)) |
| **summary.json** | The same summary as JSON |
| **summary_legacy.csv** | The pre-DEC-016 summary (collision-free rate and related columns), kept for older analysis scripts |
| **rebuild_calls.csv** | Every post-initial rebuild attempt with its latency |
| **manifest.json** | Engine model, configuration, failure rules, planners, scenarios, seeds, code revision and (for a bank run) the scenario-bank fingerprint |

Raw result columns include:

- completion status, planning failures, and failed robot IDs;
- total and mean planning latency;
- simulated duration, wall duration, and real-time factor;
- outcome (`S`/`B`/`P`/`I`/`X`), swept robot–robot and robot–obstacle
  contact and buffer episodes, first-contact time;
- stopped robots and the episode end reason;
- initial retries and waiting time (per run and per robot in
  `initial_route_by_robot`), route count, lifetime and end reason
  (`route_ends_replaced`, `_goal`, `_stopped`, `_episode_end`);
- post-initial rebuild latency (min, max, total) and initial-plan stage timing;
- sampled (legacy) robot and obstacle collision episodes;
- planned and travelled path length;
- mean and maximum final target error;
- minimum clearance, where a negative value is penetration depth; and
- tick count and virtual timestep.

`summary.csv` follows the revised outcome model; `summary_legacy.csv` keeps the
earlier columns (completion rate, collision-free rate, planning latency
percentiles, path lengths, final error and worst clearance).

## Useful options

~~~text
--planner all|voronoi|prm|visibility
--trials N
--seed N
--dt-ms MILLISECONDS
--max-sim-seconds SECONDS
--time-scale 1|10|100|200|500     # omit for maximum throughput
--speed-mps METRES_PER_SECOND
--gain PER_SECOND
--output-dir PATH
--scenario-bank DIR               # run exactly the JSON files in DIR
--expect-scenarios N
--result-set one-shot-validation|policy-comparison|horizon-sweep
--slow-call-limit-ms MS           # default 100; 0 disables
--no-route-limit-ms MS            # default 500; 0 disables
--fail-on-incomplete
~~~

The *--fail-on-incomplete* option returns exit status 2 when any trial times
out or has a planning failure, which is useful in automated regression jobs.

## Python API

~~~python
from research_sdk.headless import SimulationConfig, run_experiments, write_results
from research_sdk.ui.scenarios import ScenarioStore

scenario = ScenarioStore().load('crowded')
config = SimulationConfig(dt_s=0.01, max_simulation_s=20.0)
results = run_experiments(
    [scenario],
    ['visibility', 'prm'],
    trials=10,
    seed=42,
    config=config,
)
write_results(results, 'results/headless_example', config=config)
~~~

## Replanning experiments

| Option | Meaning |
|---|---|
| `--policy once cycle event` (or `all`) | Plan once; rebuild every replan period; or check every period and rebuild only when the reroute trigger fires |
| `--replan-ms 20 50 100 200` | Replanning period(s). Each value is a separate arm |
| `--predict-ms 0 50 100` | Obstacle motion-prediction horizon(s), same model as the UI's *Predict motion*. Each value is a separate arm |
| `--clearance-mm` | Extra planning clearance |
| `--random N` | N seeded random scenarios (`--robots`, `--obstacles`, `--moving-fraction`, `--obstacle-speed`) |
| `--patrol-fraction F`, `--patrol-points K` | Share of moving obstacles that patrol K random waypoints instead of drifting |
| `--perturb N` | N jittered copies of each saved scenario (`--jitter-mm`) |
| `--workers N` | Parallel processes (0 = all CPUs). Use 1 for timing numbers |

Obstacle motion is a pure function of time, so every planner and policy sees the identical moving world.

- **Patrol obstacles** move back and forth along *spawn → waypoint 1 → … → waypoint n* at `patrol_speed_mmps` (default 800 mm/s). The UI runtime uses the same route.
- **Drifting obstacles** move at constant velocity and bounce off the field walls.

Example (6 planned robots against 6 patrolling obstacles):

~~~shell
research-sdk-headless --random 200 --robots 6 --obstacles 6 --moving-fraction 1 \
  --patrol-fraction 1 --planner all --policy all --predict-ms 0 100 --replan-ms 20 100
python scripts/analyse_sweep.py results/<folder>      # heatmaps + best settings
python scripts/analyse_matrix.py results/<folder>     # per-arm tables and paired tests
~~~

### Additional metrics (runs.csv)

| Column | Meaning |
|---|---|
| `replan_count`, `replan_failures` | Route rebuilds after t = 0, and rebuild attempts that returned nothing |
| `rebuild_calls` / `check_calls` | Planner calls that produced a route vs. calls that only confirmed the current one |
| `planning_time_ms_rebuild` / `planning_time_ms_check` | Time spent in each kind of call (policy-check cost vs. actual planning cost) |
| `replans_active_blocked`, `replans_route_finished`, `replans_other`, `replans_scheduled` | Why each rebuild happened, judged from the executor side. *Scheduled* means the every-cycle policy |
| `direct_path_switches` | Times a robot dropped its route because the straight line became clear |
| `path_shift_mm_mean` / `_max` | Route churn: mean distance of a rebuilt route's first 1.5 m from the route it replaced |
| `heading_change_rad_per_m`, `sharp_turns` | Smoothness of the executed motion (total turning per metre; turns over 90°) |
| `minimum_robot_clearance_mm`, `minimum_obstacle_clearance_mm` | Closest approach to another robot and to an obstacle. Negative means overlap |
| `contact_time_ms` | Simulated time spent in any contact |
| `prediction_horizon_ms`, `replan_period_ms` | The settings for the run |

`summary_legacy.csv` also reports `collision_probability` (share of runs with at least one contact) with a 95% Wilson interval, `rebuild_share_of_planning_time`, and the mean cost of a check call and of a rebuild call.

The kinematic backend has no tracking-error metric. Robots follow their waypoint polyline exactly, apart from the waypoint-switch tolerance, so tracking error is only meaningful with the grSim backend.

## ACRA 2026 result sets

The final ACRA 2026 results come from three fixed result sets, all run on the
frozen scenario bank `scenarios/acra2026-200` (200 scenarios, six robots, six
patrol obstacles). `--result-set` fixes the arms and the scenarios, so nothing
else can be mixed in.

| Result set | Arms (policy, check interval, prediction horizon) | Runs |
|---|---|---:|
| `one-shot-validation` | Frozen obstacles, one route per robot, nothing executed | 600 |
| `policy-comparison` | cycle and event at 20 ms and 100 ms, horizon 0 ms | 2,400 |
| `horizon-sweep` | event at 20 ms, horizon 20 / 50 / 150 ms (the 0 ms point is the policy comparison's event @ 20 ms arm) | 1,800 |

~~~powershell
python -m research_sdk.headless --result-set one-shot-validation --output-dir results/acra2026-final/one-shot-validation
python -m research_sdk.headless --result-set policy-comparison --workers 1 --output-dir results/acra2026-final/policy-comparison
python -m research_sdk.headless --result-set horizon-sweep --workers 1 --output-dir results/acra2026-final/horizon-sweep
python scripts/build_result_tables.py
~~~

Use `--workers 1` on one named machine for the dynamic sets: the replan time
limit and all latency columns are wall-clock measurements.
`build_result_tables.py` writes the three paper tables as CSV and Markdown to
`results/acra2026-final/tables/`.

### Scenario bank rules

- `--scenario-bank DIR` runs every `*.json` directly in `DIR` and nothing else.
- `--random`, `--perturb` and `--no-saved` are rejected with a bank.
- Repeated scenario names are rejected; `--expect-scenarios N` fails the run
  unless the bank holds exactly N scenarios. `--result-set` sets it to 200 for
  the default bank.
- Every scenario in a bank run is labelled with the bank's folder name, so
  `summary.csv` aggregates over the whole bank. The manifest records the bank
  path, file count and a SHA-256 fingerprint (line endings normalised).

### Outcome model

Each run gets one outcome, by highest severity:

| Code | Meaning |
|---|---|
| `S` | Completed; no clearance below 30 mm and no contact |
| `B` | Completed; entered the 30 mm buffer but made no physical contact |
| `P` | Completed with at least one robot–robot or robot–obstacle contact |
| `I` | Not completed (a robot missed its goal, was stopped, or the episode failed) |
| `X` | Invalid scenario: a robot already overlaps a robot or obstacle at t = 0 |

Contact and buffer checks are swept: each pair's minimum clearance is taken
over the straight-line motion between consecutive 20 ms states, so a contact
that starts and ends between two samples is still counted. A sustained overlap
is one episode; separating and touching again is a second. Obstacle–obstacle
contact is not recorded.

### Initial route and failure rules

- A robot with no initial route waits at its start and retries once per check
  interval. Retries and waiting time are recorded.
- **Replan time limit:** a planning call after the initial one that takes more
  than 100 ms of wall time (`--slow-call-limit-ms`) stops that robot, flagged
  `time_limit`. The rest of the team continues. The call's time includes the
  route check, the map or roadmap rebuild and every Dijkstra search in the
  call (for PRM, up to its 5 resampling attempts).
- **No-route limit:** any robot that goes 500 ms of simulated time without a
  valid route (`--no-route-limit-ms`) fails and ends the episode, flagged
  `no_valid_path`.
- The initial plan is exempt from the time limit and reported as one-shot
  timing. Passing 0 to either option disables that rule.

### Summary columns (`summary.csv`)

One row per planner and arm. Counts keep their denominators.

| Column | Meaning |
|---|---|
| `completed_label` | Completed runs out of valid runs, e.g. `174/200` |
| `strict_label`, `buffer_only_label`, `physical_label` | S, B and P out of completed runs |
| `incomplete_label` | I out of valid runs |
| `*_rate` | The same shares as fractions; `safety_compromised_rate` = (B + P) / completed |
| `rr_contacts_per_failed_run`, `ro_contacts_per_failed_run`, `contacts_per_failed_run` | Contact episodes in P runs divided by B + P |
| `contacts_per_physical_run` | Contact episodes divided by P |
| `contacts_in_incomplete_runs` | Audit: contact episodes inside I runs |
| `ro_contacts_obstacle_initiated_share` | Diagnostic: share of robot–obstacle contacts the obstacle closed (DEC-008 attribution; not used in outcomes) |
| `episodes_failed_no_route`, `robots_stopped_time_limit` | Failure-rule counts |
| `initial_retries_per_run`, `initial_wait_ms_per_run`, `initial_wait_ms_max` | Initial no-route cost |
| `replans_per_run`, `failed_replans_per_run`, `blocked_time_pct_mean` | Replanning demand; blocked time uses the DEC-015 normalisation |
| `routes_per_run`, `route_lifetime_ms_mean`, `route_lifetime_ms_max` | How long installed routes stay in use |
| `replan_ms_min`, `_mean`, `_p95`, `_max` | Pooled over every individual post-initial rebuild attempt |
| `event_check_ms_mean` | Mean cost of a check that kept the route |
| `initial_plan_ms_mean`, `initial_search_ms_mean`, `initial_map_ms_mean` | Initial plan split into Dijkstra search and everything else |

Per-run values are means over valid runs. Each individual rebuild latency is in
`rebuild_calls.csv`, so any latency statistic can be recomputed.

### One-shot validation output

`robots.csv` holds one row per scenario, planner and robot: route availability,
total planning time split into `search_ms` (time inside the shared NetworkX
`dijkstra_path` call) and `map_ms` (the rest of the call), path length, path
excess over the straight line, turning per metre and turns sharper than 90°.
`summary.csv` has one row per planner.
