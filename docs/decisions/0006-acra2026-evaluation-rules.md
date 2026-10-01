# 0006: ACRA 2026 evaluation rules (DEC-005 to DEC-023)

Date: 2026-10-01

## Context

The code, result files and docs refer to decision ids such as `DEC-016` and
`DEC-019`. Those decisions were made in the project's research notes. This
record summarises every decision that shapes the ACRA 2026 result sets, so
each id used in the repository can be resolved here. The ids are kept as
they are; gaps in the numbering are decisions that do not affect the
evaluator.

## Decisions

| Id | Decision | Where it shows up |
|---|---|---|
| DEC-005 | All three planners use the same NetworkX Dijkstra (`networkx.dijkstra_path`) search and the same obstacle positions. Only roadmap construction differs. | `planners/common.py` `timed_dijkstra_path` |
| DEC-008 | Robot–obstacle contacts are attributed to whichever party closed the gap faster at the tick contact began. Superseded for the outcome by DEC-017; kept as a diagnostic. | `ro_contacts_obstacle_initiated_share` |
| DEC-010 | The main finding is framed as a planner-dependent compute–safety trade-off. The `event_route` trigger, labelled *full-path recalculation* in figures and tables (rebuild when any remaining segment is blocked, the practice Costa and Tonidandel, RoboCup 2023, LNAI 14140, 2024, describe for SSL planners without a separate avoidance layer), is a negative ablation that must be reported. | `supporting/event-route-ablation` |
| DEC-014 | Effective planning margin is equalised at 120 mm per planner: Voronoi 0 mm extra (its corridors already enforce 120 mm), PRM and Visibility Graph 30 mm. | `planner_variables.yaml` |
| DEC-015 | Failed planning attempts are reported as planner load, normalised by check period, simulated duration and team size, never as raw counts across periods. | `blocked_time_pct_mean` |
| DEC-016 | Hierarchical run outcome: strict (S), buffer-only (B), physical contact (P), incomplete (I), invalid initial overlap (X). Each run takes its most severe class; all contact episodes are still counted. | `classify_outcome`, `summary.csv` |
| DEC-017 | Only robot–robot and robot–obstacle contacts are recorded, and both count. Obstacle–obstacle contact is ignored. Obstacle-initiated contacts count towards the outcome. | `count_new_episodes` |
| DEC-018 | Measurement set for final results: swept clearance between consecutive 20 ms states, a 30 mm buffer logged separately from contact, distinct contact episodes, initial-route retry, separate timing of map build / search / rebuild / event check, route lifetime, and a static one-shot validation set. | `headless.py`, `docs/headless.md` |
| DEC-019 | Failure rules after the initial plan: a replan taking over 100 ms of wall time stops that robot (`time_limit`); any robot without a route for 1000 ms of simulated time fails the episode (`no_valid_path`). The initial plan is exempt. PRM keeps a fixed cap of 5 resample attempts per call. | `slow_call_limit_ms`, `no_route_limit_ms` |
| DEC-020 | The prediction horizon sweep uses 0 / 20 / 50 / 100 ms (event policy, 20 ms checks). The 0 ms point is the policy comparison's event @ 20 ms arm. | `RESULT_SETS["horizon-sweep"]` |
| DEC-022 | All three planners share one escape step (`planners/reroute.py`, `escape_waypoint`): when the robot is inside one of the planner's own inflated obstacles, it steps straight out (`max(250 mm, overlap + 120 mm)`) instead of planning. Previously only Voronoi did this; PRM and the visibility graph refused to plan. Escape steps are counted as `escape_moves`, kept out of rebuild counts and rebuild latency. | `escape_moves`, `escapes_per_run` |
| DEC-023 | Voronoi's per-tick safety checks (event gate, escape zone, direct-line and previous-route checks) use `voronoi_check_margin_mm` = 30 mm, the same margin PRM and the visibility graph already check at. The Voronoi roadmap keeps its own corridor clearance (DEC-014). Setting it to 0 restores the earlier behaviour. | `planner_variables.yaml`, `VoronoiWaypointManager(check_margin_mm=...)` |
| DEC-021 | A collision-free completion rate, (S + B) / 200, is reported alongside the hierarchy as the one number comparable with success rates in other work. | `collision_free_completion_rate` |
| DEC-024 | The 100 ms replan limit of DEC-019 is measured but no longer enforced in the main results: rebuilds over 100 ms are counted and reported, robots are not stopped. The 1 s no-route rule stays. Simulated time is paused during planner calls, so the main results are deterministic and machine-independent. The set run with the limit enforced is kept as supporting evidence. | `--slow-call-limit-ms 0`; `results/_archive-dec024-with-time-limit/` |

## Why the limits have the values they do

- **100 ms replan limit.** About six 60 Hz control frames of blind motion.
  Because it is wall-clock time, the dynamic sets are run single-worker on
  one named machine, and `time_limit` stops are reported separately from
  `no_valid_path` failures.
- **1000 ms no-route limit** (500 ms before 2026-10-01). An obstacle centred
  on a robot's path must move about 210 mm (robot radius + obstacle radius +
  30 mm buffer) to clear it. Patrol speeds in the bank are 300–1000 mm/s, so
  clearing takes up to 0.7 s for the slowest obstacle. 500 ms did not cover
  obstacles slower than about 400 mm/s. The value comes from this
  derivation, not from completion counts.
- **100 ms maximum horizon** (150 ms before 2026-10-01). At 150 ms the
  predicted obstacles grew enough that PRM and the Visibility Graph mostly
  failed to find any route in a 20-scenario preview, so that arm measured
  route blocking rather than the benefit of prediction.

The 150 ms horizon and 500 ms limit were checked only on a 20-scenario
preview. The result sets `horizon-150-check` and `no-route-500-check` exist
to repeat those checks on all 200 scenarios but were not run for the paper.

## Consequences

- All final claims come from `results/acra2026-final/`, generated from the
  frozen bank `scenarios/acra2026-200`. Earlier batches are archived as
  historical evidence and are not used for final numbers.
- Replan timing is wall-clock time and depends on the machine. Reported
  latencies and `time_limit` stops apply to the machine recorded in each
  `manifest.json` (CPU, core count, memory, OS, Python and package versions).
- The simulation itself is deterministic. The final policy, horizon and
  full-path sets were run twice at the same revision; every run that no
  `time_limit` stop touched in either batch is identical in both (4,891 of
  4,891: same outcome, same rebuild count). Stopped runs differ between the
  two batches, so the stops are scheduling noise, not a property of a
  scenario. The chance of a single rebuild exceeding 100 ms is similar for
  event and cycle triggering; cycle triggering makes about ten times as many
  rebuilds and is stopped correspondingly more often.
- Unyielding patrol obstacles can cause contacts no policy can prevent. This
  is stated as a limitation, not removed from the outcome.
