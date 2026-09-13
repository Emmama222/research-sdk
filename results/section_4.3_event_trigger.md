# 4.3 Three Roadmaps with Event Trigger

## What this compares
- **Comparing:** the three planning backends (Voronoi, PRM, Visibility) against each other,
  under the production event trigger specifically.
- **Situation:** live in grSim, dynamic environment (moving obstacles), event trigger
  (`evaluate_route`, the trigger actually shipping in production), two obstacle layouts (7
  obstacles / 5 obstacles), obstacles at 1.0 m/s, robot at 1.5 m/s, 30s per-trial timeout.
- **n:** 60 trials per backend per scenario (6 batches, 360 trials total), seed 0.
- **Source data:** `results/grsim_eventtrigger_{voronoi,prm,visibility}_s{1,2}_n60.json`,
  each with a matching `.log`.

*(Note: no offline equivalent exists for this condition. `evaluate_route` is production code
from a different branch (`backbackup`) than any of Paul's offline experiments -- it was never
run through `dynamic_scenario.py`, so there's nothing to compare against here. This section is
live-only by nature, not by omission.)*

Same three backends, same two obstacle layouts, same grSim setup as section 4.2's no-trigger
run -- only the trigger changes.

### Scenario 1, seven obstacles

| Backend    | Arrived | Replans | Nav (s)  | Collisions | Reversals |
|------------|--------:|--------:|---------:|-----------:|----------:|
| PRM        |   60/60 |   29.22 |     7.24 |       1.63 |      2.15 |
| Visibility |   60/60 |   33.98 | **4.06** |       1.92 |  **0.18** |
| Voronoi    |   60/60 | **23.93**|    7.59 |   **0.93** |      1.95 |

### Scenario 2, five obstacles

| Backend    | Arrived | Replans | Nav (s)  | Collisions | Reversals |
|------------|--------:|--------:|---------:|-----------:|----------:|
| PRM        |   60/60 |   37.22 |     9.32 |       1.60 |      4.60 |
| Visibility |   60/60 |   23.40 | **3.83** |       1.15 |  **0.08** |
| Voronoi    |   60/60 | **34.03**|    8.00 |   **1.48** |      3.55 |

**What this shows.** Compared against 4.2's no-trigger baseline, any real gating collapses
replan counts sharply: PRM drops from 127.53/158.10 replans with no trigger to 29.22/37.22
here, Voronoi from 132.57/132.70 to 23.93/34.03, Visibility from 166.83/101.38 to 33.98/23.40
-- roughly a 3-5x cut just from checking the single next waypoint instead of replanning every
tick unconditionally. Reversals fall even more sharply for PRM specifically (16.53/24.45 with
no trigger down to 2.15/4.60 here), while Visibility's reversal rate barely moves (0.30/0.10
to 0.18/0.08) since it was already stable with no gating at all. Compared against the
geometric trigger (`section_geometric_trigger.md`), Voronoi replans more under this event
trigger (23.93/34.03 vs 14.73/11.98) despite the lighter, single-waypoint check, while
Visibility's replans actually fall below its geometric-trigger count in both scenarios
(33.98/23.40 vs 45.90/34.77) -- the single-waypoint check suits a backend whose shortest path
is already close to unique.

**Setup notes.** `--trigger always` (no trigger at all, used as a lower bound for
comparison during development) caused repeated, complete grSim crashes partway through the
batch on the first attempt for three of six runs -- all-or-nothing failures, never a gradual
degradation, specifically after the heaviest continuous-replanning load. Fixed by rerunning
after confirming grSim connectivity with a short smoke test first.

PRM's rows were regenerated after finding a bug in `drive_grsim.py`'s `_prm()` wrapper: it
never passed a `seed` to `prm_dijkstra.plan()`, so every call defaulted to the same fixed
roadmap instead of resampling fresh points per replan the way `dynamic_scenario.py`'s
offline wrapper does. This surfaced as a complete, reproducible 0/60 failure under
`--trigger always` in scenario 1 (identical roadmap, disconnected from the robot's live
position, every single call). Fixed by threading the live replan counter through as the
seed; all PRM rows here and in 4.2 are the corrected, re-run results.

Raw data: `grsim_eventtrigger_{voronoi,prm,visibility}_s{1,2}_n60.json`, each with a
matching `.log`, all in `results/`.
