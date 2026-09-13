# Three trigger policies, live in grSim: no trigger vs. production trigger vs. geometric baseline

Same three backends, same two obstacle layouts, same grSim setup as `section_geometric_trigger.md`,
now varied across three replanning policies instead of one:

- **No trigger** (`--trigger always`): replan every single control tick (20/s), regardless of
  whether anything in the world changed. The absence of any event-gating at all.
- **Event trigger, production** (`--trigger emma`): `research_sdk.planners.reroute.evaluate_route`,
  the trigger actually shipping in `backbackup`'s `VoronoiWaypointManager` -- direct-line check,
  then target-moved / route-finished / active-waypoint-blocked / a periodic safety net every 5
  frames. Checks only the single next waypoint, not the whole path.
- **Geometric** (`--trigger geometric`, reported separately in `section_geometric_trigger.md`):
  da Silva Costa & Tonidandel's own trigger -- replan if any obstacle comes within 90mm of the
  *entire* remaining path.

All obstacles at 1.0 m/s, robot at 1.5 m/s, 90mm invalidation threshold where applicable,
30s per-trial timeout.

**Correction, all six PRM rows below were regenerated.** The first pass through this data found
PRM at 0/60 under no-trigger in scenario 1 -- a complete, uniform collapse. That turned out to be
a bug in `drive_grsim.py`'s own PRM wrapper, not a real property of PRM: it never varied PRM's
random seed across calls, so every single replan rebuilt the *identical* 40-point roadmap instead
of resampling fresh points each time, the way `dynamic_scenario.py`'s offline wrapper does (it
passes the running replan count as the seed). With a fixed seed, if that one roadmap happens not
to connect start to goal from the robot's live position, every call fails, forever -- which is
exactly what a 30s-per-trial, 0.5ms-per-call, near-zero-variance failure looks like. Once the
wrapper was fixed to vary the seed per call, all six PRM conditions were re-run at n=60 and the
numbers below are the corrected results. See the setup notes for how this was found.

## Scenario 1, seven obstacles

| Backend    | Trigger      | n  | Arrived | Replans | Nav (s) | Collisions | Reversals |
|------------|--------------|---:|--------:|--------:|--------:|-----------:|----------:|
| Voronoi    | No trigger   | 60 |   60/60 |  132.57 |    7.29 |       1.43 |      3.82 |
| Voronoi    | Event (prod.)| 60 |   60/60 |   23.93 |    7.59 |       0.93 |      1.95 |
| Voronoi    | Geometric    | 60 |   60/60 |   14.73 |    7.02 |       0.57 |      0.55 |
| PRM        | No trigger   | 60 |   60/60 |  127.53 |    7.53 |       2.57 |  **16.53** |
| PRM        | Event (prod.)| 60 |   60/60 |   29.22 |    7.24 |       1.63 |      2.15 |
| PRM        | Geometric    | 60 |   60/60 |   20.62 |    6.90 |       1.02 |      1.08 |
| Visibility | No trigger   | 60 |   59/60 |  166.83 |   11.67 |       2.93 |      0.30 |
| Visibility | Event (prod.)| 60 |   60/60 |   33.98 |    4.06 |       1.92 |      0.18 |
| Visibility | Geometric    | 60 |   60/60 |   45.90 |    4.72 |       2.35 |      0.17 |

## Scenario 2, five obstacles

| Backend    | Trigger      | n  | Arrived | Replans | Nav (s) | Collisions | Reversals |
|------------|--------------|---:|--------:|--------:|--------:|-----------:|----------:|
| Voronoi    | No trigger   | 60 |   60/60 |  132.70 |    7.85 |       1.67 |      4.68 |
| Voronoi    | Event (prod.)| 60 |   60/60 |   34.03 |    8.00 |       1.48 |      3.55 |
| Voronoi    | Geometric    | 60 |   60/60 |   11.98 |    7.48 |       0.43 |      0.78 |
| PRM        | No trigger   | 60 |   60/60 |  158.10 |    9.42 |       1.85 |  **24.45** |
| PRM        | Event (prod.)| 60 |   60/60 |   37.22 |    9.32 |       1.60 |      4.60 |
| PRM        | Geometric    | 60 |   60/60 |   18.28 |    6.12 |       0.93 |      0.73 |
| Visibility | No trigger   | 60 |   60/60 |  101.38 |    5.17 |       1.73 |      0.10 |
| Visibility | Event (prod.)| 60 |   60/60 |   23.40 |    3.83 |       1.15 |      0.08 |
| Visibility | Geometric    | 60 |   60/60 |   34.77 |    4.32 |       1.33 |      1.33 |

All eighteen batches above (six no-trigger, six event-trigger, six geometric) are n=60 and
complete. Several needed more than one attempt -- see the setup notes at the bottom for both the
grSim-crash issue and the PRM seed bug.

## What this shows

**Removing the trigger entirely does not break any backend's completeness -- but it breaks PRM's
directional stability far worse than the other two.** All three backends arrive (near-)100% of
the time under no-trigger in both scenarios. What separates them is reversals: PRM's no-trigger
reversal rate is **16.53 and 24.45** (scenario 1 and 2), against Voronoi's 3.82/4.68 and
Visibility's 0.30/0.10. PRM is flipping direction 4-6x more often than Voronoi and 50-250x more
often than Visibility under the same no-gating condition, yet none of that stops it from arriving.

**This directly contradicts the offline "reversals predict arrival" rule.** The offline stability
study found a clean threshold -- at or below 5.9 reversals/run always arrives, at or above 14.7
always fails, nothing in between. PRM's live reversal counts here (16.53, 24.45) sit well inside
the offline "always fails" zone, and yet PRM arrived 60/60 in both. That rule was derived from one
backend (Voronoi) under one kind of trigger (a timer) offline; it does not transfer as a general
law to a different backend under a different condition live. Worth stating plainly rather than
forcing the offline rule to fit: reversal rate still separates backends live, but the specific
numeric threshold for failure does not carry over.

**Any real trigger reduces PRM's reversal rate by roughly an order of magnitude.** From
16.53/24.45 under no-trigger to 2.15/4.60 under Emma's trigger to 1.08/0.73 under geometric --
each step, checking more of the path (nothing, then one waypoint, then the whole route) cuts
reversals further. Voronoi shows the same staircase, just less dramatically: 3.82/4.68 (none) to
1.95/3.55 (Emma) to 0.55/0.78 (geometric).

**Visibility barely reacts to which trigger is used.** Its reversal rate stays at or under 0.30
across all three triggers in both scenarios -- the same directional-stability property that showed
up in the offline stability study. Its replan count is actually *lower* under Emma's trigger than
under geometric in both scenarios (33.98 vs 45.90 in scenario 1, 23.40 vs 34.77 in scenario 2) --
the single-waypoint check apparently suits a backend whose paths already run close to a unique
shortest route.

## Setup notes

**grSim crashes under sustained no-trigger load.** `--trigger always` at n=60 caused repeated,
complete grSim crashes partway through the batch on the first attempt for three of the six
no-trigger runs (PRM/scenario 2, Visibility/scenario 1, and PRM/scenario 1 -- all-or-nothing:
either the whole batch ran clean, or every trial failed identically from sample 1, never a
gradual degradation). This happened specifically after the heaviest `always`-trigger runs
(PRM/scenario 1 at the time was 60 trials each timing out at 30s with ~588 replans -- 30 minutes
of near-continuous maximum-rate replanning, itself a symptom of the seed bug below). The
immediate fix was dropping to n=20 for the affected runs; all three were later re-run and
completed cleanly at n=60 after confirming grSim connectivity with a short smoke test first.

**PRM's fixed-seed bug.** `drive_grsim.py`'s `_prm()` wrapper never passed a `seed` to
`prm_dijkstra.plan()`, so every call defaulted to `seed=0` -- the identical 40-point roadmap,
every single tick, forever. `dynamic_scenario.py`'s offline wrapper avoids this by passing the
running replan count as the seed on every call, so it resamples fresh points each time; live
never did. With a fixed roadmap that happened not to connect start to goal from the robot's live
position, every PRM call in scenario 1 under no-trigger failed identically (0/60 arrived, ~588
replans/trial, ~0.5ms per call -- 10-50x faster than a real PRM attempt takes, the tell that it
was failing fast rather than actually searching). Confirmed by inspecting the saved
`mean_heading_deg`/`mean_shift_mm`/`replans_compared` fields, which were ~0 in nearly every
trial -- no real path was ever produced to compare against the last one. Fixed by threading the
live replan counter through as `seed` (`plan_for(..., key=replans)`), matching the offline
methodology. All six PRM rows in the tables above are the corrected, re-run results.

## Raw data

All 18 batches saved under `results/`:
`grsim_notrigger_{voronoi,prm,visibility}_s{1,2}_n60.json`,
`grsim_eventtrigger_{voronoi,prm,visibility}_s{1,2}_n60.json`, and
`grsim_geometric_{voronoi,prm,visibility}_s{1,2}_n60.json` (referenced from
`section_geometric_trigger.md`), each with a matching `.log`.
