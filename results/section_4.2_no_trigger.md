# 4.2 Three Roadmaps, No Trigger

## What this compares

- **Comparing:** the three planning backends (Voronoi, PRM, Visibility) against each other,
  with no replanning trigger at all -- replan every single control tick (`--trigger always`),
  regardless of whether anything in the world changed. The absence of an event trigger, not a
  weak one.
- **Situation:** live in grSim, dynamic environment (moving obstacles), two obstacle layouts
  (7 obstacles / 5 obstacles, `dynamic_scenario.py`'s `SCENARIO_1`/`SCENARIO_2`), obstacles at
  1.0 m/s, robot at 1.5 m/s, 30s per-trial timeout.
- **n:** 60 trials per backend per scenario live (6 batches, 360 trials total), seed 0. Offline
  comparison is n=200, from `dynamic_scenario.py`'s `periodic_50ms` trigger.
- **Source data:** live -- `results/grsim_notrigger_{voronoi,prm,visibility}_s{1,2}_n60.json`,
  each with a matching `.log`. Offline -- `results/offline_periodic_50ms_n200.txt`.

This is the ungated lower bound the other two trigger conditions (4.3's event trigger, and the
geometric trigger in `section_geometric_trigger.md`) are measured against.

**Offline has an exact mechanical equivalent: `periodic_50ms`.** `CONTROL_TICK_S = 0.05` (50ms)
in both scripts, so "replan every tick, unconditionally" (`--trigger always`, live) and "replan
every 50ms" (`trigger_periodic(1)`, offline) are the same condition, not just similar ones.

## Full comparison

Visibility misses one arrival out of 60 in scenario 1; every other cell is a clean 60/60.

### Scenario 1, seven obstacles

| Backend    | Arrived | Replans | Nav (s)  | Collisions | Reversals |
|------------|--------:|--------:|---------:|-----------:|----------:|
| PRM        |   60/60 |  127.53 |     7.53 |       2.57 | **16.53** |
| Visibility |   59/60 |  166.83 |    11.67 |       2.93 |      0.30 |
| Voronoi    |   60/60 | **132.57**|  7.29 |   **1.43** |      3.82 |

### Scenario 2, five obstacles

| Backend    | Arrived | Replans | Nav (s)  | Collisions | Reversals |
|------------|--------:|--------:|---------:|-----------:|----------:|
| PRM        |   60/60 |  158.10 |     9.42 |       1.85 | **24.45** |
| Visibility |   60/60 | **101.38**| **5.17**|  1.73 |      0.10 |
| Voronoi    |   60/60 |  132.70 |     7.85 |   **1.67**|      4.68 |

Bold marks the best value in each column, within each scenario (lowest replans/nav/collisions;
lowest reversals is best, but PRM's number is bolded above only to flag it as the standout --
read it as worst, not best).

## Live vs. offline, same condition

| Scenario | Backend | Arrived (offline -> live) | Replans (offline -> live) | Reversals (offline -> live) |
|---|---|---|---|---|
| 1 | PRM | 200/200 -> 60/60 | 76.67 -> 127.53 | 11.98 -> 16.53 |
| 1 | Visibility | 200/200 -> 59/60 | 58.08 -> 166.83 | 0.10 -> 0.30 |
| 1 | **Voronoi** | **17/200 (8.5%) -> 60/60 (100%)** | 559.13 -> 132.57 | **318.58 -> 3.82** |
| 2 | PRM | 200/200 -> 60/60 | 62.84 -> 158.10 | 6.73 -> 24.45 |
| 2 | Visibility | 200/200 -> 60/60 | 53.75 -> 101.38 | 0.04 -> 0.10 |
| 2 | **Voronoi** | **1/200 (0.5%) -> 60/60 (100%)** | 597.32 -> 132.70 | **361.12 -> 4.68** |

**This is the sharpest live/offline disagreement in the whole project.** Offline predicts
Voronoi almost completely collapses under this exact condition -- 8.5% and 0.5% arrival, with
reversal counts (318.58, 361.12) so high they're an order of magnitude past the offline study's
own "always fails" threshold of 14.7. Live, under the identical mechanism, Voronoi arrives
100% of the time in both scenarios, with reversals two orders of magnitude lower (3.82, 4.68)
than offline predicted. This isn't a magnitude gap like the geometric-trigger comparison
(`section_geometric_trigger.md`) -- it's a reversal of the conclusion itself. PRM and Visibility
don't show this: both stay complete offline and live, and their reversal counts move in the
same direction (up, live) that the geometric-trigger comparison already established as typical.

## What this shows

**Removing the trigger entirely does not break completeness, but it wrecks PRM's directional
stability.** All three backends still arrive (near-)100% of the time with no gating at all.
What separates them is reversals: PRM flips direction **16.53** and **24.45** times per trial
(scenario 1 and 2) against Voronoi's 3.82/4.68 and Visibility's 0.30/0.10 -- 4-6x more often
than Voronoi and 50-250x more often than Visibility, yet none of that stops PRM from arriving.

**This is a second, independent contradiction of the offline "reversals predict arrival" rule**
(the first and sharper one is Voronoi's own collapse, above). The offline stability study
found a clean threshold -- at or below 5.9 reversals/run always arrives, at or above 14.7
always fails, nothing in between. PRM's live reversal counts here (16.53, 24.45) sit well
inside the offline "always fails" zone, yet PRM arrived 60/60 in both. That rule was derived
from one backend (Voronoi) under one kind of trigger (a timer) offline; it does not transfer
as a general law to a different backend under a different (or absent) trigger live.

**No trigger means far more replanning for everyone, but Visibility scales worst on time.**
Replan counts jump 4-8x over the event trigger (4.3) and geometric baseline for all three
backends. Visibility's nav time nearly triples versus its event-trigger numbers (11.67s vs
4.06s in scenario 1) since it keeps replanning tight paths that hug obstacle boundaries even
when nothing relevant changed; Voronoi and PRM's nav times barely move.

**Any real trigger cuts PRM's reversal rate by roughly an order of magnitude.** From
16.53/24.45 here under no-trigger, to 2.15/4.60 under the production event trigger (4.3), to
1.08/0.73 under the geometric trigger -- each step, checking more of the path (nothing, then
one waypoint, then the whole route) cuts reversals further. See `reference_all_triggers_compared.md`
for the full three-way table this is drawn from.

## Setup notes

`--trigger always` at n=60 caused repeated, complete grSim crashes partway through the batch on
the first attempt for three of six runs (PRM/scenario 2, Visibility/scenario 1, and PRM/
scenario 1 -- all-or-nothing failures, never gradual). Fixed by dropping to n=20 for a first
pass, then re-running all three cleanly at n=60 after confirming grSim connectivity with a
smoke test first.

PRM's rows were regenerated after fixing a bug in `drive_grsim.py`'s PRM wrapper: it never
varied PRM's random seed across calls (always the implicit default, 0), so every replan
rebuilt the identical 40-point roadmap instead of resampling like `dynamic_scenario.py`'s
offline wrapper does. Under this no-trigger condition that bug surfaced as a complete,
reproducible 0/60 failure in scenario 1 (a fixed roadmap disconnected from the robot's live
position, every single call) -- this is what exposed the bug in the first place. Fixed by
threading the live replan counter through as the seed, matching the offline methodology; the
rows above are the corrected, re-run results. See `reference_all_triggers_compared.md`'s setup notes
for the full diagnosis.
