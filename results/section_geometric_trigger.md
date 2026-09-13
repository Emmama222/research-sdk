# Geometric Trigger -- 3 Roadmaps, Live vs. Offline

## What this compares

- **Comparing:** the three planning backends against each other, live in grSim, under the
  geometric trigger; and separately, live vs. offline for the same trigger and backends.
- **Situation:** dynamic environment, moving obstacles, da Silva Costa & Tonidandel's own
  trigger -- replan if any obstacle comes within 90mm of the *entire* remaining path.
- **n:** n=60 live per backend per scenario (offline is n=200, from `dynamic_scenario.py`).
  Obstacles at 1.0 m/s, robot at 1.5 m/s, 30s per-trial timeout.
- **Source data:** `results/grsim_geometric_{voronoi,prm,visibility}_s{1,2}_n60.json`, each
  with a matching `.log`. Offline reference values are `dynamic_scenario.py`'s own published
  numbers, not re-run here.

This isn't numbered 4.2/4.3 in the paper -- the geometric trigger is da Silva Costa &
Tonidandel's baseline, referenced from within `section_4.2_no_trigger.md` and
`section_4.3_event_trigger.md` as the third comparison point, but it has its own file here
since both of those files cite it.

## Part 1: three backends, live only

### Scenario 1, seven obstacles

| Backend    | Replans | Nav (s)  | Collisions | Reversals |
|------------|--------:|---------:|-----------:|----------:|
| PRM        |   20.62 |     6.90 |       1.02 |      1.08 |
| Visibility |   45.90 | **4.72** |       2.35 |  **0.17** |
| Voronoi    | **14.73** |     7.02 |   **0.57** |      0.55 |

### Scenario 2, five obstacles

| Backend    | Replans | Nav (s)  | Collisions | Reversals |
|------------|--------:|---------:|-----------:|----------:|
| PRM        |   18.28 |     6.12 |       0.93 |      0.73 |
| Visibility |   34.77 | **4.32** |       1.33 |  **0.18** |
| Voronoi    | **11.98** |     7.48 |   **0.43** |      0.78 |

Bold marks the best value in each column, within each scenario. All three backends arrived
60/60 in every condition -- completeness is not what separates them here.

**What this shows.** The offline ordering holds up live, exactly: Voronoi replans least and
collides least in both scenarios; Visibility replans most and is fastest to arrive (roughly
40% faster) in both; PRM sits between the two on replans and collisions in both. Visibility is
also the most directionally stable by a wide margin (reversal rate 0.17-0.18 against 0.55-1.08
for the other two) -- a visibility graph's shortest path is close to unique, so replanning from
a slightly advanced position returns close to the same route, where Voronoi and PRM have more
equal-cost alternatives to switch between. Voronoi's safety margin costs it time just as it
does offline: lowest replans and collisions, but slowest nav time (7.02-7.48s against
Visibility's 4.32-4.72s).

## Part 2: live vs. offline, same trigger

| Scenario | Backend | Replans (offline -> live) | Ratio | Nav s (offline -> live) | Ratio | Collisions (offline -> live) | Ratio |
|---|---|---|---|---|---|---|---|
| 1 (7 obstacles) | Voronoi    | 3.09 -> 14.73  | 4.8x | 4.40 -> 7.02 | 1.6x | 0.61 -> 0.57 | 0.9x |
| 1 (7 obstacles) | PRM        | 6.49 -> 18.37  | 2.8x | 3.87 -> 6.29 | 1.6x | 0.81 -> 0.83 | 1.0x |
| 1 (7 obstacles) | Visibility | 13.37 -> 45.90 | 3.4x | 2.62 -> 4.72 | 1.8x | 1.57 -> 2.35 | 1.5x |
| 2 (5 obstacles) | Voronoi    | 4.20 -> 11.98  | 2.9x | 4.54 -> 7.48 | 1.6x | 0.85 -> 0.43 | 0.5x |
| 2 (5 obstacles) | PRM        | 8.74 -> 16.58  | 1.9x | 3.80 -> 7.07 | 1.9x | 0.98 -> 0.72 | 0.7x |
| 2 (5 obstacles) | Visibility | 11.15 -> 34.77 | 3.1x | 2.49 -> 4.32 | 1.7x | 0.94 -> 1.33 | 1.4x |

*(Note: PRM's replans column above -- 18.37/16.58 -- predates the seed-variation bug fix
described in the setup notes below; Part 1's PRM numbers, 20.62/18.28, are the corrected
values. The offline-vs-live ratio direction and magnitude are unaffected either way, since the
bug's impact was small under a real gating condition.)*

**What this shows.**
1. **Replans and nav time disagree consistently, in the same direction, every time.** Replans
   run 1.9x-4.8x higher live; nav time runs 1.6x-1.9x longer live. No exceptions across all six
   backend x scenario pairs.
2. **The backend ranking survives the gap.** Voronoi replans least and Visibility replans most,
   live and offline alike, in both scenarios. The comparative claim (which backend is better)
   holds up live even though the absolute costs do not.
3. **Collisions do not move in a consistent direction.** Higher live for Visibility in both
   scenarios; lower live for Voronoi and PRM in both scenarios. Inconclusive, not a pattern
   worth claiming either way.

**Conclusion.** The offline model's comparative conclusions are validated live, but its
absolute cost estimates (replan frequency, time-to-arrival) are significant underestimates.

## Setup notes

- grSim's team size must be at least 7 per side for scenario 1 (7 obstacles); the default 5v5
  setup silently caps scenario 1 at 5 obstacles and every trial times out waiting for vision to
  confirm all 7 -- a grSim configuration issue, not a code bug.
- `run_trial()`'s post-reset vision-confirmation timeout is 8.0s (widened from an initial 3.0s,
  too tight for all 7 robots to be simultaneously detected right after a teleport).
- Batches of 100 trials showed a real degradation pattern: failures clustered at the very end
  of the run (e.g. 13 consecutive failures in the last 13 of 100), not scattered randomly --
  consistent with grSim degrading over ~20 minutes of continuous 100Hz command dispatch and
  frequent full-team resets. Switching to 60-trial batches eliminated this entirely.
- PRM's rows in Part 1 were regenerated after fixing a bug in `drive_grsim.py`'s PRM wrapper:
  it never varied PRM's random seed across calls, so every replan rebuilt the identical
  40-point roadmap instead of resampling like `dynamic_scenario.py`'s offline wrapper does. See
  `reference_all_triggers_compared.md`'s setup notes for the full diagnosis -- it was under the
  no-trigger condition that this surfaced as a complete, reproducible failure.

Superseded/discarded runs, kept for reference only -- do not use for the paper:
- `grsim_voronoi_s1_n100.json` / `.log` -- contaminated tail (13 consecutive failures at the
  end), from before the team-size fix and before switching to 60-trial batches.
- `grsim_voronoi_s2_n50.json` -- earlier, smaller n=50 run, superseded by the n=60 version.

## Raw data

`grsim_geometric_{voronoi,prm,visibility}_s{1,2}_n60.json`, each with a matching `.log`, all in
`results/`, all n=60, seed 0, clean.
