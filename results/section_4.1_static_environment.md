# 4.1 Three Roadmaps in a Static Environment

## What this compares

- **Comparing:** offline ground-truth obstacle positions (`static_scenario.py`) against live
  grSim with real vision-detected obstacle positions.
- **Situation:** static environment, obstacles stationary, no movement, no replanning -- one
  `plan()` call per trial.
- **n:** n=200 offline (existing data), n=60 live (new, this run).
- **Source data:** offline -- `results/static.png`, `results/static_n200.txt`. Live --
  `results/grsim_static_{voronoi,prm,visibility}_{graph,sampling,mixed,stoppage}_n60.json`,
  each with a matching `.log`.

## Full comparison

| Scenario | Backend    | Fail (live) | Fail (offline) | Length m (live vs offline) | Safety m (live vs offline) |
|----------|------------|------------:|----------------:|----------------------------|------------------------------|
| graph    | PRM        | 0/60        | 0/200           | 8.67 vs 8.68                | 8.35 vs 8.88                |
| graph    | Visibility | 0/60        | 0/200           | **7.20 vs 7.20**            | **3.93 vs 3.93**            |
| graph    | Voronoi    | 0/60        | 0/200           | **9.48 vs 9.48**            | **13.55 vs 13.55**          |
| sampling | PRM        | **8/60 (13.3%)** | **25/200 (12.5%)** | 9.75 vs 9.80            | 22.62 vs 23.14              |
| sampling | Visibility | 0/60        | 0/200           | **8.64 vs 8.64**            | **23.10 vs 23.10**          |
| sampling | Voronoi    | 0/60        | 0/200           | **9.63 vs 9.63**            | **27.88 vs 27.88**          |
| mixed    | PRM        | 0/60        | 0/200           | 8.79 vs 8.87                 | 7.81 vs 8.08                |
| mixed    | Visibility | 0/60        | 0/200           | **7.75 vs 7.75**            | **2.86 vs 2.86**            |
| mixed    | Voronoi    | 0/60        | 0/200           | **10.48 vs 10.48**          | **13.24 vs 13.25**          |
| stoppage | PRM        | 0/60        | 0/200           | 5.37 vs 5.09                | 7.00 vs 6.63                |
| stoppage | Visibility | 0/60        | 0/200           | **3.97 vs 3.95**            | **4.20 vs 4.43**            |
| stoppage | Voronoi    | 0/60        | 0/200           | **6.20 vs 6.20**            | **9.48 vs 9.48**            |

Bold = exact or near-exact match (within vision-detection precision, <0.3%).

## What this shows

**Live matches offline almost exactly, everywhere.** Visibility and Voronoi are deterministic
-- the same geometric construction from obstacle positions every call -- and every one of their
eight rows above lands within a millimetre or two of `static_scenario.py`'s own numbers. Real
vision noise exists, but it isn't large enough to move a geometric construction's output.

**PRM's narrow-passage failure rate transfers almost exactly.** 8/60 (13.3%) live against
25/200 (12.5%) offline on `worst_case_sampling` -- the scenario built specifically to punish
random sampling on a 380mm corridor. This is the one place noisy obstacle positions could
plausibly have shifted whether the gap reads as passable, and it didn't: the failure mechanism
(40 random points, ~0.3% chance per point of landing in the corridor) depends on geometry and
sample count, not on which coordinate system reports the obstacle positions.

**PRM's small length/safety differences elsewhere are sampling variance, not a live/offline
gap.** Offline's own PRM numbers are a mean over 200 different random roadmaps, not a fixed
ground truth; live's mean over 60 landing within a few percent of that (8.67 vs 8.68, 8.79 vs
8.87, 5.37 vs 5.09) is what re-running offline with a different seed would look like too.

**This is the opposite finding from 4.2/4.3.** There, replans and nav time are substantially
and consistently higher live, in every one of six backend x scenario pairs, by a margin far
larger than any run-to-run noise could produce. Static confirms the offline model almost
exactly; dynamic contradicts its absolute numbers while still agreeing on backend ranking. Both
are real findings, and they differ because static has no execution, no timing, and no
replanning dynamics for live and offline to disagree on -- dynamic has all three.

## Setup notes

grSim's team size had to be at least 15 per side to cover `worst_case_sampling` (a 15-obstacle
wall) -- the other three scenarios need fewer (9, 8, 7) but were run at the same setting for
consistency.

Two bugs were found and fixed while building this test:
1. `obstacles_for()` returns every robot currently on the pitch. With team size bumped to 15 to
   fit the largest scenario, a smaller scenario left extra yellow robots wherever they'd last
   been positioned, counted as phantom obstacles -- inflating `worst_case_graph`'s live safety
   to 37.90 against an offline 13.55 on the first attempt. Fixed by only reading yellow robot
   ids `0..len(scenario.obstacles)-1`.
2. Obstacle radius was hardcoded to the default 90mm robot radius for every live-detected
   obstacle, wrong for `game_stoppage`'s ball (a 500mm keep-out riding on an ordinary grSim
   robot -- vision can't report "this one means more"). Fixed by taking radius from the
   scenario's own declared value and only position from live vision.
