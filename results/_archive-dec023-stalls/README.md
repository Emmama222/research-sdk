# Repeat run of the final configuration (1 October 2026)

First run of the policy comparison, horizon sweep and full-path supporting set
at revision `4b04d42f` (DEC-022 shared escape step and DEC-023 30 mm Voronoi
check margin). It passed all 72 checks in `scripts/verify_results.py`. The sets
were then run a second time at the same revision; that second run is the final
set in `acra2026-final/`.

The raw result files of this first run were not kept. Its role is the repeat
batch for the determinism check, whose numbers are recorded here:

| Set | Runs stopped by the 100 ms limit (first / final run) | Stopped in both | Runs with no stop in either run that are identical |
|---|---|---|---|
| Policy comparison | 123 / 102 | 22 | 2,197 of 2,197 |
| Horizon sweep | 70 / 97 | 2 | 1,635 of 1,635 |
| Full-path supporting set | 73 / 80 | 12 | 1,059 of 1,059 |

"Identical" means the same outcome class and the same number of rebuilds.
Collision-free finishes per arm differed between the two runs by 0–5 of 200 in
all but one arm (Voronoi cycle @ 20 ms: 136 vs 145, from 22 vs 4 stops).
