# Result batches on this branch

Every batch below was written by the headless runner, kinematic backend, at a
20 ms timestep and 60 Hz control. They were **not** written by the same version
of the code, and until this commit nothing in a manifest said which version
wrote which. The `runs.csv` column count is the tell: it grew from 40 to 64 as
`headless.py` gained features between 17 and 18 September 2026.

| batch | runs | scenarios | clearance mm | `runs.csv` columns | has contact attribution | generated |
|---|---|---|---|---|---|---|
| `acra-matrix` | 2367 | 263 | 0 | 40 | no | 17 Sep |
| `acra-matrix-clr30` | 2367 | 263 | **30** | 40 | no | 17 Sep |
| `acra-matrix-v2` | 3945 | 263 | 0 | 43 | no | 17 Sep |
| `acra-horizon-sweep` | 1578 | 263 | 0 | 43 | no | 17 Sep |
| `acra-sweep-patrol` | 29400 | 200 | 0 | 44 | no | 17 Sep |
| `acra-6v6-patrol` | 7800 | 200 | 0 | 61 | no | 17 Sep |
| `acra-clearance-sweep` | 12000 | 200 | swept | 62 | no | 18 Sep |
| `headless/20260918T092542606739Z` | 600 | 1 | recorded | 64 | **yes** | 18 Sep |

Clearance is the planning margin added to robot radius plus obstacle radius.
The two 2367-run matrices are the same design at 0 and 30 mm; their `runs.csv`
lacks a `prediction_horizon_ms` column, so those batches cannot be split by
horizon after the fact.

## Which analysis runs on which batch

| script | requires | runs on |
|---|---|---|
| `scripts/analyse_matrix.py` | planner x policy rows | `acra-matrix*` |
| `scripts/analyse_sweep.py` | horizon x period rows | `acra-sweep-patrol`, `acra-6v6-patrol`, `acra-clearance-sweep` |
| `scripts/analyse_canonical.py` | `obstacle_episodes_robot_initiated` and `_obstacle_initiated` columns (DEC-008) | **none of the `acra-*` batches** |

`analyse_canonical.py` exits with "runs.csv predates DEC-008 attribution" on
`acra-6v6-patrol`, the batch it was written for. The safety metric it defines,
zero robot-initiated contacts, needs a re-run of that batch with the current
runner. Only the 600-run single-scenario batch under `headless/` carries the
columns, and it is a check of the runner, not a comparison.

## Do the committed tables reproduce?

Checked 18 September by regenerating from the committed `runs.csv` files:
`analyse_matrix.py` on all three `acra-matrix*` batches and `analyse_sweep.py`
on `acra-sweep-patrol`. Every `table_*.tex` reproduces byte for byte apart from
line endings. The CSV and `stats.json` side outputs differ because the scripts
gained columns and statistics after the outputs were committed; the numbers in
the LaTeX tables did not change.

## Reading numbers across batches

Do not compare a number from a clearance-0 batch with one from
`acra-matrix-clr30` or from the draft paper's static and dynamic tables, which
were run at 30 mm on a different harness (`dynamic_scenario.py`,
`static_scenario.py`). State the batch, its clearance and its horizon beside
every quoted value.

From this commit on, `manifest.json` records `code_revision` and `code_dirty`,
so a future batch can be matched to the code that wrote it.
