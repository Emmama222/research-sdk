# Native physics verification: 14 September 2026

The headless runner was built and exercised against native grSim/ODE in WSL
Ubuntu 26.04. See [physics.md](physics.md) for setup, criteria, and limitations.

## Existing crowded scenario

One trial per planner, PRM seed 0, 120 Hz physics, Parsian robot model.
All three runs reached their targets and stopped, with no missing execution
frames. Completion alone does not imply a physics validation pass.

| Planner | Completed | Validation pass | Observed contact episodes | Simulation time |
| --- | --- | --- | --- | --- |
| Voronoi | Yes | Yes | 0 | 12.158 s |
| PRM | Yes | No | 1 | 5.242 s |
| Visibility | Yes | No | 6 | 5.625 s |

Local artifacts: [raw runs](../results/physics-crowded-validation/runs.csv),
[summary](../results/physics-crowded-validation/summary.csv), and
[per-run evidence](../results/physics-crowded-validation/evidence/).
These generated files are ignored by Git; archive them separately when sharing
research results. Each evidence folder includes inputs, trajectories, settings,
native logs, and provenance. This is a smoke experiment, not a statistically
sufficient planner comparison.

Observed simulation/wall-time ratios were 0.75-0.83, including launch and setup.
Native physics is not guaranteed to run faster than real time. The separate
kinematic backend remains available for fast preliminary screening.

## Verification

The following combined WSL suite passed all 51 tests, including three tests
running the actual engine (closed-loop arrival/stopping, motor acceleration and
body contact, rolling-ball deceleration):

```sh
RESEARCH_RUN_PHYSICS=1 .local/physics-venv/bin/python -m pytest tests/test_grsim_physics.py tests/test_headless.py tests/test_voronoi_dijkstra.py tests/test_visibility_graph.py tests/test_prm_dijkstra.py -q
```

Ruff checks passed for the added Python modules, tests, and build recorder.
The PowerShell launcher, shell build syntax, and native source patch were checked.
No owned grSim or Xvfb processes remained after execution.

The broader Windows suite was not fully green: without the two GUI modules,
160 tests passed, three native tests were skipped, and three existing runtime
tests failed. The failures reproduce when `tests/test_ui_runtime.py` runs alone;
that file and its runtime implementation were not changed. They concern
prediction-horizon policy and scene-recomputation throttling. The two GUI test
modules could not be collected because Windows Python lacks PySide6.

## Scope

These results validate behavior within the recorded grSim model, not equivalence
to a particular physical robot. The contact metric is sampled circular proximity,
not an export of ODE contact forces. Hardware-specific claims still require model
calibration and physical measurements.

## Re-verification: 18 September 2026

Run from a clean worktree of `Emmama/headless-physics` at `a33bf8a`, before any
code change in this commit.

| environment | command | result |
| --- | --- | --- |
| WSL Ubuntu, Python 3.14.4, PySide6 present | `pytest tests -q` | 249 passed, 3 skipped (the opt-in native tests) |
| Windows, Python 3.13.15, PySide6 6.11.2 | `pytest tests/test_ui_runtime.py -q` | 19 passed |

The three `test_ui_runtime.py` failures recorded above did not reproduce in
either environment, and the Windows interpreter used here does have PySide6, so
both GUI modules collect. If they still fail on the original machine, the exact
command, interpreter path and output are needed to go further.

Regenerating the committed analyses from the committed `runs.csv` files
(`analyse_matrix.py` on the three `acra-matrix*` batches, `analyse_sweep.py` on
`acra-sweep-patrol`) reproduces every LaTeX table byte for byte apart from line
endings. The CSV and `stats.json` side outputs differ because the analysis
scripts gained columns and statistics after those outputs were committed; the
numbers in the tables are unchanged. `analyse_canonical.py` cannot run on any
`acra-*` batch: they predate the robot-initiated contact columns it requires.
See `results/README.md`.
