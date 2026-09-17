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
