# Planner scenarios

The research console stores named scenario files in this folder as JSON.

Coordinates and obstacle radii are stored in millimetres. Robot orientations
are radians. Obstacle velocity is included in the schema for future dynamic
obstacle experiments, but the initial UI treats obstacles as static.

Each obstacle may include a `planner_keys` list containing planner class import
paths. Such an obstacle is visible only to those algorithms. If the list is
missing or empty, the obstacle is shared by every algorithm for compatibility
with existing scenario files. In the UI, choose an active planner before using
the `add_obstacle` course tool.

Scenario files are reproducible experiment inputs and should be committed when
they form part of a reported result.

## Running the scenarios

Run every loose scenario file in this folder (not the bank) against every
planner:

```powershell
python -m research_sdk.headless --planner all
```

## ACRA 2026 evaluation bank

The frozen ACRA 2026 evaluation bank is stored in `acra2026-200/`. It contains
six robots with six start-to-goal pairs and six moving patrol obstacles in every
scenario.

To produce the paper results, run the three result sets; each uses exactly the
200 bank scenarios:

```powershell
python -m research_sdk.headless --result-set one-shot-validation --output-dir results/acra2026-final/one-shot-validation
python -m research_sdk.headless --result-set policy-comparison --workers 1 --output-dir results/acra2026-final/policy-comparison
python -m research_sdk.headless --result-set horizon-sweep --workers 1 --output-dir results/acra2026-final/horizon-sweep
python scripts/build_result_tables.py
```

For a quick look at the bank with other settings, pass it explicitly. Without
`--result-set` this runs the plan-once policy only and writes to
`results/headless/<timestamp>/`:

```powershell
python -m research_sdk.headless --scenario-bank scenarios/acra2026-200 --planner all
```

`--random`, `--perturb` and `--no-saved` are rejected with a bank, so nothing
else can be mixed in.

The bank was generated without running a simulation by:

```powershell
python scripts/generate_acra_scenarios.py --force
```

**Do not regenerate it once results exist.** `--force` overwrites the frozen
inputs, and every result set must use identical scenarios. Each result
manifest records the bank's SHA-256 fingerprint so a changed bank is
detectable.

Render the whole bank without running an experiment using:

```powershell
python scripts/report_batch.py --scenario-only `
  --scenario-dir scenarios/acra2026-200 `
  --scenario-output scenario_png
```
