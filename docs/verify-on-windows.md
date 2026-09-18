# Verifying this branch on Windows

Everything below was run on 18 September 2026 on Windows 10 with the
interpreter at `research-sdk\.venv\Scripts\python.exe` (Python 3.13.15, PySide6
6.11.2) from a clean checkout of `Emmama/headless-physics` plus PR #11. Each
command is followed by what it produced, so a different answer means something
changed.

Set the path once per shell:

```powershell
$env:PYTHONPATH = "src"
$py = "..\research-sdk\.venv\Scripts\python.exe"
```

## 1. The simulator and headless suites

```powershell
& $py -m pytest tests\test_builtin_sim.py tests\test_headless.py tests\test_headless_replanning.py tests\test_sim_control.py -q
```

Expected: `63 passed`. Took 7.8 s.

The same four files pass under WSL (Python 3.14). The full suite there is
`249 passed, 3 skipped`; the three skips are the native-grSim tests, which need
`RESEARCH_RUN_PHYSICS=1` and a built grSim.

## 2. A headless batch you can read in five seconds

Every shipped scenario, every planner, the event policy, two trials each:

```powershell
& $py -m research_sdk.headless scenarios --planner all --policy event --trials 2 --seed 7 --workers 4 --output-dir results\verify-windows-headless
```

Expected: 36 runs, about 5 s wall time, and per-scenario lines like these
(`crowded`, seed 7):

| planner | complete | collision-free | replans | planning ms per run |
|---|---|---|---|---|
| voronoi | 100% | 100% | 1.0 | 49.5 |
| prm | 100% | 100% | 9.5 | 127.0 |
| visibility | 100% | 0% | 57.0 | 397.3 |

Two trials is a smoke test, not a result; it shows the pipeline runs and the
ordering matches the committed batches. Then check the manifest:

```powershell
& $py -c "import json; m=json.load(open('results/verify-windows-headless/manifest.json')); print(m['run_count'], m['code_revision'], m['code_dirty'])"
```

Expected: `36 <40-character sha> False`. If `code_revision` is `None`, git
could not read the checkout from where you ran it; if `code_dirty` is `True`,
you have uncommitted changes and the batch is not reproducible from a commit.

Delete `results\verify-windows-headless` afterwards. `results/` is committed
on this branch, so anything left there will show up in `git status`.

## 3. The committed tables regenerate from the committed data

```powershell
& $py scripts\analyse_matrix.py results\acra-matrix-clr30
git diff --ignore-cr-at-eol --stat -- results/acra-matrix-clr30/analysis/*.tex
```

Expected: the `.tex` tables show no content change. The `.csv` and
`stats.json` files will differ because the script gained columns after those
outputs were committed; that is expected and the numbers in the tables do not
move. Run `git checkout -- results` afterwards. Needs `pandas`, `scipy` and
`matplotlib` in the interpreter.

## 4. What cannot be verified on Windows

`--backend grsim`, the native ODE physics, needs the grSim build in `.local/`
that `scripts/build_grsim.sh` produces on Ubuntu or WSL, plus Xvfb. Run it from
WSL as `docs/physics.md` describes; `scripts/physics.ps1` launches that from
PowerShell.

`analyse_canonical.py` cannot run on any `acra-*` batch on any platform until
`acra-6v6-patrol` is re-run with the current runner; see `results/README.md`.
