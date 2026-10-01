# Results

| Folder | Status | Contents |
|---|---|---|
| `acra2026-final/` | **Current** — the only source for final ACRA 2026 claims | The three result sets below, generated from the frozen scenario bank `scenarios/acra2026-200` with the revised measurement set (DEC-016/017/018) |
| `_archive-dec024-with-time-limit/` | Supporting, 1 Oct 2026 — not the main results | All four sets of the final configuration with the 100 ms replan limit enforced (DEC-019). Superseded as main results by DEC-024 (limit measured, not enforced); kept to show the limit's effect. See its `README.md` |
| `_archive-dec023-stalls/` | Repeat run, 1 Oct 2026 — do not use for final claims | First run of the final configuration (DEC-022 + DEC-023). Same revision as `acra2026-final/`; used only as the repeat batch for the determinism check. Raw files were not kept; see its `README.md` |
| `_archive-pre-dec023/` | Superseded 1 Oct 2026 — do not use for final claims | The full set after DEC-022 but before DEC-023 (Voronoi's per-tick checks without the 30 mm margin). Reference for the margin ablation; see its `README.md` |
| `_archive-pre-shared-escape/` | Superseded 1 Oct 2026 — do not use for final claims | The full ACRA 2026 set before DEC-022 (only Voronoi had an escape step). Voronoi numbers remain a reference; see its `README.md` |
| `_archive-pre-dec016/` | Historical — do not use for final claims | Every batch produced before the revised result model (17–20 Sep 2026). See its own `README.md` for what each batch contains |
| `plot_empty_voronoi.py`, `voronoi_empty_virtual_sites.svg` | Current | Figure of the bounded Voronoi diagram on an empty field |

Ad-hoc headless runs without `--output-dir` are written to `results/headless/<timestamp>/`.

## `acra2026-final/`

| Result set | Folder | Arms | Runs |
|---|---|---|---:|
| One-Shot Planning Validation | `one-shot-validation/` | Frozen obstacles, one route per robot, no execution | 600 |
| Dynamic Replanning Policy Comparison | `policy-comparison/` | Cycle and event at 20 ms and 100 ms checks, horizon 0 ms | 2,400 |
| Prediction Horizon Sweep | `horizon-sweep/` | Event at 20 ms checks, horizon 20 / 50 / 100 ms (0 ms point reused from the policy comparison) | 1,800 |

Figures and tables live in `acra2026-final/figures/` and `acra2026-final/tables/`;
[`acra2026-final/figures/README.md`](acra2026-final/figures/README.md) indexes
what each figure shows and how to read it.

Each folder is written once and never overwritten. A re-run goes into a new,
dated folder and the superseded one moves to the archive.

## Analysis scripts that read archived batches

`scripts/analyse_matrix.py`, `analyse_sweep.py`, `paper_numbers.py`,
`plot_headline.py`, `plot_heatmap_v2.py`, `plot_plan_efficiency.py`,
`plot_prediction_horizon.py`, `plot_tradeoff.py` and `report_batch.py` default
to paths under the old `results/acra-*` names. Pass the batch path explicitly
(now under `results/_archive-pre-dec016/`) to rerun them on historical data.

## Commands (Windows, from the repo root, single worker for clean latency)

```powershell
python -m research_sdk.headless --result-set one-shot-validation --output-dir results/acra2026-final/one-shot-validation
python -m research_sdk.headless --result-set policy-comparison --workers 1 --slow-call-limit-ms 0 --output-dir results/acra2026-final/policy-comparison
python -m research_sdk.headless --result-set horizon-sweep --workers 1 --slow-call-limit-ms 0 --output-dir results/acra2026-final/horizon-sweep
python scripts/build_result_tables.py
python scripts/plot_outcomes.py
```

Tables are written to `results/acra2026-final/tables/` as CSV and Markdown.
Failure rules (DEC-019): a post-initial replan over 100 ms stops that robot;
any robot without a route for 1000 ms fails the episode.

## Supporting result sets

Run on the same machine and settings as the main sets (single worker), then re-run the scripts:

```powershell
python -m research_sdk.headless --result-set event-route-ablation --workers 1 --slow-call-limit-ms 0 --output-dir results/acra2026-final/supporting/event-route-ablation
python scripts/build_result_tables.py
python scripts/plot_outcomes.py
```

| Set | Arms | Runs | Status | Purpose |
|---|---|---:|---|---|
| `event-route-ablation` | Full-path recalculation (`event_route`) @ 20 and 100 ms | 1,200 | **Done** (`supporting/event-route-ablation/`) | DEC-010 ablation against the policy comparison's event arms |
| `horizon-150-check` | event @ 20 ms, horizon 150 ms | 600 | Optional, not run | Full-bank repeat of the 20-scenario preview behind DEC-020 |
| `no-route-500-check` | event @ 20 ms, no-route limit 500 ms | 600 | Optional, not run | Full-bank repeat of the 20-scenario preview behind the DEC-019 amendment |

The two optional sets can be run the same way (`--result-set horizon-150-check`
or `--result-set no-route-500-check`); the plotting script draws their figures
only when their results exist. Decision ids are summarised in
[`docs/decisions/0006-acra2026-evaluation-rules.md`](../docs/decisions/0006-acra2026-evaluation-rules.md).
