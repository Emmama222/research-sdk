# Results

| Folder | Status | Contents |
|---|---|---|
| `acra2026-final/` | **Current** — the only source for final ACRA 2026 claims | The three result sets below, generated from the frozen scenario bank `scenarios/acra2026-200` with the revised measurement set (DEC-016/017/018) |
| `_archive-pre-dec016/` | Historical — do not use for final claims | Every batch produced before the revised result model (17–20 Sep 2026). See its own `README.md` for what each batch contains |
| `plot_empty_voronoi.py`, `voronoi_empty_virtual_sites.svg` | Current | Figure of the bounded Voronoi diagram on an empty field |

Ad-hoc headless runs without `--output-dir` are written to `results/headless/<timestamp>/`.

## `acra2026-final/`

| Result set | Folder | Arms | Runs |
|---|---|---|---:|
| One-Shot Planning Validation | `one-shot-validation/` | Frozen obstacles, one route per robot, no execution | 600 |
| Dynamic Replanning Policy Comparison | `policy-comparison/` | Cycle and event at 20 ms and 100 ms checks, horizon 0 ms | 2,400 |
| Prediction Horizon Sweep | `horizon-sweep/` | Event at 20 ms checks, horizon 20 / 50 / 150 ms (0 ms point reused from the policy comparison) | 1,800 |

Each folder is written once and never overwritten. A re-run goes into a new,
dated folder and the superseded one moves to the archive.

## Analysis scripts that read archived batches

`scripts/analyse_matrix.py`, `analyse_sweep.py`, `paper_numbers.py`,
`plot_headline.py`, `plot_heatmap_v2.py`, `plot_plan_efficiency.py`,
`plot_prediction_horizon.py`, `plot_tradeoff.py` and `report_batch.py` default
to paths under the old `results/acra-*` names. Pass the batch path explicitly
(now under `results/_archive-pre-dec016/`) to rerun them on historical data.
