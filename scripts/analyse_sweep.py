"""Analyse a prediction-horizon x replan-period sweep (runs.csv).

Usage:
    python scripts/analyse_sweep.py results/acra-sweep-patrol

Writes <batch>/analysis/:
  grid_<policy>.csv          one row per planner x horizon x period
  best_configs.csv           best setting per planner (safety first, then compute)
  event_vs_cycle.csv         paired event/cycle comparison at identical settings
  heatmap_<policy>.pdf/.png  collision-free % and planning time per planner
  stats.json                 exclusions and paired tests
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, LogNorm, Normalize  # noqa: E402
from scipy.stats import binomtest, wilcoxon  # noqa: E402

PLANNERS = ["voronoi", "prm", "visibility"]
PLANNER_LABEL = {"voronoi": "Voronoi", "prm": "PRM", "visibility": "Visibility graph"}
BUDGET_MS = 1000.0 / 60.0
# Validated single-hue sequential ramp (blue 100 -> 700).
BLUES = LinearSegmentedColormap.from_list(
    "seq_blue",
    ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"],
)
# Second sequential context takes the next categorical hue (orange), one hue light -> dark.
ORANGES = LinearSegmentedColormap.from_list(
    "seq_orange", ["#fde4d8", "#f7b397", "#f18a60", "#eb6834", "#c24f1f", "#943a14", "#6b290c"]
)
INK, INK_2 = "#0b0b0b", "#52514e"
POLICY_TITLE = {"event": "Event-triggered replanning", "cycle": "Time-triggered replanning"}


def load(folder: Path) -> tuple[pd.DataFrame, list]:
    runs = pd.read_csv(folder / "runs.csv")
    runs["collision_free"] = runs["collision_episodes"] == 0
    runs["replan_period_ms"] = runs["replan_period_ms"].round(3)
    failed = runs.loc[runs["status"] == "planning_failed", ["scenario", "planner"]]
    failed_keys = set(map(tuple, failed.drop_duplicates().values))
    keep = [(s, p) not in failed_keys for s, p in zip(runs["scenario"], runs["planner"])]
    return runs[keep].copy(), sorted(failed_keys)


def grid(runs: pd.DataFrame, policy: str) -> pd.DataFrame:
    sub = runs[runs.replan_policy == policy]
    g = sub.groupby(["planner", "prediction_horizon_ms", "replan_period_ms"])
    out = pd.DataFrame(
        {
            "n": g.size(),
            "collision_free_%": 100 * g.collision_free.mean(),
            "completed_%": 100 * g.completed.mean(),
            "near_miss_free_%": 100 * g.minimum_clearance_mm.apply(lambda c: (c >= 50).mean()),
            "replans_mean": g.replan_count.mean(),
            "planning_ms_median": g.planning_time_ms_total.median(),
            "planning_ms_per_sim_s_median": g.apply(
                lambda d: (d.planning_time_ms_total / (d.simulated_duration_ms / 1000)).median(),
                include_groups=False,
            ),
            "p95_call_ms_median": g.planning_time_ms_p95_call.median(),
            "runs_with_call_over_budget_%": 100
            * g.planning_time_ms_max_call.apply(lambda c: (c > BUDGET_MS).mean()),
            "time_to_goal_s_median": g.time_to_goal_ms.median() / 1000,
        }
    ).reset_index()
    return out


def best_configs(table: pd.DataFrame, policy: str) -> pd.DataFrame:
    rows = []
    for planner in PLANNERS:
        sub = table[table.planner == planner]
        if sub.empty:
            continue
        top = sub["collision_free_%"].max()
        # Within 1 percentage point of the safest setting, prefer the cheapest.
        near = sub[sub["collision_free_%"] >= top - 1.0]
        pick = near.sort_values(["planning_ms_median", "time_to_goal_s_median"]).iloc[0]
        rows.append({"policy": policy, **pick.to_dict(), "best_collision_free_%": top})
    return pd.DataFrame(rows)


def paired_event_vs_cycle(runs: pd.DataFrame) -> tuple[pd.DataFrame, dict]:
    rows, stats = [], {}
    keys = ["planner", "prediction_horizon_ms", "replan_period_ms"]
    both = runs[runs.replan_policy.isin(["event", "cycle"])]
    for (planner, horizon, period), sub in both.groupby(keys):
        wide = sub.pivot_table(
            index="scenario", columns="replan_policy",
            values=["planning_time_ms_total", "collision_free"], aggfunc="first",
        ).dropna()
        if wide.empty or "event" not in wide["collision_free"]:
            continue
        pt = wide["planning_time_ms_total"]
        cf = wide["collision_free"].astype(bool)
        only_e = int((cf["event"] & ~cf["cycle"]).sum())
        only_c = int((~cf["event"] & cf["cycle"]).sum())
        n = only_e + only_c
        rows.append(
            {
                "planner": planner,
                "prediction_horizon_ms": horizon,
                "replan_period_ms": period,
                "scenarios": len(wide),
                "cycle_over_event_planning_time_median": float((pt["cycle"] / pt["event"]).median()),
                "collision_free_event_%": 100 * cf["event"].mean(),
                "collision_free_cycle_%": 100 * cf["cycle"].mean(),
                "only_event_safe": only_e,
                "only_cycle_safe": only_c,
                "mcnemar_exact_p": float(binomtest(only_e, n, 0.5).pvalue) if n else 1.0,
                "planning_time_wilcoxon_p": float(wilcoxon(pt["event"], pt["cycle"]).pvalue)
                if (pt["event"] != pt["cycle"]).any()
                else 1.0,
            }
        )
    table = pd.DataFrame(rows)
    if not table.empty:
        stats["event_vs_cycle_min_mcnemar_p"] = float(table["mcnemar_exact_p"].min())
        stats["cycle_over_event_ratio_range"] = [
            float(table["cycle_over_event_planning_time_median"].min()),
            float(table["cycle_over_event_planning_time_median"].max()),
        ]
    return table, stats


def _heat(ax, matrix: pd.DataFrame, cmap, norm, fmt: str) -> None:
    image = ax.imshow(matrix.values, cmap=cmap, norm=norm, aspect="auto", origin="lower")
    ax.set_xticks(range(matrix.shape[1]), [f"{c:g}" for c in matrix.columns])
    ax.set_yticks(range(matrix.shape[0]), [f"{r:g}" for r in matrix.index])
    ax.tick_params(colors=INK_2, labelsize=7, length=0)
    for spine in ax.spines.values():
        spine.set_visible(False)
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            value = matrix.values[i, j]
            if np.isnan(value):
                continue
            r, g, b, _ = image.cmap(image.norm(value))
            light = 0.299 * r + 0.587 * g + 0.114 * b > 0.55
            ax.text(j, i, fmt.format(value), ha="center", va="center", fontsize=6.5,
                    color=INK if light else "white")
    return image


def heatmaps(table: pd.DataFrame, policy: str, path: Path) -> None:
    planners = [p for p in PLANNERS if p in set(table.planner)]
    fig, axes = plt.subplots(2, len(planners), figsize=(3.3 * len(planners), 5.6),
                             constrained_layout=True, squeeze=False)
    safe_norm = Normalize(vmin=table["collision_free_%"].min(), vmax=100)
    positive = table["planning_ms_median"][table["planning_ms_median"] > 0]
    time_norm = LogNorm(vmin=max(positive.min(), 0.1), vmax=positive.max())
    for col, planner in enumerate(planners):
        sub = table[table.planner == planner]
        safe = sub.pivot(index="prediction_horizon_ms", columns="replan_period_ms",
                         values="collision_free_%")
        cost = sub.pivot(index="prediction_horizon_ms", columns="replan_period_ms",
                         values="planning_ms_median")
        im1 = _heat(axes[0, col], safe, BLUES, safe_norm, "{:.0f}")
        im2 = _heat(axes[1, col], cost, ORANGES, time_norm, "{:.0f}")
        axes[0, col].set_title(PLANNER_LABEL[planner], fontsize=9, color=INK)
        axes[1, col].set_xlabel("Replan period (ms)", fontsize=8, color=INK_2)
        if col == 0:
            axes[0, col].set_ylabel("Prediction horizon (ms)", fontsize=8, color=INK_2)
            axes[1, col].set_ylabel("Prediction horizon (ms)", fontsize=8, color=INK_2)
    fig.colorbar(im1, ax=axes[0, :], shrink=0.85, label="Collision-free runs (%)")
    bar = fig.colorbar(im2, ax=axes[1, :], shrink=0.85, label="Planning time per run (ms, median)")
    ticks = [t for t in (1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000)
             if time_norm.vmin <= t <= time_norm.vmax]
    bar.set_ticks(ticks, labels=[f"{t:g}" for t in ticks])
    bar.minorticks_off()
    fig.suptitle(POLICY_TITLE.get(policy, policy), fontsize=9, color=INK_2, x=0.02, ha="left")
    for suffix in (".pdf", ".png"):
        fig.savefig(path.with_suffix(suffix), dpi=200)
    plt.close(fig)


def main(folder: Path) -> None:
    out = folder / "analysis"
    out.mkdir(exist_ok=True)
    runs, excluded = load(folder)
    stats: dict = {"excluded_initial_plan_failures": [list(k) for k in excluded]}
    bests = []
    for policy in ("event", "cycle"):
        if policy not in set(runs.replan_policy):
            continue
        table = grid(runs, policy)
        table.to_csv(out / f"grid_{policy}.csv", index=False)
        heatmaps(table, policy, out / f"heatmap_{policy}")
        bests.append(best_configs(table, policy))
    once = runs[runs.replan_policy == "once"]
    if not once.empty:
        stats["once_collision_free_%"] = {
            p: float(100 * once[once.planner == p].collision_free.mean()) for p in PLANNERS
        }
    if bests:
        pd.concat(bests).to_csv(out / "best_configs.csv", index=False)
    paired, paired_stats = paired_event_vs_cycle(runs)
    paired.to_csv(out / "event_vs_cycle.csv", index=False)
    stats.update(paired_stats)
    (out / "stats.json").write_text(json.dumps(stats, indent=2), encoding="utf-8")
    print(pd.concat(bests).round(2).to_string(index=False) if bests else "no grid")


if __name__ == "__main__":
    main(Path(sys.argv[1] if len(sys.argv) > 1 else "results/acra-sweep-patrol"))
