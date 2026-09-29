"""Paired heatmap, v2: both panels are '% of runs' and both are higher-is-better.

Replaces the v1 figure, whose top panel was collision-free % (higher better) and
bottom panel median planning time in ms (lower better, and near-deterministic
given the replan period). Here the second panel is the share of runs that kept
every planner call inside the 16.7 ms frame budget, so:

  * both panels carry the same unit (% of runs)
  * both panels point the same way (darker = better)
  * one sequential ramp and one colourbar serve the whole figure

Usage:
    python scripts/plot_heatmap_v2.py results/acra-6v6-patrol [event|cycle]
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, Normalize  # noqa: E402

PLANNERS = ["voronoi", "prm", "visibility"]
PLANNER_LABEL = {"voronoi": "Voronoi", "prm": "PRM", "visibility": "Visibility graph"}
BUDGET_MS = 1000.0 / 60.0
SETTINGS = ["prediction_horizon_ms", "replan_period_ms", "planning_clearance_mm"]
AXIS_LABEL = {
    "prediction_horizon_ms": "Prediction horizon (ms)",
    "replan_period_ms": "Replan period (ms)",
    "planning_clearance_mm": "Planning clearance (mm)",
}
BLUES = LinearSegmentedColormap.from_list(
    "seq_blue",
    ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"],
)
INK, INK_INV = "#1f2733", "#ffffff"


def load(folder: Path) -> pd.DataFrame:
    runs = pd.read_csv(folder / "runs.csv")
    runs["collision_free"] = runs["collision_episodes"] == 0
    if "obstacle_episodes_robot_initiated" in runs:
        # DEC-008: only robot-initiated contacts count against the planner.
        runs["collision_free"] = (runs["obstacle_episodes_robot_initiated"] == 0) & (
            runs["robot_collision_episodes"] == 0
        )
        print("[v2] using DEC-008 robot-initiated attribution")
    else:
        print("[v2] WARNING: no attribution columns; uncorrected metric (layout prototype only)")
    runs["replan_period_ms"] = runs["replan_period_ms"].round(3)
    if "planning_clearance_mm" not in runs:
        runs["planning_clearance_mm"] = 0.0
    runs["in_budget"] = runs["planning_time_ms_max_call"] <= BUDGET_MS
    failed = runs.loc[runs["status"] == "planning_failed", ["scenario", "planner"]]
    bad = set(map(tuple, failed.drop_duplicates().values))
    keep = [(s, p) not in bad for s, p in zip(runs["scenario"], runs["planner"])]
    return runs[keep].copy()


def axes_of(sub: pd.DataFrame) -> tuple[str, str]:
    varying = [c for c in SETTINGS if sub[c].nunique() > 1]
    if len(varying) >= 2:
        return varying[0], varying[1]
    if len(varying) == 1:
        other = next(c for c in SETTINGS if c != varying[0])
        return other, varying[0]
    return SETTINGS[0], SETTINGS[1]


def cells(ax, matrix: pd.DataFrame, norm, cmap) -> None:
    img = ax.imshow(matrix.values, cmap=cmap, norm=norm, aspect="auto", origin="lower")
    ax.set_xticks(range(matrix.shape[1]), [f"{c:g}" for c in matrix.columns])
    ax.set_yticks(range(matrix.shape[0]), [f"{r:g}" for r in matrix.index])
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.tick_params(length=0, colors="#5b6472", labelsize=9)
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            v = matrix.values[i, j]
            if np.isnan(v):
                continue
            r, g, b, _ = cmap(norm(v))
            lum = 0.299 * r + 0.587 * g + 0.114 * b
            ax.text(j, i, f"{v:.0f}", ha="center", va="center", fontsize=11,
                    color=INK_INV if lum < 0.55 else INK)


def main(folder: Path, policy: str) -> None:
    runs = load(folder)
    sub = runs[runs.replan_policy == policy]
    if sub.empty:
        raise SystemExit(f"no runs with policy {policy!r}")
    row_key, col_key = axes_of(sub)
    g = sub.groupby(["planner", row_key, col_key])
    table = pd.DataFrame(
        {"safety": 100 * g.collision_free.mean(), "budget": 100 * g.in_budget.mean()}
    ).reset_index()

    planners = [p for p in PLANNERS if p in table.planner.values]
    norm = Normalize(vmin=0, vmax=100)
    fig, axes = plt.subplots(2, len(planners), figsize=(3.2 * len(planners), 6.0),
                             squeeze=False)
    fig.subplots_adjust(hspace=0.55, wspace=0.18)

    for col, planner in enumerate(planners):
        p = table[table.planner == planner]
        for row, metric in enumerate(("safety", "budget")):
            m = p.pivot(index=row_key, columns=col_key, values=metric)
            cells(axes[row][col], m, norm, BLUES)
            if row == 0:
                axes[row][col].set_title(PLANNER_LABEL[planner], fontsize=11, color=INK, pad=8)
            if row == 1:
                axes[row][col].set_xlabel(AXIS_LABEL[col_key], fontsize=10, color="#5b6472")
            else:
                axes[row][col].set_xticklabels([])
            if col == 0:
                axes[row][col].set_ylabel(AXIS_LABEL[row_key], fontsize=10, color="#5b6472")
            else:
                axes[row][col].set_yticklabels([])

    for row, text in enumerate(("Collision-free runs", "Runs inside the 16.7 ms frame budget")):
        axes[row][0].text(-0.30, 1.30, text, transform=axes[row][0].transAxes,
                          fontsize=12, color=INK, ha="left", va="bottom", fontweight="medium")

    bar = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=BLUES), ax=axes,
                       shrink=0.72, pad=0.02)
    bar.set_label("% of runs  (darker is better)", fontsize=10, color="#5b6472")
    bar.outline.set_visible(False)
    bar.ax.tick_params(length=0, colors="#5b6472", labelsize=9)

    out = folder / "analysis" / f"heatmap_v2_{policy}"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out.with_suffix(".png"), dpi=200, bbox_inches="tight", facecolor="white")
    fig.savefig(out.with_suffix(".pdf"), bbox_inches="tight", facecolor="white")
    print("wrote", out.with_suffix(".png"))


if __name__ == "__main__":
    main(Path(sys.argv[1] if len(sys.argv) > 1 else "results/acra-6v6-patrol"),
         sys.argv[2] if len(sys.argv) > 2 else "event")
