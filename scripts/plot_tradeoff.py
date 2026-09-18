"""Safety vs planning-compute trade-off per planner (one panel each).

Usage:
    python scripts/plot_tradeoff.py results/acra-6v6-patrol

Each point is one (policy, prediction horizon, replan period) arm. Lines join
the prediction horizons of one policy at one replan period; labels give the
horizon in ms. Initial-plan failures are excluded per planner.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.ticker import FuncFormatter  # noqa: E402

PLANNERS = ["voronoi", "prm", "visibility"]
PLANNER_LABEL = {"voronoi": "Voronoi", "prm": "PRM", "visibility": "Visibility graph"}
COLOR = {"once": "#2a78d6", "cycle": "#eb6834", "event": "#1baf7a"}
LABEL = {"once": "Plan once", "cycle": "Every cycle", "event": "Event-triggered"}
INK, INK_2, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def main(folder: Path) -> None:
    runs = pd.read_csv(folder / "runs.csv")
    runs["cf"] = runs["collision_episodes"] == 0
    runs["period"] = runs["replan_period_ms"].round()
    failed = set(map(tuple, runs.loc[runs.status == "planning_failed", ["scenario", "planner"]].values))
    runs = runs[[(s, p) not in failed for s, p in zip(runs.scenario, runs.planner)]]
    arms = (
        runs.groupby(["planner", "replan_policy", "prediction_horizon_ms", "period"])
        .agg(cf=("cf", "mean"), ms=("planning_time_ms_total", "median"), n=("cf", "size"))
        .reset_index()
    )
    periods = sorted(arms.period.unique())
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.4), constrained_layout=True, sharey=True)
    for ax, planner in zip(axes, PLANNERS):
        sub = arms[arms.planner == planner]
        for policy in ("cycle", "event"):
            for period, style in zip(periods, ("-", "--")):
                line = sub[(sub.replan_policy == policy) & (sub.period == period)].sort_values(
                    "prediction_horizon_ms"
                )
                if line.empty:
                    continue
                ax.plot(line.ms, 100 * line.cf, style, color=COLOR[policy], lw=2, zorder=2,
                        marker="o", ms=8 if period == periods[0] else 7,
                        mfc=COLOR[policy] if period == periods[0] else "white", mew=2,
                        label=f"{LABEL[policy]}, {period:g} ms")
                for _, row in line.iterrows():
                    ax.annotate(f"{row.prediction_horizon_ms:g}", (row.ms, 100 * row.cf),
                                xytext=(6, -3), textcoords="offset points", fontsize=6.5,
                                color=INK_2)
        once = sub[sub.replan_policy == "once"]
        ax.plot(once.ms, 100 * once.cf, "o", color=COLOR["once"], ms=8, zorder=3,
                label=LABEL["once"])
        ax.set_xscale("log")
        ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))
        ax.set_title(f"{PLANNER_LABEL[planner]} (n = {int(sub.n.max())})", fontsize=9,
                     color=INK, loc="left")
        ax.set_xlabel("Planning time per run (ms, median, log)", fontsize=8, color=INK_2)
        ax.tick_params(colors=INK_2, labelsize=8, length=0)
        for side in ("top", "right", "left"):
            ax.spines[side].set_visible(False)
        ax.spines["bottom"].set_color(GRID)
        ax.grid(True, color=GRID, lw=0.6)
        ax.set_axisbelow(True)
    axes[0].set_ylabel("Collision-free runs (%)", fontsize=8, color=INK_2)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside upper center", ncol=5, frameon=False, fontsize=8)
    out = folder / "analysis"
    out.mkdir(exist_ok=True)
    for suffix in (".pdf", ".png"):
        fig.savefig(out / f"tradeoff{suffix}", dpi=200)


if __name__ == "__main__":
    main(Path(sys.argv[1] if len(sys.argv) > 1 else "results/acra-6v6-patrol"))
