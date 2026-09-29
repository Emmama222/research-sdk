#!/usr/bin/env python3
"""What the obstacle motion-prediction horizon buys, and what it costs.

Top row: share of runs with no robot-initiated contact, against the horizon.
Bottom row: failed rebuild attempts per run -- calls that tried to produce a
route through the predicted world and returned nothing. The bottom row is the
mechanism for the top row: a planner whose roadmap survives being shifted into
the future keeps replanning successfully as the horizon grows; one whose
roadmap does not, thrashes.

Planners are panels rather than line colours so that orange/blue keeps meaning
cycle/event across every figure in the paper.

Usage:
    python scripts/plot_prediction_horizon.py results/acra-6v6-canonical
    python scripts/plot_prediction_horizon.py <batch> --period 100
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

PLANNERS = ["voronoi", "prm", "visibility"]
LABEL = {"voronoi": "Voronoi", "prm": "PRM", "visibility": "Visibility graph"}
POLICIES = ["cycle", "event"]

INK, MUTED, GRID = "#1f2733", "#5b6472", "#e3e6ea"
COLOR = {"cycle": "#d55c25", "event": "#256abf"}


def load(folder: Path) -> pd.DataFrame:
    runs = pd.read_csv(folder / "runs.csv")
    if "obstacle_episodes_robot_initiated" in runs:
        runs["safe"] = (runs.obstacle_episodes_robot_initiated == 0) & (
            runs.robot_collision_episodes == 0
        )
    else:
        runs["safe"] = runs.collision_episodes == 0
        print("  ! no attribution columns -- safety is uncorrected")
    runs["replan_period_ms"] = runs["replan_period_ms"].round(3)
    return runs[runs.replan_policy.isin(POLICIES)]


def summarize(runs: pd.DataFrame, period: float) -> pd.DataFrame:
    sub = runs[runs.replan_period_ms == period]
    if sub.empty:
        raise SystemExit(f"no runs at replan period {period} ms")
    return (
        sub.groupby(["planner", "replan_policy", "prediction_horizon_ms"])
        .agg(
            n=("safe", "size"),
            safe=("safe", "mean"),
            fails=("replan_failures", "mean"),
            rebuilds=("rebuild_calls", "mean"),
            ttg=("time_to_goal_ms", "median"),
        )
        .reset_index()
    )


def _line_panel(ax, data, planner, column, transform, ylim, ylabel, show_y):
    for policy in POLICIES:
        rows = data[(data.planner == planner) & (data.replan_policy == policy)]
        rows = rows.sort_values("prediction_horizon_ms")
        if rows.empty:
            continue
        # Dashed with an open marker for cycle: in the safety row the two
        # policies land on top of each other, which is itself the finding,
        # and a solid pair would just hide one of them.
        style = "--o" if policy == "cycle" else "-o"
        ax.plot(rows.prediction_horizon_ms, transform(rows[column]), style,
                color=COLOR[policy], lw=1.8, ms=5.5, mew=1.4, label=policy,
                mfc="white" if policy == "cycle" else COLOR[policy],
                mec=COLOR[policy] if policy == "cycle" else "white",
                zorder=3 if policy == "event" else 4)
    ax.set_ylim(*ylim)
    ax.set_xticks(sorted(data.prediction_horizon_ms.unique()))
    ax.tick_params(colors=MUTED, labelsize=7.5, length=0)
    ax.yaxis.grid(True, color=GRID, lw=0.8, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(GRID)
    if show_y:
        ax.set_ylabel(ylabel, fontsize=8.5, color=MUTED)
    else:
        ax.set_yticklabels([])


def plot(summary: pd.DataFrame, period: float, out: Path) -> None:
    fig, axes = plt.subplots(2, len(PLANNERS), figsize=(7.2, 4.2),
                             facecolor="white", sharex=True)
    # Every safety value sits between 60% and 100%, so a zero baseline would
    # squeeze the entire result into the top quarter of the panel. These are
    # lines, not bars, so the baseline carries no area to mislead with.
    safe_lo = min(60.0, 100 * summary.safe.min() - 5)
    fails_max = 1.12 * summary.fails.max()

    for col, planner in enumerate(PLANNERS):
        _line_panel(axes[0][col], summary, planner, "safe", lambda s: 100 * s,
                    (safe_lo, 102), "Collision-free (%)", col == 0)
        axes[0][col].set_title(LABEL[planner], fontsize=9.5, color=INK, pad=6)
        _line_panel(axes[1][col], summary, planner, "fails", lambda s: s,
                    (0, fails_max), "Failed rebuilds per run", col == 0)
        axes[1][col].set_xlabel("Prediction horizon (ms)", fontsize=8.5, color=MUTED)

    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower left", bbox_to_anchor=(0.005, -0.04),
               frameon=False, fontsize=8, ncol=2, labelcolor=MUTED, handlelength=1.6)
    fig.suptitle(f"Effect of the prediction horizon (replan period {period:.0f} ms)",
                 fontsize=11, color=INK, x=0.005, ha="left", y=1.0)
    fig.tight_layout(rect=(0, 0.05, 1, 0.98))
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out.with_suffix(".png"), dpi=200, bbox_inches="tight", facecolor="white")
    fig.savefig(out.with_suffix(".pdf"), bbox_inches="tight", facecolor="white")
    print(f"wrote {out.with_suffix('.png')} and {out.with_suffix('.pdf')}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("batch", type=Path)
    ap.add_argument("--period", type=float, default=20.0, help="Replan period in ms")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    runs = load(args.batch)
    summary = summarize(runs, args.period)
    show = summary.copy()
    show["safe_%"] = (100 * show.safe).round(1)
    print(show.drop(columns=["safe"]).to_string(index=False, float_format=lambda v: f"{v:,.1f}"))
    out = args.out or args.batch / "analysis" / f"prediction_horizon_{args.period:.0f}ms"
    plot(summary, args.period, out)


if __name__ == "__main__":
    sys.exit(main())
