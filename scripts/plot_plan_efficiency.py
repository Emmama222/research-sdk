#!/usr/bin/env python3
"""Planner calls split into the ones that produced a route and the ones that did not.

Every control tick the executor consults the planner once per active robot.
``headless.py`` tags each call: ``rebuild`` when it returned a new route,
otherwise ``check`` -- the call only confirmed the route already being
followed. The two counts partition ``planner_calls`` exactly, so one stacked
bar per arm shows how much of the planning budget bought a new route.

Usage:
    python scripts/plot_plan_efficiency.py results/acra-6v6-canonical
    python scripts/plot_plan_efficiency.py <batch> --out figures/plan_efficiency
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

PLANNERS = ["voronoi", "prm", "visibility"]
LABEL = {"voronoi": "Voronoi", "prm": "PRM", "visibility": "Visibility graph"}
POLICIES = ["cycle", "event"]

INK, MUTED, GRID = "#1f2733", "#5b6472", "#e3e6ea"
# Policy hues are the ones plot_headline.py already uses, so a reader carries
# "orange = cycle, blue = event" across figures. Within a bar the dark step is
# the rebuild and the light step the check; the two light tints sit at the same
# OKLab lightness (0.905) so neither policy reads as heavier than the other.
FILL = {
    ("cycle", "rebuild"): "#d55c25",
    ("cycle", "check"): "#f9d9c2",
    ("event", "rebuild"): "#256abf",
    ("event", "check"): "#cde2fb",
}


def load(folder: Path) -> pd.DataFrame:
    runs = pd.read_csv(folder / "runs.csv")
    missing = {"rebuild_calls", "check_calls", "planner_calls"} - set(runs.columns)
    if missing:
        raise SystemExit(f"{folder}/runs.csv has no {', '.join(sorted(missing))} columns")
    runs["replan_period_ms"] = runs["replan_period_ms"].round(3)
    # 'once' plans at t=0 and never again, so it has no call budget to speak of
    # (~6 calls per run against ~700). Including it would make every other bar a
    # sliver; it is the baseline for the other figures, not for this one.
    return runs[runs.replan_policy.isin(POLICIES)]


def summarize(runs: pd.DataFrame) -> pd.DataFrame:
    grouped = (
        runs.groupby(["replan_period_ms", "planner", "replan_policy"])
        .agg(
            runs=("planner_calls", "size"),
            calls=("planner_calls", "mean"),
            rebuilds=("rebuild_calls", "mean"),
            checks=("check_calls", "mean"),
            replan_failures=("replan_failures", "mean"),
        )
        .reset_index()
    )
    grouped["useful_pct"] = 100 * grouped.rebuilds / grouped.calls
    return grouped


def _panel(ax, data: pd.DataFrame, period: float, show_ylabels: bool) -> None:
    rows = []
    for planner in PLANNERS:
        for policy in POLICIES:
            match = data[(data.planner == planner) & (data.replan_policy == policy)]
            if len(match):
                rows.append((planner, policy, match.iloc[0]))
    positions = list(range(len(rows)))[::-1]

    for pos, (planner, policy, row) in zip(positions, rows):
        ax.barh(pos, row.rebuilds, height=0.62, color=FILL[(policy, "rebuild")],
                edgecolor="white", linewidth=0.8, zorder=3)
        ax.barh(pos, row.checks, left=row.rebuilds, height=0.62,
                color=FILL[(policy, "check")], edgecolor="white", linewidth=0.8, zorder=3)
        ax.text(row.calls * 1.03, pos,
                f"{row.rebuilds:,.0f}/{row.calls:,.0f}  {row.useful_pct:.1f}%",
                va="center", ha="left", fontsize=7, color=MUTED, zorder=4)

    ax.set_yticks(positions)
    if show_ylabels:
        ax.set_yticklabels([f"{LABEL[p]} · {q}" for p, q, _ in rows],
                           fontsize=8, color=INK)
    else:
        ax.set_yticklabels([])
    ax.set_xlabel("Planner calls per run", fontsize=8.5, color=MUTED)
    ax.set_title(f"Replan period {period:.0f} ms", fontsize=9.5, color=INK, pad=6)
    ax.tick_params(colors=MUTED, labelsize=7.5, length=0)
    ax.xaxis.grid(True, color=GRID, lw=0.8, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(GRID)
    ax.margins(x=0.30, y=0.10)


def plot(summary: pd.DataFrame, out: Path) -> None:
    periods = sorted(summary.replan_period_ms.unique())
    # One x-scale across panels. Independent scales would draw the 100 ms bars
    # the same length as the 20 ms ones, hiding that the longer period costs
    # roughly a fifth as many calls -- which is half of what this figure says.
    fig, axes = plt.subplots(1, len(periods), figsize=(7.2, 2.9),
                             facecolor="white", sharex=True)
    axes = [axes] if len(periods) == 1 else list(axes)
    for index, (ax, period) in enumerate(zip(axes, periods)):
        _panel(ax, summary[summary.replan_period_ms == period], period,
               show_ylabels=index == 0)

    legend = [
        Patch(facecolor=FILL[(policy, kind)], edgecolor="white",
              label=f"{policy} · {kind}")
        for policy in POLICIES
        for kind in ("rebuild", "check")
    ]
    fig.legend(handles=legend, loc="lower left", bbox_to_anchor=(0.005, -0.08),
               frameon=False, fontsize=8, ncol=4, labelcolor=MUTED,
               columnspacing=1.4, handlelength=1.1)
    fig.suptitle("How much planning bought a new route", fontsize=11, color=INK,
                 x=0.005, ha="left", y=1.03)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out.with_suffix(".png"), dpi=200, bbox_inches="tight", facecolor="white")
    fig.savefig(out.with_suffix(".pdf"), bbox_inches="tight", facecolor="white")
    print(f"wrote {out.with_suffix('.png')} and {out.with_suffix('.pdf')}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("batch", type=Path)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    runs = load(args.batch)
    summary = summarize(runs)
    print(summary.to_string(index=False, float_format=lambda v: f"{v:,.1f}"))
    plot(summary, args.out or args.batch / "analysis" / "plan_efficiency")


if __name__ == "__main__":
    sys.exit(main())
