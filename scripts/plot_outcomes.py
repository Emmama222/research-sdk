"""Paper figures for the ACRA 2026 result sets.

Reads ``results/acra2026-final/{one-shot-validation,policy-comparison,horizon-sweep}``
and writes PDF (for LaTeX) and PNG (for review) figures to ``<root>/figures/``:

1. fig1_outcomes          Outcome breakdown (strict / buffer-only / physical /
                          incomplete) per planner and arm, for the policy
                          comparison and the horizon sweep; every bar sums to
                          the valid runs.
2. fig2_tradeoff          Compute vs safety: replans per run against the
                          physical-contact share of completed runs.
3. fig3_one_shot          One-shot validation over requests that needed a map
                          (no clear straight line): planning-time and
                          path-excess distributions per planner.
4. fig4_replan_latency    Distribution of individual post-initial rebuild
                          latencies per planner, with the 100 ms replan limit.
5. fig5_route_persistence Mean installed-route lifetime per planner and arm.
6. fig6_horizon           Prediction horizon effect: strict, physical and
                          incomplete shares and replans per run against the
                          horizon, one line per planner.

Figures whose input set is missing are skipped. Colours follow the dataviz
reference palette; the outcome colours (blue / yellow / red + neutral grey)
and the planner colours (palette slots 1-3) were checked with its validator.

Usage:
    python scripts/plot_outcomes.py [results/acra2026-final]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from matplotlib.ticker import (
    FixedLocator,
    FuncFormatter,
    MaxNLocator,
    NullFormatter,
    ScalarFormatter,
)

# --- palette (dataviz reference instance, light mode) ------------------------
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
SURFACE = "#ffffff"  # print surface
OUTCOME_COLORS = {
    "strict": "#2a78d6",  # blue
    "buffer_only": "#eda100",  # yellow
    "physical": "#e34948",  # red
    "incomplete": "#c3c2b7",  # neutral grey, recessive
}
OUTCOME_LABELS = {
    "strict": "Strict (no buffer entry)",
    "buffer_only": "Buffer only (<30 mm, no contact)",
    "physical": "Physical contact",
    "incomplete": "Incomplete",
}
PLANNERS = ("voronoi", "prm", "visibility")
PLANNER_LABEL = {"voronoi": "Voronoi", "prm": "PRM", "visibility": "Visibility Graph"}
PLANNER_COLOR = {"voronoi": "#2a78d6", "prm": "#eb6834", "visibility": "#1baf7a"}
ARM_COLOR = {  # palette slots 1-4, fixed order
    ("cycle", 20.0): "#2a78d6",
    ("event", 20.0): "#eb6834",
    ("cycle", 100.0): "#1baf7a",
    ("event", 100.0): "#eda100",
}
REPLAN_LIMIT_MS = 100.0

plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Segoe UI", "Helvetica", "Arial", "DejaVu Sans"],
        "font.size": 8,
        "axes.titlesize": 9,
        "axes.labelsize": 8,
        "axes.edgecolor": AXIS,
        "axes.linewidth": 0.6,
        "axes.labelcolor": INK_2,
        "axes.titlecolor": INK,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "xtick.labelcolor": INK_2,
        "ytick.labelcolor": INK_2,
        "xtick.major.width": 0.6,
        "ytick.major.width": 0.6,
        "grid.color": GRID,
        "grid.linewidth": 0.6,
        "grid.linestyle": "-",
        "legend.frameon": False,
        "legend.fontsize": 7.5,
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE,
        "pdf.fonttype": 42,
    }
)


def _read(path: Path) -> pd.DataFrame | None:
    return pd.read_csv(path) if path.exists() else None


def _arm_label(policy: str, period: float) -> str:
    return f"{policy.capitalize()} @ {period:g} ms"


def _clean(ax, *, grid_axis: str | None = "x") -> None:
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    if grid_axis:
        ax.grid(axis=grid_axis)
        ax.set_axisbelow(True)


def _save(fig, out: Path, name: str) -> None:
    out.mkdir(parents=True, exist_ok=True)
    for ext in ("pdf", "png"):
        fig.savefig(out / f"{name}.{ext}", dpi=200 if ext == "png" else None, bbox_inches="tight")
    plt.close(fig)
    print(f"  {name}.pdf / .png")


# --- figure 1: outcome breakdown -------------------------------------------


def _outcome_panel(ax, rows: pd.DataFrame, row_label, title: str) -> None:
    """Horizontal 100%-of-runs stacked bars, grouped by planner."""
    rows = rows.copy()
    rows["_order"] = rows["planner"].map({p: i for i, p in enumerate(PLANNERS)})
    rows = rows.sort_values(["_order", "_sub"]).reset_index(drop=True)
    y_positions, labels, y, last_planner = [], [], 0.0, None
    for _, row in rows.iterrows():
        if last_planner is not None and row["planner"] != last_planner:
            y += 0.6  # air between planner groups
        y_positions.append(y)
        labels.append(row_label(row))
        last_planner = row["planner"]
        y += 1.0
    total = int(rows["valid_runs"].max())
    for yp, (_, row) in zip(y_positions, rows.iterrows()):
        left = 0
        for key in ("strict", "buffer_only", "physical", "incomplete"):
            value = int(row[key])
            if value <= 0:
                continue
            ax.barh(
                yp, value, left=left, height=0.62, color=OUTCOME_COLORS[key],
                edgecolor=SURFACE, linewidth=1.0,
            )
            if value >= 0.07 * total:  # label only when it fits
                text_color = INK if key in ("buffer_only", "incomplete") else SURFACE
                ax.text(left + value / 2, yp, str(value), ha="center", va="center",
                        fontsize=7, color=text_color)
            left += value
    ax.set_yticks(y_positions, labels)
    ax.invert_yaxis()
    ax.set_xlim(0, total)
    ax.xaxis.set_major_locator(MaxNLocator(nbins=10, integer=True))
    ax.set_xlabel(f"Runs (of {total} matched scenarios)")
    ax.set_title(title, loc="left")
    ax.tick_params(axis="y", length=0)
    _clean(ax)
    # planner group names on the left margin
    groups = {}
    for yp, (_, row) in zip(y_positions, rows.iterrows()):
        groups.setdefault(row["planner"], []).append(yp)
    for planner, ys in groups.items():
        ax.text(-0.30, np.mean(ys), PLANNER_LABEL[planner], transform=ax.get_yaxis_transform(),
                ha="right", va="center", fontsize=8, color=INK, fontweight="bold")


def figure_outcomes(policy: pd.DataFrame | None, sweep: pd.DataFrame | None, out: Path) -> None:
    panels = []
    if policy is not None:
        p = policy.copy()
        p["_sub"] = p["replan_period_ms"] * 10 + (p["replan_policy"] == "event")
        panels.append((p, lambda r: _arm_label(r["replan_policy"], r["replan_period_ms"]),
                       "(a) Dynamic Replanning Policy Comparison"))
    if sweep is not None or policy is not None:
        base = (
            policy[(policy.replan_policy == "event") & (policy.replan_period_ms == 20.0)
                   & (policy.prediction_horizon_ms == 0.0)]
            if policy is not None else pd.DataFrame()
        )
        h = pd.concat([base, sweep if sweep is not None else pd.DataFrame()], ignore_index=True)
        if not h.empty:
            h["_sub"] = h["prediction_horizon_ms"]
            panels.append((h, lambda r: f"Horizon {r['prediction_horizon_ms']:g} ms",
                           "(b) Prediction Horizon Sweep (event @ 20 ms)"))
    if not panels:
        return
    heights = [len(p[0]) * 0.22 + 0.9 for p in panels]
    fig, axes = plt.subplots(len(panels), 1, figsize=(7.0, sum(heights)),
                             gridspec_kw={"height_ratios": heights})
    axes = np.atleast_1d(axes)
    for ax, (rows, labeler, title) in zip(axes, panels):
        _outcome_panel(ax, rows, labeler, title)
    handles = [Patch(color=OUTCOME_COLORS[k], label=OUTCOME_LABELS[k]) for k in OUTCOME_COLORS]
    fig.legend(handles=handles, loc="upper center", ncol=4, bbox_to_anchor=(0.55, 1.02))
    fig.tight_layout(rect=(0.08, 0, 1, 0.97))
    _save(fig, out, "fig1_outcomes")


# --- figure 2: compute vs safety -------------------------------------------


def figure_tradeoff(policy: pd.DataFrame | None, out: Path) -> None:
    if policy is None:
        return
    fig, ax = plt.subplots(figsize=(3.5, 2.8))
    markers = {"cycle": "s", "event": "o"}
    for _, row in policy.iterrows():
        color = PLANNER_COLOR[row["planner"]]
        filled = row["replan_period_ms"] == 20.0
        ax.scatter(
            row["replans_per_run"], 100 * (row["physical_rate"] or 0), s=42,
            marker=markers[row["replan_policy"]],
            facecolor=color if filled else SURFACE, edgecolor=color, linewidth=1.4,
            zorder=3,
        )
    ax.set_xscale("log")
    ax.set_xlabel("Successful replans per run (log scale)")
    ax.set_ylabel("Physical contact (% of completed runs)")
    _clean(ax, grid_axis="both")
    key = [
        Line2D([], [], marker="s", ls="", mfc=INK_2, mec=INK_2, label="Cycle @ 20 ms"),
        Line2D([], [], marker="s", ls="", mfc=SURFACE, mec=INK_2, mew=1.4, label="Cycle @ 100 ms"),
        Line2D([], [], marker="o", ls="", mfc=INK_2, mec=INK_2, label="Event @ 20 ms"),
        Line2D([], [], marker="o", ls="", mfc=SURFACE, mec=INK_2, mew=1.4, label="Event @ 100 ms"),
        Line2D([], [], ls="", label=""),
    ] + [Line2D([], [], marker="o", ls="", mfc=PLANNER_COLOR[p], mec=PLANNER_COLOR[p],
                label=PLANNER_LABEL[p]) for p in PLANNERS]
    ax.legend(handles=key, loc="upper left", bbox_to_anchor=(1.0, 1.0), handletextpad=0.3)
    ax.set_title("Compute vs safety", loc="left")
    _save(fig, out, "fig2_tradeoff")


# --- figure 3: one-shot validation ------------------------------------------


def _ecdf(ax, values, color, label) -> None:
    values = np.sort(np.asarray(values, dtype=float))
    if values.size:
        ax.step(values, 100 * np.arange(1, values.size + 1) / values.size, where="post",
                color=color, lw=1.6, label=label)


def figure_one_shot(robots: pd.DataFrame | None, out: Path) -> None:
    """Planning time and path excess for requests that needed a map.

    Requests with a clear straight line (two-waypoint routes) take ~0.03 ms for
    every planner and are left out; their share is stated in the caption text.
    """
    if robots is None:
        return
    ok = robots[robots["route_available"].astype(str).str.lower() == "true"]
    mapped = ok[ok["waypoints"] > 2]
    fig, (ax_t, ax_x) = plt.subplots(1, 2, figsize=(7.0, 2.5))
    for planner in PLANNERS:
        sub = mapped[mapped.planner == planner]
        if sub.empty:
            continue
        share_direct = 1 - len(sub) / max(len(ok[ok.planner == planner]), 1)
        label = f"{PLANNER_LABEL[planner]} (n = {len(sub)}; {share_direct:.0%} direct omitted)"
        _ecdf(ax_t, sub["total_ms"], PLANNER_COLOR[planner], label)
        _ecdf(ax_x, 100 * sub["path_excess"], PLANNER_COLOR[planner], label)
        med_t = sub["total_ms"].median()
        ax_t.plot([med_t], [50], "o", ms=5, color=PLANNER_COLOR[planner], mec=SURFACE, mew=1.2)
    ax_t.set_xscale("log")
    ax_t.xaxis.set_major_locator(FixedLocator([2, 3, 5, 10, 20, 30, 50]))
    ax_t.xaxis.set_major_formatter(ScalarFormatter())
    ax_t.xaxis.set_minor_formatter(NullFormatter())
    ax_t.set_xlabel("Initial planning time per robot (ms, log)")
    ax_t.set_ylabel("Routes planned within x ms (%)")
    ax_t.set_title("(a) Planning time (dot = median)", loc="left")
    ax_x.set_xscale("symlog", linthresh=1)
    ax_x.xaxis.set_major_locator(FixedLocator([0, 1, 10, 100]))
    ax_x.xaxis.set_major_formatter(ScalarFormatter())
    ax_x.xaxis.set_minor_formatter(NullFormatter())
    ax_x.set_xlabel("Path excess over straight line (%, log above 1%)")
    ax_x.set_ylabel("Routes at most x% longer (%)")
    ax_x.set_title("(b) Route length", loc="left")
    for ax in (ax_t, ax_x):
        ax.set_ylim(0, 102)
        ax.axhline(50, color=GRID, lw=0.6, zorder=0)
        _clean(ax, grid_axis="both")
    handles, labels = ax_t.get_legend_handles_labels()
    fig.tight_layout()
    fig.subplots_adjust(bottom=0.34)
    fig.legend(handles, labels, loc="lower center", ncol=3, bbox_to_anchor=(0.5, 0.0),
               handlelength=1.6, fontsize=7)
    _save(fig, out, "fig3_one_shot")


# --- figure 4: replan latency ----------------------------------------------


def figure_latency(calls: pd.DataFrame | None, out: Path) -> None:
    if calls is None or calls.empty:
        return
    planners = [p for p in PLANNERS if p in set(calls.planner)]
    fig, axes = plt.subplots(1, len(planners), figsize=(7.0, 2.6), sharey=True)
    axes = np.atleast_1d(axes)
    lo = max(calls["rebuild_ms"].min(), 1e-3)
    hi = max(calls["rebuild_ms"].max(), REPLAN_LIMIT_MS * 1.5)
    for ax, planner in zip(axes, planners):
        sub = calls[(calls.planner == planner) & (calls.prediction_horizon_ms == 0.0)]
        for (policy, period), color in ARM_COLOR.items():
            values = np.sort(sub[(sub.replan_policy == policy) & (sub.replan_period_ms == period)]
                             ["rebuild_ms"].to_numpy())
            if values.size == 0:
                continue
            ax.step(values, 100 * np.arange(1, values.size + 1) / values.size, where="post",
                    color=color, lw=1.5, label=_arm_label(policy, period))
        ax.axvline(REPLAN_LIMIT_MS, color=INK_2, lw=0.8)
        ax.text(REPLAN_LIMIT_MS * 0.88, 50, "100 ms replan limit", fontsize=6.5, color=INK_2,
                ha="right", va="center", rotation=90)
        ax.set_xscale("log")
        ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))
        ax.set_xlim(lo, hi)
        ax.set_ylim(0, 102)
        ax.set_title(PLANNER_LABEL[planner], loc="left")
        ax.set_xlabel("Rebuild latency (ms, log)")
        _clean(ax, grid_axis="both")
    axes[0].set_ylabel("Rebuilds finished within x ms (%)")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.tight_layout()
    fig.subplots_adjust(bottom=0.30)
    fig.legend(handles, labels, loc="lower center", ncol=4, bbox_to_anchor=(0.5, 0.0),
               handlelength=1.6)
    _save(fig, out, "fig4_replan_latency")


# --- figure 5: route persistence -------------------------------------------


def figure_routes(policy: pd.DataFrame | None, out: Path) -> None:
    """Grouped bars: mean installed-route lifetime per arm, one bar per planner."""
    if policy is None:
        return
    arms = list(ARM_COLOR)
    fig, ax = plt.subplots(figsize=(4.6, 2.6))
    width = 0.24
    offsets = {p: (i - 1) * width for i, p in enumerate(PLANNERS)}
    for planner in PLANNERS:
        for i, (policy_name, period) in enumerate(arms):
            row = policy[(policy.planner == planner) & (policy.replan_policy == policy_name)
                         & (policy.replan_period_ms == period)]
            if row.empty:
                continue
            value = float(row["route_lifetime_ms_mean"].iloc[0])
            ax.bar(i + offsets[planner], value, width=width, color=PLANNER_COLOR[planner],
                   edgecolor=SURFACE, linewidth=1.0,
                   label=PLANNER_LABEL[planner] if i == 0 else None)
            ax.text(i + offsets[planner], value, f"{value:.0f}", ha="center", va="bottom",
                    fontsize=6.3, color=INK_2)
    ax.set_xticks(range(len(arms)), [f"{p.capitalize()}\n{t:g} ms" for p, t in arms])
    ax.tick_params(axis="x", length=0)
    ax.set_ylabel("Mean route lifetime (ms)")
    ax.set_ylim(0, float(policy["route_lifetime_ms_mean"].max()) * 1.15)
    _clean(ax, grid_axis="y")
    ax.legend(loc="upper left", bbox_to_anchor=(1.0, 1.0))
    ax.set_title("How long a route stays in use", loc="left")
    _save(fig, out, "fig5_route_persistence")


# --- figure 6: prediction horizon effect ------------------------------------


def figure_horizon(policy: pd.DataFrame | None, sweep: pd.DataFrame | None, out: Path) -> None:
    """Small multiples: outcome shares and replanning demand against horizon."""
    if sweep is None:
        return
    base = (
        policy[(policy.replan_policy == "event") & (policy.replan_period_ms == 20.0)
               & (policy.prediction_horizon_ms == 0.0)]
        if policy is not None else pd.DataFrame()
    )
    rows = pd.concat([base, sweep], ignore_index=True)
    panels = [
        ("strict_rate", "Strict\n(% of completed runs)", 100.0),
        ("physical_rate", "Physical contact\n(% of completed runs)", 100.0),
        ("incomplete_rate", "Incomplete\n(% of all runs)", 100.0),
        ("replans_per_run", "Replans\nper run", 1.0),
    ]
    fig, axes = plt.subplots(1, len(panels), figsize=(7.0, 2.6))
    horizons = sorted(rows["prediction_horizon_ms"].unique())
    for ax, (column, title, scale) in zip(axes, panels):
        for planner in PLANNERS:
            sub = rows[rows.planner == planner].sort_values("prediction_horizon_ms")
            if sub.empty:
                continue
            ax.plot(sub["prediction_horizon_ms"], scale * sub[column], color=PLANNER_COLOR[planner],
                    lw=2, marker="o", ms=5.5, mec=SURFACE, mew=1.5, label=PLANNER_LABEL[planner])
        ax.set_xticks(horizons, [f"{h:g}" for h in horizons])
        ax.set_xlabel("Horizon (ms)")
        ax.set_title(title, loc="left", fontsize=8)
        ax.set_ylim(bottom=0)
        _clean(ax, grid_axis="y")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.tight_layout()
    fig.subplots_adjust(bottom=0.33, wspace=0.42)
    fig.legend(handles, labels, loc="lower center", ncol=3, bbox_to_anchor=(0.5, 0.0))
    _save(fig, out, "fig6_horizon")


def main() -> int:
    parser = argparse.ArgumentParser(description="Build the ACRA 2026 paper figures.")
    parser.add_argument("root", type=Path, nargs="?", default=Path("results/acra2026-final"))
    args = parser.parse_args()
    robots = _read(args.root / "one-shot-validation" / "robots.csv")
    policy = _read(args.root / "policy-comparison" / "summary.csv")
    sweep = _read(args.root / "horizon-sweep" / "summary.csv")
    calls = _read(args.root / "policy-comparison" / "rebuild_calls.csv")
    out = args.root / "figures"
    print(f"Writing figures to {out}")
    figure_outcomes(policy, sweep, out)
    figure_tradeoff(policy, out)
    figure_one_shot(robots, out)
    figure_latency(calls, out)
    figure_routes(policy, out)
    figure_horizon(policy, sweep, out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
