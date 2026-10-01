"""Paper figures for the ACRA 2026 result sets.

Reads ``results/acra2026-final/`` and writes PDF (vector, for LaTeX) and PNG
(for viewing) figures to ``<root>/figures/``. Every figure has a title that
states what the data shows, with the key numbers computed from the results,
and a one-line subtitle explaining how to read it.

Main result sets
  oneshot_planning_time_and_route_length   One-Shot Planning Validation
  policy_outcomes                          Dynamic Replanning Policy Comparison
  policy_collision_free                    "
  policy_compute_vs_safety                 "
  policy_replan_latency_20ms / _100ms      "
  policy_route_lifetime                    "
  horizon_outcomes                         Prediction Horizon Sweep
  horizon_trends                           "
Supporting sets (under <root>/supporting/<set>/), drawn only when present
  ablation_event_route                     event vs event_route trigger (DEC-010)
  evidence_horizon_150                     why the sweep stops at 100 ms (DEC-020)
  evidence_no_route_limit                  why the no-route limit is 1000 ms (DEC-019)

Colours follow the dataviz reference palette and were checked with its
validator. Usage: python scripts/plot_outcomes.py [results/acra2026-final]
"""

from __future__ import annotations

import argparse
import textwrap
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
SURFACE = "#ffffff"
OUTCOME_COLORS = {
    "strict": "#2a78d6",
    "buffer_only": "#eda100",
    "physical": "#e34948",
    "incomplete": "#c3c2b7",
}
OUTCOME_LABELS = {
    "strict": "Finished, never closer than 30 mm",
    "buffer_only": "Finished, came within 30 mm",
    "physical": "Finished, but touched something",
    "incomplete": "Did not finish",
}
PLANNERS = ("voronoi", "prm", "visibility")
PLANNER_LABEL = {"voronoi": "Voronoi", "prm": "PRM", "visibility": "Visibility Graph"}
PLANNER_COLOR = {"voronoi": "#2a78d6", "prm": "#eb6834", "visibility": "#1baf7a"}
POLICY_COLOR = {"event": "#4a3aa7", "cycle": "#eda100", "event_route": "#e87ba4"}
POLICY_LABEL = {"event": "Event trigger (ours)", "cycle": "Cycle (every check)",
                "event_route": "Full-path recalculation"}
REPLAN_LIMIT_MS = 100.0

plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["Segoe UI", "Helvetica", "Arial", "DejaVu Sans"],
        "font.size": 8,
        "axes.titlesize": 8.5,
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


# --- helpers -------------------------------------------------------------------


def _read(path: Path) -> pd.DataFrame | None:
    if not path.exists():
        return None
    df = pd.read_csv(path)
    # Summaries written before DEC-021 lack the collision-free columns; derive them.
    if "strict" in df and "collision_free_completion_rate" not in df:
        df["collision_free_completed"] = df["strict"] + df["buffer_only"]
        df["collision_free_completion_rate"] = df["collision_free_completed"] / df["valid_runs"]
        df["collision_free_completed_label"] = (
            df["collision_free_completed"].astype(str) + "/" + df["valid_runs"].astype(str)
        )
    return df


def _arm_label(policy: str, period: float) -> str:
    names = {"event": "Event", "cycle": "Cycle", "event_route": "Full-path"}
    return f"{names.get(policy, policy)} @ {period:g} ms"


def _row(df: pd.DataFrame, planner: str, policy: str, period: float, horizon: float = 0.0):
    sel = df[(df.planner == planner) & (df.replan_policy == policy)
             & (df.replan_period_ms == period) & (df.prediction_horizon_ms == horizon)]
    return sel.iloc[0] if len(sel) else None


def _clean(ax, *, grid_axis: str | None = "x") -> None:
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    if grid_axis:
        ax.grid(axis=grid_axis)
        ax.set_axisbelow(True)


SHORT_TITLES = {
    "oneshot_planning_time_and_route_length": "Planning time vs route length, by planner",
    "policy_outcomes": "Run outcomes: cycle vs event trigger, by planner",
    "policy_collision_free": "Collision-free finishes: cycle vs event trigger, by planner",
    "policy_compute_vs_safety": "Replanning effort vs contact rate, by planner and policy",
    "policy_replan_latency_20ms": "Rebuild time: cycle vs event trigger at 20 ms checks",
    "policy_replan_latency_100ms": "Rebuild time: cycle vs event trigger at 100 ms checks",
    "policy_route_lifetime": "Route lifetime: cycle vs event trigger, by planner",
    "horizon_outcomes": "Run outcomes vs prediction horizon, by planner",
    "horizon_trends": "Safety and replanning vs prediction horizon, by planner",
    "ablation_event_route": "Run outcomes: event vs full-path recalculation, by planner",
    "evidence_horizon_150": "Run outcomes vs prediction horizon, including 150 ms",
    "evidence_no_route_limit": "Run outcomes: 500 ms vs 1000 ms no-route limit, by planner",
}


PAPER = False  # set by --paper


def _titled(fig, title: str, subtitle: str, name: str | None = None) -> float:
    """Finding-style title plus a plain-language reading line; returns the free top fraction.

    Positions are in inches from the top edge, so spacing is the same for every
    figure height. Long lines wrap to the figure width.
    """
    if PAPER:  # --paper: the LaTeX caption carries the title and the finding
        return 1 - 0.05 / fig.get_size_inches()[1]
    if name in SHORT_TITLES:  # short comparison title; the finding moves to the subtitle
        subtitle = f"{title}. {subtitle}"
        title = SHORT_TITLES[name]
    width_in, height_in = fig.get_size_inches()
    t_lines = textwrap.wrap(title, width=int(width_in * 13.2))
    s_lines = textwrap.wrap(subtitle, width=int(width_in * 17.5))
    y = 1 - 0.08 / height_in
    fig.text(0.01, y, "\n".join(t_lines), ha="left", va="top", fontsize=9.5, fontweight="bold",
             color=INK, linespacing=1.25)
    y -= (0.19 * len(t_lines) + 0.06) / height_in
    fig.text(0.01, y, "\n".join(s_lines), ha="left", va="top", fontsize=7.6, color=INK_2,
             linespacing=1.3)
    y -= (0.15 * len(s_lines) + 0.12) / height_in
    return y


def _save(fig, out: Path, name: str) -> None:
    out.mkdir(parents=True, exist_ok=True)
    for ext in ("pdf", "png"):
        fig.savefig(out / f"{name}.{ext}", dpi=200 if ext == "png" else None, bbox_inches="tight")
    plt.close(fig)
    print(f"  {name}.pdf / .png")


def _outcome_legend(fig, y: float) -> None:
    handles = [Patch(color=OUTCOME_COLORS[k], label=OUTCOME_LABELS[k]) for k in OUTCOME_COLORS]
    fig.legend(handles=handles, loc="upper center", ncol=4, bbox_to_anchor=(0.5, y),
               handlelength=1.2, columnspacing=1.2)


def _outcome_axes(ax, rows: pd.DataFrame, row_label) -> None:
    """Horizontal stacked bars (strict / buffer-only / physical / incomplete), by planner."""
    rows = rows.copy()
    rows["_order"] = rows["planner"].map({p: i for i, p in enumerate(PLANNERS)})
    rows = rows.sort_values(["_order", "_sub"]).reset_index(drop=True)
    ys, labels, y, last = [], [], 0.0, None
    headers: dict[str, float] = {}
    for _, row in rows.iterrows():
        if row["planner"] != last:
            y += 0.4 if last is None else 1.1
            headers[row["planner"]] = y - 0.85
        ys.append(y)
        labels.append(row_label(row))
        last = row["planner"]
        y += 1.0
    total = int(rows["valid_runs"].max())
    for yp, (_, row) in zip(ys, rows.iterrows()):
        left = 0
        for key in ("strict", "buffer_only", "physical", "incomplete"):
            value = int(row[key])
            if value <= 0:
                continue
            ax.barh(yp, value, left=left, height=0.62, color=OUTCOME_COLORS[key],
                    edgecolor=SURFACE, linewidth=1.0)
            if value >= 0.06 * total:
                ink = INK if key in ("buffer_only", "incomplete") else SURFACE
                ax.text(left + value / 2, yp, str(value), ha="center", va="center", fontsize=7,
                        color=ink)
            left += value
    ax.set_yticks(ys, labels)
    ax.set_xlim(0, total)
    ax.xaxis.set_major_locator(MaxNLocator(nbins=10, integer=True))
    ax.set_xlabel(f"Runs (out of {total} matched scenarios)")
    ax.tick_params(axis="y", length=0)
    _clean(ax)
    for planner, hy in headers.items():
        ax.text(0, hy, PLANNER_LABEL[planner], ha="left", va="center", fontsize=8.5, color=INK,
                fontweight="bold")
    ax.set_ylim(max(ys) + 0.6, min(headers.values()) - 0.4)


def _outcome_figure(rows, row_label, title, subtitle, out, name) -> None:
    n_groups = rows["planner"].nunique()
    height = len(rows) * 0.26 + n_groups * 0.3 + 2.1
    fig, ax = plt.subplots(figsize=(7.2, height))
    _outcome_axes(ax, rows, row_label)
    free = _titled(fig, title, subtitle, name)
    _outcome_legend(fig, free)
    fig.tight_layout(rect=(0, 0, 1, free - 0.30 / height))
    _save(fig, out, name)


def _horizon_rows(policy, sweep, extra=None) -> pd.DataFrame:
    base = (policy[(policy.replan_policy == "event") & (policy.replan_period_ms == 20.0)
                   & (policy.prediction_horizon_ms == 0.0)] if policy is not None else None)
    parts = [p for p in (base, sweep, extra) if p is not None and not p.empty]
    return pd.concat(parts, ignore_index=True) if parts else pd.DataFrame()


def _pct(x) -> str:
    return f"{100 * float(x):.0f}%"


# --- One-Shot Planning Validation ---------------------------------------------


def _ecdf(ax, values, color, label, lw=1.8) -> None:
    values = np.sort(np.asarray(values, dtype=float))
    if values.size:
        ax.step(values, 100 * np.arange(1, values.size + 1) / values.size, where="post",
                color=color, lw=lw, label=label)


def figure_one_shot(robots: pd.DataFrame | None, out: Path) -> None:
    if robots is None:
        return
    ok = robots[robots["route_available"].astype(str).str.lower() == "true"]
    mapped = ok[ok["waypoints"] > 2]
    med = {p: mapped[mapped.planner == p]["total_ms"].median() for p in PLANNERS}
    exc = {p: 100 * mapped[mapped.planner == p]["path_excess"].median() for p in PLANNERS}
    fastest = min(med, key=med.get)
    title = (f"Before moving: {PLANNER_LABEL[fastest]} plans fastest (median "
             f"{med[fastest]:.1f} ms), but longer routes are the price "
             f"(median +{exc['voronoi']:.0f}% Voronoi vs +{exc['visibility']:.1f}% Visibility Graph)")
    subtitle = ("Only requests whose straight line to the goal was blocked. "
                "Each curve shows what share of routes were ready within x ms (left) "
                "or were at most x% longer than a straight line (right); further left is better.")
    fig, (ax_t, ax_x) = plt.subplots(1, 2, figsize=(7.2, 3.2))
    for planner in PLANNERS:
        sub = mapped[mapped.planner == planner]
        share_direct = 1 - len(sub) / max(len(ok[ok.planner == planner]), 1)
        label = f"{PLANNER_LABEL[planner]} (n = {len(sub)}; {share_direct:.0%} straight-line requests left out)"
        _ecdf(ax_t, sub["total_ms"], PLANNER_COLOR[planner], label)
        _ecdf(ax_x, 100 * sub["path_excess"], PLANNER_COLOR[planner], label)
        ax_t.plot([med[planner]], [50], "o", ms=5.5, color=PLANNER_COLOR[planner], mec=SURFACE,
                  mew=1.2)
        ax_t.annotate(f"{med[planner]:.1f} ms", (med[planner], 50), xytext=(4, -9),
                      textcoords="offset points", fontsize=6.8, color=INK_2)
    ax_t.set_xscale("log")
    ax_t.xaxis.set_major_locator(FixedLocator([2, 3, 5, 10, 20, 30, 50]))
    ax_t.xaxis.set_major_formatter(ScalarFormatter())
    ax_t.xaxis.set_minor_formatter(NullFormatter())
    ax_t.set_xlabel("Time to plan the first route (ms, log)")
    ax_t.set_ylabel("Routes ready within x ms (%)")
    ax_t.set_title("Planning time (dot = median)", loc="left")
    ax_x.set_xscale("symlog", linthresh=1)
    ax_x.xaxis.set_major_locator(FixedLocator([0, 1, 10, 100]))
    ax_x.xaxis.set_major_formatter(ScalarFormatter())
    ax_x.xaxis.set_minor_formatter(NullFormatter())
    ax_x.set_xlabel("Extra length over a straight line (%, log above 1%)")
    ax_x.set_ylabel("Routes at most x% longer (%)")
    ax_x.set_title("Route length", loc="left")
    for ax in (ax_t, ax_x):
        ax.set_ylim(0, 102)
        _clean(ax, grid_axis="both")
    handles, labels = ax_t.get_legend_handles_labels()
    free = _titled(fig, title, subtitle, "oneshot_planning_time_and_route_length")
    fig.tight_layout(rect=(0, 0.14, 1, free))
    fig.legend(handles, labels, loc="lower center", ncol=1, bbox_to_anchor=(0.5, 0.0), fontsize=7)
    _save(fig, out, "oneshot_planning_time_and_route_length")


# --- Dynamic Replanning Policy Comparison ---------------------------------------


def figure_policy_outcomes(policy: pd.DataFrame | None, out: Path) -> None:
    if policy is None:
        return
    rows = policy.copy()
    rows["_sub"] = rows["replan_period_ms"] * 10 + (rows["replan_policy"] == "event")
    pairs = [(p, t) for p in PLANNERS for t in (20.0, 100.0)]
    event_ok = all(
        _row(policy, p, "event", t) is not None and _row(policy, p, "cycle", t) is not None
        and _row(policy, p, "event", t)["completed"] >= _row(policy, p, "cycle", t)["completed"]
        for p, t in pairs
    )
    v_c, v_e = _row(policy, "visibility", "cycle", 20.0), _row(policy, "visibility", "event", 20.0)
    lead = ("Event triggering finishes as many runs as cycling or more"
            if event_ok else "Run outcomes by planner and policy")
    title = (f"{lead} (Visibility Graph at 20 ms: {int(v_c['completed'])} vs "
             f"{int(v_e['completed'])} of 200), but touches obstacles about as often")
    subtitle = ("Each bar is the same 200 scenarios. Cycle rebuilds the route at every check; "
                "event (ours) rebuilds only when the route ahead is blocked.")
    _outcome_figure(rows, lambda r: _arm_label(r["replan_policy"], r["replan_period_ms"]),
                    title, subtitle, out, "policy_outcomes")


def figure_collision_free(policy: pd.DataFrame | None, out: Path) -> None:
    if policy is None or "collision_free_completion_rate" not in policy:
        return
    arms = [("cycle", 20.0), ("event", 20.0), ("cycle", 100.0), ("event", 100.0)]
    best = policy.loc[policy["collision_free_completion_rate"].idxmax()]
    title = (f"Collision-free finishes out of 200: best is {PLANNER_LABEL[best['planner']]} "
             f"{_arm_label(best['replan_policy'], best['replan_period_ms'])} at "
             f"{_pct(best['collision_free_completion_rate'])}")
    subtitle = ("A run counts if every robot reached its goal without touching anything "
                "(coming within 30 mm is allowed). Comparable to 'success rate' in other work.")
    fig, ax = plt.subplots(figsize=(6.2, 3.1))
    width = 0.24
    for i, planner in enumerate(PLANNERS):
        for j, (pol, t) in enumerate(arms):
            row = _row(policy, planner, pol, t)
            if row is None:
                continue
            value = 100 * row["collision_free_completion_rate"]
            x = j + (i - 1) * width
            ax.bar(x, value, width=width, color=PLANNER_COLOR[planner], edgecolor=SURFACE,
                   linewidth=1.0, label=PLANNER_LABEL[planner] if j == 0 else None)
            ax.text(x, value, f"{value:.0f}", ha="center", va="bottom", fontsize=6.5, color=INK_2)
    ax.set_xticks(range(len(arms)), [f"{p.capitalize()}\n{t:g} ms" for p, t in arms])
    ax.tick_params(axis="x", length=0)
    ax.set_ylabel("Collision-free finishes (% of 200 runs)")
    ax.set_ylim(0, 100)
    _clean(ax, grid_axis="y")
    ax.legend(loc="upper left", bbox_to_anchor=(1.0, 1.0))
    free = _titled(fig, title, subtitle, "policy_collision_free")
    fig.tight_layout(rect=(0, 0, 1, free))
    _save(fig, out, "policy_collision_free")


def figure_tradeoff(policy: pd.DataFrame | None, out: Path) -> None:
    if policy is None:
        return
    ratios, deltas = [], []
    for p in PLANNERS:
        c, e = _row(policy, p, "cycle", 20.0), _row(policy, p, "event", 20.0)
        if c is not None and e is not None and e["replans_per_run"] > 0:
            ratios.append(c["replans_per_run"] / e["replans_per_run"])
            deltas.append(100 * (e["physical_rate"] - c["physical_rate"]))
    title = (f"Rebuilding every check costs {min(ratios):.0f}–{max(ratios):.0f}× more replans "
             f"than event triggering, yet contact differs by only "
             f"{min(deltas):+.0f} to {max(deltas):+.0f} points (20 ms checks)")
    subtitle = ("Each marker is one planner with one policy. Further left = less computing; "
                "lower = fewer runs that touched something.")
    fig, ax = plt.subplots(figsize=(5.8, 3.4))
    markers = {"cycle": "s", "event": "o"}
    for _, row in policy.iterrows():
        color = PLANNER_COLOR[row["planner"]]
        filled = row["replan_period_ms"] == 20.0
        ax.scatter(row["replans_per_run"], 100 * row["physical_rate"], s=46,
                   marker=markers.get(row["replan_policy"], "^"),
                   facecolor=color if filled else SURFACE, edgecolor=color, linewidth=1.5,
                   zorder=3)
    ax.set_xscale("log")
    ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))
    ax.set_xlabel("Successful replans per run (log scale)")
    ax.set_ylabel("Runs that touched something (% of finished runs)")
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
    free = _titled(fig, title, subtitle, "policy_compute_vs_safety")
    fig.tight_layout(rect=(0, 0, 1, free))
    _save(fig, out, "policy_compute_vs_safety")


def figure_latency(calls, policy, period: float, out: Path, extra_calls=None) -> None:
    if calls is None or calls.empty or policy is None:
        return
    data = pd.concat([c for c in (calls, extra_calls) if c is not None], ignore_index=True)
    data = data[(data.replan_period_ms == period) & (data.prediction_horizon_ms == 0.0)]
    policies = [p for p in ("cycle", "event", "event_route") if p in set(data.replan_policy)]
    fewer = []
    for p in PLANNERS:
        c, e = _row(policy, p, "cycle", period), _row(policy, p, "event", period)
        if c is not None and e is not None and e["successful_rebuilds"] > 0:
            fewer.append(c["successful_rebuilds"] / e["successful_rebuilds"])
    med = {(p, pol): data[(data.planner == p) & (data.replan_policy == pol)]["rebuild_ms"].median()
           for p in PLANNERS for pol in policies}
    title = (f"{period:g} ms checks: event triggering rebuilds {min(fewer):.0f}–{max(fewer):.0f}× "
             f"less often, and a single rebuild costs about the same "
             f"(e.g. PRM median {med[('prm', 'cycle')]:.0f} vs {med[('prm', 'event')]:.0f} ms)")
    subtitle = ("Each curve shows what share of successful rebuilds finished within x ms. "
                "Rebuilds past the 100 ms line stop the robot.")
    fig, axes = plt.subplots(1, len(PLANNERS), figsize=(7.2, 3.0), sharey=True)
    lo = max(data["rebuild_ms"].min(), 1e-2)
    hi = max(data["rebuild_ms"].max(), REPLAN_LIMIT_MS * 1.5)
    for ax, planner in zip(axes, PLANNERS):
        sub = data[data.planner == planner]
        for pol in policies:
            values = sub[sub.replan_policy == pol]["rebuild_ms"]
            over = (values > REPLAN_LIMIT_MS).sum()
            _ecdf(ax, values, POLICY_COLOR[pol],
                  f"{POLICY_LABEL[pol]}", lw=1.8)
            if len(values):
                ax.text(0.97, 0.05 + 0.09 * policies.index(pol),
                        f"{len(values):,} rebuilds, {over} over 100 ms", transform=ax.transAxes,
                        ha="right", fontsize=6.5,
                        color=POLICY_COLOR[pol] if pol != "cycle" else INK_2)
        ax.axvline(REPLAN_LIMIT_MS, color=INK_2, lw=0.8)
        ax.set_xscale("log")
        ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))
        ax.set_xlim(lo, hi)
        ax.set_ylim(0, 102)
        ax.set_title(PLANNER_LABEL[planner], loc="left")
        ax.set_xlabel("Time for one rebuild (ms, log)")
        _clean(ax, grid_axis="both")
    axes[0].set_ylabel("Rebuilds finished within x ms (%)")
    handles, labels = axes[0].get_legend_handles_labels()
    free = _titled(fig, title, subtitle, f"policy_replan_latency_{period:g}ms")
    fig.tight_layout(rect=(0, 0.10, 1, free))
    fig.legend(handles, labels, loc="lower center", ncol=len(policies), bbox_to_anchor=(0.5, 0.0))
    _save(fig, out, f"policy_replan_latency_{period:g}ms")


def figure_routes(policy: pd.DataFrame | None, out: Path) -> None:
    if policy is None:
        return
    arms = [("cycle", 20.0), ("event", 20.0), ("cycle", 100.0), ("event", 100.0)]
    ratios = []
    for p in PLANNERS:
        c, e = _row(policy, p, "cycle", 20.0), _row(policy, p, "event", 20.0)
        if c is not None and e is not None:
            ratios.append(e["route_lifetime_ms_mean"] / c["route_lifetime_ms_mean"])
    vc, ve = _row(policy, "voronoi", "cycle", 20.0), _row(policy, "voronoi", "event", 20.0)
    title = (f"With event triggering a route is kept {min(ratios):.1f}–{max(ratios):.1f}× longer "
             f"at 20 ms checks (Voronoi {vc['route_lifetime_ms_mean']:.0f} → "
             f"{ve['route_lifetime_ms_mean']:.0f} ms)")
    subtitle = "Average time a planned route is followed before it is replaced by a new one."
    fig, ax = plt.subplots(figsize=(6.2, 3.1))
    width = 0.24
    for i, planner in enumerate(PLANNERS):
        for j, (pol, t) in enumerate(arms):
            row = _row(policy, planner, pol, t)
            if row is None:
                continue
            value = float(row["route_lifetime_ms_mean"])
            x = j + (i - 1) * width
            ax.bar(x, value, width=width, color=PLANNER_COLOR[planner], edgecolor=SURFACE,
                   linewidth=1.0, label=PLANNER_LABEL[planner] if j == 0 else None)
            ax.text(x, value, f"{value:.0f}", ha="center", va="bottom", fontsize=6.5, color=INK_2)
    ax.set_xticks(range(len(arms)), [f"{p.capitalize()}\n{t:g} ms" for p, t in arms])
    ax.tick_params(axis="x", length=0)
    ax.set_ylabel("Average route lifetime (ms)")
    ax.set_ylim(0, float(policy["route_lifetime_ms_mean"].max()) * 1.15)
    _clean(ax, grid_axis="y")
    ax.legend(loc="upper left", bbox_to_anchor=(1.0, 1.0))
    free = _titled(fig, title, subtitle, "policy_route_lifetime")
    fig.tight_layout(rect=(0, 0, 1, free))
    _save(fig, out, "policy_route_lifetime")


# --- Prediction Horizon Sweep -------------------------------------------------


def _horizon_label(r) -> str:
    h = r["prediction_horizon_ms"]
    return "No prediction (0 ms)" if h == 0 else f"Horizon {h:g} ms"


def figure_horizon_outcomes(policy, sweep, out: Path) -> None:
    if sweep is None:
        return
    rows = _horizon_rows(policy, sweep)
    rows["_sub"] = rows["prediction_horizon_ms"]
    first, last = rows["prediction_horizon_ms"].min(), rows["prediction_horizon_ms"].max()

    def at(p, h, col):
        sel = rows[(rows.planner == p) & (rows.prediction_horizon_ms == h)]
        return int(sel.iloc[0][col]) if len(sel) else 0

    title = (f"Looking further ahead helps Voronoi (clean finishes {at('voronoi', first, 'strict')} "
             f"→ {at('voronoi', last, 'strict')}) but not PRM or Visibility Graph "
             f"(unfinished {at('prm', first, 'incomplete')} → {at('prm', last, 'incomplete')} and "
             f"{at('visibility', first, 'incomplete')} → {at('visibility', last, 'incomplete')})")
    subtitle = ("Horizon = how far ahead obstacles' motion is projected when planning. "
                "Event trigger, 20 ms checks; each bar is the same 200 scenarios.")
    _outcome_figure(rows, _horizon_label, title, subtitle, out, "horizon_outcomes")


def _trend_panels(axes, rows) -> None:
    panels = [
        ("strict_rate", "Clean finishes (% of finished runs)", 100.0),
        ("physical_rate", "Runs that touched something (% of finished runs)", 100.0),
        ("incomplete_rate", "Runs that did not finish (% of 200)", 100.0),
        ("replans_per_run", "Successful replans per run", 1.0),
    ]
    horizons = sorted(rows["prediction_horizon_ms"].unique())
    for ax, (column, title, scale) in zip(axes.flat, panels):
        for planner in PLANNERS:
            sub = rows[rows.planner == planner].sort_values("prediction_horizon_ms")
            if sub.empty:
                continue
            ax.plot(sub["prediction_horizon_ms"], scale * sub[column], color=PLANNER_COLOR[planner],
                    lw=2, marker="o", ms=6.5, mec=SURFACE, mew=1.4, label=PLANNER_LABEL[planner])
            last = sub.iloc[-1]
            value = scale * last[column]
            ax.annotate(f"{value:.0f}" if scale > 1 else f"{value:.1f}",
                        (last["prediction_horizon_ms"], value), xytext=(7, 0),
                        textcoords="offset points", va="center", fontsize=7, color=INK_2)
        ax.set_xticks(horizons, [("0\n(none)" if h == 0 else f"{h:g}") for h in horizons])
        ax.set_xlim(min(horizons) - 5, max(horizons) + 14)
        ax.set_xlabel("Prediction horizon (ms)")
        ax.set_title(title, loc="left")
        ax.set_ylim(bottom=0)
        ax.yaxis.set_major_locator(MaxNLocator(nbins=6, integer=True))
        _clean(ax, grid_axis="y")


def figure_horizon(policy, sweep, out: Path) -> None:
    if sweep is None:
        return
    rows = _horizon_rows(policy, sweep)
    first, last = rows["prediction_horizon_ms"].min(), rows["prediction_horizon_ms"].max()

    def val(p, h, col, scale=100.0):
        sel = rows[(rows.planner == p) & (rows.prediction_horizon_ms == h)]
        return scale * float(sel.iloc[0][col]) if len(sel) else float("nan")

    title = (f"Prediction trades computing for safety only for Voronoi: clean finishes "
             f"{val('voronoi', first, 'strict_rate'):.0f}% → {val('voronoi', last, 'strict_rate'):.0f}%, "
             f"replans {val('voronoi', first, 'replans_per_run', 1):.0f} → "
             f"{val('voronoi', last, 'replans_per_run', 1):.0f} per run")
    subtitle = ("Horizon = how far ahead (ms) obstacles are projected; 0 = no prediction. "
                "Event trigger, 20 ms checks, 200 scenarios per point.")
    fig, axes = plt.subplots(2, 2, figsize=(7.2, 6.0))
    _trend_panels(axes, rows)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    free = _titled(fig, title, subtitle, "horizon_trends")
    fig.tight_layout(rect=(0, 0.06, 1, free))
    fig.subplots_adjust(hspace=0.55, wspace=0.28)
    fig.legend(handles, labels, loc="lower center", ncol=3, bbox_to_anchor=(0.5, 0.0))
    _save(fig, out, "horizon_trends")


# --- Supporting sets -------------------------------------------------------------


def figure_event_route(policy, ablation, out: Path) -> None:
    if policy is None or ablation is None:
        return
    base = policy[policy.replan_policy == "event"]
    rows = pd.concat([base, ablation], ignore_index=True)
    rows["_sub"] = rows["replan_period_ms"] * 10 + (rows["replan_policy"] == "event_route")
    diffs = []
    for p in PLANNERS:
        e, r = _row(rows, p, "event", 20.0), _row(rows, p, "event_route", 20.0)
        if e is not None and r is not None:
            diffs.append((p, int(r["completed"]) - int(e["completed"]),
                          int(r["strict"]) - int(e["strict"]),
                          r["replans_per_run"] / max(e["replans_per_run"], 1e-9)))
    lost = ", ".join(f"{PLANNER_LABEL[p]} {d:+d}" for p, d, _, _ in diffs)
    clean = ", ".join(f"{s:+d}" for _, _, s, _ in diffs)
    title = (f"Recalculating whenever any later segment is blocked (full-path) costs "
             f"{min(x for *_, x in diffs):.1f}–{max(x for *_, x in diffs):.1f}× the replans at 20 ms "
             f"and changes finishes by {lost}, with clean finishes {clean}: "
             f"more work, no safety gain")
    subtitle = ("Event rebuilds when the segment to the next waypoint is blocked; full-path "
                "also rebuilds when any later segment is blocked, the rule Costa and Tonidandel (2024) describe "
                "for SSL planners without a separate avoidance layer. Same scenarios for both.")
    _outcome_figure(rows, lambda r: _arm_label(r["replan_policy"], r["replan_period_ms"]),
                    title, subtitle, out, "ablation_event_route")


# Best / worst highlighting for table figures: blue vs orange washes (a colour-blind-safe
# pair that also differs in lightness) plus a glyph, so the mark never relies on colour.
BEST_BG, WORST_BG = "#cfe1f7", "#fbd3bd"
BEST_MARK, WORST_MARK = "\u25b2", "\u25bc"


def _best_worst(values, higher_better: bool, min_gap) -> tuple[set, set]:
    """Indices of the best and worst values; empty when the spread is below ``min_gap``."""
    live = [(i, v) for i, v in enumerate(values) if v is not None and v == v]
    if len(live) < 2:
        return set(), set()
    vs = [v for _, v in live]
    lo, hi = min(vs), max(vs)
    if hi - lo < min_gap(lo, hi):
        return set(), set()
    best_v, worst_v = (hi, lo) if higher_better else (lo, hi)
    best = {i for i, v in live if abs(v - best_v) < 1e-9}
    worst = {i for i, v in live if abs(v - worst_v) < 1e-9}
    return best, worst


def _mark_cell(ax, x, y, col_w, row_h, kind: str) -> None:
    ax.add_patch(plt.Rectangle((x + 0.003, y + 0.02), col_w - 0.004, row_h - 0.04,
                               color=BEST_BG if kind == "best" else WORST_BG, lw=0))
    ax.text(x + col_w - 0.004, y + row_h / 2, BEST_MARK if kind == "best" else WORST_MARK,
            ha="right", va="center", fontsize=5.4, color=INK_2)


def table_event_route(policy_runs, ablation_runs, calls, ablation_calls, out: Path) -> None:
    """Event vs full-path vs cycle as a paper-ready table (PDF + PNG), built from runs.csv.

    Also splits unfinished runs by cause: a replan over 100 ms (computing time,
    hardware-dependent) or 1000 ms without a route (the obstacles left no path).
    """
    if policy_runs is None or ablation_runs is None:
        return
    runs = pd.concat([policy_runs[policy_runs.prediction_horizon_ms == 0.0], ablation_runs])
    if "outcome" in runs:
        runs = runs[runs.outcome != "X"]
    runs = runs.assign(
        compute=runs.rebuild_ms_total.fillna(0) + runs.event_check_ms_total.fillna(0),
        dnf_route=(runs.outcome == "I") & (runs.episode_end_reason.fillna("") == "no_valid_path"),
    )
    runs = runs.assign(dnf_slow=(runs.outcome == "I") & ~runs.dnf_route)
    keys = ["planner", "replan_period_ms", "replan_policy"]
    g = runs.groupby(keys)
    t = pd.DataFrame({
        "finished": g.outcome.apply(lambda o: int((o != "I").sum())),
        "cf": g.outcome.apply(lambda o: int(o.isin(["S", "B"]).sum())),
        "rebuilds": g.replan_count.mean(),
        "later": g.replans_route_blocked.mean(),
        "compute": g.compute.mean(),
        "dnf_slow": g.dnf_slow.sum(),
        "dnf_route": g.dnf_route.sum(),
        "life": g.route_lifetime_ms_total.sum() / g.route_count.sum(),
    })
    if calls is not None and ablation_calls is not None:
        rc = pd.concat([calls, ablation_calls])
        rc = rc[rc.prediction_horizon_ms == 0.0]
        t["median"] = rc.groupby(keys).rebuild_ms.median()
    else:
        t["median"] = float("nan")
    t = t.reset_index()
    # (column, header, format, higher_is_better)
    cols = [
        ("finished", "Runs\nfinished\n(of 200)", "{:.0f}", True),
        ("cf", "Finished\nwithout contact\n(of 200)", "{:.0f}", True),
        ("rebuilds", "Rebuilds\nper run", "{:.1f}", False),
        ("later", "Rebuilds from\nblocked later\nsegment (/run)", "{:.1f}", False),
        ("compute", "Planning\ntime per\nrun (ms)", "{:.0f}", False),
        ("median", "Time per\nrebuild,\nmedian (ms)", "{:.1f}", False),
        ("dnf_slow", "Failed runs:\nrebuild over\n100 ms", "{:.0f}", False),
        ("dnf_route", "Failed runs:\nno route\nfor 1 s", "{:.0f}", False),
        ("life", "Time a\nroute is\nkept (ms)", "{:.0f}", True),
    ]
    arms = (("event", "Event (ours)"), ("event_route", "Full-path"), ("cycle", "Cycle"))
    blocks = []
    for p in PLANNERS:
        for period in (20.0, 100.0):
            rows = []
            for pol, name in arms:
                r = t[(t.planner == p) & (t.replan_period_ms == period) & (t.replan_policy == pol)]
                if not r.empty:
                    rows.append((pol, name, r.iloc[0]))
            if rows and rows[0][0] == "event":
                blocks.append((p, period, rows))
    if not blocks:
        return

    def get(p, period, pol, key):
        r = t[(t.planner == p) & (t.replan_period_ms == period) & (t.replan_policy == pol)]
        return r.iloc[0][key] if not r.empty else float("nan")

    slow = {pol: int(t[t.replan_policy == pol].dnf_slow.sum()) for pol, _ in arms}
    route = {pol: int(t[t.replan_policy == pol].dnf_route.sum()) for pol, _ in arms}
    vis = [get("visibility", 20.0, pol, "rebuilds") for pol in ("event", "event_route", "cycle")]
    finding = (f"Rebuilding more often costs more and finishes fewer runs: at 20 ms checks Visibility Graph "
               f"rebuilds {vis[0]:.1f} (event), {vis[1]:.1f} (full-path) and {vis[2]:.1f} (cycle) times per run. "
               f"Across all planners, failed runs from a rebuild over 100 ms (computing time): event {slow['event']}, "
               f"full-path {slow['event_route']}, cycle {slow['cycle']}; failed runs because no route existed for "
               f"1 s (obstacles blocked every path): {route['event']}, {route['event_route']}, "
               f"{route['cycle']}")
    subtitle = (f"{finding}. Event (ours) rebuilds when the next segment is blocked; full-path also when any "
                "later segment is (Costa and Tonidandel 2024); cycle rebuilds at every check. Brackets show the "
                "change from event. In each group of three, \u25b2 blue (bold) marks the best value and "
                "\u25bc orange the worst; planning time and time per rebuild are marked only when they differ "
                "by more than 20 %, the drift between batches. The rebuild-over-100 ms column depends on the "
                "machine; the no-route column does not.")

    n_rows = sum(len(rows) for _, _, rows in blocks)
    row_h, head_h, gap = 0.24, 0.62, 0.25
    total_h = head_h + n_rows * row_h + gap * len(PLANNERS)
    fig_h = 1.45 + total_h
    fig = plt.figure(figsize=(8.8, fig_h))
    free = _titled(fig, "Event vs full-path vs cycle replanning for three planners",
                   subtitle, "table_event_route")
    ax = fig.add_axes((0.01, max(free - total_h / fig_h, 0.0), 0.98, min(total_h / fig_h, free)))
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(total_h, 0)
    x0, width = 0.16, 0.84
    col_w = width / len(cols)
    x_cols = [x0 + i * col_w for i in range(len(cols))]
    for (_, head, _, _), x in zip(cols, x_cols):
        ax.text(x + col_w - 0.019, head_h - 0.06, head, ha="right", va="bottom", fontsize=6.2,
                color=INK_2, linespacing=1.15)
    ax.plot([0, 1], [head_h, head_h], color=AXIS, lw=0.8)
    y = head_h

    def gap_for(key):
        if key in ("finished", "cf", "dnf_slow", "dnf_route"):
            return lambda lo, hi: 1
        if key in ("compute", "median"):
            return lambda lo, hi: 0.2 * max(abs(lo), 1e-9)
        if key == "life":
            return lambda lo, hi: 0.05 * max(abs(lo), 1e-9)
        return lambda lo, hi: max(0.5, 0.05 * max(abs(lo), 1e-9))

    current = None
    for p, period, rows in blocks:
        if p != current:
            current = p
            y += gap
            ax.text(0.0, y - 0.05, PLANNER_LABEL[p], ha="left", va="bottom", fontsize=7.6,
                    fontweight="bold", color=INK)
        ref = rows[0][2]
        marks = {}
        for key, _, _, higher_better in cols:
            vals = [None if (key == "later" and pol == "cycle") else row[key] for pol, _, row in rows]
            marks[key] = _best_worst(vals, higher_better, gap_for(key))
        for idx, (pol, name, row) in enumerate(rows):
            ax.text(0.012, y + row_h / 2, f"{name} @ {period:g} ms", ha="left", va="center",
                    fontsize=6.8, color=INK if pol == "event" else INK_2)
            for (key, _, fmt, higher_better), x in zip(cols, x_cols):
                if key == "later" and pol == "cycle":
                    ax.text(x + col_w - 0.019, y + row_h / 2, "–", ha="right", va="center",
                            fontsize=6.8, color=MUTED)
                    continue
                value = row[key]
                text = fmt.format(value)
                if pol != "event":
                    delta = value - ref[key]
                    if abs(delta) >= (1 if fmt == "{:.0f}" else 0.05):
                        sign = "+" if delta > 0 else "−"
                        text += f" ({sign}{fmt.format(abs(delta))})"
                best, worst = marks[key]
                if idx in best:
                    _mark_cell(ax, x, y, col_w, row_h, "best")
                elif idx in worst:
                    _mark_cell(ax, x, y, col_w, row_h, "worst")
                ax.text(x + col_w - 0.019, y + row_h / 2, text, ha="right", va="center", fontsize=6.8,
                        color=INK, fontweight="bold" if idx in best else "normal")
            y += row_h
        ax.plot([0.012, 1], [y, y], color=GRID, lw=0.5)
    _save(fig, out, "table_event_route")

def table_rebuild_latency(calls, policy_runs, out: Path) -> None:
    """Rebuild time percentiles, tail events and robots stopped, cycle vs event (PDF + PNG)."""
    if calls is None or policy_runs is None:
        return
    keys = ["planner", "replan_period_ms", "replan_policy"]
    rc = calls[calls.prediction_horizon_ms == 0.0]
    q = rc.groupby(keys).rebuild_ms
    runs = policy_runs[policy_runs.prediction_horizon_ms == 0.0]
    g = runs.groupby(keys)
    t = pd.DataFrame({
        "n": q.size(), "p50": q.median(), "p95": q.quantile(0.95), "p99": q.quantile(0.99),
        "max": q.max(), "over": q.apply(lambda x: int((x > REPLAN_LIMIT_MS).sum())),
        "per_run": g.successful_rebuilds.mean(),
        "search": 100 * g.rebuild_search_ms_total.sum() / g.rebuild_ms_total.sum(),
        "stops": g.robots_stopped_time_limit.sum(),
    })
    t["rate"] = 1e4 * t.over / t.n
    t = t.reset_index()
    cols = [
        ("per_run", "Rebuilds\nper run", "{:.1f}"),
        ("p50", "Time per\nrebuild,\nmedian (ms)", "{:.1f}"),
        ("p95", "95 % of\nrebuilds\nunder (ms)", "{:.1f}"),
        ("p99", "99 % of\nrebuilds\nunder (ms)", "{:.1f}"),
        ("max", "Slowest\nrebuild\n(ms)", "{:.0f}"),
        ("rate", "Rebuilds over\n100 ms per\n10,000", "{:.0f}"),
        ("search", "Share of time\nin Dijkstra\nsearch (%)", "{:.0f}"),
        ("stops", "Robots\nstopped by\n100 ms limit", "{:.0f}"),
    ]
    rows = []
    for p in PLANNERS:
        for period in (20.0, 100.0):
            for pol in ("cycle", "event"):
                r = t[(t.planner == p) & (t.replan_period_ms == period) & (t.replan_policy == pol)]
                if not r.empty:
                    rows.append((p, period, pol, r.iloc[0]))
    if not rows:
        return

    def cell(p, per, pol, key):
        for rp, rper, rpol, r in rows:
            if (rp, rper, rpol) == (p, per, pol):
                return r[key]
        return float("nan")

    p95_lo, p95_hi = t.p95.min(), t.p95.max()
    rate_hi = t.rate.max()
    stops = ", ".join(
        f"{PLANNER_LABEL[p]} {cell(p, 20.0, 'cycle', 'stops'):.0f} vs {cell(p, 20.0, 'event', 'stops'):.0f}"
        for p in PLANNERS)
    subtitle = (f"A typical rebuild is far below the 100 ms limit (95 % of rebuilds finish within {p95_lo:.0f}–{p95_hi:.0f} ms), "
                f"and rebuilds over 100 ms are rare (at most {rate_hi:.0f} per 10,000). Each rebuild is a new "
                f"chance of one, so rebuilding less often stops fewer robots: at 20 ms checks, cycle vs event "
                f"{stops}. Rebuild time is mostly map construction; the Dijkstra search is a few percent. "
                "Times are wall clock on one machine, successful rebuilds only. \u25b2 blue (bold) = better of the "
                "pair, \u25bc orange = worse; times marked only when they differ by 10 % or more.")

    row_h, head_h, gap = 0.24, 0.62, 0.25
    total_h = head_h + len(rows) * row_h + gap * len(PLANNERS)
    fig_h = 1.3 + total_h
    fig = plt.figure(figsize=(8.0, fig_h))
    free = _titled(fig, "Cycle vs event rebuild time for three planners", subtitle, "table_rebuild_latency")
    ax = fig.add_axes((0.01, max(free - total_h / fig_h, 0.0), 0.98, min(total_h / fig_h, free)))
    ax.set_axis_off()
    ax.set_xlim(0, 1)
    ax.set_ylim(total_h, 0)
    x0, width = 0.22, 0.78
    col_w = width / len(cols)
    x_cols = [x0 + i * col_w for i in range(len(cols))]
    for (_, head, _), x in zip(cols, x_cols):
        ax.text(x + col_w - 0.02, head_h - 0.06, head, ha="right", va="bottom", fontsize=6.4,
                color=INK_2, linespacing=1.15)
    ax.plot([0, 1], [head_h, head_h], color=AXIS, lw=0.8)
    y, current = head_h, None
    lower_better = {"per_run", "p50", "p95", "p99", "max", "rate", "stops"}

    def lat_gap(key):
        if key in ("stops",):
            return lambda lo, hi: 1
        return lambda lo, hi: 0.1 * max(abs(lo), 1e-9)

    pairs = {}
    for p, period, pol, r in rows:
        pairs.setdefault((p, period), []).append((pol, r))
    marks = {}
    for grp, members in pairs.items():
        for key, _, _ in cols:
            if key in lower_better:
                b, w = _best_worst([r[key] for _, r in members], False, lat_gap(key))
                for i, (pol, _) in enumerate(members):
                    marks[(grp, pol, key)] = "best" if i in b else "worst" if i in w else None
    for p, period, pol, r in rows:
        if p != current:
            if current is not None:
                ax.plot([0.012, 1], [y, y], color=GRID, lw=0.5)
            current = p
            y += gap
            ax.text(0.0, y - 0.05, PLANNER_LABEL[p], ha="left", va="bottom", fontsize=7.6,
                    fontweight="bold", color=INK)
        label = f"{'Event (ours)' if pol == 'event' else 'Cycle'} @ {period:g} ms"
        ax.text(0.012, y + row_h / 2, label, ha="left", va="center", fontsize=6.9,
                color=INK if pol == "event" else INK_2)
        for (key, _, fmt), x in zip(cols, x_cols):
            kind = marks.get(((p, period), pol, key))
            if kind:
                _mark_cell(ax, x, y, col_w, row_h, kind)
            ax.text(x + col_w - 0.02, y + row_h / 2, fmt.format(r[key]), ha="right", va="center",
                    fontsize=6.9, color=INK, fontweight="bold" if kind == "best" else "normal")
        y += row_h
    _save(fig, out, "table_rebuild_latency")


def figure_horizon_150(policy, sweep, check150, out: Path) -> None:
    if sweep is None or check150 is None:
        return
    rows = _horizon_rows(policy, sweep, check150)
    rows["_sub"] = rows["prediction_horizon_ms"]

    def inc(p, h):
        sel = rows[(rows.planner == p) & (rows.prediction_horizon_ms == h)]
        return int(sel.iloc[0]["incomplete"]) if len(sel) else 0

    title = (f"Why the sweep stops at 100 ms: at 150 ms PRM and Visibility Graph leave "
             f"{inc('prm', 150.0)} and {inc('visibility', 150.0)} of 200 runs unfinished "
             f"(vs {inc('prm', 100.0)} and {inc('visibility', 100.0)} at 100 ms)")
    title = title.replace("of 200", f"of {int(rows['valid_runs'].max())}")
    subtitle = ("Projected obstacles grow with the horizon until they block the routes; "
                "150 ms then measures blocked routes, not the value of prediction.")

    def label(r):
        h = r["prediction_horizon_ms"]
        return f"Horizon {h:g} ms (not used)" if h == 150 else _horizon_label(r)

    _outcome_figure(rows, label, title, subtitle, out, "evidence_horizon_150")


def figure_no_route(policy, check500, out: Path) -> None:
    if policy is None or check500 is None:
        return
    main = policy[(policy.replan_policy == "event") & (policy.replan_period_ms == 20.0)].copy()
    main["_limit"] = 1000
    alt = check500.copy()
    alt["_limit"] = 500
    rows = pd.concat([alt, main], ignore_index=True)
    rows["_sub"] = rows["_limit"]
    parts = []
    for p in PLANNERS:
        a, b = rows[(rows.planner == p) & (rows._limit == 500)], rows[(rows.planner == p) & (rows._limit == 1000)]
        if len(a) and len(b):
            parts.append(f"{PLANNER_LABEL[p]} {int(a.iloc[0]['completed'])} → {int(b.iloc[0]['completed'])}")
    title = ("Allowing 1000 ms instead of 500 ms without a route: finished runs "
             + ", ".join(parts) + f" of {int(rows['valid_runs'].max())}; "
             "most recovered runs still touch something")
    subtitle = ("A blocking obstacle needs up to ~0.7 s to move clear at the slowest bank speed "
                "(300 mm/s), so 500 ms failed robots that would have got through. Event, 20 ms.")
    _outcome_figure(rows, lambda r: f"Limit {int(r['_limit'])} ms", title, subtitle, out,
                    "evidence_no_route_limit")


def main() -> int:
    parser = argparse.ArgumentParser(description="Build the ACRA 2026 paper figures.")
    parser.add_argument("root", type=Path, nargs="?", default=Path("results/acra2026-final"))
    parser.add_argument(
        "--paper", action="store_true",
        help="omit in-figure titles and subtitles (captions live in LaTeX); writes to figures/paper/",
    )
    args = parser.parse_args()
    global PAPER
    PAPER = args.paper
    root = args.root
    robots = _read(root / "one-shot-validation" / "robots.csv")
    policy = _read(root / "policy-comparison" / "summary.csv")
    sweep = _read(root / "horizon-sweep" / "summary.csv")
    calls = _read(root / "policy-comparison" / "rebuild_calls.csv")
    support = root / "supporting"
    ablation = _read(support / "event-route-ablation" / "summary.csv")
    policy_runs = _read(root / "policy-comparison" / "runs.csv")
    ablation_runs = _read(support / "event-route-ablation" / "runs.csv")
    ablation_calls = _read(support / "event-route-ablation" / "rebuild_calls.csv")
    check150 = _read(support / "horizon-150-check" / "summary.csv")
    check500 = _read(support / "no-route-500-check" / "summary.csv")
    out = root / "figures" / ("paper" if PAPER else "")
    print(f"Writing figures to {out}")
    figure_one_shot(robots, out)
    figure_policy_outcomes(policy, out)
    figure_collision_free(policy, out)
    figure_tradeoff(policy, out)
    for period in (20.0, 100.0):
        figure_latency(calls, policy, period, out)
    figure_routes(policy, out)
    figure_horizon_outcomes(policy, sweep, out)
    figure_horizon(policy, sweep, out)
    figure_event_route(policy, ablation, out)
    table_event_route(policy_runs, ablation_runs, calls, ablation_calls, out)
    table_rebuild_latency(calls, policy_runs, out)
    figure_horizon_150(policy, sweep, check150, out)
    figure_no_route(policy, check500, out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
