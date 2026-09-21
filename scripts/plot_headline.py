"""Three headline figures, one per metric group.

    fig_collision_paired.png   paired cycle-vs-event differences with 95% CIs
    fig_replan_scaling.png     replans vs replan period, log-log
    fig_overall_timetogoal.png median time to goal, cycle vs event

Each answers the question its group exists to answer, which a heatmap cannot:

  collision  the paper's claim is a NULL ("no safety difference"). A null needs
             effect size and uncertainty, not two similar-looking colours. A
             forest plot shows the differences straddling zero AND how large an
             effect could have been detected.
  replan     cycle's cost is mechanically set by the loop rate; event's is set
             by how dynamic the scene is. On log-log that is a slope of about
             -1 against a flat line -- the mechanism, not a ranking.
  overall    event triggering arrives LATER. That is the cost the safety numbers
             hide, and the paper should show it rather than let a reviewer find it.

Usage:
    python scripts/plot_headline.py results/acra-6v6-canonical
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

Z = 1.959964
PLANNERS = ["voronoi", "prm", "visibility"]
LABEL = {"voronoi": "Voronoi", "prm": "PRM", "visibility": "Visibility graph"}
INK, MUTED, GRID = "#1f2733", "#5b6472", "#e3e6ea"
C_CYCLE, C_EVENT, C_SIG = "#d55c25", "#256abf", "#0d366b"


def load(folder: Path) -> pd.DataFrame:
    r = pd.read_csv(folder / "runs.csv")
    if "obstacle_episodes_robot_initiated" in r:
        r["collision_free"] = ((r["obstacle_episodes_robot_initiated"] == 0)
                               & (r["robot_collision_episodes"] == 0))
    else:
        r["collision_free"] = r["collision_episodes"] == 0
        print("  ! no attribution columns — collision figure is uncorrected")
    r["replan_period_ms"] = r["replan_period_ms"].round(3)
    failed = r.loc[r["status"] == "planning_failed", ["scenario", "planner"]]
    bad = set(map(tuple, failed.drop_duplicates().values))
    return r[[(s, p) not in bad for s, p in zip(r.scenario, r.planner)]].copy()


def _tidy(ax):
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=9, length=3)


# ---------------------------------------------------------------- collision
def collision_paired(r: pd.DataFrame, out: Path) -> None:
    """Forest plot of paired (cycle - event) differences, in percentage points."""
    rows = []
    for pl in [p for p in PLANNERS if p in r.planner.values]:
        for h in sorted(r.prediction_horizon_ms.unique()):
            for p in sorted(r.replan_period_ms.unique()):
                m = (r.planner == pl) & (r.prediction_horizon_ms == h) & (r.replan_period_ms == p)
                cy = r[m & (r.replan_policy == "cycle")].set_index("scenario").collision_free
                ev = r[m & (r.replan_policy == "event")].set_index("scenario").collision_free
                common = cy.index.intersection(ev.index)
                if len(common) < 10:
                    continue
                cy, ev = cy.loc[common], ev.loc[common]
                n = len(common)
                b = int((cy & ~ev).sum())      # cycle safe, event not
                c = int((~cy & ev).sum())      # event safe, cycle not
                diff = 100 * (cy.mean() - ev.mean())
                se = 100 * np.sqrt(max(b + c - (b - c) ** 2 / n, 0)) / n
                rows.append(dict(planner=pl, h=h, p=p, n=n, diff=diff,
                                 lo=diff - Z * se, hi=diff + Z * se,
                                 sig=(diff - Z * se) * (diff + Z * se) > 0))
    t = pd.DataFrame(rows)
    if t.empty:
        print("  collision: no paired cells"); return
    t = t.sort_values(["planner", "h", "p"], ascending=[True, False, False]).reset_index(drop=True)

    fig, ax = plt.subplots(figsize=(7.4, 0.32 * len(t) + 1.9))
    ax.axvline(0, color=INK, lw=1.2, zorder=1)
    y = np.arange(len(t))
    for i, row in t.iterrows():
        col = C_SIG if row.sig else MUTED
        ax.plot([row.lo, row.hi], [i, i], color=col, lw=2, alpha=0.55, zorder=2,
                solid_capstyle="round")
        ax.plot(row["diff"], i, "o", color=col, ms=6, zorder=3)
    ax.set_yticks(y, [f"{LABEL[r.planner]}  h{r.h:g} p{r.p:g}  (n={r.n})" for r in t.itertuples()])
    ax.invert_yaxis()
    ax.set_title("Paired cycle-vs-event safety difference, 95% CI",
                 fontsize=12, color=INK, loc="left", pad=10)
    span = max(abs(t.lo.min()), abs(t.hi.max())) * 1.12
    ax.set_xlim(-span, span)
    ax.set_ylim(len(t) - 0.4, -1.3)
    ax.text(span * 0.98, -1.1, "favours cycle →", ha="right", va="center",
            fontsize=8.5, color=MUTED)
    ax.text(-span * 0.98, -1.1, "← favours event", ha="left", va="center",
            fontsize=8.5, color=MUTED)
    ax.grid(axis="x", color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    _tidy(ax)
    n_sig = int(t.sig.sum())
    ax.set_xlabel(
        "Collision-free difference, cycle − event (percentage points)\n"
        f"{len(t) - n_sig} of {len(t)} cells include zero"
        f"{'' if n_sig == 0 else f'; {n_sig} do not'}",
        fontsize=10, color=MUTED)
    fig.savefig(out.with_suffix(".png"), dpi=200, bbox_inches="tight", facecolor="white")
    fig.savefig(out.with_suffix(".pdf"), bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"  wrote {out.name}.png  ({len(t)} cells, {n_sig} excluding zero)")


# ------------------------------------------------------------------- replan
def replan_scaling(r: pd.DataFrame, out: Path) -> None:
    """Replans per run against replan period, log-log, cycle vs event."""
    planners = [p for p in PLANNERS if p in r.planner.values]
    periods = sorted(r.replan_period_ms.unique())
    if len(periods) < 2:
        print("  replan: needs >= 2 replan periods"); return
    h0 = 0.0 if (r.prediction_horizon_ms == 0).any() else sorted(r.prediction_horizon_ms.unique())[0]

    fig, axes = plt.subplots(1, len(planners), figsize=(3.5 * len(planners), 3.5),
                             sharey=True, squeeze=False)
    for k, pl in enumerate(planners):
        ax = axes[0][k]
        for pol, col in (("cycle", C_CYCLE), ("event", C_EVENT)):
            m = (r.planner == pl) & (r.replan_policy == pol) & (r.prediction_horizon_ms == h0)
            g = r[m].groupby("replan_period_ms").replan_count.median()
            if g.empty:
                continue
            ax.plot(g.index, g.values.clip(min=0.5), "o-", color=col, lw=2, ms=7,
                    label=pol.capitalize(), zorder=3)
            if len(g) >= 2:
                sl = np.polyfit(np.log10(g.index), np.log10(g.values.clip(min=0.5)), 1)[0]
                ax.annotate(f"slope {sl:+.2f}", xy=(g.index[-1], g.values[-1]),
                            xytext=(-6, -16 if pol == "cycle" else 10),
                            textcoords="offset points", fontsize=8.5, color=col,
                            ha="right", fontweight="bold")
        ax.set_xscale("log"); ax.set_yscale("log")
        ax.set_title(LABEL[pl], fontsize=11, color=INK)
        ax.set_xlabel("Replan period (ms)", fontsize=10, color=MUTED)
        ax.set_xticks(periods, [f"{p:g}" for p in periods])
        ax.set_xticks([], minor=True)
        ax.xaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        ax.grid(color=GRID, lw=0.8, which="both")
        ax.set_axisbelow(True)
        _tidy(ax)
        if k == 0:
            ax.set_ylabel("Replans per run (median)", fontsize=10, color=MUTED)
            ax.legend(frameon=False, fontsize=9, labelcolor=MUTED)
    fig.suptitle("Cycle cost is set by the loop rate; event cost is set by the scene",
                 fontsize=12, color=INK, x=0.005, ha="left", y=1.04)
    fig.savefig(out.with_suffix(".png"), dpi=200, bbox_inches="tight", facecolor="white")
    fig.savefig(out.with_suffix(".pdf"), bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("  wrote", out.name + ".png")


# ------------------------------------------------------------------ overall
def time_to_goal(r: pd.DataFrame, out: Path) -> None:
    """Dumbbell: median time to goal, cycle vs event, per planner x horizon."""
    base = r.replan_period_ms.min()
    rows = []
    for pl in [p for p in PLANNERS if p in r.planner.values]:
        for h in sorted(r.prediction_horizon_ms.unique()):
            m = (r.planner == pl) & (r.prediction_horizon_ms == h) & (r.replan_period_ms == base)
            cy = r[m & (r.replan_policy == "cycle")].time_to_goal_ms.median() / 1000
            ev = r[m & (r.replan_policy == "event")].time_to_goal_ms.median() / 1000
            if np.isnan(cy) or np.isnan(ev):
                continue
            rows.append(dict(planner=pl, h=h, cycle=cy, event=ev, delta=ev - cy))
    t = pd.DataFrame(rows)
    if t.empty:
        print("  overall: nothing to plot"); return
    t = t.sort_values(["planner", "h"], ascending=[True, False]).reset_index(drop=True)

    fig, ax = plt.subplots(figsize=(7.0, 0.36 * len(t) + 1.9))
    for i, row in t.iterrows():
        ax.plot([row.cycle, row.event], [i, i], color=GRID, lw=3,
                solid_capstyle="round", zorder=1)
        ax.plot(row.cycle, i, "o", color=C_CYCLE, ms=7, zorder=3)
        ax.plot(row.event, i, "o", color=C_EVENT, ms=7, zorder=3)
        ax.annotate(f"{row.delta:+.2f} s", xy=(max(row.cycle, row.event), i),
                    xytext=(9, 0), textcoords="offset points", va="center",
                    fontsize=8.5, color=C_EVENT if row.delta > 0 else C_CYCLE)
    ax.set_yticks(range(len(t)), [f"{LABEL[r.planner]}  horizon {r.h:g} ms" for r in t.itertuples()])
    ax.invert_yaxis()
    ax.set_xlabel(f"Median time to goal (s), at a {base:g} ms replan period",
                  fontsize=10, color=MUTED)
    ax.set_title("What the safety numbers hide: event triggering arrives later",
                 fontsize=12, color=INK, loc="left", pad=10)
    ax.plot([], [], "o", color=C_CYCLE, label="Cycle")
    ax.plot([], [], "o", color=C_EVENT, label="Event")
    ax.legend(frameon=False, fontsize=9, labelcolor=MUTED, loc="lower right")
    ax.grid(axis="x", color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    _tidy(ax)
    fig.savefig(out.with_suffix(".png"), dpi=200, bbox_inches="tight", facecolor="white")
    fig.savefig(out.with_suffix(".pdf"), bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("  wrote", out.name + ".png")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("batch", type=Path)
    a = ap.parse_args()
    out = a.batch / "report"
    out.mkdir(parents=True, exist_ok=True)
    r = load(a.batch)
    collision_paired(r, out / "fig_collision_paired")
    replan_scaling(r, out / "fig_replan_scaling")
    time_to_goal(r, out / "fig_overall_timetogoal")


if __name__ == "__main__":
    main()
