"""Batch report: scenario PNG, metrics split three ways, and two ladder heatmaps.

Produces, in <batch>/report/:

    scenarios/<name>.png      what the scenario looks like (obstacles, patrol
                              routes, robot start -> target)
    metrics_overall.csv       completion, timing, path quality
    metrics_replan.csv        replan counts, trigger breakdown, planning cost
    metrics_collision.csv     contacts, attribution, clearance
    heatmap_replans.png       replans per run,     3 planners x policy ladder
    heatmap_collision_free.png  collision-free %,  3 planners x policy ladder

The two heatmaps are deliberately separate figures, each carrying ONE metric, so
neither has to mix "higher is better" with "lower is better" inside one frame.

Usage:
    python scripts/report_batch.py results/acra-6v6-canonical [--scenarios 3]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, Normalize  # noqa: E402
from matplotlib.patches import Circle  # noqa: E402

FIELD_L, FIELD_W = 9000.0, 6000.0
ROBOT_R = 90.0
PLANNERS = ["voronoi", "prm", "visibility"]
PLANNER_LABEL = {"voronoi": "Voronoi", "prm": "PRM", "visibility": "Visibility graph"}
INK, MUTED, INK_INV = "#1f2733", "#5b6472", "#ffffff"

BLUES = LinearSegmentedColormap.from_list(
    "seq_blue",
    ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"],
)
ORANGES = LinearSegmentedColormap.from_list(
    "seq_orange",
    ["#fde3d3", "#fbc3a1", "#f7a071", "#ee7a43", "#d55c25", "#ab4518", "#7c2f0f"],
)

# --- metric grouping -------------------------------------------------------
KEYS = ["scenario", "planner", "replan_policy", "prediction_horizon_ms",
        "replan_period_ms", "planning_clearance_mm", "trial", "seed"]

GROUPS = {
    "overall": [
        "status", "completed", "failed_plans", "time_to_goal_ms",
        "simulated_duration_ms", "wall_time_ms", "realtime_factor",
        "planned_path_length_mm", "travelled_distance_mm", "straight_line_mm",
        "mean_final_error_mm", "max_final_error_mm", "max_final_speed_mmps",
    ],
    "replan": [
        "replan_count", "replan_failures", "replan_attempts",
        "blocked_ms", "blocked_time_pct", "any_replan_failure",
        "rebuild_calls", "check_calls",
        "planner_calls", "planning_time_ms_total", "planning_time_ms_initial",
        "planning_time_ms_max_call", "planning_time_ms_p95_call",
        "planning_time_ms_rebuild", "planning_time_ms_check",
        "replans_active_blocked", "replans_route_blocked", "replans_route_finished",
        "replans_scheduled", "replans_other", "direct_path_switches",
        "path_shift_mm_mean", "path_shift_mm_max",
        "heading_change_rad_per_m", "sharp_turns",
    ],
    "collision": [
        "collision_episodes", "obstacle_collision_episodes", "robot_collision_episodes",
        "obstacle_episodes_obstacle_initiated", "obstacle_episodes_robot_initiated",
        "minimum_clearance_mm", "minimum_robot_clearance_mm",
        "minimum_obstacle_clearance_mm", "contact_time_ms",
    ],
}


def render_scenario(path: Path, out: Path) -> None:
    """Draw one scenario: field, obstacles with inflation, patrol routes, robots."""
    d = json.loads(path.read_text())
    fig, ax = plt.subplots(figsize=(7.2, 5.0))
    ax.add_patch(plt.Rectangle((-FIELD_L / 2, -FIELD_W / 2), FIELD_L, FIELD_W,
                               fill=False, ec="#c7ccd4", lw=1.2))
    ax.plot([0, 0], [-FIELD_W / 2, FIELD_W / 2], color="#e3e6ea", lw=1)

    for o in d.get("obstacles", []):
        pos, r = o["position_mm"], o.get("radius_mm", ROBOT_R)
        tag = f"{'Y' if o.get('is_yellow') else 'B'}{o.get('obstacle_id', '?')}"
        wps = o.get("patrol_waypoints") or []
        if wps:
            route = [pos] + [list(w) for w in wps]
            xs, ys = zip(*route)
            ax.plot(xs, ys, color="#ee7a43", lw=1.1, ls="--", alpha=0.75, zorder=1)
            ax.plot(xs[1:], ys[1:], "o", color="#ee7a43", ms=3.5, alpha=0.8, zorder=2)
        ax.add_patch(Circle(pos, r + ROBOT_R, fc="#ee7a43", alpha=0.10,
                            ec="#ee7a43", ls=":", lw=0.9, zorder=2))
        ax.add_patch(Circle(pos, r, fc="#d55c25", alpha=0.9, ec="none", zorder=3))
        ax.annotate(tag, xy=pos, xytext=(7, 6), textcoords="offset points",
                    fontsize=7.5, color="#a8430f", fontweight="bold", zorder=6)

    for rb in d.get("robots", []):
        s, t = rb["start_mm"], rb["target_mm"]
        tag = f"{'Y' if rb.get('is_yellow') else 'B'}{rb.get('robot_id', '?')}"
        ax.annotate("", xy=t, xytext=s,
                    arrowprops=dict(arrowstyle="->", color="#256abf", lw=1.4,
                                    alpha=0.85, shrinkA=6, shrinkB=6), zorder=4)
        ax.add_patch(Circle(s, ROBOT_R, fc="#3987e5", ec="none", alpha=0.95, zorder=5))
        ax.annotate(tag, xy=s, xytext=(7, 6), textcoords="offset points",
                    fontsize=7.5, color="#1c5fb0", fontweight="bold", zorder=6)
        ax.plot(*t, marker="x", color="#184f95", ms=8, mew=2, zorder=5)
        ax.annotate(f"{tag}\u2009→", xy=t, xytext=(9, 7), textcoords="offset points",
                    fontsize=7.5, color="#184f95", fontweight="bold", zorder=6)

    n_o = len(d.get("obstacles", []))
    n_p = sum(1 for o in d.get("obstacles", []) if o.get("patrol_waypoints"))
    ax.set_title(f"{d.get('name','scenario')} — {len(d.get('robots',[]))} planned robots, "
                 f"{n_o} obstacles ({n_p} patrolling)", fontsize=11, color=INK)
    ax.text(0.0, -0.045, "B = blue team, Y = yellow team.  Filled disc = start, "
            "x = target (marked \u2192).  Dashed = patrol route.  Pale ring = robot-radius inflation.",
            transform=ax.transAxes, fontsize=8, color=MUTED, ha="left", va="top")
    ax.set_xlim(-FIELD_L / 2 - 250, FIELD_L / 2 + 250)
    ax.set_ylim(-FIELD_W / 2 - 250, FIELD_W / 2 + 250)
    ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(False)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def load(folder: Path) -> pd.DataFrame:
    runs = pd.read_csv(folder / "runs.csv")
    if "obstacle_episodes_robot_initiated" in runs:
        runs["collision_free"] = ((runs["obstacle_episodes_robot_initiated"] == 0)
                                  & (runs["robot_collision_episodes"] == 0))
        runs["attribution"] = "DEC-008 robot-initiated"
    else:
        runs["collision_free"] = runs["collision_episodes"] == 0
        runs["attribution"] = "UNCORRECTED (no attribution columns)"
        print("  ! no attribution columns — collision numbers are uncorrected")
    runs["replan_period_ms"] = runs["replan_period_ms"].round(3)
    if "planning_clearance_mm" not in runs:
        runs["planning_clearance_mm"] = 0.0
    failed = runs.loc[runs["status"] == "planning_failed", ["scenario", "planner"]]
    bad = set(map(tuple, failed.drop_duplicates().values))
    runs["initialised"] = [(s, p) not in bad for s, p in zip(runs.scenario, runs.planner)]
    # A replan ATTEMPT is a successful replan plus one that returned no path.
    # A replan failure means the trigger fired and the planner could not answer:
    # the robot carries on along a route already known to be blocked.
    runs["replan_attempts"] = runs["replan_count"] + runs["replan_failures"]
    runs["any_replan_failure"] = runs["replan_failures"] > 0
    # A failed replan is retried at the next opportunity, so a single persistent
    # blocked situation is counted once per replan period. The raw count is
    # therefore NOT comparable across periods or policies. Multiplying by the
    # period converts it to the time the robot spent with no valid route, which
    # is period-independent and physically meaningful.
    runs["blocked_ms"] = runs["replan_failures"] * runs["replan_period_ms"]
    runs["blocked_time_pct"] = (
        100 * runs["blocked_ms"] / runs["simulated_duration_ms"].replace(0, np.nan)
    )
    return runs


def ladder(runs: pd.DataFrame) -> list[tuple[str, str, dict]]:
    """Ordered configurations: once | cycle@periods | event | event+pred@horizons."""
    periods = sorted(runs.replan_period_ms.unique())
    horizons = [h for h in sorted(runs.prediction_horizon_ms.unique()) if h > 0]
    base = min(periods)
    out: list[tuple[str, str, dict]] = []
    if (runs.replan_policy == "once").any():
        out.append(("once", "One shot", {"replan_policy": "once"}))
    for p in periods:
        out.append((f"cycle{p:g}", f"Cycle\n{p:g} ms",
                    {"replan_policy": "cycle", "replan_period_ms": p,
                     "prediction_horizon_ms": 0.0}))
    out.append(("event", "Event\ntrigger",
                {"replan_policy": "event", "replan_period_ms": base,
                 "prediction_horizon_ms": 0.0}))
    for h in horizons:
        out.append((f"evpred{h:g}", f"Event\n+traj {h:g} ms",
                    {"replan_policy": "event", "replan_period_ms": base,
                     "prediction_horizon_ms": h}))
    return out


def _cells(ax, M, norm, cmap, fmt):
    ax.imshow(M, cmap=cmap, norm=norm, aspect="auto", origin="upper")
    for sp in ax.spines.values():
        sp.set_visible(False)
    ax.tick_params(length=0, colors=MUTED, labelsize=9)
    for i in range(M.shape[0]):
        for j in range(M.shape[1]):
            v = M[i, j]
            if np.isnan(v):
                ax.text(j, i, "—", ha="center", va="center", color=MUTED, fontsize=10)
                continue
            r, g, b, _ = cmap(norm(v))
            lum = 0.299 * r + 0.587 * g + 0.114 * b
            ax.text(j, i, fmt.format(v), ha="center", va="center", fontsize=10,
                    color=INK_INV if lum < 0.55 else INK)


def heatmap(runs, cfgs, value, title, cbar_label, cmap, fmt, out: Path):
    planners = [p for p in PLANNERS if p in runs.planner.values]
    M = np.full((len(planners), len(cfgs)), np.nan)
    for i, pl in enumerate(planners):
        for j, (_, _, sel) in enumerate(cfgs):
            m = (runs.planner == pl) & runs.initialised
            for k, v in sel.items():
                m &= runs[k] == v
            if m.any():
                M[i, j] = value(runs[m])
    finite = M[np.isfinite(M)]
    norm = Normalize(vmin=0, vmax=100) if value.__name__ == "_cf" else \
        Normalize(vmin=0, vmax=float(finite.max()) if finite.size else 1)

    fig, ax = plt.subplots(figsize=(1.15 * len(cfgs) + 2.6, 0.82 * len(planners) + 2.2))
    _cells(ax, M, norm, cmap, fmt)
    ax.set_xticks(range(len(cfgs)), [c[1] for c in cfgs])
    ax.set_yticks(range(len(planners)), [PLANNER_LABEL[p] for p in planners])
    ax.set_title(title, fontsize=12, color=INK, pad=12, loc="left")
    bar = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap), ax=ax,
                       shrink=0.82, pad=0.02)
    bar.set_label(cbar_label, fontsize=10, color=MUTED)
    bar.outline.set_visible(False)
    bar.ax.tick_params(length=0, colors=MUTED, labelsize=9)
    fig.savefig(out.with_suffix(".png"), dpi=200, bbox_inches="tight", facecolor="white")
    fig.savefig(out.with_suffix(".pdf"), bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("  wrote", out.with_suffix(".png").name)


def blocked_time(runs: pd.DataFrame, out_dir: Path) -> None:
    """Time each run spent with no valid route, by planner and policy.

    A replan failure means the trigger fired and the planner returned no path,
    so the robot carries on along a route already known to be blocked. The raw
    failure COUNT is a retry artefact -- the same situation is re-attempted every
    replan period, so a short period inflates it several-fold. Multiplying by the
    period recovers the duration, which is what actually matters and is stable
    across periods.
    """
    planners = [p for p in PLANNERS if p in runs.planner.values]
    init = runs[runs.initialised]
    common = set.intersection(*[set(init[init.planner == p].scenario) for p in planners])
    d = init[init.scenario.isin(common) & (init.replan_policy != "once")]
    if d.empty:
        print("  blocked-time: nothing to report"); return

    t = (d.groupby(["planner", "replan_policy", "replan_period_ms"])
           .agg(n=("blocked_time_pct", "size"),
                failures_median=("replan_failures", "median"),
                blocked_ms_median=("blocked_ms", "median"),
                blocked_pct_median=("blocked_time_pct", "median"),
                blocked_pct_mean=("blocked_time_pct", "mean"))
           .reset_index())
    t.to_csv(out_dir / "metrics_blocked_time.csv", index=False)
    print(f"  metrics_blocked_time.csv  (n={len(common)} common scenarios)")

    policies = [p for p in ("cycle", "event") if p in t.replan_policy.values]
    colour = {"cycle": "#d55c25", "event": "#256abf"}
    marker = ["o", "s", "^", "D"]
    periods = sorted(t.replan_period_ms.unique())

    fig, ax = plt.subplots(figsize=(7.6, 0.85 * len(planners) + 2.4))
    for i, pl in enumerate(planners):
        row = t[t.planner == pl]
        lo, hi = row.blocked_pct_median.min(), row.blocked_pct_median.max()
        ax.plot([lo, hi], [i, i], color="#e3e6ea", lw=3, solid_capstyle="round", zorder=1)
        for pol in policies:
            for k, per in enumerate(periods):
                c = row[(row.replan_policy == pol) & (row.replan_period_ms == per)]
                if c.empty:
                    continue
                ax.plot(c.blocked_pct_median.iloc[0], i, marker[k % 4],
                        color=colour[pol], ms=8, zorder=3, alpha=0.9)
    ax.set_yticks(range(len(planners)), [PLANNER_LABEL[p] for p in planners])
    ax.invert_yaxis()
    ax.set_xlim(left=-0.6)
    ax.set_xlabel("Share of each run with no valid route (%) — lower is better",
                  fontsize=10, color=MUTED)
    ax.set_title("Time spent executing a route already known to be blocked",
                 fontsize=12, color=INK, loc="left", pad=34)
    handles = [plt.Line2D([], [], marker="o", ls="", color=colour[p],
                          label=f"{p.capitalize()} replanning") for p in policies]
    handles += [plt.Line2D([], [], marker=marker[k % 4], ls="", color=MUTED,
                           label=f"{per:g} ms period") for k, per in enumerate(periods)]
    ax.legend(handles=handles, frameon=False, fontsize=9, labelcolor=MUTED,
              ncol=len(handles), loc="lower center", bbox_to_anchor=(0.5, 1.02))
    ax.grid(axis="x", color="#e3e6ea", lw=0.8)
    ax.set_axisbelow(True)
    ax.margins(y=0.28)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color("#e3e6ea")
    ax.tick_params(colors=MUTED, labelsize=9, length=3)
    o = out_dir / "fig_blocked_time"
    fig.savefig(o.with_suffix(".png"), dpi=200, bbox_inches="tight", facecolor="white")
    fig.savefig(o.with_suffix(".pdf"), bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("  wrote fig_blocked_time.png")


def _cf(d):      # collision-free %
    return 100 * d.collision_free.mean()


def _replans(d):  # median replans per run
    return d.replan_count.median()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("batch", type=Path)
    ap.add_argument("--scenarios", type=int, default=3, help="How many scenario PNGs")
    a = ap.parse_args()

    out = a.batch / "report"
    out.mkdir(parents=True, exist_ok=True)
    runs = load(a.batch)
    print(f"  attribution: {runs.attribution.iloc[0]}")

    sdir = a.batch / "scenarios"
    files = sorted(sdir.glob("*.json"))[: a.scenarios] if sdir.is_dir() else []
    for f in files:
        render_scenario(f, out / "scenarios" / f"{f.stem}.png")
    print(f"  rendered {len(files)} scenario PNG(s)")

    keys = [k for k in KEYS if k in runs]
    for name, cols in GROUPS.items():
        cols = [c for c in cols if c in runs]
        runs[keys + cols].to_csv(out / f"metrics_{name}.csv", index=False)
        print(f"  metrics_{name}.csv  ({len(cols)} metrics)")

    blocked_time(runs, out)

    cfgs = ladder(runs)
    heatmap(runs, cfgs, _replans, "Replans per run",
            "Replans per run (median) — fewer is better", ORANGES, "{:.0f}",
            out / "heatmap_replans")
    heatmap(runs, cfgs, _cf, "Collision-free runs",
            "Collision-free runs (%) — more is better", BLUES, "{:.0f}",
            out / "heatmap_collision_free")


if __name__ == "__main__":
    main()
