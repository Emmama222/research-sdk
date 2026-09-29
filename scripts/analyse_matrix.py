"""Analyse a headless planner x replan-policy batch (runs.csv) for the paper.

Usage:
    python scripts/analyse_matrix.py results/acra-matrix

Writes <batch>/analysis/: tables (CSV + LaTeX), paired statistics (JSON),
and figures (PDF + PNG). Scenarios whose *initial* plan failed for a planner
are excluded for that planner (the failure is identical across policies).
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
from scipy.stats import binomtest, wilcoxon  # noqa: E402

PLANNERS = ["voronoi", "prm", "visibility"]
PLANNER_LABEL = {"voronoi": "Voronoi", "prm": "PRM", "visibility": "Visibility graph"}
# Arms = replan policy (+ motion prediction). Only arms present in the data are drawn.
ALL_POLICIES = ["once", "cycle", "event", "cycle+pred", "event+pred"]
POLICIES = list(ALL_POLICIES)
POLICY_LABEL = {
    "once": "Plan once",
    "cycle": "Every cycle",
    "event": "Event-triggered",
    "cycle+pred": "Every cycle + prediction",
    "event+pred": "Event + prediction",
}
# Validated categorical slots 1-5 (adjacent pairs, light surface). Slots 3-5 are
# < 3:1 contrast, so every bar carries a value label and a hatch (relief rule).
POLICY_COLOR = {
    "once": "#2a78d6",
    "cycle": "#eb6834",
    "event": "#1baf7a",
    "cycle+pred": "#eda100",
    "event+pred": "#e87ba4",
}
POLICY_HATCH = {"once": "", "cycle": "//", "event": "..", "cycle+pred": "\\\\", "event+pred": "xx"}
NEAR_MISS_MM = 50.0
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"
BUDGET_MS = 1000.0 / 60.0


def set_group(name: str) -> str:
    if name == "random":
        return "random"
    if name.endswith("~perturbed"):
        return "perturbed"
    return "saved"


def load(folder: Path) -> pd.DataFrame:
    runs = pd.read_csv(folder / "runs.csv")
    runs["group"] = runs["scenario_set"].map(set_group)
    pred = runs.get("prediction_horizon_ms", pd.Series(0.0, index=runs.index)).fillna(0.0)
    runs["arm"] = np.where(pred > 0, runs["replan_policy"] + "+pred", runs["replan_policy"])
    runs.attrs["horizons"] = sorted(set(pred[pred > 0]))
    present = [a for a in ALL_POLICIES if a in set(runs["arm"])]
    POLICIES[:] = present
    runs["collision_free"] = runs["collision_episodes"] == 0
    runs["over_budget"] = runs["planning_time_ms_max_call"] > BUDGET_MS
    # Initial-plan failures are policy-independent; drop them per planner.
    failed = runs.loc[runs["status"] == "planning_failed", ["scenario", "planner"]]
    failed_keys = set(map(tuple, failed.drop_duplicates().values))
    keep = [(s, p) not in failed_keys for s, p in zip(runs["scenario"], runs["planner"])]
    runs.attrs["excluded"] = sorted(failed_keys)
    return runs[keep].copy()


def iqr(series: pd.Series) -> str:
    q1, q2, q3 = series.quantile([0.25, 0.5, 0.75])
    return f"{q2:.1f} [{q1:.1f}, {q3:.1f}]"


def table(runs: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for planner in PLANNERS:
        for policy in POLICIES:
            sub = runs[(runs.planner == planner) & (runs.arm == policy)]
            if sub.empty:
                continue
            done = sub[sub.completed]
            rows.append(
                {
                    "planner": PLANNER_LABEL[planner],
                    "policy": POLICY_LABEL[policy],
                    "n": len(sub),
                    "completed_%": 100 * sub.completed.mean(),
                    "collision_free_%": 100 * sub.collision_free.mean(),
                    "collisions_per_run": sub.collision_episodes.mean(),
                    "replans_per_run": sub.replan_count.mean(),
                    "planner_calls_per_run": sub.planner_calls.mean(),
                    "planning_ms_per_run_median": sub.planning_time_ms_total.median(),
                    "planning_ms_per_run_iqr": iqr(sub.planning_time_ms_total),
                    "max_call_ms_median": sub.planning_time_ms_max_call.median(),
                    "runs_with_call_over_16.7ms_%": 100 * sub.over_budget.mean(),
                    "time_to_goal_s_median": done.time_to_goal_ms.median() / 1000.0,
                    "travelled_m_mean": sub.travelled_distance_mm.mean() / 1000.0,
                    "min_clearance_mm_median": sub.minimum_clearance_mm.median(),
                    "near_miss_free_%": 100 * (sub.minimum_clearance_mm >= NEAR_MISS_MM).mean(),
                    "p95_call_ms_median": sub.planning_time_ms_p95_call.median()
                    if "planning_time_ms_p95_call" in sub
                    else float("nan"),
                    "planning_ms_per_sim_s_median": (
                        sub.planning_time_ms_total / (sub.simulated_duration_ms / 1000.0)
                    ).median(),
                    "path_efficiency_mean": (
                        sub.straight_line_mm / sub.travelled_distance_mm
                    ).mean()
                    if "straight_line_mm" in sub
                    else float("nan"),
                }
            )
    return pd.DataFrame(rows)


def _compare(wide, a: str, b: str) -> dict:
    """Paired comparison of arm ``a`` against arm ``b`` on the same scenarios."""
    res: dict = {}
    pt = wide["planning_time_ms_total"]
    ratio = pt[b] / pt[a]
    res[f"planning_time_{b}_over_{a}_median_ratio"] = float(ratio.median())
    res[f"planning_time_{b}_over_{a}_iqr"] = [float(ratio.quantile(0.25)), float(ratio.quantile(0.75))]
    diff = pt[a] - pt[b]
    res["planning_time_wilcoxon_p"] = float(wilcoxon(pt[a], pt[b]).pvalue) if (diff != 0).any() else 1.0
    rc = wide["replan_count"]
    res[f"replans_{a}_mean"] = float(rc[a].mean())
    res[f"replans_{b}_mean"] = float(rc[b].mean())
    cf = wide["collision_free"].astype(bool)
    only_a = int((cf[a] & ~cf[b]).sum())
    only_b = int((~cf[a] & cf[b]).sum())
    n = only_a + only_b
    res["collision_free"] = {
        f"{a}_rate": float(cf[a].mean()),
        f"{b}_rate": float(cf[b].mean()),
        f"only_{a}_safe": only_a,
        f"only_{b}_safe": only_b,
        "mcnemar_exact_p": float(binomtest(only_a, n, 0.5).pvalue) if n else 1.0,
    }
    ttg = wide["time_to_goal_ms"][[a, b]].dropna()
    if len(ttg):
        d = (ttg[a] - ttg[b]) / 1000.0
        res[f"time_to_goal_{a}_minus_{b}_s_mean"] = float(d.mean())
        res["time_to_goal_wilcoxon_p"] = float(wilcoxon(ttg[a], ttg[b]).pvalue) if (d != 0).any() else 1.0
        res["time_to_goal_pairs"] = int(len(ttg))
    return res


COMPARISONS = [
    ("event", "cycle"),
    ("event", "once"),
    ("event+pred", "event"),
    ("event+pred", "cycle+pred"),
    ("event+pred", "cycle"),
]


def paired(runs: pd.DataFrame) -> dict:
    out: dict = {}
    for planner in PLANNERS:
        sub = runs[runs.planner == planner]
        wide = sub.pivot_table(
            index="scenario",
            columns="arm",
            values=["planning_time_ms_total", "collision_free", "time_to_goal_ms",
                    "replan_count"],
            aggfunc="first",
        )
        res: dict = {"n_scenarios": int(len(wide))}
        for a, b in COMPARISONS:
            if a in POLICIES and b in POLICIES:
                res[f"{a}_vs_{b}"] = _compare(wide, a, b)
        out[planner] = res
    return out


def _style(ax, ylabel: str) -> None:
    ax.set_ylabel(ylabel, color=INK_2, fontsize=8)
    ax.tick_params(colors=INK_2, labelsize=8, length=0)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(GRID)
    ax.yaxis.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)


def _bars(ax, values: dict, fmt: str, log: bool = False, err: dict | None = None) -> None:
    width = 0.8 / len(POLICIES)
    x = np.arange(len(PLANNERS))
    for i, policy in enumerate(POLICIES):
        heights = [values[(p, policy)] for p in PLANNERS]
        pos = x + (i - (len(POLICIES) - 1) / 2) * width
        yerr = None
        if err is not None:
            lo = [values[(p, policy)] - err[(p, policy)][0] for p in PLANNERS]
            hi = [err[(p, policy)][1] - values[(p, policy)] for p in PLANNERS]
            yerr = [lo, hi]
        bars = ax.bar(
            pos, heights, width - 0.03, color=POLICY_COLOR[policy],
            hatch=POLICY_HATCH[policy], edgecolor="white", linewidth=0,
            label=POLICY_LABEL[policy], zorder=2,
        )
        if yerr is not None:
            ax.errorbar(pos, heights, yerr=yerr, fmt="none", ecolor=INK_2,
                        elinewidth=0.8, capsize=0, zorder=3)
        for bar, h in zip(bars, heights):
            ax.annotate(fmt.format(h), (bar.get_x() + bar.get_width() / 2, h),
                        xytext=(0, 2), textcoords="offset points", ha="center",
                        va="bottom", fontsize=5.5 if len(POLICIES) > 3 else 6.5,
                        color=INK)
    ax.set_xticks(x, [PLANNER_LABEL[p] for p in PLANNERS])
    if log:
        ax.set_yscale("log")


def figure(runs: pd.DataFrame, path: Path) -> None:
    plt.rcParams.update({"font.family": "DejaVu Sans", "hatch.color": "white",
                         "hatch.linewidth": 0.8})
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.2), constrained_layout=True)
    g = runs.groupby(["planner", "arm"])
    safe = (100 * g.collision_free.mean()).to_dict()
    med = g.planning_time_ms_total.median().to_dict()
    q = {k: (v.quantile(0.25), v.quantile(0.75)) for k, v in g.planning_time_ms_total}
    replans = g.replan_count.mean().to_dict()

    _bars(axes[0], safe, "{:.0f}")
    axes[0].set_ylim(0, 110)
    _style(axes[0], "Collision-free runs (%)")
    axes[0].set_title("(a) Safety", fontsize=9, color=INK, loc="left")

    _bars(axes[1], med, "{:.0f}", log=True, err=q)
    _style(axes[1], "Planning time per run (ms, log)")
    axes[1].set_title("(b) Planning compute (median, IQR)", fontsize=9, color=INK, loc="left")

    _bars(axes[2], replans, "{:.1f}")
    _style(axes[2], "Replans per run (mean)")
    axes[2].set_title("(c) Route rebuilds", fontsize=9, color=INK, loc="left")

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside upper center", ncol=len(POLICIES), frameon=False,
               fontsize=8)
    for suffix in (".pdf", ".png"):
        fig.savefig(path.with_suffix(suffix), dpi=200)
    plt.close(fig)


def latex(df: pd.DataFrame, caption: str, label: str) -> str:
    lines = [
        r"\begin{table}[t]", r"\centering", r"\footnotesize",
        rf"\caption{{{caption}}}", rf"\label{{{label}}}",
        r"\begin{tabular}{llrrrrr}", r"\hline",
        r"Planner & Policy & Coll.-free (\%) & Replans & Plan.\ time (ms) & "
        r"Max call (ms) & Goal (s) \\",
        r"\hline",
    ]
    for _, r in df.iterrows():
        lines.append(
            f"{r['planner']} & {r['policy']} & {r['collision_free_%']:.0f} & "
            f"{r['replans_per_run']:.1f} & {r['planning_ms_per_run_median']:.0f} & "
            f"{r['max_call_ms_median']:.1f} & {r['time_to_goal_s_median']:.2f} \\\\"
        )
    lines += [r"\hline", r"\end{tabular}", r"\end{table}"]
    return "\n".join(lines) + "\n"


def main(folder: Path) -> None:
    out = folder / "analysis"
    out.mkdir(exist_ok=True)
    runs = load(folder)
    stats = {"excluded_initial_plan_failures": [list(k) for k in runs.attrs["excluded"]]}
    for group, title in (
        ("random", "Random scenarios"),
        ("perturbed", "Perturbed hand-made scenarios"),
    ):
        sub = runs[runs.group == group]
        n = sub.scenario.nunique()
        tab = table(sub)
        tab.to_csv(out / f"table_{group}.csv", index=False)
        (out / f"table_{group}.tex").write_text(
            latex(
                tab,
                f"{title} ($n={n}$ scenarios, 3 robots, 60\\,Hz control). Median planning "
                "time per run and median of the per-run largest single call.",
                f"tab:{group}",
            ),
            encoding="utf-8",
        )
        stats[group] = paired(sub)
        figure(sub, out / f"fig_{group}")
    saved = runs[runs.group == "saved"]
    table(saved).to_csv(out / "table_saved.csv", index=False)
    (out / "stats.json").write_text(json.dumps(stats, indent=2), encoding="utf-8")
    print(json.dumps(stats, indent=2))


if __name__ == "__main__":
    main(Path(sys.argv[1] if len(sys.argv) > 1 else "results/acra-matrix"))
