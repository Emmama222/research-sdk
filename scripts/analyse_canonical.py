"""Corrected analysis for the canonical 6v6 patrol batch.

Fixes applied relative to analyse_sweep.py:
  1. Safety counts only robot-initiated robot/obstacle contacts (DEC-008).
  2. Cross-planner comparisons use the common scenario intersection (DEC-009).
  3. No best-configuration selection; the whole horizon x period surface is
     reported (DEC-011).
  4. Paired tests report effect size and a 95% CI alongside every p-value.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pandas as pd
from scipy.stats import binomtest, norm

Z = norm.ppf(0.975)


def wilson(successes: int, total: int) -> tuple[float, float]:
    if total == 0:
        return (0.0, 0.0)
    p = successes / total
    denom = 1 + Z**2 / total
    centre = (p + Z**2 / (2 * total)) / denom
    margin = Z * ((p * (1 - p) / total + Z**2 / (4 * total**2)) ** 0.5) / denom
    return (max(0.0, centre - margin) * 100, min(1.0, centre + margin) * 100)


def mcnemar_exact(both: pd.DataFrame, a: str, b: str) -> dict:
    """Exact McNemar on paired binary outcomes, with a CI on the rate difference."""
    discordant_a = int(((both[a]) & (~both[b])).sum())
    discordant_b = int(((~both[a]) & (both[b])).sum())
    n = discordant_a + discordant_b
    p = binomtest(discordant_a, n, 0.5).pvalue if n else 1.0
    total = len(both)
    diff = (both[a].mean() - both[b].mean()) * 100
    # CI on the paired difference of proportions (Agresti-Min style, via discordants)
    if total:
        se = ((discordant_a + discordant_b) - (discordant_a - discordant_b) ** 2 / total) ** 0.5
        margin = Z * se / total * 100 if se > 0 else 0.0
    else:
        margin = 0.0
    return {
        "n_pairs": total,
        f"{a}_only": discordant_a,
        f"{b}_only": discordant_b,
        "diff_pct_points": round(diff, 1),
        "diff_ci95": [round(diff - margin, 1), round(diff + margin, 1)],
        "p_value": round(p, 6),
    }


def load(result_dir: Path) -> pd.DataFrame:
    d = pd.read_csv(result_dir / "runs.csv")
    required = {"obstacle_episodes_robot_initiated", "obstacle_episodes_obstacle_initiated"}
    missing = required - set(d.columns)
    if missing:
        sys.exit(f"runs.csv predates DEC-008 attribution; missing {sorted(missing)}")

    # Primary safety: zero robot-initiated robot/obstacle contacts.
    d["safe"] = d.obstacle_episodes_robot_initiated == 0
    # Sensitivity variants.
    d["safe_all_obstacle"] = d.obstacle_collision_episodes == 0
    d["safe_unfiltered"] = d.collision_episodes == 0
    d["teammate_clean"] = d.robot_collision_episodes == 0
    d["over_budget"] = d.planning_time_ms_max_call > 16.7
    return d


def usable_scenarios(d: pd.DataFrame) -> dict[str, set]:
    """Scenarios whose initial plan succeeded, per planner."""
    once = d[d.replan_policy == "once"]
    bad = once.assign(bad=once.failed_plans > 0).groupby(["planner", "scenario"])["bad"].max()
    return {p: set(bad.loc[p][~bad.loc[p].astype(bool)].index) for p in d.planner.unique()}


def main(result_dir: Path, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    d = load(result_dir)
    ok = usable_scenarios(d)
    common = set.intersection(*ok.values())

    report: dict = {
        "source": str(result_dir),
        "total_runs": len(d),
        "safety_metric": "zero robot-initiated robot/obstacle contact episodes (DEC-008)",
        "robustness": {
            p: {
                "initial_plan_failures": 200 - len(s),
                "scenarios_total": 200,
                "usable": len(s),
            }
            for p, s in sorted(ok.items())
        },
        "common_intersection_n": len(common),
    }

    # --- Surface: every planner x policy x horizon x period cell, common set ---
    c = d[d.scenario.isin(common)]
    rows = []
    for (planner, policy, horizon, period), g in c.groupby(
        ["planner", "replan_policy", "prediction_horizon_ms", "replan_period_ms"], dropna=False
    ):
        safe = int(g.safe.sum())
        lo, hi = wilson(safe, len(g))
        rows.append(
            {
                "planner": planner,
                "policy": policy,
                "horizon_ms": horizon,
                "period_ms": period,
                "n": len(g),
                "collision_free_pct": round(g.safe.mean() * 100, 1),
                "ci95_lo": round(lo, 1),
                "ci95_hi": round(hi, 1),
                "collision_free_all_obstacle_pct": round(g.safe_all_obstacle.mean() * 100, 1),
                "collision_free_unfiltered_pct": round(g.safe_unfiltered.mean() * 100, 1),
                "teammate_clean_pct": round(g.teammate_clean.mean() * 100, 1),
                "completion_pct": round(g.completed.mean() * 100, 1),
                "planning_ms_median": round(g.planning_time_ms_total.median(), 1),
                "planning_ms_per_sim_s": round(
                    (g.planning_time_ms_total / (g.simulated_duration_ms / 1000)).median(), 2
                ),
                "max_call_ms_median": round(g.planning_time_ms_max_call.median(), 2),
                "over_budget_pct": round(g.over_budget.mean() * 100, 1),
                "time_to_goal_s_median": round(g.time_to_goal_ms.median() / 1000, 2)
                if g.time_to_goal_ms.notna().any()
                else None,
                "replans_median": round(g.replan_count.median(), 1),
            }
        )
    surface = pd.DataFrame(rows).sort_values(["planner", "policy", "horizon_ms", "period_ms"])
    surface.to_csv(out_dir / "surface.csv", index=False)

    # --- Paired within-planner comparisons (own usable set, paired by scenario) ---
    paired = {}
    for planner in sorted(d.planner.unique()):
        sub = d[(d.planner == planner) & (d.scenario.isin(ok[planner]))]
        for horizon in sorted(sub.prediction_horizon_ms.dropna().unique()):
            for period in sorted(sub.replan_period_ms.dropna().unique()):
                cell = sub[
                    (sub.prediction_horizon_ms == horizon) & (sub.replan_period_ms == period)
                ]
                wide = cell.pivot_table(
                    index="scenario", columns="replan_policy", values="safe", aggfunc="first"
                ).dropna()
                if {"event", "cycle"}.issubset(wide.columns) and len(wide):
                    wide = wide.astype(bool)
                    key = f"{planner}@h{int(horizon)}_p{int(period)}"
                    paired[key] = mcnemar_exact(wide, "cycle", "event")
                    paired[key]["cycle_pct"] = round(wide["cycle"].mean() * 100, 1)
                    paired[key]["event_pct"] = round(wide["event"].mean() * 100, 1)
                    ev = cell[cell.replan_policy == "event"].planning_time_ms_total.median()
                    cy = cell[cell.replan_policy == "cycle"].planning_time_ms_total.median()
                    paired[key]["compute_ratio_cycle_over_event"] = (
                        round(cy / ev, 2) if ev else None
                    )
    report["paired_cycle_vs_event"] = paired

    # --- once vs event at a FIXED reference configuration ---
    # Taking the per-scenario best across event configs would be selection on the
    # evaluation data; a single declared reference config avoids that.
    REF_HORIZON, REF_PERIOD = 50.0, 20.0
    once_vs = {}
    for planner in sorted(d.planner.unique()):
        sub = d[(d.planner == planner) & (d.scenario.isin(ok[planner]))]
        ref = sub[
            (sub.replan_policy == "event")
            & (sub.prediction_horizon_ms == REF_HORIZON)
            & (sub.replan_period_ms == REF_PERIOD)
        ]
        wide = pd.DataFrame(
            {
                "once": sub[sub.replan_policy == "once"].groupby("scenario").safe.first(),
                "event": ref.groupby("scenario").safe.first(),
            }
        ).dropna()
        if len(wide):
            once_vs[planner] = mcnemar_exact(wide.astype(bool), "event", "once")
            once_vs[planner]["reference_config"] = f"event h{int(REF_HORIZON)} p{int(REF_PERIOD)}"
            once_vs[planner]["once_pct"] = round(wide["once"].mean() * 100, 1)
            once_vs[planner]["event_pct"] = round(wide["event"].mean() * 100, 1)
    report["paired_event_vs_once"] = once_vs

    # --- attribution accounting ---
    report["attribution"] = {
        "runs_with_any_obstacle_contact_pct": round(
            (d.obstacle_collision_episodes > 0).mean() * 100, 1
        ),
        "runs_with_robot_initiated_contact_pct": round(
            (d.obstacle_episodes_robot_initiated > 0).mean() * 100, 1
        ),
        "runs_with_obstacle_initiated_contact_pct": round(
            (d.obstacle_episodes_obstacle_initiated > 0).mean() * 100, 1
        ),
        "runs_rescued_by_attribution_pct": round(
            ((d.obstacle_collision_episodes > 0) & (d.obstacle_episodes_robot_initiated == 0)).mean()
            * 100,
            1,
        ),
        "episode_share_obstacle_initiated_pct": round(
            d.obstacle_episodes_obstacle_initiated.sum()
            / max(1, d.obstacle_collision_episodes.sum())
            * 100,
            1,
        ),
    }

    (out_dir / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2)[:4000])
    print(f"\nsurface.csv rows: {len(surface)} -> {out_dir}")


if __name__ == "__main__":
    src = Path(sys.argv[1])
    main(src, src / "analysis")
