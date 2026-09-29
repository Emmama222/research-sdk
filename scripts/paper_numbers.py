#!/usr/bin/env python3
"""Every number the paper quotes, computed from one batch, in one place.

The introduction and results sections cite figures that were previously
copied by hand from whichever batch was current. When the canonical batch
changed, two claims silently inverted. This script is the fix: it emits every
quoted value from a named batch, so the paper can be regenerated rather than
edited, and a stale number becomes a diff instead of a submission.

Writes to <batch>/analysis/:
    paper_numbers.json   machine readable, for diffing between batches
    paper_numbers.md     human readable, grouped by the claim it supports

Conventions enforced here (see acra2026-metrics-dictionary.md):
  * safety is DEC-008 robot-initiated only;
  * cross-arm comparisons are paired by scenario;
  * timing is reported as median, never mean, because it is right-skewed;
  * time-to-goal is stated as over completed runs only.

Usage:
    python scripts/paper_numbers.py results/acra-6v6-canonical
    python scripts/paper_numbers.py <new batch> --compare results/acra-6v6-canonical
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import pandas as pd

try:
    from scipy.stats import wilcoxon
except ImportError:  # keep the script usable without scipy
    wilcoxon = None

PLANNERS = ["voronoi", "prm", "visibility"]
LABEL = {"voronoi": "Voronoi", "prm": "PRM", "visibility": "Visibility graph"}
BUDGET_MS = 1000.0 / 60.0


def load(folder: Path) -> pd.DataFrame:
    runs = pd.read_csv(folder / "runs.csv")
    if "obstacle_episodes_robot_initiated" not in runs:
        raise SystemExit(
            f"{folder}/runs.csv has no DEC-008 attribution columns. Safety computed "
            "from this batch would charge the planner for obstacle-initiated contacts. "
            "Re-run the batch before quoting it."
        )
    runs["safe"] = (runs.obstacle_episodes_robot_initiated == 0) & (
        runs.robot_collision_episodes == 0
    )
    runs["over_budget"] = runs.planning_time_ms_max_call > BUDGET_MS
    runs["replan_period_ms"] = runs["replan_period_ms"].round(3)
    return runs


def _pct(series) -> float:
    return round(100 * float(series.mean()), 1)


def safety_by_policy(runs: pd.DataFrame) -> dict:
    out = {}
    for planner in PLANNERS:
        sub = runs[runs.planner == planner]
        out[planner] = {
            policy: _pct(sub[sub.replan_policy == policy].safe)
            for policy in sorted(sub.replan_policy.unique())
        }
    return out


def safety_by_horizon(runs: pd.DataFrame, policy: str = "cycle",
                      period: float | None = None) -> dict:
    """Collision-free % by prediction horizon.

    ``period`` selects one replan period; None pools both. These differ
    materially -- pooling 20 ms and 100 ms drags Voronoi's 50 ms figure from
    99.5% to 96.5% -- so any quoted number must name its slice.
    """
    sub = runs[runs.replan_policy == policy]
    if period is not None:
        sub = sub[sub.replan_period_ms == period]
    return {
        planner: {
            f"{h:.0f}": _pct(g.safe)
            for h, g in sub[sub.planner == planner].groupby("prediction_horizon_ms")
        }
        for planner in PLANNERS
    }


def failed_rebuilds_by_horizon(runs: pd.DataFrame, policy: str = "cycle",
                               period: float | None = None) -> dict:
    sub = runs[runs.replan_policy == policy]
    if period is not None:
        sub = sub[sub.replan_period_ms == period]
    return {
        planner: {
            f"{h:.0f}": round(float(g.replan_failures.mean()), 1)
            for h, g in sub[sub.planner == planner].groupby("prediction_horizon_ms")
        }
        for planner in PLANNERS
    }


def compute_ratio(runs: pd.DataFrame) -> dict:
    """Median total planning time per run, cycle over event."""
    out = {}
    for planner in PLANNERS:
        sub = runs[runs.planner == planner]
        c = sub[sub.replan_policy == "cycle"].planning_time_ms_total.median()
        e = sub[sub.replan_policy == "event"].planning_time_ms_total.median()
        out[planner] = {
            "cycle_ms": round(float(c), 1),
            "event_ms": round(float(e), 1),
            "ratio": round(float(c / e), 2) if e else None,
        }
    return out


def budget_overruns(runs: pd.DataFrame) -> dict:
    out = {}
    for planner in PLANNERS:
        sub = runs[runs.planner == planner]
        out[planner] = {
            policy: _pct(sub[sub.replan_policy == policy].over_budget)
            for policy in ("cycle", "event")
        }
    return out


def time_to_goal_cost(runs: pd.DataFrame) -> dict:
    """Paired event-minus-cycle time to goal, by planner and period.

    Paired on scenario. Only runs that completed contribute, because
    time_to_goal_ms is null otherwise -- stated explicitly so the caption can
    say "among runs that reached the goal".
    """
    out = {}
    for planner in PLANNERS:
        for period in sorted(runs.replan_period_ms.unique()):
            sub = runs[
                (runs.planner == planner)
                & (runs.replan_period_ms == period)
                & (runs.replan_policy.isin(["cycle", "event"]))
            ]
            wide = sub.pivot_table(
                index="scenario", columns="replan_policy",
                values="time_to_goal_ms", aggfunc="first",
            ).dropna()
            if not len(wide) or "event" not in wide or "cycle" not in wide:
                continue
            delta = (wide["event"] - wide["cycle"]) / 1000.0
            entry = {
                "n_paired_completed": int(len(wide)),
                "median_delta_s": round(float(delta.median()), 3),
                "mean_delta_s": round(float(delta.mean()), 3),
            }
            if wilcoxon is not None and (delta != 0).any():
                entry["wilcoxon_p"] = float(wilcoxon(wide["event"], wide["cycle"]).pvalue)
            out[f"{planner}@{period:.0f}ms"] = entry
    return out


def initial_plan_failures(runs: pd.DataFrame) -> dict:
    """Share of scenarios where the first plan produced nothing.

    Measured on the 'once' arm, which plans at t=0 and never again, so the
    figure is not contaminated by later replanning.
    """
    once = runs[runs.replan_policy == "once"]
    if once.empty:
        once = runs
    out = {}
    for planner in PLANNERS:
        g = once[once.planner == planner]
        out[planner] = {
            "scenarios": int(len(g)),
            "failed": int((g.failed_plans > 0).sum()),
            "pct": round(100 * float((g.failed_plans > 0).mean()), 1),
        }
    return out


def replanning_gap(runs: pd.DataFrame) -> dict:
    """Collision-free percentage points lost by planning once."""
    out = {}
    for planner in PLANNERS:
        sub = runs[runs.planner == planner]
        once = _pct(sub[sub.replan_policy == "once"].safe)
        out[planner] = {
            "once_pct": once,
            "cycle_minus_once": round(_pct(sub[sub.replan_policy == "cycle"].safe) - once, 1),
            "event_minus_once": round(_pct(sub[sub.replan_policy == "event"].safe) - once, 1),
        }
    return out


def collect(runs: pd.DataFrame, folder: Path) -> dict:
    return {
        "batch": folder.name,
        "runs": int(len(runs)),
        "scenarios": int(runs.scenario.nunique()),
        "attribution": "DEC-008 robot-initiated",
        "frame_budget_ms": round(BUDGET_MS, 2),
        "safety_by_policy_pct": safety_by_policy(runs),
        "safety_by_horizon_pct_cycle_pooled": safety_by_horizon(runs, "cycle"),
        "safety_by_horizon_pct_event_pooled": safety_by_horizon(runs, "event"),
        "safety_by_horizon_pct_cycle_20ms": safety_by_horizon(runs, "cycle", 20.0),
        "safety_by_horizon_pct_event_20ms": safety_by_horizon(runs, "event", 20.0),
        "safety_by_horizon_pct_cycle_100ms": safety_by_horizon(runs, "cycle", 100.0),
        "failed_rebuilds_by_horizon_cycle_pooled": failed_rebuilds_by_horizon(runs, "cycle"),
        "failed_rebuilds_by_horizon_cycle_20ms": failed_rebuilds_by_horizon(runs, "cycle", 20.0),
        "compute_cycle_over_event": compute_ratio(runs),
        "budget_overrun_pct": budget_overruns(runs),
        "time_to_goal_event_minus_cycle": time_to_goal_cost(runs),
        "initial_plan_failures": initial_plan_failures(runs),
        "replanning_gap_points": replanning_gap(runs),
    }


def _table(rows: list[list], header: list[str]) -> str:
    out = ["| " + " | ".join(header) + " |",
           "|" + "|".join("---" for _ in header) + "|"]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def to_markdown(n: dict) -> str:
    L = [f"# Paper numbers — `{n['batch']}`", "",
         f"{n['runs']} runs over {n['scenarios']} scenarios. "
         f"Safety: {n['attribution']}. Frame budget {n['frame_budget_ms']} ms.",
         "", "Generated by `scripts/paper_numbers.py`. Do not hand-edit; "
         "regenerate and diff instead.", ""]

    L += ["## Claim: event preserves safety", "",
          _table([[LABEL[p]] + [n["safety_by_policy_pct"][p].get(k, "-")
                                for k in ("once", "cycle", "event")]
                  for p in PLANNERS], ["Planner", "once %", "cycle %", "event %"]), ""]

    L += ["## Claim: event costs less compute", "",
          _table([[LABEL[p], n["compute_cycle_over_event"][p]["cycle_ms"],
                   n["compute_cycle_over_event"][p]["event_ms"],
                   f"{n['compute_cycle_over_event'][p]['ratio']}x"] for p in PLANNERS],
                 ["Planner", "cycle (ms)", "event (ms)", "saving"]), ""]

    L += ["## Claim: the deadline consequence", "",
          "Share of runs containing a call over the frame budget.", "",
          _table([[LABEL[p], f"{n['budget_overrun_pct'][p]['cycle']}%",
                   f"{n['budget_overrun_pct'][p]['event']}%"] for p in PLANNERS],
                 ["Planner", "cycle", "event"]), ""]

    L += ["## Claim: event costs time to goal", "",
          "Paired by scenario, among runs that reached the goal.", "",
          _table([[k, v["n_paired_completed"], f"{v['median_delta_s']:+.3f}",
                   f"{v['mean_delta_s']:+.3f}",
                   f"{v.get('wilcoxon_p', float('nan')):.1e}"]
                  for k, v in n["time_to_goal_event_minus_cycle"].items()],
                 ["Arm", "n", "median (s)", "mean (s)", "Wilcoxon p"]), ""]

    L += ["## Claim: prediction separates the roadmaps", "",
          "**Collision-free %, cycle policy, 20 ms replan period.** This is the "
          "slice the introduction quotes.", "",
          _table([[LABEL[p]] + [n["safety_by_horizon_pct_cycle_20ms"][p].get(h, "-")
                                for h in ("0", "50", "150")] for p in PLANNERS],
                 ["Planner", "0 ms", "50 ms", "150 ms"]), "",
          "Same, pooled over both replan periods — lower, and NOT what the "
          "introduction quotes. Name the slice wherever either is used.", "",
          _table([[LABEL[p]] + [n["safety_by_horizon_pct_cycle_pooled"][p].get(h, "-")
                                for h in ("0", "50", "150")] for p in PLANNERS],
                 ["Planner", "0 ms", "50 ms", "150 ms"]), "",
          "Mechanism — failed rebuilds per run, cycle policy, 20 ms period.", "",
          _table([[LABEL[p]] + [n["failed_rebuilds_by_horizon_cycle_20ms"][p].get(h, "-")
                                for h in ("0", "50", "150")] for p in PLANNERS],
                 ["Planner", "0 ms", "50 ms", "150 ms"]), ""]

    L += ["## Claim: robustness (CHECK DIRECTION BEFORE QUOTING)", "",
          _table([[LABEL[p], n["initial_plan_failures"][p]["failed"],
                   n["initial_plan_failures"][p]["scenarios"],
                   f"{n['initial_plan_failures'][p]['pct']}%"] for p in PLANNERS],
                 ["Planner", "failed", "scenarios", "rate"]), ""]

    L += ["## Claim: replanning at all dominates", "",
          _table([[LABEL[p], f"{n['replanning_gap_points'][p]['once_pct']}%",
                   f"+{n['replanning_gap_points'][p]['cycle_minus_once']}",
                   f"+{n['replanning_gap_points'][p]['event_minus_once']}"]
                  for p in PLANNERS],
                 ["Planner", "once", "cycle gain", "event gain"]), ""]
    return "\n".join(L)


def diff(new: dict, old: dict) -> list[str]:
    """Flag every leaf value that changed sign or moved more than 10%."""
    alerts = []

    def walk(a, b, path=""):
        if isinstance(a, dict) and isinstance(b, dict):
            for k in a:
                if k in b:
                    walk(a[k], b[k], f"{path}.{k}" if path else k)
        elif isinstance(a, (int, float)) and isinstance(b, (int, float)):
            if isinstance(a, bool) or b == 0:
                return
            if (a > 0) != (b > 0):
                alerts.append(f"SIGN FLIP  {path}: {b} -> {a}")
            elif abs(a - b) / max(abs(b), 1e-9) > 0.10:
                alerts.append(f"moved >10% {path}: {b} -> {a}")

    walk(new, old)
    return alerts


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("batch", type=Path)
    ap.add_argument("--compare", type=Path, default=None,
                    help="Earlier batch to diff against; flags sign flips")
    args = ap.parse_args()

    numbers = collect(load(args.batch), args.batch)
    out = args.batch / "analysis"
    out.mkdir(parents=True, exist_ok=True)
    (out / "paper_numbers.json").write_text(json.dumps(numbers, indent=2), encoding="utf-8")
    (out / "paper_numbers.md").write_text(to_markdown(numbers), encoding="utf-8")
    print(to_markdown(numbers))
    print(f"\nwrote {out/'paper_numbers.json'} and {out/'paper_numbers.md'}")

    if args.compare:
        old = collect(load(args.compare), args.compare)
        alerts = diff(numbers, old)
        print(f"\n=== diff against {args.compare.name} ===")
        print("\n".join(alerts) if alerts else "no material changes")
        if any(a.startswith("SIGN FLIP") for a in alerts):
            sys.exit(3)


if __name__ == "__main__":
    main()
