#!/usr/bin/env python3
"""Numbers for ACRA sections 4.3 / 4.4, computed from one batch.

Conventions (stated in the paper's Experimental Setup):
  * safe run  = no robot-initiated obstacle contact (DEC-008) and no robot-robot contact
  * a scenario is excluded for a planner if that planner's initial plan fails in any arm
  * within-planner comparisons (event vs cycle) are paired by scenario at matched settings
  * cross-planner comparisons use the common intersection of valid scenarios
  * timing: median per run; frame budget 1000/60 ms
Usage: python scripts/paper_stats.py <batch>
"""
from __future__ import annotations
import json, math, sys
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.stats import binomtest, wilcoxon

P = ["voronoi", "prm", "visibility"]
BUDGET = 1000 / 60


def load(folder):
    r = pd.read_csv(folder / "runs.csv")
    r["safe"] = (r.obstacle_episodes_robot_initiated == 0) & (r.robot_collision_episodes == 0)
    r["over"] = r.planning_time_ms_max_call > BUDGET
    r["replan_period_ms"] = r.replan_period_ms.round(0)
    r["blocked_pct"] = 100 * r.replan_failures * r.replan_period_ms / (r.simulated_duration_ms * r.robot_count)
    return r


def valid_sets(r):
    once = r[r.replan_policy == "once"]
    bad = once[once.failed_plans > 0].groupby("planner").scenario.unique()
    allsc = set(r.scenario.unique())
    v = {p: allsc - set(bad.get(p, [])) for p in P}
    fails = {p: len(allsc) - len(v[p]) for p in P}
    common = set.intersection(*v.values())
    return v, common, fails, len(allsc)


def mcnemar(a, b):
    """a,b boolean arrays paired. returns diff (a-b) pct, 95% CI, exact p, discordant counts."""
    a, b = np.asarray(a), np.asarray(b)
    n = len(a); n10 = int((a & ~b).sum()); n01 = int((~a & b).sum())
    d = (n10 - n01) / n
    # Wald CI for paired proportion difference with continuity-free variance
    var = ((n10 + n01) - (n10 - n01) ** 2 / n) / n ** 2
    se = math.sqrt(max(var, 0))
    p = binomtest(n10, n10 + n01).pvalue if n10 + n01 else 1.0
    return dict(n=n, diff=100 * d, lo=100 * (d - 1.96 * se), hi=100 * (d + 1.96 * se), p=p, a_only=n10, b_only=n01)


def main(folder):
    r = load(folder)
    v, common, fails, nsc = valid_sets(r)
    out = {"batch": folder.name, "runs": len(r), "scenarios": nsc,
           "initial_failures": fails, "valid_n": {p: len(v[p]) for p in P}, "common_n": len(common)}
    # overlap of failure sets
    bad = {p: set(r.scenario.unique()) - v[p] for p in P}
    out["fail_overlap"] = {"prm&vis": len(bad["prm"] & bad["visibility"]), "vor&prm": len(bad["voronoi"] & bad["prm"]),
                           "vor_only": len(bad["voronoi"] - bad["prm"] - bad["visibility"])}
    rv = pd.concat([r[(r.planner == p) & r.scenario.isin(v[p])] for p in P])
    rc = r[r.scenario.isin(common)]

    # ---- 4.3 per-cell paired event vs cycle (within-planner valid set)
    cells = []
    for p in P:
        for h in sorted(r.prediction_horizon_ms.unique()):
            for per in sorted(r.replan_period_ms.unique()):
                s = rv[(rv.planner == p) & (rv.prediction_horizon_ms == h) & (rv.replan_period_ms == per)]
                w = s.pivot_table(index="scenario", columns="replan_policy", values="safe", aggfunc="first").dropna()
                m = mcnemar(w["cycle"].astype(bool), w["event"].astype(bool))
                g = lambda pol, col, f="median": getattr(s[s.replan_policy == pol][col], f)()
                cells.append(dict(planner=p, h=h, period=per, **{f"mc_{k}": x for k, x in m.items()},
                                  safe_cycle=100 * g("cycle", "safe", "mean"), safe_event=100 * g("event", "safe", "mean"),
                                  safe_once=100 * g("once", "safe", "mean"),
                                  replans_cycle=g("cycle", "replan_count"), replans_event=g("event", "replan_count"),
                                  ms_cycle=g("cycle", "planning_time_ms_total"), ms_event=g("event", "planning_time_ms_total"),
                                  over_cycle=100 * g("cycle", "over", "mean"), over_event=100 * g("event", "over", "mean"),
                                  blocked_cycle=g("cycle", "blocked_pct"), blocked_event=g("event", "blocked_pct"),
                                  heading_cycle=g("cycle", "heading_change_rad_per_m"), heading_event=g("event", "heading_change_rad_per_m")))
    cells = pd.DataFrame(cells)
    out["cells"] = cells.round(3).to_dict(orient="records")

    # ---- pooled per planner (over all horizons x periods), paired on scenario x setting
    pooled = {}
    for p in P:
        s = rv[rv.planner == p]
        key = ["scenario", "prediction_horizon_ms", "replan_period_ms"]
        w = s.pivot_table(index=key, columns="replan_policy", values="safe", aggfunc="first")
        wm = w[["cycle", "event"]].dropna()
        m = mcnemar(wm["cycle"].astype(bool), wm["event"].astype(bool))
        # scenario-cluster bootstrap CI (settings within a scenario are not independent)
        dsc = (wm["cycle"].astype(int) - wm["event"].astype(int)).groupby(level=0).agg(["sum", "count"])
        rng = np.random.default_rng(0); idx = np.arange(len(dsc)); bs = []
        for _ in range(4000):
            k = rng.choice(idx, len(idx)); bs.append(dsc["sum"].values[k].sum() / dsc["count"].values[k].sum())
        m["boot_lo"], m["boot_hi"] = 100 * np.percentile(bs, 2.5), 100 * np.percentile(bs, 97.5)
        tt = s[s.replan_policy.isin(["cycle", "event"])].pivot_table(index=key, columns="replan_policy", values="time_to_goal_ms", aggfunc="first").dropna()
        dt = (tt["event"] - tt["cycle"]) / 1000
        med = lambda pol, col: float(s[s.replan_policy == pol][col].median())
        mean = lambda pol, col: float(100 * s[s.replan_policy == pol][col].mean())
        pooled[p] = dict(n_scen=len(v[p]), safe_once=mean("once", "safe"), safe_cycle=mean("cycle", "safe"), safe_event=mean("event", "safe"),
                         mc=m, replans_cycle=med("cycle", "replan_count"), replans_event=med("event", "replan_count"),
                         ms_cycle=med("cycle", "planning_time_ms_total"), ms_event=med("event", "planning_time_ms_total"),
                         over_cycle=mean("cycle", "over"), over_event=mean("event", "over"),
                         maxcall_cycle=med("cycle", "planning_time_ms_max_call"), maxcall_event=med("event", "planning_time_ms_max_call"),
                         rebuild_ms_event=med("event", "planning_time_ms_rebuild"), check_ms_event=med("event", "planning_time_ms_check"),
                         ttg_cycle=med("cycle", "time_to_goal_ms") / 1000, ttg_event=med("event", "time_to_goal_ms") / 1000,
                         ttg_delta_median=float(dt.median()), ttg_delta_mean=float(dt.mean()), ttg_n=int(len(dt)),
                         ttg_p=float(wilcoxon(tt["event"], tt["cycle"]).pvalue),
                         blocked_cycle=med("cycle", "blocked_pct"), blocked_event=med("event", "blocked_pct"),
                         heading_cycle=med("cycle", "heading_change_rad_per_m"), heading_event=med("event", "heading_change_rad_per_m"))
        # scaling slope replans vs period
        sl = {}
        for pol in ("cycle", "event"):
            a = s[(s.replan_policy == pol) & (s.replan_period_ms == 20)].replan_count.median()
            b = s[(s.replan_policy == pol) & (s.replan_period_ms == 100)].replan_count.median()
            sl[pol] = math.log(b / a) / math.log(5) if a and b else None
        pooled[p]["slope"] = sl
    out["pooled"] = pooled

    # ---- 4.4 horizon, common intersection, event and cycle, per period
    hz = {}
    for pol in ("event", "cycle"):
        for per in sorted(r.replan_period_ms.unique()):
            for p in P:
                for h in sorted(r.prediction_horizon_ms.unique()):
                    s = rc[(rc.planner == p) & (rc.replan_policy == pol) & (rc.replan_period_ms == per) & (rc.prediction_horizon_ms == h)]
                    hz[f"{pol}|{per:.0f}|{p}|{h:.0f}"] = dict(safe=100 * s.safe.mean(), replans=s.replan_count.median(),
                        ms=s.planning_time_ms_total.median(), blocked=s.blocked_pct.median(), failures=s.replan_failures.mean(),
                        clear=s.minimum_obstacle_clearance_mm.median(), ttg=s.time_to_goal_ms.median() / 1000, over=100 * s.over.mean())
    out["horizon_common"] = hz
    # paired h0 vs h50 / h150 within planner, event p20, common set
    hp = {}
    for p in P:
        for per in sorted(r.replan_period_ms.unique()):
            s = rc[(rc.planner == p) & (rc.replan_policy == "event") & (rc.replan_period_ms == per)]
            w = s.pivot_table(index="scenario", columns="prediction_horizon_ms", values="safe", aggfunc="first").dropna()
            for h in (50.0, 150.0):
                hp[f"{p}|{per:.0f}|{h:.0f}"] = mcnemar(w[h].astype(bool), w[0.0].astype(bool))
    out["horizon_paired"] = hp
    # once arm
    out["once_common"] = {p: 100 * rc[(rc.planner == p) & (rc.replan_policy == "once")].safe.mean() for p in P}
    (folder / "analysis").mkdir(exist_ok=True)
    (folder / "analysis" / "paper_stats.json").write_text(json.dumps(out, indent=1, default=float))
    cells.round(2).to_csv(folder / "analysis" / "paper_stats_cells.csv", index=False)
    return out


if __name__ == "__main__":
    o = main(Path(sys.argv[1]))
    print(json.dumps({k: o[k] for k in ("runs", "scenarios", "initial_failures", "valid_n", "common_n", "fail_overlap")}, indent=1))
