"""Tick/cross verification of results/acra2026-final against its raw data.

Recomputes every summary column the tables and figures use from runs.csv and
rebuild_calls.csv, checks manifests (same bank, same clean code revision,
single worker, failure rules) and checks that every expected figure exists.

Usage:  python scripts/verify_results.py [results/acra2026-final]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pandas as pd

ROOT = Path(sys.argv[1] if len(sys.argv) > 1 else "results/acra2026-final")
SETS = {"policy-comparison": 2400, "horizon-sweep": 1800, "supporting/event-route-ablation": 1200}
KEYS = ["planner", "replan_policy", "replan_period_ms", "prediction_horizon_ms"]
FIGS = ["oneshot_planning_time_and_route_length", "policy_outcomes", "policy_collision_free",
        "policy_compute_vs_safety", "policy_replan_latency_20ms", "policy_replan_latency_100ms",
        "policy_route_lifetime", "horizon_outcomes", "horizon_trends", "horizon_route_lifetime",
        "ablation_event_route",
        "table_event_route", "table_rebuild_latency", "table_route_quality", "table_reliability",
        "planner_difficulty"]
results: list[tuple[bool, str]] = []


def check(ok: bool, label: str) -> None:
    results.append((bool(ok), label))


def close(a, b, tol=1e-6) -> bool:
    if pd.isna(a) and pd.isna(b):
        return True
    return abs(float(a) - float(b)) <= tol * max(1.0, abs(float(b)))


manifests = {}
for name in ["one-shot-validation", *SETS]:
    path = ROOT / name / "manifest.json"
    check(path.exists(), f"{name}: manifest present")
    if path.exists():
        manifests[name] = json.loads(path.read_text(encoding="utf-8"))

if manifests:
    shas = {m.get("scenario_bank_sha256") for m in manifests.values()}
    revs = {m.get("code_revision") for m in manifests.values()}
    check(len(shas) == 1 and None not in shas, "all sets use the same scenario bank (SHA-256)")
    check(len(revs) == 1, f"all sets use the same code revision ({', '.join(r[:8] for r in revs if r)})")
    check(all(not m.get("code_dirty") for m in manifests.values()), "code tree clean for every set")
    check(all("machine" in m for m in manifests.values()), "machine recorded in every manifest")
for name in SETS:
    m = manifests.get(name)
    if not m:
        continue
    check(m.get("workers") == 1, f"{name}: single worker")
    rules = m.get("failure_rules", {})
    check(rules.get("replan_time_limit_ms") == 100.0 and rules.get("no_route_limit_ms") == 1000.0,
          f"{name}: failure rules 100 ms / 1000 ms")

for name, expected in SETS.items():
    folder = ROOT / name
    if not (folder / "runs.csv").exists():
        check(False, f"{name}: runs.csv present")
        continue
    runs = pd.read_csv(folder / "runs.csv")
    summ = pd.read_csv(folder / "summary.csv")
    calls = pd.read_csv(folder / "rebuild_calls.csv")
    check(len(runs) == expected, f"{name}: {len(runs)}/{expected} runs")
    per_arm = runs.groupby(KEYS).scenario.agg(["count", "nunique"])
    check((per_arm["count"] == 200).all() and (per_arm["nunique"] == 200).all(),
          f"{name}: every arm has 200 distinct scenarios")
    check((runs.outcome == "X").sum() == 0, f"{name}: no invalid (X) runs")
    contacts = runs.rr_contact_episodes + runs.ro_contact_episodes
    buffers = runs.rr_buffer_episodes + runs.ro_buffer_episodes
    ok = (((runs.outcome == "S") <= ((contacts == 0) & (buffers == 0)))
          & ((runs.outcome == "B") <= ((contacts == 0) & (buffers > 0)))
          & ((runs.outcome == "P") <= (contacts > 0)))
    check(ok.all(), f"{name}: outcome class matches contact/buffer episodes for every run")
    check((runs.replan_count == runs.successful_rebuilds).all(),
          f"{name}: replans = successful rebuilds (escapes not counted)")
    g = runs.groupby(KEYS)
    recomputed = pd.DataFrame({
        "completed": g.outcome.apply(lambda o: (o != "I").sum()),
        "strict": g.outcome.apply(lambda o: (o == "S").sum()),
        "buffer_only": g.outcome.apply(lambda o: (o == "B").sum()),
        "physical": g.outcome.apply(lambda o: (o == "P").sum()),
        "replans_per_run": g.replan_count.mean(),
        "failed_replans_per_run": g.replan_failures.mean(),
        "escapes_per_run": g.escape_moves.mean(),
        "robots_stopped_time_limit": g.robots_stopped_time_limit.sum(),
        "episodes_failed_no_route": g.episode_end_reason.apply(lambda e: (e == "no_valid_path").sum()),
    }).reset_index()
    merged = summ.merge(recomputed, on=KEYS, suffixes=("", "_re"))
    for col in recomputed.columns.difference(KEYS):
        check(all(close(a, b) for a, b in zip(merged[col], merged[col + "_re"])),
              f"{name}: summary {col} matches runs.csv")
    if "collision_free_completed" in summ:
        check((summ.collision_free_completed == summ.strict + summ.buffer_only).all(),
              f"{name}: collision-free = strict + buffer-only")
    c = calls.groupby(KEYS).rebuild_ms
    lat = pd.DataFrame({"n": c.size(), "mean": c.mean()}).reset_index()
    m2 = summ.merge(lat, on=KEYS, how="left")
    check(all(close(a, b) for a, b in zip(m2.successful_rebuilds, m2.n.fillna(0))),
          f"{name}: rebuild_calls.csv rows = successful rebuilds")
    check(all(close(a, b, 1e-4) for a, b in zip(m2.replan_ms_mean, m2["mean"])),
          f"{name}: mean rebuild time matches rebuild_calls.csv")

pol, hor = ROOT / "policy-comparison/summary.csv", ROOT / "horizon-sweep/summary.csv"
if pol.exists() and hor.exists():
    p, h = pd.read_csv(pol), pd.read_csv(hor)
    check(set(h.prediction_horizon_ms) == {20.0, 50.0, 100.0}
          and set(p.prediction_horizon_ms) == {0.0}, "horizon sweep 20/50/100 ms; 0 ms point from policy set")

one = ROOT / "one-shot-validation/summary.csv"
if one.exists():
    o = pd.read_csv(one)
    check(len(o) == 3 and (o.scenarios == 200).all() and (o.robot_calls == 1200).all(),
          "one-shot: 3 planners × 200 scenarios × 6 robots")

for sub in ("", "paper"):
    folder = ROOT / "figures" / sub
    missing = [f for f in FIGS for ext in ("pdf", "png") if not (folder / f"{f}.{ext}").exists()]
    check(not missing, f"figures{('/' + sub) if sub else ''}: all {len(FIGS)} PDF + PNG present"
          + (f" (missing {len(missing)})" if missing else ""))
    if not missing and (ROOT / "policy-comparison/summary.csv").exists():
        newest = max((ROOT / s / "summary.csv").stat().st_mtime for s in SETS if (ROOT / s / "summary.csv").exists())
        stale = [f for f in FIGS if (folder / f"{f}.png").stat().st_mtime < newest]
        check(not stale, f"figures{('/' + sub) if sub else ''}: newer than the results" + (f" (stale: {', '.join(stale)})" if stale else ""))
tables = ["one_shot_validation", "policy_comparison", "horizon_sweep", "event_route_ablation"]
tmiss = [t for t in tables if not (ROOT / "tables" / f"{t}.md").exists()]
check(not tmiss, "tables: all four present" + (f" (missing {', '.join(tmiss)})" if tmiss else ""))

for ok, label in results:
    print(("✓ " if ok else "✗ ") + label)
print(f"\n{sum(ok for ok, _ in results)}/{len(results)} checks passed")
