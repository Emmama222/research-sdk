"""Build the ACRA 2026 result tables from results/acra2026-final/.

Reads each result set's summary.csv (written by ``research_sdk.headless
--result-set ...``) and writes CSV and Markdown tables to ``<root>/tables/``:

* one_shot_validation  -- One-Shot Planning Validation, one row per planner
* policy_comparison    -- Dynamic Replanning Policy Comparison, planner x arm
* horizon_sweep        -- Prediction Horizon Sweep, planner x horizon; the 0 ms
                          row is the policy comparison's event @ 20 ms arm
* event_route_ablation -- Supporting: event vs full-path (event_route) at 20 and 100 ms
                          (written only when supporting/event-route-ablation
                          exists)

Every count keeps its denominator (e.g. ``174/200``); per-run values are means
over the valid runs of the 200-scenario bank.

Usage:
    python scripts/build_result_tables.py [results/acra2026-final]
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

PLANNER_ORDER = {"voronoi": 0, "prm": 1, "visibility": 2}
PLANNER_LABEL = {"voronoi": "Voronoi", "prm": "PRM", "visibility": "Visibility Graph"}
# event_route is shown as "Full-path": recalculate when any part of the route is blocked
# (Costa and Tonidandel, RoboCup 2023, LNAI 14140, 2024).
ARM_LABEL = {"event": "Event (ours)", "event_route": "Full-path", "cycle": "Cycle"}


def _read(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _num(value: str | None, digits: int = 1) -> str:
    if value in (None, "", "None", "nan"):
        return "–"
    return f"{float(value):.{digits}f}"


def _pct(value: str | None) -> str:
    if value in (None, "", "None", "nan"):
        return "–"
    return f"{100.0 * float(value):.1f}%"


def _planner(row: dict[str, str]) -> str:
    return PLANNER_LABEL.get(row["planner"], row["planner"])


def one_shot_table(rows: list[dict[str, str]]) -> tuple[list[str], list[list[str]]]:
    header = [
        "Planner", "Routes (robots)", "Scenarios with all routes", "Map-requiring requests",
        "Median ms (map-requiring)", "P95 ms (map-requiring)", "Route excess over straight line (map-requiring, median)",
        "Map ms (mean, all)", "Dijkstra ms (mean, all)", "Total ms (max)",
        "Turning rad/m", "Turns >90° per route",
    ]
    body = [
        [
            _planner(r), r["robot_routes_label"], r["scenarios_all_routes_label"],
            r.get("map_requiring_calls", "–"), _num(r.get("map_requiring_ms_median"), 2),
            _num(r.get("map_requiring_ms_p95"), 2), _pct(r.get("map_requiring_path_excess_median")),
            _num(r["map_ms_mean"], 2), _num(r["search_ms_mean"], 3), _num(r["total_ms_max"], 2),
            _num(r["turning_rad_per_m_mean"], 2), _num(r["sharp_turns_per_route"], 2),
        ]
        for r in sorted(rows, key=lambda r: PLANNER_ORDER.get(r["planner"], 9))
    ]
    return header, body


DYNAMIC_HEADER = [
    "Contact-free completions /200",
    "Completed", "Strict /C", "Buffer-only /C", "Contact /C", "RR contacts /F",
    "RO contacts /F", "Contacts /F", "Incomplete: no route for 1 s",
    "Robots stopped (rebuild >100 ms)",
    "Initial retries /run", "Initial wait ms /run", "Rebuilds /run", "Escape moves /run",
    "Failed rebuild attempts /run",
    "Failed rebuild attempt share", "Time without a route %", "Route lifetime ms (mean)", "Route lifetime ms (max)",
    "Rebuild ms (min)", "Rebuild ms (mean)", "Rebuild ms (P95)",
    "Rebuild ms (max)",
]


def _collision_free(r: dict[str, str]) -> str:
    if r.get("collision_free_completed_label"):
        return r["collision_free_completed_label"]
    return f"{int(r['strict']) + int(r['buffer_only'])}/{r['valid_runs']}"


def _dynamic_cells(r: dict[str, str]) -> list[str]:
    return [
        _collision_free(r),
        r["completed_label"], r["strict_label"], r["buffer_only_label"], r["physical_label"],
        _num(r["rr_contacts_per_failed_run"], 2), _num(r["ro_contacts_per_failed_run"], 2),
        _num(r["contacts_per_failed_run"], 2),
        f"{r['episodes_failed_no_route']}/{r['valid_runs']}",
        r["robots_stopped_time_limit"],
        _num(r["initial_retries_per_run"], 2), _num(r["initial_wait_ms_per_run"], 0),
        _num(r["replans_per_run"], 1), _num(r.get("escapes_per_run"), 1),
        _num(r["failed_replans_per_run"], 1),
        _pct(r["failed_replan_share"]), _num(r["blocked_time_pct_mean"], 2), _num(r["route_lifetime_ms_mean"], 0),
        _num(r["route_lifetime_ms_max"], 0), _num(r["replan_ms_min"], 3),
        _num(r["replan_ms_mean"], 2), _num(r["replan_ms_p95"], 2), _num(r["replan_ms_max"], 2),
    ]


def policy_table(rows: list[dict[str, str]]) -> tuple[list[str], list[list[str]]]:
    def key(r):
        return (PLANNER_ORDER.get(r["planner"], 9), float(r["replan_period_ms"]), r["replan_policy"])

    body = [
        [_planner(r), f"{r['replan_policy'].capitalize()} @ {float(r['replan_period_ms']):g} ms",
         *_dynamic_cells(r)]
        for r in sorted(rows, key=key)
    ]
    return ["Planner", "Arm", *DYNAMIC_HEADER], body


def horizon_table(
    sweep: list[dict[str, str]], policy: list[dict[str, str]]
) -> tuple[list[str], list[list[str]]]:
    baseline = [
        r for r in policy
        if r["replan_policy"] == "event"
        and float(r["replan_period_ms"]) == 20.0
        and float(r["prediction_horizon_ms"]) == 0.0
    ]
    rows = baseline + sweep

    def key(r):
        return (PLANNER_ORDER.get(r["planner"], 9), float(r["prediction_horizon_ms"]))

    body = [
        [_planner(r), f"{float(r['prediction_horizon_ms']):g} ms", *_dynamic_cells(r)]
        for r in sorted(rows, key=key)
    ]
    return ["Planner", "Horizon (event @ 20 ms)", *DYNAMIC_HEADER], body


def ablation_table(
    ablation: list[dict[str, str]], policy: list[dict[str, str]]
) -> tuple[list[str], list[list[str]]]:
    rows = [r for r in policy if r["replan_policy"] == "event"] + ablation

    def key(r):
        return (PLANNER_ORDER.get(r["planner"], 9), float(r["replan_period_ms"]), r["replan_policy"])

    body = [
        [_planner(r), f"{ARM_LABEL.get(r['replan_policy'], r['replan_policy'])} @ {float(r['replan_period_ms']):g} ms",
         *_dynamic_cells(r)]
        for r in sorted(rows, key=key)
    ]
    return ["Planner", "Arm", *DYNAMIC_HEADER], body


def _write(folder: Path, name: str, header: list[str], body: list[list[str]]) -> None:
    folder.mkdir(parents=True, exist_ok=True)
    with (folder / f"{name}.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(body)
    lines = [
        "| " + " | ".join(header) + " |",
        "|" + "|".join("---" if i < 2 else "---:" for i in range(len(header))) + "|",
        *("| " + " | ".join(row) + " |" for row in body),
    ]
    (folder / f"{name}.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Build the ACRA 2026 result tables.")
    parser.add_argument("root", type=Path, nargs="?", default=Path("results/acra2026-final"))
    args = parser.parse_args()
    one_shot = _read(args.root / "one-shot-validation" / "summary.csv")
    policy = _read(args.root / "policy-comparison" / "summary.csv")
    sweep = _read(args.root / "horizon-sweep" / "summary.csv")
    ablation = _read(args.root / "supporting" / "event-route-ablation" / "summary.csv")
    out = args.root / "tables"
    written = []
    if one_shot:
        _write(out, "one_shot_validation", *one_shot_table(one_shot))
        written.append("one_shot_validation")
    if policy:
        _write(out, "policy_comparison", *policy_table(policy))
        written.append("policy_comparison")
    if sweep or policy:
        _write(out, "horizon_sweep", *horizon_table(sweep, policy))
        written.append("horizon_sweep")
    if ablation and policy:
        _write(out, "event_route_ablation", *ablation_table(ablation, policy))
        written.append("event_route_ablation")
    print(f"Wrote {', '.join(written) or 'nothing (no summaries found)'} to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
