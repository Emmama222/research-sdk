# Final configuration with the 100 ms replan limit enforced (1 October 2026)

All four ACRA 2026 result sets at revision `4b04d42f` (committed in `50886d8`),
run with the DEC-019 failure rules: a post-initial rebuild taking over 100 ms
of wall time stops that robot, and a robot without a route for 1 s fails the
episode. All 72 checks in `scripts/verify_results.py` passed at the time.

Superseded as the main results by DEC-024 (1 October 2026), which keeps the
1 s no-route rule but no longer enforces the 100 ms limit, so that the main
results do not depend on the machine's timing. Simulated time is paused during
every planner call, so with the limit off the runs are deterministic.

Kept as supporting evidence for what the limit does on one laptop (Intel Core
Ultra 7 155H, Windows 11, Python 3.13.7):
- Every incomplete run at 20 ms checks contains a time-limit stop.
- Cycle @ 20 ms stops 4 / 25 / 55 robots (Voronoi / PRM / Visibility Graph),
  event @ 20 ms 3 / 6 / 22.
- On scenarios both triggers finished, contact-free counts do not differ
  significantly for any planner (paired exact McNemar test, p >= 0.30).
- A first run of the same configuration (`_archive-dec023-stalls/README.md`)
  reproduced every run that no stop touched (4,891 of 4,891).

`tables/` and `figures/` here were built from this set.
