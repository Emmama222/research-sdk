# Results before the shared escape step (archived 1 Oct 2026)

`acra2026-final/` here is the complete ACRA 2026 result set as it stood on
1 Oct 2026 (commit `1e683db`): one-shot validation, policy comparison, horizon
sweep, the full-path (`event_route`) supporting set, and all tables and figures.

It was superseded because only the Voronoi planner had an escape step: when a
robot was inside an obstacle zone, Voronoi stepped out while PRM and the
visibility graph refused to plan ("start or goal lies inside an inflated
obstacle"). That made part of the cross-planner difference an implementation
artefact. From DEC-022 the escape step lives in the shared layer
(`planners/reroute.py`, `escape_waypoint`) and all three planners use it; the
result sets were re-run into `results/acra2026-final/`.

Voronoi's behaviour is unchanged by the fix, so its numbers here remain a valid
reference; PRM and visibility-graph numbers are not comparable with the new set.
Do not use this folder for final claims.
