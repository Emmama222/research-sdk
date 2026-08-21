# 0003: Phoenix-Server link and vendored TeamControl planner

Date: 2026-08-02

## Context

Two sibling projects exist outside this repo: **Phoenix-Server** (`../team stuff separately/Phoenix-Server`), Team TurtleRabbit's real match server for physical robots, and **2026-TeamControl** (`../team stuff separately/2026-TeamControl`), which is officially out of commission but has a working voronoi/Dijkstra path planner under `src/TeamControl/{planner,world}/`. The ask: keep research-sdk's view of Phoenix's input/output in sync (for now, in simulation, with real-world use planned later), and get TeamControl's planner testable from here.

## Decisions

1. **Vendor the planner, don't depend on TeamControl live.** TeamControl is out of commission, so an editable install would depend on a repo nobody is maintaining. `planner/api.py`, `planner/voronoi_dijkstra.py`, `planner/waypoint_manager.py`, and the `world/{field_config.py,map_graph.py,map/*.py}` files it needs are copied into `src/research_sdk/vendored/team_control_planner/`, with only `TeamControl.*` import paths rewritten -- no other changes, so it stays a recognizable copy. Explicitly left out: `world/map/observer.py` (live vision tracking, not needed for a stateless planner call) and anything under `voronoi_planner/` (an older/alternate implementation superseded by `planner/`, per that module's own `planner_new.py` naming). `TeamControl`'s own `tests/world/test_planner_api.py` was ported verbatim (`tests/test_vendored_planner.py`) as a correctness check that the vendored copy still behaves identically -- it does.

2. **Patched one bug found while wiring real obstacles through:** `map/obstacles.py`'s `Obstacle.clearance_to_path_mm` called `self.dynamic_radius(horizon_ms)`, which doesn't exist -- only `dynamic_radius_0` does, and its docstring/body is exactly what the call site needs. This is a pre-existing bug in the original TeamControl source (confirmed against it directly), not something introduced by vendoring; it just meant no test in either repo exercised obstacle-avoidance through a real `Obstacle` object with the full Dijkstra route. Fixed with a comment marking it as a deliberate patch, per this vendored package's own rule (see its `__init__.py`): change it deliberately and say so, don't silently fork.

3. **`PhoenixAdaptor` speaks Phoenix's own documented contract exactly**, copied from `phoenix/api.py`'s docstring and `_publish_now`/`_skill_from` implementations, not reconstructed from memory: a 20 Hz world-state JSON stream on `world_port` (default 47001, loopback), and skill commands (`move_to`/`get_ball`/`kick_at`/`stop`, each carrying a TTL) to `command_port` (default 47002). Units are meters (confirmed via Phoenix's `configs/limits.toml`, "SI units", and its own tests asserting `field_length == 9.0`).

4. **`PhoenixAdaptor.fetch()` is read-only; sending a command is a separate, explicit `send_skill()` call.** This is a deliberate difference from `GrSimAdaptor`, which runs its own closed-loop controller inside `fetch()`. Phoenix already owns trajectory generation and the safety governor onboard -- research-sdk's job here is to observe and occasionally assign a skill, not implement a second control loop that would fight with Phoenix's. Every command Phoenix accepts carries a TTL and expires (robot brakes) if not renewed, so a crashed or stalled client can't leave a stale command running -- this was a factor in deciding it's safe to build against now rather than waiting.

5. **Real robots are explicitly out of scope for this integration.** The ask was to stay on the simulation side while planning for real-world use later. `PhoenixAdaptor` defaults to loopback only; nothing here assumes `configs/robots.toml` has real entries. `PhoenixAdaptor` is not wired into the MCP server (unlike `GrSimAdaptor`'s `move_robot_to`) -- letting an agent assign skills to a real match server is a materially different risk than a simulator, and that's a decision for when real-world use is actually in scope, not a default to fall into now.

6. **Millimeters-vs-meters conversion happens in exactly one place.** `phoenix_world_to_planner_input()` (`adaptors/planner.py`) is the sole point where Phoenix's meter-based world state becomes the vendored planner's millimeter-based `PlannerInput`. `target_pose` is supplied by the caller, not derived from Phoenix's world state -- Phoenix reports where things *are*, not where the strategy wants a robot to *go*.

7. **`PlannerAdaptor` is not registered in `AdaptorRegistry`.** Like `LegacyFunctionAdaptor`, `FileBridgeAdaptor`, and `SimulationAdaptor`, it's stateful (the underlying `VoronoiWaypointManager` tracks waypoint progress per robot across calls) and not safely constructible generically -- same rule as always, see `docs/adaptors.md`.

8. **Fixed a latent bug in `FileBridgeAdaptor` while testing this end to end**, unrelated to Phoenix/planner directly: `fetch()` treated "response file exists" as "response file is ready," but a non-atomic external writer (MATLAB's `fopen`/`fprintf`/`fclose`, or even plain `Path.write_text`) can leave the file briefly empty or truncated. `fetch()` now tolerates a transient `JSONDecodeError` and keeps polling instead of crashing; `examples/matlab_bridge_listener.m` was also updated to write via a temp file + `movefile` (atomic rename) as the correct pattern, rather than relying solely on the receiving side's tolerance.

9. **The offline 2D planner view (Qt UI) is intentionally disconnected from both Phoenix and grSim.** `PlannerViewTab` steps `PlannerAdaptor` locally against a simple point-mass integrator purely to visualize routing/reroute behavior -- no network, no external process. It's the fastest way to see the vendored planner actually avoid an obstacle without standing up a simulator or match server first.

## Status

Accepted. Revisit when real-world use starts: `PhoenixAdaptor` will need real `robots.toml` awareness and a decision about whether/how to expose `send_skill` over MCP, which is a materially bigger safety conversation than anything decided here.
