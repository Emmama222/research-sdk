# Architecture

## Layers

> **Target architecture.** This describes the layering agreed 2026-08-20, not
> everything the code does yet — most visibly, `GrSimAdaptor` still computes
> control *and* sends over the network in one `fetch()` call, where the model
> below treats those as two separate layers. See
> [decisions/0004-layered-pipeline-and-run-recording.md](decisions/0004-layered-pipeline-and-run-recording.md)
> for what's planned vs. what's landed.

Data flows through three layers, back-end to back-end:

### 1. Input — world model

Owns every inbound thing into the software: vision (SSL vision detection),
game controller input, and robot feedback. Its job is to populate the world
model (`core/world.py`'s `Frame`/`Robot`/`ball`) and nothing else — no
decisions get made here.

- Network receive side of the adaptors: vision multicast (`GrSimAdaptor`),
  Phoenix's world-state stream (`PhoenixAdaptor`), grSim/SSL robot feedback.
- Output: a `Frame` (or equivalent snapshot) describing where everything is,
  right now.

### 2. Middle ground — map, processing, UI

Everything that turns "where things are" into "what a robot should do next."

- Builds the map/field representation on top of the world model
  (`core/ssl_field_geometry.py`, the vendored planner's
  `world/map_graph.py`).
- Runs the planning/decision step (`PlannerAdaptor`,
  `FastPathPlannerAdaptor`) that turns world state + a target into a
  per-robot command: **robot × team × target × maximum velocity**.
- Owns the UI (`ui/app.py`). The UI lives here, not as a separate front-end
  tier — it's both an operator console over this processing and, at user
  request, the trigger for run recording (see [onboarding.md](onboarding.md)
  and [exporting.md](exporting.md)).

### 3. Output — format and send

Takes the middle layer's command and gets it to wherever it needs to go.

- Converts the command into the destination's wire format: `grSim_Packet`
  for grSim, a skill-command JSON for Phoenix, etc.
- Owns the actual network send back out — to grSim, other simulation
  backends, or (later) real robots.
- **Not yet a separate step in code.** Today this is folded into each
  adaptor's `fetch()`/`send_skill()` rather than split out on its own — see
  ADR 0004.

## Where this maps today

| Layer | Real modules |
|---|---|
| Input / world model | `core/world.py`, `adaptors/grsim.py` (vision read side), `adaptors/phoenix.py` (world-state read side) |
| Middle ground | `core/ssl_field_geometry.py`, `adaptors/planner.py`, `adaptors/fast_path_planner.py`, `vendored/team_control_planner/`, `ui/app.py` |
| Output | `adaptors/grsim.py` (command send side), `adaptors/phoenix.py`'s `send_skill()`, `proto/generated/` |
| Cross-cutting | `adaptors/` (contract: see [adaptors.md](adaptors.md)), `exporters/` (contract: see [exporting.md](exporting.md)) |

Adaptors, exporters, and MCP tools are documented separately since their
contracts don't change with this layering — see [adaptors.md](adaptors.md),
[exporting.md](exporting.md), and [mcp.md](mcp.md).
