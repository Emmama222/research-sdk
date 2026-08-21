# Adaptors

An adaptor wraps one data source — legacy code or an external API — behind a single interface so the rest of the SDK (pipelines, exporters, MCP tools) never needs source-specific logic.

## The contract

```python
class Adaptor(ABC):
    name: ClassVar[str]

    @abstractmethod
    def fetch(self, **kwargs: Any) -> Any: ...
```

That's it. `fetch` takes whatever keyword arguments the underlying source needs and returns either a single record (`dict`) or a list of them — `Pipeline` normalizes either into `list[dict]` before handing off to an exporter.

## Adding a new adaptor

1. Subclass `Adaptor` in a new file under `adaptors/`.
2. Set `name` to a short, stable identifier — it's what shows up in docs, logs, and (if registered) MCP tool listings.
3. Implement `fetch`.
4. Decide whether to register it (see below).

## Two existing adaptors, two different patterns

- **`LegacyFunctionAdaptor`** wraps an arbitrary Python callable. It needs an actual function reference at construction time, so it's *not* registered in `AdaptorRegistry` — you wire it up directly in code (see `core/pipeline.py` usage in the quickstart). There's no sane way to look up "which function" by a string name from a generic registry.
- **`APIAdaptor`** wraps a JSON HTTP API via `clients.RetryingClient`. It's registrable in principle, but only register a *pinned* instance/subclass (base URL and auth already baked in) — never one where the caller supplies the base URL, since that's effectively an open proxy. See [mcp.md](mcp.md).

## Simulation adaptors

`SimulationAdaptor` wraps a stochastic or simulation callable and runs it for a fixed number of trials, returning one record per trial:

```python
def coin_flip_bias(trial: int, p_heads: float) -> dict:
    import random
    return {"heads": random.random() < p_heads}

adaptor = SimulationAdaptor(coin_flip_bias, trials=1000)
records = adaptor.fetch(p_heads=0.5)   # [{"trial": 0, "heads": True}, {"trial": 1, "heads": False}, ...]
```

The wrapped callable receives `trial` (the trial index) plus whatever kwargs `fetch()` was given, and returns a dict (or a bare value, auto-wrapped as `{"result": value}`). Because `fetch()` already returns a list of per-trial records, it feeds directly into `Pipeline` and any `Exporter` with no extra glue — export 1000 trials to CSV and analyze the distribution downstream.

Same rule as `LegacyFunctionAdaptor`: it needs a live callable at construction time, so it's not registered in `AdaptorRegistry` — wire it up in code, not via MCP.

## External process bridge (MATLAB, etc.)

`FileBridgeAdaptor` talks to an external process that doesn't speak Python or HTTP -- MATLAB is the motivating case -- via a request/response file handoff:

```python
adaptor = FileBridgeAdaptor(
    request_path=Path("bridge_request.json"),
    response_path=Path("bridge_response.json"),
    timeout=30.0,
)
result = adaptor.fetch(target_x=1.5, target_y=0.0)
```

`fetch()` writes its kwargs as JSON to `request_path`, polls for `response_path` to appear, reads it back as JSON, and deletes both files. The other side (MATLAB, or anything else) is expected to watch `request_path` and write `response_path` when it's done -- see `examples/matlab_bridge_listener.m` for a working counterpart script.

This is deliberately the same pattern the paper backing the grSim work uses for its Matlab<->central-software link (write a controller file to shared storage, poll for a result file) -- proven to work for exactly this kind of "long-running MATLAB process, occasional Python-triggered computation" setup, and it needs nothing MATLAB-version-specific on the Python side (unlike the MATLAB Engine API, whose `matlabengine` package must match your exact MATLAB release).

Same construction-time-argument rule applies: it's not registered in `AdaptorRegistry`.

## grSim adaptor

`GrSimAdaptor` drives one robot to a target position/orientation in the grSim RoboCup simulator, closed-loop:

```python
adaptor = GrSimAdaptor()  # local defaults: command 127.0.0.1:20011, vision multicast 224.5.23.2:10020
result = adaptor.fetch(target_x=1.0, target_y=2.0, target_theta=0.0, robot_id=0, is_yellow=True)
# {"reached": True, "steps": 42, "final_x": 1.0, "final_y": 2.0, "final_theta": 0.0}
```

It sends `grSim_Packet` velocity commands over UDP and reads robot position back from `SSL_WrapperPacket` vision detection frames, using a proportional controller on position/heading error (rotated into the robot's local frame before scaling). This is the paper's baseline "PD-controller" starting point, not its learned REPS/Gaussian-Process policy -- see [decisions/0002-grsim-baseline-controller.md](decisions/0002-grsim-baseline-controller.md) for why, and for the vision/command port defaults you may need to override to match your own grSim config.

**Tuning:** `linear_gain`, `angular_gain`, `max_linear_speed`, `max_angular_speed` are starting points, not tuned values -- the paper's own finding is that hand-tuned PD gains "are not optimal and result in overshooting and overcontrol." If you see oscillation, lower the gains before assuming something else is wrong.

**Protobuf bindings:** compiled from the real grSim/SSL `.proto` files in `src/research_sdk/proto/` via `python scripts/generate_protos.py` -- rerun that after any protocol change, don't hand-edit `proto/generated/`.

Registered in `AdaptorRegistry` (every argument has a local default) and wired into the MCP server as `move_robot_to` -- but constructed lazily there, since unlike other adaptors its constructor has a real side effect (binds a socket, joins a multicast group). See [mcp.md](mcp.md).

## Phoenix-Server adaptor

`PhoenixAdaptor` reads Phoenix-Server's world-state stream and can send it skill commands, matching `phoenix.api.JsonBridge`'s documented contract exactly (world state in on port 47001, commands out on port 47002, both loopback by default):

```python
adaptor = PhoenixAdaptor()
world = adaptor.fetch()  # latest world-state snapshot, or None if nothing arrived in time
adaptor.send_skill(robot_id=0, skill="move_to", x=1.0, y=0.5, theta=0.0)
```

Unlike `GrSimAdaptor`, `fetch()` is **read-only** -- it does not run a closed-loop controller, because Phoenix already owns trajectory generation and the safety governor onboard. Issuing a command is always the separate, explicit `send_skill()` call, so calling `fetch()` in a loop (e.g. to watch state) can never accidentally drive a robot. Every command carries a TTL and expires on its own if not renewed. See [decisions/0003-phoenix-and-planner-integration.md](decisions/0003-phoenix-and-planner-integration.md) for the full reasoning, including why this stays off the MCP server for now (real robots are out of scope until that's a deliberate decision, not a default).

Registered in `AdaptorRegistry` (loopback-only defaults, no caller-supplied network target to abuse).

## Vendored planner + PlannerAdaptor

`src/research_sdk/vendored/team_control_planner/` is a vendored copy of 2026-TeamControl's voronoi/Dijkstra path planner (that project is officially out of commission, so this is a copy, not a live dependency -- see the ADR for exactly what was and wasn't brought over, and one bug patched in the process). `PlannerAdaptor` wraps its `PlannerAPI`:

```python
adaptor = PlannerAdaptor()
result = adaptor.fetch(
    robot_id=0, is_yellow=True,
    current_pose=(0.0, 0.0, 0.0), target_pose=(1000.0, 0.0, 0.0),
    obstacles=(),
)
# {"waypoints": (...), "active_target_pose": (...), "is_path_free": True, ...}
```

It's stateful (the underlying `VoronoiWaypointManager` tracks per-robot waypoint progress across calls), so like `LegacyFunctionAdaptor` it isn't registered in `AdaptorRegistry` -- construct one instance and reuse it.

`phoenix_world_to_planner_input()` (same module) bridges a `PhoenixAdaptor.fetch()` snapshot into a `PlannerInput`: it's the one place Phoenix's meters get converted to the planner's millimeters, and it turns every other tracked robot into an `Obstacle`. `target_pose` isn't part of Phoenix's world state (Phoenix reports where things are, not where you want them to go) so the caller always supplies it.

For a fully offline way to see this work without grSim or Phoenix running at all, see the Qt UI's "Planner (offline 2D)" tab -- it steps `PlannerAdaptor` locally against a simple point-mass integrator and draws the routed path.

## Should you register it?

Register in `AdaptorRegistry` only if the adaptor can be constructed with **no runtime-only arguments** — i.e. everything it needs (config, credentials) comes from environment/settings, not from the caller. That's the same bar for whether it's safe to expose as an MCP tool. If it needs a live object handed in (a function, an open connection), keep it unregistered and wire it up explicitly in whatever pipeline uses it.
