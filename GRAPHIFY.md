# research-sdk graphify

This is a visual map of the implementation on `main`. It is based on the
runtime entry points and internal imports, rather than the target architecture
described in [`docs/architecture.md`](docs/architecture.md).

## System flow

```mermaid
flowchart LR
    operator[Operator] --> qt[Qt ResearchConsole]
    scenarios[Scenario JSON] --> scenario[Scenario model]
    scenario --> qt
    scenario --> headless[Headless virtual-time runner]
    scenario --> physics[Native physics runner]
    physics --> grsim_native[grSim headless / ODE]
    grsim_native --> physics_frames[Four-camera SSL-Vision frames]
    physics_frames --> physics
    physics --> evidence[Trajectories / settings / validation results]

    vision[SSL-Vision multicast] --> sockets[Vision / grSimVision]
    sockets --> qt
    qt --> runtime[ResearchRuntime]
    qt --> execution[ExecutionController]
    execution --> runtime

    runtime --> pipeline[VisionWorldPipeline]
    pipeline --> assembler[VisionFrameAssembler]
    assembler --> snapshot[WorldSnapshotStore]
    snapshot --> worldmap[WorldMap]
    worldmap --> scene[PlanningScene]
    scene -. dynamic obstacles .-> runtime

    runtime --> selected{Selected planner}
    headless --> selected
    physics --> selected
    selected --> visibility[VisibilityGraphPlanner]
    selected --> prm[PRMPlanner]
    selected --> planner_api[PlannerAPI]
    planner_api --> waypoint[VoronoiWaypointManager]
    waypoint --> voronoi[VoronoiDijkstraPlanner]

    visibility --> paths[PlannedRobotPath]
    prm --> paths
    voronoi --> paths
    paths --> tick[waypoint execution tick]
    tick --> dispatcher[RobotCommandDispatcher]
    dispatcher --> packet[grSim packet factory / sender]
    packet --> grsim[grSim simulator]

    runtime --> metrics[RunMetrics / ExperimentRecorder]
    metrics --> results[CSV / JSONL results]
    headless --> batch[Raw and summary CSV / JSON]
    physics --> batch
```

Solid arrows show the primary control or data path. The dotted edge is used
only when motion prediction is enabled and a live scene is available; otherwise
planning uses obstacles from the selected scenario.

## Package dependencies

```mermaid
flowchart TD
    entry[main.py / research-sdk-ui] --> ui[ui]

    ui --> planners[planners]
    ui --> world[world]
    ui --> network[network]
    ui --> config[config]

    planners --> world
    planners --> config
    planners --> nx[networkx]

    world --> vision[vision]
    world --> config
    world -. render DTOs .-> ui

    workers[process_workers] --> world
    workers --> vision
    workers --> network
    workers --> ui
    workers --> config

    network --> proto[network/proto2]
    network --> config

    recorder[recorder] --> world
    exporters[exporters] --> files[CSV / JSON / Parquet]

    ui --> qtdep[PySide6]
    world --> np[numpy]
    config --> yaml[PyYAML]
    proto --> protobuf[protobuf]
```

The dotted `world -> ui` edge is a real implementation dependency:
`world/map/voronoi/voronoi_generator.py` imports render data classes from
`ui/renderer.py`. `process_workers/voronoi_map_runner.py` does the same. It is
worth keeping in mind if the renderer or the world map is extracted later.

Generated protobuf modules are treated as one node so they do not overwhelm
the graph.

## Planner family

```mermaid
flowchart TB
    input[PlannerInput / PlanningScene] --> gate{direct or cached path usable?}
    gate -->|yes| output[PlannerOutput / waypoints]
    gate -->|no| family{planner implementation}

    family --> vg[Visibility graph]
    vg --> inflate[Inflate circular obstacles]
    inflate --> visible[Connect visible vertices]

    family --> prm[Probabilistic roadmap]
    prm --> sample[Sample collision-free milestones]
    sample --> neighbours[Connect nearest neighbours]

    family --> vd[Bounded Voronoi]
    vd --> cells[Build clipped Voronoi cells]
    cells --> clearance[Keep clearance-safe edges]

    visible --> dijkstra[networkx.dijkstra_path]
    neighbours --> dijkstra
    clearance --> dijkstra
    dijkstra --> output
    output --> recorder[StepRecorder debug geometry and timing]
```

All three planner families share NetworkX Dijkstra search. Their meaningful
difference is how they construct the graph searched by Dijkstra.

## Execution lifecycle

```mermaid
stateDiagram-v2
    [*] --> NoScenario
    NoScenario --> ScenarioLoaded: load scenario
    ScenarioLoaded --> NoScenario: unload
    ScenarioLoaded --> Applying: apply to grSim
    Applying --> Ready: placement confirmed
    Applying --> Error: placement failed
    Ready --> Running: plan and run
    Running --> Paused: pause
    Paused --> Running: continue
    Paused --> Paused: single waypoint step
    Running --> Completed: all robots arrive
    Running --> Stopped: stop / emergency stop
    Paused --> Stopped: stop / emergency stop
    Completed --> Resetting: reset
    Stopped --> Resetting: reset
    Error --> Resetting: retry
    Resetting --> Ready: placement confirmed
    Resetting --> Error: placement failed
```

## Where to start reading

| Concern | Primary file |
|---|---|
| Application composition | [`src/research_sdk/ui/app.py`](src/research_sdk/ui/app.py) |
| Runtime planning and command loop | [`src/research_sdk/ui/runtime.py`](src/research_sdk/ui/runtime.py) |
| Scenario and planner session logic | [`src/research_sdk/ui/session.py`](src/research_sdk/ui/session.py) |
| Execution state machine | [`src/research_sdk/ui/execution/controller.py`](src/research_sdk/ui/execution/controller.py) |
| Vision-to-world pipeline | [`src/research_sdk/world/pipeline.py`](src/research_sdk/world/pipeline.py) |
| Dynamic world map | [`src/research_sdk/world/map/world_map.py`](src/research_sdk/world/map/world_map.py) |
| Planner facade | [`src/research_sdk/planners/api.py`](src/research_sdk/planners/api.py) |
| Planner implementations | [`src/research_sdk/planners/`](src/research_sdk/planners/) |
| Network boundaries | [`src/research_sdk/network/`](src/research_sdk/network/) |
| Current implementation notes | [`docs/current.md`](docs/current.md) |
